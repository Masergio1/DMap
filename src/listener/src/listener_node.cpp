#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <cmath>

#include <rp_stuff/dmap.h>
#include <rp_stuff/draw_helpers.h>
#include <rp_stuff/dmap_localizer.h>

#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Geometry>

ros::Publisher odom_publisher;
tf2_ros::TransformBroadcaster* tf_sender = nullptr;

GridMapping map_handler;
DMapLocalizer robot_localizer;

std::vector<Vector2f> occupied_cells;

bool has_map = false;
bool has_start_pose = false;

float map_resolution = 0.0f;
float max_influence_distance = 10.0f;
int occupied_limit = 70;

float getYawFromTransform() {
    return std::atan2(
        robot_localizer.X.linear()(1, 0),
        robot_localizer.X.linear()(0, 0)
    );
}

tf2::Quaternion makeQuaternionFromYaw(float yaw) {
    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, yaw);
    return quat;
}

void sendOdometryMessage(const ros::Time& time_stamp) {
    nav_msgs::Odometry corrected_odom;

    corrected_odom.header.stamp = time_stamp;
    corrected_odom.header.frame_id = "map";
    corrected_odom.child_frame_id = "robot";

    corrected_odom.pose.pose.position.x = robot_localizer.X.translation().x();
    corrected_odom.pose.pose.position.y = robot_localizer.X.translation().y();
    corrected_odom.pose.pose.position.z = 0.0;

    float theta = getYawFromTransform();
    corrected_odom.pose.pose.orientation = tf2::toMsg(makeQuaternionFromYaw(theta));

    odom_publisher.publish(corrected_odom);
}

void sendRobotTransform(const ros::Time& time_stamp) {
    geometry_msgs::TransformStamped tf_msg;

    tf_msg.header.stamp = time_stamp;
    tf_msg.header.frame_id = "map";
    tf_msg.child_frame_id = "robot";

    tf_msg.transform.translation.x = robot_localizer.X.translation().x();
    tf_msg.transform.translation.y = robot_localizer.X.translation().y();
    tf_msg.transform.translation.z = 0.0;

    float theta = getYawFromTransform();
    tf_msg.transform.rotation = tf2::toMsg(makeQuaternionFromYaw(theta));

    tf_sender->sendTransform(tf_msg);
}

std::vector<Vector2f> extractObstaclesFromMap(
    const nav_msgs::OccupancyGrid::ConstPtr& grid_msg
) {
    std::vector<Vector2f> result;

    const int width = grid_msg->info.width;
    const int height = grid_msg->info.height;

    for (int row = 0; row < height; ++row) {
        for (int col = 0; col < width; ++col) {
            int cell_index = row * width + col;
            int8_t cell_value = grid_msg->data[cell_index];

            if (cell_value >= occupied_limit) {
                result.emplace_back(
                    static_cast<float>(col),
                    static_cast<float>(row)
                );
            }
        }
    }

    return result;
}

void onMapReceived(const nav_msgs::OccupancyGrid::ConstPtr& map_msg) {
    map_resolution = map_msg->info.resolution;

    Vector2f map_origin(
        map_msg->info.origin.position.x,
        map_msg->info.origin.position.y
    );

    occupied_cells = extractObstaclesFromMap(map_msg);

    robot_localizer.setMap(
        occupied_cells,
        map_resolution,
        max_influence_distance
    );

    map_handler.reset(map_origin, map_resolution);

    has_map = true;

    ROS_INFO(
        "Mappa caricata correttamente. Celle occupate trovate: %lu",
        occupied_cells.size()
    );
}

std::vector<Vector2f> convertLaserScanToPoints(
    const sensor_msgs::LaserScan::ConstPtr& scan_msg
) {
    std::vector<Vector2f> points;

    points.reserve(scan_msg->ranges.size());

    for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
        const float distance = scan_msg->ranges[i];

        if (distance < scan_msg->range_min || distance > scan_msg->range_max) {
            continue;
        }

        const float angle = scan_msg->angle_min +
                            static_cast<float>(i) * scan_msg->angle_increment;

        const float px = distance * std::cos(angle);
        const float py = distance * std::sin(angle);

        points.emplace_back(px, py);
    }

    return points;
}

void onLaserScanReceived(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
    if (!has_map || !has_start_pose) {
        ROS_WARN_THROTTLE(
            1.0,
            "Nodo in attesa della mappa e della posa iniziale..."
        );
        return;
    }

    std::vector<Vector2f> laser_points = convertLaserScanToPoints(scan_msg);

    const int iterations = 10;
    bool localized = robot_localizer.localize(laser_points, iterations);

    if (!localized) {
        ROS_WARN_THROTTLE(
            1.0,
            "Localizzazione non riuscita: numero di inlier insufficiente"
        );
        return;
    }

    sendOdometryMessage(scan_msg->header.stamp);
    sendRobotTransform(scan_msg->header.stamp);

    ROS_DEBUG_THROTTLE(
        1.0,
        "Posa stimata: x = %.2f, y = %.2f",
        robot_localizer.X.translation().x(),
        robot_localizer.X.translation().y()
    );
}

void onInitialPoseReceived(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg
) {
    robot_localizer.X.translation().x() = pose_msg->pose.pose.position.x;
    robot_localizer.X.translation().y() = pose_msg->pose.pose.position.y;

    tf2::Quaternion quaternion(
        pose_msg->pose.pose.orientation.x,
        pose_msg->pose.pose.orientation.y,
        pose_msg->pose.pose.orientation.z,
        pose_msg->pose.pose.orientation.w
    );

    tf2::Matrix3x3 rotation_matrix(quaternion);

    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;

    rotation_matrix.getRPY(roll, pitch, yaw);

    robot_localizer.X.linear() =
        Eigen::Rotation2Df(static_cast<float>(yaw)).toRotationMatrix();

    has_start_pose = true;

    ROS_INFO(
        "Posa iniziale impostata: x = %.2f, y = %.2f, theta = %.2f",
        pose_msg->pose.pose.position.x,
        pose_msg->pose.pose.position.y,
        yaw
    );
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "listener_node");

    ros::NodeHandle node;

    odom_publisher = node.advertise<nav_msgs::Odometry>(
        "/odom_corrected",
        10
    );

    tf_sender = new tf2_ros::TransformBroadcaster();

    ros::Subscriber map_subscriber = node.subscribe(
        "/map",
        1,
        onMapReceived
    );

    ros::Subscriber pose_subscriber = node.subscribe(
        "/initialpose",
        10,
        onInitialPoseReceived
    );

    ros::Subscriber laser_subscriber = node.subscribe(
        "/base_scan",
        10,
        onLaserScanReceived
    );

    ROS_INFO("Nodo di localizzazione avviato.");

    ros::spin();

    delete tf_sender;
    tf_sender = nullptr;

    return 0;
}