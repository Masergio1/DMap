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

#include <Eigen/Geometry>

ros::Publisher odom_publisher;

GridMapping map_handler;
DMapLocalizer robot_localizer;

std::vector<Vector2f> occupied_cells;

bool has_map = false;
bool has_start_pose = false;

float map_resolution = 0.0f;
float max_influence_distance = 10.0f;
int occupied_limit = 100;

float getYawFromTransform() {
    return std::atan2(
        robot_localizer.X.linear()(1, 0),
        robot_localizer.X.linear()(0, 0)
    );
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

    corrected_odom.pose.pose.orientation.z = std::sin(theta * 0.5f);
    corrected_odom.pose.pose.orientation.w = std::cos(theta * 0.5f);

    odom_publisher.publish(corrected_odom);

    ROS_INFO(
        "Odometry pubblicata: x = %.2f, y = %.2f, theta = %.2f",
        corrected_odom.pose.pose.position.x,
        corrected_odom.pose.pose.position.y,
        theta
    );
}

std::vector<Vector2f> extractObstaclesFromMap(
    const nav_msgs::OccupancyGrid::ConstPtr& grid_msg
) {
    std::vector<Vector2f> result;

    const int width = grid_msg->info.width;
    const int height = grid_msg->info.height;

    for (int row = 0; row < height; ++row) {
        for (int col = 0; col < width; ++col) {
            const int cell_index = row * width + col;
            const int8_t cell_value = grid_msg->data[cell_index];

            if (cell_value == occupied_limit) {
                Vector2f world_position = map_handler.grid2world(
                    Vector2f(
                        static_cast<float>(col),
                        static_cast<float>(row)
                    )
                );

                result.emplace_back(world_position);
            }
        }
    }

    return result;
}

void onMapReceived(const nav_msgs::OccupancyGrid::ConstPtr& map_msg) {
    if (has_map) {
        ROS_WARN("Mappa già ricevuta, ignoro il nuovo messaggio.");
        return;
    }

    ROS_INFO(
        "Mappa ricevuta. Frame: %s",
        map_msg->header.frame_id.c_str()
    );

    map_resolution = map_msg->info.resolution;

    Vector2f map_origin(
        map_msg->info.origin.position.x,
        map_msg->info.origin.position.y
    );

    occupied_cells.clear();

    map_handler.reset(map_origin, map_resolution);

    occupied_cells = extractObstaclesFromMap(map_msg);

    if (occupied_cells.empty()) {
        ROS_WARN("La mappa non contiene celle occupate: attendo una mappa valida.");
        return;
    }

    // setMap modifica X: conserva una posa eventualmente arrivata prima della mappa.
    const Eigen::Isometry2f previous_pose = robot_localizer.X;
    robot_localizer.setMap(
        occupied_cells,
        map_resolution,
        max_influence_distance
    );

    robot_localizer.X.setIdentity();
    if (has_start_pose) {
        robot_localizer.X = previous_pose;
    }
    has_map = true;

    ROS_INFO(
        "Mappa processata con %lu ostacoli",
        occupied_cells.size()
    );
}

void computeScanEndpoints(
    std::vector<Vector2f>& destination,
    const sensor_msgs::LaserScan::ConstPtr& scan_msg
) {
    destination.clear();

    for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {

        const float range = scan_msg->ranges[i];

        if (!std::isfinite(range) ||
            range < scan_msg->range_min ||
            range > scan_msg->range_max) {
            continue;
        }

        const float angle =
            scan_msg->angle_min +
            static_cast<float>(i) * scan_msg->angle_increment;

        destination.emplace_back(
            range * std::cos(angle),
            range * std::sin(angle)
        );
    }
}

void onLaserScanReceived(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
    if (!has_map || !has_start_pose) {
        ROS_WARN_THROTTLE(
            1.0,
            "In attesa della mappa e della posa iniziale..."
        );
        return;
    }

    std::vector<Vector2f> scan_points;

    computeScanEndpoints(scan_points, scan_msg);

    if (scan_points.empty()) {
        ROS_WARN_THROTTLE(1.0, "Scansione senza misure valide: aggiornamento saltato.");
        return;
    }

    robot_localizer.localize(scan_points, 10);

    sendOdometryMessage(scan_msg->header.stamp);

    ROS_INFO(
        "Posa stimata: x = %.2f, y = %.2f, theta = %.2f",
        robot_localizer.X.translation().x(),
        robot_localizer.X.translation().y(),
        getYawFromTransform()
    );
}

void onInitialPoseReceived(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg
) {
    if (has_start_pose) {
        ROS_WARN("Posa iniziale già ricevuta, ignoro il nuovo messaggio.");
        return;
    }

    ROS_INFO(
        "Posa iniziale ricevuta. Frame: %s",
        pose_msg->header.frame_id.c_str()
    );
    
    robot_localizer.X.setIdentity();

    robot_localizer.X.translation().x() = pose_msg->pose.pose.position.x;
    robot_localizer.X.translation().y() = pose_msg->pose.pose.position.y;

    Eigen::Quaternionf quaternion(
        pose_msg->pose.pose.orientation.w,
        pose_msg->pose.pose.orientation.x,
        pose_msg->pose.pose.orientation.y,
        pose_msg->pose.pose.orientation.z
    );

    float yaw = quaternion.toRotationMatrix().eulerAngles(0, 1, 2)[2];

    robot_localizer.X.linear() =
        Eigen::Rotation2Df(yaw).toRotationMatrix();

    has_start_pose = true;

    ROS_INFO(
        "Posa inizializzata: x = %.2f, y = %.2f, yaw = %.2f",
        pose_msg->pose.pose.position.x,
        pose_msg->pose.pose.position.y,
        getYawFromTransform()
    );
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "listener_node");

    ros::NodeHandle node;

    odom_publisher = node.advertise<nav_msgs::Odometry>(
        "/odom_corrected",
        10
    );

    ros::Subscriber map_subscriber = node.subscribe(
        "/map",
        1,
        onMapReceived
    );

    ros::Subscriber pose_subscriber = node.subscribe(
        "/initialpose",
        1,
        onInitialPoseReceived
    );

    ros::Subscriber laser_subscriber = node.subscribe(
        "/base_scan",
        1,
        onLaserScanReceived
    );

    ROS_INFO("Nodo listener avviato.");

    ros::spin();

    return 0;
}
