#include "dmap_localizer.h"
#include <iostream>
#include <fstream>

using Matrix_2_3f = Eigen::Matrix<float, 2,3>;
using Matrix_1_2f = Eigen::Matrix<float, 1,2>;
using Matrix_1_3f = Eigen::Matrix<float, 1,3>;
using Eigen::Vector3f;
using Eigen::Matrix3f;
using Eigen::Matrix2f;
using Eigen::Rotation2Df;

using namespace std;

void DMapLocalizer::setMap(const GridMapping& mapping,
                           const Grid_<float>& distances_) {
  grid_mapping = mapping;
  distances=distances_;
  distances.colDerivative(distances_dc);
  distances.rowDerivative(distances_dr);
  X.setIdentity();
}

void DMapLocalizer::setMap(const std::vector<Eigen::Vector2f> obstacles,
                           float resolution,
                           float influence_range) {

  //1 compute the bounding box of the obstacles
  Vector2f lower_left(std::numeric_limits<float>::max(),
                      std::numeric_limits<float>::max());
  Vector2f upper_right(std::numeric_limits<float>::min(),
                       std::numeric_limits<float>::min());
  for (const auto& o: obstacles) {
    lower_left.x()=std::min(lower_left.x(), o.x());
    lower_left.y()=std::min(lower_left.y(), o.y());
    upper_right.x()=std::max(upper_right.x(), o.x());
    upper_right.y()=std::max(upper_right.y(), o.y());
  }
  grid_mapping.reset(Vector2f(lower_left.x(), upper_right.y()), resolution);

  Vector2f bbox=upper_right-lower_left;
  //3. construct the map the obstacles in grid coordinates and construct the dmap
  int rows=ceil(bbox.y())/resolution+1;
  int cols=ceil(bbox.x())/resolution+1;
  std::vector<Vector2i> grid_obstacles;
  grid_obstacles.reserve(obstacles.size());
  for (const auto& o: obstacles){
    grid_obstacles.push_back(grid_mapping.world2grid(o).cast<int>());
  }

  // calculate the influence range, squared in pixels
  int dmax_2=pow(influence_range/resolution,2);
  DMap dmap(rows, cols);
  dmap.clear();
  int ops = dmap.compute(grid_obstacles, dmax_2);
  
  //compute from the dmap the distances
  dmap.copyTo(distances, dmax_2);

  // convert it from pixels (squared) to meters
  for (auto& f: distances.cells)
    f*=resolution;

  // compute the derivatives
  distances.colDerivative(distances_dc);
  distances.rowDerivative(distances_dr);

  // initialize the robot to be in the middle of the map
  X.setIdentity();
  X.translation()=0.5*(upper_right+lower_left);
}

// initialization function, from map
void DMapLocalizer::setMap(const GridMap& grid_map,
                           float influence_range,
                           uint8_t occ_threshold) {
  grid_mapping = grid_map;
  std::vector<Vector2i> grid_obstacles;
  for (int r=0; r<grid_map.rows; ++r)
    for (int c=0; c<grid_map.cols; ++c)
      if (grid_map(r,c)<occ_threshold) {
        grid_obstacles.push_back(Vector2i(c,r));
      }

  int dmax_2=pow(influence_range/grid_mapping.resolution(),2);
  DMap dmap(grid_map.rows, grid_map.cols);
  dmap.clear();
  int ops = dmap.compute(grid_obstacles, dmax_2);
  
  //compute from the dmap the distances
  dmap.copyTo(distances, dmax_2);

  // convert it from pixels (squared) to meters
  for (auto& f: distances.cells)
    f=sqrt(f)*grid_mapping.resolution();

  // compute the derivatives
  distances.colDerivative(distances_dc);
  distances.rowDerivative(distances_dr);
}


// this is magic for now
bool DMapLocalizer::localize(const std::vector<Vector2f>& measurements,
                             int iterations) {
  int inliers=0;
  float chi2=0;
  for (int i=0; i<iterations; ++i) {
    Matrix3f H;
    Vector3f b;
    Matrix_2_3f J_icp;
    const Matrix2f J_gm = grid_mapping.world2gridDerivative();
    Matrix_1_2f J_dmap;
    Matrix_1_3f J;
    
    J_icp.block<2,2>(0,0).setIdentity();
    inliers=0;
    H.setZero();
    b.setZero();
    for (const auto& m: measurements) {
      Vector2f p_world = X*m;
      Vector2f p_grid=grid_mapping.world2grid(p_world);
      if (! distances.inside(p_grid))
        continue;
      float e=distances(p_grid);
      float e2=e*e;
      float lambda=1.f;
      if (e2>kernel_chi2) {
        lambda=kernel_chi2/sqrt(e2);
      }
        
      J_icp.col(2) << -p_world.y(), p_world.x();
      J_dmap.x()=distances_dc(p_grid);
      J_dmap.y()=distances_dr(p_grid);
      
      J = J_dmap * J_gm * J_icp;
      H += lambda*J.transpose()*J;
      b += lambda*J.transpose()*e;
      chi2+=e2;
      ++ inliers;
    }
    H+=Eigen::Matrix3f::Identity()*damping;
    Vector3f dx= H.ldlt().solve(-b);
    Isometry2f dX;
    dX.translation() << dx.x(), dx.y();
    dX.linear()=Rotation2Df(dx.z()).matrix();
    X=dX*X;
    Vector3f x;
    x.x() = X.translation().x();
    x.y() = X.translation().y();
    x.z()=Rotation2Df(X.linear()).angle();
  }
  return inliers > inliers_min;

}
