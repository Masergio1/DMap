#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "dmap.h"
#include "grid_map.h"

using Eigen::Vector2f;
using Eigen::Isometry2f;

struct DMapLocalizer {
  // robot pose
  Isometry2f X=Isometry2f::Identity();
  
  // alg parameters: 
  float damping=1.;
  int inliers_min=10;
  float kernel_chi2=1.f;

  //internal stuff
  GridMapping grid_mapping;
  Grid_<float> distances, distances_dr, distances_dc;

  //initialization function, from obstacles
  void setMap(const std::vector<Eigen::Vector2f> obstacles,
              float res,
              float influence_range);

  void setMap(const GridMapping& mapping,
              const Grid_<float>& distances_);

  // initialization function, from grid_map
  void setMap(const GridMap& grid_map,
              float influence_range=2,
              uint8_t occ_threshold=127);
  
  bool localize(const std::vector<Vector2f>& measurements,
                int iterations=1);
  
};
