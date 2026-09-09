#include <cstdlib>
#include <iostream>
#include <fstream>
#include <cmath>
#include <sys/time.h>
#include "dmap_localizer.h"
#include "draw_helpers.h"

using namespace std;
using Eigen::Isometry2f;
using Eigen::Rotation2Df;

int main(int argc, char** argv) {
  if (argc<5) {
    std::cout << "usage " << argv[0] << " <num_obstacles> <range> <resolution> <dmax>" << std::endl;
    return -1;
  }

  int num_obstacles=atoi(argv[1]);
  float range=atof(argv[2]);
  float resolution = atof(argv[3]);
  float dmax = atof(argv[4]);
  cerr << "parameters: " << endl;
  cerr << " obstacles: " << num_obstacles << endl;
  cerr << " range: " << range << endl;
  cerr << " resolution: " << resolution << endl;
  cerr << " dmax_influence: " << dmax << endl;

  // generate obstacles
  std::vector<Vector2f> obstacles;
  obstacles.reserve(num_obstacles);
  for (int i=0; i<num_obstacles; ++i) {
    obstacles.push_back(Vector2f::Random()*range);
  }

  cerr << "obstacles" << endl;
  DMapLocalizer localizer;
  localizer.setMap(obstacles, resolution, dmax);
  cerr << "localizer ready" << endl;
  cerr << "rows:  " << localizer.distances.rows << " cols: " << localizer.distances.cols << endl;

  // prepare canvas for visualization
  Canvas canvas;
  const auto& distances = localizer.distances;
  Grid_<uint8_t> image(distances.rows, distances.cols);
  
  // we draw in the image the dmap for visualization
  // 1. compute normalization
  float f_min=1e9;
  float f_max=0;
  for(auto& f: distances.cells) {
    f_min=std::min(f, f_min);
    f_max=std::max(f, f_max);
  }
  float scale=255./(f_max-f_min);

  // 2. copy the (normalized) distances
  for (size_t i=0; i<distances.cells.size(); ++i) {
    image.cells[i]=scale  * (distances.cells[i] - f_min);
  }

  // 3. sugar: add an obstacle image as alternative background
  Grid_<uint8_t> obstacle_image(distances.rows, distances.cols);
  for (int i=0; i<obstacle_image.cells.size(); ++i)
    obstacle_image.cells[i]=0;
  
  obstacle_image.draw(canvas, false);
  for (const auto& m: obstacles) {
    Vector2f m_hat_grid=localizer.grid_mapping.world2grid(m);
    drawCircle(canvas, m_hat_grid, 3, 255);
  }
  // we draw with cv and get back the result, dirty...
  memcpy(&obstacle_image.cells[0], canvas.data, distances.rows*distances.cols);

  // now run the localizer
  Isometry2f X=Eigen::Isometry2f::Identity();
  X.linear()=Rotation2Df(0.3).matrix();
  X.translation()<< 1, 0.5;
  localizer.X=X;
  bool show_obstacles=false;
  while (1) {
    if (show_obstacles)
      obstacle_image.draw(canvas, false);
    else
      image.draw(canvas, false);
    for (const auto& m: obstacles) {
      Vector2f m_hat=localizer.X*m;
      Vector2f m_hat_grid=localizer.grid_mapping.world2grid(m_hat);
      drawCircle(canvas, m_hat_grid, 3,127);
    }
    int key = showCanvas(canvas,0);
    if (key == 32) {
      show_obstacles = !show_obstacles;
      continue;
    }

    struct timeval tv_start, tv_end, tv_delta;
    gettimeofday(&tv_start,0);
    localizer.localize(obstacles, 1);
    gettimeofday(&tv_end,0);
    timersub(&tv_end, &tv_start, &tv_delta);
    cout << "time: " << tv_delta.tv_sec*1e3 + tv_delta.tv_usec*1e-3 << endl;
  }
  
}
