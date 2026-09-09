#include "dmap.h"
#include <cstdlib>
#include <fstream>
using namespace std;

int main(int argc, char** argv) {
  DMap d_map(1000, 1000);
  int num_obstacles = 10;
  int dmax2=1000*1000;
  std::vector<Vector2i> obstacles;
  obstacles.resize(num_obstacles);
  
  for (auto& o: obstacles) {
    o=Vector2i(d_map.cols*drand48(), d_map.rows*drand48());
  }
  int num_ops = d_map.compute(obstacles, dmax2);
  std::cerr << "num_ops " << num_ops << std::endl;

  Grid_<float> distances;
  d_map.copyTo(distances, dmax2);

  Grid_<float> distances_dr;
  distances.rowDerivative(distances_dr);
  Grid_<float> distances_dc;
  distances.colDerivative(distances_dc);
  

  bool equalize=false;
  int what_to_show=0;
  while (1) {
    Canvas canvas;
    switch (what_to_show) {
    case 0: distances.draw(canvas, equalize);break;
    case 1: distances_dr.draw(canvas, equalize);break;
    case 2: distances_dc.draw(canvas, equalize);break;
    default:  distances.draw(canvas, equalize);
    }
    
    int key = showCanvas(canvas, 0);
    switch (key) {
    case ' ': ++ what_to_show; break;
    case 'e': equalize = ! equalize; break;
    default:;
    }

    what_to_show = what_to_show%3;
  }

}
