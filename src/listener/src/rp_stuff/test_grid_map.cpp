#include <iostream>
#include "grid_map.h"

using Eigen::Isometry2f;


using namespace std;

int main(int argc, char** argv) {
  if (argc<2) {
    cout << "usage: " << argv[0] << " <image_file> <resolution>" << endl;
    return -1;
  }
  const char* filename=argv[1];
  float resolution=atof(argv[2]);

  cout << "Running " << argv[0]
       << " with arguments" << endl
       << "-filename:" << argv[1] << endl
       << "-resolution: " << argv[2] << endl;

  GridMap grid_map(0,0, 0.1);
  grid_map.loadFromImage(filename, resolution);
  Canvas canvas;
  Vector2f world_center=grid_map.center();
  cerr << "world_center: " << world_center.transpose() << endl;

  Vector2f grid_center=grid_map.world2grid(world_center);
  cerr << "grid_center: " << grid_center.transpose() << endl;
  
  float alpha=0;
  while(1) {
    grid_map.draw(canvas);
    Vector2f direction;
    direction[0]=cos(alpha);
    direction[1]=sin(alpha);
    float range = grid_map.scanRay(world_center, direction, 100);
    Vector2f world_hit = world_center + direction * range;
    Vector2f grid_hit  = grid_map.world2grid(world_hit);
    cerr << "world_endpoint: " << world_hit << endl;

    drawLine(canvas, grid_center, grid_hit, 127);
    
    showCanvas(canvas, 0);
    
    alpha+=0.01;
    cerr << "alpha: " << alpha << endl;
  }
}
