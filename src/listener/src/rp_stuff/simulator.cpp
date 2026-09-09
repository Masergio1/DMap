#include <iostream>
#include "grid_map.h"
#include "world_item.h"
#include "laser_scanner.h"

using namespace std;

Isometry2f fromCoefficients(float tx, float ty, float alpha) {
  Isometry2f iso;
  iso.setIdentity();
  iso.translation()<< tx, ty;
  iso.linear()=Eigen::Rotation2Df(alpha).matrix();
  return iso;
}

int main(int argc, char** argv) {
  if (argc < 2) {
    cout << "usage: " << argv[0] << " <image_file> <resolution>" << endl;
    return -1;
  }
  const char* filename = argv[1];
  float resolution = atof(argv[2]);

  cout << "Running " << argv[0] << " with arguments" << endl
       << "-filename:" << argv[1] << endl
       << "-resolution: " << argv[2] << endl;

  GridMap grid_map(0, 0, 0.1);
  grid_map.loadFromImage(filename, resolution);


  World world_object(grid_map);
  WorldItem object_0(world_object, fromCoefficients(5, 0, 0.5));
  Vector2f grid_middle(grid_map.cols/2, grid_map.rows/2);
  Vector2f world_middle = grid_map.grid2world(grid_middle);
  UnicyclePlatform robot(world_object, fromCoefficients(world_middle.x(), world_middle.y(), -0.5));
  robot.radius=1;

  LaserScan scan;
  LaserScanner scanner(scan, robot, fromCoefficients(3, 0, -0));
  scanner.radius = 0.5;

  float dt=0.1;
  Canvas canvas;
  while (true) {
    world_object.tick(dt);
    world_object.draw(canvas);
    int ret = showCanvas(canvas, dt*100);
    if (ret>0)
      std::cerr << "Key pressed: " << ret << std::endl;

    switch(ret) {
    case 81: //left;
      robot.rv+=0.1;
      break;
    case 82: //up;
      robot.tv+=0.1;
      break;
    case 83: //right;
      robot.rv-=0.1;
      break;
    case 84: //down;
      robot.tv-=0.1;
      break;
    case 32: // space
      robot.rv=0;
      robot.tv=0;
      break;
    default:;
    }
    
  }
}
