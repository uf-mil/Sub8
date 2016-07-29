/**
* Author: Patrick Emami
* Date: 2/26/16
*/
#include "tgen_geometry_obstacles.h"
#include <ros/ros.h>

unsigned char sign(double x) { return x >= 0 ? 1 : 0; }

namespace sub8 {

namespace trajectory_generator {

TGenGeometryObstacles::TGenGeometryObstacles() {
  int num_obstacles;

  if (ros::param::has("test_geometric_obstacles")) {
    ros::param::get("test_geometric_obstacles", num_obstacles);
    assert(num_obstacles >= 0); 
    assert(num_obstacles < 3);
    ROS_WARN("Using %d geometric obstacles in scene", num_obstacles);

  } else {
    num_obstacles = 0;
  }

  for (unsigned int i = 0; i < num_obstacles; ++i) {
    switch (i) {
      case 0: 
        addCube(0, 0, 0, 3); 
        break;
      case 1: 
        addCube(1, 2, 0, 1.5);
        break;
      case 2: 
        addCube(3, 4, -1, 1);
        break;
    }
  }
}

bool TGenGeometryObstacles::isNearObstacle(double x, double y, double z,
                                           double t) const {
  assert(t >= 0);  // only allow non-neg tolerances

  for (unsigned int i = 0; i < obstacles.size(); ++i) {
    RectObstacle temp = obstacles[i];
    // Basically, grow the size of the geometric obstacles based on the tolerance value

    // if negative make more negative
    // if positive make more positive
    sign(temp.xlow) ? temp.xlow += t : temp.xlow -= t;
    sign(temp.xhigh) ? temp.xhigh += t : temp.xhigh -= t;
    sign(temp.ylow) ? temp.ylow += t : temp.ylow -= t;
    sign(temp.yhigh) ? temp.yhigh += t : temp.yhigh -= t;
    sign(temp.zlow) ? temp.zlow += t : temp.zlow -= t;
    sign(temp.zhigh) ? temp.zhigh += t : temp.zhigh -= t;

    if ((x > temp.xlow) && (x < temp.xhigh) && (y > temp.ylow) &&
        (y < temp.yhigh) && (z > temp.zlow) && (z < temp.zhigh)) {
      return true;
    }
  }
  return false;
}

void TGenGeometryObstacles::addCube(double x, double y, double z, double s) {
  // side length mut be positive
  assert(s > 0);

  RectObstacle ro;
  ro.xlow = x - s / 2.0;
  ro.xhigh = x + s / 2.0;
  ro.ylow = y - s / 2.0;
  ro.yhigh = y + s / 2.0;
  ro.zlow = z - s / 2.0;
  ro.zhigh = z + s / 2.0;
  obstacles.push_back(ro);
}

void TGenGeometryObstacles::removeAllObstacles() {
  obstacles.clear();
}
}
}