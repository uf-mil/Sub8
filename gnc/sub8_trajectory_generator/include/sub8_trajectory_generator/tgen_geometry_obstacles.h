/**
* Author: Patrick Emami
* Date: 2/26/16
*/
#ifndef TGEN_GEOMETRY_OBSTACLES_H_
#define TGEN_GEOMETRY_OBSTACLES_H_

#include <vector>
#include "tgen_obstacle_checker.h"

namespace sub8 {

namespace trajectory_generator {

class TGenGeometryObstacles;

// Typedef for shared_ptr wrapper
typedef boost::shared_ptr<TGenGeometryObstacles> TGenGeometryObstaclesPtr;

class TGenGeometryObstacles : public TGenObstacleChecker {
 public:
  TGenGeometryObstacles();
  
  // Returns true if the vehicle is within tolerance of any obstacle
  virtual bool isNearObstacle(double x, double y, double z,
                              double tolerance) const override;

  // Add a cube centered at (x,y,z) with side length s for testing purposes
  void addCube(double x, double y, double z, double s);

  // Delete all elements of the vector storing the obstacles, used for resetting 
  // for a new planning task 
  void removeAllObstacles();
  
 private:
  struct RectObstacle {
    double xlow;
    double xhigh;
    double ylow;
    double yhigh;
    double zlow;
    double zhigh;
  };

  // List of obstacles
  std::vector<RectObstacle> obstacles;
};
}
}
#endif /* TGEN_GEOMETRY_OBSTACLES_H_ */