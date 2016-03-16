/**
* Author: Patrick Emami
* Date: 2/26/16
*/
#pragma once

#include <boost/shared_ptr.hpp>

namespace sub8 {

namespace trajectory_generator {

// forward declaration for shared_ptr definition
class TGenObstacleChecker;

// Typedef for shared_ptr wrapper
typedef boost::shared_ptr<TGenObstacleChecker> TGenObstacleCheckerPtr;

class TGenObstacleChecker {
 public:
  virtual bool isNearObstacle(double x, double y, double z,
                              double tolerance) const = 0;

 protected:
  virtual ~TGenObstacleChecker() {};
};
}
}
