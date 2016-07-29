/**
* Author: Patrick Emami
* Date: 9/22/15
*
*/
#ifndef SUB8_STATE_VALIDITY_CHECKER_H_
#define SUB8_STATE_VALIDITY_CHECKER_H_

#include "ompl/base/StateValidityChecker.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/base/Path.h"
#include "tgen_common.h"
#include "tgen_obstacle_checker.h"
#include "tgen_geometry_obstacles.h"

using ompl::base::StateValidityChecker;
using ompl::base::Path;
using ompl::base::State;
using ompl::base::SpaceInformationPtr;

namespace sub8 {

namespace trajectory_generator {

// forward declaration for the typedef
class Sub8StateValidityChecker;

// Typedef for shared_ptr wrapper
typedef boost::shared_ptr<Sub8StateValidityChecker> Sub8StateValidityCheckerPtr;

// Encapsulates collision checking and other functionality
// necessary for determining whether a trajectory is safe
class Sub8StateValidityChecker : public StateValidityChecker {
 public:
  Sub8StateValidityChecker(const SpaceInformationPtr& si,
                           double obstacle_tolerance)
      : StateValidityChecker(si),
        _obstacle_checking_tolerance(obstacle_tolerance),
        _obs_checker(new TGenGeometryObstacles()) {}
  /////////////////////////////////////////////////////
  // Inherited methods
  ////////////////////////////////////////////////////

  // Return true if the state is valid. Usually,
  // this means at least collision checking. If it is
  // possible that ompl::base::StateSpace::interpolate()
  // or ompl::control::ControlSpace::propagate() return states
  // that are outside of bounds, this function should also make
  // a call to ompl::control::SpaceInformation::satisfiesBounds().
  virtual bool isValid(const State* state) const override;

 private:
  TGenObstacleCheckerPtr _obs_checker;
  double _obstacle_checking_tolerance;
};
}
}
#endif /* SUB8_STATE_VALIDITY_CHECKER */