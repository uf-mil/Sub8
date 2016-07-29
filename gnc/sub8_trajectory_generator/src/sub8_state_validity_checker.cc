/**
* Author: Patrick Emami
* Date: 9/22/15
*/
#include "sub8_state_validity_checker.h"
#include "sub8_state_space.h"
#include <cmath>

using sub8::trajectory_generator::Sub8StateValidityChecker;
using sub8::trajectory_generator::Sub8StateSpace;

bool Sub8StateValidityChecker::isValid(const State* state) const {
  double x = state->as<Sub8StateSpace::StateType>()->getX();
  double y = state->as<Sub8StateSpace::StateType>()->getY();
  double z = state->as<Sub8StateSpace::StateType>()->getZ();

  bool near_obstacle =
      _obs_checker->isNearObstacle(x, y, z, _obstacle_checking_tolerance);

  // otherwise, return true so long as bounds are respected
  return !near_obstacle && si_->satisfiesBounds(state);
}