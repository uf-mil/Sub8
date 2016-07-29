/**
* Author: Patrick Emami
* Date: 9/22/15
*/
#include "space_information_generator.h"
#include "sub8_state_validity_checker.h"
#include "sub8_state_space.h"
#include "ompl/base/spaces/RealVectorBounds.h"
#include "ompl/base/SpaceInformation.h"
#include <ros/console.h>

using sub8::trajectory_generator::SpaceInformationGenerator;
using sub8::trajectory_generator::Sub8StateValidityChecker;
using sub8::trajectory_generator::Sub8StateValidityCheckerPtr;
using sub8::trajectory_generator::Sub8StateSpace;
using ompl::base::SpaceInformationPtr;
using ompl::base::RealVectorBounds;

SpaceInformationPtr SpaceInformationGenerator::generate() {
  double checking_res;
  double obstacle_tolerance;

  // Grab config parameters
  if (ros::param::has("state_validity_checking_resolution")) {
    ros::param::get("state_validity_checking_resolution", checking_res);
  } else {
    checking_res = 0.03;  // default
    ROS_WARN(
        "Unable to retrieve state validity checking resolution, using default "
        "value %f",
        checking_res);
  }
  
  if (ros::param::has("obstacle_checking_tolerance")) {
    ros::param::get("obstacle_checking_tolerance", obstacle_tolerance);
  } else {
    obstacle_tolerance = 0.5;  // default
    ROS_WARN(
        "Unable to retrieve obstacle avoidance tolerance, using default "
        "tolerance %f",
        obstacle_tolerance);
  }

  ompl::base::StateSpacePtr space(new Sub8StateSpace());
  setStateSpaceBounds(space);

  SpaceInformationPtr si_ptr(new ompl::base::SpaceInformation(space));

  // Create and set the state validity checker
  Sub8StateValidityCheckerPtr vc_ptr(
      new Sub8StateValidityChecker(si_ptr, obstacle_tolerance));
  si_ptr->setStateValidityChecker(
      static_cast<ompl::base::StateValidityCheckerPtr>(vc_ptr));

  // configuration
  si_ptr->setStateValidityCheckingResolution(checking_res);

  // Must be run once before use
  si_ptr->setup();

  return si_ptr;
}

void SpaceInformationGenerator::setStateSpaceBounds(
    const StateSpacePtr& space) {
  // Bounds on position (x, y, z)
  RealVectorBounds pos_bounds(3);

  if (ros::param::has("xmin")) {
    ros::param::get("xmin", pos_bounds.low[0]);
    ros::param::get("xmax", pos_bounds.high[0]);
    ros::param::get("ymin", pos_bounds.low[1]);
    ros::param::get("ymax", pos_bounds.high[1]);
    ros::param::get("zmin", pos_bounds.low[2]);
    ros::param::get("zmax", pos_bounds.high[2]);
  } else {
    double bound = 5; // default
    ROS_WARN(
        "Unable to retrieve state space bounds, using default values of +/- %f "
        "for each dimension",
        bound);
    // default values
    pos_bounds.low[0] = -bound;
    pos_bounds.high[0] = bound;
    pos_bounds.low[1] = -bound;
    pos_bounds.high[1] = bound;
    pos_bounds.low[2] = -bound;
    pos_bounds.high[2] = bound;
  }

  // set bounds on the R^3 component
  space->as<Sub8StateSpace>()->setBounds(pos_bounds);
}