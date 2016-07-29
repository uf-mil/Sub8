/**
 * Author: Patrick Emami
 * Date: 9/29/15
 *
 */

#include "tgen_manager.h"
#include "tgen_common.h"
#include "sub8_state_space.h"
#include "ompl/base/PlannerStatus.h"
#include "ompl/base/goals/GoalState.h"
#include "ompl/geometric/planners/rrt/RRTstar.h"
#include "ompl/geometric/planners/rrt/RRTConnect.h"
#include "ompl/geometric/planners/AnytimePathShortening.h"
#include "ompl/geometric/PathGeometric.h"
#include "ompl/base/spaces/RealVectorBounds.h"
#include <boost/filesystem.hpp>
#include <ros/console.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

// File IO to save solution to file
#if 1
#include <iostream>
#include <fstream>
#endif

using sub8::trajectory_generator::TGenManager;
using sub8::trajectory_generator::TGenMsgs;
using sub8::trajectory_generator::SpaceInformationGeneratorPtr;
using sub8::trajectory_generator::Sub8StateSpace;
using ompl::base::Planner;
using ompl::base::PlannerStatus;
using ompl::base::GoalState;
using ompl::base::ProblemDefinitionPtr;
using ompl::base::RealVectorBounds;
using ompl::geometric::PathGeometric;

namespace fs = ::boost::filesystem;

TGenManager::TGenManager(AlarmBroadcasterPtr&& ab) : _alarm_broadcaster(ab) {
  // int planner_type;
  int num_planners;
  double range;

  const std::string planning_failure_alarm =
      "sub8_trajectory_generator_planning_failure";

  _pdef = nullptr;  // Problem definition hasn't been set yet
  SpaceInformationGeneratorPtr ss_gen(new SpaceInformationGenerator());

  _sub8_si = ss_gen->generate();

  if (ros::param::has("num_planners")) {
    ros::param::get("num_planners", num_planners);
    ROS_WARN("Running the OMPL Anytime Planner with %d planners", num_planners);
  } else {
    num_planners = 5;  // default
    ROS_WARN("Unable to retrieve desired num of planners, using default %d",
             num_planners);
  }

  if (ros::param::has("range")) {
    ros::param::get("range", range);
  } else {
    range = 0.05;  // default
    ROS_WARN("Unable to retrieve desired range, using default %f", range);
  }

  // For integration testing, grab a shorter test time
  if (ros::param::has("test_solve_time")) {
    ros::param::get("test_solve_time", _solve_time);
  } else {
    if (ros::param::has("solve_time")) {
      ros::param::get("solve_time", _solve_time);
    } else {
      _solve_time = 30.0;  // default
      ROS_WARN(
          "Unable to retrieve desired allowed solve time, using default %f",
          _solve_time);
    }
  }

  ROS_WARN("Alloted time for finding a path is %f", _solve_time);

  _sub8_planner =
      PlannerPtr(new ompl::geometric::AnytimePathShortening(_sub8_si));

  for (unsigned int i = 0; i < num_planners; ++i) {
    PlannerPtr rrt_connect(new ompl::geometric::RRTConnect(_sub8_si));
    rrt_connect->as<ompl::geometric::RRTConnect>()->setRange(range);
    _sub8_planner->as<ompl::geometric::AnytimePathShortening>()->addPlanner(
        rrt_connect);
  }

  // initialize alarms
  _planning_failure_alarm =
      _alarm_broadcaster->addJSONAlarm(planning_failure_alarm);
  assert(_planning_failure_alarm != nullptr);
}

bool TGenManager::setProblemDefinition(const State* start_state,
                                       const State* goal_state) {
  // Check the state validity incase the start or goal state
  // is on top of an obstacle
  if (!(_sub8_si->getStateValidityChecker()->isValid(start_state)) ||
      !(_sub8_si->getStateValidityChecker()->isValid(goal_state))) {
    ROS_ERROR("Bad start state or goal state provided");

    ROS_ERROR("start_state.x: %f",
              start_state->as<Sub8StateSpace::StateType>()->getX());
    ROS_ERROR("start_state.y: %f",
              start_state->as<Sub8StateSpace::StateType>()->getY());
    ROS_ERROR("start_state.z: %f",
              start_state->as<Sub8StateSpace::StateType>()->getZ());
    ROS_ERROR("start_state.yaw: %f",
              start_state->as<Sub8StateSpace::StateType>()->getYaw());

    ROS_ERROR("goal_state.x: %f",
              start_state->as<Sub8StateSpace::StateType>()->getX());
    ROS_ERROR("goal_state.y: %f",
              start_state->as<Sub8StateSpace::StateType>()->getY());
    ROS_ERROR("goal_state.z: %f",
              start_state->as<Sub8StateSpace::StateType>()->getZ());
    ROS_ERROR("goal_state.yaw: %f",
              start_state->as<Sub8StateSpace::StateType>()->getYaw());

    return false;
  }

  if (!(_sub8_planner->isSetup())) {
    _sub8_planner->setup();
  }

  _sub8_planner->clear();
  _pdef = ProblemDefinitionPtr(new ProblemDefinition(_sub8_si));

  // the third arg is the threshold for max distance allowed to the goal
  _pdef->setStartAndGoalStates(start_state, goal_state);

  _sub8_planner->setProblemDefinition(_pdef);

  // Verifies that the problem definition was correctly set
  _sub8_planner->checkValidity();

  return true;
}

bool TGenManager::solve() {
  bool success = false;

  // arg for "solve" is the solve time
  PlannerStatus pstatus = _sub8_planner->solve(_solve_time);

  // Switch on the value of the PlannerStatus enum "StateType"
  // Only raise alarms on issues that require a system response.
  // Most errors here are due to programming errors, so just
  // display an error message to shame the programmer.
  switch (pstatus.operator StatusType()) {
    case PlannerStatus::UNRECOGNIZED_GOAL_TYPE:
      ROS_ERROR("%s", TGenMsgs::UNRECOGNIZED_GOAL_TYPE);
      break;
    case PlannerStatus::TIMEOUT:
      ROS_ERROR("%s", TGenMsgs::TIMEOUT);
      break;
    case PlannerStatus::APPROXIMATE_SOLUTION:
      ROS_WARN("%s", TGenMsgs::APPROXIMATE_SOLUTION);
      // What is our tolerance for this?
      success = true;
      break;
    case PlannerStatus::EXACT_SOLUTION:
      ROS_INFO("%s", TGenMsgs::EXACT_SOLUTION);
      success = true;
      break;
    case PlannerStatus::CRASH:
      ROS_ERROR("%s", TGenMsgs::CRASH);
      break;
    default:
      break;
  }

  if (!success) {
    _planning_failure_alarm->raiseAlarm();
  }
  return success;
}

void TGenManager::validateCurrentPath() {
  // Planner get start state, get goal state
  unsigned int first_invalid_state = 0;

  ProblemDefinitionPtr pdef = _sub8_planner->getProblemDefinition();
  PathGeometric* spath =
      static_cast<PathGeometric*>(pdef->getSolutionPath().get());

  // Do trajectory validation; first_invalid_state is populated by checkMotion()
  if (!_sub8_si->checkMotion(spath->getStates(), spath->getStateCount(),
                             first_invalid_state)) {
    // Log that the current trajectory needs to be re-planned
    ROS_WARN("%s", TGenMsgs::REPLAN_NEEDED);

    // Go ahead and supply the controller with a safety path
    // at this point. For now, raise alarm to tell system to
    // stop sub motion (but not abort mission)

    // naive replanning; will implement smarter re-planning later
    State* new_start_state = spath->getState(first_invalid_state);

    // Replan from the first invalid state to the goal state
    setProblemDefinition(
        new_start_state,
        static_cast<GoalState*>(pdef->getGoal().get())->getState());
    solve();

  } else {
    ROS_INFO("%s", TGenMsgs::TRAJECTORY_VALIDATED);
  }
}

State* TGenManager::poseToState(const geometry_msgs::Pose& pose) {
  State* state = _sub8_si->getStateSpace()->allocState();

  // set position
  state->as<Sub8StateSpace::StateType>()->setXYZ(
      pose.position.x, pose.position.y, pose.position.z);
  double yaw = 0;
  double qx = pose.orientation.x;
  double qy = pose.orientation.y;
  double qz = pose.orientation.z;
  double qw = pose.orientation.w;

  double t = qx * qy + qz * qw;
  if (approx(t, 0.5)) {  // singularity at north pole
    yaw = 2 * atan2(qx, qw);
  } else if (approx(t, -0.5)) {  // singularity at south pole
    yaw = -2 * atan2(qx, qw);
  } else {
    yaw = atan2(2 * qy * qw - 2 * qx * qz, 1 - 2 * pow(qy, 2) - 2 * pow(qz, 2));
  }
  // set yaw
  state->as<Sub8StateSpace::StateType>()->setYaw(yaw);

  return state;
}

sub8_msgs::PathPoint TGenManager::stateToPathPoint(const State* s) {
  sub8_msgs::PathPoint path_point;
  const Sub8StateSpace::StateType* sub8_se3 =
      s->as<Sub8StateSpace::StateType>();

  path_point.position.x = sub8_se3->getX();
  path_point.position.y = sub8_se3->getY();
  path_point.position.z = sub8_se3->getZ();
  path_point.yaw = sub8_se3->getYaw();

  return path_point;
}

std::vector<State*> TGenManager::getPath() {
#if 1
  using namespace std;

  ofstream out_file;
  out_file.open("path.txt", ios::trunc);
  (static_cast<PathGeometric*>(
       _sub8_planner->getProblemDefinition()->getSolutionPath().get()))
      ->printAsMatrix(out_file);
  out_file.close();
#endif
  std::vector<State*> states =
      (static_cast<PathGeometric*>(
           _sub8_planner->getProblemDefinition()->getSolutionPath().get()))
          ->getStates();
  return states;
}

sub8_msgs::Path TGenManager::generatePathMessage() {
  std::vector<State*> states = getPath();

  sub8_msgs::Path p_msg;
  for (State* s : states) {
    p_msg.path.push_back(stateToPathPoint(s));
  }

  return p_msg;
}