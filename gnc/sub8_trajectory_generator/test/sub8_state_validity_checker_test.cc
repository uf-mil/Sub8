/**
 * Author: Patrick Emami
 * Date: 9/21/15
 */
#include <gtest/gtest.h>
#include "sub8_state_space.h"
#include "sub8_state_validity_checker.h"
#include "space_information_generator.h"
#include "ompl/base/SpaceInformation.h"

using sub8::trajectory_generator::SpaceInformationGeneratorPtr;
using sub8::trajectory_generator::SpaceInformationGenerator;
using sub8::trajectory_generator::Sub8StateSpace;
using sub8::trajectory_generator::Sub8StateValidityChecker;
using ompl::base::SpaceInformationPtr;

TEST(Sub8StateValidityCheckerTest, testIsValid) {
  SpaceInformationGeneratorPtr ss_gen(new SpaceInformationGenerator());
  SpaceInformationPtr _si_test = ss_gen->generate();

  State* s1 = _si_test->getStateSpace()->as<Sub8StateSpace>()->allocState();

  s1->as<Sub8StateSpace::StateType>()->setX(-1000);
  s1->as<Sub8StateSpace::StateType>()->setY(0);
  s1->as<Sub8StateSpace::StateType>()->setZ(99);

  // test that this is an invalid state
  ASSERT_FALSE(_si_test->getStateValidityChecker()->isValid(s1));
}