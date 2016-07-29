/**
* Author: Patrick Emami
* Date: 3/16/15
*/
#include <gtest/gtest.h>
#include "tgen_geometry_obstacles.h"

using sub8::trajectory_generator::TGenGeometryObstacles;
using sub8::trajectory_generator::TGenGeometryObstaclesPtr;

TEST(TGenGeometryObstaclesTest, testIsNearObstacle) {
  TGenGeometryObstaclesPtr geom(new TGenGeometryObstacles());

  // add a cube centered at 0, 0, 0 in 3-space with side length 1
  geom->addCube(0, 0, 0, 1);

  // The test vehicle is at (0.6, 0, 0), which is 0.1 outside of the cube in the
  // x-dimension
  // Since the tolerance is tested with 0.5, isNearObstacle should return true
  // A tolerance of 0.5 means that the side length of the cube grows to 1.5, which
  // thus covers +/- 0.75 along each axis
  ASSERT_TRUE(geom->isNearObstacle(0.6, 0, 0, 0.5));
}

TEST(TGenGeometryObstaclesTest, testIsNotNearObstacle) {
  TGenGeometryObstaclesPtr geom(new TGenGeometryObstacles());

  // add a cube centered at 0, 0, 0 in 3-space with side length 1
  geom->addCube(0, 0, 0, 1);

  // The test vehicle is at (0.6, 0, 0), which is 0.1 outside of the cube in the
  // x-dimension
  // Since the tolerance is tested with 0.5, isNearObstacle should return true
  // A tolerance of 0.5 means that the side length of the cube grows to 1.5, which
  // thus covers +/- 0.75 along each axis
  ASSERT_FALSE(geom->isNearObstacle(3, 2, 1, 0.5));
}