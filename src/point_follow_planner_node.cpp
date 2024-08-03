// Copyright 2023 amsl

#include "point_follow_planner/point_follow_planner.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "point_follow_planner");
  PointFollowPlanner planner;
  planner.process();
  return 0;
}
