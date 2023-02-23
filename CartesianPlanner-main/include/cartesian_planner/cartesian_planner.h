/***********************************************************************************
 *  C++ Source Codes for "Autonomous Driving on Curvy Roads without Reliance on
 *  Frenet Frame: A Cartesian-based Trajectory Planning Method".
 ***********************************************************************************
 *  Copyright (C) 2022 Bai Li
 *  Users are suggested to cite the following article when they use the source codes.
 *  Bai Li et al., "Autonomous Driving on Curvy Roads without Reliance on
 *  Frenet Frame: A Cartesian-based Trajectory Planning Method",
 *  IEEE Transactions on Intelligent Transportation Systems, 2022.
 ***********************************************************************************/

#pragma once

#include "cartesian_planner_config.h"

#include "dp_planner.h"
#include "trajectory_optimizer.h"



namespace cartesian_planner {

class CartesianPlanner {
public:
  struct StartState {
    double x, y, z, theta, v, phi, a, omega;
  };

  explicit CartesianPlanner(const CartesianPlannerConfig &config, const Env &env, const Wor &world)
    : config_(config), opti_(config, env, world), env_(env){}

  bool Plan(const StartState &state, DiscretizedTrajectory &result);

  float pointDistance(TrajectoryPoint p1, TrajectoryPoint p2)
  {
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y)+ (p1.z-p2.z)*(p1.z-p2.z));
  }

  double getposealpha(geometry_msgs::PoseStamped nextpose,geometry_msgs::PoseStamped pose)
  {
    Eigen::Vector3d direction(nextpose.pose.position.x - pose.pose.position.x, nextpose.pose.position.y - pose.pose.position.y,
                            nextpose.pose.position.z - pose.pose.position.z);
    //terrain slop judge

    Eigen::Vector2d ground_direction(direction[0],direction[1]);

    double ground_lane = ground_direction.norm();

    double hight_change = abs(direction[2]);
    if(ground_lane<=0.01||hight_change<=0.01)
          return 0;
          
    return std::atan(hight_change/ground_lane);
  }


private:
  CartesianPlannerConfig config_;
  TrajectoryOptimizer opti_;
  Env env_;
  Trajectory last_result_data_;

};


}