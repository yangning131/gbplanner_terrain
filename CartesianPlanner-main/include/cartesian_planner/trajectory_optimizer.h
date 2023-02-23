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

#include "trajectory_nlp.h"

#include "math/aabox2d.h"
#include "math/polygon2d.h"

#include "discretized_trajectory.h"
#include "cartesian_planner_config.h"

namespace cartesian_planner {

using math::Polygon2d;
using math::AABox2d;
using math::Box2d;


class TrajectoryOptimizer {
public:
  TrajectoryOptimizer(const CartesianPlannerConfig &config, const Env &env, const Wor &world);

  bool OptimizeIteratively(const DiscretizedTrajectory &coarse, const Constraints &constraints, States &result);

private:
  CartesianPlannerConfig config_;
  Env env_;
  Wor world_;
  VehicleParam vehicle_;
  TrajectoryNLP nlp_;

  void CalculateInitialGuess(States &states) const;

  void CalculateheightAndalpha(States &states) const;

  bool FormulateCorridorConstraints(States &states, Constraints &constraints);

  bool GenerateBox(double time, double z, double &x, double &y, double radius, AABox2d &result) const;

  inline bool CheckCollision(double time, double z, double x, double y, const AABox2d &bound) const {
    Box2d box(bound);
    box.Shift({x, y});

    // return env_->CheckCollision(time, box);
    return world_->CheckStaticCollision(box, z);

  }

  double getposealpha(double x1, double y1, double z1, double x0, double y0 ,double z0) const
  {
    Eigen::Vector3d direction(x1 - x0, y1 - y0, z1 - z0);
    //terrain slop judge
    Eigen::Vector2d ground_direction(direction[0],direction[1]);
    double ground_lane = ground_direction.norm();
    double hight_change = abs(direction[2]);
    if(ground_lane<=0.01||hight_change<=0.01)
          return 0;
    return std::atan(hight_change/ground_lane);
  }

};

}