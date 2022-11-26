/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#pragma once

#include <ros/ros.h>

#include <unordered_map>
#include <tf2_eigen/tf2_eigen.h>
#include <autoware_msgs/DetectedObjectArray.h>

#include "planning_core/planning_common/obstacle.h"
#include "planning_core/planning_common/planning_visual.h"
#include "common/utils/common_visual.h"

namespace planning {

class MockPredictor {
 public:
  MockPredictor(const double prediction_horizon, const double dt)
      : prediction_horizon_(prediction_horizon), dt_(dt) {}
  virtual ~MockPredictor() = default;

  virtual void Init();

  virtual void UpdateVehicleLaneMap(const std::vector<Obstacle>& obstacles);

  virtual bool GeneratePrediction(std::vector<Obstacle>* obstacles) = 0;

 protected:
  void VisualizePrediction(const std::vector<Obstacle>& obstacles);
  autoware_msgs::DetectedObject constructAutowareObject(const Obstacle& obstacle);

 protected:
  ros::Publisher predict_pub_;
  ros::Publisher obstacles_pub_;

  double dt_ = 0.0;
  double prediction_horizon_ = 0.0;

  // (VehicleId, LandId)
  std::unordered_map<int, int> vehicle_lane_map_;
};

}  // namespace planning
