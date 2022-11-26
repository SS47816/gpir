/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#include "planning_core/mock_predictor/mock_predictor.h"

#include <glog/logging.h>
#include <visualization_msgs/MarkerArray.h>

#include "common/utils/color_map.h"
#include "common/utils/math.h"
#include "hdmap/hdmap.h"
#include "hdmap/road_network/lane_map.h"

namespace planning {

constexpr double kAngleThreshold = M_PI / 10.0;

void MockPredictor::Init() {
  ros::NodeHandle node;
  predict_pub_ = node.advertise<visualization_msgs::MarkerArray>("/prediction", 1);
  obstacles_pub_ = node.advertise<autoware_msgs::DetectedObjectArray>("planning/ground_truth/objects", 1);
}

void MockPredictor::UpdateVehicleLaneMap(
    const std::vector<Obstacle>& obstacles) {
  for (const auto& obstacle : obstacles) {
    int id = obstacle.id();
    const auto& state = obstacle.state();
    if (vehicle_lane_map_.find(id) == vehicle_lane_map_.end() ||
        vehicle_lane_map_[id] == 0) {
      // * NeartestLane() is quite expensive, avoid using it as much as possible
      vehicle_lane_map_[id] =
          hdmap::HdMap::NearestLane(state.position, state.heading, false);
    } else {
      int lane_id = vehicle_lane_map_[id];
      auto lane = hdmap::LaneMap::GetLane(lane_id);
      std::pair<double, double> lane_boundary;
      auto sl_pair =
          lane->GetProjectionAndBoundary(state.position, &lane_boundary);
      auto waypoint = lane->GetWayPoint(sl_pair.first);
      if (sl_pair.second > lane_boundary.first ||
          sl_pair.second < lane_boundary.second ||
          std::fabs(common::NormalizeAngle(state.heading - waypoint.heading) >
                    kAngleThreshold)) {
        vehicle_lane_map_[id] =
            hdmap::HdMap::NearestLane(state.position, state.heading, false);
        continue;
      }

      if (sl_pair.first > lane->length()) {
        if (lane->HasSuccessor())
          vehicle_lane_map_[id] = lane->next_lanes().front();
        else
          vehicle_lane_map_[id] = 0;
      }
    }
  }
}

void MockPredictor::VisualizePrediction(const std::vector<Obstacle>& obstacles) {
  visualization_msgs::MarkerArray markers;
  visualization_msgs::Marker line_marker, node_marker;
  autoware_msgs::DetectedObjectArray objects_msg;

  line_marker.header.frame_id = "map";
  line_marker.header.stamp = ros::Time::now();
  line_marker.type = visualization_msgs::Marker::LINE_STRIP;
  line_marker.action = visualization_msgs::Marker::MODIFY;
  line_marker.pose.orientation.w = 1;
  line_marker.color = common::ColorMap::at(common::Color::kOrange, 0.5).toRosMsg();
  line_marker.scale.x = line_marker.scale.y = line_marker.scale.z = 0.1;
  line_marker.lifetime = ros::Duration(0.1);

  node_marker.header = line_marker.header;
  node_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  node_marker.action = visualization_msgs::Marker::MODIFY;
  node_marker.pose.orientation.w = 1;
  node_marker.color = common::ColorMap::at(common::Color::kMagenta).toRosMsg();
  node_marker.scale.x = node_marker.scale.y = node_marker.scale.z = 0.3;
  node_marker.lifetime = ros::Duration(0.1);

  geometry_msgs::Point point;
  for (const auto& obstacle : obstacles) {
    line_marker.id = obstacle.id();
    node_marker.id = 1000 * line_marker.id + 1;

    size_t size = obstacle.prediction().size();
    line_marker.points.clear();
    node_marker.points.clear();
    line_marker.points.reserve(size);
    node_marker.points.reserve(size);

    for (const auto state : obstacle.prediction()) {
      point.x = state.position.x();
      point.y = state.position.y();
      point.z = 0.1;
      line_marker.points.emplace_back(point);
      node_marker.points.emplace_back(point);
    }
    markers.markers.emplace_back(line_marker);
    markers.markers.emplace_back(node_marker);

    objects_msg.objects.emplace_back(constructAutowareObject(obstacle));
  }

  predict_pub_.publish(markers);

  objects_msg.header = line_marker.header;
  obstacles_pub_.publish(objects_msg);
}

autoware_msgs::DetectedObject MockPredictor::constructAutowareObject(const Obstacle& obstacle)
{
  autoware_msgs::DetectedObject autoware_object;
  autoware_object.header.frame_id = "map";
  autoware_object.header.stamp = ros::Time::now();
  autoware_object.id = obstacle.id();

  const auto bbox = obstacle.BoundingBox();
  common::CommonVisual::StateToPose(bbox.center(), bbox.angle(), &autoware_object.pose, bbox.height()/2);
  autoware_object.dimensions.x = bbox.length();
  autoware_object.dimensions.y = bbox.width();
  autoware_object.dimensions.z = bbox.height();
  autoware_object.pose_reliable = true;
  autoware_object.velocity.linear.x = obstacle.speed();
  autoware_object.velocity_reliable = true;
  autoware_object.valid = true;
  autoware_object.label = "car";

  // Convex hull
  autoware_object.convex_hull.header = autoware_object.header;

  Eigen::Isometry3d T_wo;
  tf2::fromMsg(autoware_object.pose, T_wo);

  const auto dims = autoware_object.dimensions;
  Eigen::Vector3d fl_pt{dims.x/2, dims.y/2, -dims.z/2};
  Eigen::Vector3d fr_pt{dims.x/2, -dims.y/2, -dims.z/2};
  Eigen::Vector3d rr_pt{-dims.x/2, -dims.y/2, -dims.z/2};
  Eigen::Vector3d rl_pt{-dims.x/2, dims.y/2, -dims.z/2};
  std::vector<Eigen::Vector3d> rect{std::move(fl_pt), std::move(fr_pt), std::move(rr_pt), std::move(rl_pt)};
  for (size_t i = 0; i < 4; i++)
  {
    Eigen::Vector3d pt = T_wo * rect[i];
    geometry_msgs::Point32 point;
    point.x = pt.x();
    point.y = pt.y();
    point.z = pt.z();
    autoware_object.convex_hull.polygon.points.emplace_back(point);
  }
  autoware_object.convex_hull.polygon.points.push_back(autoware_object.convex_hull.polygon.points.front());

  autoware_msgs::Lane trajectory;
  for (const auto& state : obstacle.prediction()) {
    autoware_msgs::Waypoint waypoint;
    waypoint.pose.pose.position.x = state.position.x();
    waypoint.pose.pose.position.y = state.position.y();
    
    trajectory.waypoints.emplace_back(waypoint);
    autoware_object.candidate_trajectories.lanes.emplace_back(trajectory);
  }

  return autoware_object;
}

}  // namespace planning