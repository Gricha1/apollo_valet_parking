/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "modules/planning/planning_component.h"

#include "cyber/common/file.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/common/util/message_util.h"
#include "modules/common/util/util.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/map/pnc_map/pnc_map.h"
#include "modules/planning/common/history.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/navi_planning.h"
#include "modules/planning/on_lane_planning.h"

#include <cmath>
#include <fstream>

namespace apollo {
namespace planning {

using apollo::cyber::ComponentBase;
using apollo::hdmap::HDMapUtil;
using apollo::perception::TrafficLightDetection;
using apollo::relative_map::MapMsg;
using apollo::routing::RoutingRequest;
using apollo::routing::RoutingResponse;
using apollo::storytelling::Stories;
using apollo::cyber::Time;


bool PlanningComponent::flag_trajectory = false;
bool example_updated = false;
int PlanningComponent::index_previous_nearest_point = -1;
int PlanningComponent::last_sended_trajectory_end_index = -1;
int PlanningComponent::last_sended_trajectory_start_index = -1;
std::vector<std::pair<double, double>> PlanningComponent::trajectory = {};
std::vector<point_info> polamp_trajectory_info = {};
std::vector<std::pair<double, double>> vector_obsts_position = {};
std::vector<std::pair<double, double>> vector_obsts_params = {};
std::vector<double> vector_obsts_thetas = {};
std::vector<std::pair<double, double>> vector_obsts_velocity = {};
ADCTrajectory* example_of_adc_trajectory;
//new changes
int count_of_repeat = 0;

bool PlanningComponent::Init() {
  injector_ = std::make_shared<DependencyInjector>();

  if (FLAGS_use_navigation_mode) {
    planning_base_ = std::make_unique<NaviPlanning>(injector_);
    AWARN << "NAVI";
  } else {
    planning_base_ = std::make_unique<OnLanePlanning>(injector_);
    AWARN << "ONLANE";
  }

  ACHECK(ComponentBase::GetProtoConfig(&config_))
      << "failed to load planning config file "
      << ComponentBase::ConfigFilePath();

  if (FLAGS_planning_offline_learning ||
      config_.learning_mode() != PlanningConfig::NO_LEARNING) {
    if (!message_process_.Init(config_, injector_)) {
      AERROR << "failed to init MessageProcess";
      return false;
    }
  }

  planning_base_->Init(config_);

  routing_reader_ = node_->CreateReader<RoutingResponse>(
      config_.topic_config().routing_response_topic(),
      [this](const std::shared_ptr<RoutingResponse>& routing) {
        AINFO << "Received routing data: run routing callback."
              << routing->header().DebugString();
        std::lock_guard<std::mutex> lock(mutex_);
        routing_.CopyFrom(*routing);
      });

  traffic_light_reader_ = node_->CreateReader<TrafficLightDetection>(
      config_.topic_config().traffic_light_detection_topic(),
      [this](const std::shared_ptr<TrafficLightDetection>& traffic_light) {
        ADEBUG << "Received traffic light data: run traffic light callback.";
        std::lock_guard<std::mutex> lock(mutex_);
        traffic_light_.CopyFrom(*traffic_light);
      });

  pad_msg_reader_ = node_->CreateReader<PadMessage>(
      config_.topic_config().planning_pad_topic(),
      [this](const std::shared_ptr<PadMessage>& pad_msg) {
        ADEBUG << "Received pad data: run pad callback.";
        std::lock_guard<std::mutex> lock(mutex_);
        pad_msg_.CopyFrom(*pad_msg);
      });

  story_telling_reader_ = node_->CreateReader<Stories>(
      config_.topic_config().story_telling_topic(),
      [this](const std::shared_ptr<Stories>& stories) {
        ADEBUG << "Received story_telling data: run story_telling callback.";
        std::lock_guard<std::mutex> lock(mutex_);
        stories_.CopyFrom(*stories);
      });


  //custom changes
  trajectory_reader_ = node_->CreateReader<roi_boundary_message>(
      "from_python_to_apollo", MessageCallback);

  obstacles_reader_ = node_->CreateReader<perception::PerceptionObstacles>(
      "/apollo/perception/obstacles", MessageCallback_obst);


  if (FLAGS_use_navigation_mode) {
    relative_map_reader_ = node_->CreateReader<MapMsg>(
        config_.topic_config().relative_map_topic(),
        [this](const std::shared_ptr<MapMsg>& map_message) {
          ADEBUG << "Received relative map data: run relative map callback.";
          std::lock_guard<std::mutex> lock(mutex_);
          relative_map_.CopyFrom(*map_message);
        });
  }
  planning_writer_ = node_->CreateWriter<ADCTrajectory>(
      config_.topic_config().planning_trajectory_topic());

  /*
    custom changes: take boundaries from roi decider
  */
  roi_boundary_writer_ = node_->CreateWriter<roi_boundary_message>(
      "get_roi_boundaries_topic");
  //-------------------------------------------------

  rerouting_writer_ = node_->CreateWriter<RoutingRequest>(
      config_.topic_config().routing_request_topic());

  planning_learning_data_writer_ = node_->CreateWriter<PlanningLearningData>(
      config_.topic_config().planning_learning_data_topic());

  return true;
}

void PlanningComponent::MessageCallback_obst(
    const std::shared_ptr<perception::PerceptionObstacles>& msg) {
  AWARN << "message obst working" << std::endl;
  vector_obsts_position = {};
  vector_obsts_params = {};
  vector_obsts_thetas = {};
  vector_obsts_velocity = {};
  //custom changes: add dynamic, static obst info
  for (auto& it : msg->perception_obstacle()) {
    vector_obsts_position.push_back(std::make_pair<double, double> 
                (it.position().x(), it.position().y()));
    vector_obsts_params.push_back(std::make_pair<double, double> 
                (it.length(), it.width()));
    vector_obsts_thetas.push_back(it.theta());
    vector_obsts_velocity.push_back(std::make_pair<double, double> 
                (it.velocity().x(), it.velocity().y()));
  }
  
  //DEBUG
  AWARN << "obsts:" << vector_obsts_position.size() 
        << std::endl;
  for (auto& it : vector_obsts_position) {
    AWARN << " x: " << it.first
          << " y: " << it.second 
          << std::endl;
  }
  for (auto& it : vector_obsts_params) {
    AWARN << " length: " << it.first
          << " width: " << it.second 
          << std::endl;
  }
  for (auto& it : vector_obsts_thetas) {
    AWARN << "theta: " << it 
          << std::endl;
  }
  for (auto& it : vector_obsts_velocity) {
    AWARN << " x_vel: " << it.first
          << " y_vel: " << it.second 
          << std::endl;
  }
}


void PlanningComponent::MessageCallback(
    const std::shared_ptr<roi_boundary_message>& msg) {

  AWARN << "new using message start: " << flag_trajectory;
  AWARN << "count of points: " << msg->point_size() << std::endl;
  for (int i = 0; i < msg->point_size(); i++) {
    trajectory.push_back(std::make_pair<double,double> 
                (msg->point(i).x(), msg->point(i).y()));
    //update info of polamp trajectory
    point_info temp_point;
    temp_point.x = msg->point(i).x();
    temp_point.y = msg->point(i).y();
    temp_point.phi = msg->point(i).phi();
    temp_point.v = msg->point(i).v();
    temp_point.a = msg->point(i).a();
    temp_point.steer = msg->point(i).steer();
    polamp_trajectory_info.push_back(temp_point);

    /*
    //delete !!!
    delete_polamp.push_back(temp_point);
    */
    
  }

  
  flag_trajectory = true;
  AWARN << "new using message end: " << flag_trajectory;
}

bool PlanningComponent::Proc(
    const std::shared_ptr<prediction::PredictionObstacles>&
        prediction_obstacles,
    const std::shared_ptr<canbus::Chassis>& chassis,
    const std::shared_ptr<localization::LocalizationEstimate>&
        localization_estimate) {
  ACHECK(prediction_obstacles != nullptr);

  // check and process possible rerouting request
  CheckRerouting();

  // process fused input data
  local_view_.prediction_obstacles = prediction_obstacles;
  local_view_.chassis = chassis;
  local_view_.localization_estimate = localization_estimate;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!local_view_.routing ||
        hdmap::PncMap::IsNewRouting(*local_view_.routing, routing_)) {
      local_view_.routing =
          std::make_shared<routing::RoutingResponse>(routing_);
    }
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    local_view_.traffic_light =
        std::make_shared<TrafficLightDetection>(traffic_light_);
    local_view_.relative_map = std::make_shared<MapMsg>(relative_map_);
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    local_view_.pad_msg = std::make_shared<PadMessage>(pad_msg_);
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    local_view_.stories = std::make_shared<Stories>(stories_);
  }

  if (!CheckInput()) {
    AERROR << "Input check failed";
    return false;
  }

  if (config_.learning_mode() != PlanningConfig::NO_LEARNING) {
    // data process for online training
    message_process_.OnChassis(*local_view_.chassis);
    message_process_.OnPrediction(*local_view_.prediction_obstacles);
    message_process_.OnRoutingResponse(*local_view_.routing);
    message_process_.OnStoryTelling(*local_view_.stories);
    message_process_.OnTrafficLightDetection(*local_view_.traffic_light);
    message_process_.OnLocalization(*local_view_.localization_estimate);
  }

  // publish learning data frame for RL test
  if (config_.learning_mode() == PlanningConfig::RL_TEST) {
    PlanningLearningData planning_learning_data;
    LearningDataFrame* learning_data_frame =
        injector_->learning_based_data()->GetLatestLearningDataFrame();
    if (learning_data_frame) {
      planning_learning_data.mutable_learning_data_frame()
                            ->CopyFrom(*learning_data_frame);
      common::util::FillHeader(node_->Name(), &planning_learning_data);
      planning_learning_data_writer_->Write(planning_learning_data);
    } else {
      AERROR << "fail to generate learning data frame";
      return false;
    }
    return true;
  }

  ADCTrajectory adc_trajectory_pb;

  auto roi_boundaries_pb = std::make_shared<roi_boundary_message>();
  unsigned size_obsts = vector_obsts_position.size();
  for (unsigned i = 0; i < size_obsts; i++) {
    auto roi_point = roi_boundaries_pb->add_point();
    roi_point->set_x(vector_obsts_position[i].first);
    roi_point->set_y(vector_obsts_position[i].second);
    roi_point->set_length(vector_obsts_params[i].first);
    roi_point->set_width(vector_obsts_params[i].second);
    roi_point->set_theta(vector_obsts_thetas[i]);
    roi_point->set_v_x(vector_obsts_velocity[i].first);
    roi_point->set_v_y(vector_obsts_velocity[i].second);
    roi_boundaries_pb->set_timestamp(Time::Now().ToNanosecond());
  }
  AWARN << " debug obst info: " << roi_boundaries_pb->point_size()
        << std::endl;
  for (int i = 0; i < roi_boundaries_pb->point_size(); i++) { 
    AWARN << " x: " << roi_boundaries_pb->point(i).x()
          << " y: " << roi_boundaries_pb->point(i).y()
          << std::endl;
  }

  AWARN << "planning base Run Once" << std::endl;
  planning_base_->RunOnce(local_view_, &adc_trajectory_pb, 
                          &roi_boundaries_pb,
                          &roi_boundary_writer_,
                          flag_trajectory,
                          &trajectory,
                          &polamp_trajectory_info);
  AWARN << "planning base Run Once done" << std::endl; 

  /*
  if (roi_boundaries_pb->point_size() != 0) {
    roi_point originFramePointAbsoluteCoordinates = roi_boundaries_pb
                            ->point(roi_boundaries_pb
                            ->point_size() - 1);
    unsigned size_obsts = vector_obsts_position.size();
    for (unsigned i = 0; i < size_obsts; i++) {
      auto roi_point = roi_boundaries_pb->add_point();
      roi_point->set_x(vector_obsts_position[i].first - originFramePointAbsoluteCoordinates.x());
      roi_point->set_y(vector_obsts_position[i].second - originFramePointAbsoluteCoordinates.y());
      roi_point->set_length(vector_obsts_params[i].first);
      roi_point->set_width(vector_obsts_params[i].second);
      roi_point->set_theta(vector_obsts_thetas[i]);
      roi_point->set_v_x(vector_obsts_velocity[i].first);
      roi_point->set_v_y(vector_obsts_velocity[i].second);
      roi_boundaries_pb->set_timestamp(Time::Now().ToNanosecond());
    }

    AWARN << "debug planning comp AFTER roi point" << std::endl;
    roi_boundary_writer_->Write(roi_boundaries_pb);
    AWARN << "debug planning comp AFTER message" << std::endl;


    //DEBUG
    AWARN << " debug obst info: " << roi_boundaries_pb->point_size()
          << std::endl;
    for (int i = 0; i < roi_boundaries_pb->point_size(); i++) { 
      AWARN << " x: " << roi_boundaries_pb->point(i).x()
            << " y: " << roi_boundaries_pb->point(i).y()
            << std::endl;
    }
  }
  //roi_boundaries_pb->clear_point();
  //roi_boundaries_pb->clear_timestamp();
  */


  common::util::FillHeader(node_->Name(), &adc_trajectory_pb);
  // modify trajectory relative time due to the timestamp change in header
  auto start_time = adc_trajectory_pb.header().timestamp_sec();
  const double dt = start_time - adc_trajectory_pb.header().timestamp_sec();
  for (auto& p : *adc_trajectory_pb.mutable_trajectory_point()) {
    p.set_relative_time(p.relative_time() + dt);
  }

  if (flag_trajectory && !example_updated) {
    AWARN << "update example" << std::endl;
    example_of_adc_trajectory = &adc_trajectory_pb;
    example_updated = true;
  }

  bool debug_a_star = false;
  /*
  if (!debug_a_star) {
    if (roi_boundaries_pb->point_size() > int(size_obsts)) {
      adc_trajectory_pb.clear_trajectory_point();
      adc_trajectory_pb.clear_debug();
    }
  }
  */
  
  AWARN << "flag traj: " << flag_trajectory
        << " updated example: " << example_updated << std::endl;

  if (!debug_a_star) {
    if (flag_trajectory) {
      //adc_trajectory_pb.clear_debug();
      (*example_of_adc_trajectory).clear_debug();

      // get vehicle normalized state, and origin point(ablosute coordinates)
      AWARN << std::endl
            << "DEBUG origin point and vehicle state" << std::endl;
      const auto& vehicle_state = injector_->vehicle_state()->vehicle_state();
      double vehicle_x = vehicle_state.x();
      double vehicle_y = vehicle_state.y();
      roi_point originFramePointAbsoluteCoordinates = roi_boundaries_pb
                            ->point(roi_boundaries_pb
                            ->point_size() - 1);
      double normalized_vehicle_x = vehicle_x 
                    - originFramePointAbsoluteCoordinates.x();
      double normalized_vehicle_y = vehicle_y 
                    - originFramePointAbsoluteCoordinates.y();
      AWARN << std::endl
            << "init vehicle normalized state:" << std::endl
            << "x: " << normalized_vehicle_x << " "
            << "y: " << normalized_vehicle_y
            << std::endl;

      // find point on Rl trajectory nearest(distance) to vehicle position
      // and accumulated s, time 
      double time_for_step = 0.1;
      std::vector<double> point_accumulated_s;
      std::vector<double> ts;
      int index_nearest_point = 0;
      double nearest_point_t = 0;
      double nearest_point_accumulated_s = 0;
      GetNearPointToVehicleAndAccumulatedInfo(&point_accumulated_s,
                                              &ts, 
                                              &index_nearest_point,
                                              &nearest_point_t, 
                                              &nearest_point_accumulated_s,
                                              time_for_step, 
                                              normalized_vehicle_x, normalized_vehicle_y);
      AWARN << "nearest point index: " << index_nearest_point << std::endl
            << "nearest point time: " << nearest_point_t << std::endl
            << "nearest point accumulates s: " << nearest_point_accumulated_s
            << std::endl;
      index_previous_nearest_point = index_nearest_point;
    
      //set gear for each point in ADCtrajectory
      std::vector<bool> gears_of_points;
      SetGearsForTrajectoryPoints(&gears_of_points);
      AWARN << std::endl
        << "output points with gear size: " << gears_of_points.size()
        << std::endl;
      for (unsigned i = 0; i < gears_of_points.size(); i++) {
        AWARN << std::endl
        << "ind point: " << i 
        << " gear: " << gears_of_points[i]
        << std::endl;
      }
      
      // get trajectory with same gear
      double shift_s;
      double shift_t;
      std::vector<double> v;
      std::vector<double> a;
      std::vector<point_info> current_polamp_traj;
      bool current_trajectory_gear = true;
      getTrajectoryWithSameGear(&current_polamp_traj,
                                &current_trajectory_gear,
                                &v, &a, 
                                &shift_s, &shift_t,
                                index_nearest_point,
                                time_for_step,
                                point_accumulated_s, ts,
                                gears_of_points);
      AWARN << std::endl
            << "current trajectory size: " << current_polamp_traj.size()
            << std::endl
            << "last sended trajectory index: " 
            << last_sended_trajectory_end_index
            << std::endl;
      /*
      //std::vector<double> x;
      //std::vector<double> y;
      //std::vector<double> phi;
      std::vector<double> v;
      std::vector<double> a;
      for (auto &it : polamp_trajectory_info) {
        x.push_back(it.x);
        y.push_back(it.y);
        phi.push_back(it.phi);
        v.push_back(it.v);
        a.push_back(it.a);
      }
      int point_count = 0;
      double shift_s;
      double shift_t;
      int current_ind = 0;
      bool current_gear;
      bool previous_gear = true;
      current_trajectory_gear = true;
      last_sended_trajectory_start_index = 0;
      for (auto &it : polamp_trajectory_info) {
        current_gear = gears_of_points[current_ind];
        if (current_gear != previous_gear) {
          a[current_ind - 1] = - v[current_ind - 1] / time_for_step;
          a[current_ind] += v[current_ind] / time_for_step;
          v[current_ind] = 0;
          if (current_ind > index_nearest_point) {
            current_polamp_traj.push_back(it);
            last_sended_trajectory_end_index = current_ind;
            break;
          }
          shift_s = point_accumulated_s[current_ind];
          shift_t = ts[current_ind];
          point_count += current_ind;
          current_polamp_traj = {};
          current_trajectory_gear = !current_trajectory_gear;
          last_sended_trajectory_start_index = current_ind;
        }
        current_polamp_traj.push_back(it);
        previous_gear = current_gear;
        current_ind++;
      }
      if (int(polamp_trajectory_info.size()) == current_ind) {
        last_sended_trajectory_end_index = current_ind - 1;
        a[current_ind - 2] = - v[current_ind - 2] / time_for_step;
        a[current_ind - 1] = 0;
        v[current_ind - 1] = 0;
      }
      int debug_index = 0;
      int debug_current_ind = point_count;
      for (auto &it : polamp_trajectory_info) {
        AWARN << "second debug path point: " << debug_index
        << " second debug x: " << it.x
        << " y: " << it.y
        //<< " track: " << debug_tracking_angle[debug_index]
        << " phi: " << phi[debug_index]
        << " norm angle: " 
        //<< std::abs(common::math::NormalizeAngle(
        //            debug_tracking_angle[debug_index] 
        //            - phi[debug_index]))
        << " gear: " << gears_of_points[debug_index]
        << std::endl;
        debug_index++;
      }
      */

      // update ADC message info
      ADCTrajectory& adc_trajectory_pb_polamp = *example_of_adc_trajectory;
      adc_trajectory_pb_polamp.clear_trajectory_point();
      auto gear = canbus::Chassis::GEAR_DRIVE;
      int kappa_coef = 1;
      if (!current_trajectory_gear) {
        gear = canbus::Chassis::GEAR_REVERSE;
        kappa_coef = -1;
      }
      adc_trajectory_pb_polamp.set_gear(gear);
      double dx_current_to_previous = 0;
      double dy_current_to_previous = 0;
      double previous_x = current_polamp_traj.begin()->x;
      double previous_y = current_polamp_traj.begin()->y;
      double current_point_time = -time_for_step;
      double current_point_accumulated_s = 0;
      double dist_current_to_previous = 0;
      int current_index = last_sended_trajectory_start_index;
      for (auto &it : current_polamp_traj) {
        current_point_time += time_for_step;
        dx_current_to_previous = previous_x - it.x;
        dy_current_to_previous = previous_y - it.y;
        previous_x = it.x;
        previous_y = it.y;
        dist_current_to_previous = sqrt(dx_current_to_previous * dx_current_to_previous 
                    + dy_current_to_previous * dy_current_to_previous);
        current_point_accumulated_s += dist_current_to_previous;
        current_index++;
        AWARN << "path point: " << current_index - 1
        << " x: " << it.x + originFramePointAbsoluteCoordinates.x()
        << " + " << it.x + originFramePointAbsoluteCoordinates.x() 
                - int(it.x + originFramePointAbsoluteCoordinates.x())
        << " y: " << it.y + originFramePointAbsoluteCoordinates.y()
        << " + " << it.y + originFramePointAbsoluteCoordinates.y() 
                - int(it.y + originFramePointAbsoluteCoordinates.y())
        << " accumulated_s: " << current_point_accumulated_s 
                - nearest_point_accumulated_s + shift_s
        << " t: " << current_point_time - nearest_point_t + shift_t
        << " gear: " << gears_of_points[current_index - 1]
        << std::endl 
        << " old a: " << it.a << std::endl
        << " old v: " << it.v << std::endl;
        it.v = v[current_index - 1];
        it.a = a[current_index - 1];
        auto next_traj_point = adc_trajectory_pb_polamp.add_trajectory_point();
        auto* path_point = next_traj_point->mutable_path_point();
        path_point->set_x(it.x + originFramePointAbsoluteCoordinates.x());
        path_point->set_y(it.y + originFramePointAbsoluteCoordinates.y());
        path_point->set_theta(it.phi);
        path_point->set_s(kappa_coef * (current_point_accumulated_s 
                        - nearest_point_accumulated_s + shift_s));
        path_point->set_kappa(kappa_coef * std::tan(it.steer) / 2.8448);
        if (current_index != int(current_polamp_traj.size())) {
          if (v[current_index] == it.v){
            next_traj_point->set_a(0);
            //DEBUG
            AWARN << " a: " << 0;
          }
          else {
            next_traj_point->set_a(it.a);
            //DEBUG
            AWARN << " a: " << it.a;
          }
        }
        else {
          next_traj_point->set_a(it.a);
          //DEBUG
          AWARN << " a: " << it.a;
        }
        next_traj_point->set_v(it.v);
        //DEBUG
        AWARN << " v: " << it.v
              << std::endl;

        next_traj_point->set_steer(it.steer);
        next_traj_point->set_relative_time(current_point_time 
                                  - nearest_point_t + shift_t);
      }

      //DEBUG
      AWARN << "nearest index point: "
            << index_nearest_point
            << std::endl;
      AWARN << "nearest index point before current traj: "
            << index_previous_nearest_point
            << std::endl;
      //DEBUG
      AWARN << "vehicle state: "
            << " x: " << normalized_vehicle_x + originFramePointAbsoluteCoordinates.x() 
            << " + " 
            << normalized_vehicle_x + originFramePointAbsoluteCoordinates.x() - int(normalized_vehicle_x + + originFramePointAbsoluteCoordinates.x())
            << " y: " << normalized_vehicle_y + + originFramePointAbsoluteCoordinates.y() 
            << " + " 
            << normalized_vehicle_y + originFramePointAbsoluteCoordinates.y() - int(normalized_vehicle_y + originFramePointAbsoluteCoordinates.y()) 
            << std::endl;
      common::util::FillHeader(node_->Name(), &adc_trajectory_pb_polamp);
      //DEBUG
      AWARN << "current polamp traj size: " 
          << adc_trajectory_pb_polamp.trajectory_point_size()
          << std::endl
          << "path points size: "
          << adc_trajectory_pb_polamp.path_point_size()
          << " gear: " << current_trajectory_gear
          << std::endl;
      AWARN << "current traj start ind: " << last_sended_trajectory_start_index
            << std::endl;
      AWARN << "current traj end ind: " << last_sended_trajectory_end_index
            << std::endl;
      //for (const auto& p : adc_trajectory_pb_polamp.trajectory_point()) {
      //    AWARN << "point ind: " << debug_current_ind << " "
      //    << p.DebugString();
      //    debug_current_ind++;
      //}

      planning_writer_->Write(adc_trajectory_pb_polamp);

      // record in history
      auto* history = injector_->history();
      history->Add(adc_trajectory_pb_polamp);

      return true;
    }
    else {
      planning_writer_->Write(adc_trajectory_pb);

      // record in history
      auto* history = injector_->history();
      history->Add(adc_trajectory_pb);

      return true;
    }
  }
  //DEBUG
  AWARN << "it never happens" 
        << std::endl;
  
  /*
  ADCTrajectory adc_trajectory_test;
  
  common::util::FillHeader(node_->Name(), &adc_trajectory_test);

  for (auto &p : adc_trajectory_pb.mutable_trajectory_point()) {
        auto next_traj_point = adc_trajectory_test.add_trajectory_point();
        auto* path_point = next_traj_point->mutable_path_point();
        path_point->set_x(p.mutable_path_point().x());
        path_point->set_y(p.mutable_path_point().y());
        //path_point->set_theta(it.phi);
        //path_point->set_s(current_point_accumulated_s - nearest_point_accumulated_s);
        //path_point->set_kappa(it.v * std::tan(it.steer) / 2.8448);
        //next_traj_point->set_a(it.a);
        //next_traj_point->set_v(it.v);
        //next_traj_point->set_steer(it.steer);
        next_traj_point->set_relative_time(p.relative_time());
        //next_traj_point->set_steer(it.steer);
        //apollo::common::PathPoint path_point; 
        //path_point.set_kappa(0);
        //traj_point->set_allocated_path_point(&path_point);
  }

  planning_writer_->Write(adc_trajectory_test);

  AWARN << "debug A* trajectory: " 
        << adc_trajectory_test.trajectory_point_size()
        << std::endl;
  for (const auto& p : adc_trajectory_test.trajectory_point()) {
      //CUSTOM
      AWARN << p.DebugString();
  }

  // record in history
  auto* history = injector_->history();
  history->Add(adc_trajectory_test);
  */
  //adc_trajectory_pb.clear_trajectory_point();
  
  planning_writer_->Write(adc_trajectory_pb);
  //custom changes
  const auto& vehicle_state = injector_->vehicle_state()->vehicle_state();
  double normalized_vehicle_x = vehicle_state.x();
  double normalized_vehicle_y = vehicle_state.y();
  AWARN << "vehicle state: "
        << " x: " << normalized_vehicle_x
        << " y: " << normalized_vehicle_y
        << std::endl;
  AWARN << "debug A* trajectory: " 
        << adc_trajectory_pb.trajectory_point_size()
        << std::endl;
  AWARN << "traj type: " 
        << adc_trajectory_pb.trajectory_type()
        << std::endl;
  AWARN << adc_trajectory_pb.gear();
  for (const auto& p : adc_trajectory_pb.trajectory_point()) {
      //CUSTOM
      AWARN << p.DebugString();
  }

  
  int debug_point_num = 0;

  //DEBUG
  std::ofstream result ("/apollo/modules/planning/ADC_trajectory.txt", 
                            std::ios_base::app);
  if(result.is_open()) {
    for (auto& p : *adc_trajectory_pb.mutable_trajectory_point()) {
        debug_point_num++;
        result << "point: " << debug_point_num - 1
              << " x: " << p.path_point().x() << "  " 
              << " y: " << p.path_point().y() << "  " 
              << " theta: " << p.path_point().theta() << "  " 
              << " kappa: " << p.path_point().kappa() << "  "
              << " s: " << p.path_point().s() << "  " 
              << " v: " << p.v() << "  " 
              << " a: " << p.a() << "  " 
              << " t: " << p.relative_time() << "\n";
    }
    result << "\n******************************************\n";
    result.close();
  }
  else 
    AINFO << "Unable to open file";

  // record in history
  auto* history = injector_->history();
  history->Add(adc_trajectory_pb);

  return true;
}

void PlanningComponent::getTrajectoryWithSameGear(
                      std::vector<point_info>* current_polamp_traj,
                      bool* current_trajectory_gear,
                      std::vector<double>* v,
                      std::vector<double>* a,
                      double* shift_s,
                      double* shift_t,
                      int index_nearest_point,
                      double time_for_step,
                      std::vector<double> point_accumulated_s,
                      std::vector<double> ts,
                      std::vector<bool> gears_of_points) {
  AWARN << std::endl
        << "DEBUG getTrajectoryWithSameGear"
        << std::endl
        << "current polamp traj size: " << current_polamp_traj->size()
        << std::endl
        << "nearby point index: " << index_nearest_point
        << std::endl;
  for (auto &it : polamp_trajectory_info) {
    //x.push_back(it.x);
    //y.push_back(it.y);
    //phi.push_back(it.phi);
    v->push_back(it.v);
    a->push_back(it.a);
  }
  int point_count = 0;
  int current_ind = 0;
  bool current_gear;
  bool previous_gear = true;
  (*current_trajectory_gear) = true;
  last_sended_trajectory_start_index = 0;
  for (auto &it : polamp_trajectory_info) {
    current_gear = gears_of_points[current_ind];
    if (current_gear != previous_gear) {
      (*a)[current_ind - 1] = - (*v)[current_ind - 1] / time_for_step;
      (*a)[current_ind] += (*v)[current_ind] / time_for_step;
      (*v)[current_ind] = 0;
      // copy first point of the next trajectory
      if (current_ind > index_nearest_point) {
        current_polamp_traj->push_back(it);
        last_sended_trajectory_end_index = current_ind;
        break;
      }
      (*shift_s) = point_accumulated_s[current_ind];
      (*shift_t) = ts[current_ind];
      point_count += current_ind;
      (*current_polamp_traj) = {};
      (*current_trajectory_gear) = !(*current_trajectory_gear);
      last_sended_trajectory_start_index = current_ind;
    }
    current_polamp_traj->push_back(it);
    previous_gear = current_gear;
    current_ind++;
  }
  if (int(polamp_trajectory_info.size()) == current_ind) {
    last_sended_trajectory_end_index = current_ind - 1;
    (*a)[current_ind - 2] = - (*v)[current_ind - 2] / time_for_step;
    (*a)[current_ind - 1] = 0;
    (*v)[current_ind - 1] = 0;
  }
  //int debug_index = 0;
  //int debug_current_ind = point_count;
  //for (auto &it : polamp_trajectory_info) {
  //  AWARN << "second debug path point: " << debug_index
  //  << " second debug x: " << it.x
  //  << " y: " << it.y
    //<< " track: " << debug_tracking_angle[debug_index]
    //<< " phi: " << phi[debug_index]
    //<< " norm angle: " 
    //<< std::abs(common::math::NormalizeAngle(
    //            debug_tracking_angle[debug_index] 
    //            - phi[debug_index]))
  //  << " gear: " << gears_of_points[debug_index]
  //  << std::endl;
  //  debug_index++;
  //}
}

void PlanningComponent::SetGearsForTrajectoryPoints(std::vector<bool>* gears_of_points){
  AWARN << std::endl
        << "DEBUG SetGearsForTrajectoryPoints"
        << "input points with gear size: " << gears_of_points->size() 
        << std::endl;
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> phi;
  std::vector<double> v;
  std::vector<double> a;
  for (auto &it : polamp_trajectory_info) {
    x.push_back(it.x);
    y.push_back(it.y);
    phi.push_back(it.phi);
    v.push_back(it.v);
    a.push_back(it.a);
  }
  size_t horizon = x.size();
  double heading_angle = phi.front();
  double tracking_angle;
  if (x[1] - x[0] != 0) {
      if (x[1] < x[0]) {
        tracking_angle = y[1] - y[0] > 0 ? M_PI 
              + std::atan((y[1] - y[0]) / (x[1] - x[0]))
              : M_PI 
              - std::atan((y[1] - y[0]) / (x[1] - x[0]));
      }
      else {
        tracking_angle = std::atan((y[1] - y[0]) / (x[1] - x[0]));
      }
  }
  else {
    tracking_angle = y[1] - y[0] > 0 ? M_PI_2 : -M_PI_2;
  }
  for (size_t i = 0; i < horizon - 1; ++i) {
    heading_angle = phi[i];
    if (x[i + 1] - x[i] != 0) {
      if (x[i + 1] < x[i]) {
        tracking_angle = y[i + 1] - y[i] > 0 ? M_PI 
              + std::atan((y[i + 1] - y[i]) / (x[i + 1] - x[i]))
              : -M_PI 
              + std::atan((y[i + 1] - y[i]) / (x[i + 1] - x[i]));
      }
      else {
        tracking_angle = std::atan((y[i + 1] - y[i]) / (x[i + 1] - x[i]));
      }
    }
    else {
      tracking_angle = y[i + 1] - y[i] > 0 ? M_PI_2 : -M_PI_2;
    }
    //debug_tracking_angle.push_back(tracking_angle);
    bool gear =
      std::abs(common::math::NormalizeAngle(tracking_angle 
                - heading_angle)) < (M_PI_2);
    gears_of_points->push_back(gear);
    AWARN << std::endl
        << "tracking angle: " << tracking_angle
        << " heading angle: " << heading_angle
        << " gear: " << gear
        << std::endl;
  }
  gears_of_points->push_back((*gears_of_points)[gears_of_points->size() - 1]);
}

void PlanningComponent::GetNearPointToVehicleAndAccumulatedInfo(
                            std::vector<double>* point_accumulated_s,
                            std::vector<double>* ts, int* index_nearest_point,
                            double* nearest_point_t, double* nearest_point_accumulated_s,
                            double time_for_step, 
                            double normalized_vehicle_x, double normalized_vehicle_y) {
  AWARN << std::endl
        << "DEBUG func - get nearest point:" << std::endl
        << "time for step: " << time_for_step << std::endl
        << "normalized_vehicle_x: " << normalized_vehicle_x << std::endl
        << "normalized_vehicle_y: " << normalized_vehicle_y << std::endl
        << "before get nearest_point: " << std::endl
        << "last traj start index: " 
        << last_sended_trajectory_start_index << std::endl
        << "last traj end index: " 
        << last_sended_trajectory_end_index << std::endl;
  int current_index = 0;
  double dx_nearest_to_vehicle = polamp_trajectory_info.begin()->x 
                              - normalized_vehicle_x;
  double dy_nearest_to_vehicle = polamp_trajectory_info.begin()->y 
                              - normalized_vehicle_y;
  double nearest_dist = sqrt(dx_nearest_to_vehicle * dx_nearest_to_vehicle 
              + dy_nearest_to_vehicle * dy_nearest_to_vehicle);
  double previous_x = polamp_trajectory_info.begin()->x;
  double previous_y = polamp_trajectory_info.begin()->y;
  double dx_current_to_previous;
  double dy_current_to_previous;
  double dist_current_to_previous;
  double dx_current_to_vehicle; 
  double dy_current_to_vehicle;
  double current_dist_to_vehicle;
  double current_point_time = -time_for_step;
  double current_point_accumulated_s = 0;
  for (auto &it : polamp_trajectory_info) {
    current_point_time += time_for_step;
    dx_current_to_previous = previous_x - it.x;
    dy_current_to_previous = previous_y - it.y;
    previous_x = it.x;
    previous_y = it.y;
    dist_current_to_previous = sqrt(dx_current_to_previous * dx_current_to_previous 
                  + dy_current_to_previous * dy_current_to_previous);
    current_point_accumulated_s += dist_current_to_previous;
    dx_current_to_vehicle = it.x - normalized_vehicle_x;
    dy_current_to_vehicle = it.y - normalized_vehicle_y;
    current_dist_to_vehicle = sqrt(dx_current_to_vehicle 
                                  * dx_current_to_vehicle 
                                  + dy_current_to_vehicle 
                                  * dy_current_to_vehicle);
    if (current_index >= last_sended_trajectory_start_index
        && (last_sended_trajectory_end_index == -1 
        || current_index <= last_sended_trajectory_end_index)
        && current_index >= index_previous_nearest_point) {
      if (current_dist_to_vehicle < nearest_dist) {
        nearest_dist = current_dist_to_vehicle;
        (*nearest_point_t) = current_point_time;
        (*nearest_point_accumulated_s) = current_point_accumulated_s;
        (*index_nearest_point) = current_index;
        }
      else if (current_dist_to_vehicle == nearest_dist
          && (*index_nearest_point) == index_previous_nearest_point) {
        nearest_dist = current_dist_to_vehicle;
        (*nearest_point_t) = current_point_time;
        (*nearest_point_accumulated_s) = current_point_accumulated_s;
        (*index_nearest_point) = current_index;
      }
    }
    point_accumulated_s->push_back(current_point_accumulated_s);
    ts->push_back(current_point_time);
    current_index++;
  }
  if ((*index_nearest_point) == index_previous_nearest_point) {
    count_of_repeat++;
    //AWARN << "debug plan comp: " 
    //      << "count in " << (*index_nearest_point)
    //      << " " << count_of_repeat << std::endl;
  }
  else {
    count_of_repeat = 0;
  }
  if (count_of_repeat >= 2 
      && int(polamp_trajectory_info.size()) 
          != (*index_nearest_point) + 1) {
    (*index_nearest_point) = (*index_nearest_point) + 1;
    (*nearest_point_t) = (*ts)[(*index_nearest_point)];
    (*nearest_point_accumulated_s) = (*point_accumulated_s)[(*index_nearest_point)];
    count_of_repeat = 0;
  }

  for (unsigned i = 0; i < point_accumulated_s->size(); i++) {
    AWARN << std::endl
          << "accumulated s: " << (*point_accumulated_s)[i]
          << " accumulated time: " << (*ts)[i]
          << std::endl;
  }
  return;
}

void PlanningComponent::CheckRerouting() {
  auto* rerouting = injector_->planning_context()
                        ->mutable_planning_status()
                        ->mutable_rerouting();
  if (!rerouting->need_rerouting()) {
    return;
  }
  common::util::FillHeader(node_->Name(), rerouting->mutable_routing_request());
  rerouting->set_need_rerouting(false);
  rerouting_writer_->Write(rerouting->routing_request());
}

bool PlanningComponent::CheckInput() {
  ADCTrajectory trajectory_pb;
  auto* not_ready = trajectory_pb.mutable_decision()
                        ->mutable_main_decision()
                        ->mutable_not_ready();

  if (local_view_.localization_estimate == nullptr) {
    not_ready->set_reason("localization not ready");
  } else if (local_view_.chassis == nullptr) {
    not_ready->set_reason("chassis not ready");
  } else if (HDMapUtil::BaseMapPtr() == nullptr) {
    not_ready->set_reason("map not ready");
  } else {
    // nothing
  }

  if (FLAGS_use_navigation_mode) {
    if (!local_view_.relative_map->has_header()) {
      not_ready->set_reason("relative map not ready");
    }
  } else {
    if (!local_view_.routing->has_header()) {
      not_ready->set_reason("routing not ready");
    }
  }

  if (not_ready->has_reason()) {
    AERROR << not_ready->reason() << "; skip the planning cycle.";
    common::util::FillHeader(node_->Name(), &trajectory_pb);
    planning_writer_->Write(trajectory_pb);
    return false;
  }
  return true;
}

}  // namespace planning
}  // namespace apollo
