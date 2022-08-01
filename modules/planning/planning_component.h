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

#pragma once

#include <memory>

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "cyber/message/raw_message.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/perception/proto/traffic_light_detection.pb.h"

#include "modules/perception/proto/perception_obstacle.pb.h"

#include "modules/planning/common/message_process.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/planning_base.h"
#include "modules/planning/proto/learning_data.pb.h"
#include "modules/planning/proto/pad_msg.pb.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"
#include "modules/routing/proto/routing.pb.h"
#include "modules/storytelling/proto/story.pb.h"

namespace apollo {
namespace planning {

class PlanningComponent final
    : public cyber::Component<prediction::PredictionObstacles, canbus::Chassis,
                              localization::LocalizationEstimate> {
 public:
  PlanningComponent() = default;

  ~PlanningComponent() = default;

 public:

  static std::vector<std::pair<double, double>> trajectory;
  static bool flag_trajectory;
  static int last_sended_trajectory_end_index;
  static int last_sended_trajectory_start_index;
  static int index_previous_nearest_point;

  static void MessageCallback_obst(
    const std::shared_ptr<perception::PerceptionObstacles>& msg);
  static void MessageCallback(
    const std::shared_ptr<roi_boundary_message>& msg);
  static void UpdateADCMessageInfo(
                  ADCTrajectory& adc_trajectory_pb_polamp,
                  std::vector<point_info> current_polamp_traj,
                  double time_for_step, double shift_s, double shift_t,
                  double nearest_point_accumulated_s,
                  double nearest_point_t,
                  bool current_trajectory_gear,
                  roi_point originFramePointAbsoluteCoordinates,
                  std::vector<double> v, std::vector<double> a);
  static void getTrajectoryWithSameGear(
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
                      std::vector<bool> gears_of_points);
  static void SetGearsForTrajectoryPoints(std::vector<bool>* gears_of_points);
  static void GetNearPointToVehicleAndAccumulatedInfo(
                            std::vector<double>* point_accumulated_s,
                            std::vector<double>* ts, int* index_nearest_point,
                            double* nearest_point_t, double* nearest_point_accumulated_s,
                            double time_for_step, 
                            double normalized_vehicle_x, double normalized_vehicle_y);

  bool Init() override;

  bool Proc(const std::shared_ptr<prediction::PredictionObstacles>&
                prediction_obstacles,
            const std::shared_ptr<canbus::Chassis>& chassis,
            const std::shared_ptr<localization::LocalizationEstimate>&
                localization_estimate) override;

 private:
  void CheckRerouting();
  bool CheckInput();

 private:

  std::shared_ptr<cyber::Reader<perception::TrafficLightDetection>>
      traffic_light_reader_;
  std::shared_ptr<cyber::Reader<routing::RoutingResponse>> routing_reader_;
  std::shared_ptr<cyber::Reader<planning::PadMessage>> pad_msg_reader_;
  std::shared_ptr<cyber::Reader<relative_map::MapMsg>> relative_map_reader_;
  std::shared_ptr<cyber::Reader<storytelling::Stories>> story_telling_reader_;

  std::shared_ptr<cyber::Writer<ADCTrajectory>> planning_writer_;
  
  std::shared_ptr<cyber::Reader<roi_boundary_message>> trajectory_reader_;
  std::shared_ptr<cyber::Reader<perception::PerceptionObstacles>> obstacles_reader_;
  std::shared_ptr<cyber::Writer<roi_boundary_message>> roi_boundary_writer_;
  
  std::shared_ptr<cyber::Writer<routing::RoutingRequest>> rerouting_writer_;
  std::shared_ptr<cyber::Writer<PlanningLearningData>>
      planning_learning_data_writer_;

  std::mutex mutex_;
  perception::TrafficLightDetection traffic_light_;
  routing::RoutingResponse routing_;
  planning::PadMessage pad_msg_;
  relative_map::MapMsg relative_map_;
  storytelling::Stories stories_;

  LocalView local_view_;

  std::unique_ptr<PlanningBase> planning_base_;
  std::shared_ptr<DependencyInjector> injector_;

  PlanningConfig config_;
  MessageProcess message_process_;
};

CYBER_REGISTER_COMPONENT(PlanningComponent)

}  // namespace planning
}  // namespace apollo
