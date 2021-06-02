

#pragma once

#include <memory>

#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/tasks/deciders/decider.h"

namespace apollo {
namespace planning {

class BehaviourTreeDecider : public Decider {
 public:
  BehaviourTreeDecider(
      const TaskConfig& config,
      const std::shared_ptr<DependencyInjector>& injector);

 private:
  common::Status Process(Frame* frame,
                         ReferenceLineInfo* reference_line_info) override;
                         
  bool IsThereFrontObstacle();
  bool IsLongTermBlockingObstacle();
  bool IsDestination (Frame* const frame);
  bool DetectCollision(Frame* const frame,const ReferenceLineInfo* reference_line_info);
  
  common::TrajectoryPoint InferFrontAxeCenterFromRearAxeCenter(
      const common::TrajectoryPoint& traj_point);
      
  bool IsL1free();
  bool IsL2free();
  bool IsL3free();
  bool IsFfree();
  bool IsBfree();
  bool IsR1free();
  bool IsR2free();
  bool IsR3free();
  
  bool SpeedOccObs_10(const ReferenceLineInfo* reference_line_info);
  bool SpeedOccObs11_20(const ReferenceLineInfo* reference_line_info);
  bool SpeedOccObs21_30(const ReferenceLineInfo* reference_line_info);
  bool SpeedOccObs31_40(const ReferenceLineInfo* reference_line_info);
  bool SpeedOccObs41_50(const ReferenceLineInfo* reference_line_info); 
  bool SpeedOccObsMore50(const ReferenceLineInfo* reference_line_info); 
  
  bool EgoSpeed_10(Frame* const frame);
  bool EgoSpeed11_20(Frame* const frame);
  bool EgoSpeed21_30(Frame* const frame);
  bool EgoSpeed31_40(Frame* const frame);
  bool EgoSpeed41_50(Frame* const frame);
  bool EgoSpeedMore50(Frame* const frame);
  
  bool KeepLane(ReferenceLineInfo* const reference_line_info);
  bool SwitchToLeft(ReferenceLineInfo* const reference_line_info);
  bool SwitchToRight(ReferenceLineInfo* const reference_line_info);
  
  bool CallCond(char letter,Frame* const frame, ReferenceLineInfo* const reference_line_info);
  int ExcuteTree(std::string tree,Frame* const frame, ReferenceLineInfo* const reference_line_info);
  // places 2d array , 2d array for places and 1 dimension for : Occupied?,Occupant Obstacle Id , IsInversed?
  //   
  //   --------
  //  /occ?  /
  //  -------------------
  //  | L1  | L2  | L3  |
  //  -------------------
  //  |Back |     |Front|
  //  -------------------
  //  | R1  | R2  | R3  |
  //  -------------------
  private:
  bool  Places[3][3] = {}; // initialized to zeros // is place occupied
  std::string obs_IDs[3][3] = {{"x","x","x"},{"x","x","x"},{"x","x","x"}};
  static bool Keep_lane_;
  static bool Switch_to_left_;
  static bool Switch_to_right_;
  static char running_action ;
  static int running_action_counter ;
 // bool IsEgoSpeedCloseToLaneLimit(const Frame& frame);

};

}  // namespace planning
}  // namespace apollo
