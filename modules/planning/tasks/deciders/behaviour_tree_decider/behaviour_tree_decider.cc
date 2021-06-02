

#include "modules/planning/tasks/deciders/behaviour_tree_decider/behaviour_tree_decider.h"

#include <algorithm>
#include <memory>
#include <string>
#include <fstream>
#include <map>

//#include "modules/planning/common/obstacle_blocking_analyzer.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"

typedef bool (*FnPtrCond)();

namespace apollo {
namespace planning {

using apollo::common::Status;
using apollo::common::VehicleConfigHelper;

bool BehaviourTreeDecider::Keep_lane_ = false;
bool BehaviourTreeDecider::Switch_to_left_ = false;
bool BehaviourTreeDecider::Switch_to_right_ = false;
char  BehaviourTreeDecider::running_action = 'X';
int  BehaviourTreeDecider::running_action_counter = 0 ;

double epsilon = 5; // km/h
int current_place_x = 1 ;
int current_place_y = 1 ;

BehaviourTreeDecider::BehaviourTreeDecider(
    const TaskConfig& config,
    const std::shared_ptr<DependencyInjector>& injector)
    : Decider(config, injector) {}

Status BehaviourTreeDecider::Process(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  // Sanity checks.
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  
  const ReferenceLine& reference_line = reference_line_info->reference_line();
  
  ADEBUG << "behaviour tree decider...";
  ///// get x y of the starting planning point
  /*
  common::TrajectoryPoint planning_start_point = frame->PlanningStartPoint();
    
  if (FLAGS_use_front_axe_center_in_path_planning) {
    planning_start_point =
        InferFrontAxeCenterFromRearAxeCenter(planning_start_point);
  }
  ADEBUG << "MAIS: Plan at the starting point: x = "
         << planning_start_point.path_point().x()
         << ", y = " << planning_start_point.path_point().y() << ", and angle = " << planning_start_point.path_point().theta();*/// delete
  ////
  
  
  double ego_speed = frame->PlanningStartPoint().v(); //   in m/s
  double x_SafeDistance = 4 * ego_speed;//3 * ego_speed;// 3 seconds rule
//  AINFO << "ego speed = " << ego_speed << "safe distance is "<< x_SafeDistance;
 // AINFO << IsThereFrontObstacle();
  
  // vehicle parameters found in /apollo/modules/common/data/vehicle_param.pb.txt
  // future work: automatically find them
  
  double front_edge_to_center = 3.41;
  double back_edge_to_center = 0.73;
 // double left_edge_to_center = 0.9;
 // double right_edge_to_center = 0.9;
  
  common::TrajectoryPoint planning_start_point = frame->PlanningStartPoint();
  auto adc_sl_info = reference_line.ToFrenetFrame(planning_start_point);
  double adc_frenet_s = adc_sl_info.first[0];
//  double adc_frenet_l = adc_sl_info.second[0];
//  AINFO << "adc s: " << adc_frenet_s <<" adc l : " << adc_frenet_l ;
//  double offset_to_map = 0.0;
//  reference_line.GetOffsetToMap(adc_frenet_s, &offset_to_map);
//  double adc_l_to_lane_center = adc_frenet_l + offset_to_map; // l to center of the goal lane, not the current one
  // ADC's lane width.
  double lane_left_width = 0.0;
  double lane_right_width = 0.0;
  double adc_lane_width = 0.0;
  if (!reference_line.GetLaneWidth(adc_frenet_s, &lane_left_width,
                                   &lane_right_width)) {
    AWARN << "Failed to get lane width at planning start point.";
    adc_lane_width = 3.4;
  } else {
    adc_lane_width = lane_left_width + lane_right_width;
  }
  
  //AINFO << "adc lane width: " << adc_lane_width << "adc frenet l: " << adc_frenet_l << " adc l to lane center " << adc_l_to_lane_center; 
  
      ////// Filling the Places matrix://////
  for (int i=0; i<3;++i)
    for (int j=0; j<3; ++j){
            obs_IDs[i][j] = "x";
            Places[i][j]=0;    
    }

      
  auto indexed_obstacles = reference_line_info->path_decision()->obstacles();
  for (const auto* obstacle : indexed_obstacles.Items()) {
    // Mais: don't forget to exclude DEST obstacle.
    //sl_boundary
    const auto obstacle_sl = obstacle->PerceptionSLBoundary();
 //   AINFO << "s: " << obstacle_sl.start_s() << " start l: " << obstacle_sl.start_l() << " end l: "<< obstacle_sl.end_l(); 
    
    if (reference_line.IsOnLane(obstacle->PerceptionSLBoundary())){
   // 	AINFO << obstacle->Id() << " is on lane , speed is " << obstacle->speed();
    	if( obstacle_sl.start_s() > (adc_frenet_s + front_edge_to_center)  &&  obstacle_sl.start_s() - (adc_frenet_s + front_edge_to_center) < x_SafeDistance  ){
    	  // front place is occupied
    	  Places[1][2] = 1;
    	  obs_IDs[1][2] = obstacle->Id(); 
    	}
    	else if((adc_frenet_s - back_edge_to_center) > obstacle_sl.end_s()  && std::abs(obstacle_sl.end_s() - (adc_frenet_s + back_edge_to_center)) < x_SafeDistance){
    	  // back place is occupied
    	  Places[1][0] = 1;
    	  obs_IDs[1][0] = obstacle->Id(); 
    	}
    }
    else{
      //  AINFO << obstacle->Id() << " is not on lane  , speed is " << obstacle->speed(); 
        if ( obstacle_sl.start_l() >   (-1.5*adc_lane_width) && obstacle_sl.end_l() < (-0.5 *adc_lane_width)){
    //      AINFO << "obstacle is on right lane";
          double obstacle_s = (obstacle_sl.start_s() + obstacle_sl.end_s())/2;
          if (obstacle_s < adc_frenet_s + 1.5*x_SafeDistance){
               if (obstacle_s > adc_frenet_s + 0.5 * x_SafeDistance ){
                 //R3 is occupied
                 Places[2][2] = 1;
                 obs_IDs[2][2] = obstacle->Id(); 
               } else if (obstacle_s > adc_frenet_s - 0.5 * x_SafeDistance ){
                 //R2 is occupied
                 Places[2][1] = 1;
                 obs_IDs[2][1] = obstacle->Id(); 
               } else{ 
                   if (obstacle_s > adc_frenet_s - 1.5 * x_SafeDistance){
                     //R1 is occupied
                     Places[2][0] = 1;
                     obs_IDs[2][0] = obstacle->Id(); 
                   }
                 }
          }
        } 
        else if ( obstacle_sl.start_l() >   (0.5*adc_lane_width)   &&  obstacle_sl.end_l() < (1.5 *adc_lane_width)){
        //  AINFO << "obstacle is on left lane";
          double obstacle_s = (obstacle_sl.start_s() + obstacle_sl.end_s())/2;
          if (obstacle_s < adc_frenet_s + 1.5*x_SafeDistance){
               if (obstacle_s > adc_frenet_s + 0.5 * x_SafeDistance ){
                 //L3 is occupied
                 Places[0][2] = 1;
                 obs_IDs[0][2] = obstacle->Id(); 
               } else if (obstacle_s > adc_frenet_s - 0.5 * x_SafeDistance ){
                 //L2 is occupied
                 Places[0][1] = 1;
                 obs_IDs[0][1] = obstacle->Id(); 
               } else{ 
                   if (obstacle_s > adc_frenet_s - 1.5 * x_SafeDistance){
                     //L1 is occupied
                     Places[0][0] = 1;
                     obs_IDs[0][0] = obstacle->Id(); 
                   }
                 }
          }
        } 
        else {
          ADEBUG << "obstacle is not on a neighnor lane";
        }
    }
 
  }
  //////////////////////////////////////////std::abs
  auto* mutable_path_decider_status = injector_->planning_context()
                                          ->mutable_planning_status()
                                         ->mutable_path_decider();
  std::string front_obstacle_id = mutable_path_decider_status->front_static_obstacle_id();
           
  const Obstacle* front_obstacle =
      reference_line_info->path_decision()->obstacles().Find(front_obstacle_id);
  
      
  AINFO <<"x_SafeDistance : "<< x_SafeDistance;
  AINFO << "Places:";
  AINFO << Places[0][0] << "  " << Places[0][1] << "  " << Places[0][2]  ;
  AINFO << obs_IDs[0][0] << "  " << obs_IDs[0][1] << "  " << obs_IDs[0][2]  ;
  AINFO << Places[1][0] << "     " << Places[1][2]  ;
  AINFO << obs_IDs[1][0] << "     " << obs_IDs[1][2]  ;
  AINFO << Places[2][0] << "  " << Places[2][1]  << "  " << Places[2][2]  ;
  AINFO << obs_IDs[2][0] << "  " << obs_IDs[2][1]  << "  " << obs_IDs[2][2]  ;
  /*
  AINFO << "is Back free?  " << IsBfree() ;
  AINFO << "if not what is the obstacle speed? is less than 10 " << SpeedOccObs_10(reference_line_info) <<" is between 10-20? " << SpeedOccObs10_20(reference_line_info) <<" is more? " << SpeedOccObs20_30(reference_line_info); 
  AINFO << "is front free?  " << IsFfree() ;
  AINFO << "if not what is the obstacle speed? is less than 10 " << SpeedOccObs_10(reference_line_info) <<" is between 10-20? " << SpeedOccObs10_20(reference_line_info) <<" is more? " << SpeedOccObs20_30(reference_line_info); 
  */
  if (DetectCollision(frame,reference_line_info)){
    //  AINFO << "Collision is detected ... " ;
      std::ofstream result ("/apollo/modules/planning/tasks/deciders/behaviour_tree_decider/results.txt",std::ios_base::app);
      if(result.is_open()){
        result << "Collision detected.\n";
        result.close();
      }
      else
        AINFO << "Unable to open file";
  }
  
  ///check if goal is reached 
  if (IsDestination(frame)){
     // AINFO << "goal is reached ... " ;
      std::ofstream result ("/apollo/modules/planning/tasks/deciders/behaviour_tree_decider/results.txt",std::ios_base::app);
      if(result.is_open()){
        result << "goal is reached.\n";
        result.close();
      }
      else
        AINFO << "Unable to open file";
  }
  
  common::VehicleState vehicle_state = frame->vehicle_state();
  bool IsEgoOnLane = reference_line.IsOnLane(common::math::Vec2d(vehicle_state.x(), vehicle_state.y()));
  AINFO << "Is ego on lane?   " << IsEgoOnLane;
  

  std::string Struct="";
  std::ifstream TreeStruct ("/apollo/modules/planning/tasks/deciders/behaviour_tree_decider/TreeStruct.txt");
  char ch;
  if (TreeStruct.is_open()){
    while (TreeStruct.get(ch)){
      if(ch != '\n')
        Struct += ch;
    }
   // AINFO << "Tree structure: " << Struct ;
    TreeStruct.close();
  }
  else
    AINFO << "Unable to open Tree structure.";

  /*
  // enable change lane :
 // bool is_prioritize_change_lane = false ;
  std::list<ReferenceLineInfo>* reference_line_info_list =
      frame->mutable_reference_line_info();

  auto iter = reference_line_info_list->begin();
  int i = 1;
  while (iter != reference_line_info_list->end()) {
    AINFO << i <<" iter->IsChangeLanePath(): " << iter->IsChangeLanePath();
    AINFO << i <<" iter->IsNeighborLanePath(): " << iter->IsNeighborLanePath();

    ++iter;
    ++i;
  }
//  reference_line_info_list->splice(reference_line_info_list->begin(),
  //                            *reference_line_info_list, iter);
  AINFO << "reference_line_info->IsChangeLanePath(): "
         << reference_line_info_list->begin()->IsChangeLanePath();
         
  const auto& current_path_id = reference_line_info->Lanes().Id();

  AINFO << "current path id: "<< current_path_id; */ //delete
  // end
  
  
  //needs to find out how to initialize ego_info
  //const auto *collision_obstacle = frame->FindCollisionObstacle(ego_info);
 // if (collision_obstacle != nullptr) {
  //  AINFO << "MAIS JA : Found collision with obstacle: " << collision_obstacle->Id();
  //}    
      
    // this should be only tested when the decision of switch lane is taken and we don't want to retake the decision
    // skip behaviour_tree_decider if reused path
  if (FLAGS_enable_skip_path_tasks && reference_line_info->path_reusable()) {
    // for debug
    ADEBUG << "skip due to reusing path";
    return Status::OK();
  }
  if (running_action=='Y' || running_action=='Z')
    running_action_counter +=1;
  if (IsThereFrontObstacle()&&IsLongTermBlockingObstacle() && std::abs(front_obstacle->speed() - ego_speed)< 0.3  ){
   // AINFO << "Excuting tree structure ..." << "tree size: " << Struct.size();
    int result = ExcuteTree(Struct,frame,reference_line_info);
    AINFO << "result is " << result;
    if((result == -1 && running_action_counter > 40 && IsEgoOnLane)||(!Switch_to_left_&&!Switch_to_right_&&!Keep_lane_)){    
     // AINFO << "Invalid tree .. ";
      std::ofstream result ("/apollo/modules/planning/tasks/deciders/behaviour_tree_decider/results.txt",std::ios_base::app);
      if(result.is_open()){
        result << "invalid tree.\n";
        result.close();
      }
    }
  }
  Switch_to_left_ = false;
  Switch_to_right_ = false;
  Keep_lane_ = false ;
  //SwitchToRight(reference_line_info);
  return Status::OK();
}

//bool BehaviourTreeDecider::IsEgoSpeedCloseToLaneLimit(const Frame& frame) {
 // return (  frame.PlanningStartPoint().v() - LaneSpeedLimits???  ) < epsilon;
//}

int BehaviourTreeDecider::ExcuteTree(std::string tree,Frame* const frame, ReferenceLineInfo* const reference_line_info){
  //AINFO << "passed string: "<<tree;
  if (tree[0]=='X'){
    KeepLane(reference_line_info);
    Keep_lane_ = true;
    return 1;
  }
  if (tree[0]=='Y'){
    SwitchToLeft(reference_line_info);
    Switch_to_left_ = true;
    if (running_action !='Y'){
        running_action = 'Y';
        running_action_counter = 1;
    }
    return -1;
  }
  if (tree[0]=='Z'){
    SwitchToRight(reference_line_info);
    Switch_to_right_ = true;
    if (running_action !='Z'){
        running_action = 'Z';
        running_action_counter = 1;
    }
    return -1;
  }
  if (tree[0]!='&' && tree[0]!='/')
    return CallCond(tree[0],frame,reference_line_info);
    
  int i =0;
  //AINFO <<  "checking: "<< tree[0] <<" "<< tree[tree.size()-1] <<" ";
  if (tree[i]=='/'){
    int result = 0;
    i+=2;
    while(tree[i]!=')'){
      if(tree[i]=='&' || tree[i]=='/'){
        int j=2;
        int counter = -1;
        while (counter<0){
          if (tree[i+j]==')')
            counter++;
          if (tree[i+j]=='(')
            counter--;
          ++j;
        }
        if (!result){
          int res = ExcuteTree(tree.substr(i,j),frame,reference_line_info);
          if (res == -1)
            return -1;
          else
            result = result || res;
          }
        i=i+j;
      } else{
        if (!result){
          int res = ExcuteTree(tree.substr(i,1),frame,reference_line_info);
          if (res == -1)
            return -1;
          else
             result = result || res;
          }
        i++;
      }
    }
    return result;
  } else if (tree[i]=='&'){
    int result = 1;
    i+=2;
    while(tree[i]!=')'){
      if(tree[i]=='&' || tree[i]=='/'){
        int j=2;
        int counter = -1;
        while (counter<0){
          if (tree[i+j]==')')
            counter++;
          if (tree[i+j]=='(')
            counter--;
          ++j;
        }
        if (result){
          int res = ExcuteTree(tree.substr(i,j),frame,reference_line_info);
          if (res == -1)
            return -1;
          else
            result = result && res;
          }
        i=i+j;
      } else{
        if (result){
          int res = ExcuteTree(tree.substr(i,1),frame,reference_line_info);
          if (res == -1)
            return -1;
          else
            result = result && res;
          }
        i++;
      }
    }
    return result;
  }
  return 0;
}

bool BehaviourTreeDecider::CallCond(char letter,Frame* const frame, ReferenceLineInfo* const reference_line_info){
  bool result = false;
  switch(letter)
	{
	case 'c':
		result = IsL1free();
		break;
	case 'd':
		result = !IsL1free();
		break;
	case 'e':
		result = IsL2free();
		break;
	case 'f':
		result = !IsL2free();
		break;
	case 'g':
		result = IsL3free();
		break;
	case 'h':
		result = !IsL3free();
		break;
	case 'i':
		result = IsR1free();
		break;
	case 'j':
		result = !IsR1free();
		break;
	case 'k':
		result = IsR2free();
		break;
	case 'l':
		result = !IsR2free();
		break;
	case 'm':
		result = IsR3free();
		break;
	case 'n':
		result = !IsR3free();
		break;
	case 'o':
		result = SpeedOccObs_10(reference_line_info);
		break;
	case 'p':
		result = SpeedOccObs11_20(reference_line_info);
		break;
	case 'q':
		result = SpeedOccObs21_30(reference_line_info);
		break;
	case 'r':
		result = SpeedOccObs31_40(reference_line_info);
		break;
	case 's':
		result = SpeedOccObs41_50(reference_line_info);
		break;
	case 't':
		result = SpeedOccObsMore50(reference_line_info);
		break;
	case 'u':
		result = EgoSpeed_10(frame);
		break;
	case 'v':
		result = EgoSpeed11_20(frame);
		break;
	case 'w':
		result = EgoSpeed21_30(frame);
		break;
	case 'x':
		result = EgoSpeed31_40(frame);
		break;
	case 'y':
		result = EgoSpeed41_50(frame);
		break;
	case 'z':
		result = EgoSpeedMore50(frame);
		break;
	default:
		AINFO << "Invalid letter. wrong tree structure." ; 
	}//switch
  return result;
}

bool BehaviourTreeDecider::IsThereFrontObstacle(){
  auto* mutable_path_decider_status = injector_->planning_context()
                                          ->mutable_planning_status()
                                         ->mutable_path_decider();
  ADEBUG << "Blocking obstacle ID["
           << mutable_path_decider_status->front_static_obstacle_id() << "]";
           
  if (mutable_path_decider_status->front_static_obstacle_id() == "" )
    return false;
  else
    return true;

}

bool BehaviourTreeDecider::IsLongTermBlockingObstacle() {
  if (injector_->planning_context()
          ->planning_status()
          .path_decider()
          .front_static_obstacle_cycle_counter() >=
      FLAGS_long_term_blocking_obstacle_cycle_threshold) {
    ADEBUG << "The blocking obstacle is long-term existing.";
    return true;
  } else {
    ADEBUG << "The blocking obstacle is not long-term existing.";
    return false;
  }
}


bool BehaviourTreeDecider::IsDestination (Frame* const frame){

  double destination_x = 444290 ;
 // double destination_y = 6185649 ;
  //double delta = 1.5;
  common::VehicleState vehicle_state = frame->vehicle_state();

  ADEBUG << "MAIS: vehicle x state: " <<vehicle_state.x()<<" , vehicle y state: "<< vehicle_state.y();
  
  if (  destination_x  < vehicle_state.x() ) {
    return true;
  }
  return false;
}



bool BehaviourTreeDecider::DetectCollision(Frame* const frame, const ReferenceLineInfo* reference_line_info){

//  double front_edge_to_center = 3.41;
 // double back_edge_to_center = 0.73;
 // double left_edge_to_center = 0.9;
 // double right_edge_to_center = 0.9; 
  double epsilon = 2.3; // meter
  common::VehicleState vehicle_state = frame->vehicle_state();
  ADEBUG << "ego x: " <<vehicle_state.x()<<" , ego y: "<< vehicle_state.y();
 // const ReferenceLine& reference_line = reference_line_info->reference_line();
//  common::TrajectoryPoint planning_start_point = frame->PlanningStartPoint();
 // auto adc_sl_info = reference_line.ToFrenetFrame(planning_start_point);
 // double adc_frenet_s = adc_sl_info.first[0];
 // double adc_frenet_l = adc_sl_info.second[0];
 // AINFO << "ego s: "<<adc_frenet_s <<"ego l: "<<adc_frenet_l;
  //Mais:
//  double adc_start_l = adc_frenet_l - right_edge_to_center;
//  double adc_end_l = adc_frenet_l + left_edge_to_center;
 // double adc_start_s = adc_frenet_s + front_edge_to_center;
 // double adc_end_s = adc_frenet_s - back_edge_to_center;
   
  auto indexed_obstacles = reference_line_info->path_decision().obstacles();
  for (const auto* obstacle : indexed_obstacles.Items()) {
   // const auto obstacle_sl = obstacle->PerceptionSLBoundary();
    auto perception_obstacle = obstacle->Perception();
    auto obstacle_position =  perception_obstacle.position();

    ADEBUG << "obs x: " << obstacle_position.x() <<" obs y: "<<obstacle_position.y() ;
    double delta_x = vehicle_state.x() - obstacle_position.x();
    double delta_y = vehicle_state.y() - obstacle_position.y();
    double distance = std::pow(delta_x, 2) + std::pow(delta_y, 2);
    distance = std::sqrt(distance);
    ADEBUG << "distance is:" << distance;
    if (distance<epsilon){
      AINFO << "Collision is detected. distance is "<<distance;
      return true;
    }
    
   // double obs_s = (obstacle_sl.end_s() - obstacle_sl.start_s())/2;
   // AINFO << "obs s " << obs_s << " adc start l "<< adc_start_l << " adc end l "<<adc_end_l;
  //  AINFO << " obs start l "<< obstacle_sl.start_l() << " obs end l "<<obstacle_sl.end_l();
  //  if ((adc_start_l > obstacle_sl.start_l()&& adc_start_l < obstacle_sl.end_l())||(adc_end_l > obstacle_sl.start_l()&& adc_end_l < obstacle_sl.end_l())){
  //    if (obs_s < 0.5 && obs_s > -0.5){
    //    AINFO << "Collision is detected.";
   //     return true;
   //   }

    //}
  }
  return false;
}


bool BehaviourTreeDecider::EgoSpeed_10(Frame* const frame){

  common::VehicleState vehicle_state = frame->vehicle_state();
  double EgoSpeed = vehicle_state.linear_velocity();
 // AINFO << "MAIS: Ego Speed: " <<EgoSpeed;
  
  return EgoSpeed <= 2.78;
}

bool BehaviourTreeDecider::EgoSpeed11_20(Frame* const frame){

  common::VehicleState vehicle_state = frame->vehicle_state();
  double EgoSpeed = vehicle_state.linear_velocity();
  //AINFO << "MAIS: Ego Speed: " <<EgoSpeed;
  
  return EgoSpeed >2.78 && EgoSpeed <=5.56;
}

bool BehaviourTreeDecider::EgoSpeed21_30(Frame* const frame){

  common::VehicleState vehicle_state = frame->vehicle_state();
  double EgoSpeed = vehicle_state.linear_velocity();
  //AINFO << "MAIS: Ego Speed: " <<EgoSpeed;
  
  return EgoSpeed >5.56 && EgoSpeed <=8.33;
}

bool BehaviourTreeDecider::EgoSpeed31_40(Frame* const frame){

  common::VehicleState vehicle_state = frame->vehicle_state();
  double EgoSpeed = vehicle_state.linear_velocity();
  //AINFO << "MAIS: Ego Speed: " <<EgoSpeed;
  
  return EgoSpeed >8.33 && EgoSpeed <=11.11;
}

bool BehaviourTreeDecider::EgoSpeed41_50(Frame* const frame){

  common::VehicleState vehicle_state = frame->vehicle_state();
  double EgoSpeed = vehicle_state.linear_velocity();
 // AINFO << "MAIS: Ego Speed: " <<EgoSpeed;
  
  return EgoSpeed >11.11 && EgoSpeed <=13.89;
}

bool BehaviourTreeDecider::EgoSpeedMore50(Frame* const frame){

  common::VehicleState vehicle_state = frame->vehicle_state();
  double EgoSpeed = vehicle_state.linear_velocity();
  //AINFO << "MAIS: Ego Speed: " <<EgoSpeed;
  
  return EgoSpeed > 13.89;
}

bool BehaviourTreeDecider::SpeedOccObs_10(const ReferenceLineInfo* reference_line_info){
  std::string occupant_obstacle_id = obs_IDs[current_place_x][current_place_y];
  if(occupant_obstacle_id == "x")
    return 0;
  const Obstacle* occupant_obstacle =
      reference_line_info->path_decision().obstacles().Find(occupant_obstacle_id);
  if (occupant_obstacle == nullptr) {
    AINFO << "Occupant obstacle is not found.";
    return 0;
  }

  double obstacle_speed =  occupant_obstacle->speed();
  return obstacle_speed <=2.78;
}
bool BehaviourTreeDecider::SpeedOccObs11_20(const ReferenceLineInfo* reference_line_info){
  std::string occupant_obstacle_id = obs_IDs[current_place_x][current_place_y];
  if(occupant_obstacle_id == "x")
    return 0;
  const Obstacle* occupant_obstacle =
      reference_line_info->path_decision().obstacles().Find(occupant_obstacle_id);
  if (occupant_obstacle == nullptr) {
    ADEBUG << "Occupant obstacle is not found.";
    return 0;
  }

  double obstacle_speed =  occupant_obstacle->speed();
  return obstacle_speed >2.78 && obstacle_speed <=5.56;
}

bool BehaviourTreeDecider::SpeedOccObs21_30(const ReferenceLineInfo* reference_line_info){
  std::string occupant_obstacle_id = obs_IDs[current_place_x][current_place_y];
  if(occupant_obstacle_id == "x")
    return 0;
  const Obstacle* occupant_obstacle =
      reference_line_info->path_decision().obstacles().Find(occupant_obstacle_id);
  if (occupant_obstacle == nullptr) {
    ADEBUG << "Occupant obstacle is not found.";
    return 0;
  }

  double obstacle_speed =  occupant_obstacle->speed();
  return obstacle_speed >5.56 && obstacle_speed <=8.33;
}

bool BehaviourTreeDecider::SpeedOccObs31_40(const ReferenceLineInfo* reference_line_info){
  std::string occupant_obstacle_id = obs_IDs[current_place_x][current_place_y];
  if(occupant_obstacle_id == "x")
    return 0;
  const Obstacle* occupant_obstacle =
      reference_line_info->path_decision().obstacles().Find(occupant_obstacle_id);
  if (occupant_obstacle == nullptr) {
    ADEBUG << "Occupant obstacle is not found.";
    return 0;
  }

  double obstacle_speed =  occupant_obstacle->speed();
  return obstacle_speed >8.33 && obstacle_speed <=11.11;
}

bool BehaviourTreeDecider::SpeedOccObs41_50(const ReferenceLineInfo* reference_line_info){
  std::string occupant_obstacle_id = obs_IDs[current_place_x][current_place_y];
  if(occupant_obstacle_id == "x")
    return 0;
  const Obstacle* occupant_obstacle =
      reference_line_info->path_decision().obstacles().Find(occupant_obstacle_id);
  if (occupant_obstacle == nullptr) {
    ADEBUG << "Occupant obstacle is not found.";
    return 0;
  }

  double obstacle_speed =  occupant_obstacle->speed();
  return obstacle_speed >11.11 && obstacle_speed <=13.89;
}

bool BehaviourTreeDecider::SpeedOccObsMore50(const ReferenceLineInfo* reference_line_info){
  std::string occupant_obstacle_id = obs_IDs[current_place_x][current_place_y];
  if(occupant_obstacle_id == "x")
    return 0;
  const Obstacle* occupant_obstacle =
      reference_line_info->path_decision().obstacles().Find(occupant_obstacle_id);
  if (occupant_obstacle == nullptr) {
    ADEBUG << "Occupant obstacle is not found.";
    return 0;
  }

  double obstacle_speed =  occupant_obstacle->speed();
  return obstacle_speed >13.89;
}


bool BehaviourTreeDecider::IsL1free(){
  current_place_x = 0 ;
  current_place_y = 0 ;
  return !(Places[0][0]);
}

bool BehaviourTreeDecider::IsL2free(){
  current_place_x = 0 ;
  current_place_y = 1 ;
  return !(Places[0][1]);
}

bool BehaviourTreeDecider::IsL3free(){
  current_place_x = 0 ;
  current_place_y = 2 ;
  return !(Places[0][2]);
}

bool BehaviourTreeDecider::IsFfree(){
  current_place_x = 1 ;
  current_place_y = 2 ;
  return !(Places[1][2]);
}

bool BehaviourTreeDecider::IsBfree(){
  current_place_x = 1 ;
  current_place_y = 0 ;
  return !(Places[1][0]);
}

bool BehaviourTreeDecider::IsR1free(){
  current_place_x = 2 ;
  current_place_y = 0 ;
  return !(Places[2][0]);
}

bool BehaviourTreeDecider::IsR2free(){
  current_place_x = 2 ;
  current_place_y = 1 ;
  return !(Places[2][1]);
}

bool BehaviourTreeDecider::IsR3free(){
  current_place_x = 2 ;
  current_place_y = 2 ;
  return !(Places[2][2]);
}

// three actions in the behaviour tree : keep lane , switch to left lane , switch to right lane:
//keep lane 
bool BehaviourTreeDecider::KeepLane(ReferenceLineInfo* const reference_line_info){
  reference_line_info->set_Keep_lane(true);
  reference_line_info->set_Switch_to_left(false);
  reference_line_info->set_Switch_to_right(false);
  ADEBUG <<"action keep line has been applied.";
  return true;
}
// switch to left lane 
bool BehaviourTreeDecider::SwitchToLeft(ReferenceLineInfo* const reference_line_info){
  reference_line_info->set_Keep_lane(false);
  reference_line_info->set_Switch_to_left(true);
  reference_line_info->set_Switch_to_right(false);
  ADEBUG <<"action switch to right has been applied.";
  return true;
}
// switch to right lane
bool BehaviourTreeDecider::SwitchToRight(ReferenceLineInfo* const reference_line_info){
  reference_line_info->set_Keep_lane(false);
  reference_line_info->set_Switch_to_left(false);
  reference_line_info->set_Switch_to_right(true);
  ADEBUG <<"action switch to left has been applied.";
  return true;
}


common::TrajectoryPoint BehaviourTreeDecider::InferFrontAxeCenterFromRearAxeCenter(
    const common::TrajectoryPoint& traj_point) {
  double front_to_rear_axe_distance =
      VehicleConfigHelper::GetConfig().vehicle_param().wheel_base();
  common::TrajectoryPoint ret = traj_point;
  ret.mutable_path_point()->set_x(
      traj_point.path_point().x() +
      front_to_rear_axe_distance * std::cos(traj_point.path_point().theta()));
  ret.mutable_path_point()->set_y(
      traj_point.path_point().y() +
      front_to_rear_axe_distance * std::sin(traj_point.path_point().theta()));
  return ret;
}


}  // namespace planning
}  // namespace apollo

/* call tree functions by strings:



bool cond1() { ... return ; }
bool cond2() { ... return ; }
//function pointer
typedef bool (*FnPtr)();

int main() {
    // initialization:
    std::map<std::string, FnPtr> myMap;
    myMap["a"] = cond1;
    myMap["b"] = cond2;

    // usage:
    std::string s("b");
    int res = myMap[s]();
    
}

*/

