//
// Created by thomas on 20.12.17.
//

#include <hl_planner/Planner.h>

#include <hl_planner/HierarchyMap.h>
#include <hl_planner/Action.h>
#include <hl_planner/Plan.h>
#include <hl_planner/State.h>

Planner::Planner()
  : execute_action_client_(nh_, "execute_action", false), hierarchy_service_client_(nh_.serviceClient<semantic_mapping_v2::HierarchySrv>("hierarchy_srv")),
    last_state_(actionlib::SimpleClientGoalState(actionlib::SimpleClientGoalState::PENDING))
{
//  while(!execute_action_client_.waitForServer(ros::Duration(1.0))){
//    ROS_WARN("EXECUTE ACTION SERVER NOT UP");
//    ros::spinOnce();
//  }
//  while(!hierarchy_service_client_.waitForExistence(ros::Duration(1.0))){
//    ROS_WARN("HIERARCHY SERVICE NOT EXISTING");
//    ros::spinOnce();
//  }

  curr_goal_.action = -1;
}


void Planner::activeCb(){

}


void Planner::feedbackCb(const execution::ExecuteFeedbackConstPtr& feedback){

}


void Planner::doneCb(const actionlib::SimpleClientGoalState& state, const execution::ExecuteResultConstPtr& result){
  curr_goal_.action = -1;
  last_state_ = state;
}


void Planner::sendGoal(const geometry_msgs::Pose& pose, int action, int target){
  curr_goal_.pose = pose;
  curr_goal_.action = action;
  curr_goal_.target = target;
  execute_action_client_.sendGoal(curr_goal_, boost::bind(&Planner::doneCb, this, _1, _2),
                                  boost::bind(&Planner::activeCb, this), boost::bind(&Planner::feedbackCb, this, _1));
  ROS_INFO("SEND GOAL %d %d %.3lf %.3lf %.3lf %.3lf", curr_goal_.action, curr_goal_.target,
           curr_goal_.pose.position.x, curr_goal_.pose.position.y, curr_goal_.pose.orientation.z, curr_goal_.pose.orientation.w);
}


semantic_mapping_v2::HierarchySrvResponse Planner::getHierarchy(int max_tries){
  for(int i=0; i<max_tries; i++){
    semantic_mapping_v2::HierarchySrvRequest req;
    semantic_mapping_v2::HierarchySrvResponse res;
    if(hierarchy_service_client_.call(req, res)){
      return res;
    }
    ROS_WARN("%d. TRY SEMANTIC MAP CALL FAILED", i);
  }
  ROS_ERROR("SEMANTIC MAP CALL FAILED, ABORTING");
  return semantic_mapping_v2::HierarchySrvResponse();
}


SearchPlan Planner::greedyPlan(const HierarchyMap &graph, const State &state){
  SearchPlan plan(state);
  State curr_state = state;
  std::deque<SearchAction> search_actions_ = curr_state.getPossibleFullSearchActions();
  while(!search_actions_.empty()){
    float max_search_speed = 0.f;
    SearchAction best_action;
    for(const auto& a : search_actions_){
      float search_speed = graph.getSearchSpeed(curr_state.current_room_, a.target_);
      if(search_speed > max_search_speed){
        max_search_speed = search_speed;
        best_action = a;
      }
    }
    std::cout << max_search_speed << std::endl;
    curr_state = plan.addAction(best_action, graph, curr_state.current_room_, curr_state.quick_searched_);
    search_actions_ = curr_state.getPossibleFullSearchActions();
  }

  std::cout << plan.getPlanString() << std::endl;
  return plan;
}


SearchPlan plan(const HierarchyMap &graph, const SearchPlan& old_plan, float& cutoff_time, int level){
  std::deque<SearchAction> search_actions_ = old_plan.end_state_.getPossibleSearchActions();
  if(search_actions_.empty()){
    return old_plan;
  }

  SearchPlan best_plan(old_plan.end_state_, std::vector<SearchAction>(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), 0.f);
  for(const auto& a : search_actions_){
    SearchPlan new_plan = old_plan;
    new_plan.addAction(a, graph, old_plan.end_state_.current_room_, old_plan.end_state_.quick_searched_);
    if(new_plan.expected_search_time_ > cutoff_time)
      continue;

    new_plan = plan(graph, new_plan, cutoff_time, level+1);
    if(new_plan.expected_search_time_ < best_plan.expected_search_time_){
      best_plan = new_plan;
      if(best_plan.expected_search_time_ < cutoff_time)
        cutoff_time = best_plan.expected_search_time_;
    }
  }
  return best_plan;
}


Plan Planner::generatePlan(const HierarchyMap &graph, const State& state){
  SearchPlan greedy_plan = greedyPlan(graph, state);
  SearchPlan best_plan = plan(graph, SearchPlan(state), greedy_plan.expected_search_time_, 0);
  if(best_plan.expected_search_time_ > greedy_plan.expected_search_time_)
    best_plan = greedy_plan;

  std::cout << "Best plan: " << best_plan.getPlanString() << std::endl;



  return Plan();
}


void Planner::run(int obj){
  ros::Rate rate(PLANNER_RATE);
  while(ros::ok()){
    semantic_mapping_v2::HierarchySrvResponse hierarchy = getHierarchy(HIERARCHY_MAX_TRIES);
    if(hierarchy.rooms.empty())
      return;

    HierarchyMap graph_map(hierarchy, obj);

  }
}


//bool Planner::exploreRoom(semantic_mapping_v2::HierarchyLinkMsg link){
//  ros::Rate rate(10.0);
//  ros::Rate wait_rate(0.5);
//  int this_room = num_visited_rooms_;
//
//  geometry_msgs::Pose pose = link.door1_pose;
//  double angle = 2*std::acos(pose.orientation.w);
//  pose.position.x += 0.2*std::cos(angle);
//  pose.position.y += 0.2*std::sin(angle);
//  sendGoal(pose, 0, num_visited_rooms_);
//  while(curr_goal_.action != -1){
//    ros::spinOnce();
//    rate.sleep();
//  }
//  if(last_state_.state_ != actionlib::SimpleClientGoalState::SUCCEEDED){
//    ROS_ERROR("MOVE TO ROOM FAILED");
//    return false;
//  }
//  num_visited_rooms_++;
//  wait_rate.sleep();
//
//  sendGoal(geometry_msgs::Pose(), 1, 0);
//  while(curr_goal_.action != -1){
//    ros::spinOnce();
//  }
//  if(last_state_.state_ != actionlib::SimpleClientGoalState::SUCCEEDED){
//    ROS_ERROR("EXPLORE FAILED");
//    return false;
//  }
//
//  semantic_mapping_v2::HierarchySrvRequest req;
//  semantic_mapping_v2::HierarchySrvResponse res;
//  if(!hierarchy_service_client_.call(req, res)){
//    ROS_ERROR("SEMANTIC MAP CALL FAILED");
//    return false;
//  }
//
//  for(const auto& l : res.links){
//    if(l.room1 == this_room && l.room2 < 0){
//      if(!exploreRoom(link))
//        return false;
//    }
//  }
//
//  if(!hierarchy_service_client_.call(req, res)){
//    ROS_ERROR("SEMANTIC MAP CALL FAILED");
//    return false;
//  }
//
//  semantic_mapping_v2::HierarchyLinkMsg link_updated;
//  for(const auto& l : res.links){
//    if(link.room1 == l.room1 && l.room2 == this_room){
//      link_updated = l;
//    }
//  }
//  if(link_updated.room1 < 0){
//    ROS_ERROR("LINK NOT FOUND AGAIN");
//    return false;
//  }
//
//  wait_rate.sleep();
//  pose = link_updated.door2_pose;
//  angle = 2*std::acos(pose.orientation.w);
//  pose.position.x += 0.2*std::cos(angle);
//  pose.position.y += 0.2*std::sin(angle);
//  sendGoal(pose, 0, link_updated.room2);
//  while(curr_goal_.action != -1){
//    ros::spinOnce();
//  }
//  if(last_state_.state_ != actionlib::SimpleClientGoalState::SUCCEEDED){
//    ROS_ERROR("MOVE BACK TO ROOM FAILED");
//    return false;
//  }
//
//  return true;
//}