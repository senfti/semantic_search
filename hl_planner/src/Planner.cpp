//
// Created by thomas on 20.12.17.
//

#include <hl_planner/Planner.h>

Planner::Planner()
  : execute_action_client_(nh_, "execute_action", false), hierarchy_service_client_(nh_.serviceClient<semantic_mapping_v2::HierarchySrv>("hierarchy_srv")), last_state_(actionlib::SimpleClientGoalState(actionlib::SimpleClientGoalState::PENDING))
{
  while(!execute_action_client_.waitForServer(ros::Duration(1.0))){
    ROS_WARN("EXECUTE ACTION SERVER NOT UP");
    ros::spinOnce();
  }
  while(!hierarchy_service_client_.waitForExistence(ros::Duration(1.0))){
    ROS_WARN("HIERARCHY SERVICE NOT EXISTING");
    ros::spinOnce();
  }

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


bool Planner::exploreRoom(semantic_mapping_v2::HierarchyLinkMsg link){
  ros::Rate rate(10.0);
  ros::Rate wait_rate(0.5);
  int this_room = num_visited_rooms_;

  geometry_msgs::Pose pose = link.door1_pose;
  double angle = 2*std::acos(pose.orientation.w);
  pose.position.x += 0.2*std::cos(angle);
  pose.position.y += 0.2*std::sin(angle);
  sendGoal(pose, 0, num_visited_rooms_);
  while(curr_goal_.action != -1){
    ros::spinOnce();
    rate.sleep();
  }
  if(last_state_.state_ != actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_ERROR("MOVE TO ROOM FAILED");
    return false;
  }
  num_visited_rooms_++;
  wait_rate.sleep();

  sendGoal(geometry_msgs::Pose(), 1, 0);
  while(curr_goal_.action != -1){
    ros::spinOnce();
  }
  if(last_state_.state_ != actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_ERROR("EXPLORE FAILED");
    return false;
  }

  semantic_mapping_v2::HierarchySrvRequest req;
  semantic_mapping_v2::HierarchySrvResponse res;
  if(!hierarchy_service_client_.call(req, res)){
    ROS_ERROR("SEMANTIC MAP CALL FAILED");
    return false;
  }

  for(const auto& l : res.links){
    if(l.room1 == this_room && l.room2 < 0){
      if(!exploreRoom(link))
        return false;
    }
  }

  if(!hierarchy_service_client_.call(req, res)){
    ROS_ERROR("SEMANTIC MAP CALL FAILED");
    return false;
  }

  semantic_mapping_v2::HierarchyLinkMsg link_updated;
  for(const auto& l : res.links){
    if(link.room1 == l.room1 && l.room2 == this_room){
      link_updated = l;
    }
  }
  if(link_updated.room1 < 0){
    ROS_ERROR("LINK NOT FOUND AGAIN");
    return false;
  }

  wait_rate.sleep();
  pose = link_updated.door2_pose;
  angle = 2*std::acos(pose.orientation.w);
  pose.position.x += 0.2*std::cos(angle);
  pose.position.y += 0.2*std::sin(angle);
  sendGoal(pose, 0, link_updated.room2);
  while(curr_goal_.action != -1){
    ros::spinOnce();
  }
  if(last_state_.state_ != actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_ERROR("MOVE BACK TO ROOM FAILED");
    return false;
  }

  return true;
}


void Planner::exploreAll(){
  sendGoal(geometry_msgs::Pose(), 1, 0);
  while(curr_goal_.action != -1){
    ros::spinOnce();
  }
  if(last_state_.state_ != actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_ERROR("FIRST EXPLORE FAILED");
    return;
  }

  semantic_mapping_v2::HierarchySrvRequest req;
  semantic_mapping_v2::HierarchySrvResponse res;
  if(!hierarchy_service_client_.call(req, res)){
    ROS_ERROR("FIRST SEMANTIC MAP CALL FAILED");
    return;
  }

  for(const auto& link : res.links){
    if(!exploreRoom(link))
      return;
    std::cout << "FINISHED ROOM" << std::endl;
    std::cin.get();
  }
}
