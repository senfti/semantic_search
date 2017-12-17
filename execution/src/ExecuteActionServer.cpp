//
// Created by thomas on 17.12.17.
//

#include "execution/ExecuteActionServer.h"

ExecuteActionServer::ExecuteActionServer()
      : action_server_(nh_, "execute_action", false), action_client_(nh_, "move_base", false)
{
  goal_.action = -1;
  action_server_.registerGoalCallback(boost::bind(&ExecuteActionServer::goalCb, this));
  action_server_.registerPreemptCallback(boost::bind(&ExecuteActionServer::preemptCb, this));
  action_server_.start();

  ros::spinOnce();
  while(!action_client_.waitForServer(ros::Duration(1.0))){
    ROS_WARN("MOVE_BASE Action Server not there");
    ros::spinOnce();
  }
  map_switch_sub_ = nh_.subscribe("map_switch", 1, &ExecuteActionServer::mapSwitchCb, this);
  ROS_INFO("INITIALIZATION FINISHED");
}


void ExecuteActionServer::doneCb(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result){
  if(state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("GOAL REACHED");
  }
  else{
    ROS_INFO("%s:", state.text_.c_str());
  }
  if(move_base_state_ != MoveBaseState::STOPPED)
    move_base_state_ = MoveBaseState::FINISHED;
}


void ExecuteActionServer::feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback){
  ROS_INFO("MOVE_BASE WORKING");
}


void ExecuteActionServer::activeCb(){
  move_base_state_ = MoveBaseState::RUNNING;
  ROS_INFO("MOVE_BASE started Action");
}


void ExecuteActionServer::goalCb(){
  action_client_.cancelAllGoals();
  goal_ = *(action_server_.acceptNewGoal());
  ROS_INFO("NEW GOAL: %d, %d, %.3lf, %.3lf, %.3lf, %.3lf", goal_.action, goal_.target, goal_.pose.position.x, goal_.pose.position.y, goal_.pose.orientation.z, goal_.pose.orientation.w);
}


void ExecuteActionServer::preemptCb(){
  ROS_INFO("Preempted");
  execution::ExecuteResult result;
  result.error_number = 3;
  action_server_.setPreempted(result);
  action_client_.cancelAllGoals();
  goal_.action = -1;
  move_base_state_ = MoveBaseState::STOPPED;
}


void ExecuteActionServer::mapSwitchCb(const std_msgs::Int16ConstPtr &msg){
  if(goal_.action == 0 && msg->data == goal_.target){
    action_client_.cancelAllGoals();
    execution::ExecuteResult result;
    result.error_number = 0;
    result.num_reached_poses = 1;
    action_server_.setSucceeded(result, "SUCCESS");
    goal_.action = -1;
    move_base_state_ = MoveBaseState::STOPPED;
  }
  else{
    action_client_.cancelAllGoals();
    execution::ExecuteResult result;
    result.error_number = 3 + (goal_.action != 0 ? 1 : 0) + (msg->data != goal_.target ? 2 : 0);
    result.num_reached_poses = 0;
    action_server_.setAborted(result, "ABORTED");
    goal_.action = -1;
    move_base_state_ = MoveBaseState::STOPPED;
  }
}


void ExecuteActionServer::run(){
  ros::Rate rate(10.0);
  while(ros::ok()){
    ros::spinOnce();
    if(goal_.action == 0){
      if(move_base_state_ == MoveBaseState::WAITING) {
        sendMoveBaseGoal(goal_.pose);
      }
      else if(move_base_state_ == MoveBaseState::RUNNING){
        ;
      }
      else if(move_base_state_ == MoveBaseState::STOPPED){
        move_base_state_ = MoveBaseState::WAITING;
      }
      else if(move_base_state_ == MoveBaseState::FINISHED){
        if(action_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
          execution::ExecuteResult result;
          result.error_number = 0;
          result.num_reached_poses = 1;
          action_server_.setSucceeded(result, "SUCCESS");
        }
        else if(action_client_.getState() == actionlib::SimpleClientGoalState::PREEMPTED){
          execution::ExecuteResult result;
          result.error_number = 1;
          result.num_reached_poses = 0;
          action_server_.setPreempted(result, "PREEMPTED");
        }
        else{
          execution::ExecuteResult result;
          result.error_number = 2;
          result.num_reached_poses = 0;
          action_server_.setAborted(result, "ABORTED");
        }
        goal_.action = -1;
        move_base_state_ = MoveBaseState::WAITING;
      }
    }
    else if(goal_.action == 1){
      goal_.action = -1;
    }
    else if(goal_.action == 2){
      goal_.action = -1;
    }
    rate.sleep();
  }
}


void ExecuteActionServer::sendMoveBaseGoal(const geometry_msgs::Pose& pose){
  static unsigned int seq = 0;
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.header.seq = seq++;
  goal.target_pose.header.frame_id = "/map";
  goal.target_pose.pose = pose;

  ROS_INFO("SENT GOAL: %.3lf, %.3lf, %.3lf, %.3lf", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w);
  action_client_.sendGoal(goal, boost::bind(&ExecuteActionServer::doneCb, this, _1, _2), boost::bind(&ExecuteActionServer::activeCb, this), boost::bind(&ExecuteActionServer::feedbackCb, this, _1));
}
