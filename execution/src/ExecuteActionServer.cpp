//
// Created by thomas on 17.12.17.
//

#include "execution/ExecuteActionServer.h"
#include <std_msgs/Int8.h>

ExecuteActionServer::ExecuteActionServer()
      : action_server_(nh_, "execute_action", false), action_client_(nh_, "move_base", false), explorer_(&tf_listener_), searcher_(&tf_listener_)
{
  ros::NodeHandle("~").param("MOVE_MAX_ROT_VEL", MOVE_MAX_ROT_VEL, MOVE_MAX_ROT_VEL);
  ros::NodeHandle("~").param("MOVE_MAX_TRANS_VEL", MOVE_MAX_TRANS_VEL, MOVE_MAX_TRANS_VEL);

  goal_.action = -1;
  action_server_.registerGoalCallback(boost::bind(&ExecuteActionServer::goalCb, this));
  action_server_.registerPreemptCallback(boost::bind(&ExecuteActionServer::preemptCb, this));
  action_server_.start();

  ros::spinOnce();
  while(!action_client_.waitForServer(ros::Duration(1.0))){
    ROS_WARN("MOVE_BASE Action Server not there");
    ros::spinOnce();
    break;
  }
  map_switch_sub_ = nh_.subscribe("map_switch", 1, &ExecuteActionServer::mapSwitchCb, this);
  frontier_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("current_frontier", 1, true);
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("navigation_velocity_smoother/raw_cmd_vel", 1);
  curr_action_pub_ = nh_.advertise<std_msgs::Int8>("current_action", 1);
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
  //ROS_INFO("MOVE_BASE WORKING");
}


void ExecuteActionServer::activeCb(){
  move_base_state_ = MoveBaseState::RUNNING;
  ROS_INFO("MOVE_BASE started Action");
}


void ExecuteActionServer::goalCb(){
  action_client_.cancelAllGoals();
  goal_ = *(action_server_.acceptNewGoal());
  move_to_first_reached_ = false;
  ROS_INFO("NEW GOAL: %d, %d, %.3lf, %.3lf, %.3lf, %.3lf", goal_.action, goal_.target, goal_.pose.position.x, goal_.pose.position.y, goal_.pose.orientation.z, goal_.pose.orientation.w);
}


void ExecuteActionServer::preemptCb(){
  ROS_INFO("Preempted");
  execution::ExecuteResult result;
  result.result_number = -3;
  action_server_.setPreempted(result);
  action_client_.cancelAllGoals();
  goal_.action = -1;
  move_base_state_ = MoveBaseState::STOPPED;
}


void ExecuteActionServer::mapSwitchCb(const semantic_mapping_v2::RoomSwitchMsgConstPtr &msg){
  if(goal_.action == 0 && msg->new_room == goal_.target){
    tf::Transform transform;
    tf::Transform pose;
    tf::poseMsgToTF(msg->transform, transform);
    tf::poseMsgToTF(goal_.pose, pose);
    geometry_msgs::Pose new_pose;
    tf::poseTFToMsg(pose*transform, new_pose);
    sendMoveBaseGoal(new_pose);
  }
  else{
    action_client_.cancelAllGoals();
    execution::ExecuteResult result;
    result.result_number = -3 - (goal_.action != 0 ? 1 : 0) - (msg->new_room != goal_.target ? 2 : 0);
    action_server_.setAborted(result, "ABORTED");
    goal_.action = -1;
    move_base_state_ = MoveBaseState::STOPPED;
  }
}


void ExecuteActionServer::doMoveTo(){
  if(move_base_state_ == MoveBaseState::WAITING) {
    if(!move_to_first_reached_){
      std::system(("rosrun dynamic_reconfigure dynparam set /move_base/DWAPlannerROS max_rot_vel " + std::to_string(MOVE_MAX_ROT_VEL)).c_str());
      std::system(("rosrun dynamic_reconfigure dynparam set /move_base/DWAPlannerROS max_trans_vel " + std::to_string(MOVE_MAX_TRANS_VEL)).c_str());
      tf::Transform tmp;
      tf::poseMsgToTF(goal_.pose, tmp);
      tmp.setOrigin(tmp.getOrigin()-tf::Vector3(0.5*tmp.getBasis().getColumn(0).x(), 0.5*tmp.getBasis().getColumn(0).y(),0.0));
      tf::poseTFToMsg(tmp, goal_.pose);
      sendMoveBaseGoal(goal_.pose);
    }
  }
  else if(move_base_state_ == MoveBaseState::GOAL_SENT){
    ;
  }
  else if(move_base_state_ == MoveBaseState::RUNNING){
    ;
  }
  else if(move_base_state_ == MoveBaseState::STOPPED){
    move_base_state_ = MoveBaseState::WAITING;
  }
  else if(move_base_state_ == MoveBaseState::FINISHED){
    if(action_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      if(!move_to_first_reached_){
        move_to_first_reached_ = true;
        tf::Transform tmp;
        tf::poseMsgToTF(goal_.pose, tmp);
        tmp.setOrigin(tmp.getOrigin()+tf::Vector3(tmp.getBasis().getColumn(0).x(), tmp.getBasis().getColumn(0).y(),0.0));
        tf::poseTFToMsg(tmp, goal_.pose);
        sendMoveBaseGoal(goal_.pose);
      }
      else{
        execution::ExecuteResult result;
        result.result_number = 0;
        action_server_.setSucceeded(result, "SUCCESS");
        goal_.action = -1;
        move_base_state_ = MoveBaseState::WAITING;
        move_to_first_reached_ = false;
      }
    }
    else if(action_client_.getState() == actionlib::SimpleClientGoalState::PREEMPTED){
      execution::ExecuteResult result;
      result.result_number = -1;
      action_server_.setPreempted(result, "PREEMPTED");
      goal_.action = -1;
      move_base_state_ = MoveBaseState::WAITING;
    }
    else{
      execution::ExecuteResult result;
      result.result_number = -2;
      action_server_.setAborted(result, "ABORTED");
      goal_.action = -1;
      move_base_state_ = MoveBaseState::WAITING;
    }
  }
}


void ExecuteActionServer::doExplore(){
  if(!explorer_.running()){
    explorer_.start();
  }
  if(explorer_.finished()){
    ROS_INFO("Finished exploration");
    if(move_base_state_ == MoveBaseState::WAITING) {
      ;
    }
    else if(move_base_state_ == MoveBaseState::GOAL_SENT){
      action_client_.cancelAllGoals();
    }
    else if(move_base_state_ == MoveBaseState::RUNNING){
      action_client_.cancelAllGoals();
    }
    else if(move_base_state_ == MoveBaseState::STOPPED){
      ;
    }
    else if(move_base_state_ == MoveBaseState::FINISHED){
      ;
    }
    execution::ExecuteResult result;
    result.result_number = (explorer_.doorFoundStopped() ? 1 : 0);
    action_server_.setSucceeded(result, "SUCCESS");
    goal_.action = -1;
    return;
  }

  if(move_base_state_ == MoveBaseState::WAITING) {
    geometry_msgs::PoseStamped pose;
    pose.pose = explorer_.getNextFrontier();
    pose.header.frame_id = "map";
    pose.header.stamp = ros::Time::now();
    static unsigned seq = 0;
    pose.header.seq = seq++;
    frontier_pub_.publish(pose);
    sendMoveBaseGoal(pose.pose);
  }
  else if(move_base_state_ == MoveBaseState::GOAL_SENT){
    ;
  }
  else if(move_base_state_ == MoveBaseState::RUNNING){
    if(explorer_.hasFrontierChanged()){
      geometry_msgs::PoseStamped pose;
      pose.pose = explorer_.getNextFrontier();
      pose.header.frame_id = "map";
      pose.header.stamp = ros::Time::now();
      static unsigned seq = 0;
      pose.header.seq = seq++;
      frontier_pub_.publish(pose);
      sendMoveBaseGoal(pose.pose);
    }
  }
  else if(move_base_state_ == MoveBaseState::STOPPED){
    ;
  }
  else if(move_base_state_ == MoveBaseState::FINISHED){
    if(action_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      geometry_msgs::PoseStamped pose;
      pose.pose = explorer_.getNextFrontier();
      pose.header.frame_id = "map";
      pose.header.stamp = ros::Time::now();
      static unsigned seq = 0;
      pose.header.seq = seq++;
      frontier_pub_.publish(pose);
      sendMoveBaseGoal(pose.pose);
    }
    else if(action_client_.getState() == actionlib::SimpleClientGoalState::PREEMPTED){
      ;
    }
    else{
      if(explorer_.did_abort_){
        ROS_INFO("ABORTED completely");
        execution::ExecuteResult result;
        result.result_number = -2;
        action_server_.setAborted(result, "ABORTED");
        goal_.action = -1;
      }
      else{
        ROS_INFO("ABORTED first time");
        geometry_msgs::PoseStamped pose;
        pose.pose = explorer_.getNextFrontier();
        pose.header.frame_id = "map";
        pose.header.stamp = ros::Time::now();
        static unsigned seq = 0;
        pose.header.seq = seq++;
        frontier_pub_.publish(pose);
        sendMoveBaseGoal(pose.pose);
        explorer_.did_abort_ = true;
      }
    }
  }
}


void ExecuteActionServer::doSearch(){
  if(!searcher_.running()){
    searcher_.start(goal_.target, goal_.action == 3, goal_.pose);
  }
  static int i = 0;
  if(i%20 == 0)
    searcher_.doCalculations();
  if(searcher_.finished()){
    ROS_INFO("Finished search");
    if(move_base_state_ == MoveBaseState::WAITING) {
      ;
    }
    else if(move_base_state_ == MoveBaseState::GOAL_SENT){
      action_client_.cancelAllGoals();
    }
    else if(move_base_state_ == MoveBaseState::RUNNING){
      action_client_.cancelAllGoals();
    }
    else if(move_base_state_ == MoveBaseState::STOPPED){
      ;
    }
    else if(move_base_state_ == MoveBaseState::FINISHED){
      ;
    }
    execution::ExecuteResult result;
    result.result_number = (searcher_.objFound() ? 100 : 0);
    action_server_.setSucceeded(result, "SUCCESS");
    goal_.action = -1;
    return;
  }

  if(move_base_state_ == MoveBaseState::WAITING) {
    geometry_msgs::PoseStamped pose;
    pose.pose = searcher_.getNextViewPose();
    pose.header.frame_id = "map";
    pose.header.stamp = ros::Time::now();
    static unsigned seq = 0;
    pose.header.seq = seq++;
    frontier_pub_.publish(pose);
    sendMoveBaseGoal(pose.pose);
  }
  else if(move_base_state_ == MoveBaseState::GOAL_SENT){
    ;
  }
  else if(move_base_state_ == MoveBaseState::RUNNING){
    if(searcher_.hasViewPoseChanged()){
      geometry_msgs::PoseStamped pose;
      pose.pose = searcher_.getNextViewPose();
      pose.header.frame_id = "map";
      pose.header.stamp = ros::Time::now();
      static unsigned seq = 0;
      pose.header.seq = seq++;
      frontier_pub_.publish(pose);
      sendMoveBaseGoal(pose.pose);
    }
  }
  else if(move_base_state_ == MoveBaseState::STOPPED){
    ;
  }
  else if(move_base_state_ == MoveBaseState::FINISHED){
    if(action_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      geometry_msgs::PoseStamped pose;
      pose.pose = searcher_.getNextViewPose();
      pose.header.frame_id = "map";
      pose.header.stamp = ros::Time::now();
      static unsigned seq = 0;
      pose.header.seq = seq++;
      frontier_pub_.publish(pose);
      sendMoveBaseGoal(pose.pose);
    }
    else if(action_client_.getState() == actionlib::SimpleClientGoalState::PREEMPTED){
      ;
    }
    else{
      if(searcher_.did_abort_){
        ROS_INFO("ABORTED completely");
        execution::ExecuteResult result;
        result.result_number = -2;
        action_server_.setAborted(result, "ABORTED");
        goal_.action = -1;
      }
      else{
        ROS_INFO("ABORTED first time");
        geometry_msgs::PoseStamped pose;
        pose.pose = searcher_.getNextViewPose();
        pose.header.frame_id = "map";
        pose.header.stamp = ros::Time::now();
        static unsigned seq = 0;
        pose.header.seq = seq++;
        frontier_pub_.publish(pose);
        sendMoveBaseGoal(pose.pose);
        searcher_.did_abort_ = true;
      }
    }
  }
}


void ExecuteActionServer::doStartRotation(){
  static tf::StampedTransform old_transform;
  static bool started = false;
  static float angle = 0.f;
  static ros::Time start_time;
  if(!started){
    start_time = ros::Time::now();
    tf_listener_.lookupTransform("map", "base_link", ros::Time(0), old_transform);
    started = true;
    angle = 0.f;
  }

  tf::StampedTransform transform;
  tf_listener_.lookupTransform("map", "base_link", ros::Time(0), transform);
  tf::Transform diff = transform.inverse()*old_transform;
  angle += tf::getYaw(diff.getRotation());
  if(angle > 2*M_PI || angle < -2*M_PI){
    started = false;
    execution::ExecuteResult result;
    result.result_number = 0;
    action_server_.setSucceeded(result, "SUCCESS");
    goal_.action = -1;
  }
  old_transform = transform;
  if(ros::Time::now()-start_time > ros::Duration(10.0)){
    started = false;
    execution::ExecuteResult result;
    result.result_number = -11;
    action_server_.setAborted(result, "ABORTED");
    goal_.action = -1;
  }
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = 0.0;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = 1.0;
  vel_pub_.publish(cmd_vel);
}


void ExecuteActionServer::run(){
  ros::Rate rate(10.0);
  while(ros::ok()){
    ros::spinOnce();
    if(goal_.action == 0){
      doMoveTo();
      explorer_.stop();
      searcher_.stop();
      start_rotation_state_machine_.reset();
    }
    else if(goal_.action == 1){
      doExplore();
      searcher_.stop();
      start_rotation_state_machine_.reset();
    }
    else if(goal_.action == 2 || goal_.action == 3){
      doSearch();
      explorer_.stop();
      start_rotation_state_machine_.reset();
    }
    else if(goal_.action == 4){
      doStartRotation();
      explorer_.stop();
      searcher_.stop();
    }
    else{
      move_base_state_ = MoveBaseState::WAITING;
      explorer_.stop();
      searcher_.stop();
      start_rotation_state_machine_.reset();
    }
    //ROS_INFO("MOVE_BASE_STATE: %d", int(move_base_state_));
    std_msgs::Int8 action_nr;
    action_nr.data = goal_.action;
    curr_action_pub_.publish(action_nr);
    rate.sleep();
  }
}


void ExecuteActionServer::sendMoveBaseGoal(const geometry_msgs::Pose& pose){
  static unsigned int seq = 0;
  move_base_goal_.target_pose.header.stamp = ros::Time::now();
  move_base_goal_.target_pose.header.seq = seq++;
  move_base_goal_.target_pose.header.frame_id = "/map";
  move_base_goal_.target_pose.pose = pose;

  ROS_INFO("SENT GOAL: %.3lf, %.3lf, %.3lf, %.3lf", move_base_goal_.target_pose.pose.position.x, move_base_goal_.target_pose.pose.position.y,
           move_base_goal_.target_pose.pose.orientation.z, move_base_goal_.target_pose.pose.orientation.w);
  action_client_.sendGoal(move_base_goal_, boost::bind(&ExecuteActionServer::doneCb, this, _1, _2),
                          boost::bind(&ExecuteActionServer::activeCb, this), boost::bind(&ExecuteActionServer::feedbackCb, this, _1));
  move_base_state_ = MoveBaseState::GOAL_SENT;
}
