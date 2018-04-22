//
// Created by thomas on 17.12.17.
//

#ifndef EXECUTION_ACTIONSERVER_H
#define EXECUTION_ACTIONSERVER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <execution/ExecuteAction.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <semantic_mapping_v2/RoomSwitchMsg.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/OccupancyGrid.h>

#include <execution/StartRotationStateMachine.h>
#include <execution/Explorer.h>
#include <execution/Searcher.h>

class ExecuteActionServer{
  public:
    enum class MoveBaseState {WAITING, GOAL_SENT, RUNNING, STOPPED, FINISHED};
    const int START_ROTATION_STEPS = 8;
    float MOVE_MAX_ROT_VEL = 5.0;
    float MOVE_MAX_TRANS_VEL = 0.5;
    float PEEK_TURN_SPEED = 0.3;
    int SEARCHER_CALCULATION_SKIPS = 10;

  protected:
    ros::NodeHandle nh_;
    tf::TransformListener tf_listener_;
    actionlib::SimpleActionServer<execution::ExecuteAction> action_server_;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> action_client_;
    execution::ExecuteGoal goal_;

    MoveBaseState move_base_state_ = MoveBaseState::WAITING;
    move_base_msgs::MoveBaseGoal move_base_goal_;

    bool move_to_first_reached_ = false;
    bool move_to_map_switched_ = false;
    float move_offset_dist_;
    StartRotationStateMachine start_rotation_state_machine_;
    Explorer explorer_;
    Searcher searcher_;

    nav_msgs::OccupancyGrid curr_map_;

    ros::Subscriber map_switch_sub_;
    ros::Subscriber door_pose_sub_;
    ros::Subscriber map_sub_;

    ros::Publisher frontier_pub_;
    ros::Publisher vel_pub_;
    ros::Publisher curr_action_pub_;
    ros::Publisher room_explored_pub_;

    void sendMoveBaseGoal(const geometry_msgs::Pose& pose);
    void doMoveTo();
    void doExplore();
    void doSearch();
    void doEnterRoomRotation();
    void doStartRotation();
    tf::Transform offsetPose(tf::Transform pose, double dist, bool forward = true);
    geometry_msgs::Pose offsetPose(const geometry_msgs::Pose& pose, double dist, bool forward = true);

  public:
    ExecuteActionServer();

    void activeCb();
    void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);
    void doneCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result);
    void mapCb(const nav_msgs::OccupancyGridConstPtr& msg);

    void goalCb();
    void preemptCb();

    void mapSwitchCb(const semantic_mapping_v2::RoomSwitchMsgConstPtr& msg);
    void doorPoseCb(const geometry_msgs::PoseArrayConstPtr& msg);

    void run();
};

#endif //EXECUTION_ACTIONSERVER_H
