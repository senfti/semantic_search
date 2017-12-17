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
#include <std_msgs/Int16.h>
#include <execution/StartRotationStateMachine.h>

class ExecuteActionServer{
  public:
    enum class MoveBaseState {WAITING, RUNNING, STOPPED, FINISHED};
    const int START_ROTATION_STEPS = 8;
    const float MOVE_MAX_ROT_VEL = 5.0;
    const float MOVE_MAX_TRANS_VEL = 0.5;

  protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<execution::ExecuteAction> action_server_;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> action_client_;
    execution::ExecuteGoal goal_;

    MoveBaseState move_base_state_ = MoveBaseState::WAITING;
    StartRotationStateMachine start_rotation_state_machine_;

    ros::Subscriber map_switch_sub_;

    void sendMoveBaseGoal(const geometry_msgs::Pose& pose);
    void doMoveTo();
    void doExplore();
    void doSearch();
    void doStartRotation();

  public:
    ExecuteActionServer();

    void activeCb();
    void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);
    void doneCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result);

    void goalCb();
    void preemptCb();

    void mapSwitchCb(const std_msgs::Int16ConstPtr& msg);

    void run();
};

#endif //EXECUTION_ACTIONSERVER_H
