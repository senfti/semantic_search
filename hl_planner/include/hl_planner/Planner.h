//
// Created by thomas on 20.12.17.
//

#ifndef HL_PLANNER_PLANNER_H
#define HL_PLANNER_PLANNER_H

#include <actionlib/client/simple_action_client.h>
#include <semantic_mapping_v2/HierarchySrv.h>
#include <execution/ExecuteAction.h>

#include <hl_planner/Plan.h>
#include <hl_planner/State.h>
#include <hl_planner/HierarchyMap.h>

class Planner{
  public:
    double PLANNER_RATE = 0.2;
    int HIERARCHY_MAX_TRIES = 5;

  private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionClient<execution::ExecuteAction> execute_action_client_;
    ros::ServiceClient hierarchy_service_client_;

    execution::ExecuteGoal curr_goal_;
    actionlib::SimpleClientGoalState last_state_;

    State state_;

    semantic_mapping_v2::HierarchySrvResponse getHierarchy(int max_tries);

  public:
    Planner();

    void activeCb();
    void feedbackCb(const execution::ExecuteFeedbackConstPtr& feedback);
    void doneCb(const actionlib::SimpleClientGoalState& state, const execution::ExecuteResultConstPtr& result);

    SearchPlan greedyPlan(const HierarchyMap& graph, const State& state);
    Plan generatePlan(const HierarchyMap& graph, const State& state);

    //bool exploreRoom(semantic_mapping_v2::HierarchyLinkMsg link);
    void run(int obj);

    void sendGoal(const geometry_msgs::Pose& pose, int action, int target);
};

#endif //HL_PLANNER_PLANNER_H
