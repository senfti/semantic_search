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
    int HIERARCHY_MAX_TRIES = 5;

  private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionClient<execution::ExecuteAction> execute_action_client_;
    ros::ServiceClient hierarchy_service_client_;

    State state_;
    std::vector<int> explored_rooms_;

    semantic_mapping_v2::HierarchySrvResponse getHierarchy(int max_tries);

  public:
    Planner();

    SearchPlan greedyPlan(const HierarchyMap& graph, const State& state);
    Plan generateFullPlan(const SearchPlan &search_plan, State state, const HierarchyMap& graph);
    Plan generatePlan(const HierarchyMap& graph, const State& state);

    //bool exploreRoom(semantic_mapping_v2::HierarchyLinkMsg link);
    void run(int obj, std::string run_name = std::string());
    void justPlan(int obj);
    void exploreAll();

    actionlib::SimpleClientGoalState sendGoal(const Action& action);
};

#endif //HL_PLANNER_PLANNER_H
