//
// Created by thomas on 20.12.17.
//

#include <hl_planner/Planner.h>

#include <hl_planner/HierarchyMap.h>
#include <hl_planner/Action.h>
#include <hl_planner/Plan.h>
#include <hl_planner/State.h>

Planner::Planner()
: execute_action_client_(nh_, "execute_action", false), hierarchy_service_client_(nh_.serviceClient<semantic_mapping_v2::HierarchySrv>("hierarchy_srv"))
{
  ros::NodeHandle("~").param("UNEXPLORED_SEARCH_TIME_ESTIMATE", HierarchyMap::UNEXPLORED_SEARCH_TIME_ESTIMATE, HierarchyMap::UNEXPLORED_SEARCH_TIME_ESTIMATE);
  ros::NodeHandle("~").param("UNEXPLORED_QUICK_SEARCH_TIME_ESTIMATE", HierarchyMap::UNEXPLORED_QUICK_SEARCH_TIME_ESTIMATE, HierarchyMap::UNEXPLORED_QUICK_SEARCH_TIME_ESTIMATE);
  ros::NodeHandle("~").param("UNEXPLORED_PROB_ESTIMATE", HierarchyMap::UNEXPLORED_PROB_ESTIMATE, HierarchyMap::UNEXPLORED_PROB_ESTIMATE);
  ros::NodeHandle("~").param("UNEXPLORED_QUICK_SEARCH_PROB_ESTIMATE", HierarchyMap::UNEXPLORED_QUICK_SEARCH_PROB_ESTIMATE, HierarchyMap::UNEXPLORED_QUICK_SEARCH_PROB_ESTIMATE);

  while(!execute_action_client_.waitForServer(ros::Duration(1.0))){
    ROS_WARN("EXECUTE ACTION SERVER NOT UP");
    ros::spinOnce();
  }
  while(!hierarchy_service_client_.waitForExistence(ros::Duration(1.0))){
    ROS_WARN("HIERARCHY SERVICE NOT EXISTING");
    ros::spinOnce();
  }
}


actionlib::SimpleClientGoalState Planner::sendGoal(const Action& action){
  execution::ExecuteGoal goal;
  goal.pose = action.pose_;
  goal.action = action.type_;
  goal.target_obj = action.target_obj_;
  goal.target_room = action.target_room_;
  ROS_ERROR("SEND GOAL %d %d %d %.3lf %.3lf %.3lf %.3lf", goal.action, goal.target_obj, goal.target_room, goal.pose.position.x, goal.pose.position.y, goal.pose.orientation.z, goal.pose.orientation.w);
  //std::cin.get();
  execute_action_client_.sendGoal(goal);
  while(!execute_action_client_.waitForResult(ros::Duration(1.0)))
    ros::spinOnce();
  std::cout << "GOAL EXECUTED, state = " << execute_action_client_.getState().toString() << std::endl;
  return execute_action_client_.getState();
}


semantic_mapping_v2::HierarchySrvResponse Planner::getHierarchy(int max_tries){
  for(int i=0; i<max_tries; i++){
    semantic_mapping_v2::HierarchySrvRequest req;
    req.debug_room = -1;
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
    curr_state = plan.addAction(best_action, graph, curr_state.current_room_, curr_state.quick_searched_);
    search_actions_ = curr_state.getPossibleFullSearchActions();
  }

  return plan;
}


SearchPlan finishGreedy(const HierarchyMap &graph, SearchPlan search_plan, float cutoff_time){
  State curr_state = search_plan.end_state_;
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
    curr_state = search_plan.addAction(best_action, graph, curr_state.current_room_, curr_state.quick_searched_);
    if(search_plan.expected_search_time_ >= cutoff_time)
      return search_plan;
    search_actions_ = curr_state.getPossibleFullSearchActions();
  }
  return search_plan;
}


SearchPlan plan(const HierarchyMap &graph, const SearchPlan& old_plan, float& cutoff_time, float cutoff_prob, int level){
  if(level >= cutoff_prob)
    return finishGreedy(graph, old_plan, cutoff_time);

  std::deque<SearchAction> search_actions_ = old_plan.end_state_.getPossibleSearchActions();
  if(search_actions_.empty()){
    return old_plan;
  }

  SearchPlan best_plan(old_plan.end_state_, std::vector<SearchAction>(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), 0.f);
  for(const auto& a : search_actions_){
    SearchPlan new_plan = old_plan;
    new_plan.addAction(a, graph, old_plan.end_state_.current_room_, old_plan.end_state_.quick_searched_);
    if(new_plan.expected_search_time_ >= cutoff_time)
      continue;

    new_plan = plan(graph, new_plan, cutoff_time, cutoff_prob, level+1);
    if(new_plan.expected_search_time_ < best_plan.expected_search_time_){
      best_plan = new_plan;
      if(best_plan.expected_search_time_ < cutoff_time)
        cutoff_time = best_plan.expected_search_time_;
    }
  }
  return best_plan;
}


Plan Planner::generateFullPlan(const SearchPlan &search_plan, State state, const HierarchyMap& graph){
  Plan plan;
  for(const auto& action : search_plan.actions_){
    std::vector<int> room_path = graph.travel_path_[state.current_room_][action.target_];
    std::vector<geometry_msgs::Pose> waypoints = graph.travel_waypoints_[state.current_room_][action.target_];
    for(int i=1; i<room_path.size(); i++){
      plan.actions_.push_back(Action(Action::MOVE_TO, graph.searched_obj_, room_path[i], waypoints[i]));
      state.changeState(*plan.actions_.rbegin());
    }
    if(std::find(state.not_explored_.begin(), state.not_explored_.end(), state.current_room_) != state.not_explored_.end()){
      plan.actions_.push_back(Action(Action::EXPLORE, graph.searched_obj_, state.current_room_, geometry_msgs::Pose()));
      state.changeState(*plan.actions_.rbegin());
    }
    plan.actions_.push_back(Action(action.type_ == SearchAction::SEARCH ? Action::SEARCH : Action::QUICK_SEARCH, graph.searched_obj_,
                                   state.current_room_, graph.quick_search_poses_[state.current_room_]));
    state.changeState(*plan.actions_.rbegin());
  }
  if(plan.actions_.front().type_ == Action::MOVE_TO && graph.not_visited_[plan.actions_.front().target_room_]){
    int first_unvisited = 0;
    for(first_unvisited=0; first_unvisited<graph.not_visited_.size(); first_unvisited++)
      if(graph.not_visited_[first_unvisited])
        break;
    if(first_unvisited<graph.not_visited_.size() && first_unvisited != plan.actions_.front().target_room_){
      std::cout << "swapped " << plan.actions_.front().target_room_ << " with " << first_unvisited << std::endl;
      for(auto& a : plan.actions_){
        if(a.target_room_ == first_unvisited)
          a.target_room_ = plan.actions_.front().target_room_;
        else if(a.target_room_ == plan.actions_.front().target_room_)
          a.target_room_ = first_unvisited;
      }
    }
  }
  std::cout << plan.getPlanString() << std::endl;
  return plan;
}


Plan Planner::generatePlan(const HierarchyMap &graph, const State& state){
  ros::Time start = ros::Time::now();
  SearchPlan greedy_plan = greedyPlan(graph, state);
  std::cout << "greedy: " << greedy_plan.getPlanString() << std::endl;

  float cutoff_time = greedy_plan.expected_search_time_;
  SearchPlan best_plan = plan(graph, SearchPlan(state), cutoff_time, 5, 0);
  if(best_plan.expected_search_time_ > greedy_plan.expected_search_time_)
    best_plan = greedy_plan;
  std::cout << "best_plan: " << best_plan.getPlanString() << std::endl;


  std::cout << "Complete best plan in " << (ros::Time::now()-start).toSec() << ": " << best_plan.getPlanString() << std::endl;

  return generateFullPlan(best_plan, state, graph);
}


void Planner::run(int obj){
  state_.resetState();
  while(ros::ok()){
    ros::Duration(2.0).sleep();
    semantic_mapping_v2::HierarchySrvResponse hierarchy = getHierarchy(HIERARCHY_MAX_TRIES);
    if(hierarchy.rooms.empty())
      return;

    if(hierarchy.rooms[hierarchy.curr_room].obj_probs.empty()){
      sendGoal(Action(Action::ROTATE, obj, hierarchy.curr_room));
      continue;
    }

    HierarchyMap graph_map(hierarchy, obj);
    std::cout << graph_map;
    state_.updateState(graph_map, hierarchy.curr_room);
    std::cout << state_;

    Plan plan = generatePlan(graph_map, state_);
    if(plan.finished())
      std::cout << "OBJECT NOT FOUND!" << std::endl;

    actionlib::SimpleClientGoalState execution_state = sendGoal(plan.actions_.front());
    if(execution_state != actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_ERROR("EXECUTION FAILED!");
      std::cout << "Stop trying [y]: ";
      std::string s;
      std::cin >> s;
      if(!s.empty() && s[0] == 'y'){
        ROS_ERROR("QUIT SEARCH");
        return;
      }
    }
    else{
      auto result = execute_action_client_.getResult();
      if(result->result_number == 0){
        state_.changeState(plan.actions_.front());
        if(plan.actions_.front().type_ == Action::EXPLORE)
          explored_rooms_.push_back(plan.actions_.front().target_room_);
        if(plan.actions_.front().type_ == Action::MOVE_TO && graph_map.not_visited_[plan.actions_.front().target_room_])
          sendGoal(Action(Action::ROTATE, obj, hierarchy.curr_room));
      }
      else if(result->result_number == 100){
        std::cout << "OBJECT FOUND" << std::endl;
        return;
      }
    }
  }
}


void Planner::justPlan(int obj){
  std::cout << "Start just plan" << std::endl;
  state_ = State();
  semantic_mapping_v2::HierarchySrvResponse hierarchy = getHierarchy(HIERARCHY_MAX_TRIES);
  if(hierarchy.rooms.empty())
    return;

  std::cout << "Got " << hierarchy.rooms.size() << " rooms" << std::endl;

  HierarchyMap graph_map(hierarchy, obj);
  state_.updateState(graph_map, hierarchy.curr_room);
  std::cout << state_;

  Plan plan = generatePlan(graph_map, state_);
}


void Planner::exploreAll(){
  state_ = State();
  while(ros::ok()){
    semantic_mapping_v2::HierarchySrvResponse hierarchy = getHierarchy(HIERARCHY_MAX_TRIES);
    if(hierarchy.rooms.empty())
      return;

    if(hierarchy.rooms[hierarchy.curr_room].obj_probs.empty()){
      sendGoal(Action(Action::ROTATE, -1, hierarchy.curr_room));
      continue;
    }

    HierarchyMap graph_map(hierarchy, 0);
    std::cout << graph_map;
    state_.updateState(graph_map, hierarchy.curr_room);
    std::cout << state_;

    if(state_.not_explored_.empty())
      return;

    Action a(Action::MOVE_TO,0,0);
    if(hierarchy.curr_room != state_.not_explored_.back()){
      std::vector<int> room_path = graph_map.travel_path_[state_.current_room_][state_.not_explored_.back()];
      std::vector<geometry_msgs::Pose> waypoints = graph_map.travel_waypoints_[state_.current_room_][state_.not_explored_.back()];
      a = Action(Action::MOVE_TO, -1, room_path[1], waypoints[1]);
    }
    else
      a = Action(Action::EXPLORE, -1, state_.not_explored_.back(), geometry_msgs::Pose());
    actionlib::SimpleClientGoalState execution_state = sendGoal(a);
    if(execution_state != actionlib::SimpleClientGoalState::SUCCEEDED){
      std::cout << "EXECUTION FAILED, TRYING FURTHER" << std::endl;
    }
    else{
      auto result = execute_action_client_.getResult();
      if(result->result_number == 0){
        if(a.type_ == Action::MOVE_TO && graph_map.not_visited_[a.target_room_])
          sendGoal(Action(Action::ROTATE, -1, hierarchy.curr_room));
        state_.changeState(a);
      }
    }
  }
}
