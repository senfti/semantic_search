//
// Created by thomas on 31.12.17.
//

#ifndef HL_PLANNER_PLAN_H
#define HL_PLANNER_PLAN_H

#include <hl_planner/Action.h>
#include <hl_planner/State.h>
#include <hl_planner/HierarchyMap.h>

class SearchPlan{
  public:
    std::vector<SearchAction> actions_;
    State end_state_;
    float expected_search_time_ = 0.f;
    float full_search_time_ = 0.f;
    float found_prob_ = 0.f;

    SearchPlan(const State& end_state = State(), const std::vector<SearchAction>& actions = std::vector<SearchAction>(),
               float expected_search_time = 0.f, float full_search_time = 0.f, float find_prob = 0.f)
      : actions_(actions), end_state_(end_state), expected_search_time_(expected_search_time), full_search_time_(full_search_time), found_prob_(find_prob)
    {}

    State addAction(const SearchAction& action, const HierarchyMap& graph, int curr_room){
      actions_.push_back(action);
      end_state_.changeState(action);
      float prob = graph.search_prob_[action.target_];
      float time = graph.getExpectedSearchTime(curr_room, action.target_);
      expected_search_time_ += (1.f-found_prob_)*time;
      full_search_time_ += graph.getFullSearchTime(curr_room, action.target_);
      found_prob_ = 1.f-(1.f-found_prob_)*(1.f-prob);

      return end_state_;
    }

    std::string getPlanString() const {
      std::string s;
      for(const auto& a : actions_)
         s = s + (a.type_ == SearchAction::SEARCH ? "F" : "Q") + std::to_string(a.target_) + " ";
      s = s + "\t\t" + "exp: " + std::to_string(expected_search_time_) + ", full: " + std::to_string(full_search_time_) + ", prob: " + std::to_string(found_prob_);
      return s;
    }
};

class Plan{
  public:
    std::list<Action> actions_;
    std::list<State> states_;

    bool finished() const { return actions_.empty(); }

    std::string getPlanString() {
      std::vector<std::string> type_strings = {"M", "E", "S", "Q"};
      std::string s;
      for(const auto& a : actions_)
        s = s + type_strings[a.type_] + std::to_string(a.target_room_) + " ";
      return s;
    }
};

#endif //HL_PLANNER_PLAN_H
