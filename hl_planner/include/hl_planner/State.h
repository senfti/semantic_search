//
// Created by thomas on 31.12.17.
//

#ifndef HL_PLANNER_STATE_H
#define HL_PLANNER_STATE_H

#include <list>
#include <deque>
#include <hl_planner/Action.h>
#include <hl_planner/HierarchyMap.h>

class State{
  public:
    int current_room_ = 0;
    std::list<int> searchable_;
    std::list<int> not_explored_;
    size_t num_rooms_ = 0;

    State() {};
    State(int current_room, const std::list<int>& searchable, const std::list<int>& not_explored, size_t num_rooms)
      : current_room_(current_room), searchable_(searchable), not_explored_(not_explored), num_rooms_(num_rooms){
    }
    State(const HierarchyMap& graph, int current_room){
      updateState(graph, current_room);
    }

    bool fullySearched() const { return searchable_.empty(); }

    void updateState(const HierarchyMap& graph, int current_room){
      current_room_ = current_room;
      if(graph.search_prob_.size() > num_rooms_){
        for(int i=num_rooms_; i<graph.search_prob_.size(); i++){
          searchable_.push_back(i);
          not_explored_.push_back(i);
        }
      }
      num_rooms_ = graph.search_prob_.size();
    }

    void changeState(const Action& action){
      if(action.type_ == Action::SEARCH){
        searchable_.remove(action.target_room_);
        not_explored_.remove(action.target_room_);
        current_room_ = action.target_room_;
      }
      else if(action.type_ == Action::QUICK_SEARCH){
        not_explored_.remove(action.target_room_);
        current_room_ = action.target_room_;
      }
      else if(action.type_ == Action::EXPLORE){
        not_explored_.remove(action.target_room_);
        current_room_ = action.target_room_;
      }
      else if(action.type_ == Action::MOVE_TO){
        current_room_ = action.target_room_;
      }
    }

    void changeState(const SearchAction& action){
      if(action.type_ == SearchAction::SEARCH){
        searchable_.remove(action.target_);
        not_explored_.remove(action.target_);
        current_room_ = action.target_;
      }
      else if(action.type_ == SearchAction::QUICK_SEARCH){
        not_explored_.remove(action.target_);
        current_room_ = action.target_;
      }
    }

    void resetState(){
      searchable_.clear();
      for(int i=0; i<num_rooms_; i++){
        searchable_.push_back(i);
      }
    }

    std::deque<SearchAction> getPossibleFullSearchActions() const {
      std::deque<SearchAction> actions;
      for(const auto& s : searchable_){
        actions.push_back(SearchAction(SearchAction::SEARCH, s));
      }
      return actions;
    }

    friend std::ostream& operator<<(std::ostream& os, const State& state){
      os << "State: Current Room=" << state.current_room_ << ", Num Rooms=" << state.num_rooms_ << std::endl;
      os << "Not Searched: ";
      for(const auto& s : state.searchable_)
        os << s << " ";
      os << std::endl << "Not Explored: ";
      for(const auto& s : state.not_explored_)
        os << s << " ";
      os << std::endl << std::endl;

      return os;
    }
};

#endif //HL_PLANNER_STATE_H
