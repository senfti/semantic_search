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
    std::list<int> quick_searchable_;
    std::list<int> quick_searched_;
    std::list<int> not_explored_;
    size_t num_rooms_ = 0;

    State() {};
    State(int current_room, const std::list<int>& searchable, const std::list<int>& quick_searchable, const std::list<int>& not_explored, size_t num_rooms)
      : current_room_(current_room), searchable_(searchable), quick_searchable_(quick_searchable), not_explored_(not_explored), num_rooms_(num_rooms){
    }
    State(const HierarchyMap& graph, int current_room){
      updateState(graph, current_room);
    }

    bool fullySearched() const { return searchable_.empty(); }

    void updateState(const HierarchyMap& graph, int current_room){
      current_room_ = current_room;
      if(graph.search_prob_.size() > num_rooms_){
        for(int i=num_rooms_; i<graph.search_prob_.size(); i++){
          if(graph.not_explored_[i]){
            searchable_.push_back(i);
            not_explored_.push_back(i);
          }
          else{
            searchable_.push_back(i);
            quick_searchable_.push_back(i);
          }
        }
      }
      num_rooms_ = graph.search_prob_.size();
    }

    void changeState(const Action& action){
      if(action.type_ == Action::SEARCH){
        searchable_.remove(action.target_room_);
        quick_searchable_.remove(action.target_room_);
        not_explored_.remove(action.target_room_);
        current_room_ = action.target_room_;
      }
      else if(action.type_ == Action::QUICK_SEARCH){
        quick_searchable_.remove(action.target_room_);
        quick_searched_.push_back(action.target_room_);
        not_explored_.remove(action.target_room_);
        current_room_ = action.target_room_;
      }
      else if(action.type_ == Action::EXPLORE){
        not_explored_.remove(action.target_room_);
        quick_searchable_.push_back(action.target_room_);
        current_room_ = action.target_room_;
      }
      else if(action.type_ == Action::MOVE_TO){
        current_room_ = action.target_room_;
      }
    }

    void changeState(const SearchAction& action){
      if(action.type_ == SearchAction::SEARCH){
        searchable_.remove(action.target_);
        quick_searchable_.remove(action.target_);
        not_explored_.remove(action.target_);
        current_room_ = action.target_;
      }
      else if(action.type_ == SearchAction::QUICK_SEARCH){
        quick_searchable_.remove(action.target_);
        quick_searched_.push_back(action.target_);
        not_explored_.remove(action.target_);
        current_room_ = action.target_;
      }
    }

    std::deque<SearchAction> getPossibleSearchActions() const {
      std::deque<SearchAction> actions;
      for(const auto& s : searchable_)
        actions.push_back(SearchAction(SearchAction::SEARCH, s));
      for(const auto& s : quick_searchable_)
        actions.push_back(SearchAction(SearchAction::QUICK_SEARCH, s));
      return actions;
    }

    std::deque<SearchAction> getPossibleFullSearchActions() const {
      std::deque<SearchAction> actions;
      for(const auto& s : searchable_){
        actions.push_back(SearchAction(SearchAction::SEARCH, s));
      }
      return actions;
    }
};

#endif //HL_PLANNER_STATE_H
