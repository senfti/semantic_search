//
// Created by thomas on 31.12.17.
//

#ifndef HL_PLANNER_ACTION_H
#define HL_PLANNER_ACTION_H

#include <geometry_msgs/Pose.h>

class SearchAction{
  public:
    enum Type{ SEARCH, QUICK_SEARCH };

    Type type_;
    int target_;

    SearchAction(Type type = Type::SEARCH, int target = 0) : type_(type), target_(target){}
};

class Action{
  public:
    enum Type{ MOVE_TO, EXPLORE, SEARCH, QUICK_SEARCH };

    Type type_;
    int target_;
    geometry_msgs::Pose pose_;

    Action(Type type, int target, const geometry_msgs::Pose& pose = geometry_msgs::Pose()) : type_(type), target_(target), pose_(pose) {}

    bool operator==(const Action& rhs){
      return type_ == rhs.type_ && target_ == rhs.target_ && pose_.position.x == rhs.pose_.position.x && pose_.position.y == rhs.pose_.position.y && pose_.orientation.w == pose_.orientation.w;
    }
};

#endif //HL_PLANNER_ACTION_H
