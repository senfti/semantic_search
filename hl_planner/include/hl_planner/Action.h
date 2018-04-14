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
    enum Type{ MOVE_TO, EXPLORE, SEARCH, QUICK_SEARCH, ROTATE, START_ROTATE };

    Type type_;
    int target_obj_;
    int target_room_;
    geometry_msgs::Pose pose_;

    Action(Type type, int target_obj, int target_room, const geometry_msgs::Pose& pose = geometry_msgs::Pose())
      : type_(type), target_obj_(target_obj), target_room_(target_room), pose_(pose) {}

    bool operator==(const Action& rhs){
      return type_ == rhs.type_ && target_obj_ == rhs.target_obj_ && target_room_ == rhs.target_room_
             && pose_.position.x == rhs.pose_.position.x && pose_.position.y == rhs.pose_.position.y && pose_.orientation.w == pose_.orientation.w;
    }

    std::string getActionString() const {
      std::string s;
      switch(type_){
        case MOVE_TO:           s = "MOVE_TO ";       break;
        case EXPLORE:           s = "EXPLORE ";       break;
        case SEARCH:            s = "SEARCH ";        break;
        case QUICK_SEARCH:      s = "QUICK_SEARCH ";  break;
        case ROTATE:            s = "ROTATE ";        break;
        case START_ROTATE:      s = "START_ROTATE ";  break;
      }
      s += "OBJ " + std::to_string(target_obj_) + " ROOM " + std::to_string(target_room_);
      return s;
    }
};

#endif //HL_PLANNER_ACTION_H
