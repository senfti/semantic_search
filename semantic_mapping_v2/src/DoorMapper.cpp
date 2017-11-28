//
// Created by thomas on 20.11.17.
//

#include "semantic_mapping_v2/DoorMapper.h"
#include <tf/transform_datatypes.h>


DoorMapper::DoorMapper(int this_room, const Door &door)
      : this_room_(this_room)
{
  if(door.isValid() && door.this_room_ == this_room_)
    addDoor(door);
}


bool DoorMapper::isDoorNearPose(const tf::Transform& pose) const{
  for(int i=0; i<doors_.size(); i++){
    if((pose.getOrigin() - doors_[i].pose_.getOrigin()).length() < MIN_DOOR_DIST){
      return true;
    }
  }
  return false;
}


bool DoorMapper::addDoor(const tf::Transform &pose, int proposal_count, int other_room){
  if(isDoorNearPose(pose))
    return false;

  doors_.push_back(Door(this_room_, pose, proposal_count, other_room));
  return true;
}


bool DoorMapper::addDoor(const Door& door){
  if(isDoorNearPose(door.pose_))
    return false;

  doors_.push_back(door);
  return true;
}


bool DoorMapper::setDoorRoom(const tf::Transform &pose, int other_room){
  for(int i=0; i<doors_.size(); i++){
    if((pose.getOrigin() - doors_[i].pose_.getOrigin()).length() < MIN_DOOR_DIST){
      doors_[i].other_room_ = other_room;
      return true;
    }
  }

  return false;
}


bool DoorMapper::addDoorProposal(const tf::Transform &pose){
  for(int i=0; i<doors_.size(); i++){
    if((pose.getOrigin() - doors_[i].pose_.getOrigin()).length() < MIN_DOOR_DIST){
      doors_[i].pose_.setOrigin((doors_[i].pose_.getOrigin()*doors_[i].proposal_count_ + pose.getOrigin()) / (doors_[i].proposal_count_+1));
      doors_[i].pose_.setRotation(doors_[i].pose_.getRotation().slerp(pose.getRotation(), doors_[i].proposal_count_/double(doors_[i].proposal_count_+1)));
      doors_[i].proposal_count_++;
      return true;
    }
  }

  doors_.push_back(Door(this_room_, pose));
  return false;
}


Door DoorMapper::droveThroughDoor(const tf::Transform &robot_pose) const{
  for(int i=0; i<doors_.size(); i++){
    tf::Vector3 diff = robot_pose.getOrigin() - doors_[i].pose_.getOrigin();
    if(diff.length() < MIN_DOOR_DIST && doors_[i].pose_.getBasis().getColumn(0).dot(diff) > 0 && doors_[i].pose_.getBasis().getColumn(0).dot(robot_pose.getBasis().getColumn(0)) > 0){
      return doors_[i];
    }
  }

  return Door();
}


geometry_msgs::PoseArray DoorMapper::getDoorPoseMsg() const{
  static int seq = 0;
  geometry_msgs::PoseArray msg;
  msg.header.seq = seq++;
  msg.header.frame_id = "map";
  msg.header.stamp = ros::Time::now();
  msg.poses.resize(doors_.size());
  for(int i=0; i<doors_.size(); i++){
    tf::poseTFToMsg(doors_[i].pose_, msg.poses[i]);
  }

  return msg;
}
