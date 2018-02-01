//
// Created by thomas on 20.11.17.
//

#include "semantic_mapping_v2/DoorMapper.h"
#include <tf/transform_datatypes.h>

double Door::MAX_SAME_DOOR_DIST = 1.0;
double Door::MAX_SAME_DOOR_ANGLE = M_PI_2;
double Door::MIN_THROUGH_DOOR_DIST = 0.1;
int Door::MAX_CONFIDENCE = 10;

Door::Door(int this_room, const tf::Transform& pose, int other_room, int id, int counterpart_id, int confidence)
  : id_(id), counterpart_id_(counterpart_id), pose_(pose), this_room_(this_room), other_room_(other_room), pose_array_(std::min(MAX_CONFIDENCE, confidence), pose)
{
}


bool Door::isInBehindDoorRect(float x, float y) const{
  tf::Vector3 pos = pose_.inverse()*tf::Vector3(x,y,0.f);
  return (pos.x() > 0.0 && pos.x() < 1.0 && pos.y() > -0.7 && pos.y() < 0.7);

}

bool Door::isBehindDoor(float x, float y) const{
  tf::Vector3 pos = pose_.inverse()*tf::Vector3(x,y,0.f);
  return ((std::abs(pos.y())-0.7) / pos.x() < 0.8 && pos.x() > 0.1);
}

double Door::getIsDoorConfidence(const tf::Transform &pose) const{
  double dist = (pose_.getOrigin()-pose.getOrigin()).length();
  double angle = pose_.getRotation().angleShortestPath(pose.getRotation());
  return (MAX_SAME_DOOR_DIST-dist)/MAX_SAME_DOOR_DIST * std::max(MAX_SAME_DOOR_ANGLE-angle/MAX_SAME_DOOR_ANGLE, 0.001);
}

bool Door::didDriveThrough(const tf::Transform &robot_pose) const{
  tf::Transform robot_door = pose_.inverse()*robot_pose;
  return (robot_door.getOrigin().length() < MAX_SAME_DOOR_DIST && robot_door.getOrigin().x() > MIN_THROUGH_DOOR_DIST && robot_door.getBasis().getColumn(0).x() > 0.5);
}

void Door::updatePose(const tf::Transform &pose){
  if(pose_array_.size() < MAX_CONFIDENCE)
    pose_array_.push_back(pose);
  else
    pose_array_[array_pos_] = pose;

  array_pos_ = (array_pos_+1)%MAX_CONFIDENCE;

  tf::Vector3 pos(0.0,0.0,0.0);
  for(const auto& p : pose_array_)
    pos += p.getOrigin();
  pose_.setOrigin(pos/double(pose_array_.size()));

  pos = tf::Vector3(0.0,0.0,0.);
  for(const auto& p : pose_array_){
    pos += p.getBasis().getColumn(0);
  }
  pose_.setRotation(tf::createQuaternionFromYaw(std::atan2(pos.y(), pos.x())));
}

void Door::flipPose(){
  pose_.setRotation(tf::Quaternion(tf::Vector3(0,0,1), tf::getYaw(pose_.getRotation()) + M_PI));
  for(auto& p : pose_array_)
    p.setRotation(tf::Quaternion(tf::Vector3(0,0,1), tf::getYaw(p.getRotation()) + M_PI));
}


DoorMapper::DoorMapper(int this_room, const Door &door)
      : this_room_(this_room)
{
  if(door.isValid() && door.getRoom() == this_room_)
    addDoor(door);
}


DoorMapper::DoorMapper(const DoorMapper &rhs){
  doors_ = rhs.doors_;
  this_room_ = rhs.this_room_;
}


int DoorMapper::isDoorNearPose(const tf::Transform& pose) const{
  double best_confidence = 0.0;
  int best_door = -1;
  for(int i=0; i<doors_.size(); i++){
    double confidence = doors_[i].getIsDoorConfidence(pose);
    if(confidence > best_confidence){
      best_door = i;
      best_confidence = confidence;
    }
  }
  return best_door;
}


bool DoorMapper::addDoor(const tf::Transform &pose, int other_room, int counterpart_id){
  if(isDoorNearPose(pose) >= 0)
    return false;

  boost::lock_guard<boost::mutex> lock(doors_mutex_);
  doors_.push_back(Door(this_room_, pose, other_room, Door::getID(), counterpart_id, 1));
  return true;
}


bool DoorMapper::addDoor(const Door& door){
  if(isDoorNearPose(door.getPose()) >= 0)
    return false;

  boost::lock_guard<boost::mutex> lock(doors_mutex_);
  doors_.push_back(door);
  return true;
}


bool DoorMapper::setDoorRoom(int id, int other_room, int counterpart_id){
  for(int i=0; i<doors_.size(); i++){
    if(doors_[i].getId() == id){
      boost::lock_guard<boost::mutex> lock(doors_mutex_);
      doors_[i].setCounterpart(counterpart_id, other_room);
      return true;
    }
  }

  return false;
}


bool DoorMapper::addDoorProposal(const tf::Transform &pose, int new_id){
  int best_door = isDoorNearPose(pose);
  if(best_door >= 0){
    boost::lock_guard<boost::mutex> lock(doors_mutex_);
    doors_[best_door].updatePose(pose);
    return false;
  }

  boost::lock_guard<boost::mutex> lock(doors_mutex_);
  doors_.push_back(Door(this_room_, pose, -1, new_id, -1, 1));
  return true;
}


Door DoorMapper::droveThroughDoor(const tf::Transform &robot_pose) const{
  for(int i=0; i<doors_.size(); i++){
    if(doors_[i].didDriveThrough(robot_pose)){
      return doors_[i];
    }
  }

  return Door();
}


Door DoorMapper::getDoor(int id) const{
  for(int i=0; i<doors_.size(); i++){
    if(doors_[i].getId() == id)
      return doors_[i];
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
    tf::poseTFToMsg(doors_[i].getPose(), msg.poses[i]);
  }

  return msg;
}
