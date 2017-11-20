//
// Created by thomas on 20.11.17.
//

#include "semantic_mapping_v2/DoorMapper.h"
#include <tf/transform_datatypes.h>

void DoorMapper::addDoorProposal(const tf::Transform &pose){
  for(int i=0; i<door_poses_.size(); i++){
    if((pose.getOrigin() - door_poses_[i].getOrigin()).length() < MIN_DOOR_DIST){
      door_poses_[i].setOrigin((door_poses_[i].getOrigin()*pose_proposal_count_[i] + pose.getOrigin()) / (pose_proposal_count_[i]+1));
      door_poses_[i].setRotation(door_poses_[i].getRotation().slerp(pose.getRotation(), pose_proposal_count_[i]/double(pose_proposal_count_[i]+1)));
      pose_proposal_count_[i]++;
      return;
    }
  }

  door_poses_.push_back(pose);
  pose_proposal_count_.push_back(1);
}


geometry_msgs::PoseArray DoorMapper::getDoorPoseMsg() const{
  static int seq = 0;
  geometry_msgs::PoseArray msg;
  msg.header.seq = seq++;
  msg.header.frame_id = "map";
  msg.header.stamp = ros::Time::now();
  msg.poses.resize(door_poses_.size());
  for(int i=0; i<door_poses_.size(); i++){
    tf::poseTFToMsg(door_poses_[i], msg.poses[i]);
  }

  return msg;
}
