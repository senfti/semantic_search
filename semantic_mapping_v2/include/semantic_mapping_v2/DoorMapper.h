//
// Created by thomas on 20.11.17.
//

#ifndef SEMANTIC_MAPPING_V2_DOORMAPPER_H
#define SEMANTIC_MAPPING_V2_DOORMAPPER_H

#include <tf/tf.h>
#include <geometry_msgs/PoseArray.h>

class DoorMapper{
  public:
    double MIN_DOOR_DIST = 1.0;

  private:
    std::vector<tf::Transform> door_poses_;
    std::vector<int> pose_proposal_count_;

  public:
    void addDoorProposal(const tf::Transform& pose);
    std::vector<tf::Transform> getDoorPoses() const { return door_poses_; }
    geometry_msgs::PoseArray getDoorPoseMsg() const;
};

#endif //SEMANTIC_MAPPING_V2_DOORMAPPER_H
