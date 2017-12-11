//
// Created by thomas on 20.11.17.
//

#ifndef SEMANTIC_MAPPING_V2_DOORMAPPER_H
#define SEMANTIC_MAPPING_V2_DOORMAPPER_H

#include <tf/tf.h>
#include <geometry_msgs/PoseArray.h>
#include "gmapping/gridfastslam/gridslamprocessor.h"

class RoomMapper;

class Door{
  public:
    int id_ = -1;
    int counterpart_id_ = -1;
    tf::Transform pose_;
    int proposal_count_ = 0;
    int this_room_ = -1;
    int other_room_ = -1;

    Door() {}
    Door(int this_room, tf::Transform pose, int proposal_count = 1, int other_room = -1, int id = getID(), int counterpart_id = -1)
          : id_(id), counterpart_id_(counterpart_id), pose_(pose), proposal_count_(proposal_count), this_room_(this_room), other_room_(other_room) {}

    bool isValid() const { return id_ >= 0; }
    bool hasOtherRoom() const { return other_room_ >= 0; }

    GMapping::OrientedPoint getPose2D() const { return GMapping::OrientedPoint(pose_.getOrigin().x(), pose_.getOrigin().y(), tf::getYaw(pose_.getRotation())); }
    bool isBehindDoor(float x, float y) const;

    static int getID() { static int id=0; return id++; }
};

class DoorMapper{
  public:
    double MIN_DOOR_DIST = 1.0;
    int MAX_CONFIDENCE = 10;

  private:
    std::vector<Door> doors_;
    int this_room_ = -1;

  public:
    DoorMapper(int this_room, const Door& door = Door());

    Door isDoorNearPose(const tf::Transform& pose) const;
    bool addDoor(const tf::Transform& pose, int proposal_count = 1, int other_room = -1, int counterpart_id = -1);
    bool addDoor(const Door& door);
    bool setDoorRoom(const tf::Transform& pose, int other_room = -1, int counterpart_id = -1);
    bool addDoorProposal(const tf::Transform& pose, int new_id);

    Door droveThroughDoor(const tf::Transform &robot_pose) const;

    std::vector<Door> getDoors() const { return doors_; }
    Door getDoor(int id) const;
    geometry_msgs::PoseArray getDoorPoseMsg() const;
};

#endif //SEMANTIC_MAPPING_V2_DOORMAPPER_H
