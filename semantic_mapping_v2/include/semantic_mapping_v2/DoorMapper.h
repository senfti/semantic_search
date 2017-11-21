//
// Created by thomas on 20.11.17.
//

#ifndef SEMANTIC_MAPPING_V2_DOORMAPPER_H
#define SEMANTIC_MAPPING_V2_DOORMAPPER_H

#include <tf/tf.h>
#include <geometry_msgs/PoseArray.h>

class RoomMapper;

class Door{
  public:
    tf::Transform pose_;
    int proposal_count_ = 0;
    int this_room_ = -1;
    int other_room_ = -1;

    Door() {}
    Door(int this_room, tf::Transform pose, int proposal_count = 1, int other_room = -1)
          : pose_(pose), proposal_count_(proposal_count), this_room_(this_room), other_room_(other_room) {}

    bool isValid() const { return this_room_ >= 0; }
    bool hasOtherRoom() const { return other_room_ >= 0; }
};

class DoorMapper{
  public:
    double MIN_DOOR_DIST = 1.0;

  private:
    std::vector<Door> doors_;
    int this_room_ = -1;

  public:
    DoorMapper(int this_room, const Door& door = Door());

    bool isDoorNearPose(const tf::Transform& pose) const;
    bool addDoor(const tf::Transform& pose, int proposal_count = 1, int other_room = -1);
    bool addDoor(const Door& door);
    bool setDoorRoom(const tf::Transform& pose, int other_room = -1);
    bool addDoorProposal(const tf::Transform& pose);

    Door droveThroughDoor(const tf::Transform &robot_pose) const;

    std::vector<Door> getDoors() const { return doors_; }
    geometry_msgs::PoseArray getDoorPoseMsg() const;
};

#endif //SEMANTIC_MAPPING_V2_DOORMAPPER_H
