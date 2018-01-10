//
// Created by thomas on 20.11.17.
//

#ifndef SEMANTIC_MAPPING_V2_DOORMAPPER_H
#define SEMANTIC_MAPPING_V2_DOORMAPPER_H

#include <tf/tf.h>
#include <geometry_msgs/PoseArray.h>
#include "gmapping/gridfastslam/gridslamprocessor.h"
#include <boost/thread.hpp>

class RoomMapper;

class Door{
  public:
    static double MAX_SAME_DOOR_DIST;
    static double MAX_SAME_DOOR_ANGLE;
    static double MIN_THROUGH_DOOR_DIST;
    static int MAX_CONFIDENCE;

  private:
    int id_ = -1;
    int counterpart_id_ = -1;
    tf::Transform pose_;
    int this_room_ = -1;
    int other_room_ = -1;
    std::vector<tf::Transform> pose_array_;
    int array_pos_ = 0;

  public:
    Door() {}
    Door(int this_room, const tf::Transform& pose, int other_room, int id, int counterpart_id, int confidence);

    bool isValid() const { return id_ >= 0; }
    bool hasOtherRoom() const { return other_room_ >= 0; }

    GMapping::OrientedPoint getPose2D() const { return GMapping::OrientedPoint(pose_.getOrigin().x(), pose_.getOrigin().y(), tf::getYaw(pose_.getRotation())); }
    bool isBehindDoor(float x, float y) const;

    double getIsDoorConfidence(const tf::Transform &pose) const;

    int getId() const { return id_; }
    int getCounterpartId() const { return counterpart_id_; }
    tf::Transform getPose() const { return pose_; }
    int getRoom() const { return this_room_; }
    int getOtherRoom() const { return other_room_; }
    bool didDriveThrough(const tf::Transform &robot_pose) const;

    void setCounterpart(int counterpart_id, int other_room) { counterpart_id_ = counterpart_id; other_room_ = other_room; }
    void flipPose();
    void updatePose(const tf::Transform &pose);

    static int getID() { static int id=0; return id++; }
};

class DoorMapper{
  public:

  private:
    std::vector<Door> doors_;
    boost::mutex doors_mutex_;
    int this_room_ = -1;

  public:
    DoorMapper(int this_room, const Door& door = Door());
    DoorMapper(const DoorMapper& rhs);

    int isDoorNearPose(const tf::Transform& pose) const;
    bool addDoor(const tf::Transform& pose, int other_room = -1, int counterpart_id = -1);
    bool addDoor(const Door& door);
    bool setDoorRoom(int id, int other_room = -1, int counterpart_id = -1);
    bool addDoorProposal(const tf::Transform& pose, int new_id);

    Door droveThroughDoor(const tf::Transform &robot_pose) const;

    std::vector<Door> getDoors() {
      boost::lock_guard<boost::mutex> lock(doors_mutex_);
      return doors_;
    }
    Door getDoor(int id) const;
    geometry_msgs::PoseArray getDoorPoseMsg() const;
};

#endif //SEMANTIC_MAPPING_V2_DOORMAPPER_H
