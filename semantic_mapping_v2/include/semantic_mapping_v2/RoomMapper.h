//
// Created by thomas on 19.11.17.
//

#ifndef SEMANTIC_MAPPING_V2_ROOMMAPPER_H
#define SEMANTIC_MAPPING_V2_ROOMMAPPER_H

#include "semantic_mapping_v2/SlamGMapping.h"
#include "semantic_mapping_v2/OctoMapper.h"
#include "semantic_mapping_v2/DoorMapper.h"
#include "semantic_mapping_v2/RoomTypeMapper.h"
#include "semantic_mapping_v2/ObjectMapper.h"

#include <semantic_mapping_v2/HierarchyLinkMsg.h>

class RoomMapper : public SlamGMapping{
  public:
    static double getObjProbGivenRoom(int obj, int room);
    static std::vector<std::vector<double>> prob_map;
    static double getObjProbGivenRoomPrior(int obj);

  protected:
    int idx_ = -1;

    std::vector<OctoMapper*> octo_maps_;
    std::vector<DoorMapper*> door_mappers_;
    std::vector<ObjectMapper*> obj_mappers_;
    std::vector<RoomTypeMapper*> room_type_mappers_;
    boost::mutex maps_mutex_;

    nav_msgs::OccupancyGrid obstacle_map_;
    boost::mutex obstacle_map_mutex_;
    bool was_map_updated_ = false;

    tf::StampedTransform camera_to_base_transform_;
    tf::StampedTransform base_to_laser_transform_;

    double downsample_voxel_size_ = 0.03;
    double m_pointcloudMinZ = 0.25;
    double m_pointcloudMaxZ = 1.8;
    double ROOM_HIT_MISS_RATIO = 1000.0;
    double OBJ_HIT_MISS_RATIO = 100.0;

    boost::thread* octomapping_thread_ = nullptr;
    sensor_msgs::PointCloud2 latest_cloud_;
    boost::mutex latest_cloud_mutex_;

  public:
    RoomMapper(int idx, tf::TransformListener* tf, GMapping::OrientedPoint initial_pose, const tf::Transform& initial_map_to_odom, const Door& door = Door());
    ~RoomMapper();

    virtual void cloudCb(const sensor_msgs::PointCloud2::ConstPtr& cloud);
    void doorCb(const geometry_msgs::PoseArray::ConstPtr& msg);
    void visionCb(const vision::VisionMsgConstPtr& msg);

    void doOctomapping();

    int getBestParticleIdx() const { return std::min(gsp_->getBestParticleIndex(), int(octo_maps_.size())-1); }
    GMapping::OrientedPoint getParticlePose2D(int particle_idx, ros::Time time) const;
    tf::Transform getParticlePose3D(int particle_idx, ros::Time time) const;
    GMapping::OrientedPoint getBestParticlePose2D(ros::Time time) const;
    tf::Transform getBestParticlePose3D(ros::Time time) const;

    bool wasMapUpdated() const { return was_map_updated_; }
    bool resetWasMapUpdated();

    void activate();
    tf::Transform activate(const GMapping::OrientedPoint& robot, const Door& door);
    void deactivate();
    void downprojectMap();

    std::vector<Door> getDoors() const { return door_mappers_[getBestParticleIdx()]->getDoors(); }
    void setDoorRoom(int id, int other_room, int counterpart_id);
    geometry_msgs::PoseArray getDoorPoseMsg() const { return door_mappers_[getBestParticleIdx()]->getDoorPoseMsg(); }
    Door droveThroughDoor() const { return door_mappers_[getBestParticleIdx()]->droveThroughDoor(getBestParticlePose3D(ros::Time::now())); }

    nav_msgs::OccupancyGrid getMap() {
      boost::mutex::scoped_lock lock(obstacle_map_mutex_);
      return obstacle_map_;
    }
    nav_msgs::OccupancyGrid getDoorBlockedMap();

    octomap_msgs::Octomap getBinaryOctoMapMsg(const ros::Time& rostime = ros::Time::now()) const {
      return octo_maps_[getBestParticleIdx()]->getBinaryOctoMapMsg(rostime);
    }
    octomap_msgs::Octomap getFullOctoMapMsg(const ros::Time& rostime = ros::Time::now()){
      boost::lock_guard<boost::mutex> maps_lock(maps_mutex_);
      return octo_maps_[getBestParticleIdx()]->getFullOctoMapMsg(rostime);
    }
    visualization_msgs::MarkerArray getOccupiedCellMsg(const ros::Time& rostime = ros::Time::now()) const{
      return octo_maps_[getBestParticleIdx()]->getOccupiedCellMsg(rostime);
    }
    visualization_msgs::MarkerArray getOccupiedAndFreeCellMsg(visualization_msgs::MarkerArray& free_cells, const ros::Time& rostime = ros::Time::now()) const{
      return octo_maps_[getBestParticleIdx()]->getOccupiedAndFreeCellMsg(free_cells, rostime);
    }
    void getAllPublishMsgs(visualization_msgs::MarkerArray& occupied_cells_vis_array, octomap_msgs::Octomap& octomap_binary,
                           octomap_msgs::Octomap& octomap_full, visualization_msgs::MarkerArray& free_cells_vis_array,
                           const ros::Time& rostime = ros::Time::now()){
      octo_maps_[getBestParticleIdx()]->getAllPublishMsgs(occupied_cells_vis_array, octomap_binary, octomap_full, free_cells_vis_array, rostime);
    }
    float getOccupancy(float x, float y, float z) const{
      return octo_maps_[getBestParticleIdx()]->getOccupancy(x,y,z);
    }
    float getOccupancy(const pcl::PointXYZ& pos) const { return getOccupancy(pos.x, pos.y, pos.z); }

    visualization_msgs::MarkerArray getObjectProbMsg(int id) const;
    visualization_msgs::MarkerArray getRoomProbMsg(int id);
    geometry_msgs::PoseArray getParticlePoseMsg() const;

    semantic_mapping_v2::ObjectMapMsg getObjMapMsg(int obj_id) {
      boost::lock_guard<boost::mutex> maps_lock(maps_mutex_);
      return obj_mappers_[getBestParticleIdx()]->getObjMapMsg(obj_id);
    }
    std::vector<semantic_mapping_v2::ObjectMapMsg> getAllObjMapMsgs() {
      boost::lock_guard<boost::mutex> maps_lock(maps_mutex_);
      return obj_mappers_[getBestParticleIdx()]->getAllObjMapMsgs();
    }

    semantic_mapping_v2::RoomTypeMapMsg getRoomTypeMapMsg(int obj_id) {
      boost::lock_guard<boost::mutex> maps_lock(maps_mutex_);
      return room_type_mappers_[getBestParticleIdx()]->getRoomTypeMapMsg(obj_id);
    }
    std::vector<semantic_mapping_v2::RoomTypeMapMsg> getAllRoomTypeMapMsgs() {
      boost::lock_guard<boost::mutex> maps_lock(maps_mutex_);
      return room_type_mappers_[getBestParticleIdx()]->getAllRoomTypeMapMsgs();
    }

    std::vector<float> getObjectProbs(std::vector<size_t>& order) {
      boost::lock_guard<boost::mutex> maps_lock(maps_mutex_);
      return obj_mappers_[getBestParticleIdx()]->getObjectProbs(*octo_maps_[getBestParticleIdx()], door_mappers_[getBestParticleIdx()]->getDoors(), order);
    }
    std::vector<float> getRoomTypeProbs(std::vector<size_t>& order) {
      boost::lock_guard<boost::mutex> maps_lock(maps_mutex_);
      return room_type_mappers_[getBestParticleIdx()]->getRoomProb(getMap(), door_mappers_[getBestParticleIdx()]->getDoors(), order);
    }
    std::vector<float> getObjectProbsComplete(std::vector<float>& room_probs, std::vector<size_t>& order);

    std::string getRoomName(int id) const { return room_type_mappers_[0]->getName(id); }
    std::vector<std::string> getRoomNames() const { return room_type_mappers_[0]->getNames(); }
};


#endif //SEMANTIC_MAPPING_V2_ROOMMAPPER_H
