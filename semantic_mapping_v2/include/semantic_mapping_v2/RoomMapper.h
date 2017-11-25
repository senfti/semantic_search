//
// Created by thomas on 19.11.17.
//

#ifndef SEMANTIC_MAPPING_V2_ROOMMAPPER_H
#define SEMANTIC_MAPPING_V2_ROOMMAPPER_H

#include "semantic_mapping_v2/SlamGMapping.h"
#include "semantic_mapping_v2/OctoMapper.h"
#include "semantic_mapping_v2/DoorMapper.h"
#include "semantic_mapping_v2/RoomTypeMapper.h"

class RoomMapper : public SlamGMapping{
  protected:
    int idx_ = -1;

    std::vector<OctoMapper*> octo_maps_;
    std::vector<DoorMapper> door_mappers_;
    RoomTypeMapper room_type_mapper_;

    nav_msgs::OccupancyGrid obstacle_map_;
    boost::mutex obstacle_map_mutex_;
    bool was_map_updated_ = false;

    tf::StampedTransform camera_to_base_transform_;

    double downsample_voxel_size_ = 0.03;
    double m_pointcloudMinZ = -std::numeric_limits<double>::max();
    double m_pointcloudMaxZ = std::numeric_limits<double>::max();

    ros::Time activate_time_ = ros::TIME_MAX;
    double octomap_wait_time_ = 2.0;

  public:
    RoomMapper(int idx, const Door& door = Door());
    ~RoomMapper();

    virtual void cloudCb(const sensor_msgs::PointCloud2::ConstPtr& cloud);
    void doorCb(const geometry_msgs::PoseArray::ConstPtr& msg);
    void visionCb(const vision::VisionMsgConstPtr& msg);

    int getBestParticleIdx() const { return gsp_->getBestParticleIndex(); }
    GMapping::OrientedPoint getParticlePose2D(int particle_idx, ros::Time time) const;
    tf::Transform getParticlePose3D(int particle_idx, ros::Time time) const;
    GMapping::OrientedPoint getBestParticlePose2D(ros::Time time) const { return getParticlePose2D(getBestParticleIdx(), time); }
    tf::Transform getBestParticlePose3D(ros::Time time) const { return getParticlePose3D(getBestParticleIdx(), time); }

    bool wasMapUpdated() const { return was_map_updated_; }
    bool resetWasMapUpdated();

    void activate();
    void deactivate();
    void downprojectMap();

    std::vector<Door> getDoors() const { return door_mappers_[getBestParticleIdx()].getDoors(); }
    void setDoorRoom(const tf::Transform& pose, int other_room);
    geometry_msgs::PoseArray getDoorPoseMsg() const { return door_mappers_[getBestParticleIdx()].getDoorPoseMsg(); }
    Door droveThroughDoor() const { return door_mappers_[getBestParticleIdx()].droveThroughDoor(getBestParticlePose3D(ros::Time::now())); }

    nav_msgs::OccupancyGrid getMap() {
      boost::mutex::scoped_lock lock(obstacle_map_mutex_);
      return obstacle_map_;
    }
    octomap_msgs::Octomap getBinaryOctoMapMsg(const ros::Time& rostime = ros::Time::now()) const {
      return octo_maps_[getBestParticleIdx()]->getBinaryOctoMapMsg(rostime);
    }
    octomap_msgs::Octomap getFullOctoMapMsg(const ros::Time& rostime = ros::Time::now()) const{
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
};


#endif //SEMANTIC_MAPPING_V2_ROOMMAPPER_H
