//
// Created by thomas on 19.11.17.
//

#ifndef SEMANTIC_MAPPING_V2_ROOMMAPPER_H
#define SEMANTIC_MAPPING_V2_ROOMMAPPER_H

#include "semantic_mapping_v2/SlamGMapping.h"
#include "semantic_mapping_v2/OctoMapper.h"

class RoomMapper : public SlamGMapping{
  protected:
    std::vector<OctoMapper*> octo_maps_;
    nav_msgs::OccupancyGrid obstacle_map_;
    bool octomaps_started_ = false;

    tf::StampedTransform camera_to_base_transform_;

    double downsample_voxel_size_ = 0.03;
    double m_pointcloudMinZ = -std::numeric_limits<double>::max();
    double m_pointcloudMaxZ = std::numeric_limits<double>::max();

  public:
    RoomMapper();

    virtual void cloudCb(const sensor_msgs::PointCloud2::ConstPtr& cloud);

    int getBestParticleIdx() const { return gsp_->getBestParticleIndex(); }

    nav_msgs::OccupancyGrid getMap() { return obstacle_map_; }
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
