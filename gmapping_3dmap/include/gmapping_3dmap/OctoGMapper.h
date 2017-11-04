//
// Created by thomas on 02.11.17.
//

#ifndef GMAPPING_3DMAP_OCTOMAPMAPPER_H
#define GMAPPING_3DMAP_OCTOMAPMAPPER_H

#include "gmapping_3dmap/OctoMapper.h"
#include "gmapping_3dmap/slam_gmapping.h"

#include <map>
#include <list>

class OctoGMapper : public SlamGMapping{
  protected:
    std::list<sensor_msgs::PointCloud2> cloud_list_;
    std::vector<OctoMapper*> octo_maps_;
    std::vector<GMapping::GridSlamProcessor::TNode*> associated_nodes_;
    bool octomaps_started_ = false;

    tf::StampedTransform camera_to_base_transform_;

    ros::Publisher marker_pub_;
    ros::Publisher binary_map_pub_;
    ros::Publisher full_map_pub_;
    ros::Publisher point_cloud_pub_;
    ros::Publisher map_pub_;
    ros::Publisher fmarker_pub_;
    ros::Subscriber cloud_sub_;

    double downsample_voxel_size_ = 0.03;
    double m_pointcloudMinX = -std::numeric_limits<double>::max();
    double m_pointcloudMaxX = std::numeric_limits<double>::max();
    double m_pointcloudMinY = -std::numeric_limits<double>::max();
    double m_pointcloudMaxY = std::numeric_limits<double>::max();
    double m_pointcloudMinZ = -std::numeric_limits<double>::max();
    double m_pointcloudMaxZ = std::numeric_limits<double>::max();

    virtual void updateOctoMaps();
    void publishMessages();

  public:
    OctoGMapper();

    virtual void cloudCb(const sensor_msgs::PointCloud2::ConstPtr& cloud);
    int getIdxFromNodePtr(GMapping::GridSlamProcessor::TNode* ptr) const;
};

#endif //GMAPPING_3DMAP_OCTOMAPMAPPER_H
