//
// Created by thomas on 15.11.17.
//

#ifndef SEMANTIC_MAPPING_V2_RoomMapper_H
#define SEMANTIC_MAPPING_V2_RoomMapper_H

#include "semantic_mapping_v2/OctoMapper.h"
#include "semantic_mapping_v2/SlamGMapping.h"
#include "semantic_mapping_v2/ObjectMapper.h"
#include "vision/VisionMsg.h"

#include <map>
#include <list>

class RoomMapper : public SlamGMapping{
  protected:
    std::vector<OctoMapper*> octo_maps_;
    bool octomaps_started_ = false;

    std::list<vision::VisionMsgConstPtr> vision_msgs_;
    ObjectMap object_map_;
    std::vector<float> room_probs_;

    tf::StampedTransform camera_to_base_transform_;

    ros::Publisher marker_pub_;
    ros::Publisher binary_map_pub_;
    ros::Publisher full_map_pub_;
    ros::Publisher point_cloud_pub_;
    ros::Publisher map_pub_;
    ros::Publisher fmarker_pub_;
    ros::Subscriber cloud_sub_;
    ros::Subscriber vision_sub_;

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
    RoomMapper();

    void enableObjectMapping(bool enable = true);
    void enableOctoMapping(bool enable = true);
    void startSlam(bool enable = true);
    void enable();
    void disable();

    virtual void cloudCb(const sensor_msgs::PointCloud2::ConstPtr& cloud);
    virtual void visionCb(const vision::VisionMsgConstPtr& msg);

    nav_msgs::OccupancyGrid getMap();
    OctoMapper* getMap();

};

#endif //SEMANTIC_MAPPING_V2_RoomMapper_H
