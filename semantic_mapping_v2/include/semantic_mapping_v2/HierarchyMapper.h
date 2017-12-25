//
// Created by thomas on 19.11.17.
//

#ifndef SEMANTIC_MAPPING_V2_HIERARCHYMAPPER_H
#define SEMANTIC_MAPPING_V2_HIERARCHYMAPPER_H

#include "semantic_mapping_v2/RoomMapper.h"

#include <semantic_mapping_v2/MapSrv.h>
#include <semantic_mapping_v2/OctomapSrv.h>
#include <semantic_mapping_v2/HierarchySrv.h>
#include <semantic_mapping_v2/ObjectMapSrv.h>
#include <semantic_mapping_v2/ObjectProbSrv.h>
#include <semantic_mapping_v2/RoomTypeMapSrv.h>
#include <semantic_mapping_v2/RoomTypeProbSrv.h>

class HierarchyMapper{
  protected:
    std::vector<RoomMapper*> room_mapper_;
    int current_mapper_ = -1;
    ros::Time last_map_switch_time_;

    ros::Subscriber laser_sub_;
    ros::Subscriber cloud_sub_;
    ros::Subscriber door_pose_sub_;
    ros::Subscriber vision_sub_;

    ros::Publisher map_pub_;
    ros::Publisher gmap_pub_;
    ros::Publisher map_door_blocked_pub_;
    ros::Publisher map_switch_pub_;
    ros::Publisher map_info_pub_;
    ros::Publisher marker_pub_;
    ros::Publisher door_pose_pub_;
    std::vector<ros::Publisher> obj_prob_pub_;
    std::vector<ros::Publisher> room_prob_pub_;
    ros::Publisher particle_pose_pub_;

    ros::ServiceServer gmap_srv_;
    ros::ServiceServer map_srv_;
    ros::ServiceServer map_door_blocked_srv_;
    ros::ServiceServer octomap_srv_;
    ros::ServiceServer obj_map_srv_;
    ros::ServiceServer room_type_map_srv_;
    ros::ServiceServer obj_prob_srv_;
    ros::ServiceServer room_type_prob_srv_;
    ros::ServiceServer hierarchy_srv_;

    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster* tfB_;
    ros::NodeHandle nh_;
    boost::thread* transform_thread_;

    void transformPublishLoop(double transform_publish_period);

    double transform_publish_period_;
    double publish_period_;
    int debug_publish_interval_ = std::numeric_limits<int>::max();
    double MIN_MAP_SWITCH_TIME = 2.0;

  public:
    HierarchyMapper();
    ~HierarchyMapper();

    void addMapper(const Door& door = Door());
    void switchMapper(int mapper_idx, const Door& door = Door());

    void cloudCb(const sensor_msgs::PointCloud2::ConstPtr& cloud);
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    void doorPoseCb(const geometry_msgs::PoseArray::ConstPtr& msg);
    void visionCb(const vision::VisionMsgConstPtr& msg);

    bool gmapSrvCb(semantic_mapping_v2::MapSrv::Request& req, semantic_mapping_v2::MapSrv::Response& res);
    bool mapSrvCb(semantic_mapping_v2::MapSrv::Request& req, semantic_mapping_v2::MapSrv::Response& res);
    bool mapDoorBlockedSrvCb(semantic_mapping_v2::MapSrv::Request& req, semantic_mapping_v2::MapSrv::Response& res);
    bool octomapSrvCb(semantic_mapping_v2::OctomapSrv::Request& req, semantic_mapping_v2::OctomapSrv::Response& res);
    bool roomTypeMapSrvCb(semantic_mapping_v2::RoomTypeMapSrv::Request& req, semantic_mapping_v2::RoomTypeMapSrv::Response& res);
    bool roomTypeProbSrvCb(semantic_mapping_v2::RoomTypeProbSrv::Request& req, semantic_mapping_v2::RoomTypeProbSrv::Response& res);
    bool objMapSrvCb(semantic_mapping_v2::ObjectMapSrv::Request& req, semantic_mapping_v2::ObjectMapSrv::Response& res);
    bool objProbSrvCb(semantic_mapping_v2::ObjectProbSrv::Request& req, semantic_mapping_v2::ObjectProbSrv::Response& res);
    bool hierarchySrvCb(semantic_mapping_v2::HierarchySrv::Request& req, semantic_mapping_v2::HierarchySrv::Response& res);

    //void publish();
    void downprojecAndPublish();

    void run();
};

#endif //SEMANTIC_MAPPING_V2_HIERARCHYMAPPER_H
