//
// Created by thomas on 19.11.17.
//

#ifndef SEMANTIC_MAPPING_V2_HIERARCHYMAPPER_H
#define SEMANTIC_MAPPING_V2_HIERARCHYMAPPER_H

#include "semantic_mapping_v2/RoomMapper.h"

class HierarchyMapper{
  protected:
    std::vector<RoomMapper*> room_mapper_;
    int current_mapper_ = -1;

    ros::Subscriber laser_sub_;
    ros::Subscriber cloud_sub_;
    ros::Subscriber door_pose_sub_;

    ros::Publisher map_pub_;
    ros::Publisher gmap_pub_;
    ros::Publisher map_info_pub_;
    ros::Publisher marker_pub_;
    ros::Publisher door_pose_pub_;

    tf::TransformBroadcaster* tfB_;
    ros::NodeHandle nh_;
    boost::thread* transform_thread_;

    void transformPublishLoop(double transform_publish_period);

    double transform_publish_period_;

  public:
    HierarchyMapper();
    ~HierarchyMapper();

    void addMapper(const Door& door = Door());
    void switchMapper(int mapper_idx);

    void cloudCb(const sensor_msgs::PointCloud2::ConstPtr& cloud);
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    void doorPoseCb(const geometry_msgs::PoseArray::ConstPtr& msg);

    void publish();

    void run();
};

#endif //SEMANTIC_MAPPING_V2_HIERARCHYMAPPER_H
