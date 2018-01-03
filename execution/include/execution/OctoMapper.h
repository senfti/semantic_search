//
// Created by thomas on 19.11.17.
// Based on Ros Octomap_Server: https://github.com/OctoMap/octomap_mapping/tree/kinetic-devel/octomap_server
//

#ifndef SEMANTIC_MAPPING_V2_OCTOMAPPER_H
#define SEMANTIC_MAPPING_V2_OCTOMAPPER_H

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/ColorRGBA.h>

#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <octomap_server/OctomapServerConfig.h>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>


#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <octomap_msgs/conversions.h>

#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>

#include <boost/thread.hpp>

class OctoMapper {
  public:
    typedef pcl::PointXYZ PCLPoint;
    typedef pcl::PointCloud<pcl::PointXYZ> PCLPointCloud;
    typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PCLPointCloudPtr;
    typedef octomap::OcTree OcTreeT;
    typedef octomap_msgs::GetOctomap OctomapSrv;
    typedef octomap_msgs::BoundingBoxQuery BBXSrv;

  protected:
    OcTreeT* m_octree;
    OcTreeT* count_octree_;
    octomap::KeyRay m_keyRay;  // temp storage for ray casting
    octomap::OcTreeKey m_updateBBXMin;
    octomap::OcTreeKey m_updateBBXMax;

    double m_maxRange;
    std::string m_worldFrameId; // the map frame
    std::string m_baseFrameId; // base of the robot for ground plane filtering
    bool m_useHeightMap;
    std_msgs::ColorRGBA m_color;
    std_msgs::ColorRGBA m_colorFree;
    double m_colorFactor;

    bool m_latchedTopics;
    bool m_publishFreeSpace;

    double m_res;
    unsigned m_treeDepth;
    unsigned m_maxTreeDepth;

    double m_occupancyMinZ;
    double m_occupancyMaxZ;
    double m_minSizeX;
    double m_minSizeY;
    double downprojection_height_;

    double occupancy_thresh_ = 0.5;
    double downproject_occ_thresh_ = 0.6;
    int downproject_erode_iters_ = 4;
    int downproject_dilate_iters_ = 2;

    bool m_compressMap;
    bool m_initConfig;
    bool m_useColoredMap;

  public:
    OctoMapper(ros::NodeHandle private_nh_ = ros::NodeHandle("~"));
    OctoMapper(const OctoMapper& rhs);
    virtual ~OctoMapper();

    void insertCloud(const PCLPointCloud& cloud, const PCLPointCloud& cloud_ground, const tf::Point& sensorOriginTf);

    float getOccupancy(float x, float y, float z);
    float getOccupancy(const pcl::PointXYZ& pos) { return getOccupancy(pos.x, pos.y, pos.z); }
    bool isOccupied(float x, float y, float z, float thresh) { return getOccupancy(x,y,z) > thresh; }
    bool isOccupied(const pcl::PointXYZ& min, const pcl::PointXYZ& max, float thresh) { return isOccupied(min.x, min.y, min.z, max.x, max.y, max.z, thresh); }
    bool isOccupied(float x_min, float y_min, float z_min, float x_max, float y_max, float z_max, float thresh);
    float getOccupancy(const pcl::PointXYZ& min, const pcl::PointXYZ& max) {return getOccupancy(min.x, min.y, min.z, max.x, max.y, max.z); }
    float getOccupancy(float x_min, float y_min, float z_min, float x_max, float y_max, float z_max);
    int getCount(float x, float y, float z);
    int getCount(const pcl::PointXYZ& pos) { return getCount(pos.x, pos.y, pos.z); }
    int getCount(const pcl::PointXYZ& min, const pcl::PointXYZ& max) {return getCount(min.x, min.y, min.z, max.x, max.y, max.z); }
    int getCount(float x_min, float y_min, float z_min, float x_max, float y_max, float z_max);

    visualization_msgs::MarkerArray getOccupiedCellMsg(const ros::Time &rostime);

  protected:
    inline static void updateMinKey(const octomap::OcTreeKey& in, octomap::OcTreeKey& min) {
      for (unsigned i = 0; i < 3; ++i)
        min[i] = std::min(in[i], min[i]);
    };

    inline static void updateMaxKey(const octomap::OcTreeKey& in, octomap::OcTreeKey& max) {
      for (unsigned i = 0; i < 3; ++i)
        max[i] = std::max(in[i], max[i]);
    };


    virtual void insertScan(const tf::Point& sensorOriginTf, const PCLPointCloud& cloud, const PCLPointCloud& cloud_ground);
};

#endif //SEMANTIC_MAPPING_V2_OCTOMAPPER_H
