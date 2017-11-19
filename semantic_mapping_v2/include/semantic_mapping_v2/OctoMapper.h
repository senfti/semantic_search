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


class OctoMapper {
  public:
    typedef pcl::PointXYZ PCLPoint;
    typedef pcl::PointCloud<pcl::PointXYZ> PCLPointCloud;
    typedef octomap::OcTree OcTreeT;
    typedef octomap_msgs::GetOctomap OctomapSrv;
    typedef octomap_msgs::BoundingBoxQuery BBXSrv;

  protected:
    OcTreeT* m_octree;
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

    double m_pointcloudMinX;
    double m_pointcloudMaxX;
    double m_pointcloudMinY;
    double m_pointcloudMaxY;
    double m_pointcloudMinZ;
    double m_pointcloudMaxZ;
    double m_occupancyMinZ;
    double m_occupancyMaxZ;
    double m_minSizeX;
    double m_minSizeY;
    double downprojection_height_;

    bool m_compressMap;
    bool m_initConfig;
    bool m_useColoredMap;

  public:
    OctoMapper(ros::NodeHandle private_nh_ = ros::NodeHandle("~"));
    OctoMapper(const OctoMapper& rhs);
    virtual ~OctoMapper();
    virtual bool octomapBinarySrv(OctomapSrv::Request  &req, OctomapSrv::GetOctomap::Response &res);
    virtual bool octomapFullSrv(OctomapSrv::Request  &req, OctomapSrv::GetOctomap::Response &res);
    bool clearBBXSrv(BBXSrv::Request& req, BBXSrv::Response& resp);
    bool resetSrv(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp);

    void insertCloud(PCLPointCloud cloud, const tf::Transform& sensorToWorld);
    virtual bool openFile(const std::string& filename);
    void setOctree(OcTreeT* octree) {
      delete m_octree;
      m_octree = nullptr;
      m_octree = octree;
    }

    void getAllPublishMsgs(visualization_msgs::MarkerArray& occupied_cells_vis_array, octomap_msgs::Octomap& octomap_binary,
                           octomap_msgs::Octomap& octomap_full, sensor_msgs::PointCloud2& octomap_point_cloud_centers,
                           nav_msgs::OccupancyGrid& projected_map, visualization_msgs::MarkerArray& free_cells_vis_array,
                           const ros::Time& rostime = ros::Time::now());

    void insertDownprojected(nav_msgs::OccupancyGrid &map);


  protected:
    inline static void updateMinKey(const octomap::OcTreeKey& in, octomap::OcTreeKey& min) {
      for (unsigned i = 0; i < 3; ++i)
        min[i] = std::min(in[i], min[i]);
    };

    inline static void updateMaxKey(const octomap::OcTreeKey& in, octomap::OcTreeKey& max) {
      for (unsigned i = 0; i < 3; ++i)
        max[i] = std::max(in[i], max[i]);
    };

    /// Test if key is within update area of map (2D, ignores height)
    inline bool isInUpdateBBX(const OcTreeT::iterator& it) const {
      // 2^(tree_depth-depth) voxels wide:
      unsigned voxelWidth = (1 << (m_maxTreeDepth - it.getDepth()));
      octomap::OcTreeKey key = it.getIndexKey(); // lower corner of voxel
      return (key[0] + voxelWidth >= m_updateBBXMin[0]
              && key[1] + voxelWidth >= m_updateBBXMin[1]
              && key[0] <= m_updateBBXMax[0]
              && key[1] <= m_updateBBXMax[1]);
    }

    octomap_msgs::Octomap getBinaryOctoMapMsg(const ros::Time& rostime = ros::Time::now()) const;
    octomap_msgs::Octomap getFullOctoMapMsg(const ros::Time& rostime = ros::Time::now()) const;

    virtual void insertScan(const tf::Point& sensorOrigin, const PCLPointCloud& cloud);
    static std_msgs::ColorRGBA heightMapColor(double h);
};

#endif //SEMANTIC_MAPPING_V2_OCTOMAPPER_H
