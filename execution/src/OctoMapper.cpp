//
// Created by thomas on 19.11.17.
// Based on Ros Octomap_Server: https://github.com/OctoMap/octomap_mapping/tree/kinetic-devel/octomap_server
//

#include <execution/OctoMapper.h>
#include <opencv2/opencv.hpp>

using namespace octomap;
using octomap_msgs::Octomap;

bool is_equal (double a, double b, double epsilon = 1.0e-7)
{
  return std::abs(a - b) < epsilon;
}

OctoMapper::OctoMapper(ros::NodeHandle private_nh_)
      : m_octree(NULL),
        count_octree_(NULL),
        m_maxRange(4.0),
        m_worldFrameId("/map"), m_baseFrameId("base_footprint"),
        m_useHeightMap(true),
        m_useColoredMap(false),
        m_colorFactor(0.8),
        m_latchedTopics(true),
        m_publishFreeSpace(false),
        m_res(0.125),
        m_treeDepth(0),
        m_maxTreeDepth(0),
        m_occupancyMinZ(0.25),
        m_occupancyMaxZ(std::numeric_limits<double>::max()),
        m_minSizeX(0.0), m_minSizeY(0.0),
        m_compressMap(true),
        m_initConfig(true),
        downprojection_height_(1.05)
{
  double probHit, probMiss, thresMin, thresMax;

  ros::NodeHandle private_nh(private_nh_);
  private_nh.param("Octomap/frame_id", m_worldFrameId, m_worldFrameId);
  private_nh.param("Octomap/base_frame_id", m_baseFrameId, m_baseFrameId);
  private_nh.param("Octomap/height_map", m_useHeightMap, m_useHeightMap);
  private_nh.param("Octomap/colored_map", m_useColoredMap, m_useColoredMap);
  private_nh.param("Octomap/color_factor", m_colorFactor, m_colorFactor);

  private_nh.param("Octomap/occupancy_min_z", m_occupancyMinZ,m_occupancyMinZ);
  private_nh.param("Octomap/occupancy_max_z", m_occupancyMaxZ,m_occupancyMaxZ);
  private_nh.param("Octomap/occupancy_thresh", occupancy_thresh_,occupancy_thresh_);
  private_nh.param("Octomap/downproject_occ_thresh", downproject_occ_thresh_,downproject_occ_thresh_);
  private_nh.param("Octomap/downproject_erode_iters", downproject_erode_iters_,downproject_erode_iters_);
  private_nh.param("Octomap/downproject_dilate_iters", downproject_dilate_iters_,downproject_dilate_iters_);
  private_nh.param("Octomap/min_x_size", m_minSizeX,m_minSizeX);
  private_nh.param("Octomap/min_y_size", m_minSizeY,m_minSizeY);
  private_nh.param("Octomap/downprojection_height", downprojection_height_,downprojection_height_);

  private_nh.param("Octomap/sensor_model/max_range", m_maxRange, m_maxRange);

  private_nh.param("Octomap/resolution", m_res, m_res);
  private_nh.param("Octomap/sensor_model/hit", probHit, 0.65);
  private_nh.param("Octomap/sensor_model/miss", probMiss, 0.35);
  private_nh.param("Octomap/sensor_model/min", thresMin, 0.12);
  private_nh.param("Octomap/sensor_model/max", thresMax, 0.97);
  private_nh.param("Octomap/compress_map", m_compressMap, m_compressMap);

  if (m_useHeightMap && m_useColoredMap) {
    ROS_WARN_STREAM("You enabled both height map and RGB color registration. This is contradictory. Defaulting to height map.");
    m_useColoredMap = false;
  }

  if (m_useColoredMap) {
    ROS_ERROR_STREAM("Colored map requested in launch file - node not running/compiled to support colors, please define COLOR_OCTOMAP_SERVER and recompile or launch the octomap_color_server node");
  }


  // initialize octomap object & params
  m_octree = new OcTreeT(m_res);
  m_octree->setProbHit(probHit);
  m_octree->setProbMiss(probMiss);
  m_octree->setClampingThresMin(thresMin);
  m_octree->setClampingThresMax(thresMax);
  m_treeDepth = m_octree->getTreeDepth();
  m_maxTreeDepth = m_treeDepth;

  double r, g, b, a;
  private_nh.param("Octomap/color/r", r, 0.0);
  private_nh.param("Octomap/color/g", g, 0.0);
  private_nh.param("Octomap/color/b", b, 1.0);
  private_nh.param("Octomap/color/a", a, 1.0);
  m_color.r = r;
  m_color.g = g;
  m_color.b = b;
  m_color.a = a;

  private_nh.param("Octomap/color_free/r", r, 0.0);
  private_nh.param("Octomap/color_free/g", g, 1.0);
  private_nh.param("Octomap/color_free/b", b, 0.0);
  private_nh.param("Octomap/color_free/a", a, 1.0);
  m_colorFree.r = r;
  m_colorFree.g = g;
  m_colorFree.b = b;
  m_colorFree.a = a;

  private_nh.param("Octomap/publish_free_space", m_publishFreeSpace, m_publishFreeSpace);

  private_nh.param("Octomap/latch", m_latchedTopics, m_latchedTopics);
}

OctoMapper::OctoMapper(const OctoMapper& rhs){
  boost::lock_guard<boost::mutex> lock(data_mutex_);
  m_octree = new OcTreeT(*rhs.m_octree);
  m_keyRay = rhs.m_keyRay;
  m_updateBBXMin = rhs.m_updateBBXMin;
  m_updateBBXMax = rhs.m_updateBBXMax;

  m_maxRange = rhs.m_maxRange;
  m_worldFrameId = rhs.m_worldFrameId;
  m_baseFrameId = rhs.m_baseFrameId;
  m_useHeightMap = rhs.m_useHeightMap;
  m_color = rhs.m_color;
  m_colorFree = rhs.m_colorFree;
  m_colorFactor = rhs.m_colorFactor;

  m_latchedTopics = rhs.m_latchedTopics;
  m_publishFreeSpace = rhs.m_publishFreeSpace;

  m_res = rhs.m_res;
  m_treeDepth = rhs.m_treeDepth;
  m_maxTreeDepth = rhs.m_maxTreeDepth;

  m_occupancyMinZ = rhs.m_occupancyMinZ;
  m_occupancyMaxZ = rhs.m_occupancyMaxZ;
  occupancy_thresh_ = rhs.occupancy_thresh_;
  downproject_occ_thresh_ = rhs.downproject_occ_thresh_;
  downproject_erode_iters_ = rhs.downproject_erode_iters_;
  downproject_dilate_iters_ = rhs.downproject_dilate_iters_;
  m_minSizeX = rhs.m_minSizeX;
  m_minSizeY = rhs.m_minSizeY;
  downprojection_height_ = rhs.downprojection_height_;

  m_compressMap = rhs.m_compressMap;

  m_initConfig = rhs.m_initConfig;
  m_useColoredMap = rhs.m_useColoredMap;
}

OctoMapper::~OctoMapper(){
  if (m_octree){
    delete m_octree;
    m_octree = NULL;
  }
}


void OctoMapper::insertCloud(PCLPointCloud cloud, const tf::Transform& sensorToWorldTf){
  ros::WallTime startTime = ros::WallTime::now();

  Eigen::Matrix4f sensorToWorld;
  pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);

  // directly transform to map frame:
  pcl::transformPointCloud(cloud, cloud, sensorToWorld);

  insertScan(sensorToWorldTf.getOrigin(), cloud);

  double total_elapsed = (ros::WallTime::now() - startTime).toSec();
  ROS_DEBUG("Pointcloud insertion in OctoMapper done (%zu pts , %f sec)", cloud.size(), total_elapsed);
}


void OctoMapper::insertScan(const tf::Point& sensorOriginTf, const PCLPointCloud& cloud){
  boost::unique_lock<boost::mutex> lock(data_mutex_);
  OcTreeT* octree = new OcTreeT(*m_octree);
  octomap::KeyRay keyRay = m_keyRay;  // temp storage for ray casting
  octomap::OcTreeKey updateBBXMin = m_updateBBXMin;
  octomap::OcTreeKey updateBBXMax = m_updateBBXMax;
  lock.unlock();
  point3d sensorOrigin = pointTfToOctomap(sensorOriginTf);

  if (!octree->coordToKeyChecked(sensorOrigin, updateBBXMin) || !octree->coordToKeyChecked(sensorOrigin, updateBBXMax))  {
    ROS_ERROR_STREAM("Could not generate Key for origin "<<sensorOrigin);
  }

  // instead of direct scan insertion, compute update to filter ground:
  KeySet free_cells, occupied_cells;

  // free on ray, occupied on endpoint:
  for (PCLPointCloud::const_iterator it = cloud.begin(); it != cloud.end(); ++it){
    point3d point(it->x, it->y, it->z);
    // maxrange check
    if ((m_maxRange < 0.0) || ((point - sensorOrigin).norm() <= m_maxRange) ) {
      // free cells
      if (octree->computeRayKeys(sensorOrigin, point, keyRay)){
        free_cells.insert(keyRay.begin(), keyRay.end());
      }
      // occupied endpoint
      OcTreeKey key;
      if (octree->coordToKeyChecked(point, key)){
        occupied_cells.insert(key);

        updateMinKey(key, updateBBXMin);
        updateMaxKey(key, updateBBXMax);
      }
    }
    else {// ray longer than maxrange:;
      point3d new_end = sensorOrigin + (point - sensorOrigin).normalized() * m_maxRange;
      if (octree->computeRayKeys(sensorOrigin, new_end, keyRay)){
        free_cells.insert(keyRay.begin(), keyRay.end());

        octomap::OcTreeKey endKey;
        if (octree->coordToKeyChecked(new_end, endKey)){
          free_cells.insert(endKey);
          updateMinKey(endKey, updateBBXMin);
          updateMaxKey(endKey, updateBBXMax);
        }
        else{
          ROS_ERROR_STREAM("Could not generate Key for endpoint "<<new_end);
        }
      }
    }
  }

  // mark free cells only if not seen occupied in this cloud
  for(KeySet::iterator it = free_cells.begin(), end=free_cells.end(); it!= end; ++it){
    if (occupied_cells.find(*it) == occupied_cells.end()){
      octree->updateNode(*it, false);
    }
  }

  // now mark all occupied cells:
  for (KeySet::iterator it = occupied_cells.begin(), end=occupied_cells.end(); it!= end; it++) {
    octree->updateNode(*it, true);
  }

  octomap::point3d minPt, maxPt;
  ROS_DEBUG_STREAM("Bounding box keys (before): " << updateBBXMin[0] << " " <<updateBBXMin[1] << " " << updateBBXMin[2] << " / " <<updateBBXMax[0] << " "<<updateBBXMax[1] << " "<< updateBBXMax[2]);

  minPt = octree->keyToCoord(updateBBXMin);
  maxPt = octree->keyToCoord(updateBBXMax);
  ROS_DEBUG_STREAM("Updated area bounding box: "<< minPt << " - "<<maxPt);
  ROS_DEBUG_STREAM("Bounding box keys (after): " << updateBBXMin[0] << " " <<updateBBXMin[1] << " " << updateBBXMin[2] << " / " <<updateBBXMax[0] << " "<<updateBBXMax[1] << " "<< updateBBXMax[2]);

  if (m_compressMap)
    octree->prune();

  lock.lock();
  delete m_octree;
  m_octree = octree;
  m_keyRay = keyRay;  // temp storage for ray casting
  m_updateBBXMin = updateBBXMin;
  m_updateBBXMax = updateBBXMax;
  lock.unlock();
}



float OctoMapper::getOccupancy(float x, float y, float z) {
  boost::lock_guard<boost::mutex> lock(data_mutex_);
  OcTreeT::NodeType* node = m_octree->search(x,y,z);
  if(!node)
    return -1.f;
  else
    return node->getOccupancy();
}



bool OctoMapper::isOccupied(float x_min, float y_min, float z_min, float x_max, float y_max, float z_max, float thresh) {
  for(float x = x_min+0.5f*m_res; x < x_max; x+=m_res){
    for(float y = y_min+0.5f*m_res; y < y_max; y+=m_res){
      for(float z = z_min+0.5f*m_res; z < z_max; z+=m_res){
        if(isOccupied(x,y,z,thresh))
          return true;
      }
    }
  }
  return false;
}


float OctoMapper::getOccupancy(float x_min, float y_min, float z_min, float x_max, float y_max, float z_max) {
  float occ = 0.f;
  for(float x = x_min+0.5f*m_res; x < x_max; x+=m_res){
    for(float y = y_min+0.5f*m_res; y < y_max; y+=m_res){
      for(float z = z_min+0.5f*m_res; z < z_max; z+=m_res){
        occ = std::max(occ, getOccupancy(x,y,z));
      }
    }
  }
  return occ;
}
