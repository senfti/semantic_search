//
// Created by thomas on 19.11.17.
// Based on Ros Octomap_Server: https://github.com/OctoMap/octomap_mapping/tree/kinetic-devel/octomap_server
//

#include <execution_test/OctoMapper.h>
#include <opencv2/opencv.hpp>

using namespace octomap;
using octomap_msgs::Octomap;

bool is_equal (double a, double b, double epsilon = 1.0e-7)
{
  return std::abs(a - b) < epsilon;
}

OctoMapper::OctoMapper(double resolution, ros::NodeHandle private_nh_)
      : m_octree(NULL),
        count_octree_(NULL),
        m_maxRange(4.0),
        m_worldFrameId("/map"), m_baseFrameId("base_footprint"),
        m_useHeightMap(true),
        m_useColoredMap(false),
        m_colorFactor(0.8),
        m_latchedTopics(true),
        m_publishFreeSpace(false),
        m_res(1/resolution),
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

  count_octree_ = new OcTreeT(m_res);
  count_octree_->setProbHit(1. - ( 1. / (1. + exp(1.0))));
  count_octree_->setProbMiss(1. - ( 1. / (1. + exp(-1.0))));
  count_octree_->setClampingThresMin(0.0);
  count_octree_->setClampingThresMax(1.0);
  m_treeDepth = count_octree_->getTreeDepth();
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
  m_octree = new OcTreeT(*rhs.m_octree);
  count_octree_ = new OcTreeT(*rhs.count_octree_);
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
  if (count_octree_){
    delete count_octree_;
    count_octree_ = NULL;
  }
}


void OctoMapper::insertCloud(const PCLPointCloud& cloud, const PCLPointCloud& cloud_ground, const tf::Point& sensorOriginTf){
  ros::WallTime startTime = ros::WallTime::now();
  insertScan(sensorOriginTf, cloud, cloud_ground);
  double total_elapsed = (ros::WallTime::now() - startTime).toSec();
  ROS_DEBUG("Pointcloud insertion in OctoMapper done (%zu pts , %f sec)", cloud.size(), total_elapsed);
}


void OctoMapper::insertScan(const tf::Point& sensorOriginTf, const PCLPointCloud& cloud, const PCLPointCloud& cloud_ground){
  point3d sensorOrigin = pointTfToOctomap(sensorOriginTf);

  if (!m_octree->coordToKeyChecked(sensorOrigin, m_updateBBXMin) || !m_octree->coordToKeyChecked(sensorOrigin, m_updateBBXMax))  {
    ROS_ERROR_STREAM("Could not generate Key for origin "<<sensorOrigin);
  }

  // instead of direct scan insertion, compute update to filter ground:
  KeySet free_cells, occupied_cells;
  for (PCLPointCloud::const_iterator it = cloud_ground.begin(); it != cloud_ground.end(); ++it){
    point3d point(it->x, it->y, it->z);
    // maxrange check
    if ((m_maxRange > 0.0) && ((point - sensorOrigin).norm() > m_maxRange) ) {
      point = sensorOrigin + (point - sensorOrigin).normalized() * m_maxRange;
    }

    // only clear space (ground points)
    if (m_octree->computeRayKeys(sensorOrigin, point, m_keyRay)){
      free_cells.insert(m_keyRay.begin(), m_keyRay.end());
    }

    octomap::OcTreeKey endKey;
    if (m_octree->coordToKeyChecked(point, endKey)){
      updateMinKey(endKey, m_updateBBXMin);
      updateMaxKey(endKey, m_updateBBXMax);
    } else{
      ROS_ERROR_STREAM("Could not generate Key for endpoint "<<point);
    }
  }

  // free on ray, occupied on endpoint:
  for (PCLPointCloud::const_iterator it = cloud.begin(); it != cloud.end(); ++it){
    point3d point(it->x, it->y, it->z);
    // maxrange check
    if ((m_maxRange < 0.0) || ((point - sensorOrigin).norm() <= m_maxRange) ) {
      // free cells
      if (m_octree->computeRayKeys(sensorOrigin, point, m_keyRay)){
        free_cells.insert(m_keyRay.begin(), m_keyRay.end());
      }
      // occupied endpoint
      OcTreeKey key;
      if (m_octree->coordToKeyChecked(point, key)){
        occupied_cells.insert(key);

        updateMinKey(key, m_updateBBXMin);
        updateMaxKey(key, m_updateBBXMax);
      }
    }
    else {// ray longer than maxrange:;
      point3d new_end = sensorOrigin + (point - sensorOrigin).normalized() * m_maxRange;
      if (m_octree->computeRayKeys(sensorOrigin, new_end, m_keyRay)){
        free_cells.insert(m_keyRay.begin(), m_keyRay.end());

        octomap::OcTreeKey endKey;
        if (m_octree->coordToKeyChecked(new_end, endKey)){
          free_cells.insert(endKey);
          updateMinKey(endKey, m_updateBBXMin);
          updateMaxKey(endKey, m_updateBBXMax);
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
      m_octree->updateNode(*it, false);
    }
  }

  // now mark all occupied cells:
  for (KeySet::iterator it = occupied_cells.begin(), end=occupied_cells.end(); it!= end; it++) {
    m_octree->updateNode(*it, true);
  }

  octomap::point3d minPt, maxPt;
  ROS_DEBUG_STREAM("Bounding box keys (before): " << m_updateBBXMin[0] << " " <<m_updateBBXMin[1] << " " << m_updateBBXMin[2] << " / " <<m_updateBBXMax[0] << " "<<m_updateBBXMax[1] << " "<< m_updateBBXMax[2]);

  minPt = m_octree->keyToCoord(m_updateBBXMin);
  maxPt = m_octree->keyToCoord(m_updateBBXMax);
  ROS_DEBUG_STREAM("Updated area bounding box: "<< minPt << " - "<<maxPt);
  ROS_DEBUG_STREAM("Bounding box keys (after): " << m_updateBBXMin[0] << " " <<m_updateBBXMin[1] << " " << m_updateBBXMin[2] << " / " <<m_updateBBXMax[0] << " "<<m_updateBBXMax[1] << " "<< m_updateBBXMax[2]);

  if (m_compressMap)
    m_octree->prune();


  if (!count_octree_->coordToKeyChecked(sensorOrigin, m_updateBBXMin) || !count_octree_->coordToKeyChecked(sensorOrigin, m_updateBBXMax))  {
    ROS_ERROR_STREAM("Could not generate Key for origin "<<sensorOrigin);
  }

  // instead of direct scan insertion, compute update to filter ground:
  KeySet free_cells2, occupied_cells2;
  for (PCLPointCloud::const_iterator it = cloud_ground.begin(); it != cloud_ground.end(); ++it){
    point3d point(it->x, it->y, it->z);
    // maxrange check
    if ((m_maxRange > 0.0) && ((point - sensorOrigin).norm() > m_maxRange) ) {
      point = sensorOrigin + (point - sensorOrigin).normalized() * m_maxRange;
    }

    // only clear space (ground points)
    if (count_octree_->computeRayKeys(sensorOrigin, point, m_keyRay)){
      free_cells2.insert(m_keyRay.begin(), m_keyRay.end());
    }

    octomap::OcTreeKey endKey;
    if (count_octree_->coordToKeyChecked(point, endKey)){
      updateMinKey(endKey, m_updateBBXMin);
      updateMaxKey(endKey, m_updateBBXMax);
    } else{
      ROS_ERROR_STREAM("Could not generate Key for endpoint "<<point);
    }
  }

  // free on ray, occupied on endpoint:
  for (PCLPointCloud::const_iterator it = cloud.begin(); it != cloud.end(); ++it){
    point3d point(it->x, it->y, it->z);
    // maxrange check
    if ((m_maxRange < 0.0) || ((point - sensorOrigin).norm() <= m_maxRange) ) {
      // free cells
      if (count_octree_->computeRayKeys(sensorOrigin, point, m_keyRay)){
        free_cells2.insert(m_keyRay.begin(), m_keyRay.end());
      }
      // occupied endpoint
      OcTreeKey key;
      if (count_octree_->coordToKeyChecked(point, key)){
        occupied_cells2.insert(key);

        updateMinKey(key, m_updateBBXMin);
        updateMaxKey(key, m_updateBBXMax);
      }
    }
    else {// ray longer than maxrange:;
      point3d new_end = sensorOrigin + (point - sensorOrigin).normalized() * m_maxRange;
      if (count_octree_->computeRayKeys(sensorOrigin, new_end, m_keyRay)){
        free_cells2.insert(m_keyRay.begin(), m_keyRay.end());

        octomap::OcTreeKey endKey;
        if (count_octree_->coordToKeyChecked(new_end, endKey)){
          free_cells2.insert(endKey);
          updateMinKey(endKey, m_updateBBXMin);
          updateMaxKey(endKey, m_updateBBXMax);
        }
        else{
          ROS_ERROR_STREAM("Could not generate Key for endpoint "<<new_end);
        }
      }
    }
  }

  // mark free cells only if not seen occupied in this cloud
//  for(KeySet::iterator it = free_cells2.begin(), end=free_cells2.end(); it!= end; ++it){
//    if (occupied_cells2.find(*it) == occupied_cells2.end()){
//      count_octree_->updateNode(*it, true);
//    }
//  }

  // now mark all occupied cells:
  for (KeySet::iterator it = occupied_cells2.begin(), end=occupied_cells2.end(); it!= end; it++) {
    count_octree_->updateNode(*it, true);
  }

  ROS_DEBUG_STREAM("Bounding box keys (before): " << m_updateBBXMin[0] << " " <<m_updateBBXMin[1] << " " << m_updateBBXMin[2] << " / " <<m_updateBBXMax[0] << " "<<m_updateBBXMax[1] << " "<< m_updateBBXMax[2]);

  minPt = count_octree_->keyToCoord(m_updateBBXMin);
  maxPt = count_octree_->keyToCoord(m_updateBBXMax);
  ROS_DEBUG_STREAM("Updated area bounding box: "<< minPt << " - "<<maxPt);
  ROS_DEBUG_STREAM("Bounding box keys (after): " << m_updateBBXMin[0] << " " <<m_updateBBXMin[1] << " " << m_updateBBXMin[2] << " / " <<m_updateBBXMax[0] << " "<<m_updateBBXMax[1] << " "<< m_updateBBXMax[2]);

  if (m_compressMap)
    count_octree_->prune();
}



float OctoMapper::getOccupancy(float x, float y, float z) {
  OcTreeT::NodeType* node = m_octree->search(x,y,z);
  if(!node)
    return 0.f;
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


int OctoMapper::getCount(float x, float y, float z){
  OcTreeT::NodeType* node = count_octree_->search(x,y,z);
  if(!node)
    return 0;
  else
    return node->getLogOdds();
}


int OctoMapper::getCount(float x_min, float y_min, float z_min, float x_max, float y_max, float z_max){
  int count = 0;
  for(float x = x_min+0.5f*m_res; x < x_max; x+=m_res){
    for(float y = y_min+0.5f*m_res; y < y_max; y+=m_res){
      for(float z = z_min+0.5f*m_res; z < z_max; z+=m_res){
        count += getCount(x,y,z);
      }
    }
  }
  return count;
}


std_msgs::ColorRGBA getProbColor(double prob){
  int v = prob*13;
  std_msgs::ColorRGBA c;
  c.a = 1.0;
  switch(v){
    case 0:   c.r = 0.0;  c.g = 0.0;  c.b = 0.0;  break;
    case 1:   c.r = 0.0;  c.g = 0.0;  c.b = 0.0;  break;
    case 2:   c.r = 0.0;  c.g = 0.0;  c.b = 0.0;  break;
    case 3:   c.r = 0.0;  c.g = 0.0;  c.b = 0.0;  break;
    case 4:   c.r = 0.2;  c.g = 0.0;  c.b = 0.0;  break;
    case 5:   c.r = 0.5;  c.g = 0.0;  c.b = 0.0;  break;
    case 6:   c.r = 0.8;  c.g = 0.0;  c.b = 0.0;  break;
    case 7:   c.r = 1.0;  c.g = 0.0;  c.b = 0.0;  break;
    case 8:   c.r = 1.0;  c.g = 1.0;  c.b = 0.0;  break;
    case 9:   c.r = 0.0;  c.g = 1.0;  c.b = 0.0;  break;
    case 10:  c.r = 0.0;  c.g = 1.0;  c.b = 1.0;  break;
    case 11:  c.r = 0.0;  c.g = 0.0;  c.b = 1.0;  break;
    case 12:  c.r = 1.0;  c.g = 0.0;  c.b = 1.0;  break;
    default:  c.r = 1.0;  c.g = 0.7;  c.b = 1.0;  break;
  }
  return c;
}

visualization_msgs::MarkerArray OctoMapper::getOccupiedCellMsg(const ros::Time &rostime) {
  visualization_msgs::MarkerArray occupiedNodesVis;
  occupiedNodesVis.markers.resize(m_treeDepth+1);

  // now, traverse all leafs in the tree:
  for (OcTreeT::iterator it = m_octree->begin(m_maxTreeDepth), end = m_octree->end(); it != end; ++it){
    if(it->getOccupancy() > occupancy_thresh_){
      double z = it.getZ();
      if(z > m_occupancyMinZ && z < m_occupancyMaxZ){
        double x = it.getX();
        double y = it.getY();

        unsigned idx = it.getDepth();
        assert(idx < occupiedNodesVis.markers.size());

        geometry_msgs::Point cubeCenter;
        cubeCenter.x = x;
        cubeCenter.y = y;
        cubeCenter.z = z;

        occupiedNodesVis.markers[idx].points.push_back(cubeCenter);
        if (m_useHeightMap){
          double minX, minY, minZ, maxX, maxY, maxZ;
          m_octree->getMetricMin(minX, minY, minZ);
          m_octree->getMetricMax(maxX, maxY, maxZ);

          double h = (1.0 - std::min(std::max((cubeCenter.z-minZ)/ (maxZ - minZ), 0.0), 1.0)) *m_colorFactor;
          occupiedNodesVis.markers[idx].colors.push_back(getProbColor(it->getOccupancy())); //heightMapColor(h));
        }
      }
    }
  }

  // finish MarkerArray:
  for (unsigned i= 0; i < occupiedNodesVis.markers.size(); ++i){
    double size = m_octree->getNodeSize(i);

    occupiedNodesVis.markers[i].header.frame_id = m_worldFrameId;
    occupiedNodesVis.markers[i].header.stamp = rostime;
    occupiedNodesVis.markers[i].ns = "map";
    occupiedNodesVis.markers[i].id = i;
    occupiedNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
    occupiedNodesVis.markers[i].scale.x = size;
    occupiedNodesVis.markers[i].scale.y = size;
    occupiedNodesVis.markers[i].scale.z = size;
    occupiedNodesVis.markers[i].pose.orientation.x = occupiedNodesVis.markers[i].pose.orientation.y = occupiedNodesVis.markers[i].pose.orientation.z = 0.0;
    occupiedNodesVis.markers[i].pose.orientation.w = 1.0;
    if (!m_useColoredMap)
      occupiedNodesVis.markers[i].color = m_color;


    if (occupiedNodesVis.markers[i].points.size() > 0)
      occupiedNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
    else
      occupiedNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
  }
  return occupiedNodesVis;
}


std_msgs::ColorRGBA getCountColor(double count){
  int v = count*10;
  std_msgs::ColorRGBA c;
  c.a = 1.0;
  switch(v){
    case 0:   c.r = 0.0;  c.g = 0.0;  c.b = 0.0;  break;
    case 1:   c.r = 0.2;  c.g = 0.0;  c.b = 0.0;  break;
    case 2:   c.r = 0.5;  c.g = 0.0;  c.b = 0.0;  break;
    case 3:   c.r = 0.8;  c.g = 0.0;  c.b = 0.0;  break;
    case 4:   c.r = 1.0;  c.g = 0.0;  c.b = 0.0;  break;
    case 5:   c.r = 1.0;  c.g = 1.0;  c.b = 0.0;  break;
    case 6:   c.r = 0.0;  c.g = 1.0;  c.b = 0.0;  break;
    case 7:   c.r = 0.0;  c.g = 1.0;  c.b = 1.0;  break;
    case 8:   c.r = 0.0;  c.g = 0.0;  c.b = 1.0;  break;
    case 9:   c.r = 1.0;  c.g = 0.0;  c.b = 1.0;  break;
    default:  c.r = 1.0;  c.g = 0.7;  c.b = 1.0;  break;
  }
  return c;
}

visualization_msgs::MarkerArray OctoMapper::getCountMsg(const ros::Time &rostime, float scale){
  visualization_msgs::MarkerArray occupiedNodesVis;
  occupiedNodesVis.markers.resize(m_treeDepth+1);

  // now, traverse all leafs in the tree:
  for (OcTreeT::iterator it = count_octree_->begin(m_maxTreeDepth), end = count_octree_->end(); it != end; ++it){
    if(it->getLogOdds()*scale > 0.2){
      double z = it.getZ();
      if(z > m_occupancyMinZ && z < m_occupancyMaxZ){
        double x = it.getX();
        double y = it.getY();

        unsigned idx = it.getDepth();
        assert(idx < occupiedNodesVis.markers.size());

        geometry_msgs::Point cubeCenter;
        cubeCenter.x = x;
        cubeCenter.y = y;
        cubeCenter.z = z;

        occupiedNodesVis.markers[idx].points.push_back(cubeCenter);
        if (m_useHeightMap){
          double minX, minY, minZ, maxX, maxY, maxZ;
          count_octree_->getMetricMin(minX, minY, minZ);
          count_octree_->getMetricMax(maxX, maxY, maxZ);

          double h = (1.0 - std::min(std::max((cubeCenter.z-minZ)/ (maxZ - minZ), 0.0), 1.0)) *m_colorFactor;
          occupiedNodesVis.markers[idx].colors.push_back(getCountColor(it->getLogOdds()*scale)); //heightMapColor(h));
        }
      }
    }
  }

  // finish MarkerArray:
  for (unsigned i= 0; i < occupiedNodesVis.markers.size(); ++i){
    double size = count_octree_->getNodeSize(i);

    occupiedNodesVis.markers[i].header.frame_id = m_worldFrameId;
    occupiedNodesVis.markers[i].header.stamp = rostime;
    occupiedNodesVis.markers[i].ns = "map";
    occupiedNodesVis.markers[i].id = i;
    occupiedNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
    occupiedNodesVis.markers[i].scale.x = size;
    occupiedNodesVis.markers[i].scale.y = size;
    occupiedNodesVis.markers[i].scale.z = size;
    occupiedNodesVis.markers[i].pose.orientation.x = occupiedNodesVis.markers[i].pose.orientation.y = occupiedNodesVis.markers[i].pose.orientation.z = 0.0;
    occupiedNodesVis.markers[i].pose.orientation.w = 1.0;
    if (!m_useColoredMap)
      occupiedNodesVis.markers[i].color = m_color;


    if (occupiedNodesVis.markers[i].points.size() > 0)
      occupiedNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
    else
      occupiedNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
  }
  return occupiedNodesVis;
}
