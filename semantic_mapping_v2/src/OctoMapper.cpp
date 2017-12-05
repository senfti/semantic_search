//
// Created by thomas on 19.11.17.
// Based on Ros Octomap_Server: https://github.com/OctoMap/octomap_mapping/tree/kinetic-devel/octomap_server
//

#include <semantic_mapping_v2/OctoMapper.h>
#include <opencv2/opencv.hpp>

using namespace octomap;
using octomap_msgs::Octomap;

bool is_equal (double a, double b, double epsilon = 1.0e-7)
{
  return std::abs(a - b) < epsilon;
}

OctoMapper::OctoMapper(ros::NodeHandle private_nh_)
      : m_octree(NULL),
        m_maxRange(4.0),
        m_worldFrameId("/map"), m_baseFrameId("base_footprint"),
        m_useHeightMap(true),
        m_useColoredMap(false),
        m_colorFactor(0.8),
        m_latchedTopics(true),
        m_publishFreeSpace(false),
        m_res(0.1),
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
  private_nh.param("Octomap/sensor_model/miss", probMiss, 0.4);
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

bool OctoMapper::openFile(const std::string& filename){
  if (filename.length() <= 3)
    return false;

  std::string suffix = filename.substr(filename.length()-3, 3);
  if (suffix== ".bt"){
    if (!m_octree->readBinary(filename)){
      return false;
    }
  }
  else if (suffix == ".ot"){
    AbstractOcTree* tree = AbstractOcTree::read(filename);
    if (!tree){
      return false;
    }
    if (m_octree){
      delete m_octree;
      m_octree = NULL;
    }
    m_octree = dynamic_cast<OcTreeT*>(tree);
    if (!m_octree){
      ROS_ERROR("Could not read OcTree in file, currently there are no other types supported in .ot");
      return false;
    }
  }
  else{
    return false;
  }

  ROS_INFO("Octomap file %s loaded (%zu nodes).", filename.c_str(),m_octree->size());

  m_treeDepth = m_octree->getTreeDepth();
  m_maxTreeDepth = m_treeDepth;
  m_res = m_octree->getResolution();
  double minX, minY, minZ;
  double maxX, maxY, maxZ;
  m_octree->getMetricMin(minX, minY, minZ);
  m_octree->getMetricMax(maxX, maxY, maxZ);

  m_updateBBXMin[0] = m_octree->coordToKey(minX);
  m_updateBBXMin[1] = m_octree->coordToKey(minY);
  m_updateBBXMin[2] = m_octree->coordToKey(minZ);

  m_updateBBXMax[0] = m_octree->coordToKey(maxX);
  m_updateBBXMax[1] = m_octree->coordToKey(maxY);
  m_updateBBXMax[2] = m_octree->coordToKey(maxZ);

  return true;
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
  point3d sensorOrigin = pointTfToOctomap(sensorOriginTf);

  if (!m_octree->coordToKeyChecked(sensorOrigin, m_updateBBXMin) || !m_octree->coordToKeyChecked(sensorOrigin, m_updateBBXMax))  {
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
}



float OctoMapper::getOccupancy(float x, float y, float z) const {
  OcTreeT::NodeType* node = m_octree->search(x,y,z);
  if(!node)
    return -1.f;
  else
    return node->getOccupancy();
}


void OctoMapper::getAllPublishMsgs(visualization_msgs::MarkerArray& occupied_cells_vis_array, octomap_msgs::Octomap& octomap_binary,
                                   octomap_msgs::Octomap& octomap_full, visualization_msgs::MarkerArray& free_cells_vis_array,
                                   const ros::Time& rostime)
{
  ros::WallTime startTime = ros::WallTime::now();
  size_t octomapSize = m_octree->size();
  if (octomapSize <= 1){
    ROS_WARN("Nothing to publish, octree is empty");
    return;
  }

  if(m_publishFreeSpace)
    occupied_cells_vis_array = getOccupiedAndFreeCellMsg(free_cells_vis_array, rostime);
  else
    occupied_cells_vis_array = getOccupiedCellMsg(rostime);

  if (m_latchedTopics)
    octomap_binary = getBinaryOctoMapMsg(rostime);

  if (m_latchedTopics)
    octomap_full = getFullOctoMapMsg(rostime);


  double total_elapsed = (ros::WallTime::now() - startTime).toSec();
  ROS_DEBUG("Map publishing in OctoMapper took %f sec", total_elapsed);
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

visualization_msgs::MarkerArray OctoMapper::getOccupiedCellMsg(const ros::Time &rostime) const{
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
    if (!m_useColoredMap)
      occupiedNodesVis.markers[i].color = m_color;


    if (occupiedNodesVis.markers[i].points.size() > 0)
      occupiedNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
    else
      occupiedNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
  }
  return occupiedNodesVis;
}


visualization_msgs::MarkerArray OctoMapper::getOccupiedAndFreeCellMsg(visualization_msgs::MarkerArray& free_cells, const ros::Time &rostime) const{
  free_cells.markers.resize(m_treeDepth+1);
  visualization_msgs::MarkerArray occupiedNodesVis;
  occupiedNodesVis.markers.resize(m_treeDepth+1);

  for (OcTreeT::iterator it = m_octree->begin(m_maxTreeDepth), end = m_octree->end(); it != end; ++it){
    double z = it.getZ();
    if (z > m_occupancyMinZ && z < m_occupancyMaxZ){
      double size = it.getSize();
      double x = it.getX();
      double y = it.getY();
      unsigned idx = it.getDepth();
      assert(idx < occupiedNodesVis.markers.size());

      geometry_msgs::Point cubeCenter;
      cubeCenter.x = x;
      cubeCenter.y = y;
      cubeCenter.z = z;

      if (it->getOccupancy() > occupancy_thresh_){
        occupiedNodesVis.markers[idx].points.push_back(cubeCenter);
        if (m_useHeightMap){
          double minX, minY, minZ, maxX, maxY, maxZ;
          m_octree->getMetricMin(minX, minY, minZ);
          m_octree->getMetricMax(maxX, maxY, maxZ);

          double h = (1.0 - std::min(std::max((cubeCenter.z-minZ)/ (maxZ - minZ), 0.0), 1.0)) *m_colorFactor;
          occupiedNodesVis.markers[idx].colors.push_back(getProbColor(it->getOccupancy())); //heightMapColor(h));
        }
      }
      else{
        free_cells.markers[idx].points.push_back(cubeCenter);
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
    if (!m_useColoredMap)
      occupiedNodesVis.markers[i].color = m_color;


    if (occupiedNodesVis.markers[i].points.size() > 0)
      occupiedNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
    else
      occupiedNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
  }

  // finish FreeMarkerArray:
  for (unsigned i= 0; i < free_cells.markers.size(); ++i){
    double size = m_octree->getNodeSize(i);

    free_cells.markers[i].header.frame_id = m_worldFrameId;
    free_cells.markers[i].header.stamp = rostime;
    free_cells.markers[i].ns = "map";
    free_cells.markers[i].id = i;
    free_cells.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
    free_cells.markers[i].scale.x = size;
    free_cells.markers[i].scale.y = size;
    free_cells.markers[i].scale.z = size;
    free_cells.markers[i].color = m_colorFree;


    if (free_cells.markers[i].points.size() > 0)
      free_cells.markers[i].action = visualization_msgs::Marker::ADD;
    else
      free_cells.markers[i].action = visualization_msgs::Marker::DELETE;
  }

  return occupiedNodesVis;
}


void OctoMapper::clearBBX(pcl::PointXYZ min, pcl::PointXYZ max){
  double thresMin = m_octree->getClampingThresMin();
  for(OcTreeT::leaf_bbx_iterator it = m_octree->begin_leafs_bbx(point3d(min.x,min.y,min.z),point3d(max.x,max.y,max.z)), end=m_octree->end_leafs_bbx(); it!= end; ++it){
    it->setLogOdds(octomap::logodds(thresMin));
  }
  m_octree->updateInnerOccupancy();
}


void OctoMapper::reset(){
  m_octree->clear();
}


Octomap OctoMapper::getBinaryOctoMapMsg(const ros::Time &rostime) const{
  Octomap map;
  map.header.frame_id = m_worldFrameId;
  map.header.stamp = rostime;

  if(!octomap_msgs::binaryMapToMsg(*m_octree, map))
    ROS_ERROR("Error serializing OctoMap");

  return map;
}

Octomap OctoMapper::getFullOctoMapMsg(const ros::Time &rostime) const{
  Octomap map;
  map.header.frame_id = m_worldFrameId;
  map.header.stamp = rostime;

  if (!octomap_msgs::fullMapToMsg(*m_octree, map))
    ROS_ERROR("Error serializing OctoMap");

  return map;
}


nav_msgs::OccupancyGrid OctoMapper::addDownprojected(const nav_msgs::OccupancyGrid &map) const{
  nav_msgs::OccupancyGrid downprojected_map = map;
  cv::Mat_<uchar> tmp_map(map.info.height, map.info.width, uchar(0));
  geometry_msgs::Quaternion orig_quat = downprojected_map.info.origin.orientation;
  double res = 1.00 / downprojected_map.info.resolution;
  if(is_equal(orig_quat.w, 1.0) && is_equal(orig_quat.x, 0.0) && is_equal(orig_quat.y, 0.0) && is_equal(orig_quat.z, 0.0)){   // should be here
    for(OcTreeT::iterator it = m_octree->begin(m_maxTreeDepth), end = m_octree->end(); it != end; ++it){
      if(it->getOccupancy() > downproject_occ_thresh_ && it.getZ() <= downprojection_height_){
        double x_min = ((it.getX() - downprojected_map.info.origin.position.x) - m_res/2.0)*res;
        double y_min = ((it.getY() - downprojected_map.info.origin.position.y) - m_res/2.0)*res;
        double x_max = x_min + m_res*res;
        double y_max = y_min + m_res*res;
        for(int x=std::round(x_min); x<x_max-0.5; x++){
          for(int y=std::round(y_min); y<y_max-0.5; y++){
            if(x >= 0 && x < downprojected_map.info.width && y >= 0 && y < downprojected_map.info.height){
              tmp_map(y,x) = 255;
              //downprojected_map.data[y * downprojected_map.info.width + x] = 100;
            }
          }
        }
      }
    }
  }
  else{                                                                                                 //but maybe rotation between origin and map coordinates
    double yaw = std::atan2(2.0 * (orig_quat.y * orig_quat.z + orig_quat.w * orig_quat.x), orig_quat.w * orig_quat.w - orig_quat.x * orig_quat.x - orig_quat.y * orig_quat.y + orig_quat.z * orig_quat.z);
    double cos_yaw = std::cos(yaw);
    double sin_yaw = std::sin(yaw);
    for(OcTreeT::iterator it = m_octree->begin(m_maxTreeDepth), end = m_octree->end(); it != end; ++it){
      if(m_octree->isNodeOccupied(*it) && it.getZ() <= downprojection_height_){
        double xd = it.getX() - downprojected_map.info.origin.position.x;
        double yd = it.getY() - downprojected_map.info.origin.position.y;
        int x = (cos_yaw * xd - sin_yaw * yd) * res;
        int y = (sin_yaw * xd + cos_yaw * yd) * res;
        if(x >= 0 && x < downprojected_map.info.width && y >= 0 && y < downprojected_map.info.height)
          tmp_map(y,x) = 255;
          //downprojected_map.data[y * downprojected_map.info.width + x] = 100;
      }
    }
  }
//  cv::Mat sdf;
  //cv::resize(tmp_map, sdf, cv::Size(tmp_map.cols*2, tmp_map.rows*2),0,0,cv::INTER_NEAREST);
//  cv::flip(tmp_map,sdf,0);
//  cv::imshow("1", sdf);
  cv::dilate(tmp_map, tmp_map, cv::Mat_<uchar>::ones(3, 3), cv::Point(-1,-1), downproject_dilate_iters_);
  //cv::resize(tmp_map, sdf, cv::Size(tmp_map.cols*2, tmp_map.rows*2),0,0,cv::INTER_NEAREST);
//  cv::flip(tmp_map,sdf,0);
//  cv::imshow("2", sdf);
  cv::erode(tmp_map, tmp_map, cv::Mat_<uchar>::ones(3, 3), cv::Point(-1,-1), downproject_erode_iters_);
  //cv::resize(tmp_map, sdf, cv::Size(tmp_map.cols*2, tmp_map.rows*2),0,0,cv::INTER_NEAREST);
//  cv::flip(tmp_map,sdf,0);
//  cv::imshow("3", sdf);
//  cv::waitKey(1);
  for(int x=0; x<tmp_map.cols; x++){
    for(int y=0; y<tmp_map.rows; y++){
      if(tmp_map(y,x)){
        downprojected_map.data[y * downprojected_map.info.width + x] = 100;
      }
    }
  }

  return downprojected_map;
}


std_msgs::ColorRGBA OctoMapper::heightMapColor(double h) {
  std_msgs::ColorRGBA color;
  color.a = 1.0;
  // blend over HSV-values (more colors)

  double s = 1.0;
  double v = 1.0;

  h -= floor(h);
  h *= 6;
  int i;
  double m, n, f;

  i = floor(h);
  f = h - i;
  if (!(i & 1))
    f = 1 - f; // if i is even
  m = v * (1 - s);
  n = v * (1 - s * f);

  switch (i) {
    case 6:
    case 0:
      color.r = v; color.g = n; color.b = m;
      break;
    case 1:
      color.r = n; color.g = v; color.b = m;
      break;
    case 2:
      color.r = m; color.g = v; color.b = n;
      break;
    case 3:
      color.r = m; color.g = n; color.b = v;
      break;
    case 4:
      color.r = n; color.g = m; color.b = v;
      break;
    case 5:
      color.r = v; color.g = m; color.b = n;
      break;
    default:
      color.r = 1; color.g = 0.5; color.b = 0.5;
      break;
  }

  return color;
}


bool OctoMapper::isOccupied(float x_min, float y_min, float z_min, float x_max, float y_max, float z_max, float thresh) const{
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
