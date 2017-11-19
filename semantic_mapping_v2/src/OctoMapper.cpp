//
// Created by thomas on 19.11.17.
//

#include <semantic_mapping_v2/OctoMapper.h>

using namespace octomap;
using octomap_msgs::Octomap;

bool is_equal (double a, double b, double epsilon = 1.0e-7)
{
  return std::abs(a - b) < epsilon;
}

OctoMapper::OctoMapper(ros::NodeHandle private_nh_)
      : m_octree(NULL),
        m_maxRange(-1.0),
        m_worldFrameId("/map"), m_baseFrameId("base_footprint"),
        m_useHeightMap(true),
        m_useColoredMap(false),
        m_colorFactor(0.8),
        m_latchedTopics(true),
        m_publishFreeSpace(true),
        m_res(0.05),
        m_treeDepth(0),
        m_maxTreeDepth(0),
        m_pointcloudMinX(-std::numeric_limits<double>::max()),
        m_pointcloudMaxX(std::numeric_limits<double>::max()),
        m_pointcloudMinY(-std::numeric_limits<double>::max()),
        m_pointcloudMaxY(std::numeric_limits<double>::max()),
        m_pointcloudMinZ(-std::numeric_limits<double>::max()),
        m_pointcloudMaxZ(std::numeric_limits<double>::max()),
        m_occupancyMinZ(-std::numeric_limits<double>::max()),
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

  private_nh.param("Octomap/pointcloud_min_x", m_pointcloudMinX,m_pointcloudMinX);
  private_nh.param("Octomap/pointcloud_max_x", m_pointcloudMaxX,m_pointcloudMaxX);
  private_nh.param("Octomap/pointcloud_min_y", m_pointcloudMinY,m_pointcloudMinY);
  private_nh.param("Octomap/pointcloud_max_y", m_pointcloudMaxY,m_pointcloudMaxY);
  private_nh.param("Octomap/pointcloud_min_z", m_pointcloudMinZ,m_pointcloudMinZ);
  private_nh.param("Octomap/pointcloud_max_z", m_pointcloudMaxZ,m_pointcloudMaxZ);
  private_nh.param("Octomap/occupancy_min_z", m_occupancyMinZ,m_occupancyMinZ);
  private_nh.param("Octomap/occupancy_max_z", m_occupancyMaxZ,m_occupancyMaxZ);
  private_nh.param("Octomap/min_x_size", m_minSizeX,m_minSizeX);
  private_nh.param("Octomap/min_y_size", m_minSizeY,m_minSizeY);
  private_nh.param("Octomap/downprojection_height", downprojection_height_,downprojection_height_);

  private_nh.param("Octomap/sensor_model/max_range", m_maxRange, m_maxRange);

  private_nh.param("Octomap/resolution", m_res, m_res);
  private_nh.param("Octomap/sensor_model/hit", probHit, 0.7);
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
  if (m_latchedTopics)
    ROS_INFO("Publishing latched (single publish will take longer, all topics are prepared)");
  else
    ROS_INFO("Publishing non-latched (topics are only prepared as needed, will only be re-published on map change");
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

  m_pointcloudMinX = rhs.m_pointcloudMinX;
  m_pointcloudMaxX = rhs.m_pointcloudMaxX;
  m_pointcloudMinY = rhs.m_pointcloudMinY;
  m_pointcloudMaxY = rhs.m_pointcloudMaxY;
  m_pointcloudMinZ = rhs.m_pointcloudMinZ;
  m_pointcloudMaxZ = rhs.m_pointcloudMaxZ;
  m_occupancyMinZ = rhs.m_occupancyMinZ;
  m_occupancyMaxZ = rhs.m_occupancyMaxZ;
  m_minSizeX = rhs.m_minSizeX;
  m_minSizeY = rhs.m_minSizeY;

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

  PCLPointCloud pc_ground; // segmented ground plane
  PCLPointCloud pc_nonground; // everything else

  // directly transform to map frame:
  pcl::transformPointCloud(cloud, cloud, sensorToWorld);

  insertScan(sensorToWorldTf.getOrigin(), cloud);

  double total_elapsed = (ros::WallTime::now() - startTime).toSec();
  ROS_DEBUG("Pointcloud insertion in OctoMapper done (%zu+%zu pts (ground/nonground), %f sec)", pc_ground.size(), cloud.size(), total_elapsed);
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


void OctoMapper::getAllPublishMsgs(visualization_msgs::MarkerArray& occupied_cells_vis_array, octomap_msgs::Octomap& octomap_binary,
                                   octomap_msgs::Octomap& octomap_full, sensor_msgs::PointCloud2& octomap_point_cloud_centers,
                                   nav_msgs::OccupancyGrid& projected_map, visualization_msgs::MarkerArray& free_cells_vis_array,
                                   const ros::Time& rostime)
{
  ros::WallTime startTime = ros::WallTime::now();
  size_t octomapSize = m_octree->size();
  if (octomapSize <= 1){
    ROS_WARN("Nothing to publish, octree is empty");
    return;
  }

  bool publishFreeMarkerArray = m_publishFreeSpace;
  bool publishMarkerArray = m_latchedTopics;
  bool publishPointCloud = m_latchedTopics;
  bool publishBinaryMap = m_latchedTopics;
  bool publishFullMap = m_latchedTopics;

  // init markers for free space:
  visualization_msgs::MarkerArray freeNodesVis;
  // each array stores all cubes of a different size, one for each depth level:
  freeNodesVis.markers.resize(m_treeDepth+1);

  geometry_msgs::Pose pose;
  pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

  // init markers:
  visualization_msgs::MarkerArray occupiedNodesVis;
  // each array stores all cubes of a different size, one for each depth level:
  occupiedNodesVis.markers.resize(m_treeDepth+1);

  // init pointcloud:
  pcl::PointCloud<PCLPoint> pclCloud;

  // now, traverse all leafs in the tree:
  for (OcTreeT::iterator it = m_octree->begin(m_maxTreeDepth), end = m_octree->end(); it != end; ++it){
    bool inUpdateBBX = isInUpdateBBX(it);

    if (m_octree->isNodeOccupied(*it)){
      double z = it.getZ();
      if (z > m_occupancyMinZ && z < m_occupancyMaxZ)
      {
        double size = it.getSize();
        double x = it.getX();
        double y = it.getY();

        //create marker:
        if (publishMarkerArray){
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
            occupiedNodesVis.markers[idx].colors.push_back(heightMapColor(h));
          }
        }

        // insert into pointcloud:
        if (publishPointCloud) {
          pclCloud.push_back(PCLPoint(x, y, z));
        }

      }
    } else{ // node not occupied => mark as free in 2D map if unknown so far
      double z = it.getZ();
      if (z > m_occupancyMinZ && z < m_occupancyMaxZ)
      {
        if (m_publishFreeSpace){
          double x = it.getX();
          double y = it.getY();

          //create marker for free space:
          if (publishFreeMarkerArray){
            unsigned idx = it.getDepth();
            assert(idx < freeNodesVis.markers.size());

            geometry_msgs::Point cubeCenter;
            cubeCenter.x = x;
            cubeCenter.y = y;
            cubeCenter.z = z;

            freeNodesVis.markers[idx].points.push_back(cubeCenter);
          }
        }

      }
    }
  }

  // finish MarkerArray:
  if (publishMarkerArray){
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
    occupied_cells_vis_array = occupiedNodesVis;
  }


  // finish FreeMarkerArray:
  if (publishFreeMarkerArray){
    for (unsigned i= 0; i < freeNodesVis.markers.size(); ++i){
      double size = m_octree->getNodeSize(i);

      freeNodesVis.markers[i].header.frame_id = m_worldFrameId;
      freeNodesVis.markers[i].header.stamp = rostime;
      freeNodesVis.markers[i].ns = "map";
      freeNodesVis.markers[i].id = i;
      freeNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
      freeNodesVis.markers[i].scale.x = size;
      freeNodesVis.markers[i].scale.y = size;
      freeNodesVis.markers[i].scale.z = size;
      freeNodesVis.markers[i].color = m_colorFree;


      if (freeNodesVis.markers[i].points.size() > 0)
        freeNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
      else
        freeNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
    }

    free_cells_vis_array = freeNodesVis;
  }


  // finish pointcloud:
  if (publishPointCloud){
    sensor_msgs::PointCloud2 cloud;
    pcl::toROSMsg (pclCloud, cloud);
    cloud.header.frame_id = m_worldFrameId;
    cloud.header.stamp = rostime;

    octomap_point_cloud_centers = cloud;
  }

  if (publishBinaryMap)
    octomap_binary = getBinaryOctoMapMsg(rostime);

  if (publishFullMap)
    octomap_full = getFullOctoMapMsg(rostime);


  double total_elapsed = (ros::WallTime::now() - startTime).toSec();
  ROS_DEBUG("Map publishing in OctoMapper took %f sec", total_elapsed);
}

bool OctoMapper::octomapBinarySrv(OctomapSrv::Request  &req, OctomapSrv::Response &res)
{
  ros::WallTime startTime = ros::WallTime::now();
  ROS_INFO("Sending binary map data on service request");
  res.map.header.frame_id = m_worldFrameId;
  res.map.header.stamp = ros::Time::now();
  if (!octomap_msgs::binaryMapToMsg(*m_octree, res.map))
    return false;

  double total_elapsed = (ros::WallTime::now() - startTime).toSec();
  ROS_INFO("Binary octomap sent in %f sec", total_elapsed);
  return true;
}

bool OctoMapper::octomapFullSrv(OctomapSrv::Request  &req, OctomapSrv::Response &res)
{
  ROS_INFO("Sending full map data on service request");
  res.map.header.frame_id = m_worldFrameId;
  res.map.header.stamp = ros::Time::now();

  if (!octomap_msgs::fullMapToMsg(*m_octree, res.map))
    return false;

  return true;
}

bool OctoMapper::clearBBXSrv(BBXSrv::Request& req, BBXSrv::Response& resp){
  point3d min = pointMsgToOctomap(req.min);
  point3d max = pointMsgToOctomap(req.max);

  double thresMin = m_octree->getClampingThresMin();
  for(OcTreeT::leaf_bbx_iterator it = m_octree->begin_leafs_bbx(min,max), end=m_octree->end_leafs_bbx(); it!= end; ++it){
    it->setLogOdds(octomap::logodds(thresMin));
  }
  m_octree->updateInnerOccupancy();

  return true;
}

bool OctoMapper::resetSrv(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp) {
  visualization_msgs::MarkerArray occupiedNodesVis;
  occupiedNodesVis.markers.resize(m_treeDepth +1);
  ros::Time rostime = ros::Time::now();
  m_octree->clear();

  ROS_INFO("Cleared octomap");

  for (unsigned i= 0; i < occupiedNodesVis.markers.size(); ++i){
    occupiedNodesVis.markers[i].header.frame_id = m_worldFrameId;
    occupiedNodesVis.markers[i].header.stamp = rostime;
    occupiedNodesVis.markers[i].ns = "map";
    occupiedNodesVis.markers[i].id = i;
    occupiedNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
    occupiedNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
  }

  visualization_msgs::MarkerArray freeNodesVis;
  freeNodesVis.markers.resize(m_treeDepth +1);

  for (unsigned i= 0; i < freeNodesVis.markers.size(); ++i){
    freeNodesVis.markers[i].header.frame_id = m_worldFrameId;
    freeNodesVis.markers[i].header.stamp = rostime;
    freeNodesVis.markers[i].ns = "map";
    freeNodesVis.markers[i].id = i;
    freeNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
    freeNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
  }

  return true;
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


void OctoMapper::insertDownprojected(nav_msgs::OccupancyGrid &map){
  geometry_msgs::Quaternion orig_quat = map.info.origin.orientation;
  double res = 1.00 / map.info.resolution;
  if(is_equal(orig_quat.w, 1.0) && is_equal(orig_quat.x, 0.0) && is_equal(orig_quat.y, 0.0) && is_equal(orig_quat.z, 0.0)){   // should be here
    for(OcTreeT::iterator it = m_octree->begin(m_maxTreeDepth), end = m_octree->end(); it != end; ++it){
      if(m_octree->isNodeOccupied(*it) && it.getZ() <= downprojection_height_){
        int x = (it.getX() - map.info.origin.position.x) * res;
        int y = (it.getY() - map.info.origin.position.y) * res;
        if(x >= 0 && x < map.info.width && y >= 0 && y < map.info.height)
          map.data[y * map.info.width + x] = 100;
      }
    }
  }
  else{                                                                                                 //but maybe rotation between origin and map coordinates
    double yaw = std::atan2(2.0 * (orig_quat.y * orig_quat.z + orig_quat.w * orig_quat.x), orig_quat.w * orig_quat.w - orig_quat.x * orig_quat.x - orig_quat.y * orig_quat.y + orig_quat.z * orig_quat.z);
    double cos_yaw = std::cos(yaw);
    double sin_yaw = std::sin(yaw);
    for(OcTreeT::iterator it = m_octree->begin(m_maxTreeDepth), end = m_octree->end(); it != end; ++it){
      if(m_octree->isNodeOccupied(*it) && it.getZ() <= downprojection_height_){
        double xd = it.getX() - map.info.origin.position.x;
        double yd = it.getY() - map.info.origin.position.y;
        int x = (cos_yaw * xd - sin_yaw * yd) * res;
        int y = (sin_yaw * xd + cos_yaw * yd) * res;
        if(x >= 0 && x < map.info.width && y >= 0 && y < map.info.height)
          map.data[y * map.info.width + x] = 100;
      }
    }
  }
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



