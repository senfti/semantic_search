//
// Created by thomas on 02.11.17.
//

#include "gmapping_3dmap/OctoGMapper.h"
#include <pcl/filters/voxel_grid.h>

OctoGMapper::OctoGMapper()
  : octo_maps_(particles_), associated_nodes_(particles_, nullptr)
{
  for(auto& map : octo_maps_)
    map = new OctoMapper();

  private_nh_.param("Octomap/downsample_voxel_size", downsample_voxel_size_, downsample_voxel_size_);
  private_nh_.param("Octomap/pointcloud_min_x", m_pointcloudMinX,m_pointcloudMinX);
  private_nh_.param("Octomap/pointcloud_max_x", m_pointcloudMaxX,m_pointcloudMaxX);
  private_nh_.param("Octomap/pointcloud_min_y", m_pointcloudMinY,m_pointcloudMinY);
  private_nh_.param("Octomap/pointcloud_max_y", m_pointcloudMaxY,m_pointcloudMaxY);
  private_nh_.param("Octomap/pointcloud_min_z", m_pointcloudMinZ,m_pointcloudMinZ);
  private_nh_.param("Octomap/pointcloud_max_z", m_pointcloudMaxZ,m_pointcloudMaxZ);

  marker_pub_ = node_.advertise<visualization_msgs::MarkerArray>("occupied_cells_vis_array", 1);
  binary_map_pub_ = node_.advertise<octomap_msgs::Octomap>("octomap_binary", 1);
  full_map_pub_ = node_.advertise<octomap_msgs::Octomap>("octomap_full", 1);
  point_cloud_pub_ = node_.advertise<sensor_msgs::PointCloud2>("octomap_point_cloud_centers", 1);
  map_pub_ = node_.advertise<nav_msgs::OccupancyGrid>("projected_map", 5);
  fmarker_pub_ = node_.advertise<visualization_msgs::MarkerArray>("free_cells_vis_array", 1);

  cloud_sub_ = node_.subscribe("/camera/depth_registered/points", 1, &OctoGMapper::cloudCb, this);

  tf::TransformListener listener;
  listener.waitForTransform("base_link", "camera_rgb_optical_frame", ros::Time(0), ros::Duration(2.0));
  listener.lookupTransform("base_link", "camera_rgb_optical_frame", ros::Time(0), camera_to_base_transform_);
}

void OctoGMapper::cloudCb(const sensor_msgs::PointCloud2::ConstPtr &cloud){
  //ROS_WARN("Octomaps start");
  ros::Time t = ros::Time::now();
  if(!gsp_->getIndexes().empty()){
    //ROS_WARN("index size %d", int(gsp_->getIndexes().size()));
    for(int i=0; i<gsp_->getIndexes().size(); i++){
      //ROS_WARN("index %d: %d", i, gsp_->getIndexes()[i]);
      if(i != gsp_->getIndexes()[i]){
        delete octo_maps_[i];
        octo_maps_[i] = new OctoMapper(*octo_maps_[gsp_->getIndexes()[i]]);
      }
    }
    gsp_->resetIndexes();
  }
  //ROS_WARN("Octomaps map switching in %.3lf", ros::Time::now().toSec() - t.toSec());


  OctoMapper::PCLPointCloud pc; // input cloud for filtering and ground-detection
  pcl::fromROSMsg(*cloud, pc);
  pcl::VoxelGrid<OctoMapper::PCLPoint> sor;
  sor.setInputCloud(pc.makeShared());
  sor.setLeafSize(downsample_voxel_size_, downsample_voxel_size_, downsample_voxel_size_);
  sor.filter(pc);

  Eigen::Matrix4f cam_to_base;
  pcl_ros::transformAsMatrix(camera_to_base_transform_, cam_to_base);
  pcl::transformPointCloud(pc, pc, cam_to_base);

  // set up filter for height range, also removes NANs:
  pcl::PassThrough<OctoMapper::PCLPoint> pass_x;
  pass_x.setFilterFieldName("x");
  pass_x.setFilterLimits(m_pointcloudMinX, m_pointcloudMaxX);
  pcl::PassThrough<OctoMapper::PCLPoint> pass_y;
  pass_y.setFilterFieldName("y");
  pass_y.setFilterLimits(m_pointcloudMinY, m_pointcloudMaxY);
  pcl::PassThrough<OctoMapper::PCLPoint> pass_z;
  pass_z.setFilterFieldName("z");
  pass_z.setFilterLimits(m_pointcloudMinZ, m_pointcloudMaxZ);
  pass_x.setInputCloud(pc.makeShared());
  pass_x.filter(pc);
  pass_y.setInputCloud(pc.makeShared());
  pass_y.filter(pc);
  pass_z.setInputCloud(pc.makeShared());
  pass_z.filter(pc);

  for(int i=0; i<octo_maps_.size(); i++){
    GMapping::OrientedPoint point;
    GMapping::OrientedPoint point1 = gsp_->getParticles()[i].pose;
    GMapping::OrientedPoint point2 = point1;
    double time1 = ros::Time::now().toSec();
    double time2 = time1;
    for(GMapping::GridSlamProcessor::TNode* n = gsp_->getParticles()[i].node; n; n = n->parent){
      point2 = point1;
      point1 = n->pose;
      time2 = time1;
      time1 = n->reading ? n->reading->getTime() : time1;
      if(n->reading == nullptr || n->reading->getTime() < cloud->header.stamp.toSec())
        break;
    }
    if(time1 == time2)
      point = point1;
    else
      point = GMapping::interpolate(point1, time1, point2, time2, cloud->header.stamp.toSec());

    tf::Transform base_to_map_transform(tf::Quaternion(tf::Vector3(0.0, 0.0, 1.0), point.theta), tf::Vector3(point.x, point.y, 0.0));
    octo_maps_[i]->insertCloud(pc, base_to_map_transform);
    //ROS_WARN("Octomap %d in %.3lf", i, ros::Time::now().toSec() - t.toSec());
  }
  ROS_WARN("Octomaps update in %.3lf, downsample: %.3lf", ros::Time::now().toSec() - t.toSec(), downsample_voxel_size_);
}

int OctoGMapper::getIdxFromNodePtr(GMapping::GridSlamProcessor::TNode* ptr) const {
  for(int i=0; i<associated_nodes_.size(); i++){
    if(associated_nodes_[i] == ptr)
      return i;
  }
  return -1;
}

void OctoGMapper::updateOctoMaps(){

  ROS_WARN("here OctoGMapper");

  /*std::vector<GMapping::GridSlamProcessor::Particle> particles = gsp_->getParticles();
  std::vector<std::vector<GMapping::OrientedPoint>> poses(particles_);
  std::vector<std::vector<double>> times(particles_);
  std::vector<int> start_map(particles_,-1);

  for(int i=0; i<particles.size(); i++){
    for(GMapping::GridSlamProcessor::TNode* n = particles[i].node; n; n = n->parent){
      if(n->reading != nullptr){
        poses[i].push_back(n->pose);
        times[i].push_back(n->reading->getTime());
      }
      start_map[i] = getIdxFromNodePtr(n);
      if(start_map[i] >= 0)
        break;
    }
    if(octomaps_started_ && start_map[i] < 0){
      std::cout << "Big Problem! no start map found" << std::endl;
    }
    else if(octomaps_started_ && start_map[i] != i)
      copyTo(octo_maps_[start_map[i]], octo_maps_[i]);

    auto pose_it = poses[i].rbegin() + 1;
    auto time_it = times[i].rbegin() + 1;
    for(const auto& cloud : cloud_list_){
      double t = cloud.header.stamp.toSec();
      if(t >= *times[i].rbegin() && t <= *times[i].begin()){
        if(time_it+1 != times[i].rend() && t > *(time_it+1))
          time_it++;
        GMapping::OrientedPoint point = GMapping::interpolate(*(pose_it-1), *(time_it-1), *pose_it, *time_it, t);
        tf::Transform base_to_map_transform(tf::Quaternion(tf::Vector3(0.0,0.0,1.0), point.theta), tf::Vector3(point.x, point.y, 0.0));
        octo_maps_[i].insertCloud(cloud, camera_to_base_transform_*base_to_map_transform);
      }
    }
  }

  cloud_list_.clear();

  visualization_msgs::MarkerArray occupied_cells_vis_array;
  octomap_msgs::Octomap octomap_binary;
  octomap_msgs::Octomap octomap_full;
  sensor_msgs::PointCloud2 octomap_point_cloud_centers;
  nav_msgs::OccupancyGrid projected_map;
  visualization_msgs::MarkerArray free_cells_vis_array;

  octo_maps_[gsp_->getBestParticleIndex()].getAllPublishMsgs(occupied_cells_vis_array, octomap_binary, octomap_full,
                                                             octomap_point_cloud_centers, projected_map, free_cells_vis_array);

  if(!occupied_cells_vis_array.markers.empty())
    marker_pub_.publish(occupied_cells_vis_array);
  if(octomap_binary.resolution != 0.0)
    binary_map_pub_.publish(octomap_binary);
  if(octomap_full.resolution != 0.0)
    full_map_pub_.publish(octomap_full);
  if(octomap_point_cloud_centers.height != 0)
    point_cloud_pub_.publish(octomap_point_cloud_centers);
  if(projected_map.info.width != 0)
    map_pub_.publish(projected_map);
  if(!free_cells_vis_array.markers.empty())
    fmarker_pub_.publish(free_cells_vis_array);*/

//  static OctoMapper map;
//
//  GMapping::GridSlamProcessor::Particle best = gsp_->getParticles()[gsp_->getBestParticleIndex()];
//  for(GMapping::GridSlamProcessor::TNode* n = best.node; n; n = n->parent)
//  {
//    if(!n->reading)
//    {
//      ROS_DEBUG("Reading is NULL");
//      continue;
//    }
//    for(auto it=cloud_list_.begin(); it!=cloud_list_.end(); ++it){
//      if(it->header.stamp.toSec() > n->reading->getTime()){
//        GMapping::OrientedPoint point = n->pose;
//        tf::Transform base_to_map_transform(tf::Quaternion(tf::Vector3(0.0, 0.0, 1.0), point.theta), tf::Vector3(point.x, point.y, 0.0));
//        map.insertCloud(*it, base_to_map_transform*camera_to_base_transform_);
//        break;
//      }
//    }
//  }

  visualization_msgs::MarkerArray occupied_cells_vis_array;
  octomap_msgs::Octomap octomap_binary;
  octomap_msgs::Octomap octomap_full;
  sensor_msgs::PointCloud2 octomap_point_cloud_centers;
  nav_msgs::OccupancyGrid projected_map;
  visualization_msgs::MarkerArray free_cells_vis_array;

  octo_maps_[gsp_->getBestParticleIndex()]->getAllPublishMsgs(
        occupied_cells_vis_array, octomap_binary, octomap_full, octomap_point_cloud_centers, projected_map, free_cells_vis_array);

  if(!occupied_cells_vis_array.markers.empty())
    marker_pub_.publish(occupied_cells_vis_array);
  if(octomap_binary.resolution != 0.0)
    binary_map_pub_.publish(octomap_binary);
  if(octomap_full.resolution != 0.0)
    full_map_pub_.publish(octomap_full);
  if(octomap_point_cloud_centers.height != 0)
    point_cloud_pub_.publish(octomap_point_cloud_centers);
  if(projected_map.info.width != 0)
    map_pub_.publish(projected_map);
  if(!free_cells_vis_array.markers.empty())
    fmarker_pub_.publish(free_cells_vis_array);

  cloud_list_.clear();

  ROS_WARN("here OctoGMapper end");
}
