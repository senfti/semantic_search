//
// Created by thomas on 19.11.17.
//

//
// Created by thomas on 02.11.17.
//

#include "semantic_mapping_v2/RoomMapper.h"
#include <pcl/filters/voxel_grid.h>

RoomMapper::RoomMapper()
      : octo_maps_(particles_)
{
  for(auto& map : octo_maps_)
    map = new OctoMapper();

  private_nh_.param("Octomap/downsample_voxel_size", downsample_voxel_size_, downsample_voxel_size_);
  private_nh_.param("Octomap/pointcloud_min_z", m_pointcloudMinZ,m_pointcloudMinZ);
  private_nh_.param("Octomap/pointcloud_max_z", m_pointcloudMaxZ,m_pointcloudMaxZ);

  bool got_transform = false;
  while(!got_transform && ros::ok()){
    try{
      tf_.waitForTransform("base_link", "camera_rgb_optical_frame", ros::Time::now(), ros::Duration(2.0));
      tf_.lookupTransform("base_link", "camera_rgb_optical_frame", ros::Time::now(), camera_to_base_transform_);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }
  }
}

void RoomMapper::cloudCb(const sensor_msgs::PointCloud2::ConstPtr &cloud){
  if(!got_first_scan_)
    return;

  ros::Time t = ros::Time::now();
  if(!gsp_->getIndexes().empty()){
    for(int i=0; i<gsp_->getIndexes().size(); i++){
      if(i != gsp_->getIndexes()[i]){
        delete octo_maps_[i];
        octo_maps_[i] = new OctoMapper(*octo_maps_[gsp_->getIndexes()[i]]);
      }
    }
    gsp_->resetIndexes();
  }

  OctoMapper::PCLPointCloud pc; // input cloud for filtering and ground-detection
  pcl::fromROSMsg(*cloud, pc);

  Eigen::Matrix4f cam_to_base;
  pcl_ros::transformAsMatrix(camera_to_base_transform_, cam_to_base);
  pcl::transformPointCloud(pc, pc, cam_to_base);

  // set up filter for height range, also removes NANs:
  pcl::PassThrough<OctoMapper::PCLPoint> pass_z;
  pass_z.setFilterFieldName("z");
  pass_z.setFilterLimits(m_pointcloudMinZ, m_pointcloudMaxZ);
  pass_z.setInputCloud(pc.makeShared());
  pass_z.filter(pc);

  pcl::VoxelGrid<OctoMapper::PCLPoint> voxel_grid_filter;
  voxel_grid_filter.setInputCloud(pc.makeShared());
  voxel_grid_filter.setLeafSize(downsample_voxel_size_, downsample_voxel_size_, downsample_voxel_size_);
  voxel_grid_filter.filter(pc);

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
  }
  ROS_WARN("Octomaps update in %.3lf, downsample: %.3lf", ros::Time::now().toSec() - t.toSec(), downsample_voxel_size_);

  if(was_map_updated_){
    obstacle_map_ = octo_maps_[getBestParticleIdx()]->addDownprojected(getGMap());
  }
}