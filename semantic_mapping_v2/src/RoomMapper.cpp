//
// Created by thomas on 19.11.17.
//

//
// Created by thomas on 02.11.17.
//

#include "semantic_mapping_v2/RoomMapper.h"
#include <pcl/filters/voxel_grid.h>

RoomMapper::RoomMapper(int idx, const Door& door)
      : idx_(idx), octo_maps_(particles_, nullptr), door_mappers_(particles_, DoorMapper(idx, door))
{
  for(auto& map : octo_maps_)
    map = new OctoMapper();

  obstacle_map_.info.resolution = 10;

  private_nh_.param("Octomap/downsample_voxel_size", downsample_voxel_size_, downsample_voxel_size_);
  private_nh_.param("Octomap/pointcloud_min_z", m_pointcloudMinZ,m_pointcloudMinZ);
  private_nh_.param("Octomap/pointcloud_max_z", m_pointcloudMaxZ,m_pointcloudMaxZ);
  private_nh_.param("octomap_wait_time", octomap_wait_time_,octomap_wait_time_);

  bool got_transform = false;
  while(!got_transform && ros::ok()){
    got_transform = true;
    try{
      tf_.waitForTransform("base_link", "camera_rgb_optical_frame", ros::Time::now(), ros::Duration(2.0));
      tf_.lookupTransform("base_link", "camera_rgb_optical_frame", ros::Time::now(), camera_to_base_transform_);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      got_transform = false;
    }
  }
}

RoomMapper::~RoomMapper(){
  for(auto& map : octo_maps_)
    delete map;
}

void RoomMapper::cloudCb(const sensor_msgs::PointCloud2::ConstPtr &cloud){
  if(!isInitialized() || cloud->header.stamp < activate_time_)
    return;

  ros::Time t = ros::Time::now();
  if(!gsp_->getIndexes().empty()){
    for(int i=0; i<gsp_->getIndexes().size(); i++){
      if(i != gsp_->getIndexes()[i]){
        delete octo_maps_[i];
        octo_maps_[i] = new OctoMapper(*octo_maps_[gsp_->getIndexes()[i]]);
        door_mappers_[i] = door_mappers_[gsp_->getIndexes()[i]];
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
    octo_maps_[i]->insertCloud(pc, getParticlePose3D(i, cloud->header.stamp));
  }
  ROS_WARN("Octomaps update in %.3lf, downsample: %.3lf", ros::Time::now().toSec() - t.toSec(), downsample_voxel_size_);

  if(obstacle_map_.header.stamp != getGMap().header.stamp){
    downprojectMap();
    was_map_updated_ = true;
  }
}


void RoomMapper::doorCb(const geometry_msgs::PoseArray::ConstPtr& msg){
  if(!isInitialized())
    return;
  
  for(int i=0; i<door_mappers_.size(); i++){
    tf::Transform transform = getParticlePose3D(i, msg->header.stamp);
    for(const auto& pose : msg->poses){
      tf::Transform tf_pose;
      tf::poseMsgToTF(pose, tf_pose);
      door_mappers_[i].addDoorProposal(transform*tf_pose);
    }
  }
}


void RoomMapper::downprojectMap(){
  boost::mutex::scoped_lock lock(obstacle_map_mutex_);
  obstacle_map_ = octo_maps_[getBestParticleIdx()]->addDownprojected(getGMap());
}


GMapping::OrientedPoint RoomMapper::getParticlePose2D(int particle_idx, ros::Time time) const{
  GMapping::OrientedPoint point;
  GMapping::OrientedPoint point1 = gsp_->getParticles()[particle_idx].pose;
  GMapping::OrientedPoint point2 = point1;
  double time1 = ros::Time::now().toSec();
  double time2 = time1;
  for(GMapping::GridSlamProcessor::TNode* n = gsp_->getParticles()[particle_idx].node; n; n = n->parent){
    point2 = point1;
    point1 = n->pose;
    time2 = time1;
    time1 = n->reading ? n->reading->getTime() : time1;
    if(n->reading == nullptr || n->reading->getTime() < time.toSec())
      break;
  }
  if(time1 == time2)
    point = point1;
  else
    point = GMapping::interpolate(point1, time1, point2, time2, time.toSec());

  return point;
}


tf::Transform RoomMapper::getParticlePose3D(int particle_idx, ros::Time time) const{
  GMapping::OrientedPoint point = getParticlePose2D(particle_idx, time);
  tf::Transform pose(tf::Quaternion(tf::Vector3(0.0, 0.0, 1.0), point.theta), tf::Vector3(point.x, point.y, 0.0));

  return pose;
}


bool RoomMapper::resetWasMapUpdated() {
  if(was_map_updated_){
    was_map_updated_ = false;
    return true;
  }
  return false;
}


void RoomMapper::setDoorRoom(const tf::Transform& pose, int other_room){
  for(int i=0; i<door_mappers_.size(); i++){
    door_mappers_[i].setDoorRoom(pose, other_room);
  }
}

