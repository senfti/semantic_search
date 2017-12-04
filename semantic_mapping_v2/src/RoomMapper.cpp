//
// Created by thomas on 19.11.17.
//

//
// Created by thomas on 02.11.17.
//

#include "semantic_mapping_v2/RoomMapper.h"
#include <pcl/filters/voxel_grid.h>

RoomMapper::RoomMapper(int idx, tf::TransformListener* tf, GMapping::OrientedPoint initial_pose, const tf::Transform& initial_map_to_odom, const Door& door)
      : idx_(idx), octo_maps_(particles_, nullptr), door_mappers_(particles_, nullptr), obj_mappers_(particles_, nullptr), SlamGMapping(tf, initial_pose, initial_map_to_odom)
{
  try{
    for(auto &map : octo_maps_)
      map = new OctoMapper();
    for(auto &map : door_mappers_)
      map = new DoorMapper(idx, door);
    for(auto &map : obj_mappers_)
      map = new ObjectMapper();
  }
  catch(std::exception& e){
    std::cout << e.what() << std::endl;
    ros::Rate r(0.1);
    r.sleep();
    ros::shutdown();
  }

  obstacle_map_.info.resolution = 10;

  private_nh_.param("Octomap/downsample_voxel_size", downsample_voxel_size_, downsample_voxel_size_);
  private_nh_.param("Octomap/pointcloud_min_z", m_pointcloudMinZ,m_pointcloudMinZ);
  private_nh_.param("Octomap/pointcloud_max_z", m_pointcloudMaxZ,m_pointcloudMaxZ);

  bool got_transform = false;
  while(!got_transform && ros::ok()){
    got_transform = true;
    try{
      tf_->waitForTransform("base_link", "camera_rgb_optical_frame", ros::Time::now(), ros::Duration(2.0));
      tf_->lookupTransform("base_link", "camera_rgb_optical_frame", ros::Time::now(), camera_to_base_transform_);
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
  for(auto& map : door_mappers_)
    delete map;
  for(auto& map : obj_mappers_)
    delete map;
}

void RoomMapper::cloudCb(const sensor_msgs::PointCloud2::ConstPtr &cloud){
  if(!isInitialized() || cloud->header.stamp < activate_time_ || !processed_scan_)
    return;
  processed_scan_ = false;

  ros::Time t = ros::Time::now();
  if(!gsp_->getIndexes().empty()){
    try{
      for(int i=0; i<gsp_->getIndexes().size(); i++){
        if(i != gsp_->getIndexes()[i]){
          delete octo_maps_[i];
          octo_maps_[i] = new OctoMapper(*octo_maps_[gsp_->getIndexes()[i]]);
          delete door_mappers_[i];
          door_mappers_[i] = new DoorMapper(*door_mappers_[gsp_->getIndexes()[i]]);
          delete obj_mappers_[i];
          obj_mappers_[i] = new ObjectMapper(*obj_mappers_[gsp_->getIndexes()[i]]);
        }
      }
    }
    catch(std::exception& e){
      ROS_ERROR("%s",e.what());
      ros::Rate r(0.1);
      r.sleep();
      ros::shutdown();
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
  ROS_INFO("Octomaps update in %.3lf, downsample: %.3lf", ros::Time::now().toSec() - t.toSec(), downsample_voxel_size_);

  if(obstacle_map_.header.stamp != getGMap().header.stamp){
    downprojectMap();
    was_map_updated_ = true;
  }
}


void RoomMapper::doorCb(const geometry_msgs::PoseArray::ConstPtr& msg){
  if(!isInitialized() || msg->header.stamp < activate_time_)
    return;

  for(int i=0; i<door_mappers_.size(); i++){
    tf::Transform transform = getParticlePose3D(i, msg->header.stamp);
    for(const auto& pose : msg->poses){
      tf::Transform tf_pose;
      tf::poseMsgToTF(pose, tf_pose);
      door_mappers_[i]->addDoorProposal(transform*tf_pose, Door::getID());
    }
  }
}


void RoomMapper::visionCb(const vision::VisionMsgConstPtr &msg){
  if(!isInitialized() || msg->header.stamp < activate_time_)
    return;

  ros::Time start = ros::Time::now();
  room_type_mapper_.processMsg(msg);
  std::cout << "Room " << idx_ << " Type: " << room_type_mapper_.getBestName() << std::endl;
  ros::Time mid = ros::Time::now();

  pcl::PointCloud<pcl::PointXYZ> cloud(msg->detection_samples.size(), 1);
  for(int i=0; i<cloud.size(); i++){
    cloud[i] = pcl::PointXYZ(msg->detection_samples[i].x, msg->detection_samples[i].y, msg->detection_samples[i].z);
  }
  Eigen::Matrix4f cam_to_base;
  pcl_ros::transformAsMatrix(camera_to_base_transform_, cam_to_base);
  pcl::transformPointCloud(cloud, cloud, cam_to_base);

  for(int i=0; i<particles_; i++){
    pcl::PointCloud<pcl::PointXYZ> cloud_trans;
    Eigen::Matrix4f sensorToWorld;
    pcl_ros::transformAsMatrix(getParticlePose3D(i, msg->header.stamp), sensorToWorld);
    pcl::transformPointCloud(cloud, cloud_trans, sensorToWorld);
    obj_mappers_[i]->addCloud(cloud_trans, msg->detection_samples);
  }

  ROS_INFO("VISION CALLBACK IN %.6lf / %.6lf s", (mid-start).toSec(), (ros::Time::now() - mid).toSec());
}


void RoomMapper::downprojectMap(){
  boost::mutex::scoped_lock lock(obstacle_map_mutex_);
  obstacle_map_ = octo_maps_[getBestParticleIdx()]->addDownprojected(getGMap());
}


GMapping::OrientedPoint RoomMapper::getParticlePose2D(int particle_idx, ros::Time time) const{
  GMapping::GridSlamProcessor::Particle particle = gsp_->getParticles()[particle_idx];
  if(!particle.node)
    return GMapping::OrientedPoint(0,0,0);
  if(time.toSec() >= particle.node->reading->getTime() || !particle.node->parent || !particle.node->parent->reading)
    return particle.pose;

  GMapping::OrientedPoint newer_pose = particle.pose;
  double newer_time = particle.node->reading->getTime();
  for(GMapping::GridSlamProcessor::TNode* n = particle.node->parent; n && n->reading; n = n->parent){
    if(time.toSec() > n->reading->getTime())
      return GMapping::interpolate(n->pose, n->reading->getTime(), newer_pose, newer_time, time.toSec());

    newer_pose = n->pose;
    newer_time = n->reading->getTime();
  }

  return newer_pose;
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


void RoomMapper::activate(){
  if(octo_maps_.size() < particles_){
    octo_maps_.resize(particles_, nullptr);
    door_mappers_.resize(particles_, nullptr);
    obj_mappers_.resize(particles_, nullptr);
    try{
      for(int i=1; i<octo_maps_.size(); i++){
        octo_maps_[i] = new OctoMapper(*octo_maps_[0]);
        door_mappers_[i] = new DoorMapper(*door_mappers_[0]);
        obj_mappers_[i] = new ObjectMapper(*obj_mappers_[0]);
      }
    }
    catch(std::exception& e){
      ROS_ERROR("%s",e.what());
      ros::Rate r(0.1);
      r.sleep();
      ros::shutdown();
    }
  }
  activate_time_ = ros::Time::now() + ros::Duration(settle_time_);
}

GMapping::OrientedPoint transformPointIntoFrame(const GMapping::OrientedPoint& point, const GMapping::OrientedPoint& frame){
  double x_tmp = point.x-frame.x;
  double y_tmp = point.y-frame.y;
  return GMapping::OrientedPoint(x_tmp*std::cos(-frame.theta) - y_tmp*std::sin(-frame.theta), x_tmp*std::sin(-frame.theta) + y_tmp*std::cos(-frame.theta), point.theta - frame.theta);
}

void turnMPI(GMapping::OrientedPoint& point){
  point.x = -point.x;
  point.y = -point.y;
  point.theta = point.theta + M_PI;
}

GMapping::OrientedPoint invertFrame(GMapping::OrientedPoint& frame){
  return transformPointIntoFrame(GMapping::OrientedPoint(0.0,0.0,0.0), frame);
}

std::string o2s(GMapping::OrientedPoint p) { return std::to_string(p.x) + " " + std::to_string(p.y) + " " + std::to_string(p.theta); }
void RoomMapper::activate(const GMapping::OrientedPoint& robot, const Door& door){
  Door door2 = door_mappers_[0]->getDoor(door.counterpart_id_);
  if(door2.isValid()){
    GMapping::OrientedPoint door1_pose = door.getPose2D();
    GMapping::OrientedPoint door2_pose = door2.getPose2D();
    GMapping::OrientedPoint pose = transformPointIntoFrame(robot, door1_pose);
    turnMPI(pose);
    GMapping::OrientedPoint map = invertFrame(door2_pose);
    pose = transformPointIntoFrame(pose, map);

    resume(pose);
  }
  else{
    ROS_ERROR("Counterpart door exists but not found, counterpart_id: %d, number of doors in new room: %d", door.counterpart_id_, int(door_mappers_[0]->getDoors().size()));
    for(const auto& d : door_mappers_[0]->getDoors()){
      std::cout << d.id_ << std::endl;
    }
  }

  activate();
}


void RoomMapper::deactivate(){
  int best_particle = 0;
  if(got_first_scan_){
    int best_particle = getBestParticleIdx();
  }

  std::cout << best_particle << std::endl;
  if(best_particle != 0){
    delete octo_maps_[0];
    octo_maps_[0] = octo_maps_[best_particle];
    delete door_mappers_[0];
    door_mappers_[0] = door_mappers_[best_particle];
    delete obj_mappers_[0];
    obj_mappers_[0] = obj_mappers_[best_particle];
  }
  for(int i=1; i<octo_maps_.size(); i++){
    if(i != best_particle){
      delete octo_maps_[i];
      octo_maps_[i] = nullptr;
      delete door_mappers_[i];
      door_mappers_[i] = nullptr;
      delete obj_mappers_[i];
      obj_mappers_[i] = nullptr;
    }
  }
  octo_maps_.resize(1);
  door_mappers_.resize(1);
  obj_mappers_.resize(1);
}


void RoomMapper::setDoorRoom(const tf::Transform& pose, int other_room, int counterpart_id){
  for(int i=0; i<door_mappers_.size(); i++){
    door_mappers_[i]->setDoorRoom(pose, other_room, counterpart_id);
  }
}


visualization_msgs::MarkerArray RoomMapper::getObjectProbMsg(int id) const {
  visualization_msgs::MarkerArray res = obj_mappers_[getBestParticleIdx()]->getProbMsg(id);
  std::vector<geometry_msgs::Point> points;
  std::vector<std_msgs::ColorRGBA> colors;
  for(int i=0; i<res.markers[0].points.size(); i++){
    auto& p = res.markers[0].points[i];
    float scale_2 = res.markers[0].scale.x/2;
    if(octo_maps_[getBestParticleIdx()]->isOccupied(p.x-scale_2,p.y-scale_2,p.z-scale_2,p.x+scale_2,p.y+scale_2,p.z+scale_2,0.5)){
      points.push_back(p);
      colors.push_back(res.markers[0].colors[i]);
    }
  }
  res.markers[0].points = points;
  res.markers[0].colors = colors;

  return res;
}


GMapping::OrientedPoint RoomMapper::getBestParticlePose2D(ros::Time time) const{
  tf::Transform transform = getBestParticlePose3D(time);
  return GMapping::OrientedPoint(transform.getOrigin().x(), transform.getOrigin().y(), tf::getYaw(transform.getRotation()));
}


tf::Transform RoomMapper::getBestParticlePose3D(ros::Time time) const{
  tf::StampedTransform transform;
  try{
    tf_->lookupTransform("map", "base_link", time, transform);
    return transform;
  }
  catch (tf::TransformException ex){
  }
  try{
    tf_->lookupTransform("map", "base_link", ros::Time(0), transform);
    return transform;
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }
  return tf::Transform();
}


geometry_msgs::PoseArray RoomMapper::getParticlePoseMsg() const{
  static int seq = 0;
  geometry_msgs::PoseArray msg;
  msg.header.seq = seq++;
  msg.header.frame_id = "map";
  msg.header.stamp = ros::Time::now();
  msg.poses.resize(particles_);
  for(int i=0; i<particles_; i++){
    tf::Transform pose(tf::Quaternion(tf::Vector3(0.0,0.0,1.0), gsp_->getParticles()[i].pose.theta), tf::Vector3(gsp_->getParticles()[i].pose.x, gsp_->getParticles()[i].pose.y, 0.0));
    tf::poseTFToMsg(pose, msg.poses[i]);
  }

  return msg;
}
