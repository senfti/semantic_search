//
// Created by thomas on 19.11.17.
//

//
// Created by thomas on 02.11.17.
//

#include "semantic_mapping_v2/RoomMapper.h"
#include <pcl/filters/voxel_grid.h>

std::vector<std::vector<double>> RoomMapper::prob_map;
std::vector<std::vector<double>> RoomMapper::prob_map_per_2d_cell;

double RoomMapper::getObjProbGivenRoom(int obj, int room){
  if(prob_map.empty()){
    ros::NodeHandle private_nh("~");
    std::string sim_file = "/home/thomas/semantic_search/src/semantic_mapping_v2/data/obj_places_map.dat";
    private_nh.param("RoomMapper/OBJ_PLACE_MAP_FILE", sim_file, sim_file);
    std::ifstream f(sim_file);
    if(!f.good()){
      ROS_WARN("CANNOT OPEN OBJ_PLACE_MAP_FILE %s", sim_file.c_str());
      return 1.f;
    }
    std::vector<double> tmp;
    while(!f.eof()){
      double v;
      f >> v;
      tmp.push_back(v);
    }
    int no = ObjectMapper::getObjNames().size();
    int nr = tmp.size()/no;
    prob_map.resize(nr);
    for(int r=0; r<nr; r++){
      prob_map[r].resize(no);
      for(int o=0; o<no; o++){
        prob_map[r][o] = tmp[r*no+o];
      }
    }
  }
  return prob_map[room][obj];
}

double RoomMapper::getObjProbGivenRoomObjPrior(int obj){
  static std::vector<double> prob_sum;
  if(prob_sum.empty()){
    if(prob_map.empty())
      getObjProbGivenRoom(0,0);
    int no = prob_map[0].size();
    int nr = prob_map.size();
    prob_sum.resize(no,0.f);
    for(int o=0; o<no; o++){
      for(int r=0; r<nr; r++){
        prob_sum[o] += prob_map[r][o];
      }
    }
    for(int o=0; o<no; o++){
      prob_sum[o] /= nr;
      //std::cout << "__________________ " << prob_sum[o]  << std::endl;
    }
  }
  return prob_sum[obj];
}

double RoomMapper::getObjProbGivenRoomRoomPrior(int room){
  static std::vector<double> prob_sum;
  if(prob_sum.empty()){
    if(prob_map.empty())
      getObjProbGivenRoom(0,0);
    int no = prob_map[0].size();
    int nr = prob_map.size();
    prob_sum.resize(nr,0.f);
    for(int r=0; r<nr; r++){
      for(int o=0; o<no; o++){
        prob_sum[r] += prob_map[r][o];
      }
    }
    for(int r=0; r<nr; r++){
      prob_sum[r] /= nr;
      //std::cout << "__________________ " << prob_sum[o]  << std::endl;
    }
  }
  return prob_sum[room];
}

double RoomMapper::getObjProbGivenRoomPerCell(int obj, int room, bool recalc, float recalc_fill){
  double fill_fraction;
  if(prob_map_per_2d_cell.empty() || recalc){
    if(recalc_fill < 0)
      ros::NodeHandle("~").param("OBJ_FILL_FRACTION", fill_fraction, fill_fraction);
    else
      fill_fraction = recalc_fill;
    ROS_ERROR("OBJ FILL %lf", fill_fraction);
    if(prob_map.empty())
      getObjProbGivenRoom(0,0);
    int no = prob_map[0].size();
    int nr = prob_map.size();
    prob_map_per_2d_cell.resize(nr);
    for(int r=0; r<nr; r++){
      prob_map_per_2d_cell[r].resize(no);
      for(int o=0; o<no; o++){
        prob_map_per_2d_cell[r][o] = prob_map[r][o]*fill_fraction;
      }
    }
  }
  return prob_map_per_2d_cell[room][obj];
}

double RoomMapper::getObjProbGivenRoomObjPriorPerCell(int obj, bool recalc){
  static std::vector<double> prob_sum;
  if(prob_sum.empty() || recalc){
    if(prob_map_per_2d_cell.empty())
      getObjProbGivenRoomPerCell(0,0);
    int no = prob_map_per_2d_cell[0].size();
    int nr = prob_map_per_2d_cell.size();
    prob_sum.resize(no,0.f);
    for(int o=0; o<no; o++){
      for(int r=0; r<nr; r++){
        prob_sum[o] += prob_map_per_2d_cell[r][o];
      }
    }
    for(int o=0; o<no; o++){
      prob_sum[o] /= nr;
      //std::cout << "__________________ " << prob_sum[o]  << std::endl;
    }
  }
  return prob_sum[obj];
}

double RoomMapper::getObjProbGivenRoomRoomPriorPerCell(int room, bool recalc){
  static std::vector<double> prob_sum;
  if(prob_sum.empty() || recalc){
    if(prob_map_per_2d_cell.empty())
      getObjProbGivenRoomPerCell(0,0);
    int no = prob_map_per_2d_cell[0].size();
    int nr = prob_map_per_2d_cell.size();
    prob_sum.resize(nr,0.f);
    for(int r=0; r<nr; r++){
      for(int o=0; o<no; o++){
        prob_sum[r] += prob_map_per_2d_cell[r][o];
      }
    }
    for(int r=0; r<nr; r++){
      prob_sum[r] /= nr;
      //std::cout << "__________________ " << prob_sum[o]  << std::endl;
    }
  }
  return prob_sum[room];
}


GMapping::OrientedPoint transformPointForward(const GMapping::OrientedPoint& point, const GMapping::OrientedPoint& current_frame){
  return GMapping::OrientedPoint(point.x*std::cos(current_frame.theta) - point.y*std::sin(current_frame.theta) + current_frame.x,
                                 point.x*std::sin(current_frame.theta) + point.y*std::cos(current_frame.theta) + current_frame.y,
                                 point.theta + current_frame.theta);
}

GMapping::OrientedPoint transformPointBackward(const GMapping::OrientedPoint& point, const GMapping::OrientedPoint& frame){
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
  return transformPointBackward(GMapping::OrientedPoint(0.0,0.0,0.0), frame);
}



RoomMapper::RoomMapper(int idx, tf::TransformListener* tf, GMapping::OrientedPoint initial_pose, const tf::Transform& initial_map_to_odom, const Door& door)
      : SlamGMapping(tf, initial_pose, initial_map_to_odom), idx_(idx),
        octo_maps_(particles_, nullptr), door_mappers_(particles_, nullptr), obj_mappers_(particles_, nullptr), room_type_mappers_(particles_, nullptr)
{
  try{
    for(auto &map : octo_maps_)
      map = new OctoMapper();
    for(auto &map : door_mappers_)
      map = new DoorMapper(idx, door);
    for(auto &map : obj_mappers_)
      map = new ObjectMapper();
    for(auto &map : room_type_mappers_)
      map = new RoomTypeMapper();
  }
  catch(std::exception& e){
    ROS_ERROR("%s",e.what());
    std::cout << e.what() << std::endl;
    ros::Rate r(0.1);
    r.sleep();
    ros::shutdown();
  }

  obstacle_map_.info.resolution = 10;

  private_nh_.param("Octomap/downsample_factor", downsample_factor_, downsample_factor_);
  private_nh_.param("Octomap/pointcloud_min_z", m_pointcloudMinZ,m_pointcloudMinZ);
  private_nh_.param("Octomap/pointcloud_max_z", m_pointcloudMaxZ,m_pointcloudMaxZ);
  private_nh_.param("RoomMapper/NUM_USED_DETECTION_SAMPLES", NUM_USED_DETECTION_SAMPLES, NUM_USED_DETECTION_SAMPLES);

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
    try{
      tf_->waitForTransform("base_laser_link", "base_link", ros::Time::now(), ros::Duration(2.0));
      tf_->lookupTransform("base_laser_link", "base_link", ros::Time::now(), base_to_laser_transform_);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      got_transform = false;
    }
  }

  octomapping_thread_ = new boost::thread(boost::bind(&RoomMapper::doOctomapping, this));
}

RoomMapper::~RoomMapper(){
  octomapping_thread_->join();
  delete octomapping_thread_;
  for(auto& map : octo_maps_)
    delete map;
  for(auto& map : door_mappers_)
    delete map;
  for(auto& map : obj_mappers_)
    delete map;
  for(auto& map : room_type_mappers_)
    delete map;
}

void RoomMapper::doOctomapping(){
  ros::Rate rate(10.0);
  while(ros::ok()){
    boost::unique_lock<boost::mutex> lock(latest_cloud_mutex_);
    if(!latest_cloud_.data.empty()){
      ros::Time t = ros::Time::now();

      OctoMapper::PCLPointCloudPtr pc_ground(new OctoMapper::PCLPointCloud());
      pcl::fromROSMsg(latest_cloud_, *pc_ground);
      ros::Time stamp = latest_cloud_.header.stamp;
      latest_cloud_ = sensor_msgs::PointCloud2();
      lock.unlock();

      int downsample_factor = 4;
      OctoMapper::PCLPointCloudPtr pc(new OctoMapper::PCLPointCloud(pc_ground->width/downsample_factor, pc_ground->height/downsample_factor));
      for(int x=0; x<pc_ground->width/downsample_factor; x++){
        for(int y=0; y<pc_ground->height/downsample_factor; y++){
          pc->at(x,y) = pc_ground->at(downsample_factor*x,downsample_factor*y);
        }
      }

      Eigen::Matrix4f cam_to_base;
      pcl_ros::transformAsMatrix(camera_to_base_transform_, cam_to_base);
      pcl::transformPointCloud(*pc, *pc, cam_to_base);

      pcl::PassThrough<OctoMapper::PCLPoint> pass_z;
      pass_z.setFilterFieldName("z");
      pass_z.setFilterLimits(-1.0, m_pointcloudMinZ);
      pass_z.setInputCloud(pc);
      pass_z.filter(*pc_ground);

      pass_z.setFilterFieldName("z");
      pass_z.setFilterLimits(m_pointcloudMinZ, m_pointcloudMaxZ);
      pass_z.setInputCloud(pc);
      pass_z.filter(*pc);

//      pcl::VoxelGrid<OctoMapper::PCLPoint> voxel_grid_filter;
//      voxel_grid_filter.setInputCloud(pc);
//      voxel_grid_filter.setLeafSize(downsample_voxel_size_, downsample_voxel_size_, downsample_voxel_size_);
//      voxel_grid_filter.filter(*pc);
//      voxel_grid_filter.setInputCloud(pc_ground);
//      voxel_grid_filter.filter(*pc_ground);

      boost::unique_lock<boost::mutex> maps_lock(maps_mutex_);
      for(int i = 0; i < octo_maps_.size(); i++){
        octo_maps_[i]->insertCloud(*pc, *pc_ground, getParticlePose3D(i, stamp));
      }
      maps_lock.unlock();

      ROS_INFO("Octomaps update in %.3lf, downsample: %d", ros::Time::now().toSec() - t.toSec(), downsample_factor_);
    }
    else
      lock.unlock();
    rate.sleep();
  }
}

void RoomMapper::cloudCb(const sensor_msgs::PointCloud2::ConstPtr &cloud){
  if(!isInitialized() || cloud->header.stamp < activate_time_ || !processed_scan_)
    return;
  processed_scan_ = false;

  boost::unique_lock<boost::mutex> gsp_lock(gsp_mutex_);
  std::vector<unsigned int> indices = gsp_->getIndexes();
  gsp_lock.unlock();
  if(!indices.empty()){
    try{
      boost::lock_guard<boost::mutex> lock(maps_mutex_);
      for(int i=0; i<indices.size(); i++){
        if(i != indices[i]){
          delete octo_maps_[i];
          octo_maps_[i] = new OctoMapper(*octo_maps_[indices[i]]);
          delete door_mappers_[i];
          door_mappers_[i] = new DoorMapper(*door_mappers_[indices[i]]);
          delete obj_mappers_[i];
          obj_mappers_[i] = new ObjectMapper(*obj_mappers_[indices[i]]);
          delete room_type_mappers_[i];
          room_type_mappers_[i] = new RoomTypeMapper(*room_type_mappers_[indices[i]]);
        }
      }
    }
    catch(std::exception& e){
      ROS_ERROR("%s",e.what());
      std::cout << e.what() << std::endl;
      ros::Rate r(0.1);
      r.sleep();
      ros::shutdown();
    }
    gsp_lock.lock();
    gsp_->resetIndexes();
    gsp_lock.unlock();
  }

  boost::lock_guard<boost::mutex> lock(latest_cloud_mutex_);
  latest_cloud_ = *cloud;


//  if(obstacle_map_.header.stamp != getGMap().header.stamp){
//    downprojectMap();
//    was_map_updated_ = true;
//  }
}


bool RoomMapper::doorCb(const geometry_msgs::PoseArray::ConstPtr& msg){
  if(!isInitialized() || msg->header.stamp < activate_time_)
    return false;

  int id = Door::getID();
  bool new_door = false;
  for(int i=0; i<door_mappers_.size(); i++){
    tf::Transform transform = getParticlePose3D(i, msg->header.stamp);
    for(const auto& pose : msg->poses){
      tf::Transform tf_pose;
      tf::poseMsgToTF(pose, tf_pose);
      if(door_mappers_[i]->addDoorProposal(transform*tf_pose, id))
        new_door = true;
    }
  }
  return new_door;
}


void RoomMapper::visionCb(const vision::VisionMsgConstPtr &msg){
  if(!isInitialized() || msg->header.stamp < activate_time_)
    return;

  ros::Time start = ros::Time::now();
  for(int i=0; i<particles_; i++){
    room_type_mappers_[i]->processMsg(msg, getParticlePose2D(i, msg->header.stamp));
  }
  //std::cout << "Room " << idx_ << " Type: " << room_type_mapper_.getBestName() << std::endl;
  ros::Time mid = ros::Time::now();

  pcl::PointCloud<pcl::PointXYZ> cloud(NUM_USED_DETECTION_SAMPLES, 1);
  for(int i=0; i<cloud.size(); i++){
    cloud[i] = pcl::PointXYZ(msg->objects.samples[i].point.x, msg->objects.samples[i].point.y, msg->objects.samples[i].point.z);
  }
  Eigen::Matrix4f cam_to_base;
  pcl_ros::transformAsMatrix(camera_to_base_transform_, cam_to_base);
  pcl::transformPointCloud(cloud, cloud, cam_to_base);

  for(int i=0; i<particles_; i++){
    pcl::PointCloud<pcl::PointXYZ> cloud_trans;
    Eigen::Matrix4f sensorToWorld;
    pcl_ros::transformAsMatrix(getParticlePose3D(i, msg->header.stamp), sensorToWorld);
    pcl::transformPointCloud(cloud, cloud_trans, sensorToWorld);
    std::pair<cv::Point, cv::Size> obj_size = obj_mappers_[i]->addCloud(cloud_trans, msg->objects, m_pointcloudMinZ, m_pointcloudMaxZ);
    room_type_mappers_[i]->resizeToObjMap(obj_size.first, obj_size.second);
  }

  ROS_INFO("VISION CALLBACK IN %.6lf / %.6lf s", (mid-start).toSec(), (ros::Time::now() - mid).toSec());
}


void RoomMapper::downprojectMap(){
  boost::lock_guard<boost::mutex> maps_lock(maps_mutex_);
  boost::mutex::scoped_lock lock(obstacle_map_mutex_);
  obstacle_map_ = octo_maps_[getBestParticleIdx()]->addDownprojected(getGMap());
//  if(obstacle_map_.data.empty())
//    return;
//
//  for(int x=0; x<obstacle_map_.info.width; x++){
//    for(int y=0; y<obstacle_map_.info.height; y++){
//      float pos_x = obstacle_map_.info.origin.position.x + x*obstacle_map_.info.resolution;
//      float pos_y = obstacle_map_.info.origin.position.y + y*obstacle_map_.info.resolution;
//      for(const auto& door : door_mappers_[getBestParticleIdx()]->getDoors()){
//        if(door.isBehindDoor(pos_x,pos_y) && !door.isInBehindDoorRect(pos_x, pos_y)){
//          obstacle_map_.data[y * obstacle_map_.info.width + x] = 100;
//          break;
//        }
//      }
//    }
//  }
}


nav_msgs::OccupancyGrid RoomMapper::getDoorBlockedMap(){
  nav_msgs::OccupancyGrid map = getMap();
  if(map.data.empty())
    return nav_msgs::OccupancyGrid();

  double res = 1.0/map.info.resolution;
  boost::unique_lock<boost::mutex> maps_lock(maps_mutex_);
  std::vector<Door> doors = door_mappers_[getBestParticleIdx()]->getDoors();
  maps_lock.unlock();
  for(const auto& door : doors){
    double x_door = ((door.getPose().getOrigin().x() - map.info.origin.position.x))*res;
    double y_door = ((door.getPose().getOrigin().y() - map.info.origin.position.y))*res;
    double angle = tf::getYaw(door.getPose().getRotation()) + M_PI_2;
    for(double r=0.0; r<1.0*res; r+=0.02*res){
      int x = x_door+r*std::cos(angle);
      int y = y_door+r*std::sin(angle);
      map.data[y * map.info.width + x] = 100;
      x = x_door-r*std::cos(angle);
      y = y_door-r*std::sin(angle);
      map.data[y * map.info.width + x] = 100;
    }
  }
  return map;
}


GMapping::OrientedPoint RoomMapper::getParticlePose2D(int particle_idx, ros::Time time){
  boost::unique_lock<boost::mutex> gsp_lock(gsp_mutex_);
  GMapping::GridSlamProcessor::Particle particle = gsp_->getParticles()[particle_idx];
  GMapping::OrientedPoint result;
  if(!particle.node){
    result = GMapping::OrientedPoint(0, 0, 0);
  }
  else if(time.toSec() >= particle.node->reading->getTime() || !particle.node->parent || !particle.node->parent->reading){
    tf::StampedTransform t1, t2;
    try{
      tf_->lookupTransform("odom", "base_link", ros::Time(particle.node->reading->getTime()), t1);
      try{
        tf_->lookupTransform("odom", "base_link", time, t2);
        tf::Transform diff = t1.inverse()*t2;
        result = transformPointForward(GMapping::OrientedPoint(base_to_laser_transform_.getOrigin().x() + diff.getOrigin().x(),
                                                               base_to_laser_transform_.getOrigin().y() + diff.getOrigin().y(),
                                                               tf::getYaw(base_to_laser_transform_.getRotation()) + tf::getYaw(diff.getRotation())), particle.node->pose);
      }
      catch (tf::TransformException ex){
        tf_->lookupTransform("odom", "base_link", ros::Time(0), t2);
        tf::Transform diff = t1.inverse()*t2;
        result = transformPointForward(GMapping::OrientedPoint(base_to_laser_transform_.getOrigin().x() + diff.getOrigin().x(),
                                                               base_to_laser_transform_.getOrigin().y() + diff.getOrigin().y(),
                                                               tf::getYaw(base_to_laser_transform_.getRotation()) + tf::getYaw(diff.getRotation())), particle.node->pose);
      }
    }
    catch (tf::TransformException ex){
      result = transformPointForward(GMapping::OrientedPoint(base_to_laser_transform_.getOrigin().x(), base_to_laser_transform_.getOrigin().y(), tf::getYaw(base_to_laser_transform_.getRotation())),
                                     particle.pose);
    }
  }
  else{
    GMapping::OrientedPoint newer_pose = particle.pose;
    double newer_time = particle.node->reading->getTime();
    for(GMapping::GridSlamProcessor::TNode* n = particle.node->parent; n && n->reading; n = n->parent){
      if(time.toSec() > n->reading->getTime()){
        try{
          tf::StampedTransform t1, t2;
          tf_->lookupTransform("odom", "base_link", ros::Time(newer_time), t1);
          tf_->lookupTransform("odom", "base_link", ros::Time(time), t2);
          tf::Transform diff = t1.inverse()*t2;
          result = transformPointForward(GMapping::OrientedPoint(base_to_laser_transform_.getOrigin().x() + diff.getOrigin().x(),
                                                                 base_to_laser_transform_.getOrigin().y() + diff.getOrigin().y(),
                                                                 tf::getYaw(base_to_laser_transform_.getRotation()) + tf::getYaw(diff.getRotation())), newer_pose);
        }
        catch (tf::TransformException ex){
          ROS_ERROR("%s",ex.what());
          result = transformPointForward(GMapping::OrientedPoint(base_to_laser_transform_.getOrigin().x(), base_to_laser_transform_.getOrigin().y(), tf::getYaw(base_to_laser_transform_.getRotation())),
                                         GMapping::interpolate(n->pose, n->reading->getTime(), newer_pose, newer_time, time.toSec()));
        }
        break;
      }
      newer_pose = n->pose;
      newer_time = n->reading->getTime();
    }
  }
  gsp_lock.unlock();

  return result;
}


tf::Transform RoomMapper::getParticlePose3D(int particle_idx, ros::Time time){
  GMapping::OrientedPoint point = getParticlePose2D(particle_idx, time);
  return tf::Transform(tf::Quaternion(tf::Vector3(0.0, 0.0, 1.0), point.theta), tf::Vector3(point.x, point.y, 0.0));
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
    boost::lock_guard<boost::mutex> lock(maps_mutex_);
    octo_maps_.resize(particles_, nullptr);
    door_mappers_.resize(particles_, nullptr);
    obj_mappers_.resize(particles_, nullptr);
    room_type_mappers_.resize(particles_, nullptr);
    try{
      for(int i=1; i<octo_maps_.size(); i++){
        octo_maps_[i] = new OctoMapper(*octo_maps_[0]);
        door_mappers_[i] = new DoorMapper(*door_mappers_[0]);
        obj_mappers_[i] = new ObjectMapper(*obj_mappers_[0]);
        room_type_mappers_[i] = new RoomTypeMapper(*room_type_mappers_[0]);
      }
    }
    catch(std::exception& e){
      ROS_ERROR("%s",e.what());
      std::cout << e.what() << std::endl;
      ros::Rate r(0.1);
      r.sleep();
      ros::shutdown();
    }
  }
  activate_time_ = ros::Time::now() + ros::Duration(settle_time_);
  was_map_updated_ = true;
}


tf::Transform RoomMapper::activate(const GMapping::OrientedPoint& robot, const Door& door){
  Door door2 = door_mappers_[0]->getDoor(door.getCounterpartId());
  tf::Transform transform;
  if(door2.isValid()){
    transform = door2.getPose().inverse()*door.getPose()*tf::Transform(tf::createQuaternionFromYaw(M_PI), tf::Vector3(0.0,0.0,0.0));
    GMapping::OrientedPoint door1_pose = door.getPose2D();
    GMapping::OrientedPoint door2_pose = door2.getPose2D();
    GMapping::OrientedPoint pose = transformPointBackward(robot, door1_pose);
    turnMPI(pose);
    pose = transformPointForward(pose, door2_pose);

    resume(pose);
  }
  else{
    ROS_ERROR("Counterpart door exists but not found, counterpart_id: %d, number of doors in new room: %d", door.getCounterpartId(), int(door_mappers_[0]->getDoors().size()));
    for(const auto& d : door_mappers_[0]->getDoors()){
      std::cout << d.getId() << std::endl;
    }
  }

  activate();
  return transform;
}


void RoomMapper::deactivate(){
  int best_particle = 0;
  if(got_first_scan_){
    int best_particle = getBestParticleIdx();
  }

  std::cout << best_particle << std::endl;
  boost::lock_guard<boost::mutex> maps_lock(maps_mutex_);
  if(best_particle != 0){
    delete octo_maps_[0];
    octo_maps_[0] = octo_maps_[best_particle];
    delete door_mappers_[0];
    door_mappers_[0] = door_mappers_[best_particle];
    delete obj_mappers_[0];
    obj_mappers_[0] = obj_mappers_[best_particle];
    delete room_type_mappers_[0];
    room_type_mappers_[0] = room_type_mappers_[best_particle];
  }
  for(int i=1; i<octo_maps_.size(); i++){
    if(i != best_particle){
      delete octo_maps_[i];
      octo_maps_[i] = nullptr;
      delete door_mappers_[i];
      door_mappers_[i] = nullptr;
      delete obj_mappers_[i];
      obj_mappers_[i] = nullptr;
      delete room_type_mappers_[i];
      room_type_mappers_[i] = nullptr;
    }
  }
  octo_maps_.resize(1);
  door_mappers_.resize(1);
  obj_mappers_.resize(1);
  room_type_mappers_.resize(1);
  //obj_mappers_[0]->applyObjAppearVanish();
}


void RoomMapper::setDoorRoom(int id, int other_room, int counterpart_id){
  for(int i=0; i<door_mappers_.size(); i++){
    door_mappers_[i]->setDoorRoom(id, other_room, counterpart_id);
  }
}


visualization_msgs::MarkerArray RoomMapper::getObjectProbMsg(int id) {
  boost::lock_guard<boost::mutex> maps_lock(maps_mutex_);
  visualization_msgs::MarkerArray res = obj_mappers_[getBestParticleIdx()]->getProbMsg(*octo_maps_[getBestParticleIdx()], id);
  if(res.markers.empty())
    return visualization_msgs::MarkerArray();

  std::vector<geometry_msgs::Point> points;
  std::vector<std_msgs::ColorRGBA> colors;
  float scale_2 = 1.f/(obj_mappers_[getBestParticleIdx()]->getResolution()*2);
  for(int i=0; i<res.markers[0].points.size(); i++){
    auto& p = res.markers[0].points[i];
    if(octo_maps_[getBestParticleIdx()]->isOccupied(p.x-scale_2,p.y-scale_2,p.z-scale_2,p.x+scale_2,p.y+scale_2,p.z+scale_2,0.5)){
      bool behind = false;
      for(const auto& door : door_mappers_[getBestParticleIdx()]->getDoors()){
        if(door.isBehindDoor(p.x, p.y)){
          behind = true;
          break;
        }
      }
      if(!behind){
        points.push_back(p);
        colors.push_back(res.markers[0].colors[i]);
      }
    }
  }
  res.markers[0].points = points;
  res.markers[0].colors = colors;

  return res;
}

visualization_msgs::MarkerArray RoomMapper::getRoomProbMsg(int id) {
  boost::lock_guard<boost::mutex> maps_lock(maps_mutex_);
  visualization_msgs::MarkerArray res = room_type_mappers_[getBestParticleIdx()]->getProbMsg(id);
  nav_msgs::OccupancyGrid map = getMap();
  if(res.markers.empty() || map.data.empty())
    return visualization_msgs::MarkerArray();

  cv::Mat_<uchar> mask(map.info.height, map.info.width, uchar(0));
  for(int x=0; x<mask.cols; x++){
    for(int y=0; y<mask.rows; y++){
      if(map.data[y * map.info.width + x] >= 0)
        mask(y,x) = 255;
    }
  }
  cv::dilate(mask, mask, cv::Mat_<uchar>::ones(3,3), cv::Point(-1,-1), 5);

  std::vector<geometry_msgs::Point> points;
  std::vector<std_msgs::ColorRGBA> colors;
  double reso = 1.00 / map.info.resolution;
  for(int i=0; i<res.markers[0].points.size(); i++){
    auto& p = res.markers[0].points[i];
    int x = (p.x - map.info.origin.position.x)*reso;
    int y = (p.y - map.info.origin.position.y)*reso;
    if(x>=0 && x<mask.cols && y>=0 && y<mask.rows && mask(y,x) > 0){
      bool behind = false;
      for(const auto& door : door_mappers_[getBestParticleIdx()]->getDoors()){
        if(door.isBehindDoor(p.x, p.y)){
          behind = true;
          break;
        }
      }
      if(!behind){
        points.push_back(p);
        colors.push_back(res.markers[0].colors[i]);
      }
    }
  }

  res.markers[0].points = points;
  res.markers[0].colors = colors;

  return res;
}


GMapping::OrientedPoint RoomMapper::getBestParticlePose2D(ros::Time time){
  tf::Transform transform = getBestParticlePose3D(time);
  return GMapping::OrientedPoint(transform.getOrigin().x(), transform.getOrigin().y(), tf::getYaw(transform.getRotation()));
}


tf::Transform RoomMapper::getBestParticlePose3D(ros::Time time){
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


geometry_msgs::PoseArray RoomMapper::getParticlePoseMsg(){
  static int seq = 0;
  geometry_msgs::PoseArray msg;
  msg.header.seq = seq++;
  msg.header.frame_id = "map";
  msg.header.stamp = ros::Time::now();
  msg.poses.resize(particles_);
  for(int i=0; i<particles_; i++){
    tf::poseTFToMsg(getParticlePose3D(i, msg.header.stamp), msg.poses[i]);
  }

  return msg;
}

//
//std::vector<float> RoomMapper::getObjectProbsComplete(std::vector<float>& room_type_probs, std::vector<size_t>& order){
//  std::vector<float> obj_probs = getObjectProbs(order);
//  std::vector<float> room_probs = getRoomTypeProbs(order);
//  room_type_probs.resize(room_probs.size());
//  std::vector<double> Pro(room_probs.size(),0.0);
//  std::vector<double> Prr(room_probs.size(),0.0);
//  double sum=0.0;
//  double max = 0.f;
//  double N = room_probs.size();
//  double v = -std::log(N);
//  for(int r=0; r<room_probs.size(); r++){
//    for(int o=0; o<obj_probs.size(); o++){
//      Pro[r] += std::log(obj_probs[o]*getObjProbGivenRoom(o,r)/N/(getObjProbGivenRoomPrior(o)/N) + (1.0-obj_probs[o])*(1.0-getObjProbGivenRoom(o,r))/N/(1.0-getObjProbGivenRoomPrior(o)/N));
//    }
//    max = std::max(max, Pro[r]);
//  }
//  for(int r=0; r<room_probs.size(); r++){
//    Pro[r] = std::exp(Pro[r]-max);
//    sum += Pro[r];
//  }
//  for(int r=0; r<room_probs.size(); r++){
//    Pro[r] = Pro[r]/sum;
//  }
//  sum=0.0;
//  for(int r=0; r<room_probs.size(); r++){
//    Prr[r] = room_probs[r]*ROOM_HIT_MISS_RATIO+(1.0-room_probs[r]);
//    sum += Prr[r];
//  }
//  for(int r=0; r<room_probs.size(); r++){
//    Prr[r] /= sum;
//  }
//  sum=0.0;
//  for(int r=0; r<room_probs.size(); r++){
//    Pro[r] = Pro[r]*OBJ_HIT_MISS_RATIO+(1.0-Pro[r]);
//    sum += Pro[r];
//  }
//  for(int r=0; r<room_probs.size(); r++){
//    Pro[r] /= sum;
//  }
//  sum = 0.0;
//  for(int r=0; r<room_probs.size(); r++){
//    Prr[r] = Prr[r]*Pro[r];
//    sum += Prr[r];
//  }
//  for(int r=0; r<room_probs.size(); r++){
//    room_type_probs[r] = Prr[r] / sum;
//  }
//
//  std::vector<float> res(obj_probs.size());
//  for(int o=0; o<obj_probs.size(); o++){
//    double prob = 0.0;
//    double prob_inv = 0.0;
//    for(int r=0; r<room_probs.size(); r++){
//      prob += room_type_probs[r]*getObjProbGivenRoom(o,r);
//      prob_inv += room_type_probs[r]*(1.0-getObjProbGivenRoom(o,r));
//    }
//    prob = prob*obj_probs[o];
//    prob_inv = prob_inv*(1.0-obj_probs[o]);
//    res[o] = prob/(prob+prob_inv);
//  }
//  return res;
//}
