//
// Created by thomas on 19.11.17.
//

#include "semantic_mapping_v2/HierarchyMapper.h"
#include <tf/transform_datatypes.h>
#include <semantic_mapping_v2/RoomSwitchMsg.h>

HierarchyMapper::HierarchyMapper()
  : service_spinner_(1, &service_queue_)
{
  addMapper(Door());

  laser_sub_ = nh_.subscribe("scan_filtered", 1, &HierarchyMapper::laserCallback, this);
  cloud_sub_ = nh_.subscribe("/camera/depth_registered/points", 1, &HierarchyMapper::cloudCb, this);
  door_pose_sub_ = nh_.subscribe("door_poses", 1, &HierarchyMapper::doorPoseCb, this);
  vision_sub_ = nh_.subscribe("vision_result", 1, &HierarchyMapper::visionCb, this);

  map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  gmap_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("gmap", 1, true);
  map_switch_pub_ = nh_.advertise<semantic_mapping_v2::RoomSwitchMsg>("map_switch", 1);
  map_door_blocked_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("map_door_blocked", 1, true);
  map_info_pub_ = nh_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
  marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("occupied_cells_vis_array", 1, true);
  door_pose_pub_ = nh_.advertise<geometry_msgs::PoseArray>("mapper_door_poses", 1, true);
  particle_pose_pub_ = nh_.advertise<geometry_msgs::PoseArray>("particle_poses", 1, true);

  ros::NodeHandle service_nh;
  service_nh.setCallbackQueue(&service_queue_);
  gmap_srv_ = service_nh.advertiseService("gmap_srv", &HierarchyMapper::gmapSrvCb, this);
  map_srv_ = service_nh.advertiseService("map_srv", &HierarchyMapper::mapSrvCb, this);
  map_door_blocked_srv_ = service_nh.advertiseService("map_door_blocked_srv", &HierarchyMapper::mapDoorBlockedSrvCb, this);
  octomap_srv_ = service_nh.advertiseService("octomap_srv", &HierarchyMapper::octomapSrvCb, this);
  obj_map_srv_ = service_nh.advertiseService("obj_map_srv", &HierarchyMapper::objMapSrvCb, this);
  room_type_map_srv_ = service_nh.advertiseService("room_type_map_srv", &HierarchyMapper::roomTypeMapSrvCb, this);
  obj_prob_srv_ = service_nh.advertiseService("obj_prob_srv", &HierarchyMapper::objProbSrvCb, this);
  room_type_prob_srv_ = service_nh.advertiseService("room_type_prob_srv", &HierarchyMapper::roomTypeProbSrvCb, this);
  hierarchy_srv_ = service_nh.advertiseService("hierarchy_srv", &HierarchyMapper::hierarchySrvCb, this);

  ros::NodeHandle("~").param("transform_publish_period", transform_publish_period_, 0.05);
  ros::NodeHandle("~").param("publish_period", publish_period_, 0.5);
  ros::NodeHandle("~").param("debug_publish_interval", debug_publish_interval_, debug_publish_interval_);

  tfB_ = new tf::TransformBroadcaster();
  transform_thread_ = new boost::thread(boost::bind(&HierarchyMapper::transformPublishLoop, this, transform_publish_period_));
  service_spinner_.start();

  std::cout << "HierarchyMapping started" << std::endl;
}

HierarchyMapper::~HierarchyMapper(){
  service_spinner_.stop();
  transform_thread_->join();
  delete transform_thread_;
  for(auto& mapper : room_mapper_)
    delete mapper;
  delete tfB_;

  ROS_ERROR("THIS IS THE END");
  std::cout << "THIS IS THE END" << std::endl;
}


void HierarchyMapper::addMapper(const Door& door){
  boost::unique_lock<boost::mutex> lock(mapper_mutex_);
  if(door.isValid()){
    tf::StampedTransform transform;
    try{
      tf_listener_.lookupTransform("map", "base_laser_link", ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }
    room_mapper_.push_back(new RoomMapper(room_mapper_.size(), &tf_listener_,
                                          GMapping::OrientedPoint(transform.getOrigin().x(),transform.getOrigin().y(),tf::getYaw(transform.getRotation())),
                                          room_mapper_[current_mapper_]->getTransform(), door));
  }
  else{
    tf::StampedTransform transform;
    try{
      tf_listener_.waitForTransform("base_laser_link", "odom", ros::Time::now(), ros::Duration(2.0));
      tf_listener_.lookupTransform("base_laser_link", "odom", ros::Time::now(), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }
    room_mapper_.push_back(new RoomMapper(room_mapper_.size(), &tf_listener_, GMapping::OrientedPoint(0,0,0), transform, door));
  }
  ROS_INFO("Added MAPPER %d", int(room_mapper_.size()) - 1);
  lock.unlock();
  switchMapper(room_mapper_.size() - 1);
}


void HierarchyMapper::switchMapper(int mapper_idx, const Door& door){
  int old_mapper = current_mapper_;
  current_mapper_ = -1;
  if(old_mapper >= 0 && old_mapper < room_mapper_.size())
    room_mapper_[old_mapper]->deactivate();

  tf::Transform door_tf;
  if(mapper_idx >= 0 && mapper_idx < room_mapper_.size()){
    if(door.isValid()){
      tf::StampedTransform transform;
      try{
        tf_listener_.lookupTransform("map", "base_laser_link", ros::Time(0), transform);
      }
      catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
      }
      GMapping::OrientedPoint p(transform.getOrigin().x(), transform.getOrigin().y(), tf::getYaw(transform.getRotation()));
      current_mapper_ = mapper_idx;
      door_tf = room_mapper_[current_mapper_]->activate(p, door);
    }
    else{
      current_mapper_ = mapper_idx;
      room_mapper_[current_mapper_]->activate();
    }
  }

  semantic_mapping_v2::RoomSwitchMsg msg;
  msg.new_room = current_mapper_;
  tf::poseTFToMsg(door_tf, msg.transform);
  if(!map_switch_pub_.getTopic().empty())
    map_switch_pub_.publish(msg);

  last_map_switch_time_ = ros::Time::now();
  ROS_INFO("Switched to MAPPER %d", current_mapper_);
}


void HierarchyMapper::cloudCb(const sensor_msgs::PointCloud2::ConstPtr& cloud){
  if(current_mapper_ >= 0 && current_mapper_ < room_mapper_.size())
    room_mapper_[current_mapper_]->cloudCb(cloud);
}


void HierarchyMapper::laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
  if(current_mapper_ >= 0 && current_mapper_ < room_mapper_.size())
    room_mapper_[current_mapper_]->laserCallback(scan);
}


void HierarchyMapper::doorPoseCb(const geometry_msgs::PoseArray::ConstPtr &msg){
  if(current_mapper_ >= 0 && current_mapper_ < room_mapper_.size())
    room_mapper_[current_mapper_]->doorCb(msg);
}


void HierarchyMapper::visionCb(const vision::VisionMsgConstPtr &msg){
  if(current_mapper_ >= 0 && current_mapper_ < room_mapper_.size())
    room_mapper_[current_mapper_]->visionCb(msg);
}


void HierarchyMapper::transformPublishLoop(double transform_publish_period){
  if(transform_publish_period == 0)
    return;

  ros::Rate r(1.0 / transform_publish_period);
  while(ros::ok()){
    if(current_mapper_ >= 0 && current_mapper_ < room_mapper_.size()){
      tfB_->sendTransform(room_mapper_[current_mapper_]->getTransform());
    }
    r.sleep();
  }
}


//void HierarchyMapper::publish(){
//  ros::Time sdf = ros::Time::now();
//  gmap_pub_.publish(room_mapper_[current_mapper_]->getGMap());
//  nav_msgs::OccupancyGrid map = room_mapper_[current_mapper_]->getMap();
//  if(map.data.size() > 0){
//    map_pub_.publish(map);
//    map_info_pub_.publish(map.info);
//  }
//  map = room_mapper_[current_mapper_]->getDoorBlockedMap();
//  if(map.data.size() > 0){
//    map_door_blocked_pub_.publish(map);
//  }
//  marker_pub_.publish(room_mapper_[current_mapper_]->getOccupiedCellMsg());
//  door_pose_pub_.publish(room_mapper_[current_mapper_]->getDoorPoseMsg());
//  ros::Time sdf2 = ros::Time::now();
//
//  if(obj_prob_pub_.empty()){
//    obj_prob_pub_.resize(80);
//    for(int i=0; i<80; i++){
//      std::string s = "obj_" + ObjectMapper::getObjName(i);
//      std::replace( s.begin(), s.end(), ' ', '_');
//      obj_prob_pub_[i] = nh_.advertise<visualization_msgs::MarkerArray>(s, 1, true);
//    }
//  }
//  for(int i=0; i<80; i++)
//    obj_prob_pub_[i].publish(room_mapper_[current_mapper_]->getObjectProbMsg(i));
//
//  std::vector<size_t> order;
//  std::vector<float> probs = room_mapper_[current_mapper_]->getRoomTypeProbs(order);
//  if(room_prob_pub_.empty() && !room_mapper_[current_mapper_]->getRoomName(1).empty()){
//    int num = room_mapper_[current_mapper_]->getRoomNames().size();
//    room_prob_pub_.resize(num);
//    for(int i=0; i<num; i++){
//      std::string s = "room_" + room_mapper_[current_mapper_]->getRoomName(i);
//      std::replace( s.begin(), s.end(), ' ', '_');
//      room_prob_pub_[i] = nh_.advertise<visualization_msgs::MarkerArray>(s, 1, true);
//    }
//  }
//  if(!room_prob_pub_.empty())
//    for(int i=0; i<room_mapper_[current_mapper_]->getRoomNames().size(); i++)
//      room_prob_pub_[i].publish(room_mapper_[current_mapper_]->getRoomProbMsg(i));
//  if(!probs.empty())
//    for(int i=0; i<probs.size(); i++){
//      std::cout << room_mapper_[current_mapper_]->getRoomName(order[i]) << ": " << probs[order[i]] << std::endl;
//  }
//  std::cout << std::endl;
//  probs = room_mapper_[current_mapper_]->getObjectProbs(order);
//  for(int i=0; i<probs.size(); i++)
//    std::cout << ObjectMapper::getObjName(order[i]) << ": " << probs[order[i]] << std::endl;
//
//  std::cout << "publish time " << (sdf2-sdf).toSec() << " " << (ros::Time::now() - sdf2).toSec() << std::endl;
//}


void HierarchyMapper::downprojecAndPublish(){
  ros::Time start = ros::Time::now();
  room_mapper_[current_mapper_]->downprojectMap();
  gmap_pub_.publish(room_mapper_[current_mapper_]->getGMap());
  nav_msgs::OccupancyGrid map = room_mapper_[current_mapper_]->getMap();
  if(map.data.size() > 0){
    map_pub_.publish(map);
    map_info_pub_.publish(map.info);
  }
  map = room_mapper_[current_mapper_]->getDoorBlockedMap();
  if(map.data.size() > 0){
    map_door_blocked_pub_.publish(map);
  }
  marker_pub_.publish(room_mapper_[current_mapper_]->getOccupiedCellMsg());
  particle_pose_pub_.publish(room_mapper_[current_mapper_]->getParticlePoseMsg());
  door_pose_pub_.publish(room_mapper_[current_mapper_]->getDoorPoseMsg());

  static int counter=0;
  if((counter%debug_publish_interval_) == debug_publish_interval_-1){
    if(obj_prob_pub_.empty()){
      obj_prob_pub_.resize(80);
      for(int i=0; i<80; i++){
        std::string s = "obj_" + ObjectMapper::getObjName(i);
        std::replace( s.begin(), s.end(), ' ', '_');
        obj_prob_pub_[i] = nh_.advertise<visualization_msgs::MarkerArray>(s, 1, true);
      }
    }
    for(int i=0; i<80; i++)
      obj_prob_pub_[i].publish(room_mapper_[current_mapper_]->getObjectProbMsg(i));

    std::vector<size_t> order;
    std::vector<float> probs = room_mapper_[current_mapper_]->getRoomTypeProbs(order);
    if(room_prob_pub_.empty() && !room_mapper_[current_mapper_]->getRoomName(1).empty()){
      int num = room_mapper_[current_mapper_]->getRoomNames().size();
      room_prob_pub_.resize(num);
      for(int i=0; i<num; i++){
        std::string s = "room_" + room_mapper_[current_mapper_]->getRoomName(i);
        std::replace( s.begin(), s.end(), ' ', '_');
        room_prob_pub_[i] = nh_.advertise<visualization_msgs::MarkerArray>(s, 1, true);
      }
    }
    if(!room_prob_pub_.empty())
      for(int i=0; i<room_mapper_[current_mapper_]->getRoomNames().size(); i++)
        room_prob_pub_[i].publish(room_mapper_[current_mapper_]->getRoomProbMsg(i));
    if(!probs.empty()){
      for(int i = 0; i < probs.size(); i++){
        std::cout << room_mapper_[current_mapper_]->getRoomName(order[i]) << ": " << probs[order[i]] << std::endl;
      }
    }
    std::cout << std::endl;
    probs = room_mapper_[current_mapper_]->getObjectProbs(order);
    for(int i=0; i<probs.size(); i++)
      std::cout << ObjectMapper::getObjName(order[i]) << ": " << probs[order[i]] << std::endl;
  }
  counter++;

  std::cout << "Published in " << (ros::Time::now() - start).toSec() << std::endl;
}


void HierarchyMapper::run(){
  ros::Rate rate(50);
  ros::Time last_publish = ros::Time::now();
  while(ros::ok()){
    ros::spinOnce();
    if(current_mapper_ >= 0 && current_mapper_ < room_mapper_.size() && room_mapper_[current_mapper_]->isInitialized()){
//      if(room_mapper_[current_mapper_]->resetWasMapUpdated()){
//        publish();
//        last_publish = ros::Time::now();
//      }
//      else if(ros::Time::now() - last_publish > ros::Duration(0.5)){
//        downprojecAndPublishMap();
//        last_publish = ros::Time::now();
//      }
        if(ros::Time::now() - last_publish > ros::Duration(publish_period_)){
          downprojecAndPublish();
          last_publish = ros::Time::now();
        }

      Door door = room_mapper_[current_mapper_]->droveThroughDoor();
      if(door.isValid() && (ros::Time::now() - last_map_switch_time_).toSec() > MIN_MAP_SWITCH_TIME){
        ROS_INFO("Door %d, c_id: %d, r: %d, c_r: %d", door.getId(), door.getCounterpartId(), door.getRoom(), door.getOtherRoom());
        int other_room = door.getOtherRoom();
        if(other_room < 0){
          int new_id = Door::getID();
          room_mapper_[current_mapper_]->setDoorRoom(door.getId(), room_mapper_.size(), new_id);

          Door counterpart_door(room_mapper_.size(), door.getPose(), current_mapper_, new_id, door.getId(), 5);
          counterpart_door.flipPose();
          ROS_INFO("New Door %d, c_id: %d, r: %d, c_r: %d", counterpart_door.getId(), counterpart_door.getCounterpartId(), counterpart_door.getRoom(), counterpart_door.getOtherRoom());
          addMapper(counterpart_door);
        }
        else
          switchMapper(other_room, door);
      }
    }
    rate.sleep();
  }
  ROS_WARN("OUT OF MAIN LOOP NOW");
  std::cout << "OUT OF MAIN LOOP NOW" << std::endl;
}


bool HierarchyMapper::gmapSrvCb(semantic_mapping_v2::MapSrv::Request& req, semantic_mapping_v2::MapSrv::Response& res){
  ROS_INFO("SERVICE GMAP");
  boost::lock_guard<boost::mutex> maps_lock(mapper_mutex_);
  int id = (req.room_id >= 0 ? req.room_id : current_mapper_);
  if(id < 0 || id >= room_mapper_.size())
    return false;

  res.map = room_mapper_[id]->getGMap();
  return true;
}


bool HierarchyMapper::mapSrvCb(semantic_mapping_v2::MapSrv::Request& req, semantic_mapping_v2::MapSrv::Response& res){
  ROS_INFO("SERVICE MAP");
  boost::lock_guard<boost::mutex> maps_lock(mapper_mutex_);
  int id = (req.room_id >= 0 ? req.room_id : current_mapper_);
  if(id < 0 || id >= room_mapper_.size())
    return false;

  res.map = room_mapper_[id]->getMap();
  return true;
}


bool HierarchyMapper::mapDoorBlockedSrvCb(semantic_mapping_v2::MapSrv::Request &req, semantic_mapping_v2::MapSrv::Response &res){
  ROS_INFO("SERVICE MAP DOOR BLOCKED");
  boost::lock_guard<boost::mutex> maps_lock(mapper_mutex_);
  int id = (req.room_id >= 0 ? req.room_id : current_mapper_);
  if(id < 0 || id >= room_mapper_.size())
    return false;

  res.map = room_mapper_[id]->getDoorBlockedMap();
  return true;
}


bool HierarchyMapper::octomapSrvCb(semantic_mapping_v2::OctomapSrv::Request& req, semantic_mapping_v2::OctomapSrv::Response& res){
  ROS_INFO("SERVICE OCTOMAP");
  boost::lock_guard<boost::mutex> maps_lock(mapper_mutex_);
  int id = (req.room_id >= 0 ? req.room_id : current_mapper_);
  if(id < 0 || id >= room_mapper_.size())
    return false;

  res.map = room_mapper_[id]->getFullOctoMapMsg();
  return true;
}


bool HierarchyMapper::roomTypeMapSrvCb(semantic_mapping_v2::RoomTypeMapSrv::Request& req, semantic_mapping_v2::RoomTypeMapSrv::Response& res){
  ROS_INFO("SERVICE ROOM_MAP");
  boost::lock_guard<boost::mutex> maps_lock(mapper_mutex_);
  int id = (req.room_id >= 0 ? req.room_id : current_mapper_);
  if(id < 0 || id >= room_mapper_.size())
    return false;

  if(req.id < 0)
    res.maps = room_mapper_[id]->getAllRoomTypeMapMsgs();
  else
    res.maps = std::vector<semantic_mapping_v2::RoomTypeMapMsg>({room_mapper_[id]->getRoomTypeMapMsg(req.id)});

  return true;
}


bool HierarchyMapper::roomTypeProbSrvCb(semantic_mapping_v2::RoomTypeProbSrv::Request& req, semantic_mapping_v2::RoomTypeProbSrv::Response& res){
  ROS_INFO("SERVICE ROOM_PROB");
  boost::lock_guard<boost::mutex> maps_lock(mapper_mutex_);
  int id = (req.room_id >= 0 ? req.room_id : current_mapper_);
  if(id < 0 || id >= room_mapper_.size())
    return false;

  std::vector<size_t> order;
  res.probs = room_mapper_[id]->getRoomTypeProbs(order);
  res.names = room_mapper_[id]->getRoomNames();

  return true;
}


bool HierarchyMapper::objMapSrvCb(semantic_mapping_v2::ObjectMapSrv::Request& req, semantic_mapping_v2::ObjectMapSrv::Response& res){
  ROS_INFO("SERVICE OBJ_MAP");
  boost::lock_guard<boost::mutex> maps_lock(mapper_mutex_);
  int id = (req.room_id >= 0 ? req.room_id : current_mapper_);
  if(id < 0 || id >= room_mapper_.size())
    return false;

  if(req.id < 0)
    res.maps = room_mapper_[id]->getAllObjMapMsgs();
  else
    res.maps = std::vector<semantic_mapping_v2::ObjectMapMsg>({room_mapper_[id]->getObjMapMsg(req.id)});

  return true;
}


bool HierarchyMapper::objProbSrvCb(semantic_mapping_v2::ObjectProbSrv::Request& req, semantic_mapping_v2::ObjectProbSrv::Response& res){
  ROS_INFO("SERVICE OBJ_PROB");
  boost::lock_guard<boost::mutex> maps_lock(mapper_mutex_);
  int id = (req.room_id >= 0 ? req.room_id : current_mapper_);
  if(id < 0 || id >= room_mapper_.size())
    return false;

  std::vector<size_t> order;
  res.probs = room_mapper_[id]->getObjectProbs(order);
  res.names = ObjectMapper::getObjNames();

  return true;
}


bool HierarchyMapper::hierarchySrvCb(semantic_mapping_v2::HierarchySrv::Request& req, semantic_mapping_v2::HierarchySrv::Response& res){
  ROS_INFO("SERVICE HIERARCHY");
  boost::lock_guard<boost::mutex> maps_lock(mapper_mutex_);
  ros::Time start = ros::Time::now();
  for(int i=0; i<room_mapper_.size(); i++){
    semantic_mapping_v2::RoomMsg room;
    room.header.stamp = ros::Time::now();
    room.id = i;
    std::vector<size_t> order;
    room.obj_probs_2 = room_mapper_[i]->getObjectProbs(order);
    room.room_type_probs_2 = room_mapper_[i]->getRoomTypeProbs(order);
    room.obj_probs = room_mapper_[i]->getObjectProbsComplete(room.room_type_probs, order);
    res.rooms.push_back(room);
  }

  for(int i=0; i<room_mapper_.size(); i++){
    std::vector<Door> doors = room_mapper_[i]->getDoors();
    for(const auto& door : doors){
      if(door.getOtherRoom() <= i){
        semantic_mapping_v2::HierarchyLinkMsg link;
        link.header.stamp = ros::Time::now();
        if(door.getOtherRoom() < 0){
          link.room1 = i;
          link.room2 = door.getOtherRoom();
        }
        else{
          link.room1 = door.getOtherRoom();
          link.room2 = i;
        }
        tf::poseTFToMsg(door.getPose(), link.door1_pose);
        if(door.getOtherRoom() >= 0){
          std::vector<Door> other_doors = room_mapper_[door.getOtherRoom()]->getDoors();
          for(const auto& door2 : other_doors){
            if(door2.getId() == door.getCounterpartId()){
              tf::poseTFToMsg(door2.getPose(), link.door2_pose);
              break;
            }
          }
        }
        res.links.push_back(link);
      }
    }
  }
  ROS_INFO("SERVICE HIERARCHY IN %.4lf", (ros::Time::now()-start).toSec());
  return true;
}



