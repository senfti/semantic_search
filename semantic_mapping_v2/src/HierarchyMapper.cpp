//
// Created by thomas on 19.11.17.
//

#include "semantic_mapping_v2/HierarchyMapper.h"


HierarchyMapper::HierarchyMapper(){
  addMapper(Door());

  laser_sub_ = nh_.subscribe("scan", 1, &HierarchyMapper::laserCallback, this);
  cloud_sub_ = nh_.subscribe("/camera/depth_registered/points", 1, &HierarchyMapper::cloudCb, this);
  door_pose_sub_ = nh_.subscribe("door_poses", 1, &HierarchyMapper::doorPoseCb, this);
  vision_sub_ = nh_.subscribe("vision_result", 1, &HierarchyMapper::visionCb, this);

  map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  gmap_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("gmap", 1, true);
  map_info_pub_ = nh_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
  marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("occupied_cells_vis_array", 1, true);
  door_pose_pub_ = nh_.advertise<geometry_msgs::PoseArray>("mapper_door_poses", 1, true);
  obj_prob_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("obj_prob", 1, true);

  ros::NodeHandle("~").param("transform_publish_period", transform_publish_period_, 0.05);
  tfB_ = new tf::TransformBroadcaster();
  transform_thread_ = new boost::thread(boost::bind(&HierarchyMapper::transformPublishLoop, this, transform_publish_period_));
  std::cout << "HierarchyMapping started" << std::endl;
}

HierarchyMapper::~HierarchyMapper(){
  transform_thread_->join();
  delete transform_thread_;
  for(auto& mapper : room_mapper_)
    delete mapper;
  delete tfB_;
}


void HierarchyMapper::addMapper(const Door& door){
  room_mapper_.push_back(new RoomMapper(room_mapper_.size(), door));
  ROS_INFO("Added MAPPER %d", int(room_mapper_.size()) - 1);
  switchMapper(room_mapper_.size() - 1);
}


void HierarchyMapper::switchMapper(int mapper_idx, const Door& door){
  int old_mapper = current_mapper_;
  current_mapper_ = -1;
  if(old_mapper >= 0 && old_mapper < room_mapper_.size())
    room_mapper_[old_mapper]->deactivate();

  if(mapper_idx >= 0 && mapper_idx < room_mapper_.size()){
    if(door.isValid()){
      tf::StampedTransform transform;
      try{
        tf_listener_.lookupTransform("map", "base_link", ros::Time(0), transform);
      }
      catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
      }
      GMapping::OrientedPoint p(transform.getOrigin().x(), transform.getOrigin().y(), tf::getYaw(transform.getRotation()));
      current_mapper_ = mapper_idx;
      room_mapper_[current_mapper_]->activate(p, door);
    }
    else{
      current_mapper_ = mapper_idx;
      room_mapper_[current_mapper_]->activate();
    }
  }
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


void HierarchyMapper::publish(){
  gmap_pub_.publish(room_mapper_[current_mapper_]->getGMap());
  nav_msgs::OccupancyGrid map = room_mapper_[current_mapper_]->getMap();
  if(map.data.size() > 0){
    map_pub_.publish(map);
    map_info_pub_.publish(map.info);
  }
  marker_pub_.publish(room_mapper_[current_mapper_]->getOccupiedCellMsg());
  door_pose_pub_.publish(room_mapper_[current_mapper_]->getDoorPoseMsg());
  obj_prob_pub_.publish(room_mapper_[current_mapper_]->getObjectProbMsg(56));
}


void HierarchyMapper::downprojecAndPublishMap(){
  room_mapper_[current_mapper_]->downprojectMap();
  nav_msgs::OccupancyGrid map = room_mapper_[current_mapper_]->getMap();
  if(map.data.size() > 0){
    map_pub_.publish(map);
    map_info_pub_.publish(map.info);
  }
  door_pose_pub_.publish(room_mapper_[current_mapper_]->getDoorPoseMsg());
}


void HierarchyMapper::run(){
  ros::Rate rate(50);
  ros::Time last_publish = ros::TIME_MAX;
  while(ros::ok()){
    ros::spinOnce();
    if(current_mapper_ >= 0 && current_mapper_ < room_mapper_.size() && room_mapper_[current_mapper_]->isInitialized()){
      if(room_mapper_[current_mapper_]->resetWasMapUpdated()){
        publish();
        last_publish = ros::Time::now();
      }
      else if(ros::Time::now() - last_publish > ros::Duration(0.5)){
        downprojecAndPublishMap();
        last_publish = ros::Time::now();
      }

      Door door = room_mapper_[current_mapper_]->droveThroughDoor();
      if(door.isValid()){
        ROS_INFO("Door %d, c_id: %d, r: %d, c_r: %d", door.id_, door.counterpart_id_, door.this_room_, door.other_room_);
        int other_room = door.other_room_;
        if(other_room < 0){
          int new_id = Door::getID();
          room_mapper_[current_mapper_]->setDoorRoom(door.pose_, room_mapper_.size(), new_id);

          Door counterpart_door(room_mapper_.size(), door.pose_, std::min(door.proposal_count_, 5), current_mapper_, new_id, door.id_);
          counterpart_door.pose_.setRotation(tf::Quaternion(tf::Vector3(0,0,1), door.pose_.getRotation().getAngle() + M_PI));
          ROS_INFO("New Door %d, c_id: %d, r: %d, c_r: %d", counterpart_door.id_, counterpart_door.counterpart_id_, counterpart_door.this_room_, counterpart_door.other_room_);
          addMapper(counterpart_door);
        }
        else
          switchMapper(other_room, door);
      }
    }
    rate.sleep();
  }
}

