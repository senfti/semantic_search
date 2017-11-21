//
// Created by thomas on 19.11.17.
//

#include "semantic_mapping_v2/HierarchyMapper.h"


HierarchyMapper::HierarchyMapper(){
  addMapper(Door());

  laser_sub_ = nh_.subscribe("scan", 1, &HierarchyMapper::laserCallback, this);
  cloud_sub_ = nh_.subscribe("/camera/depth_registered/points", 1, &HierarchyMapper::cloudCb, this);
  door_pose_sub_ = nh_.subscribe("door_poses", 1, &HierarchyMapper::doorPoseCb, this);

  map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  gmap_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("gmap", 1, true);
  map_info_pub_ = nh_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
  marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("occupied_cells_vis_array", 1, true);
  door_pose_pub_ = nh_.advertise<geometry_msgs::PoseArray>("mapper_door_poses", 1, true);

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
  room_mapper_.push_back(new RoomMapper(room_mapper_.size()));

  switchMapper(room_mapper_.size() - 1);
}


void HierarchyMapper::switchMapper(int mapper_idx){
  current_mapper_ = mapper_idx;
  if(current_mapper_ >= 0 && current_mapper_ < room_mapper_.size())
    room_mapper_[current_mapper_]->activate();
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
}


void HierarchyMapper::run(){
  ros::Rate rate(50);
  while(ros::ok()){
    ros::spinOnce();
    if(current_mapper_ >= 0 && current_mapper_ < room_mapper_.size()){
      if(room_mapper_[current_mapper_]->resetWasMapUpdated())
        publish();

      Door door = room_mapper_[current_mapper_]->droveThroughDoor();
      std::cout << "DOOOOOOOOOOOOOOOR: " << door.other_room_ << " " << door.this_room_ << " " << door.proposal_count_ << std::endl;
      if(door.isValid()){
        room_mapper_[current_mapper_]->setDoorRoom(door.pose_, room_mapper_.size());
        door.pose_.setRotation(tf::Quaternion(tf::Vector3(0,0,1), door.pose_.getRotation().getAngle() + M_PI));
        addMapper(door);
      }
    }
    rate.sleep();
  }


//  ros::Rate rate(50);
//  ros::Time t = ros::Time::now();
//  int state = 0;
//  while(ros::ok()){
//    ros::spinOnce();
//    if(current_mapper_ >= 0 && current_mapper_ < room_mapper_.size() && room_mapper_[current_mapper_]->resetWasMapUpdated())
//      publish();
//    rate.sleep();
//    if(ros::Time::now().toSec() - t.toSec() > 30 && state == 0){
//      addMapper();
//      ROS_WARN("New mapper");
//      state=1;
//    }
//    if(ros::Time::now().toSec() - t.toSec() > 58 && state == 1){
//      switchMapper(0);
//      ROS_WARN("Back to mapper 0");
//      state=2;
//    }
//    if(ros::Time::now().toSec() - t.toSec() > 88 && state == 2){
//      addMapper();
//      ROS_WARN("New mapper");
//      state=3;
//    }
//    if(ros::Time::now().toSec() - t.toSec() > 114 && state == 3){
//      switchMapper(0);
//      ROS_WARN("Back to mapper 0");
//      state=4;
//    }
//    if(ros::Time::now().toSec() - t.toSec() > 135 && state == 4){
//      switchMapper(1);
//      ROS_WARN("Back to mapper 1");
//      state=5;
//    }
//  }
}

