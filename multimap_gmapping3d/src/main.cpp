/*
 * slam_gmapping
 * Copyright (c) 2008, Willow Garage, Inc.
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 * 
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 */

/* Author: Brian Gerkey */

#include <ros/ros.h>

#include "multimap_gmapping3d/HierarchyOctoGMapper.h"

int
main(int argc, char** argv)
{
  std::string params;
  for(int i=1; i<argc; i++)
    params += std::string(argv[i]) + " ";
  ros::init(argc, argv, "slam_gmapping");

  //ROS_WARN("%d, %s", argc, params.c_str());

  HierarchyOctoGMapper mapper;
  ros::Rate rate(50);
  //ros::Time t = ros::Time::now();
  //int state = 5;
  while(ros::ok()){
    //if(t > ros::Time::now())
    //  t = ros::Time::now();
    ros::spinOnce();
    rate.sleep();
//    if(ros::Time::now().toSec() - t.toSec() > 18 && state == 0){
//      mapper.addMapper();
//      ROS_WARN("New mapper");
//      state=1;
//    }
//    if(ros::Time::now().toSec() - t.toSec() > 58 && state == 1){
//      mapper.startMapper(0);
//      ROS_WARN("Back to mapper 0");
//      state=2;
//    }
//    if(ros::Time::now().toSec() - t.toSec() > 88 && state == 2){
//      mapper.addMapper();
//      ROS_WARN("New mapper");
//      state=3;
//    }
//    if(ros::Time::now().toSec() - t.toSec() > 114 && state == 3){
//      mapper.startMapper(0);
//      ROS_WARN("Back to mapper 0");
//      state=4;
//    }
//    if(ros::Time::now().toSec() - t.toSec() > 135 && state == 4){
//      mapper.startMapper(1);
//      ROS_WARN("Back to mapper 1");
//      state=5;
//    }
  }
  ros::spin();

  return(0);
}

