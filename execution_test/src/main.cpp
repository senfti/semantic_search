#include <ros/ros.h>
#include <execution_test/ExecuteActionServer.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "execution_test");
//  ExecuteActionServer server;
//  server.run();

  tf::TransformListener l;
  ros::Rate r(2);
  r.sleep();
  Searcher s(&l);
  geometry_msgs::Pose p;
  s.start(25, 0);
  ros::Rate ra(5.0);
  int bla = 1;
  while(ros::ok()){
    ros::spinOnce();
    //s.objFound();
//    if(bla%16 == 0)
//      s.publishMaps();
    s.doCalculations(true);
  }

//  Explorer ex(&l);
//  ros::Rate ra(5.0);
//  ros::spinOnce();
//  ra.sleep();
//  ros::spinOnce();
//  ra.sleep();
//  ros::spinOnce();
//  ra.sleep();
//  ros::spinOnce();
//  ra.sleep();
//  ros::spinOnce();
//  ra.sleep();
//  ros::spinOnce();
//  ra.sleep();
//  ros::spinOnce();
//  ra.sleep();
//  ros::spinOnce();
//  ra.sleep();
//  ros::spinOnce();
//  ra.sleep();
//
//  ex.start(31);
//  while(ros::ok()){
//    ros::spinOnce();
//    ra.sleep();
//  }

  return 0;
}