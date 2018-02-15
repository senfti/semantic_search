#include <ros/ros.h>
#include <execution_test/ExecuteActionServer.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "execution");
//  ExecuteActionServer server;
//  server.run();
  tf::TransformListener l;
  ros::Rate r(2);
  r.sleep();
  Searcher s(&l);
  geometry_msgs::Pose p;
  s.start(22, false, p);
  ros::Rate ra(5.0);
  while(ros::ok()){
    ros::spinOnce();
//    s.objFound();
//    s.doCalculations();
  }

  return 0;
}