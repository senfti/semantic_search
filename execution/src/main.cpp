#include <ros/ros.h>
#include <execution/ExecuteActionServer.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "execution");
//  ExecuteActionServer server;
//  server.run();
  tf::TransformListener l;
  ros::Rate r(0.5);
  r.sleep();
  Searcher s(56, 0, &l);

  ros::Rate ra(2.0);
  while(ros::ok()){
    ros::spinOnce();
    s.objFound();
    ra.sleep();
  }

  return 0;
}