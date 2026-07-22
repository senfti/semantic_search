#include <ros/ros.h>
#include <execution/ExecuteActionServer.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "execution");
  ExecuteActionServer server;
  server.run();
//  tf::TransformListener l;
//  ros::Rate r(0.5);
//  r.sleep();
//  Searcher s(&l);
//  geometry_msgs::Pose p;
//  std::cout << "Obj nr: ";
//  int sdf;
//  std::cin >> sdf;
//  s.start(sdf, false, p);
//  ros::Rate ra(2.0);
//  while(ros::ok()){
//    ros::spinOnce();
//    s.objFound();
//    s.doCalculations();
//    ra.sleep();
//  }

  return 0;
}