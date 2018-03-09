#include <ros/ros.h>
#include <execution_test/ExecuteActionServer.h>

std::vector<std::pair<float,float>> params = {
  std::pair<float,float>(0.1,0.001),
//  std::pair<float,float>(0.05,0.00005),
//  std::pair<float,float>(0.05,0.0001),
//  std::pair<float,float>(0.05,0.0002),
//  std::pair<float,float>(0.15,0.00005),
//  std::pair<float,float>(0.15,0.0001),
//  std::pair<float,float>(0.15,0.0002),
//  std::pair<float,float>(0.3,0.00005),
//  std::pair<float,float>(0.3,0.0001),
//  std::pair<float,float>(0.3,0.0002)
};

int main(int argc, char** argv){
  ros::init(argc, argv, "execution");
//
////
  tf::TransformListener l;
//  ros::Rate r(2);
//  r.sleep();
  for(auto& p : params){
    Searcher s(&l);
    s.testrun(p.first, p.second);
  }
  ros::Rate ra(1);
  while(ros::ok()){
    ros::spinOnce();
    ra.sleep();
  }

  return 0;
//
//  ExecuteActionServer server;
//  server.run();

//  tf::TransformListener l;
//  ros::Rate r(2);
//  r.sleep();
//  Searcher s(&l);
//  geometry_msgs::Pose p;
//  s.start(22, false, p);
//  ros::Rate ra(5.0);
//  while(ros::ok()){
//    ros::spinOnce();
////    s.objFound();
////    s.doCalculations();
//  }

  return 0;
}