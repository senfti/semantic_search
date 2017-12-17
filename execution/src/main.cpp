#include <ros/ros.h>
#include <execution/ExecuteActionServer.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "execution");
  ExecuteActionServer server;
  server.run();

  return 0;
}