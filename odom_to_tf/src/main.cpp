#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

tf::TransformBroadcaster* odom_broadcaster;
std::string odom_topic = "odom";

void odomCb(const nav_msgs::OdometryConstPtr& msg){
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.frame_id = msg->header.frame_id;
  odom_trans.child_frame_id = msg->child_frame_id;
  odom_trans.header.stamp = msg->header.stamp;
  odom_trans.header.seq = msg->header.seq;
  odom_trans.transform.translation.x = msg->pose.pose.position.x;
  odom_trans.transform.translation.y = msg->pose.pose.position.y;
  odom_trans.transform.translation.z = msg->pose.pose.position.z;
  odom_trans.transform.rotation.x = msg->pose.pose.orientation.x;
  odom_trans.transform.rotation.y = msg->pose.pose.orientation.y;
  odom_trans.transform.rotation.z = msg->pose.pose.orientation.z;
  odom_trans.transform.rotation.w = msg->pose.pose.orientation.w;

  odom_broadcaster->sendTransform(odom_trans);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "odom_to_tf");

  if(argc >= 2){
    odom_topic = argv[1];
  }

  ros::NodeHandle n;
  tf::TransformBroadcaster odom_broadcaster_tmp;
  odom_broadcaster = &odom_broadcaster_tmp;

  ros::Subscriber odom_sub = n.subscribe("/"+odom_topic, 1, odomCb);
  ros::Rate rate(20.0);
  while(ros::ok()){
    ros::spinOnce();
    rate.sleep();
  }
}