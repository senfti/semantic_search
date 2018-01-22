#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf/transform_datatypes.h>

ros::Publisher filtered_pub;
float min_angle = -1.95, max_angle = 1.95, min_z=0.1, max_z=1.0, max_range=4.0;
float true_angle_min = -1.f, true_angle_max = -1.f, true_increment = -1.f;

std::vector<float> cloud_ranges;
bool got_cloud = false;
tf2_ros::TransformListener* tf2_listener;
tf2_ros::Buffer* tf2_buffer;

ros::Time scan_time;
sensor_msgs::LaserScan filtered;

void scanCb(const sensor_msgs::LaserScanConstPtr& msg){
  filtered = *msg;
  scan_time = msg->header.stamp;
  int min_cutoff = std::max(0, int(std::ceil((min_angle-msg->angle_min)/msg->angle_increment)));
  int max_cutoff = std::min(int(msg->ranges.size()), int(std::floor((max_angle-msg->angle_min)/msg->angle_increment))+1);
  int num_ranges = max_cutoff-min_cutoff;

  true_angle_min = filtered.angle_min = msg->angle_min + min_cutoff*msg->angle_increment;
  true_angle_max = filtered.angle_max = msg->angle_min + max_cutoff*msg->angle_increment;
  true_increment = msg->angle_increment;
  filtered.scan_time = msg->scan_time + min_cutoff*msg->time_increment;
  if(cloud_ranges.empty()){
    cloud_ranges.resize(num_ranges, max_range);
  }

  filtered.ranges = std::vector<float>(num_ranges);
  filtered.intensities = std::vector<float>(num_ranges);
  for(int i=0; i<num_ranges; i++){
    filtered.ranges[i] = msg->ranges[i+min_cutoff];
  }
  if(!msg->intensities.empty()){
    for(int i = 0; i < num_ranges; i++){
      filtered.intensities[i] = msg->intensities[i + min_cutoff];
    }
  }
}

void cloudCb(const sensor_msgs::PointCloud2ConstPtr& msg){
  if(cloud_ranges.empty())
    return;

  for(auto& r : cloud_ranges)
    r = max_range;

  sensor_msgs::PointCloud2 cloud;
  tf2_buffer->transform(*msg, cloud, "base_laser_link", ros::Duration(0.2));
  geometry_msgs::TransformStamped scan_trans;
  geometry_msgs::TransformStamped cloud_trans;
  try{
    scan_trans = tf2_buffer->lookupTransform("base_laser_link", "odom", scan_time, ros::Duration(0.2));
    cloud_trans = tf2_buffer->lookupTransform("base_laser_link", "odom", msg->header.stamp, ros::Duration(0.2));
  }
  catch(tf2::TransformException &ex) {
    ROS_WARN("Could NOT transform %s", ex.what());
    return;
  }
  tf::Transform scan_tf, cloud_tf;
  tf::transformMsgToTF(scan_trans.transform, scan_tf);
  tf::transformMsgToTF(cloud_trans.transform, cloud_tf);
  tf::Transform diff = scan_tf.inverse()*cloud_tf;
  double angle = tf::getYaw(diff.getRotation());

  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x"), iter_y(cloud, "y"), iter_z(cloud, "z"); iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z){
    if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z) || *iter_z > max_z || *iter_z < min_z){
      continue;
    }
    int i = (atan2(*iter_y, *iter_x)-true_angle_min-angle)/true_increment;
    std::cout << angle << std::endl;
    if(i>=0 && i<cloud_ranges.size()){
      cloud_ranges[i] = std::min(cloud_ranges[i], float(hypot(*iter_x, *iter_y))+0.05f);
    }
  }
  for(int i=0; i<cloud_ranges.size(); i++){
    filtered.ranges[i] = std::min(filtered.ranges[i], cloud_ranges[i]);
  }
  filtered_pub.publish(filtered);
}

int main(int argc, char** argv){
  ros::init (argc, argv, "costmap_laser_node");
  ros::NodeHandle nh;
  tf2_ros::Buffer buffer;
  tf2_buffer = &buffer;
  tf2_ros::TransformListener listener(buffer);
  tf2_listener = &listener;
  if(argc >= 6){
    min_angle = std::stof(argv[1]);
    max_angle = std::stof(argv[2]);
    min_z = std::stof(argv[3]);
    max_z = std::stof(argv[4]);
    max_range = std::stof(argv[5]);
  }
  std::cout << "filter between " << min_angle << " and " << max_angle << std::endl;
  std::cout << "filter z between " << min_z << " and " << max_z << std::endl;
  std::cout << "max_range " << max_range << std::endl;
  ros::Subscriber scan_sub = nh.subscribe("/scan", 2, scanCb);
  ros::Subscriber cloud_sub = nh.subscribe("/camera/depth_registered/points", 1, cloudCb);
  filtered_pub = nh.advertise<sensor_msgs::LaserScan>("scan_costmap", 50);

  while(!tf2_buffer->canTransform("base_laser_link", "camera_rgb_optical_frame", ros::Time(0), ros::Duration(1.0)))
    std::cout << "WAIT FOR TRANSFORM" << std::endl;
  while(!tf2_buffer->canTransform("base_laser_link", "camera_rgb_optical_frame", ros::Time(0), ros::Duration(1.0)))
    std::cout << "WAIT FOR TRANSFORM" << std::endl;
  while(!tf2_buffer->canTransform("base_laser_link", "camera_rgb_optical_frame", ros::Time(0), ros::Duration(1.0)))
    std::cout << "WAIT FOR TRANSFORM" << std::endl;

  ros::Rate rate(50);
  while(ros::ok()){
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}

