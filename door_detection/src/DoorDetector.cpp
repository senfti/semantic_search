//
// Created by thomas on 09.11.17.
//

#include "door_detection/DoorDetector.h"
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <geometry_msgs/PoseArray.h>
#include <pcl/filters/passthrough.h>

const float PI_180 = M_PI/180.0;

//#define DEBUG_OUTPUT

#define PRINT_PARAM(x) std::cout << #x << " = " << x << " " << std::endl;

DoorDetector::DoorDetector()
{
  ros::NodeHandle private_nh("~");
  private_nh.param("door_detection/MIN_DOOR_HEIGHT", MIN_DOOR_HEIGHT, MIN_DOOR_HEIGHT);
  private_nh.param("door_detection/MAX_DOOR_HEIGHT", MAX_DOOR_HEIGHT, MAX_DOOR_HEIGHT);

  private_nh.param("door_detection/OCC_RES", OCC_RES, OCC_RES);

  private_nh.param("door_detection/FILL_THRESH", FILL_THRESH, FILL_THRESH);
  private_nh.param("door_detection/MIN_WIDTH", MIN_WIDTH, MIN_WIDTH);
  private_nh.param("door_detection/MIN_DEPTH", MIN_DEPTH, MIN_DEPTH);
  private_nh.param("door_detection/MAX_DEPTH", MAX_DEPTH, MAX_DEPTH);

  private_nh.param("door_detection/LASER_IN_DOOR_NARROWING_FACTOR", LASER_IN_DOOR_NARROWING_FACTOR, LASER_IN_DOOR_NARROWING_FACTOR);
  private_nh.param("door_detection/LASER_IN_FRONT_OF_DOOR_NARROWING_FACTOR", LASER_IN_FRONT_OF_DOOR_NARROWING_FACTOR, LASER_IN_FRONT_OF_DOOR_NARROWING_FACTOR);
  private_nh.param("door_detection/LASER_IN_FRONT_OF_DOOR_DEPTH_AREA", LASER_IN_FRONT_OF_DOOR_DEPTH_AREA, LASER_IN_FRONT_OF_DOOR_DEPTH_AREA);
  private_nh.param("door_detection/MAX_LASER_IN_ZONE", MAX_LASER_IN_ZONE, MAX_LASER_IN_ZONE);

  private_nh.param("door_detection/LASER_DOORFRAME_WIDTH", LASER_DOORFRAME_WIDTH, LASER_DOORFRAME_WIDTH);
  private_nh.param("door_detection/LASER_DOORFRAME_DEPTH", LASER_DOORFRAME_DEPTH, LASER_DOORFRAME_DEPTH);
  private_nh.param("door_detection/MIN_LASER_IN_DOORFRAME", MIN_LASER_IN_DOORFRAME, MIN_LASER_IN_DOORFRAME);

  private_nh.param("door_detection/RATE", RATE, RATE);

  private_nh.param("door_detection/MIN_ANGLE_DIFF", MIN_ANGLE_DIFF, MIN_ANGLE_DIFF);
  private_nh.param("door_detection/MIN_DIST_DIFF", MIN_DIST_DIFF, MIN_DIST_DIFF);
  private_nh.param("door_detection/MAX_DISCARD_TIME", MAX_DISCARD_TIME, MAX_DISCARD_TIME);

  PRINT_PARAM(MIN_DOOR_HEIGHT)
  PRINT_PARAM(MAX_DOOR_HEIGHT)
  PRINT_PARAM(OCC_RES)
  PRINT_PARAM(FILL_THRESH)
  PRINT_PARAM(MIN_WIDTH)
  PRINT_PARAM(MIN_DEPTH)
  PRINT_PARAM(MAX_DEPTH)
  PRINT_PARAM(LASER_IN_DOOR_NARROWING_FACTOR)
  PRINT_PARAM(LASER_IN_FRONT_OF_DOOR_NARROWING_FACTOR)
  PRINT_PARAM(LASER_IN_FRONT_OF_DOOR_DEPTH_AREA)
  PRINT_PARAM(MAX_LASER_IN_ZONE)
  PRINT_PARAM(LASER_DOORFRAME_WIDTH)
  PRINT_PARAM(LASER_DOORFRAME_DEPTH)
  PRINT_PARAM(MIN_LASER_IN_DOORFRAME)
  PRINT_PARAM(RATE)
  PRINT_PARAM(MIN_ANGLE_DIFF)
  PRINT_PARAM(MIN_DIST_DIFF)
  PRINT_PARAM(MAX_DISCARD_TIME)

  IMG_SIZE = cv::Size(3*OCC_RES, 4*OCC_RES);
  MIN_WIDTH *= OCC_RES;
  MIN_DEPTH *= OCC_RES;
  MAX_DEPTH *= OCC_RES;
  LASER_IN_FRONT_OF_DOOR_DEPTH_AREA *= OCC_RES;
  LASER_DOORFRAME_WIDTH *= OCC_RES;
  LASER_DOORFRAME_DEPTH *= OCC_RES;

  cloud_sub_ = nh_.subscribe("/door_asus/camera/depth_registered/points", 1, &DoorDetector::cloudCb, this);
  laser_sub_ = nh_.subscribe("/scan_filtered", 1, &DoorDetector::laserCb, this);
  door_pose_pub_ = nh_.advertise<geometry_msgs::PoseArray>("door_poses", 5);

  tf_listener_.waitForTransform("base_link", "door_asus/camera_rgb_optical_frame", ros::Time::now(), ros::Duration(2.0));
  tf_listener_.lookupTransform("base_link", "door_asus/camera_rgb_optical_frame", ros::Time::now(), camera_to_base_transform_);
  tf_listener_.waitForTransform("base_link", "base_laser_link", ros::Time::now(), ros::Duration(2.0));
  tf_listener_.lookupTransform("base_link", "base_laser_link", ros::Time::now(), laser_to_base_transform_);
}


inline double makeBetweenPi(double angle){
  while(angle > M_PI)
    angle -= 2*M_PI;
  while(angle < -M_PI)
    angle += 2*M_PI;
  return angle;
}

bool DoorDetector::useCloud(){
  tf::StampedTransform transform;
  try{
    tf_listener_.lookupTransform("base_link", "odom", ros::Time(0), transform);
  }
  catch (tf::TransformException& ex){
    ROS_ERROR("%s", ex.what());
    return false;
  }

  if(last_used_transform_.frame_id_.empty() || (last_used_transform_.stamp_ - transform.stamp_).toSec() > MAX_DISCARD_TIME){
    last_used_transform_ = transform;
    return true;
  }

  double angle = tf::getYaw(transform.getRotation()) - tf::getYaw(last_used_transform_.getRotation());
  angle = makeBetweenPi(angle);
  double dist = (transform.getOrigin() - last_used_transform_.getOrigin()).length();
  if(std::abs(angle) < MIN_ANGLE_DIFF && dist < MIN_DIST_DIFF){
    std::cout << "NOTHING CHANGED" << std::endl;
    return false;
  }
  last_used_transform_ = transform;
  return true;
}


bool DoorDetector::isInRotatedRect(const pcl::PointXYZ& p, const cv::RotatedRect& rect) const{
  cv::Point2f tmp = pointToPixel(p) - rect.center;
  float x = std::cos(rect.angle*PI_180)*tmp.x + std::sin(rect.angle*PI_180)*tmp.y;
  float y = -std::sin(rect.angle*PI_180)*tmp.x + std::cos(rect.angle*PI_180)*tmp.y;

  return (std::abs(x) <= rect.size.width/2.f && std::abs(y) <= rect.size.height/2.f);
}



cv::Point2f DoorDetector::pointToPixel(const pcl::PointXYZ& p) const{
  return cv::Point2f(p.x*OCC_RES + OCC_RES, p.y*OCC_RES + IMG_SIZE.height/2);
}


geometry_msgs::Point DoorDetector::pixelToPoint(const cv::Point2f& p) const{
  geometry_msgs::Point point;
  point.x = (p.x - OCC_RES) / OCC_RES;
  point.y = (p.y - IMG_SIZE.height/2) / OCC_RES;
  point.z = 0.0;
  return point;
}


geometry_msgs::Pose DoorDetector::rectToPose(const cv::RotatedRect& rect) const{
  geometry_msgs::Pose pose;
  pose.position = pixelToPoint(rect.center);
  float angle = (rect.angle+90)*PI_180;
  angle = makeBetweenPi(angle);
  if(std::abs(angle) > M_PI_2)
    angle += M_PI;
  pose.orientation.w = std::cos(angle/2);
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = std::sin(angle/2);
  return pose;
}

#ifdef DEBUG_OUTPUT
cv::Mat_<cv::Vec3b> tmp(4*50, 3*50, cv::Vec3b(0,0,0));
#endif
cv::RotatedRect DoorDetector::isContourDoor(const std::vector<cv::Point>& contour) const{
  cv::RotatedRect rect = cv::minAreaRect(contour);
  if(rect.size.width < rect.size.height){
    std::swap(rect.size.width, rect.size.height);
    rect.angle += 90;         // OpenCV is in degree
  }
  float w = rect.size.width;
  float h = rect.size.height;

  cv::Mat_<uchar> contour_img(IMG_SIZE, uchar(0));
  cv::drawContours(contour_img, std::vector<std::vector<cv::Point>>({contour}), 0, cv::Scalar(255), -1);
  int contour_pixels = cv::countNonZero(contour_img);

#ifdef DEBUG_OUTPUT
  for(const auto& p : contour)
    tmp(p.y, p.x) = cv::Vec3b(255,255,255);
  cv::drawContours(tmp, std::vector<std::vector<cv::Point>>({contour}), 0, cv::Scalar(255,255,255), -1);
  cv::Point2f rect_points[4]; rect.points(rect_points);
  for(int j = 0; j < 4; j++)
    cv::line(tmp, rect_points[j], rect_points[(j+1)%4], cv::Scalar(0,0,255), 1, 8);

  if(w >= MIN_WIDTH && h >= MIN_DEPTH && h <= MAX_DEPTH && contour_pixels/(w*h) >= FILL_THRESH){
    tmp(rect.center.y, rect.center.x) = cv::Vec3b(255,0,0);
  }
#endif

  //first test if laser scan in door
  int in_zone = 0;
  cv::RotatedRect laser_check_rect(rect);
  laser_check_rect.size.width *= LASER_IN_DOOR_NARROWING_FACTOR;
#ifdef DEBUG_OUTPUT
  laser_check_rect.points(rect_points);
  for(int j = 0; j < 4; j++)
    cv::line(tmp, rect_points[j], rect_points[(j+1)%4], cv::Scalar(0,0,255), 1, 8);
#endif
  for(const auto& p : laser_cloud_)
    in_zone += (isInRotatedRect(p, laser_check_rect) ? 1 : 0);

  laser_check_rect.size.width = rect.size.width * LASER_IN_FRONT_OF_DOOR_NARROWING_FACTOR;
  laser_check_rect.size.height += 2*LASER_IN_FRONT_OF_DOOR_DEPTH_AREA;
  for(const auto& p : laser_cloud_)
    in_zone += (isInRotatedRect(p, laser_check_rect) ? 1 : 0);
#ifdef DEBUG_OUTPUT
  laser_check_rect.points(rect_points);
  for(int j = 0; j < 4; j++)
    cv::line(tmp, rect_points[j], rect_points[(j+1)%4], cv::Scalar(0,0,255), 1, 8);
#endif

  int at_doorframe = 0;
  laser_check_rect.size.width = rect.size.width*LASER_IN_DOOR_NARROWING_FACTOR;
  laser_check_rect.size.height = rect.size.height+LASER_DOORFRAME_DEPTH*2;
  for(const auto& p : laser_cloud_)
    at_doorframe -= (isInRotatedRect(p, laser_check_rect) ? 1 : 0);
  laser_check_rect.size.width = rect.size.width+LASER_DOORFRAME_WIDTH*2;
  for(const auto& p : laser_cloud_)
    at_doorframe += (isInRotatedRect(p, laser_check_rect) ? 1 : 0);
#ifdef DEBUG_OUTPUT
  laser_check_rect.points(rect_points);
  for(int j = 0; j < 4; j++)
    cv::line(tmp, rect_points[j], rect_points[(j+1)%4], cv::Scalar(0,0,255), 1, 8);

  std::cout << in_zone << "____________________" << at_doorframe << "____________________" << contour_pixels << "____________________" << w*h << std::endl;
  if(in_zone <= MAX_LASER_IN_ZONE && at_doorframe >= MIN_LASER_IN_DOORFRAME)
    tmp(rect.center.y, rect.center.x) = cv::Vec3b(0,255,0);
#endif

  if(w >= MIN_WIDTH && h >= MIN_DEPTH && h <= MAX_DEPTH && contour_pixels/(w*h) >= FILL_THRESH && in_zone <= MAX_LASER_IN_ZONE && at_doorframe >= MIN_LASER_IN_DOORFRAME){
    return rect;
  }

  return cv::RotatedRect();
}


void DoorDetector::cloudCb(const sensor_msgs::PointCloud2ConstPtr &msg){
  if(!useCloud())
    return;

  ros::Time start = ros::Time::now();
  pcl::PointCloud<pcl::PointXYZ> cloud, cloud_2m, cloud_under2m; // input cloud for filtering and ground-detection
  pcl::fromROSMsg(*msg, cloud);

  Eigen::Matrix4f cam_to_base;
  pcl_ros::transformAsMatrix(camera_to_base_transform_, cam_to_base);
  pcl::transformPointCloud(cloud, cloud, cam_to_base);

  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud.makeShared());
  pass.setFilterFieldName ("z");
  pass.setFilterLimits(MIN_DOOR_HEIGHT, MAX_DOOR_HEIGHT);
  pass.filter(cloud_2m);

  pass.setFilterLimits(0.f, MIN_DOOR_HEIGHT);
  pass.filter(cloud_under2m);

  cv::Mat_<uchar> occupancy(IMG_SIZE, uchar(0));
  for(const auto& p : cloud_2m.points){
    occupancy(pointToPixel(p)) = 255;
  }
#ifdef DEBUG_OUTPUT
  cv::imshow("occ_orig", occupancy);
#endif

  //for removing the wall and door
  cv::Mat_<uchar> occupancy_under(IMG_SIZE, uchar(0));
  for(const auto& p : cloud_under2m.points){
    occupancy_under(pointToPixel(p)) = 255;
    occupancy(pointToPixel(p)) = 0;
  }
#ifdef DEBUG_OUTPUT
  cv::imshow("occ_under", occupancy_under);
  cv::imshow("occ", occupancy);
  tmp = cv::Mat_<cv::Vec3b>(4*50, 3*50, cv::Vec3b(0,0,0));
  cv::line(tmp, pointToPixel(pcl::PointXYZ(0,0,0)), pointToPixel(pcl::PointXYZ(1,0,0)), cv::Scalar(100,100,100));
  cv::line(tmp, pointToPixel(pcl::PointXYZ(0,-1,0)), pointToPixel(pcl::PointXYZ(0,1,0)), cv::Scalar(100,100,100));
#endif

  cv::dilate(occupancy, occupancy, cv::Mat_<uchar>::ones(3,3), cv::Point(-1,-1), 3);
  cv::erode(occupancy, occupancy, cv::Mat_<uchar>::ones(3,3), cv::Point(-1,-1), 3);
  cv::erode(occupancy, occupancy, cv::Mat_<uchar>::ones(3,3), cv::Point(-1,-1), 3);
  cv::dilate(occupancy, occupancy, cv::Mat_<uchar>::ones(3,3), cv::Point(-1,-1), 3);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(occupancy, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

  static unsigned seq = 0;
  geometry_msgs::PoseArray door_poses;
  door_poses.header.frame_id = "base_link";
  door_poses.header.stamp = msg->header.stamp;
  door_poses.header.seq = seq++;
  for(const auto& contour : contours){
    cv::RotatedRect rect = isContourDoor(contour);
    if(rect.size.area() >= MIN_WIDTH*MIN_DEPTH){
      geometry_msgs::Pose pose = rectToPose(rect);
      if(pose.position.x >= 0)
        door_poses.poses.push_back(pose);
    }
  }
  door_pose_pub_.publish(door_poses);
  std::cout << door_poses.poses.size() << " doors found in " << (ros::Time::now() - start).toSec() << " s" << std::endl;

#ifdef DEBUG_OUTPUT
  for(const auto& p : laser_cloud_){
    cv::Point pi = pointToPixel(p);
    if(cv::Rect(cv::Point(0,0), IMG_SIZE).contains(pi))
      tmp(pi.y, pi.x) = cv::Vec3b(0,255,255);
  }
  cv::resize(tmp, tmp, cv::Size(tmp.cols*4, tmp.rows*4), 0, 0, cv::INTER_NEAREST);
  cv::imshow("rects", tmp);
  cv::waitKey(1);
#endif

}


void DoorDetector::laserCb(const sensor_msgs::LaserScanConstPtr &msg){
  laser_cloud_.clear();
  for(int i=0; i<msg->ranges.size(); i++){
    if(msg->ranges[i] >= msg->range_min && msg->ranges[i] <= msg->range_max){
      float angle = msg->angle_min + msg->angle_increment * i;
      laser_cloud_.push_back(pcl::PointXYZ(msg->ranges[i] * std::cos(angle), msg->ranges[i] * std::sin(angle), 0.f));
    }
  }
  Eigen::Matrix4f laser_to_base;
  pcl_ros::transformAsMatrix(laser_to_base_transform_, laser_to_base);
  pcl::transformPointCloud(laser_cloud_, laser_cloud_, laser_to_base);
}


void DoorDetector::run(){
  ros::Rate rate(RATE);
  while(ros::ok()){
    ros::spinOnce();
    rate.sleep();
  }
}
