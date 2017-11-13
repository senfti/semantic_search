//
// Created by thomas on 09.11.17.
//

#include "door_detection/DoorDetector.h"
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

DoorDetector::DoorDetector()
  : it_(nh_)
{
  image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, &DoorDetector::imageCb, this);
  //depth_image_sub_ = it_.subscribe("/camera/depth_registered/image_raw", 1, &DoorDetector::depthImageCb, this);
  //cloud_sub_ = nh_.subscribe("/camera/depth_registered/points", 1, &DoorDetector::cloudCb, this);
  laser_sub_ = nh_.subscribe("/scan_filtered", 1, &DoorDetector::laserCb, this);
  //tmp_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/tmp_cloud", 2);
  //image_pub_ = nh_.advertise<sensor_msgs::Image> ("tmp_img", 30);


  tf_listener_.waitForTransform("base_link", "camera_rgb_optical_frame", ros::Time::now(), ros::Duration(2.0));
  tf_listener_.lookupTransform("base_link", "camera_rgb_optical_frame", ros::Time::now(), camera_to_base_transform_);
}

void DoorDetector::imageCb(const sensor_msgs::ImageConstPtr &msg){
  cv_bridge::CvImagePtr cv_ptr;
  try{
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat img;
  cv_ptr->image.copyTo(img);

  cv::imshow("image", img);
}

void visualizeGray(cv::Mat_<double> img, const std::string& win_name, float min, float max){
  cv::Mat out;
  img.convertTo(out, CV_8U, 150.f/(max-min), min);
  cv::Mat tmp(out.rows, out.cols, CV_8UC1, cv::Scalar(255));
  cv::merge(std::vector<cv::Mat>({out, tmp, tmp}), out);
  cv::cvtColor(out, out, CV_HSV2BGR);
  cv::imshow(win_name, out);
}

void DoorDetector::depthImageCb(const sensor_msgs::ImageConstPtr &msg){
  cv_bridge::CvImagePtr cv_ptr;
  try{
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
  }
  catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat gray, edges;
  cv_ptr->image.copyTo(depth_img_);
  visualizeGray(depth_img_, "depth_orig", 0.0, 4.0);

  int close_size = 11;
  cv::Mat_<uchar> bad_values(depth_img_.rows, depth_img_.cols, uchar(0)), bad_closed;
  for(int x=0; x<depth_img_.cols; x++){
    for(int y=0; y<depth_img_.rows; y++){
      if(depth_img_.at<float>(y,x) != depth_img_.at<float>(y,x)){
        bad_values(y,x) = 255;
      }
    }
  }
  cv::erode(bad_values, bad_closed, cv::Mat_<uchar>::ones(2*close_size+1, 2*close_size+1));
  cv::dilate(bad_closed, bad_closed, cv::Mat_<uchar>::ones(2*close_size+1, 2*close_size+1));
  cv::Mat diff = bad_values - bad_closed;
  cv::imshow("bad_values", bad_values);
  cv::imshow("bad_closed", bad_closed);
  cv::imshow("diff", diff);
  for(int x=0; x<depth_img_.cols; x++){
    for(int y=0; y<depth_img_.rows; y++){
      if(bad_values(y,x) && !bad_closed(y,x)){
        float mean = 0.f;
        int num = 0;
        for(int xr=std::max(0,x-close_size); xr<std::min(x+1+close_size, depth_img_.cols); xr++){
          for(int yr=std::max(0,y-close_size); yr<std::min(y+1+close_size, depth_img_.rows); yr++){
            if(!bad_values(yr,xr)){
              mean += depth_img_.at<float>(yr, xr);
              num++;
            }
          }
        }
        mean /= num;
        depth_img_.at<float>(y, x) = mean;
      }
    }
  }
//  double min, max;
//  cv::minMaxIdx(depth_img_, &min, &max);
//  std::cout << min << ", " << max << std::endl;

  //depth_img_.convertTo(gray, CV_8U, 64.0);
//  cv::bitwise_or(cv::Mat(depth_img_ != depth_img_), cv::Mat(depth_img_ > 4.0), sdf);
//  depth_img_.setTo(4, sdf);
  visualizeGray(depth_img_, "depth", 0.0, 4.0);

  cv::Mat grad_x, grad_y, grad;
  cv::Sobel(depth_img_, grad_x, -1, 1, 0, 1);
  cv::Sobel(depth_img_, grad_y, -1, 0, 1, 1);
  cv::sqrt(grad_x.mul(grad_x)+grad_y.mul(grad_y), grad);
  edges = cv::Mat(grad > 0.5);
  cv::imshow("edges", edges);
  cv::waitKey(1);


//  depth_img_.convertTo(gray, CV_8U, 64.0);
//  cv::medianBlur(gray, gray, 7);
//
//  float median = 0;
//  cv::Mat hist;
//  const float tmp[] = {0, 255};
//  const float *ranges = {tmp};
//  const int n_channels = 0;
//  const int hsize = 256;
//  cv::calcHist(&gray, 1, &n_channels, cv::Mat(), hist, 1, &hsize, &ranges);
//  normalize(hist, hist, 1, 0, cv::NORM_L1);
//  float cdf = 0;
//  for (int i = 1; i < 256; i++) {
//    if (cdf > 0.5) {
//      median = i;
//      break;
//    }
//    cdf += hist.at<float>(i);
//  }
//  double lower = std::max(0.0, (1.0 - 0.33) * median);
//  double upper = std::min(255.0, (1.0 + 0.33) * median);
//  cv::Canny(gray, edges, lower, upper);
//
//  cv::imshow("edges", edges);
//  cv::waitKey(1);
}

void DoorDetector::cloudCb(const sensor_msgs::PointCloud2ConstPtr &msg){
  pcl::PointCloud<pcl::PointXYZRGB> pc; // input cloud for filtering and ground-detection
  pcl::fromROSMsg(*msg, pc);

  Eigen::Matrix4f cam_to_base;
  pcl_ros::transformAsMatrix(camera_to_base_transform_, cam_to_base);
  pcl::transformPointCloud(pc, pc, cam_to_base);

  sensor_msgs::PointCloud2 tmp;
  pcl::toROSMsg(pc, tmp);
  tmp.header.frame_id = "base_link";
  tmp_cloud_pub_.publish(tmp);

  std::vector<cv::Mat_<uchar>> imgs(14);
  for(int i=0; i<14; i++)
    imgs[i] = cv::Mat_<uchar>(160,160,uchar(0));

  for(auto it=pc.begin(); it!=pc.end(); ++it){
    if(it->x < 4.0 && it->x > -4.0 && it->y < 4.0 && it->y > -4.0 && it->z < 2.79 && it->z > 0.0){
      imgs[it->z*5.0](80-(it->y)*19, 80+(it->x)*19) = 255;
    }
  }

  for(int i=0; i<14; i++){
    cv::resize(imgs[i], imgs[i], cv::Size(320, 320), 0, 0, cv::INTER_NEAREST);
    cv::imshow("pointcloud_" + std::to_string(i), imgs[i]);
    cv::moveWindow("pointcloud_" + std::to_string(i), 380*((i+1)%5),350*((i+1)/5));
  }
  cv::waitKey(1);

  sensor_msgs::Image image;
  pcl::toROSMsg(pc, image);
  image_pub_.publish(image);
}

int getVotes(float posx, float posy, float angle, float width, const sensor_msgs::LaserScanConstPtr &msg){
  int votes = 0;
  for(int i=0; i<msg->ranges.size(); i++){
    float a = msg->angle_min + msg->angle_increment*i + angle;
    float x = std::abs(msg->ranges[i]*std::sin(a) + posx);
    float y = std::abs(msg->ranges[i]*std::cos(a) + posy);
    votes += (x > width - 0.015 && x < width + 0.015 && y < 0.1);
  }
  return votes;
}

void DoorDetector::laserCb(const sensor_msgs::LaserScanConstPtr &msg){
//  int oposite_offset = std::round(M_PI / msg->angle_increment);
//  int min_idx=0;
//  float min_dist = 999999999999.9f;
//  for(int i=0; i<msg->ranges.size()-oposite_offset; i++){
//    if(msg->ranges[i] < msg->range_max && msg->ranges[i+oposite_offset] < msg->range_max){
//      float dist = msg->ranges[i] + msg->ranges[i + oposite_offset];
//      if(dist < min_dist){
//        min_dist = dist;
//        min_idx = i;
//      }
//    }
//  }

  int max_votes = 0, xb, yb, ab, wb;
  for(float x=-0.5; x<=0.5; x+=0.02){
    for(float y=-0.2; y<=0.2; y+=0.02){
      for(float angle=-M_PI/6; angle <= M_PI/6; angle+=M_PI/90){
        for(float width=1.0; width <= 1.5; width+=0.01){
          int votes = getVotes(x,y,angle, width, msg);
          if(votes > max_votes){
            max_votes = votes;
            xb=x; yb=y; ab=angle; wb=width;
          }
        }
      }
    }
    std::cout << x << std::endl;
  }

  cv::Mat_<cv::Vec3b> img(160,160, cv::Vec3b(0,0,0));
  cv::RotatedRect rect(cv::Point(yb+wb*std::sin(ab), xb+wb*std::cos(ab)), cv::Size(20,1), ab);
  cv::Point2f p[4];
  rect.points(p);
  cv::line(img, p[0], p[1], cv::Scalar(255,0,0));
  for(int i=0; i<msg->ranges.size(); i++){
    if(msg->ranges[i] >= msg->range_min && msg->ranges[i] <= msg->range_max){
      float angle = msg->angle_min+msg->angle_increment*i;
      img(80-msg->ranges[i]*std::sin(angle)*19, 80+msg->ranges[i]*std::cos(angle)*19) = cv::Vec3b(0,0,255);
    }
  }
//  img(80-msg->ranges[min_idx]*std::sin(msg->angle_min+msg->angle_increment*min_idx)*19, 80+msg->ranges[min_idx]*std::cos(msg->angle_min+msg->angle_increment*min_idx)*19) = cv::Vec3b(0,255,0);
//  min_idx += oposite_offset;
//  img(80-msg->ranges[min_idx]*std::sin(msg->angle_min+msg->angle_increment*min_idx)*19, 80+msg->ranges[min_idx]*std::cos(msg->angle_min+msg->angle_increment*min_idx)*19) = cv::Vec3b(0,255,0);

  cv::resize(img, img, cv::Size(640, 640), 0, 0, cv::INTER_NEAREST);
  cv::imshow("laser scan", img);
  cv::moveWindow("laser scan", 0, 0);
  cv::waitKey(1);
}

void DoorDetector::run(){
  ros::Rate rate(2);
  while(ros::ok()){
    ros::spinOnce();
    rate.sleep();
  }
}
