#include <vector>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <opencv2/opencv.hpp>

bool run = true;
int idx = 0;
int old_idx = -1;

void input(){
  while(run) {
    if(std::cin.get() != 'e')
      idx = idx+1;
    else
      run = false;
  }
}

int main(int argc, char** argv){
  ros::init (argc, argv, "offline_test");
  ros::NodeHandle nh;
  ros::Rate rate(10);

  ros::Publisher pub_rgb = nh.advertise<sensor_msgs::Image>("camera/rgb/image_raw", 1);

  sensor_msgs::Image rgb_msg;
  while(run){
    if(old_idx != idx){
        cv::Mat rgb;
        rgb = cv::imread("/home/thomas/Schreibtisch/test_images/uni/office-13.jpg", CV_LOAD_IMAGE_UNCHANGED);
        if(rgb.empty()){
          std::cout << "Cannot load image!" << std::endl;
          return -1;
        }
        static int seq = 1;
        cv_bridge::CvImage msg;
        msg.header.frame_id = "camera_rgb_optical_frame";
        msg.header.seq = seq++;
        msg.header.stamp = ros::Time::now();

        msg.encoding = sensor_msgs::image_encodings::BGR8;
        msg.image = rgb;
        rgb_msg =*msg.toImageMsg();

        old_idx = idx;
    }
    pub_rgb.publish(rgb_msg);
    rate.sleep();
  }
  
  return 0;
}
