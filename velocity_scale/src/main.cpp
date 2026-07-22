#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

ros::Publisher vel_pub;
float angular_scale = 1.2;

void velCb(const geometry_msgs::TwistConstPtr& msg){
  geometry_msgs::Twist res = *msg;
  res.angular.z *= angular_scale;
  vel_pub.publish(res);
}

int main(int argc, char** argv){
  ros::init (argc, argv, "velocity_scale");
  ros::NodeHandle nh;
  if(argc >= 2){
    angular_scale = std::stof(argv[1]);
  }
  std::cout << "scale " << angular_scale << std::endl;
  ros::Subscriber vel_sub = nh.subscribe("/base_cmd_vel", 2, velCb);
  vel_pub = nh.advertise<geometry_msgs::Twist>("navigation_velocity_smoother/raw_cmd_vel", 1);
  ros::Rate rate(50);
  while(ros::ok()){
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}

