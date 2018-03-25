#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

ros::Publisher filtered_pub;
float min_angle = -1.95, max_angle = 1.95;

void scanCb(const sensor_msgs::LaserScanConstPtr& msg){
  static int count=0;
  sensor_msgs::LaserScan filtered = *msg;
  int min_cutoff = std::max(0, int(std::ceil((min_angle-msg->angle_min)/msg->angle_increment)));
  int max_cutoff = std::min(int(msg->ranges.size()), int(std::floor((max_angle-msg->angle_min)/msg->angle_increment))+1);
  int num_ranges = max_cutoff-min_cutoff;

  filtered.angle_min = msg->angle_min + min_cutoff*msg->angle_increment;
  filtered.angle_max = msg->angle_min + max_cutoff*msg->angle_increment;
  filtered.scan_time = msg->scan_time + min_cutoff*msg->time_increment;

  filtered.ranges = std::vector<float>(num_ranges);
  filtered.intensities = std::vector<float>(num_ranges);
  for(int i=0; i<num_ranges; i++){
    bool neighbor_in = (i>0 && msg->ranges[i+min_cutoff-1] < 4.0) || (i<num_ranges-1 && msg->ranges[i+1+min_cutoff] < 4.0) || i==0 || i==num_ranges-1;
    filtered.ranges[i] = ((msg->ranges[i+min_cutoff] < 4.0 || neighbor_in || count!=0) ? msg->ranges[i+min_cutoff] : 4.0);
  }
  if(!msg->intensities.empty()){
    for(int i = 0; i < num_ranges; i++){
      filtered.intensities[i] = msg->intensities[i + min_cutoff];
    }
  }
  filtered_pub.publish(filtered);
  count = (count+1)%10;
}

int main(int argc, char** argv){
  ros::init (argc, argv, "my_laser_filter");
  ros::NodeHandle nh;
  if(argc >= 3){
    min_angle = std::stof(argv[1]);
    max_angle = std::stof(argv[2]);
  }
  std::cout << "filter between " << min_angle << " and " << max_angle << std::endl;
  ros::Subscriber scan_sub = nh.subscribe("/scan", 10, scanCb);
  filtered_pub = nh.advertise<sensor_msgs::LaserScan>("scan_filtered", 50);

  ros::Rate rate(50);
  while(ros::ok()){
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}

