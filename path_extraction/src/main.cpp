//
// Created by thomas on 07.05.18.
//

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include <std_msgs/Int8.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <semantic_mapping_v2/RoomSwitchMsg.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <opencv2/opencv.hpp>

int getRoom(ros::Time time, const std::vector<semantic_mapping_v2::RoomSwitchMsg>& msg, const std::vector<ros::Time>& msg_times){
  int room = 0;
  for(int i=0; i<msg.size() && time > msg_times[i]; i++)
    room = msg[i].new_room;

  return room;
}

int getAction(ros::Time time, const std::vector<std_msgs::Int8>& msg, const std::vector<ros::Time>& msg_times){
  int room = 0;
  for(int i=0; i<msg.size() && time > msg_times[i]; i++)
    room = msg[i].data;

  return room;
}

void drawLine(cv::Mat& map, const nav_msgs::OccupancyGrid& m, geometry_msgs::Pose p1, geometry_msgs::Pose p2, cv::Vec4b color){
  cv::Point start((p1.position.x-m.info.origin.position.x)/m.info.resolution, (p1.position.y-m.info.origin.position.y)/m.info.resolution);
  cv::Point end((p2.position.x-m.info.origin.position.x)/m.info.resolution, (p2.position.y-m.info.origin.position.y)/m.info.resolution);
  cv::line(map, start, end, color, 2);
}

void markPose(cv::Mat& map, const nav_msgs::OccupancyGrid& m, geometry_msgs::Pose p, cv::Vec4b color){
  cv::Point start((p.position.x-m.info.origin.position.x)/m.info.resolution, (p.position.y-m.info.origin.position.y)/m.info.resolution);
  tf::Transform ptf;
  tf::poseMsgToTF(p, ptf);
  double angle = tf::getYaw(ptf.getRotation());
  cv::Point end = start + cv::Point(70*std::cos(angle), 70*std::sin(angle));
  cv::arrowedLine(map,start, end,color,4,8,0,0.3);
}

//void oneRoom(){
//  rosbag::Bag bag;
//  std::cout << "Filename: ";
//  std::string filename;
//  std::cin >> filename;
//  filename = "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/"+filename+".bag",;
//  std::cout << filename << std::endl;
//  bag.open(filename, rosbag::bagmode::Read);
//
//  std::vector<std::string> topics;
//  topics.push_back("/robot_path");
//  topics.push_back("/map");
//  topics.push_back("/current_action");
//  topics.push_back("/map_switch");
//  rosbag::View msgs(bag, rosbag::TopicQuery(topics));
//
//  std::vector<std_msgs::Int8> actions;
//  std::vector<ros::Time> action_times;
//  std::vector<nav_msgs::OccupancyGrid> maps;
//  std::vector<nav_msgs::Path> paths;
//  std::vector<semantic_mapping_v2::RoomSwitchMsg> room_switches;
//  std::vector<ros::Time> switch_times;
//
//  foreach(rosbag::MessageInstance const m, msgs){
//        std_msgs::Int8::ConstPtr s = m.instantiate<std_msgs::Int8>();
//        if(s != NULL){
//          actions.push_back(*s);
//          action_times.push_back(m.getTime());
//        }
//        nav_msgs::OccupancyGrid::ConstPtr s2 = m.instantiate<nav_msgs::OccupancyGrid>();
//        if(s2 != NULL)
//          maps.push_back(*s2);
//        nav_msgs::Path::ConstPtr s3 = m.instantiate<nav_msgs::Path>();
//        if(s3 != NULL)
//          paths.push_back(*s3);
//          semantic_mapping_v2::RoomSwitchMsg::ConstPtr s4 = m.instantiate<semantic_mapping_v2::RoomSwitchMsg>();
//        if(s4 != NULL){
//          room_switches.push_back(*s4);
//          switch_times.push_back(m.getTime());
//        }
//      }
//  bag.close();
//
//  nav_msgs::OccupancyGrid m = maps.back();
//  cv::Mat_<cv::Vec3b> map(m.info.height, m.info.width, cv::Vec3b(255,255,255));
//  for(int x=0; x<map.cols; x++){
//    for(int y=0; y<map.rows; y++){
//      if(m.data[x+y*m.info.width]==0){
//        map(y, x) = cv::Vec3b(230, 230, 230);
//      }
//      if(m.data[x+y*m.info.width]>0){
//        map(y, x) = cv::Vec3b(0, 0, 0);
//      }
//    }
//  }
//  cv::resize(map, map, cv::Size(map.cols*4, map.rows*4),0,0,cv::INTER_NEAREST);
//  m.info.resolution /= 4.0;
//
//  std::vector<geometry_msgs::PoseStamped> path = paths.back().poses;
//  int action_idx = 0;
//  int type = -1;
//  int old_type = -2;
//  geometry_msgs::Pose old_pose;
//  geometry_msgs::Pose end_pose;
//  for(const auto& p : path){
//    for(int i=action_idx; i<actions.size() && action_times[i] < p.header.stamp; i++){
//      action_idx++;
//      type = actions[i].data;
//    }
//    if(type==2 && old_type==2)
//      drawLine(map, m, old_pose, p.pose, cv::Vec3b(0,255,0));
//    else if(type==old_type)
//      drawLine(map, m, old_pose, p.pose, cv::Vec3b(255,0,0));
//    if(type==2)
//      end_pose = p.pose;
//    old_pose = p.pose;
//    old_type = type;
//  }
//  markPose(map, m, path.front().pose, cv::Vec3b(0,0,255));
//  markPose(map, m, end_pose, cv::Vec3b(255,0,255));
//
//  int minx = m.info.width, maxx=0, miny=m.info.height, maxy=0;
//  for(int x=0; x<map.cols; x++){
//    for(int y=0; y<map.rows; y++){
//      if(map(y, x) != cv::Vec3b(255,255,255)){
//        if(minx > x)
//          minx = x;
//        if(maxx < x)
//          maxx = x;
//        if(miny > y)
//          miny = y;
//        if(maxy < y)
//          maxy = y;
//      }
//    }
//  }
//  map = map(cv::Rect(cv::Point(minx, miny), cv::Point(maxx,maxy)));
//  cv::flip(map,map,0);
//  cv::imshow("map", map);
//  cv::waitKey(0);
//}

void multiRoom(std::string name){
  rosbag::Bag bag;
  std::string filename = "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/output"+name+".bag";
  std::cout << filename << std::endl;
  bag.open(filename, rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.push_back("/robot_path");
  topics.push_back("/map");
  topics.push_back("/current_action");
  topics.push_back("/map_switch");
  rosbag::View msgs(bag, rosbag::TopicQuery(topics));

  std::vector<std_msgs::Int8> actions;
  std::vector<ros::Time> action_times;
  std::vector<nav_msgs::OccupancyGrid> maps;
  std::vector<nav_msgs::Path> paths;
  std::vector<semantic_mapping_v2::RoomSwitchMsg> room_switches;
  std::vector<ros::Time> switch_times;

    foreach(rosbag::MessageInstance const m, msgs){
          std_msgs::Int8::ConstPtr s = m.instantiate<std_msgs::Int8>();
          if(s != NULL){
            actions.push_back(*s);
            action_times.push_back(m.getTime());
          }
          nav_msgs::OccupancyGrid::ConstPtr s2 = m.instantiate<nav_msgs::OccupancyGrid>();
          if(s2 != NULL)
            maps.push_back(*s2);
          nav_msgs::Path::ConstPtr s3 = m.instantiate<nav_msgs::Path>();
          if(s3 != NULL)
            paths.push_back(*s3);
          semantic_mapping_v2::RoomSwitchMsg::ConstPtr s4 = m.instantiate<semantic_mapping_v2::RoomSwitchMsg>();
          if(s4 != NULL){
            room_switches.push_back(*s4);
            switch_times.push_back(m.getTime());
          }
        }
  bag.close();

  std::vector<nav_msgs::OccupancyGrid> ms;
  for(const auto& m : maps){
    int room = getRoom(m.header.stamp, room_switches, switch_times);
    if(ms.size() <= room)
      ms.push_back(m);
    else
      ms[room] = m;
  }
  for(int r=0; r<ms.size(); r++){
    auto m = ms[r];
    cv::Mat_<cv::Vec4b> map(m.info.height, m.info.width, cv::Vec4b(255,255,255,0));
    for(int x=0; x<map.cols; x++){
      for(int y=0; y<map.rows; y++){
        if(m.data[x+y*m.info.width]==0){
          map(y, x) = cv::Vec4b(230, 230, 230,255);
        }
        if(m.data[x+y*m.info.width]>0){
          map(y, x) = cv::Vec4b(0, 0, 0,255);
        }
      }
    }
    cv::resize(map, map, cv::Size(map.cols*8, map.rows*8),0,0,cv::INTER_NEAREST);
    m.info.resolution /= 8.0;

    std::vector<geometry_msgs::PoseStamped> path = paths.back().poses;
    int old_action = -2;
    int old_room = -1;
    geometry_msgs::Pose old_pose;
    geometry_msgs::Pose end_pose;
    for(const auto& p : path){
      int action = getAction(p.header.stamp, actions, action_times);
      int room = getRoom(p.header.stamp, room_switches, switch_times);
      if(room==r && old_room == r && action==2 && old_action!=-2)
        drawLine(map, m, old_pose, p.pose, cv::Vec4b(0,255,0,255));
      else if(/*room==r && old_room == r && */action==1 && old_action!=-2)
        drawLine(map, m, old_pose, p.pose, cv::Vec4b(255,0,0,255));
      else if(/*room==r && old_room == r &&*/ action!=1 && action!=2 && old_action!=-2)
        drawLine(map, m, old_pose, p.pose, cv::Vec4b(0,0,255,255));
      if(action==2)
        end_pose = p.pose;
      old_pose = p.pose;
      old_action = action;
      old_room = room;
    }
    //if(r==0)
      //markPose(map, m, path.front().pose, cv::Vec4b(255,255,0,255));
    //if(room_switches.empty() || room_switches.back().new_room==r)
      //markPose(map, m, path.back().pose, cv::Vec4b(255,0,255,255));

    int minx = m.info.width, maxx=0, miny=m.info.height, maxy=0;
    for(int x=0; x<map.cols; x++){
      for(int y=0; y<map.rows; y++){
        if(map(y, x) != cv::Vec4b(255,255,255,0)){
          if(minx > x)
            minx = x;
          if(maxx < x)
            maxx = x;
          if(miny > y)
            miny = y;
          if(maxy < y)
            maxy = y;
        }
      }
    }
    map = map(cv::Rect(cv::Point(minx, miny), cv::Point(maxx,maxy)));
    cv::flip(map,map,0);
    std::cout << "/home/thomas/Masterarbeit/output/example_runs/" + name + "_" + std::to_string(r) + ".png" << std::endl;
    cv::imwrite("/home/thomas/Masterarbeit/output/example_runs/" + name + "_" + std::to_string(r) + ".png", map);
  }
}

int main(){
  std::vector<std::string> files = {
//    "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/apple_f11.bag",
//      "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/IST_toilet_full_unexp25.bag",
//      "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/IST_toilet_full_unexp26.bag",
//      "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/IST_toilet_full_unexp27.bag",
//      "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/IST_toilet_full_unexp28.bag",
//      "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/IST_toilet_full_unexp29.bag",
//      "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/IST_toilet_full_unexp30.bag",
      "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/output/wzs5_all.bag"//,
//      "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/IST_wine_glass_full_unexp20.bag",
//      "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/IST_wine_glass_full_unexp21.bag",
//      "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/IST_wine_glass_full_unexp22.bag",
//      "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/IST_wine_glass_full_unexp23.bag",
//      "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/IST_wine_glass_full_unexp24.bag",
//      "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/knife_f11.bag",
//      "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/knife_f12.bag",
//      "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/knife_f13.bag",
//      "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/knife_f21.bag",
//      "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/knife_f22.bag",
//      "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/knife_f23.bag",
//      "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/knife_s11.bag",
//      "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/knife_s12.bag",
//      "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/knife_s13.bag",
//      "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/knife_s14.bag",
//      "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/knife_s15.bag",
//      "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/knife_s21.bag",
//      "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/knife_s22.bag",
//      "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/knife_s23.bag",
//      "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/mouse_f1.bag",
//      "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/mouse_f2.bag",
//      "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/mouse_f3.bag",
//      "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/mouse_f4.bag",
//      "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/mouse_f11.bag",
//      "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/mouse_f12.bag",
//      "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/mouse_f13.bag",
//      "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/mouse_s1.bag",
//      "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/mouse_s2.bag",
//      "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/mouse_s3.bag",
//      "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/mouse_s11.bag",
//      "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/mouse_s12.bag",
//      "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/mouse_s13.bag",
//      "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/remote_table_f11.bag",
//      "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/remote_table_f12.bag",
//      "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/remote_table_f13.bag",
//      "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/remote_table_f21.bag",
//      "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/remote_table_f22.bag",
//      "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/remote_table_f23.bag",
//      "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/remote_table_s11.bag",
//      "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/remote_table_s12.bag",
//      "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/remote_table_s13.bag",
//      "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/remote_table_s21.bag",
//      "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/remote_table_s22.bag",
//      "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/remote_table_s23.bag",
//      "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/spoon_f11.bag",
//      "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/spoon_f12.bag",
//      "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/spoon_f13.bag",
//      "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/spoon_f21.bag",
//      "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/spoon_f22.bag",
//      "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/spoon_f23.bag",
//      "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/spoon_s11.bag",
//      "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/spoon_s12.bag",
//      "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/spoon_s13.bag",
//      "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/spoon_s21.bag",
//      "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/spoon_s22.bag",
//      "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/spoon_s23.bag",
//    "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/search_bottle_lab_1.bag",
//    "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/search_microwave_lab_1.bag",
//    "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/search_mouse_lab_1.bag",
//    "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/search_sink_lab_1.bag",
//    "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/lab_1_bowl_kitchen_full_unexplored.bag",
//    "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/lab_2_bowl_kitchen_full_unexplored.bag",
//    "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/lab_3_bowl_kitchen_full_unexplored.bag",
//    "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/lab_4_bowl_kitchen_full_unexplored.bag",
//    "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/lab_5_bowl_kitchen_full_unexplored.bag",
//    "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/lab_6_bowl_kitchen_full_unexplored.bag",
//    "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/lab_7_bowl_kitchen_full_unexplored.bag",
//    "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/lab_8_bowl_kitchen_stupid_exp_unexplored.bag",
//    "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/lab_9_bowl_kitchen_stupid_unexp_unexplored.bag",
//    "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/lab_10_bowl_kitchen_full_unexp_unexplored.bag",
//    "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/lab_11_bowl_kitchen_full_unexp_unexplored.bag",
//    "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/IST_toilet_exp1.bag",
//    "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/IST_toilet_stupid_exp_1.bag",
//    "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/IST_toilet_stupid_exp_2.bag",
//    "/media/thomas/efe87a75-9b65-4f32-bd7d-8ff566ecf8a6/rosbag/IST_wine_glass_exp1.bag"
  };
  for(auto& f : files){
    f = f.substr(f.rfind('/'),f.rfind('.')-f.rfind('/'));
    multiRoom(f);
  }
  return 0;
}