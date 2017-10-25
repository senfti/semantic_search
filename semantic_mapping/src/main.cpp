#include "semantic_mapping/main.h"
#include "semantic_mapping/functions.h"
#include <geometry_msgs/PoseArray.h>
#include <kb_query/KbQuerySrv.h>
#include <fstream>
#include <chrono>
#include <cv_bridge/cv_bridge.h>

const std::string obj_label_file = "/home/thomas/darknet/data/coco.names";
const std::string place_label_file = "/home/thomas/BVLCcaffe/models/googlenet_places205/categories_places205.csv";


void visualizeProbMap(cv::Mat_<double> map, const std::string& win_name){
  cv::Mat out;
  map.convertTo(out, CV_8U, 150.f);
  cv::flip(out, out, 0);
  cv::Mat tmp(out.rows, out.cols, CV_8UC1, cv::Scalar(255));
  cv::merge(std::vector<cv::Mat>({out, tmp, tmp}), out);
  cv::cvtColor(out, out, CV_HSV2BGR);
  cv::imshow(win_name, out);
}



SemanticMappingApp::SemanticMappingApp(ros::NodeHandle& nh)
  : nh_(nh)
{
  vision_sub_ = nh_.subscribe("vision_result", 100, &SemanticMappingApp::visionCb, this);
  map_sub_ = nh_.subscribe("map", 10, &SemanticMappingApp::mapCb, this);
  entropy_sub_ = nh_.subscribe("/slam_gmapping/entropy", 1, &SemanticMappingApp::entropyCb, this);
  vision_pose_pub_ = nh_.advertise<geometry_msgs::PoseArray>("vision_poses", 2);
  obj_map_pub_ = nh_.advertise<prob_map_view::ProbMapMsg>("object_maps", 2);
  place_map_pub_ = nh_.advertise<prob_map_view::ProbMapMsg>("place_maps", 2);

  std::ifstream labels(obj_label_file);
  if(!labels.good()){
    std::cout << "Unable to open labels file " << obj_label_file;
    return;
  }
  std::string line;
  while (std::getline(labels, line))
    object_labels_.push_back(line);

  std::ifstream labels2(place_label_file);
  if(!labels2.good()){
    std::cout << "Unable to open labels file " << place_label_file;
    return;
  }
  while (std::getline(labels2, line))
    place_labels_.push_back(line);

  object_maps_.resize(object_labels_.size());
  for(int i=0; i<object_maps_.size(); i++){
    object_prior_p_[i] = OBJ_MAP_PRIOR;
    object_prior_l_[i] = pToL(object_prior_p_[i]);
    object_maps_[i] = cv::Mat_<double>(MAP_BASE_SIZE,MAP_BASE_SIZE, object_prior_p_[i]);
  }
  place_maps_.resize(NUM_PLACE_TYPES);
  for(int i=0; i<place_maps_.size(); i++){
    place_prior_[i] = 1.f/NUM_PLACE_TYPES;
    place_maps_[i] = cv::Mat_<double>(MAP_BASE_SIZE,MAP_BASE_SIZE, place_prior_[i]);
  }
  origin_ = cv::Point(object_maps_[0].cols/2, object_maps_[0].rows/2);
}


SemanticMappingApp::~SemanticMappingApp(){
}

void SemanticMappingApp::run(){
  ros::Rate rate(4);
  while(ros::ok()){
    ros::spinOnce();
    rate.sleep();
  }
}


std::vector<cv::Point> SemanticMappingApp::getCornersFromDetection(const Object &obj, const cv::Point2d& position, double direction){
  std::vector<cv::Point> corners(4);

  corners[0] = (cv::Point2d(std::round(position.x + obj.x1_*std::cos(direction) - obj.z1_*std::sin(direction)),
                           std::round(position.y + obj.x1_*std::sin(direction) + obj.z1_*std::cos(direction))) * OBJ_PIXEL_PER_METER + cv::Point2d(origin_)) * PROB_DRAW_OVERSAMPLING_FACTOR;
  corners[1] = (cv::Point2d(std::round(position.x + obj.x1_*std::cos(direction) - obj.z2_*std::sin(direction)),
                           std::round(position.y + obj.x1_*std::sin(direction) + obj.z2_*std::cos(direction))) * OBJ_PIXEL_PER_METER + cv::Point2d(origin_)) * PROB_DRAW_OVERSAMPLING_FACTOR;
  corners[2] = (cv::Point2d(std::round(position.x + obj.x2_*std::cos(direction) - obj.z2_*std::sin(direction)),
                           std::round(position.y + obj.x2_*std::sin(direction) + obj.z2_*std::cos(direction))) * OBJ_PIXEL_PER_METER + cv::Point2d(origin_)) * PROB_DRAW_OVERSAMPLING_FACTOR;
  corners[3] = (cv::Point2d(std::round(position.x + obj.x2_*std::cos(direction) - obj.z2_*std::sin(direction)),
                           std::round(position.y + obj.x2_*std::sin(direction) + obj.z2_*std::cos(direction))) * OBJ_PIXEL_PER_METER + cv::Point2d(origin_)) * PROB_DRAW_OVERSAMPLING_FACTOR;

  return corners;
}


std::vector<cv::Mat_<double>> SemanticMappingApp::generateDetectionProbMaps(const VisionResult &vision_result, const cv::Mat_<double> &seen_map){
  std::vector<cv::Mat_<double>> detection_prob_maps(vision_result.objects_.size(), cv::Mat_<double>(seen_map.rows*PROB_DRAW_OVERSAMPLING_FACTOR, seen_map.cols*PROB_DRAW_OVERSAMPLING_FACTOR, 0.f));
  cv::Point2d pos(vision_result.pose_.getOrigin().getX(), vision_result.pose_.getOrigin().getY());
  double dir = std::atan2(vision_result.pose_.getBasis().getColumn(0).getY(), vision_result.pose_.getBasis().getColumn(0).getX());
  double pose_sigma = pose_sigma_ * OBJ_PIXEL_PER_METER * PROB_DRAW_OVERSAMPLING_FACTOR;
  int gauss_kernel_size = std::round(3*pose_sigma);
  for(int i=0; i<detection_prob_maps.size(); i++){
    cv::fillConvexPoly(detection_prob_maps[i], getCornersFromDetection(vision_result.objects_[i], pos, dir), cv::Scalar(1.0));
    cv::GaussianBlur(detection_prob_maps[i], detection_prob_maps[i], cv::Size(2*gauss_kernel_size+1, 2*gauss_kernel_size+1), pose_sigma, pose_sigma, cv::BORDER_REPLICATE);
    cv::resize(detection_prob_maps[i], detection_prob_maps[i], seen_map.size());
  }

  return detection_prob_maps;
}


void SemanticMappingApp::updateObjectMaps(const VisionResult &vision_result, const cv::Mat_<double> &seen_map){
  std::vector<cv::Mat_<double>> detection_prob_maps = generateDetectionProbMaps(vision_result, seen_map);

//  std::vector<double> prior_factor(object_prior_p_.size());
//  for(int i=0; i<object_prior_p_.size(); i++){
//    prior_factor[i] = object_prior_p_[i] / (1.f-object_prior_p_[i]);
//  }

  for(int x=0; x<seen_map.cols; x++){
    for(int y=0; y<seen_map.rows; y++){
      double seen = seen_map(y, x);
      if(seen != 0.f){
        std::vector<double> p(NUM_OBJECT_TYPES), p2(NUM_OBJECT_TYPES);
        seen *= 0.1;
        std::vector<double> not_obj_prob(NUM_OBJECT_TYPES, 0.99);
        for(int i = 0; i < detection_prob_maps.size(); i++){
          double det_prob = detection_prob_maps[i](y, x);
          if(det_prob > 0.0){
            for(int o = 0; o < NUM_OBJECT_TYPES; o++){
              not_obj_prob[o] *= (1.0 - vision_result.objects_[i].prob_[o] * det_prob);
            }
          }
        }
        for(int o=0; o<NUM_OBJECT_TYPES; o++){
          p[o] = object_maps_[o](y, x);
          object_maps_[o](y, x) *= (seen*(1.0-not_obj_prob[o])/object_prior_p_[o] + 1.0 - seen) / (seen * not_obj_prob[o]/(1.0-object_prior_p_[o])+1.0-seen);
          p2[o] = object_maps_[o](y, x);
        }
      }
    }
  }
//  cv::Mat out = lToP(object_maps_[72]);
//  cv::resize(out, out, cv::Size(object_maps_[72].cols*5, object_maps_[72].rows*5), 0, 0, cv::INTER_NEAREST);
//  visualizeProbMap(out, object_labels_[72]);
  prob_map_view::ProbMapMsg msg;
  msg.names = object_labels_;
  msg.img_are_log = false;
  msg.images.resize(msg.names.size());
  for(int i=0; i<msg.images.size(); i++){
    msg.images[i].rows = object_maps_[i].rows;
    msg.images[i].cols = object_maps_[i].cols;
    msg.images[i].type = object_maps_[i].type();
    msg.images[i].data.assign(object_maps_[i].datastart, object_maps_[i].dataend);
  }
  obj_map_pub_.publish(msg);
}


void SemanticMappingApp::updatePlaceMaps(const VisionResult& vision_result, const cv::Mat_<double>& seen_map){
  for(int x=0; x<seen_map.cols; x++){
    for(int y=0; y<seen_map.rows; y++){
      if(seen_map(y, x) != 0.0){
        double sum = 0.0;
        double seen_prob = seen_map(y, x) * PLACE_VISIBLE_PROB;
        for(int i = 0; i < place_maps_.size(); i++){
          place_maps_[i](y, x) = place_maps_[i](y, x) * (seen_prob*vision_result.place_guesses_[i].prob_ + (1-seen_prob)*place_prior_[i]) / place_prior_[i];
          sum += place_maps_[i](y,x);
        }
        for(int i = 0; i < place_maps_.size(); i++){
          place_maps_[i](y, x) /= sum;
        }
      }
    }
  }

  prob_map_view::ProbMapMsg msg;
  msg.names = place_labels_;
  msg.img_are_log = false;
  msg.images.resize(msg.names.size());
  for(int i=0; i<msg.images.size(); i++){
    msg.images[i].rows = place_maps_[i].rows;
    msg.images[i].cols = place_maps_[i].cols;
    msg.images[i].type = place_maps_[i].type();
    msg.images[i].data.assign(place_maps_[i].datastart, place_maps_[i].dataend);
  }
  place_map_pub_.publish(msg);
}


cv::Mat_<double> SemanticMappingApp::generateSeenMap(const VisionResult &vision_result){
  cv::Point2d pos(vision_result.pose_.getOrigin().getX(), vision_result.pose_.getOrigin().getY());
  double dir = std::atan2(vision_result.pose_.getBasis().getColumn(0).getY(), vision_result.pose_.getBasis().getColumn(0).getX());
  const std::vector<double>& dists = vision_result.max_dists_;
  std::vector<cv::Point> visible_poly(dists.size()*2);

  double angle_step = XTION_FOV_2*2.f / (dists.size()-1);
  int minx = 1000, maxx = -1000, miny = 1000, maxy = -1000;
  for(int i=0; i<dists.size(); i++){
    double angle = dir - XTION_FOV_2 + i*angle_step;
    double dist = std::min(std::max(dists[i] / std::cos(i*angle_step), MIN_VISION_DIST), MAX_VISION_DIST);
    if(!std::isfinite(dist))
      dist = MAX_VISION_DIST;
    visible_poly[i] = (pos + cv::Point2d(dist*std::cos(angle), dist*std::sin(angle))) * OBJ_PIXEL_PER_METER + cv::Point2d(origin_);
    visible_poly[2*dists.size()-1-i] = (pos + cv::Point2d(MIN_VISION_DIST*std::cos(angle), MIN_VISION_DIST*std::sin(angle))) * OBJ_PIXEL_PER_METER + cv::Point2d(origin_);
    minx = std::min(std::min(visible_poly[i].x, visible_poly[2*dists.size()-1-i].x), minx);
    miny = std::min(std::min(visible_poly[i].y, visible_poly[2*dists.size()-1-i].y), miny);
    maxx = std::max(std::max(visible_poly[i].x, visible_poly[2*dists.size()-1-i].x), maxx);
    maxy = std::max(std::max(visible_poly[i].y, visible_poly[2*dists.size()-1-i].y), maxy);
  }
  int gauss_kernel_size = std::round(pose_sigma_ * 3 * OBJ_PIXEL_PER_METER);
  minx -= gauss_kernel_size;
  miny -= gauss_kernel_size;
  maxx += gauss_kernel_size;
  maxy += gauss_kernel_size;
  int padx1 = 0, padx2 = 0, pady1 = 0, pady2 = 0;
  while(minx < -padx1)
    padx1 += MAP_BASE_SIZE/2;
  while(miny < -pady1)
    pady1 += MAP_BASE_SIZE/2;
  while(maxx > object_maps_[0].cols + padx2)
    padx2 += MAP_BASE_SIZE/2;
  while(maxy > object_maps_[0].rows + pady2)
    pady2 += MAP_BASE_SIZE/2;

  static cv::Mat_<double> sum_out(object_maps_[0].rows, object_maps_[0].cols, 0.f);
  if(padx1 != 0 || padx2 != 0 || pady1 != 0 || pady2 != 0){
    for(auto& p : visible_poly)
      p += cv::Point(padx1, pady1);
    for(int i=0; i<object_maps_.size(); i++)
      cv::copyMakeBorder(object_maps_[i], object_maps_[i], pady1, pady2, padx1, padx2, cv::BORDER_CONSTANT, object_prior_p_[i]);
    for(int i=0; i<place_maps_.size(); i++)
      cv::copyMakeBorder(place_maps_[i], place_maps_[i], pady1, pady2, padx1, padx2, cv::BORDER_CONSTANT, place_prior_[i]);
    cv::copyMakeBorder(sum_out, sum_out, pady1, pady2, padx1, padx2, cv::BORDER_CONSTANT, 0.0);
    origin_ += cv::Point(padx1, pady1);
  }

  cv::Mat_<double> seen_map(object_maps_[0].rows, object_maps_[0].cols, 0.f);
  cv::fillPoly(seen_map, std::vector<std::vector<cv::Point>>({visible_poly}), 1.f, cv::LINE_4);
  cv::GaussianBlur(seen_map, seen_map, cv::Size(2*gauss_kernel_size+1, 2*gauss_kernel_size+1), pose_sigma_ * OBJ_PIXEL_PER_METER, pose_sigma_ * OBJ_PIXEL_PER_METER, cv::BORDER_REPLICATE);

  cv::Mat out;
  cv::resize(seen_map, out, cv::Size(seen_map.cols*5, seen_map.rows*5), 0, 0, cv::INTER_NEAREST);
  //cv::flip(out, out, 0);
  visualizeProbMap(out, "visible");
  sum_out = sum_out + seen_map;
  cv::resize(sum_out, out, cv::Size(sum_out.cols*5, sum_out.rows*5), 0, 0, cv::INTER_NEAREST);
  //cv::flip(out, out, 0);
  visualizeProbMap(out, "sum_visible");
  cv::waitKey(1);

  return seen_map;
}


void SemanticMappingApp::publishVisionPoses(const vision::VisionMsgConstPtr &msg){
  if(vision_results_.size() > 0){
    geometry_msgs::PoseArray m;
    m.header = msg->pose.header;
    m.poses.reserve(vision_results_.size());
    for(const auto &v : vision_results_){
      geometry_msgs::Pose p;
      tf::poseTFToMsg(v.pose_, p);
      m.poses.push_back(p);
    }
    vision_pose_pub_.publish(m);
  }
}


void SemanticMappingApp::visionCb(const vision::VisionMsgConstPtr& msg){
  auto begin = std::chrono::steady_clock::now();
  VisionResult res = VisionResult(*msg);
  vision_results_.push_back(res);
  cv::Mat_<double> seen_map = generateSeenMap(res);
  updatePlaceMaps(res, seen_map);
  updateObjectMaps(res, seen_map);

  if(PUBLISH_VISION_POSES){
    publishVisionPoses(msg);
  }
}


void SemanticMappingApp::mapCb(const nav_msgs::OccupancyGridConstPtr& msg){
}


void SemanticMappingApp::entropyCb(const std_msgs::Float64ConstPtr &msg){
  pose_sigma_ = msg->data * SIGMA_PER_ENTROPY;
}



int main(int argc, char** argv){
  ros::init (argc, argv, "semantic_mapping");
  ros::NodeHandle nh;
  SemanticMappingApp app(nh);

  app.run();
  return 0;
}

