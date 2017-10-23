#include "semantic_mapping/main.h"
#include <geometry_msgs/PoseArray.h>
#include <kb_query/KbQuerySrv.h>
#include <fstream>
#include <chrono>

const std::string label_file = "/home/thomas/darknet/data/coco.names";


void visualizeProbMap(cv::Mat_<float> map, const std::string& win_name){
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

  std::ifstream labels(label_file);
  if(!labels.good()){
    std::cout << "Unable to open labels file " << label_file;
    return;
  }
  std::string line;
  while (std::getline(labels, line))
    object_labels_.push_back(line);

  object_maps_.resize(object_labels_.size());
  for(int i=0; i<place_maps_.size(); i++){
    object_prior_p_[i] = OBJ_MAP_PRIOR;
    object_prior_l_[i] = pToL(object_prior_p_[i]);
    object_maps_[i] = cv::Mat_<float>(MAP_BASE_SIZE,MAP_BASE_SIZE, object_prior_l_[i]);
  }
  place_maps_.resize(NUM_PLACE_TYPES);
  for(int i=0; i<place_maps_.size(); i++){
    place_prior_[i] = pToL(1.f/NUM_PLACE_TYPES);
    place_maps_[i] = cv::Mat_<float>(MAP_BASE_SIZE,MAP_BASE_SIZE, place_prior_[i]);
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


std::vector<cv::Mat_<float>> SemanticMappingApp::generateDetectionProbMaps(const VisionResult &vision_result, const cv::Mat_<float> &seen_map){
  std::vector<cv::Mat_<float>> detection_prob_maps(vision_result.objects_.size(), cv::Mat_<float>(seen_map.rows*PROB_DRAW_OVERSAMPLING_FACTOR, seen_map.cols*PROB_DRAW_OVERSAMPLING_FACTOR, 0.f));
  cv::Point2d pos(vision_result.pose_.getOrigin().getX(), vision_result.pose_.getOrigin().getY());
  float dir = std::atan2(vision_result.pose_.getBasis().getColumn(0).getY(), vision_result.pose_.getBasis().getColumn(0).getX());
  float pose_sigma = pose_sigma_ * OBJ_PIXEL_PER_METER * PROB_DRAW_OVERSAMPLING_FACTOR;
  int gauss_kernel_size = std::round(3*pose_sigma);
  for(int i=0; i<detection_prob_maps.size(); i++){
    cv::fillConvexPoly(detection_prob_maps[i], getCornersFromDetection(vision_result.objects_[i], pos, dir), cv::Scalar(1.0));
    cv::GaussianBlur(seen_map, seen_map, cv::Size(2*gauss_kernel_size+1, 2*gauss_kernel_size+1), pose_sigma, pose_sigma, cv::BORDER_REPLICATE);
    cv::resize(detection_prob_maps[i], detection_prob_maps[i], seen_map.size());
  }

  return detection_prob_maps;
}


void SemanticMappingApp::updateObjectMaps(const VisionResult &vision_result, const cv::Mat_<float> &seen_map){
  std::vector<cv::Mat_<float>> detection_prob_maps = generateDetectionProbMaps(vision_result, seen_map);

  std::vector<float> prior_factor(object_prior_p_.size());
  for(int i=0; i<object_prior_p_.size(); i++){
    prior_factor[i] = object_prior_p_[i] / (1.f-object_prior_p_[i]);
  }

  for(int x=0; x<seen_map.cols; x++){
    for(int y=0; y<seen_map.rows; y++){
      if(seen_map(y, x) != 0.f){
        for(int i = 0; i < object_maps_.size(); i++){
          float inv_prob = 0.99;
          for(int j=0; j<detection_prob_maps.size(); j++){
            inv_prob *= (1.f-detection_prob_maps[j](y,x)*vision_result.objects_[j].prob_[i]);
          }
          float seen_factor = object_prior_p_[i] + seen_map(y,x) * (1.f-object_prior_p_[i]);
          float inverse_sensor_model = pToL(seen_factor*(1.f-inv_prob)+prior_factor[i]*(1-seen_factor)*inv_prob);
          object_maps_[i](y, x) = object_maps_[i](y, x) + inverse_sensor_model - object_prior_l_[i];
        }
      }
    }
  }
  cv::Mat out = lToP(object_maps_[72]);
  cv::resize(out, out, cv::Size(object_maps_[72].cols*5, object_maps_[72].rows*5), 0, 0, cv::INTER_NEAREST);
  visualizeProbMap(out, object_labels_[72]);
}


void SemanticMappingApp::updatePlaceMaps(const VisionResult& vision_result, const cv::Mat_<float>& seen_map){
  float num_inv = 1.f/NUM_PLACE_TYPES;
  float num_m1_inv = 1.f/(NUM_PLACE_TYPES-1);

  for(int x=0; x<seen_map.cols; x++){
    for(int y=0; y<seen_map.rows; y++){
      if(seen_map(y, x) != 0.f){
        float seen_prob = num_inv + seen_map(y,x) * (PLACE_VISIBLE_PROB-num_inv);
        for(int i = 0; i < place_maps_.size(); i++){
          float inverse_sensor_model = pToL(num_m1_inv * (1.f - seen_prob - vision_result.place_guesses_[i].prob_) + (1.f + num_inv) * vision_result.place_guesses_[i].prob_ * seen_prob);
          place_maps_[i](y, x) = place_maps_[i](y, x) + inverse_sensor_model - place_prior_[i];
        }
      }
    }
  }
//  for(int i=0; i<place_maps_.size(); i++){
//    cv::Mat_<float> update_map = pToL(cv::Mat_<float>(
//          (1.f - seen_map - vision_result.place_guesses_[i].prob_) / (NUM_PLACE_TYPES-1) +
//          (NUM_PLACE_TYPES/(NUM_PLACE_TYPES-1))*vision_result.place_guesses_[i].prob_*seen_map));   // P(m) = P(m|~R) * P(~R) + P(m|R) P(R) = (1-P(m|R)*(1-P(R)) + P(m|R)*P(R)
//    place_maps_[i] = place_maps_[i] + update_map - place_prior_[i];
//  }

  std::vector<cv::Mat_<float>> tmp(NUM_PLACE_TYPES);
  cv::Mat_<float> sum(place_maps_[0].rows, place_maps_[0].cols, 0.f);
  for(int i=0; i<place_maps_.size(); i++){
    tmp[i] = lToP(place_maps_[i]);
    sum = sum + tmp[i];
  }
  for(int i=0; i<place_maps_.size(); i++){
    tmp[i] = tmp[i] / sum;
    place_maps_[i] = pToL(tmp[i]);
  }
  cv::resize(tmp[129], tmp[129], cv::Size(place_maps_[129].cols*5, place_maps_[129].rows*5), 0, 0, cv::INTER_NEAREST);
  visualizeProbMap(tmp[129], vision_result.place_guesses_[129].class_);
}


cv::Mat_<float> SemanticMappingApp::generateSeenMap(const VisionResult &vision_result){
  cv::Point2d pos(vision_result.pose_.getOrigin().getX(), vision_result.pose_.getOrigin().getY());
  float dir = std::atan2(vision_result.pose_.getBasis().getColumn(0).getY(), vision_result.pose_.getBasis().getColumn(0).getX());
  const std::vector<float>& dists = vision_result.max_dists_;
  std::vector<cv::Point> visible_poly(dists.size()*2);

  float angle_step = XTION_FOV_2*2.f / (dists.size()-1);
  int minx = 1000, maxx = -1000, miny = 1000, maxy = -1000;
  for(int i=0; i<dists.size(); i++){
    float angle = dir - XTION_FOV_2 + i*angle_step;
    float dist = std::min(std::max(dists[i] / std::cos(i*angle_step), MIN_VISION_DIST), MAX_VISION_DIST);
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

  if(padx1 != 0 || padx2 != 0 || pady1 != 0 || pady2 != 0){
    for(auto& p : visible_poly)
      p += cv::Point(padx1, pady1);
    for(auto& map : object_maps_)
      cv::copyMakeBorder(map, map, pady1, pady2, padx1, padx2, cv::BORDER_CONSTANT, 0.1);
    for(int i=0; i<place_maps_.size(); i++)
      cv::copyMakeBorder(place_maps_[i], place_maps_[i], pady1, pady2, padx1, padx2, cv::BORDER_CONSTANT, place_prior_[i]);
    origin_ += cv::Point(padx1, pady1);
  }

  cv::Mat_<float> seen_map(object_maps_[0].rows, object_maps_[0].cols, 0.f);
  cv::fillPoly(seen_map, std::vector<std::vector<cv::Point>>({visible_poly}), 1.f, cv::LINE_4);
  cv::GaussianBlur(seen_map, seen_map, cv::Size(2*gauss_kernel_size+1, 2*gauss_kernel_size+1), pose_sigma_ * OBJ_PIXEL_PER_METER, pose_sigma_ * OBJ_PIXEL_PER_METER, cv::BORDER_REPLICATE);

  cv::Mat out;
  cv::resize(seen_map, out, cv::Size(seen_map.cols*5, seen_map.rows*5), 0, 0, cv::INTER_NEAREST);
  visualizeProbMap(out, "visible");
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
  cv::Mat_<float> seen_map = generateSeenMap(res);
  updatePlaceMaps(res, seen_map);
  updateObjectMaps(res, seen_map);

  if(PUBLISH_VISION_POSES){
    vision_results_.push_back(res);
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























/*
void SemanticMappingApp::updatePlaceMaps(const nav_msgs::OccupancyGridConstPtr& msg){
  return;

  int width = msg->info.width;
  int height = msg->info.height;
  float resolution = msg->info.resolution;
  float orig_x = msg->info.origin.position.x;
  float orig_y = msg->info.origin.position.y;

  if(place_maps_[0].empty()){
    for(auto& map : place_maps_)
      map = cv::Mat_<float>(msg->info.height, msg->info.width, 1.f/place_maps_.size());
  }
  else if(place_maps_[0].rows != height || place_maps_[0].cols != width){
    int left = static_cast<int>((old_map_data_.origin.position.x - msg->info.origin.position.x) / resolution);
    int top = static_cast<int>((old_map_data_.origin.position.y - msg->info.origin.position.y) / resolution);
    int right = width - place_maps_[0].cols - left;
    int bottom = height - place_maps_[0].rows - top;
    for(auto& map : place_maps_)
      cv::copyMakeBorder(map, map, top, bottom, left, right, cv::BORDER_CONSTANT, cv::Scalar(1.f/place_maps_.size()));
  }
  old_map_data_ = msg->info;

  cv::Mat_<uchar> map(height, width, uchar(0));
  cv::Mat_<uchar> visible(height, width, uchar(0));
  for(int x=0; x<msg->info.width; x++){
    for(int y=0; y<msg->info.height; y++){
      if(msg->data[y*msg->info.width+x] >= 0){
        map(y,x) = 255;
        std::vector<double> log_probs(place_maps_.size());
        for(int i=0; i<place_maps_.size(); i++){
          log_probs[i] = place_maps_[i](y,x) > 0.0 ? std::log(place_maps_[i](y,x)) : -1000000000.0;
        }
        for(int i=processed_views_; i<vision_results_.size(); i++){
          const auto& v = vision_results_[i];
          float importance = static_cast<float>(v.getImportance(x*resolution + orig_x, y*resolution + orig_y));
          if(importance > 0.0){
            visible(y,x) = visible(y,x) | uchar(255);
            for(int i=0; i<v.place_guesses_.size(); i++){
              log_probs[v.place_guesses_[i].id_] += std::log(v.place_guesses_[i].prob_);
            }
          }
        }
        double sum_probs = 0.0;
        for(int i=0; i<place_labels_.size(); i++){
          log_probs[i] = std::pow(2, log_probs[i]);
          sum_probs += log_probs[i];
        }
        for(int i=0; i<place_labels_.size(); i++){
          if(visible(y,x))
            place_maps_[i](y, x) = float(log_probs[i] / sum_probs);
        }
      }
    }
  }
  processed_views_ = vision_results_.size();

  auto begin = std::chrono::steady_clock::now();
  cv::flip(map, map, 0);
  cv::imshow("map", map);
  //cv::imwrite("/tmp/map.png", map);
  cv::flip(visible, visible, 0);
  cv::imshow("visible", visible);
  //cv::imwrite("/tmp/visible.png", visible);

  cv::Mat tmp(place_maps_[0].rows, place_maps_[0].cols, CV_8UC1, cv::Scalar(255));
  cv::Mat other(place_maps_[0].rows, place_maps_[0].cols, CV_32FC1, cv::Scalar(0));
  for(int i=0; i<place_maps_.size(); i++){
    cv::Mat out;
    place_maps_[i].convertTo(out, CV_8U, 150.f);
    cv::flip(out, out, 0);
    cv::merge(std::vector<cv::Mat>({out, tmp, tmp}), out);
    cv::cvtColor(out, out, CV_HSV2BGR);
    //cv::imwrite("/tmp/" + place_labels_[i] + ".png", out);
    if(!place_labels_[i].compare("office") || !place_labels_[i].compare("corridor") || !place_labels_[i].compare("kitchen") || !place_labels_[i].compare("kitchenette"))
      cv::imshow(place_labels_[i], out);
    else
      other = other + place_maps_[i];
  }

  other.convertTo(other, CV_8U, 150.f);
  cv::flip(other, other, 0);
  cv::merge(std::vector<cv::Mat>({other, tmp, tmp}), other);
  cv::cvtColor(other, other, CV_HSV2BGR);
  cv::imshow("other", other);
  //cv::imwrite("/tmp/other.png", other);
  cv::waitKey(1);
  std::cout << "Debug images in " <<std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - begin).count() << " ms" << std::endl;
}*/