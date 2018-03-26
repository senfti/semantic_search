//
// Created by thomas on 23.11.17.
//

#ifndef SEMANTIC_MAPPING_V2_ROOMTYPEMAPPER_H
#define SEMANTIC_MAPPING_V2_ROOMTYPEMAPPER_H

#include "vision/VisionMsg.h"
#include <opencv2/opencv.hpp>
#include "gmapping/utils/point.h"
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include "semantic_mapping_v2/DoorMapper.h"
#include <semantic_mapping_v2/RoomTypeMapMsg.h>
#include <boost/thread.hpp>
#include <boost/thread/lock_guard.hpp>

const int NUM_ROOM_TYPES = 60;

class RoomTypeMap{
  private:
    float resolution_;
    int base_size_;
    cv::Mat_<float> prob_map_;
    cv::Mat_<uchar> seen_map_;
    cv::Point origin_;
    std::string name_;

  public:
    RoomTypeMap(float resolution, float start_size, float initial_value, const std::string& name = "");
    RoomTypeMap(float resolution, int base_size, int width, int height, const cv::Point& origin, float initial_value, const std::string& name = "");
    RoomTypeMap(const cv::Mat_<float>& prob_map, const cv::Mat_<uchar>& seen_map, const cv::Point& origin, float resolution, int base_size);

    RoomTypeMap(const RoomTypeMap& rhs);
    RoomTypeMap& operator=(const RoomTypeMap& rhs);

    void resize(int left, int right, int top, int bottom, float prior);
    void setProb(int x, int y, float prob) {
      assert(x>=0 && x<prob_map_.cols && y>=0 && y<prob_map_.rows); prob_map_(y, x) = prob; seen_map_(y,x) = 1; }
    void setProb(float x, float y, float prob) { setProb(getXPixel(x), getYPixel(y), prob); }

    float getProb(int x, int y) const { return prob_map_(y, x); }
    float getProb(float x, float y) const { return getProb(getXPixel(x), getYPixel(y)); }
    bool wasSeen(int x, int y) const { return seen_map_(y,x) > 0; }
    bool wasSeen(float x, float y) const { return wasSeen(getXPixel(x), getYPixel(y)); }

    int getXPixel(float x) const { return x*resolution_ + origin_.x; }
    int getYPixel(float y) const { return y*resolution_ + origin_.y; }

    float getXWorld(float x) const { return (x-origin_.x)/resolution_; }
    float getYWorld(float y) const { return (y-origin_.y)/resolution_; }

    int getBaseSize() const { return base_size_; }
    int getWidth() const { return prob_map_.cols; }
    int getHeight() const { return prob_map_.rows; }
    float getResolution() const { return resolution_; }
    cv::Point getOrigin() const { return origin_; }

    void setName(const std::string& name) { name_ = name; }

    visualization_msgs::MarkerArray getProbMsg(int id=0) const;
    semantic_mapping_v2::RoomTypeMapMsg getRoomTypeMapMsg() const;

    cv::Mat_<float> getMap() const { return prob_map_; }
    cv::Mat_<uchar> getSeenMap() const { return seen_map_; }
};

class RoomTypeMapper{
  public:
    static float CELL_MIN_PROB;
    static float CELL_MAX_PROB;
    static float ROOM_MAX_PROB;
    static float ROOM_DEFAULT_RESOLUTION;
    static float ASUS_FOV;
    static float MIN_DIST;
    static float MAX_DIST;
    static bool PARAMS_LOADED;

    int NUM_CLASSES = 0;
    float ROOM_PRIOR_PROB = 1.f/NUM_ROOM_TYPES;

    static float getRoomSimilarity(int i, int j);

  private:
    std::vector<RoomTypeMap> prob_maps_;
    boost::mutex maps_mutex_;
    std::vector<std::string> names_;

    bool resizeUntilFitting(std::vector<cv::Point>& points);
    void updateProbs(const vision::VisionMsgConstPtr& msg, int x, int y);

  public:
    RoomTypeMapper();
    RoomTypeMapper(const RoomTypeMapper& rhs);
    RoomTypeMapper(const std::vector<cv::Mat_<float>>& prob_maps, const cv::Mat_<uchar>& seen_map, const cv::Point& origin, float resolution, int base_size);

    void processMsg(const vision::VisionMsgConstPtr& msg, const GMapping::OrientedPoint& pose);
    void resizeToObjMap(const cv::Point& origin, const cv::Size& size);

    //std::vector<double> getProbs() const { return probs_; }
    std::vector<std::string> getNames() const { return names_; }
    std::string getName(int idx) const { return (idx<names_.size() ? names_[idx] : ""); }

//    std::string getBestName() const { return names_[std::max_element(probs_.begin(), probs_.end())-probs_.begin()]; }
//    double getBestProb() const { return *std::max_element(probs_.begin(), probs_.end()); }

    visualization_msgs::MarkerArray getProbMsg(int id) const {
      if(prob_maps_.empty())
        return visualization_msgs::MarkerArray();
      return prob_maps_[id].getProbMsg(id);
    }

    std::vector<float> getRoomProb(const nav_msgs::OccupancyGrid& map, const std::vector<Door>& doors, std::vector<size_t>& order);
    std::vector<float> getTypeCellNumberEstimate(const nav_msgs::OccupancyGrid& map, const std::vector<Door>& doors);

    semantic_mapping_v2::RoomTypeMapMsg getRoomTypeMapMsg(int room_type_id) {
      boost::lock_guard<boost::mutex> lock(maps_mutex_);
      return (prob_maps_.size() > room_type_id ? prob_maps_[room_type_id].getRoomTypeMapMsg() : semantic_mapping_v2::RoomTypeMapMsg());
    }
    std::vector<semantic_mapping_v2::RoomTypeMapMsg> getAllRoomTypeMapMsgs();

    std::vector<RoomTypeMap> getMaps() {
      boost::lock_guard<boost::mutex> lock(maps_mutex_);
      std::vector<RoomTypeMap> maps = prob_maps_;
      return maps;
    }
};

#endif //SEMANTIC_MAPPING_V2_ROOMTYPEMAPPER_H
