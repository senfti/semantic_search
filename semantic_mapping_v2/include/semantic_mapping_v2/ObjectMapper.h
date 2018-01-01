//
// Created by thomas on 15.11.17.
//

#ifndef SEMANTIC_MAPPING_V2_OBJECTMAP_H
#define SEMANTIC_MAPPING_V2_OBJECTMAP_H

#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vision/ObjectDetectionMsg.h>
#include <visualization_msgs/MarkerArray.h>
#include <semantic_mapping_v2/ObjectMapMsg.h>

#include <semantic_mapping_v2/DoorMapper.h>
#include <boost/thread.hpp>
#include <boost/thread/lock_guard.hpp>

class OctoMapper;

class ObjectMap{
  private:

    float resolution_;
    int base_size_;
    float max_height_;
    std::vector<cv::Mat_<float>> prob_maps_;
    std::vector<cv::Mat_<uchar>> count_maps_;
    cv::Point origin_;

  public:
    ObjectMap(float resolution, float start_size, float max_height, float initial_value);
    ObjectMap(float resolution, int base_size, int width, int height, const cv::Point& origin, float max_height, float initial_value);
    ObjectMap(float resolution, int base_size, int width, int height, const cv::Point& origin, float max_height, OctoMapper& octomap);
    ObjectMap(const ObjectMap& object_map, const ObjectMap& occ_map, cv::Mat_<float> obj_from_room);

    ObjectMap operator*(const ObjectMap& rhs) const;

    ObjectMap(const ObjectMap& rhs);
    ObjectMap& operator=(const ObjectMap& rhs);

    void resize(int left, int right, int top, int bottom, float prior);

    void insertMax(int x, int y, int z, float prob);
    void insertMax(float x, float y, float z, float prob) { insertMax(getXPixel(x), getYPixel(y), getZPixel(z), prob); }
    inline void insertProb(int x, int y, int z, float prob, float prior, float V_H, float V_M, float min, float max);
    void insertProb(float x, float y, float z, float prob, float prior, float V_H, float V_M, float min, float max) {
      insertProb(getXPixel(x), getYPixel(y), getZPixel(z), prob, prior, V_H, V_M, min, max);
    }
    void applyObjAppearVanish(float still_there_prob, float got_there_prob);
    void resetProbs(float prior);

    float getProb(int x, int y, int z) const { return prob_maps_[z](y, x); }
    float getProb(float x, float y, float z) const { return  getProb(getXPixel(x), getYPixel(y), getZPixel(z)); }

    int getXPixel(float x) const { return x*resolution_ + origin_.x; }
    int getYPixel(float y) const { return y*resolution_ + origin_.y; }
    int getZPixel(float z) const { return std::min(z*resolution_, float(prob_maps_.size()-1)); }

    float getXWorld(float x) const { return (x-origin_.x+0.5f)/resolution_; }
    float getYWorld(float y) const { return (y-origin_.y+0.5f)/resolution_; }
    float getZWorld(float z) const { return (z+0.5f)/resolution_; }

    int getBaseSize() const { return base_size_; }
    int getWidth() const { return prob_maps_[0].cols; }
    int getHeight() const { return prob_maps_[0].rows; }
    int getZSteps() const { return std::ceil(max_height_*resolution_); }
    float getMaxHeight() const { return max_height_; }
    float getResolution() const { return resolution_; }
    cv::Point getOrigin() const { return origin_; }

    visualization_msgs::MarkerArray getProbMsg(int id=0) const;

    float getObjectProb(const ObjectMap& occupancy_map, float prior, float expected_room_size) const;
    float getObjectProb(const cv::Mat_<float>& behind_door_mask, float prior, float expected_room_size) const;
    float getMaxObjectProb(const ObjectMap& occupancy_map) const;

    semantic_mapping_v2::ObjectMapMsg getObjMapMsg() const;
    cv::Mat_<float> get2D(const cv::Mat_<float>& behind_door_mask, const ObjectMap& occ_map) const;
};


inline void ObjectMap::insertProb(int x, int y, int z, float prob, float prior, float V_H, float V_M, float min, float max){
  float p = prob_maps_[z](y,x);
  float update = (prob*V_H + (1.f-prob)*V_M);
  float tmp = update / prior * p;
  float tmp2 = (1.f-update) / (1.f-prior) * (1.f-p);
  prob_maps_[z](y,x) = std::min(max, std::max(min, tmp/(tmp+tmp2)));
  count_maps_[z](y,x) = count_maps_[z](y,x) == uchar(255) ? uchar(255) : count_maps_[z](y,x)+uchar(1);
}


class ObjectMapper{
  public:
    static std::vector<std::string> getObjNames();
    static std::string getObjName(int idx);

    float OBJ_PRIOR_PROB = 0.01f;
    float OBJ_MIN_PROB = 0.0001f;
    float OBJ_MAX_PROB = 0.9f;

    float OBJ_DEFAULT_RESOLUTION = 4.f;
    float OBJ_DEFUALT_MAX_HEIGHT = 1.6f;
    float ROOM_EXPECTED_SIZE = 32.f;

    float V_H = 0.3;
    float V_M = 0.0002;

    float STILL_THERE_PROB = 0.9f;
    float GOT_THERE_PROB = 0.0005f;

    float FOUND_THRESH = 0.5f;

  private:
    std::vector<ObjectMap> maps_;
    ObjectMap current_search_map_;
    int current_search_obj_ = -1;
    boost::mutex maps_mutex_;
    float max_height_ = OBJ_DEFUALT_MAX_HEIGHT;

    bool expandUntilFitting(const pcl::PointXYZ& min, const pcl::PointXYZ& max);

  public:
    ObjectMapper();
    ObjectMapper(const ObjectMapper& rhs);
    std::pair<cv::Point,cv::Size> addCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud, const vision::ObjectDetectionMsg& msg, float min_z, float max_z);
    void applyObjAppearVanish();
    void resetCurrentSearchMap(int obj);

    visualization_msgs::MarkerArray getProbMsg(int id) {
      boost::lock_guard<boost::mutex> lock(maps_mutex_);
      return (id < maps_.size() ? maps_[id].getProbMsg(id) : visualization_msgs::MarkerArray());
    }

    std::vector<float> getObjectProbs(OctoMapper& octo_mapper, const std::vector<Door>& doors, std::vector<size_t>& order);
    bool objectFound(OctoMapper& octo_mapper, const std::vector<Door>& doors);
    float getResolution() const { return maps_[0].getResolution(); }

    semantic_mapping_v2::ObjectMapMsg getObjMapMsg(int obj_id) {
      boost::lock_guard<boost::mutex> lock(maps_mutex_);
      return (maps_.size() > obj_id ? maps_[obj_id].getObjMapMsg() : semantic_mapping_v2::ObjectMapMsg());
    }
    std::vector<semantic_mapping_v2::ObjectMapMsg> getAllObjMapMsgs();

    std::vector<ObjectMap> getMaps() {
      boost::lock_guard<boost::mutex> lock(maps_mutex_);
      return maps_;
    }
};

#endif //SEMANTIC_MAPPING_V2_OBJECTMAP_H
