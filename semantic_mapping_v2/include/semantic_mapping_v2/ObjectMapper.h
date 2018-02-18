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
#include <assert.h>


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
    ObjectMap(float resolution, int base_size, int width, int height, const cv::Point& origin, float max_height, float initial_value, uchar initial_count = 0);
    ObjectMap(float resolution, int base_size, int width, int height, const cv::Point& origin, float max_height, OctoMapper& octomap, const std::vector<Door>& doors);
    ObjectMap(const ObjectMap& object_map, const ObjectMap& occ_map, cv::Mat_<float> obj_from_room);

    ObjectMap operator*(const ObjectMap& rhs) const;

    ObjectMap(const ObjectMap& rhs);
    ObjectMap& operator=(const ObjectMap& rhs);

    void resize(int left, int right, int top, int bottom, float prior);

    inline void insertMax(int x, int y, int z, float prob);
    void insertMax(float x, float y, float z, float prob) { insertMax(getXPixel(x), getYPixel(y), getZPixel(z), prob); }
    inline void insertProb(int x, int y, int z, float prob, float prior, float V_H, float V_M, float min, float max);
    void insertProb(float x, float y, float z, float prob, float prior, float V_H, float V_M, float min, float max) {
      insertProb(getXPixel(x), getYPixel(y), getZPixel(z), prob, prior, V_H, V_M, min, max);
    }
    void applyObjAppearVanish(float still_there_prob, float got_there_prob);

    float getProb(int x, int y, int z) const { return prob_maps_[z](y, x); }
    float getProb(float x, float y, float z) const { return  getProb(getXPixel(x), getYPixel(y), getZPixel(z)); }
    uchar getCount(int x, int y, int z) const { return count_maps_[z](y, x); }
    uchar getCount(float x, float y, float z) const { return  getCount(getXPixel(x), getYPixel(y), getZPixel(z)); }

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

    visualization_msgs::MarkerArray getProbMsg(int id=0, float thresh = 0.f) const;
    visualization_msgs::MarkerArray getProbMsg(OctoMapper& occ, int id=0, float thresh = 0.f) const;

    float getObjectProb(const ObjectMap& occupancy_map, bool multiply_occ = true) const;
    float getObjectProb(float prior, float expected_room_size) const;
    std::pair<geometry_msgs::Pose, float> getObjMax(const ObjectMap& occupancy_map);

    semantic_mapping_v2::ObjectMapMsg getObjMapMsg() const;
    cv::Mat_<float> get2D(const cv::Mat_<float>& behind_door_mask) const;
    cv::Mat_<float> get2D(const cv::Mat_<float>& behind_door_mask, const ObjectMap& occ_map) const;
    cv::Mat_<uchar> getCount2D(const cv::Mat_<float>& behind_door_mask) const;

    std::vector<float> getProbDistribution(const cv::Mat_<float>& behind_door_mask) const;
};


inline void ObjectMap::insertProb(int x, int y, int z, float prob, float prior, float V_H, float V_M, float min, float max){
  assert(z>=0 && z<prob_maps_.size() && x>=0 && x<prob_maps_[0].cols && y>=0 && y<prob_maps_[0].rows);
  float p = prob_maps_[z](y,x);
  float update = (prob*V_H + (1.f-prob)*V_M);
  float tmp = update / prior * p;
  float tmp2 = (1.f-update) / (1.f-prior) * (1.f-p);
  prob_maps_[z](y,x) = std::min(max, std::max(min, tmp/(tmp+tmp2)));
  count_maps_[z](y,x) = std::min(count_maps_[z](y,x)+1, 100);
}


inline void ObjectMap::insertMax(int x, int y, int z, float prob){
  assert(z>=0 && z<prob_maps_.size() && x>=0 && x<prob_maps_[0].cols && y>=0 && y<prob_maps_[0].rows);
  prob_maps_[z](y,x) = std::max(prob, prob_maps_[z](y,x));
  count_maps_[z](y,x) = std::min(count_maps_[z](y,x)+1, 100);
}


class ObjectMapper{
  public:
    static std::vector<std::string> getObjNames();
    static std::string getObjName(int idx);

    static float OBJ_PRIOR_PROB;
    static float OBJ_MIN_PROB;
    static float OBJ_MAX_PROB;

    static float OBJ_DEFAULT_RESOLUTION;
    static float OBJ_DEFUALT_MAX_HEIGHT;
    static float ROOM_EXPECTED_SIZE;

    static float V_H;
    static float V_M;

    static float STILL_THERE_PROB;
    static float GOT_THERE_PROB;
    static bool PARAMS_LOADED;

    static float OBJ_FROM_ROOM_CONFIDENCE;

  private:
    std::vector<ObjectMap> maps_;
    boost::mutex maps_mutex_;
    float max_height_ = OBJ_DEFUALT_MAX_HEIGHT;

    bool expandUntilFitting(const pcl::PointXYZ& min, const pcl::PointXYZ& max);

  public:
    ObjectMapper();
    ObjectMapper(const ObjectMapper& rhs);
    ObjectMapper& operator=(const ObjectMapper& rhs);

    std::pair<cv::Point,cv::Size> addCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud, const vision::ObjectDetectionMsg& msg, float min_z, float max_z);
    void applyObjAppearVanish();

    visualization_msgs::MarkerArray getProbMsg(int id) {
      boost::lock_guard<boost::mutex> lock(maps_mutex_);
      return (id < maps_.size() ? maps_[id].getProbMsg(id) : visualization_msgs::MarkerArray());
    }
    visualization_msgs::MarkerArray getProbMsg(OctoMapper& occ, int id) {
      boost::lock_guard<boost::mutex> lock(maps_mutex_);
      return (id < maps_.size() ? maps_[id].getProbMsg(occ, id) : visualization_msgs::MarkerArray());
    }

    std::vector<float> getObjectProbs(OctoMapper& octo_mapper, const std::vector<Door>& doors, std::vector<size_t>& order);
    std::vector<std::pair<geometry_msgs::Pose, float>> getObjMax(OctoMapper& octo_mapper, const std::vector<Door>& doors);
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
