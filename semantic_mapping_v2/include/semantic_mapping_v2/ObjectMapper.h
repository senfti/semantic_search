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

    ObjectMap(const ObjectMap& rhs);
    ObjectMap& operator=(const ObjectMap& rhs);

    void resize(int left, int right, int top, int bottom, float prior);

    void insertMax(int x, int y, int z, float prob);
    void insertMax(float x, float y, float z, float prob) { insertMax(getXPixel(x), getYPixel(y), getZPixel(z), prob); }
    inline void insertProb(int x, int y, int z, float prob, float prior, float V_H, float V_M, float min, float max);
    void insertProb(float x, float y, float z, float prob, float prior, float V_H, float V_M, float min, float max) {
      insertProb(getXPixel(x), getYPixel(y), getZPixel(z), prob, prior, V_H, V_M, min, max);
    }

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
    float getResolution() const { return resolution_; }
    cv::Point getOrigin() const { return origin_; }

    visualization_msgs::MarkerArray getProbMsg(int id=0) const;

    float getObjectProb(const ObjectMap& occupancy_map, float prior, float expected_room_size) const;

    semantic_mapping_v2::ObjectMapMsg getObjMapMsg() const;
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

    float OBJ_DEFAULT_RESOLUTION = 3.f;
    float OBJ_DEFUALT_MAX_HEIGHT = 2.f;
    float ROOM_EXPECTED_SIZE = 32.f;

    float V_H = 0.9;
    float V_M = 0.45;

  private:
    std::vector<ObjectMap> maps_;
    float max_height_ = OBJ_DEFUALT_MAX_HEIGHT;

    bool expandUntilFitting(const pcl::PointXYZ& min, const pcl::PointXYZ& max);
    std::vector<float> curr_probs_;

  public:
    ObjectMapper();
    void addCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud, const vision::ObjectDetectionMsg& msg);

    visualization_msgs::MarkerArray getProbMsg(int id) const { return (id < maps_.size() ? maps_[id].getProbMsg(id) : visualization_msgs::MarkerArray()); }

    std::vector<float> getObjectProbs(const OctoMapper& octo_mapper, const std::vector<Door>& doors, std::vector<size_t>& order);
    float getResolution() const { return maps_[0].getResolution(); }

    semantic_mapping_v2::ObjectMapMsg getObjMapMsg(int obj_id) const { return (maps_.size() > obj_id ? maps_[obj_id].getObjMapMsg() : semantic_mapping_v2::ObjectMapMsg()); }
    std::vector<semantic_mapping_v2::ObjectMapMsg> getAllObjMapMsgs() const;
};

#endif //SEMANTIC_MAPPING_V2_OBJECTMAP_H
