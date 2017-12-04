//
// Created by thomas on 15.11.17.
//

#ifndef SEMANTIC_MAPPING_V2_OBJECTMAP_H
#define SEMANTIC_MAPPING_V2_OBJECTMAP_H

#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vision/ObjectDetectionSample.h>
#include <visualization_msgs/MarkerArray.h>

const int NUM_OBJECTS = 80;
const float OBJ_PRIOR_PROB = 0.01f;
const float OBJ_CONFIDENCE = 0.99f;
const float OBJ_MIN_PROB = 0.0001f;
const float OBJ_MAX_PROB = 0.99f;

const float OBJ_DEFAULT_RESOLUTION = 3.f;
const float OBJ_DEFUALT_MAX_HEIGHT = 2.f;

class ObjectMap{
  private:

    float resolution_ = OBJ_DEFAULT_RESOLUTION;
    int base_size_;
    float max_height_ = OBJ_DEFUALT_MAX_HEIGHT;
    std::vector<cv::Mat_<float>> prob_maps_;
    cv::Point origin_;

  public:
    ObjectMap(float resolution = OBJ_DEFAULT_RESOLUTION, float start_size = 10.0f, float max_height = OBJ_DEFUALT_MAX_HEIGHT, float initial_value = OBJ_PRIOR_PROB);
    ObjectMap(float resolution, int base_size, int width, int height, const cv::Point& origin, float max_height = OBJ_DEFUALT_MAX_HEIGHT, float initial_value = OBJ_PRIOR_PROB);

    void resize(float left, float right, float top, float bottom);

    void insertMax(int x, int y, int z, float prob);
    void insertMax(float x, float y, float z, float prob) { insertMax(getXPixel(x), getYPixel(y), getZPixel(z), prob); }
    void insertProb(int x, int y, int z, float prob);
    void insertProb(float x, float y, float z, float prob) { insertProb(getXPixel(x), getYPixel(y), getZPixel(z), prob); }

    float getProb(int x, int y, int z) const {
      if(z<0 || z>= prob_maps_.size())
        std::cout << "get-------------------------------Z out: " << z << "size: " << prob_maps_.size() << std::endl;
      if(x<0 || x>= prob_maps_[z].cols)
        std::cout << "get-------------------------------X out: " << x << "size: " << prob_maps_[z].cols << std::endl;
      if(y<0 || y>= prob_maps_[z].rows)
        std::cout << "get-------------------------------Y out: " << y << "size: " << prob_maps_[z].rows << std::endl;

      return prob_maps_[z](y, x);
    }

    float getProb(float x, float y, float z) const { return  getProb(getXPixel(x), getYPixel(y), getZPixel(z)); }

    int getXPixel(float x) const { return x*resolution_ + origin_.x; }
    int getYPixel(float y) const { return y*resolution_ + origin_.y; }
    int getZPixel(float z) const { return std::min(z*resolution_, float(prob_maps_.size()-1)); }

    int getBaseSize() const { return base_size_; }
    int getWidth() const { return prob_maps_[0].cols; }
    int getHeight() const { return prob_maps_[0].rows; }
    int getZSteps() const { return std::ceil(max_height_*resolution_); }
    float getResolution() const { return resolution_; }
    cv::Point getOrigin() const { return origin_; }

    visualization_msgs::MarkerArray getProbMsg(int id=0) const;
};


class ObjectMapper{
  private:
    std::vector<ObjectMap> maps_;
    float max_height_ = OBJ_DEFUALT_MAX_HEIGHT;

    bool expandUntilFitting(const pcl::PointXYZ& min, const pcl::PointXYZ& max);

  public:
    ObjectMapper();
    void addCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud, const std::vector<vision::ObjectDetectionSample>& samples);

    visualization_msgs::MarkerArray getProbMsg(int id) const { return maps_[id].getProbMsg(id); }
};

#endif //SEMANTIC_MAPPING_V2_OBJECTMAP_H
