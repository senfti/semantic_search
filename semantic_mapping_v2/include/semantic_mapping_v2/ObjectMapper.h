//
// Created by thomas on 15.11.17.
//

#ifndef SEMANTIC_MAPPING_V2_OBJECTMAP_H
#define SEMANTIC_MAPPING_V2_OBJECTMAP_H

#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

const int NUM_OBJECTS = 80;
const float OBJ_PRIOR_PROB = 0.01f;
const float OBJ_CONFIDENCE = 0.3f;
const float OBJ_MIN_PROB = 0.0001f;
const float OBJ_MAX_PROB = 0.99f;

class ObjectMap{
  private:

    float resolution_ = 4.f;
    int base_size_;
    float max_height_ = 1.75f;
    std::vector<cv::Mat_<float>> prob_maps_;
    cv::Point origin_;

  public:
    ObjectMap(float resolution = 4.f, float start_size = 10.0f, float max_height = 1.75f, float initial_value = OBJ_PRIOR_PROB);
    ObjectMap(float resolution, int base_size, int width, int height, const cv::Point& origin, float max_height = 1.75f, float initial_value = OBJ_PRIOR_PROB);

    void resize(float left, float right, float top, float bottom);

    void insertMax(int x, int y, int z, float prob);
    void insertMax(float x, float y, float z, float prob) { insertMax(getXPixel(x), getYPixel(y), getZPixel(z), prob); }
    void insertProb(int x, int y, int z, float prob);
    void insertProb(float x, float y, float z, float prob) { insertProb(getXPixel(x), getYPixel(y), getZPixel(z), prob); }

    float getProb(int x, int y, int z) const { return prob_maps_[z](y, x); }
    float getProb(float x, float y, float z) const { return  getProb(getXPixel(x), getYPixel(y), getZPixel(z)); }

    int getXPixel(float x) const { return x*resolution_ + origin_.x; }
    int getYPixel(float y) const { return y*resolution_ + origin_.y; }
    int getZPixel(float z) const { return z*resolution_; }

    int getBaseSize() const { return base_size_; }
    int getWidth() const { return prob_maps_[0].cols; }
    int getHeight() const { return prob_maps_[0].rows; }
    int getZSteps() const { return std::ceil(max_height_*resolution_); }
    float getResolution() const { return resolution_; }
    cv::Point getOrigin() const { return origin_; }
};


class ObjectMapper{
  private:
    std::vector<ObjectMap> maps_;
    float max_height_ = 1.75f;

    bool expandUntilFitting(const pcl::PointXYZ& min, const pcl::PointXYZ& max);

  public:
    ObjectMapper();
    void addCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud, const std::vector<std::vector<float>>& probs);

};

#endif //SEMANTIC_MAPPING_V2_OBJECTMAP_H
