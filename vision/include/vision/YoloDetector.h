//
// Created by thomas on 12.08.17.
//

#ifndef VISION_YOLODETECTOR_H
#define VISION_YOLODETECTOR_H

#define GPU
#define OPENCV
#define CUDNN

#include <network.h>
#include <detection_layer.h>
#include <cost_layer.h>
#include <utils.h>
#include <parser.h>
#include <box.h>
#include <region_layer.h>
#include <option_list.h>
#include <opencv2/opencv.hpp>


class YoloDetection{
  public:
    std::string label_;
    float prob_;
    float x1_, x2_, y1_, y2_;

    YoloDetection(const std::string& label, float prob, float x1, float x2, float y1, float y2);
    void draw(cv::Mat& img, cv::Scalar border_color, int border_thickness, cv::Scalar text_color, float text_size, int text_thickness);
    void scale(float factor);

    friend bool operator>(const YoloDetection& lhs, const YoloDetection& rhs);
};

inline bool operator>(const YoloDetection& lhs, const YoloDetection& rhs){
  return lhs.prob_ > rhs.prob_;
}

class YoloDetector{
  private:
    std::vector<std::string> labels_;
    network net_;
    bool is_ok_ = false;

    bool loadLabels(const std::string& label_file);

  public:
    YoloDetector(const std::string& label_file, const std::string& config_file, const std::string& weight_file);
    std::vector<YoloDetection> detect(const cv::Mat& img, float thresh, float hier_thresh, float nms);
    bool isOk() const { return is_ok_; }
};

#endif //VISION_YOLODETECTOR_H
