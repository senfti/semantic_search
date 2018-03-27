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
#include "vision/ObjectDetectionMsg.h"

static const int NUM_OBJECT_TYPES = 80;
static const int NUM_USED_OBJECT_TYPES = 61;

class YoloDetection{
  public:
    std::string label_;
    int id_;
    std::vector<float> prob_;
    float x1_, x2_, y1_, y2_;

    YoloDetection(const std::string& label, int id, float* prob, float x1, float x2, float y1, float y2, float z, const std::vector<bool>& used_classes);
    void draw(cv::Mat& img, cv::Scalar border_color, int border_thickness, cv::Scalar text_color, float text_size, int text_thickness);
    void scale(float factor);
};

class YoloDetector{
  private:
    std::vector<std::string> labels_;
    std::vector<bool> used_classes_;
    network* net_;
    bool is_ok_ = false;

    bool loadLabels(const std::string& label_file);
    void getRegionBoxes(layer l, int w, int h, float thresh, float **probs, float *box_probs, box *boxes, int only_objectness, int *map, float tree_thresh);      // from darknet implementation with minor changes

  public:
    YoloDetector(const std::string& label_file, const std::string& config_file, const std::string& weight_file);
    ~YoloDetector();
    std::vector<YoloDetection> detect(const cv::Mat& img, float thresh, float hier_thresh, float nms);
    bool isOk() const { return is_ok_; }
};

#endif //VISION_YOLODETECTOR_H
