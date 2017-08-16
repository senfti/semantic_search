//
// Created by thomas on 12.08.17.
//

#include "vision/YoloDetector.h"
#include <fstream>
#include <iostream>

YoloDetection::YoloDetection(const std::string& label, float prob, float x1, float x2, float y1, float y2)
      : label_(label), prob_(prob), x1_(x1), x2_(x2), y1_(y1), y2_(y2)
{
}

void YoloDetection::draw(cv::Mat& img, cv::Scalar border_color, int border_thickness, cv::Scalar text_color, float text_size, int text_thickness){
  cv::rectangle(img, cv::Point(x1_, y1_), cv::Point(x2_, y2_), border_color, border_thickness);
  cv::putText(img, label_ + std::to_string(prob_).substr(0, 5), cv::Point(x1_, y1_), cv::FONT_HERSHEY_COMPLEX, text_size, text_color, text_thickness);
}

void YoloDetection::scale(float factor){
  x1_ *= factor;
  x2_ *= factor;
  y1_ *= factor;
  y2_ *= factor;
}



YoloDetector::YoloDetector(const std::string &label_file, const std::string &config_file, const std::string &weight_file){
  cuda_set_device(0);

  if(!loadLabels(label_file))
    return;

  net_ = parse_network_cfg(const_cast<char*>(config_file.c_str()));
  load_weights(&net_, const_cast<char*>(weight_file.c_str()));
  set_batch_network(&net_, 1);
  is_ok_ = true;
}

bool YoloDetector::loadLabels(const std::string &label_file){
  std::ifstream file(label_file);
  while(file.good()){
    std::string label;
    std::getline(file, label);
    if(label.length() > 0)
      labels_.push_back(label);
    else
      break;
  }
  if(labels_.size() < 1)
    return false;
  return true;
}

std::vector<YoloDetection> YoloDetector::detect(const cv::Mat &img, float thresh, float hier_thresh, float nms){
  cv::Mat tmp;
  cv::resize(img, tmp, cv::Size(net_.w, net_.h));
  cv::cvtColor(tmp, tmp, CV_BGR2RGB);
  tmp.convertTo(tmp, CV_32FC3, 1/255.0);
  std::vector<cv::Mat> channels;
  cv::split(tmp, channels);
  cv::vconcat(channels, tmp);

  network_predict(net_, (float*)(tmp.data));

  layer last_layer = net_.layers[net_.n-1];
  int output_size = last_layer.w*last_layer.h*last_layer.n;
  box boxes[output_size];
  float* probs[output_size];
  for(int i=0; i<output_size; i++)
    probs[i] = new float[last_layer.classes];
  get_region_boxes(net_.layers[net_.n-1], 1, 1, thresh, probs, boxes, 0, 0, hier_thresh);
  if(nms != 0.f)
    do_nms_obj(boxes, probs, last_layer.w*last_layer.h*last_layer.n, last_layer.classes, nms);

  std::vector<YoloDetection> detections;
  for(int i=0; i<output_size; i++){
    float prob = -1.f;
    int max_idx = -1;
    for(int j=0; j<last_layer.classes; j++){
      if(probs[i][j] > prob){
        prob = probs[i][j];
        max_idx = j;
      }
    }
    float x1 = (boxes[i].x-boxes[i].w/2.)*img.cols;
    float x2 = (boxes[i].x+boxes[i].w/2.)*img.cols;
    float y1 = (boxes[i].y-boxes[i].h/2.)*img.rows;
    float y2 = (boxes[i].y+boxes[i].h/2.)*img.rows;
    if(prob > thresh){
      detections.push_back(YoloDetection(labels_[max_idx], prob, x1, x2, y1, y2));
    }
  }

  std::sort(detections.begin(), detections.end(), operator>);
  return detections;
}
