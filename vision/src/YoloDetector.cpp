//
// Created by thomas on 12.08.17.
//

#include "vision/YoloDetector.h"
#include <fstream>
#include <iostream>
#include <chrono>

YoloDetection::YoloDetection(const std::string& label, int id, float* prob, float x1, float x2, float y1, float y2, float z)
      : label_(label), id_(id), prob_(prob, prob + NUM_OBJECT_TYPES), x1_(x1), x2_(x2), y1_(y1), y2_(y2)
{
}

void YoloDetection::draw(cv::Mat& img, cv::Scalar border_color, int border_thickness, cv::Scalar text_color, float text_size, int text_thickness){
  cv::rectangle(img, cv::Point(x1_, y1_), cv::Point(x2_, y2_), border_color, border_thickness);
  cv::putText(img, label_ + std::to_string(prob_[id_]).substr(0, 5), cv::Point(x1_, y1_), cv::FONT_HERSHEY_COMPLEX, text_size, text_color, text_thickness);
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


// from darknet implementation
box getRegionBox(float *x, float *biases, int n, int index, int i, int j, int w, int h, int stride)
{
  box b;
  b.x = (i + x[index + 0*stride]) / w;
  b.y = (j + x[index + 1*stride]) / h;
  b.w = exp(x[index + 2*stride]) * biases[2*n]   / w;
  b.h = exp(x[index + 3*stride]) * biases[2*n+1] / h;
  return b;
}


// from darknet implementation
int entryIndex(layer l, int batch, int location, int entry)
{
  int n =   location / (l.w*l.h);
  int loc = location % (l.w*l.h);
  return batch*l.outputs + n*l.w*l.h*(l.coords+l.classes+1) + entry*l.w*l.h + loc;
}


// from darknet implementation with small changes
void YoloDetector::getRegionBoxes(layer l, int w, int h, float thresh, float **probs, float *box_probs, box *boxes, int only_objectness, int *map, float tree_thresh){
  int i,j,n,z;
  float *predictions = l.output;
  if (l.batch == 2) {
    float *flip = l.output + l.outputs;
    for (j = 0; j < l.h; ++j) {
      for (i = 0; i < l.w/2; ++i) {
        for (n = 0; n < l.n; ++n) {
          for(z = 0; z < l.classes + 5; ++z){
            int i1 = z*l.w*l.h*l.n + n*l.w*l.h + j*l.w + i;
            int i2 = z*l.w*l.h*l.n + n*l.w*l.h + j*l.w + (l.w - i - 1);
            float swap = flip[i1];
            flip[i1] = flip[i2];
            flip[i2] = swap;
            if(z == 0){
              flip[i1] = -flip[i1];
              flip[i2] = -flip[i2];
            }
          }
        }
      }
    }
    for(i = 0; i < l.outputs; ++i){
      l.output[i] = (l.output[i] + flip[i])/2.;
    }
  }
  for (i = 0; i < l.w*l.h; ++i){
    int row = i / l.w;
    int col = i % l.w;
    for(n = 0; n < l.n; ++n){

      int index = n*l.w*l.h + i;
      int obj_index = entryIndex(l, 0, n*l.w*l.h + i, 4);
      int box_index = entryIndex(l, 0, n*l.w*l.h + i, 0);
      float scale = predictions[obj_index];
      boxes[index] = getRegionBox(predictions, l.biases, n, box_index, col, row, l.w, l.h, l.w*l.h);
      if(1){
        int max = w > h ? w : h;
        boxes[index].x =  (boxes[index].x - (max - w)/2./max) / ((float)w/max);
        boxes[index].y =  (boxes[index].y - (max - h)/2./max) / ((float)h/max);
        boxes[index].w *= (float)max/w;
        boxes[index].h *= (float)max/h;
      }

      boxes[index].x *= w;
      boxes[index].y *= h;
      boxes[index].w *= w;
      boxes[index].h *= h;

      int class_index = entryIndex(l, 0, n*l.w*l.h + i, 5);
      if(l.softmax_tree){

        hierarchy_predictions(predictions + class_index, l.classes, l.softmax_tree, 0, l.w*l.h);
        if(map){
          for(j = 0; j < 200; ++j){
            int class_index = entryIndex(l, 0, n*l.w*l.h + i, 5 + map[j]);
            float prob = scale*predictions[class_index];
            probs[index][j] = (prob > thresh) ? prob : 0;
          }
        } else {
          int j =  hierarchy_top_prediction(predictions + class_index, l.softmax_tree, tree_thresh, l.w*l.h);
          probs[index][j] = (scale > thresh) ? scale : 0;
#if 0 // cannot be l.classes - what is this doing here ?
          probs[index][l.classes] = scale;
#endif
        }
      }
      else {
        for(j = 0; j < l.classes; ++j){
          int class_index = entryIndex(l, 0, n*l.w*l.h + i, 5 + j);
          probs[index][j] = scale*predictions[class_index];
        }
        probs[index][l.classes] = scale;
      }
      if(only_objectness){
        probs[index][0] = scale;
        probs[index][l.classes] = scale;
      }
    }
  }
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
  float box_probs[output_size];
  for(int i=0; i<output_size; i++)
    probs[i] = new float[last_layer.classes + 1];
  getRegionBoxes(net_.layers[net_.n-1], 1, 1, 0.0, probs, box_probs, boxes, 0, 0, hier_thresh);
  if(nms != 1.f)
    do_nms_obj(boxes, probs, last_layer.w*last_layer.h*last_layer.n, last_layer.classes, nms);

  std::vector<YoloDetection> detections;
  for(int i=0; i<output_size; i++){
    if(probs[i][last_layer.classes] > thresh){                // this tests box prob
      float prob = -1.f;
      int max_idx = -1;
      for(int j = 0; j < last_layer.classes; j++){
        if(probs[i][j] > prob){
          prob = probs[i][j];
          max_idx = j;
        }
      }
      float x1 = std::max((boxes[i].x - boxes[i].w / 2.f) * img.cols, 0.f);
      float x2 = std::min((boxes[i].x + boxes[i].w / 2.f) * img.cols, img.cols-1.f);
      float y1 = std::max((boxes[i].y - boxes[i].h / 2.f) * img.rows, 0.f);
      float y2 = std::min((boxes[i].y + boxes[i].h / 2.f) * img.rows, img.rows-1.f);
      if(prob > thresh){
        detections.push_back(YoloDetection(labels_[max_idx], max_idx, probs[i], x1, x2, y1, y2, 0.f));
      }
    }
  }

  return detections;
}
