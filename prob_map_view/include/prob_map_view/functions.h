//
// Created by thomas on 24.10.17.
//

#ifndef SEMANTIC_MAPPING_FUNCTIONS_H
#define SEMANTIC_MAPPING_FUNCTIONS_H

#include <opencv2/opencv.hpp>

template<typename T>
inline T pToL(T p) {
  return std::log(p / (1-p));
}

inline cv::Mat_<float> pToL(cv::Mat_<float> p, float min_log){
  cv::Mat_<float> res;
  p.copyTo(res);
  for (float &v : cv::Mat_<float>(res)) {
    v = std::log10(v); v = (v > min_log ? (-min_log+v)/(-min_log) : 0.0);
  }
  return res;
}

template<typename T>
inline T lToP(T l) {
  return 1 - 1 / (1 + std::exp(l));
}
template<>
inline cv::Mat_<float> lToP(cv::Mat_<float> l){
  cv::Mat_<float> tmp;
  cv::exp(l, tmp);
  return tmp / (1+tmp);
}

#endif //SEMANTIC_MAPPING_FUNCTIONS_H
