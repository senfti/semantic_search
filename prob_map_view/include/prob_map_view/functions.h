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
template<>
inline cv::Mat_<float> pToL(cv::Mat_<float> p){
  cv::Mat_<float> res;
  cv::log(p / (1-p), res);
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
