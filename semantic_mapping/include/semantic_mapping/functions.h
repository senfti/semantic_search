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
inline cv::Mat_<double> pToL(cv::Mat_<double> p){
  cv::Mat_<double> res;
  cv::log(p / (1-p), res);
  return res;
}

template<typename T>
inline T lToP(T l) {
  return 1 - 1 / (1 + std::exp(l));
}
template<>
inline cv::Mat_<double> lToP(cv::Mat_<double> l){
  cv::Mat_<double> tmp;
  cv::exp(l, tmp);
  return tmp / (1+tmp);
}

#endif //SEMANTIC_MAPPING_FUNCTIONS_H
