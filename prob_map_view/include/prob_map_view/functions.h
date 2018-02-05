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
  p.copyTo(res);
  for (double &v : cv::Mat_<double>(res)) {
    v = std::log10(v); v = (v > -3.0 ? (3.0+v)/3.0 : 0.0);
  }
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
