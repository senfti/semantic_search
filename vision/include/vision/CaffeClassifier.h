#ifndef VISION_CAFFECLASSIFIER_H
#define VISION_CAFFECLASSIFIER_H

#include <string>
#include <iostream>
#include <vector>
#include <caffe/caffe.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class CaffeRecognition{
  public:
    std::string label_;
    int id_;
    float prob_;

    CaffeRecognition(const std::string& label, int id, float prob)
          : label_(label), id_(id), prob_(prob){
    }
};

// based on caffe classification example
class CaffeClassifier {
  private:
    caffe::Net<float>* net_;
    cv::Size input_geometry_;
    int num_channels_;
    cv::Mat mean_;
    std::vector<std::string> labels_;
    bool is_ok_ = false;

    void setMean(const std::string& mean_file);
    std::vector<float> predict(const cv::Mat& img);
    void wrapInputLayer(std::vector<cv::Mat>* input_channels);
    void preprocess(const cv::Mat& img, std::vector<cv::Mat>* input_channels);

  public:
    CaffeClassifier(const std::string& model_file, const std::string& trained_file,
                    const std::string& mean_file, const std::string& label_file);
    ~CaffeClassifier();
    std::vector<CaffeRecognition> classify(const cv::Mat& img, int N = 5);
    std::vector<CaffeRecognition> classify(const cv::Mat& img, float min_prob);
    bool isOk() const { return is_ok_; }
    std::string getName(int id) const { return labels_[id]; }
};

#endif //VISION_CAFFECLASSIFIER_H
