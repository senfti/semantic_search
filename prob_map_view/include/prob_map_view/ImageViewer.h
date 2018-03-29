//
// Created by thomas on 26.03.18.
//

#ifndef PROB_MAP_VIEW_IMAGEVIEWER_H
#define PROB_MAP_VIEW_IMAGEVIEWER_H

#include "../../src/ImageViewer_B.h"
#include "prob_map_view/ImagePanel.h"
#include <vector>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>

class ImageViewer : public ImageViewer_B{
  protected:
    cv::Mat img_;

  public:
    ImageViewer(const wxString& win_name);
    virtual void exit( wxCommandEvent& event ){
      ros::shutdown();
    }
    virtual void activate( wxActivateEvent& event );

    void updateImage(const cv::Mat& img);
    void save(const std::string& folder, const std::string& postfix = std::string(""));
};

#endif //PROB_MAP_VIEW_IMAGEVIEWER_H
