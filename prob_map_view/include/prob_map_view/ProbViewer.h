//
// Created by thomas on 23.10.17.
//

#ifndef SEMANTIC_MAPPING_PROBVIEWER_H
#define SEMANTIC_MAPPING_PROBVIEWER_H

#include "../../src/ProbViewer_B.h"
#include <string>
#include <opencv2/opencv.hpp>
#include <wx/choice.h>

class ProbViewer : public ProbViewer_B{
  protected:
    std::vector<cv::Mat_<double>> prob_images_;
    cv::Mat_<uchar> occ_image_;
    std::vector<std::string> prob_names_;
    cv::Mat_<double> curr_img_;
    bool img_are_log_;
    wxChoice* select_choice_;

  public:
    ProbViewer(const wxString& win_name, const std::vector<std::string>& prob_names, bool img_are_log);     //very dangerous

    virtual void onChoice( wxCommandEvent& event );
    virtual void onCheck( wxCommandEvent& event );
    virtual void onMouseMove( wxMouseEvent& event );

    void updateImages(const std::vector<cv::Mat_<double>>& prob_images, const cv::Mat_<uchar>& occ);
    void setCurrent();
};

#endif //SEMANTIC_MAPPING_PROBVIEWER_H
