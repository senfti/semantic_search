//
// Created by thomas on 13.08.17.
//

#ifndef PROB_VIEW_MAIN_H
#define PROB_VIEW_MAIN_H

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <wx/wx.h>
#include <prob_map_view/ProbMapMsg.h>
#include "prob_map_view/ProbViewer.h"
#include "prob_map_view/ImageViewer.h"
#include <std_msgs/String.h>


class ProbViewApp : public wxApp{
  private:
    ros::NodeHandle* node_handle_ = nullptr;
    ros::Subscriber place_sub_;
    ros::Subscriber obj_sub_;
    ros::Subscriber base_obj_sub_;
    ros::Subscriber base_room_sub_;
    ros::Subscriber search_sub_;
    ros::Subscriber sdf_sub_;
    ros::Subscriber save_all_sub_;

    ProbViewer* place_viewer_ = nullptr;
    ProbViewer* obj_viewer_ = nullptr;
    ProbViewer* base_obj_viewer_ = nullptr;
    ProbViewer* base_room_viewer_ = nullptr;
    ProbViewer* search_prob_viewer_ = nullptr;
    ImageViewer* room_type_viewer_ = nullptr;
    ImageViewer* base_room_type_viewer_ = nullptr;
    std::vector<ProbViewer*> sdf_viewer_;

    wxTimer* input_timer_ = nullptr;

  public:
    ProbViewApp();
    ~ProbViewApp();

    virtual bool OnInit();
    virtual int OnExit();
    void input(wxTimerEvent &event);

    void showMaxClassMat(cv::Mat_<uchar>& max_idx_mat, cv::Mat_<uchar>& occ, const std::vector<std::string>& names, bool base);

    void placeProbCb(const prob_map_view::ProbMapMsgConstPtr& msg);
    void objProbCb(const prob_map_view::ProbMapMsgConstPtr& msg);
    void baseObjProbCb(const prob_map_view::ProbMapMsgConstPtr& msg);
    void baseRoomProbCb(const prob_map_view::ProbMapMsgConstPtr& msg);
    void searchProbCb(const prob_map_view::ProbMapMsgConstPtr& msg);
    void sdfProbCb(const prob_map_view::ProbMapMsgConstPtr& msg);
    void saveAllCb(const std_msgs::StringConstPtr& msg);

    void saveAll(const std::string& postfix = std::string(""), std::string folder = std::string());
};

extern ProbViewApp* prob_view_app;

#endif //PROB_VIEW_MAIN_H
