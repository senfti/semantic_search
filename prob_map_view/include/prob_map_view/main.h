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


class ProbViewApp : public wxApp{
  private:
    ros::NodeHandle* node_handle_ = nullptr;
    ros::Subscriber place_sub_;
    ros::Subscriber obj_sub_;
    ros::Subscriber obj_loc_sub_;

    ProbViewer* place_viewer_ = nullptr;
    ProbViewer* obj_viewer_ = nullptr;
    ProbViewer* obj_loc_viewer_ = nullptr;

    wxTimer* input_timer_ = nullptr;

  public:
    ProbViewApp();
    ~ProbViewApp();

    virtual bool OnInit();
    virtual int OnExit();
    void input(wxTimerEvent &event);

    void placeProbCb(const prob_map_view::ProbMapMsgConstPtr& msg);
    void objProbCb(const prob_map_view::ProbMapMsgConstPtr& msg);
    void objLocProbCb(const prob_map_view::ProbMapMsgConstPtr& msg);
};

#endif //PROB_VIEW_MAIN_H
