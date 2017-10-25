#include "prob_map_view/main.h"
#include <cv_bridge/cv_bridge.h>

ProbViewApp* prob_view_app;
IMPLEMENT_APP(ProbViewApp)

ProbViewApp::ProbViewApp(){
}

ProbViewApp::~ProbViewApp(){
}

bool ProbViewApp::OnInit(){
  prob_view_app = this;
  ros::init (argc, argv, "prob_map_view");
  node_handle_ = new ros::NodeHandle();
  place_sub_ = node_handle_->subscribe("place_maps", 1, &ProbViewApp::placeProbCb, this);
  obj_sub_ = node_handle_->subscribe("object_maps", 1, &ProbViewApp::objProbCb, this);

  input_timer_ = new wxTimer(this);
  Connect(input_timer_->GetId(), wxEVT_TIMER, wxTimerEventHandler(ProbViewApp::input), NULL, this);
  input_timer_->Start(100);
  return true;
}

int ProbViewApp::OnExit(){
  return 0;
}

void ProbViewApp::input(wxTimerEvent &event){
  ros::spinOnce();
}

void ProbViewApp::placeProbCb(const prob_map_view::ProbMapMsgConstPtr& msg){
  if(place_viewer_ == nullptr){
    place_viewer_ = new ProbViewer("Place Prob Viewer", msg->names, msg->img_are_log);
    place_viewer_->Show(true);
  }

  std::vector<cv::Mat_<double>> imgs(msg->images.size());
  for(int i=0; i<imgs.size(); i++){
    cv::Mat(msg->images[i].rows, msg->images[i].cols, msg->images[i].type, (void*)(msg->images[i].data.data())).copyTo(imgs[i]);
  }

  place_viewer_->updateImages(imgs);
}

void ProbViewApp::objProbCb(const prob_map_view::ProbMapMsgConstPtr& msg){
  if(obj_viewer_ == nullptr){
    obj_viewer_ = new ProbViewer("Object Prob Viewer", msg->names, msg->img_are_log);
    obj_viewer_->Show(true);
  }

  std::vector<cv::Mat_<double>> imgs(msg->images.size());
  for(int i=0; i<imgs.size(); i++){
    cv::Mat(msg->images[i].rows, msg->images[i].cols, msg->images[i].type, (void*)(msg->images[i].data.data())).copyTo(imgs[i]);
  }

  obj_viewer_->updateImages(imgs);
}

