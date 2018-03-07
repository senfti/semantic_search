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
  place_sub_ = node_handle_->subscribe("room_prob_map_view", 1, &ProbViewApp::placeProbCb, this);
  obj_sub_ = node_handle_->subscribe("obj_prob_map_view", 1, &ProbViewApp::objProbCb, this);
  base_obj_sub_ = node_handle_->subscribe("base_obj_prob_map_view", 1, &ProbViewApp::baseObjProbCb, this);
  base_room_sub_ = node_handle_->subscribe("base_room_prob_map_view", 1, &ProbViewApp::baseRoomProbCb, this);
  sdf_sub_ = node_handle_->subscribe("sdf", 10, &ProbViewApp::sdfProbCb, this);
  save_all_sub_ = node_handle_->subscribe("save_all", 1, &ProbViewApp::saveAllCb, this);

  input_timer_ = new wxTimer(this);
  Connect(input_timer_->GetId(), wxEVT_TIMER, wxTimerEventHandler(ProbViewApp::input), NULL, this);
  input_timer_->Start(100);
  return true;
}

int ProbViewApp::OnExit(){
  return 0;
}

void ProbViewApp::input(wxTimerEvent &event){
  if(ros::ok())
    ros::spinOnce();
  else{
    if(place_viewer_ != nullptr)
      place_viewer_->Close();
    if(obj_viewer_ != nullptr)
      obj_viewer_->Close();
    if(base_obj_viewer_ != nullptr)
      base_obj_viewer_->Close();
    if(base_room_viewer_ != nullptr)
      base_room_viewer_->Close();
    for(auto & v : sdf_viewer_){
      if(v != nullptr)
        v->Close();
    }
    wxExit();
  }
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
  cv::Mat_<uchar> occ;
  cv::Mat(msg->occupancy.rows, msg->occupancy.cols, msg->occupancy.type, (void*)(msg->occupancy.data.data())).copyTo(occ);

  place_viewer_->updateImages(imgs, occ);
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
  cv::Mat_<uchar> occ;
  cv::Mat(msg->occupancy.rows, msg->occupancy.cols, msg->occupancy.type, (void*)(msg->occupancy.data.data())).copyTo(occ);

  obj_viewer_->updateImages(imgs, occ);
}

void ProbViewApp::baseObjProbCb(const prob_map_view::ProbMapMsgConstPtr& msg){
  std::cout << "loc" << std::endl;
  if(base_obj_viewer_ == nullptr){
    base_obj_viewer_ = new ProbViewer("Base Object Prob Viewer", msg->names, msg->img_are_log);
    base_obj_viewer_->Show(true);
  }

  std::vector<cv::Mat_<double>> imgs(msg->images.size());
  for(int i=0; i<imgs.size(); i++){
    cv::Mat(msg->images[i].rows, msg->images[i].cols, msg->images[i].type, (void*)(msg->images[i].data.data())).copyTo(imgs[i]);
  }
  cv::Mat_<uchar> occ;
  cv::Mat(msg->occupancy.rows, msg->occupancy.cols, msg->occupancy.type, (void*)(msg->occupancy.data.data())).copyTo(occ);

  base_obj_viewer_->updateImages(imgs, occ);
}

void ProbViewApp::baseRoomProbCb(const prob_map_view::ProbMapMsgConstPtr &msg){
  std::cout << "loc" << std::endl;
  if(base_room_viewer_ == nullptr){
    base_room_viewer_ = new ProbViewer("Base Room Prob Viewer", msg->names, msg->img_are_log);
    base_room_viewer_->Show(true);
  }

  std::vector<cv::Mat_<double>> imgs(msg->images.size());
  for(int i=0; i<imgs.size(); i++){
    cv::Mat(msg->images[i].rows, msg->images[i].cols, msg->images[i].type, (void*)(msg->images[i].data.data())).copyTo(imgs[i]);
  }
  cv::Mat_<uchar> occ;
  cv::Mat(msg->occupancy.rows, msg->occupancy.cols, msg->occupancy.type, (void*)(msg->occupancy.data.data())).copyTo(occ);

  base_room_viewer_->updateImages(imgs, occ);
}

void ProbViewApp::sdfProbCb(const prob_map_view::ProbMapMsgConstPtr &msg){
  int idx = msg->img_are_log;
  if(sdf_viewer_.size() <= idx)
    sdf_viewer_.resize(idx+1, nullptr);
  if(sdf_viewer_[idx] == nullptr){
    sdf_viewer_[idx] = new ProbViewer("sdf"+wxString(std::to_string(idx)), msg->names, 0);
    sdf_viewer_[idx]->Show(true);
  }

  std::vector<cv::Mat_<double>> imgs(msg->images.size());
  for(int i=0; i<imgs.size(); i++){
    cv::Mat(msg->images[i].rows, msg->images[i].cols, msg->images[i].type, (void*)(msg->images[i].data.data())).copyTo(imgs[i]);
  }
  cv::Mat_<uchar> occ;
  cv::Mat(msg->occupancy.rows, msg->occupancy.cols, msg->occupancy.type, (void*)(msg->occupancy.data.data())).copyTo(occ);

  sdf_viewer_[idx]->updateImages(imgs, occ);
}

void ProbViewApp::saveAllCb(const std_msgs::StringConstPtr &msg){
  saveAll(msg->data);
}

void ProbViewApp::saveAll(const std::string& postfix){
  std::string folder = "/media/thomas/EE702FC8702F967D/studium/Masterarbeit/output/hierarchy/";
  wxMkdir(folder);
  if(place_viewer_ != nullptr)
    place_viewer_->save(folder, postfix);
  if(obj_viewer_ != nullptr)
    obj_viewer_->save(folder, postfix);
  if(base_obj_viewer_ != nullptr)
    base_obj_viewer_->save(folder, postfix);
  if(base_room_viewer_ != nullptr)
    base_room_viewer_->save(folder, postfix);
  for(auto & v : sdf_viewer_){
    if(v != nullptr)
      v->save(folder, postfix);
  }
}
