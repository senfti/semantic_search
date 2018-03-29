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
    if(room_type_viewer_ != nullptr)
      room_type_viewer_->Close();
    }
    wxExit();
  }
}

uchar colorFromNum(uchar num){
  switch(num){
    case 0:   return 0;
    case 1:   return 60;
    case 2:   return 120;
    case 3:   return 30;
    case 4:   return 90;
    case 5:   return 150;
    case 6:   return 15;
    case 7:   return 100;
    case 8:   return 130;
    case 9:   return 170;
    case 10:  return 40;
    case 11:  return 7;
    default:  return uchar(rand()%180);
  }
}

const int MIN_PER_CLASS = 1;
void ProbViewApp::showMaxClassMat(cv::Mat_<uchar>& max_idx_mat, cv::Mat_<uchar>& occ, const std::vector<std::string>& names){
  std::vector<uchar> color_nr(names.size(),0);
  std::vector<int> color_times(names.size(),0);
  cv::Mat_<uchar> color_mat(max_idx_mat.rows, max_idx_mat.cols, uchar(0));
  cv::Mat_<uchar> sat_mat(max_idx_mat.rows, max_idx_mat.cols, uchar(255));
  cv::Mat_<uchar> occ_small;
  cv::resize(occ, occ_small, cv::Size(max_idx_mat.cols, max_idx_mat.rows));
  std::vector<uchar> max_idx;
  int colors = 0;
  for(int x=0; x<max_idx_mat.cols; x++){
    for(int y=0; y<max_idx_mat.rows; y++){
      if(occ_small(y,x) <= 128 || max_idx_mat(y,x) == 255)
        continue;
      color_times[max_idx_mat(y,x)]++;
//      if(color_times[max_idx_mat(y,x)]==MIN_PER_CLASS){
//        color_nr[max_idx_mat(y, x)] = colorFromNum(colors);
//        if(std::find(max_idx.begin(), max_idx.end(), max_idx_mat(y,x)) != max_idx.end())
//          std::cout << "problem" << std::endl;
//        max_idx.push_back(max_idx_mat(y,x));
//        colors++;
//      }
    }
  }
  std::vector<size_t> idx(color_times.size());
  iota(idx.begin(), idx.end(), 0);
  sort(idx.begin(), idx.end(),[&color_times](size_t i1, size_t i2) {return color_times[i1] > color_times[i2];});
  for(int i=0; i<12; i++){
    color_nr[idx[i]] = colorFromNum(i);
    max_idx.push_back(idx[i]);
  }
  for(int i=12; i<color_times.size(); i++){
    color_times[idx[i]] = 0;
  }
  for(int x=0; x<max_idx_mat.cols; x++){
    for(int y=0; y<max_idx_mat.rows; y++){
      if(max_idx_mat(y,x) != 255 && color_times[max_idx_mat(y,x)] >= MIN_PER_CLASS && occ_small(y,x) > 128){
        color_mat(y,x) = color_nr[max_idx_mat(y,x)];
      }
      else{
        sat_mat(y,x) = 0;
      }
    }
  }
  cv::Mat_<uchar> tmp;
  cv::flip(color_mat, color_mat, 0);
  cv::flip(sat_mat, sat_mat, 0);
  cv::flip(occ, tmp, 0);
  cv::resize(color_mat, color_mat, cv::Size(8*color_mat.cols, 8*color_mat.rows), 0, 0, cv::INTER_NEAREST);
  cv::resize(sat_mat, sat_mat, cv::Size(color_mat.cols, color_mat.rows), 0, 0, cv::INTER_NEAREST);
  cv::resize(tmp, tmp, cv::Size(color_mat.cols, color_mat.rows), 0, 0, cv::INTER_NEAREST);
  cv::Mat out;
  cv::merge(std::vector<cv::Mat>({color_mat, sat_mat, tmp}), out);
  cv::cvtColor(out, out, CV_HSV2RGB);
  cv::copyMakeBorder(out,out,0,0,0,200,cv::BORDER_CONSTANT, cv::Scalar(0,0,0));
  int y = 10;
  idx = std::vector<size_t> (max_idx.size());
  iota(idx.begin(), idx.end(), 0);
  sort(idx.begin(), idx.end(),[&max_idx,&color_nr](size_t i1, size_t i2) {return color_nr[max_idx[i1]] < color_nr[max_idx[i2]];});
  for(auto& i : idx){
    int c = max_idx[i];
    if(color_times[c] < MIN_PER_CLASS)
      continue;
    cv::Mat_<cv::Vec3b> color(1,1,cv::Vec3b(color_nr[c], 255, 255));
    cv::cvtColor(color, color, cv::COLOR_HSV2RGB);
    cv::rectangle(out, cv::Point(color_mat.cols+10, y), cv::Point(color_mat.cols+20, y+10), cv::Scalar(color(0,0)[0],color(0,0)[1],color(0,0)[2]), -1);
    cv::putText(out, names[c], cv::Point(color_mat.cols+30, y+10), cv::FONT_HERSHEY_COMPLEX, 0.35, cv::Scalar(color(0,0)[0],color(0,0)[1],color(0,0)[2]), 1);
    y+=20;
  }
  room_type_viewer_->updateImage(out);
}

void ProbViewApp::placeProbCb(const prob_map_view::ProbMapMsgConstPtr& msg){
  if(msg->images.empty())
    return;

  if(place_viewer_ == nullptr){
    std::vector<std::string> names = msg->names;
    names.push_back("max");
    names.push_back("entropy");
    place_viewer_ = new ProbViewer("Place Prob Viewer", names, msg->img_are_log);
    place_viewer_->Show(true);
    room_type_viewer_ = new ImageViewer("RoomTypeClassViewer");
    room_type_viewer_->Show(true);
  }

  std::vector<cv::Mat_<float>> imgs(msg->images.size());
  cv::Mat_<float> max_mat(msg->images[0].rows, msg->images[0].cols, 1/60.f);
  cv::Mat_<float> ent_mat(msg->images[0].rows, msg->images[0].cols, 0.f);
  cv::Mat_<uchar> max_idx_mat(max_mat.rows, max_mat.cols, uchar(255));
  std::cout << "baseRoom " << msg->images[0].cols << " "  << msg->images[0].rows << std::endl;
  for(int i=0; i<imgs.size(); i++){
    cv::Mat(msg->images[i].rows, msg->images[i].cols, msg->images[i].type, (void*)(msg->images[i].data.data())).copyTo(imgs[i]);
    for(int x=0; x<imgs[i].cols; x++){
      for(int y=0; y<imgs[i].rows; y++){
        if(imgs[i](y,x) > max_mat(y,x)){
          max_mat(y,x) = imgs[i](y,x);
          max_idx_mat(y,x) = uchar(i);
        }
        ent_mat(y,x)-=imgs[i](y,x)*std::log2(imgs[i](y,x))/6.f;
      }
    }
  }
  imgs.push_back(max_mat);
  imgs.push_back(ent_mat);
  cv::Mat_<uchar> occ;
  cv::Mat(msg->occupancy.rows, msg->occupancy.cols, msg->occupancy.type, (void*)(msg->occupancy.data.data())).copyTo(occ);
  showMaxClassMat(max_idx_mat, occ, msg->names);

  place_viewer_->updateImages(imgs, occ);
}

void ProbViewApp::objProbCb(const prob_map_view::ProbMapMsgConstPtr& msg){
  if(msg->images.empty())
    return;

  if(obj_viewer_ == nullptr){
    obj_viewer_ = new ProbViewer("Object Prob Viewer", msg->names, msg->img_are_log);
    obj_viewer_->Show(true);
  }

  std::vector<cv::Mat_<float>> imgs(msg->images.size());
  std::cout << "Obj " << msg->images[0].cols << " "  << msg->images[0].rows << std::endl;
  for(int i=0; i<imgs.size(); i++){
    cv::Mat(msg->images[i].rows, msg->images[i].cols, msg->images[i].type, (void*)(msg->images[i].data.data())).copyTo(imgs[i]);
  }
  cv::Mat_<uchar> occ;
  cv::Mat(msg->occupancy.rows, msg->occupancy.cols, msg->occupancy.type, (void*)(msg->occupancy.data.data())).copyTo(occ);
  occ = cv::max(occ, 128);

  obj_viewer_->updateImages(imgs, occ);
}

void ProbViewApp::baseObjProbCb(const prob_map_view::ProbMapMsgConstPtr& msg){
  if(msg->images.empty())
    return;

  std::cout << "loc" << std::endl;
  if(base_obj_viewer_ == nullptr){
    base_obj_viewer_ = new ProbViewer("Base Object Prob Viewer", msg->names, msg->img_are_log);
    base_obj_viewer_->Show(true);
  }

  std::vector<cv::Mat_<float>> imgs(msg->images.size());
  std::cout << "baseObj " << msg->images[0].cols << " "  << msg->images[0].rows << std::endl;
  for(int i=0; i<imgs.size(); i++){
    cv::Mat(msg->images[i].rows, msg->images[i].cols, msg->images[i].type, (void*)(msg->images[i].data.data())).copyTo(imgs[i]);
  }
  cv::Mat_<uchar> occ;
  cv::Mat(msg->occupancy.rows, msg->occupancy.cols, msg->occupancy.type, (void*)(msg->occupancy.data.data())).copyTo(occ);
  occ = cv::max(occ, 128);

  base_obj_viewer_->updateImages(imgs, occ);
}

void ProbViewApp::baseRoomProbCb(const prob_map_view::ProbMapMsgConstPtr &msg){
  if(msg->images.empty())
    return;

  std::cout << "loc" << std::endl;
  if(base_room_viewer_ == nullptr){
    base_room_viewer_ = new ProbViewer("Base Room Prob Viewer", msg->names, msg->img_are_log);
    base_room_viewer_->Show(true);
  }

  std::vector<cv::Mat_<float>> imgs(msg->images.size());
  std::cout << "baseRoom " << msg->images[0].cols << " "  << msg->images[0].rows << std::endl;
  for(int i=0; i<imgs.size(); i++){
    cv::Mat(msg->images[i].rows, msg->images[i].cols, msg->images[i].type, (void*)(msg->images[i].data.data())).copyTo(imgs[i]);
  }
  cv::Mat_<uchar> occ;
  cv::Mat(msg->occupancy.rows, msg->occupancy.cols, msg->occupancy.type, (void*)(msg->occupancy.data.data())).copyTo(occ);

  base_room_viewer_->updateImages(imgs, occ);
}

void ProbViewApp::sdfProbCb(const prob_map_view::ProbMapMsgConstPtr &msg){
  if(msg->images.empty())
    return;

  int idx = msg->img_are_log;
  if(sdf_viewer_.size() <= idx)
    sdf_viewer_.resize(idx+1, nullptr);
  if(sdf_viewer_[idx] == nullptr){
    sdf_viewer_[idx] = new ProbViewer("sdf"+wxString(std::to_string(idx)), msg->names, 0);
    sdf_viewer_[idx]->Show(true);
  }

  std::vector<cv::Mat_<float>> imgs(msg->images.size());
  std::cout << "sdf" << idx << " " << msg->images[0].cols << " "  << msg->images[0].rows << std::endl;
  for(int i=0; i<imgs.size(); i++){
    cv::Mat(msg->images[i].rows, msg->images[i].cols, msg->images[i].type, (void*)(msg->images[i].data.data())).copyTo(imgs[i]);
  }
  cv::Mat_<uchar> occ;
  cv::Mat(msg->occupancy.rows, msg->occupancy.cols, msg->occupancy.type, (void*)(msg->occupancy.data.data())).copyTo(occ);
  occ = cv::max(occ, 128);

  sdf_viewer_[idx]->updateImages(imgs, occ);
}

void ProbViewApp::saveAllCb(const std_msgs::StringConstPtr &msg){
  saveAll(msg->data);
}

void ProbViewApp::saveAll(const std::string& postfix){
  std::string folder = "/home/thomas/Masterarbeit/output/images"+std::to_string(ros::Time::now().toSec())+"/";
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
  if(room_type_viewer_ != nullptr)
    room_type_viewer_->save(folder, postfix);
}
