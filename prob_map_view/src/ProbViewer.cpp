//
// Created by thomas on 23.10.17.
//

#include "prob_map_view/ProbViewer.h"
#include "prob_map_view/functions.h"
#include <wx/dcbuffer.h>
#include "prob_map_view/ImagePanel.h"

#include "prob_map_view/main.h"

ProbViewer::ProbViewer(const wxString& win_name, const std::vector<std::string>& prob_names, bool img_are_log, bool default_max)
  : ProbViewer_B(nullptr, wxID_ANY, win_name), prob_images_(prob_names.size()), prob_names_(prob_names), img_are_log_(img_are_log), default_max_(default_max)
{
  wxArrayString strings;
  for(const auto& n : prob_names_)
    strings.push_back(wxString(n));
  select_choice_ = new wxChoice( this, wxID_ANY, wxDefaultPosition, wxSize( 250,-1 ), strings, 0 );
  select_choice_->SetSelection( 0 );
  bSizer2->Add( select_choice_, 0, wxALL, 5 );
  image_panel_->SetBackgroundStyle(wxBG_STYLE_PAINT);
  select_choice_->Connect( wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler( ProbViewer::onChoice ), NULL, this );
  if(default_max_)
    rescale_checkbox_->SetValue(true);
}

void ProbViewer::onChoice( wxCommandEvent& event ){
  std::cout << "choice" << std::endl;
  if(use2)
    setCurrent2();
  else
    setCurrent();
}

void ProbViewer::onCheck( wxCommandEvent& event ){
  std::cout << "check" << std::endl;
  setCurrent();
}

void ProbViewer::onMouseMove( wxMouseEvent& event ){
  int x = event.GetX(), y = event.GetY();
  if(event.GetX() < curr_img_.cols && event.GetY() < curr_img_.rows){
    current_prob_text_->SetLabel(wxString::Format("%.5f", curr_img_(y,x)));
    curr_log_text_->SetLabel(wxString::Format("%.5f", std::log(curr_img_(y,x))));
  }
}

void ProbViewer::updateImages(const std::vector<cv::Mat_<float>>& prob_images, const cv::Mat_<uchar>& occ){
  prob_images_ = prob_images;
  occ_image_ = occ;
  setCurrent();
}

void ProbViewer::setCurrent(){
  if(select_choice_->GetSelection() >= prob_images_.size())
    select_choice_->SetSelection(prob_images_.size()-1);

  prob_images_[select_choice_->GetSelection()].copyTo(curr_img_);
  if(curr_img_.cols > 0){
    if(img_are_log_ && !log_checkbox_->IsChecked())
      curr_img_ = lToP(curr_img_);
    else if(!img_are_log_ && log_checkbox_->IsChecked())
      curr_img_ = pToL(curr_img_, (prob_names_.size() == 80 ? -6.0 : -3.0));

    double min, max;
    cv::minMaxIdx(curr_img_, &min, &max);
    min_text_->SetLabel(wxString::Format("%.5lf", min));
    max_text_->SetLabel(wxString::Format("%.5lf", max));

    cv::flip(curr_img_, curr_img_, 0);
    int scale_factor = 8;//std::min(1200 / curr_img_.cols, 800 / curr_img_.rows);
    cv::resize(curr_img_, curr_img_, cv::Size(curr_img_.cols*scale_factor, curr_img_.rows*scale_factor), 0, 0, cv::INTER_NEAREST);

    cv::Mat out;
    if(rescale_checkbox_->IsChecked()){
      if(min == max)
        out = cv::Mat_<float>::ones(curr_img_.rows, curr_img_.cols);
      else
        out = ((curr_img_ - min) / (max - min));
      out.convertTo(out, CV_8U, 150.f);
    }
    else
      curr_img_.convertTo(out, CV_8U, 150.f);

    cv::Mat tmp(out.rows, out.cols, CV_8UC1, cv::Scalar(255)), tmp2;
    cv::flip(occ_image_, tmp2, 0);
    cv::resize(tmp2, tmp2, cv::Size(out.cols, out.rows), 0, 0, cv::INTER_NEAREST);
    cv::merge(std::vector<cv::Mat>({out, tmp, tmp2}), out);
    cv::cvtColor(out, out, CV_HSV2RGB);

    image_panel_->setImage(out);
    if(IsActive()){
      Fit();
      Refresh();
    }
  }
}

void ProbViewer::updateImages2(const std::vector<cv::Mat_<float>>& prob_images, const cv::Mat_<uchar>& occ){
  use2 = true;
  prob_images_ = prob_images;
  occ_image_ = occ;
  setCurrent2();
}

void ProbViewer::setCurrent2(){
  if(select_choice_->GetSelection() >= prob_images_.size())
    select_choice_->SetSelection(prob_images_.size()-1);

  prob_images_[select_choice_->GetSelection()].copyTo(curr_img_);
  if(curr_img_.cols > 0){
    if(img_are_log_ && !log_checkbox_->IsChecked())
      curr_img_ = lToP(curr_img_);
    else if(!img_are_log_ && log_checkbox_->IsChecked())
      curr_img_ = pToL(curr_img_, (prob_names_.size() == 80 ? -6.0 : -3.0));

    double min, max;
    cv::minMaxIdx(curr_img_, &min, &max);
    min_text_->SetLabel(wxString::Format("%.5lf", min));
    max_text_->SetLabel(wxString::Format("%.5lf", max));

    cv::flip(curr_img_, curr_img_, 0);
    int scale_factor = 8;//std::min(1200 / curr_img_.cols, 800 / curr_img_.rows);
    cv::resize(curr_img_, curr_img_, cv::Size(curr_img_.cols*scale_factor, curr_img_.rows*scale_factor), 0, 0, cv::INTER_NEAREST);

    cv::Mat out;
    if(rescale_checkbox_->IsChecked()){
      if(min == max)
        out = cv::Mat_<float>::ones(curr_img_.rows, curr_img_.cols);
      else
        out = ((curr_img_ - min) / (max - min));
      out.convertTo(out, CV_8U, 150.f);
    }
    else
      curr_img_.convertTo(out, CV_8U, 150.f);

    cv::Mat tmp(out.rows, out.cols, CV_8UC1, cv::Scalar(255)), tmp2;
    cv::flip(occ_image_, tmp2, 0);
    cv::resize(tmp2, tmp2, cv::Size(out.cols, out.rows), 0, 0, cv::INTER_NEAREST);
    cv::merge(std::vector<cv::Mat>({out, tmp, tmp2}), out);
    cv::cvtColor(out, out, CV_HSV2RGB);

    cv::merge(std::vector<cv::Mat>({tmp2, tmp2, tmp2}), out);
    for(int x=0; x<out.cols; x++){
      for(int y=0; y<out.rows; y++){
        if(tmp2.at<uchar>(y,x) > 250)
          out.at<cv::Vec3b>(y,x) = cv::Vec3b(230,230,230);
        else if(tmp2.at<uchar>(y,x) > 192)
          out.at<cv::Vec3b>(y,x) = cv::Vec3b(0,0,0);
        else
          out.at<cv::Vec3b>(y,x) = cv::Vec3b(255,255,255);
        if(curr_img_(y,x) > 0.25)
          out.at<cv::Vec3b>(y,x) = cv::Vec3b(255,0,0);
      }
    }

    image_panel_->setImage(out);
    if(IsActive()){
      Fit();
      Refresh();
    }
  }
}


void ProbViewer::saveAll( wxCommandEvent& event ){
  prob_view_app->saveAll();
}

void ProbViewer::save(const std::string& folder, const std::string& postfix){
  if(use2){
    int i=0;
    for(auto &image : prob_images_){
      cv::Mat_<float> img;
      image.copyTo(img);
      if(img.cols > 0){
        if(img_are_log_ && !log_checkbox_->IsChecked())
          img = lToP(img);
        else if(!img_are_log_ && log_checkbox_->IsChecked())
          img = pToL(img, (prob_names_.size() == 80 ? -6.0 : -3.0));

        double min, max;
        cv::minMaxIdx(img, &min, &max);

        cv::flip(img, img, 0);
        int scale_factor = 8;//std::min(1200 / img.cols, 800 / img.rows);
        cv::resize(img, img, cv::Size(img.cols*scale_factor, img.rows*scale_factor), 0, 0, cv::INTER_NEAREST);

        cv::Mat out;
        if(rescale_checkbox_->IsChecked()){
          if(min == max)
            out = cv::Mat_<float>::ones(img.rows, img.cols);
          else
            out = ((img - min) / (max - min));
          out.convertTo(out, CV_8U, 150.f);
        }
        else
          img.convertTo(out, CV_8U, 150.f);

        cv::Mat tmp(out.rows, out.cols, CV_8UC1, cv::Scalar(255)), tmp2;
        cv::flip(occ_image_, tmp2, 0);
        cv::resize(tmp2, tmp2, cv::Size(out.cols, out.rows), 0, 0, cv::INTER_NEAREST);
        cv::merge(std::vector<cv::Mat>({out, tmp, tmp2}), out);
        cv::cvtColor(out, out, CV_HSV2RGB);

        cv::merge(std::vector<cv::Mat>({tmp2, tmp2, tmp2}), out);
        for(int x=0; x<out.cols; x++){
          for(int y=0; y<out.rows; y++){
            if(tmp2.at<uchar>(y,x) > 250)
              out.at<cv::Vec3b>(y,x) = cv::Vec3b(230,230,230);
            else if(tmp2.at<uchar>(y,x) > 192)
              out.at<cv::Vec3b>(y,x) = cv::Vec3b(0,0,0);
            else
              out.at<cv::Vec3b>(y,x) = cv::Vec3b(255,255,255);
            if(img(y,x) > 0.25)
              out.at<cv::Vec3b>(y,x) = cv::Vec3b(255,0,0);
          }
        }
        cv::cvtColor(out, out, CV_RGB2BGR);
        cv::imwrite(folder + "/" + std::string(GetTitle()) + "_" + prob_names_[i] + postfix + std::to_string(max) + ".png", out);
        i++;
      }
    }
    return;
  }
  int i=0;
  for(auto &image : prob_images_){
    cv::Mat_<float> img;
    image.copyTo(img);
    if(img.cols > 0){
      if(img_are_log_ && !log_checkbox_->IsChecked())
        img = lToP(img);
      else if(!img_are_log_ && log_checkbox_->IsChecked())
        img = pToL(img, (prob_names_.size() == 80 ? -4.0 : -3.0));

      double min, max;
      cv::minMaxIdx(img, &min, &max);

      cv::flip(img, img, 0);
      int scale_factor = 8;
      cv::resize(img, img, cv::Size(img.cols*scale_factor, img.rows*scale_factor), 0, 0, cv::INTER_NEAREST);

      cv::Mat out;
      if(rescale_checkbox_->IsChecked()){
        if(min == max)
          out = cv::Mat_<float>::ones(img.rows, img.cols);
        else
          out = ((img - min) / (max - min));
        out.convertTo(out, CV_8U, 120.f);
      }
      else
        img.convertTo(out, CV_8U, 120.f);

      cv::Mat tmp(out.rows, out.cols, CV_8UC1, cv::Scalar(255)), tmp2;
      cv::flip(occ_image_, tmp2, 0);
      cv::resize(tmp2, tmp2, cv::Size(out.cols, out.rows), 0, 0, cv::INTER_NEAREST);
      cv::merge(std::vector<cv::Mat>({out, tmp, tmp2}), out);
      cv::cvtColor(out, out, CV_HSV2BGR);
      for(int x=0; x<out.cols; x++){
        for(int y=0; y<out.rows; y++){
          if(out.at<cv::Vec3b>(y,x) == cv::Vec3b(0,0,128))
            out.at<cv::Vec3b>(y,x) = cv::Vec3b(255,255,255);
        }
      }

      cv::imwrite(folder + "/" + std::string(GetTitle()) + "_" + prob_names_[i] + postfix + std::to_string(max) + ".png", out);
      i++;
    }
  }
}

void ProbViewer::activate(wxActivateEvent &event){
  image_panel_->updateSize();
  Fit();
  Refresh();
}
