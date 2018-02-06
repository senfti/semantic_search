//
// Created by thomas on 23.10.17.
//

#include "prob_map_view/ProbViewer.h"
#include "prob_map_view/functions.h"
#include <wx/dcbuffer.h>
#include "prob_map_view/ImagePanel.h"

#include "prob_map_view/main.h"

ProbViewer::ProbViewer(const wxString& win_name, const std::vector<std::string>& prob_names, bool img_are_log)
  : ProbViewer_B(nullptr, wxID_ANY, win_name), prob_images_(prob_names.size()), prob_names_(prob_names), img_are_log_(img_are_log)
{
  wxArrayString strings;
  for(const auto& n : prob_names_)
    strings.push_back(wxString(n));
  select_choice_ = new wxChoice( this, wxID_ANY, wxDefaultPosition, wxSize( 250,-1 ), strings, 0 );
  select_choice_->SetSelection( 0 );
  bSizer2->Add( select_choice_, 0, wxALL, 5 );
  image_panel_->SetBackgroundStyle(wxBG_STYLE_PAINT);
  select_choice_->Connect( wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler( ProbViewer::onChoice ), NULL, this );
}

void ProbViewer::onChoice( wxCommandEvent& event ){
  std::cout << "choice" << std::endl;
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

void ProbViewer::updateImages(const std::vector<cv::Mat_<double>>& prob_images, const cv::Mat_<uchar>& occ){
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
      curr_img_ = pToL(curr_img_);

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
        out = cv::Mat_<double>::ones(curr_img_.rows, curr_img_.cols);
      else
        out = ((curr_img_ - min) / (max - min));
      out.convertTo(out, CV_8U, 150.f);
    }
    else
      curr_img_.convertTo(out, CV_8U, 150.f);

    cv::Mat tmp(out.rows, out.cols, CV_8UC1, cv::Scalar(255)), tmp2;
    cv::flip(occ_image_, tmp2, 0);
    cv::resize(tmp2, tmp2, cv::Size(out.cols, out.rows), 0, 0, cv::INTER_NEAREST);
    std::cout << (out.type()&255) << " " << (tmp.type()&255) << " " << (tmp2.type()&255) << std::endl;
    std::cout << (out.depth()&255) << " " << (tmp.depth()&255) << " " << (tmp2.depth()&255) << std::endl;
    std::cout << (out.size().width) << " " << (out.size().height) << (tmp2.size().width) << " " << " " << (tmp2.size().height) << std::endl;
    cv::merge(std::vector<cv::Mat>({out, tmp, tmp2}), out);
    cv::cvtColor(out, out, CV_HSV2RGB);

    image_panel_->setImage(out);
    Fit();
    Refresh();
  }
}

void ProbViewer::saveAll( wxCommandEvent& event ){
  prob_view_app->saveAll();
}

void ProbViewer::save(const std::string& folder){
  int i=0;
  for(auto &img : prob_images_){
    if(img.cols > 0){
      if(img_are_log_ && !log_checkbox_->IsChecked())
        img = lToP(img);
      else if(!img_are_log_ && log_checkbox_->IsChecked())
        img = pToL(img);

      double min, max;
      cv::minMaxIdx(img, &min, &max);
      min_text_->SetLabel(wxString::Format("%.5lf", min));
      max_text_->SetLabel(wxString::Format("%.5lf", max));

      cv::flip(img, img, 0);
      int scale_factor = 8;
      cv::resize(img, img, cv::Size(img.cols*scale_factor, img.rows*scale_factor), 0, 0, cv::INTER_NEAREST);

      cv::Mat out;
      if(rescale_checkbox_->IsChecked()){
        if(min == max)
          out = cv::Mat_<double>::ones(img.rows, img.cols);
        else
          out = ((img - min) / (max - min));
        out.convertTo(out, CV_8U, 150.f);
      }
      else
        img.convertTo(out, CV_8U, 150.f);

      cv::Mat tmp(out.rows, out.cols, CV_8UC1, cv::Scalar(255)), tmp2;
      cv::flip(occ_image_, tmp2, 0);
      cv::resize(tmp2, tmp2, cv::Size(out.rows, out.cols), 0, 0, cv::INTER_NEAREST);
      cv::merge(std::vector<cv::Mat>({out, tmp, tmp2}), out);
      cv::cvtColor(out, out, CV_HSV2BGR);

      cv::imwrite(folder + "/" + std::string(GetTitle()) + prob_names_[i] + ".png", out);
      i++;
    }
  }

}
