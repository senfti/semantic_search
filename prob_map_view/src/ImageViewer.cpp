//
// Created by thomas on 26.03.18.
//
#include <prob_map_view/ImageViewer.h>

ImageViewer::ImageViewer(const wxString &win_name)
  : ImageViewer_B(nullptr, wxID_ANY, win_name)
{
  image_panel_->SetBackgroundStyle(wxBG_STYLE_PAINT);
}

void ImageViewer::updateImage(const cv::Mat& img){
  img.copyTo(img_);
  image_panel_->setImage(img);
  if(IsActive()){
    Fit();
    Refresh();
  }
}

void ImageViewer::save(const std::string& folder, const std::string& postfix){
  cv::imwrite(folder + "/" + std::string(GetTitle()) + postfix + ".png", img_);
}

void ImageViewer::activate(wxActivateEvent &event){
  image_panel_->updateSize();
  Fit();
  Refresh();
}
