//
// Created by thomas on 24.10.17.
//

#ifndef PROB_MAP_VIEW_IMAGEPANEL_H
#define PROB_MAP_VIEW_IMAGEPANEL_H

#include <wx/panel.h>
#include <wx/image.h>
#include <opencv2/opencv.hpp>


class ImagePanel : public wxPanel{
  protected:
    bool bitmap_valid_ = false;
    wxBitmap bitmap_;
    virtual void paint(wxPaintEvent& event);

  public:
    ImagePanel(wxWindow* parent, wxWindowID id = wxID_ANY, const wxPoint& pos = wxDefaultPosition,
               const wxSize& size = wxDefaultSize, long style = wxTAB_TRAVERSAL | wxFULL_REPAINT_ON_RESIZE, const wxString& name = "image_panel");
    ~ImagePanel();

    virtual void setImage(const cv::Mat& image);
};


#endif //PROB_MAP_VIEW_IMAGEPANEL_H
