//
// Created by thomas on 24.10.17.
//

#include "prob_map_view/ImagePanel.h"
#include <wx/dcbuffer.h>
#include <wx/wx.h>

ImagePanel::ImagePanel(wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size, long style, const wxString& name)
      : wxPanel(parent, id, pos, size, style, name), bitmap_()
{
  SetBackgroundStyle(wxBG_STYLE_PAINT);
  Connect(wxEVT_PAINT, wxPaintEventHandler(ImagePanel::paint), NULL, this);
}

ImagePanel::~ImagePanel(){
  Disconnect(wxEVT_PAINT, wxPaintEventHandler(ImagePanel::paint), NULL, this);
}

void ImagePanel::setImage(const cv::Mat& image){
  SetMinClientSize(wxSize(image.cols, image.rows));
  unsigned char *data = new unsigned char[image.total() * image.channels()];
  memcpy(data, image.data, image.total() * image.channels() * sizeof(unsigned char));
  bitmap_ = wxBitmap(wxImage(image.cols, image.rows, data));
  bitmap_valid_ = true;
  Refresh();
}

void ImagePanel::paint(wxPaintEvent& event){
  wxAutoBufferedPaintDC dc(this);

  dc.SetBrush(*wxBLACK_BRUSH);
  dc.DrawRectangle(wxPoint(0, 0), GetClientSize());
  if(bitmap_valid_)
    dc.DrawBitmap(bitmap_, 0, 0);
}
