//
// Created by thomas on 28.11.17.
//

#include <wx/wx.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

class MyWindow : public wxFrame
{
  private:
    void publish(wxTimerEvent &event);
    void mouseEvent(wxMouseEvent &event);
    void stop(wxMouseEvent &event);
    ros::NodeHandle node_handle_;
    ros::Publisher vel_pub_;
    wxTimer timer_;

    bool l=false, m=false, r=false;

    bool stopped_ = false;
    geometry_msgs::Twist msg;

    const float SPEED = 0.25;
    const float ANG_SPEED = 1.0;

  public:
    MyWindow( wxWindow* parent, wxWindowID id = wxID_ANY, const wxString& title = wxT("ProbViewer"), const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 800,800 ), long style = wxDEFAULT_FRAME_STYLE|wxTAB_TRAVERSAL );
};

class MyApp : public wxApp{
  private:
    MyWindow* window_;

  public:
    MyApp();

    bool OnInit();
};


IMPLEMENT_APP(MyApp)

MyApp::MyApp(){
}

bool MyApp::OnInit(){
  ros::init(argc, argv, "mouse_control");

  window_ = new MyWindow(nullptr);
  window_->Show();
}

MyWindow::MyWindow(wxWindow *parent, wxWindowID id, const wxString &title, const wxPoint &pos, const wxSize &size, long style)
      : wxFrame( parent, id, title, pos, size, style ), timer_(this)
{
  vel_pub_ = node_handle_.advertise<geometry_msgs::Twist>("/mouse_control/cmd_vel", 1, true);
  Connect(timer_.GetId(), wxEVT_TIMER, wxTimerEventHandler(MyWindow::publish), NULL, this);

  Connect(wxEVT_LEFT_DOWN, wxMouseEventHandler(MyWindow::mouseEvent), NULL, this);
  Connect(wxEVT_RIGHT_DOWN, wxMouseEventHandler(MyWindow::mouseEvent), NULL, this);
  Connect(wxEVT_MIDDLE_DOWN, wxMouseEventHandler(MyWindow::mouseEvent), NULL, this);
  Connect(wxEVT_MOTION, wxMouseEventHandler(MyWindow::stop), NULL, this);

  timer_.Start(10);
}

void MyWindow::publish(wxTimerEvent &event){
  vel_pub_.publish(msg);
}

void MyWindow::mouseEvent(wxMouseEvent &event){
  if(stopped_){
    stopped_ = false;
    return;
  }

  if(event.m_leftDown){
    if(r)
      r = false;
    else
      l = !l;
  }
  if(event.m_middleDown)
    m = !m;
  if(event.m_rightDown){
    if(l)
      l = false;
    else
      r = !r;
  }

  if(l){
    if(m){
      msg.linear.x = SPEED;
      msg.angular.z = ANG_SPEED / 2.0;
    }
    else{
      msg.linear.x = 0.f;
      msg.angular.z = ANG_SPEED;
    }
  }
  else if(r){
    if(m){
      msg.linear.x = SPEED;
      msg.angular.z = -ANG_SPEED / 2.0;
    }
    else{
      msg.linear.x = 0.f;
      msg.angular.z = -ANG_SPEED;
    }
  }
  else if(m){
    msg.linear.x = SPEED;
    msg.angular.z = 0.f;
  }
  else{
    msg.linear.x = 0.f;
    msg.angular.z = 0.f;
  }
  std::cout << l << " " << m << " " << r << std::endl;
}

void MyWindow::stop(wxMouseEvent &event){
  l = false;
  m = false;
  r = false;
  stopped_ = true;
  msg.angular.z = 0.f;
  msg.linear.x = 0.f;
  std::cout << l << " " << m << " " << r << std::endl;
  if(event.m_x < 375 || event.m_x > 425 || event.m_y < 375 || event.m_y > 425)
    WarpPointer(400,400);
}

