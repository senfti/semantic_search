//
// Created by thomas on 17.12.17.
//

#include <cstdlib>
#include <execution/StartRotationStateMachine.h>

bool StartRotationStateMachine::start(){
  if(state_ < 0){
    std::system(("rosrun dynamic_reconfigure dynparam set /move_base/DWAPlannerROS max_rot_vel " + std::to_string(TURN_SPEED)).c_str());
    state_ = 0;
    return true;
  }
  return false;
}

bool StartRotationStateMachine::next(geometry_msgs::Pose &pose){
  if(state_ < 0)
    start();

  if(state_ == STEPS){
    state_ = -1;
    return false;
  }
  else{
    state_++;
    pose.position.x = 0.0;
    pose.position.y = 0.0;
    pose.position.z = 0.0;
    pose.orientation.w = std::cos(state_*M_PI/STEPS);
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = std::sin(state_*M_PI/STEPS);
    return true;
  }
}
