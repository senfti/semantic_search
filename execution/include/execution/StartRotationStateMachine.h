//
// Created by thomas on 17.12.17.
//

#include <geometry_msgs/Pose.h>

#ifndef EXECUTION_STARTROTATIONSTATEMACHINE_H
#define EXECUTION_STARTROTATIONSTATEMACHINE_H

class StartRotationStateMachine{
  public:
    int state_ = -1;

    const int STEPS = 3;
    const float TURN_SPEED = 2.0;

    bool start();
    bool next(geometry_msgs::Pose& pose);
    void reset() { state_ = -1; }
};

#endif //EXECUTION_STARTROTATIONSTATEMACHINE_H
