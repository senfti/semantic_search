//
// Created by thomas on 17.08.17.
//

#ifndef SEMANTIC_MAPPING_VISIONRESULT_H
#define SEMANTIC_MAPPING_VISIONRESULT_H

#include "vision/VisionMsg.h"
#include <tf/tf.h>
#include <iostream>

class KnowledgeBase;

class Object{
  public:
    Object(const vision::ObjectDetectionMsg& msg);

    float x1_, x2_, y1_, y2_, z_;
    float prob_ = 0.f;
    int id_;
    std::string class_;
};

class PlaceGuess{
  public:
    PlaceGuess(const vision::PlaceRecognitionMsg& msg);

    float prob_ = 0.f;
    std::string class_;
    int id_;

    friend bool operator>(const PlaceGuess& lhs, const PlaceGuess& rhs);
};

class VisionResult{
  public:
    const double MIN_DIST_BOUND = 0.5;
    const double MAX_DIST_BOUND = 5.0;

    VisionResult(const vision::VisionMsg& msg);

    ros::Time time_;
    tf::Pose pose_;
    std::vector<Object> objects_;
    std::vector<PlaceGuess> place_guesses_;

    double getSumPlaceProbs() const;
    double getSumObjectProbs() const;
    void improve(KnowledgeBase& kb);
    double getImportance(double x, double y) const;

    friend std::ostream& operator<<(std::ostream& os, const VisionResult& vision_result);
};

std::ostream& operator<<(std::ostream& os, const VisionResult& vision_result);
bool operator>(const PlaceGuess& lhs, const PlaceGuess& rhs);

#endif //SEMANTIC_MAPPING_VISIONRESULT_H
