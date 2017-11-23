//
// Created by thomas on 23.11.17.
//

#ifndef SEMANTIC_MAPPING_V2_ROOMTYPEMAPPER_H
#define SEMANTIC_MAPPING_V2_ROOMTYPEMAPPER_H

#include "vision/VisionMsg.h"

class RoomTypeMapper{
  public:
    const double MIN_PROB = 0.001;
    const double CONFIDENCE = 0.2;

  private:
    std::vector<double> probs_;
    std::vector<std::string> names_;

    int num_types_ = 0;

  public:
    void processMsg(const vision::VisionMsgConstPtr& msg);

    std::vector<double> getProbs() const { return probs_; }
    std::vector<std::string> getNames() const { return names_; }

    std::string getBestName() const { return names_[std::max_element(probs_.begin(), probs_.end())-probs_.begin()]; }
    double getBestProb() const { return *std::max_element(probs_.begin(), probs_.end()); }
};

#endif //SEMANTIC_MAPPING_V2_ROOMTYPEMAPPER_H
