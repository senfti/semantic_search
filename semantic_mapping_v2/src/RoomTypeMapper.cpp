//
// Created by thomas on 23.11.17.
//

#include "semantic_mapping_v2/RoomTypeMapper.h"

template <typename T>
std::vector<size_t> ordered(std::vector<T> const& values) {
  std::vector<size_t> indices(values.size());
  std::iota(begin(indices), end(indices), static_cast<size_t>(0));

  std::sort(
        begin(indices), end(indices),
        [&](size_t a, size_t b) { return values[a] < values[b]; }
  );
  return indices;
}

void RoomTypeMapper::processMsg(const vision::VisionMsgConstPtr& msg){
  if(num_types_ == 0){
    num_types_ = msg->place_guesses.size();
    probs_.resize(num_types_, 1.0 / num_types_);
    names_.resize(num_types_);
    for(int i=0; i<num_types_; i++){
      names_[i] = msg->place_guesses[i].name;
    }
  }

  double sum = 0.0;
  for(int i=0; i<num_types_; i++){
    probs_[i] = msg->place_guesses[i].prob*CONFIDENCE + probs_[i]*(1.0-CONFIDENCE);
    sum += probs_[i];
  }
  std::vector<int> not_low_idx(num_types_);
  for(int i=0; i<num_types_; i++){
    probs_[i] /= sum;
    not_low_idx[i] = i;
  }

  while(true){
    std::vector<int> old_not_low_idx = not_low_idx;
    not_low_idx.clear();
    bool did_thresh = false;
    for(int i=0; i<old_not_low_idx.size(); i++){
      if(probs_[old_not_low_idx[i]] <= MIN_PROB){
        probs_[old_not_low_idx[i]] = MIN_PROB;
        did_thresh = true;
      }
      else
        not_low_idx.push_back(old_not_low_idx[i]);
    }
    if(!did_thresh)
      break;

    double rest_sum = 1.0-(num_types_-not_low_idx.size())*MIN_PROB;
    sum = 0.0;
    for(int i=0; i<not_low_idx.size(); i++){
      sum += probs_[not_low_idx[i]];
    }
    for(int i=0; i<not_low_idx.size(); i++){
      probs_[not_low_idx[i]] *= rest_sum/sum;
    }
  }
  std::vector<size_t> indices_sorted = ordered(probs_);
  for(int i=indices_sorted.size()-1; i>indices_sorted.size()-11; i--){
    std::cout << names_[indices_sorted[i]] << ": " << probs_[indices_sorted[i]] << std::endl;
  }
}

