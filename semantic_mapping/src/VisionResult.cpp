//
// Created by thomas on 17.08.17.
//

#include "semantic_mapping/VisionResult.h"
#include "semantic_mapping/KnowledgeBase.h"

const double ASUS_FOV = 57.0*M_PI/180.0;
const double ASUS_FOV_COS = std::cos(ASUS_FOV/2.0);

Object::Object(const vision::ObjectDetectionMsg &msg)
  : x1_(msg.x1), x2_(msg.x2), y1_(msg.y1), y2_(msg.y2), z1_(msg.z1), z2_(msg.z2), prob_(msg.prob), class_(msg.name), id_(msg.id)
{
}

PlaceGuess::PlaceGuess(const vision::PlaceRecognitionMsg &msg)
  : prob_(msg.prob), class_(msg.name), id_(msg.id)
{
}

VisionResult::VisionResult(const vision::VisionMsg &msg)
  : time_(msg.pose.header.stamp)
{
  tf::poseMsgToTF(msg.pose.pose, pose_);
  objects_.reserve(msg.objects.size());
  for(const auto& o : msg.objects)
    objects_.push_back(Object(o));
  place_guesses_.reserve(msg.place_guesses.size());
  for(const auto& p : msg.place_guesses)
    place_guesses_.push_back(PlaceGuess(p));

  max_dists_ = msg.view_dists;
}

//double VisionResult::getSumPlaceProbs() const{
//  double sum = 0.0;
//  for(const auto& p : place_guesses_)
//    sum += p.prob_;
//  return sum;
//}
//
//double VisionResult::getSumObjectProbs() const{
//  double sum = 0.0;
//  for(const auto& o : objects_)
//    sum += o.prob_;
//  return sum;
//}
//
//void VisionResult::improve(KnowledgeBase &kb){
//  int numObj = objects_.size();
//  int numPlaces = place_guesses_.size();
//  if(numObj == 0 || numPlaces == 0)
//    return;
//
//  std::vector<float> similarity = kb.getValues(KbQuery::makeQueries(objects_, place_guesses_));
//  if(similarity.empty())
//    return;
//
//  double sum_object_probs = getSumObjectProbs();
//  double sum_place_probs = getSumPlaceProbs();
//  std::vector<double> prob_updates(place_guesses_.size(), 1.0);
//  // calculate prob of place under objects (without prior)
//  for(int o=0; o<objects_.size(); o++){
//    for(int p=0; p<place_guesses_.size(); p++){
//      prob_updates[p] += objects_[o].prob_*similarity[o*place_guesses_.size()+p];
//    }
//  }
//  // normalize prob of place under objects (without prior)
//  for(int p=0; p<place_guesses_.size(); p++)
//    prob_updates[p] /= sum_object_probs;
//
//  // apply prob of place under objects to prior
//  double prob_sum = 0.0;
//  for(int p=0; p<place_guesses_.size(); p++){
//    place_guesses_[p].prob_ *= prob_updates[p];
//    prob_sum += place_guesses_[p].prob_;
//  }
//  // normalize (overall probability has to be the same as before)
//  for(int p=0; p<place_guesses_.size(); p++)
//    place_guesses_[p].prob_ *= sum_place_probs / prob_sum;
//
//  std::sort(place_guesses_.begin(), place_guesses_.end(), operator>);
//}
//
//double VisionResult::getImportance(double x, double y) const{
//  double diff_x = x-pose_.getOrigin().getX();
//  double diff_y = y-pose_.getOrigin().getY();
//  double dist = std::sqrt(diff_x*diff_x + diff_y*diff_y);
//  if(dist > MAX_DIST_BOUND)
//    return 0.0;
//  double angle = atan2(diff_y,diff_x) - atan2(pose_.getBasis().getColumn(0).getY(),pose_.getBasis().getColumn(0).getX());
//
//  int segment = (angle + ASUS_FOV / 2.0) / ASUS_FOV * max_dists_.size();
//  if(segment < 0 || segment >= max_dists_.size() || dist > max_dists_[segment])
//    return -1.0;
//
//  return 1.0;
//
//  /*double dist_normalized = (std::sqrt(dist) - (MAX_DIST_BOUND + MIN_DIST_BOUND) / 2.0) / ((MAX_DIST_BOUND - MIN_DIST_BOUND) / 2.0);
//  return (cos_angle - ASUS_FOV_COS*0.5)/(1.0-ASUS_FOV_COS*0.5)  // angular weight
//      * (1.0 - dist_normalized*dist_normalized);                // distance weight*/
//
//}

std::ostream& operator<<(std::ostream& os, const VisionResult& vision_result){
  os << std::setprecision(3) << vision_result.time_ << ": "
     << vision_result.pose_.getOrigin().getX() << ", " << vision_result.pose_.getOrigin().getY() << "; "
     << tf::getYaw(vision_result.pose_.getRotation())*180.0/M_PI << std::endl;
  for(const auto& p : vision_result.place_guesses_)
    os << p.class_ << " " << std::to_string(p.prob_).substr(0, 5) << "; \t";
  os << std::endl;
  for(const auto& o : vision_result.objects_)
    os << o.class_ << " " << std::to_string(*std::max_element(o.prob_.begin(), o.prob_.end())).substr(0, 5) << "; \t";
  os << std::endl;
  os << std::endl;

  return os;
}

bool operator>(const PlaceGuess& lhs, const PlaceGuess& rhs){
  return lhs.prob_ > rhs.prob_;
}
