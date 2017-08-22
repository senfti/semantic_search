//
// Created by thomas on 18.08.17.
//

#include <ros/ros.h>
#include <kb_query/KbQuerySrv.h>
#include <map>
#include "VisionResult.h"

#ifndef SEMANTIC_MAPPING_KNOWLEDGEBASE_H
#define SEMANTIC_MAPPING_KNOWLEDGEBASE_H

class KbQuery{
  public:
    std::string t1_, t2_;

    KbQuery(std::string t1, std::string t2) : t1_(t1), t2_(t2) {}

    static std::vector<KbQuery> makeQueries(const std::vector<Object>& objs, const std::vector<PlaceGuess>& places);
    static std::vector<KbQuery> makeQueries(const std::vector<std::string>& obj_names, const std::vector<std::string>& places_names);
};

bool operator<(const KbQuery& lhs, const KbQuery& rhs);

class KnowledgeBase{
  private:
    ros::ServiceClient reason_srv_client_;
    std::map<KbQuery, float> cache_;

  public:
    KnowledgeBase(ros::NodeHandle& nh);

    float getValue(const KbQuery& query);
    std::vector<float> getValues(const std::vector<KbQuery>& queries);
};

#endif //SEMANTIC_MAPPING_KNOWLEDGEBASE_H
