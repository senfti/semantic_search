//
// Created by thomas on 18.08.17.
//

#include "semantic_mapping/KnowledgeBase.h"

bool operator<(const KbQuery &lhs, const KbQuery &rhs){
  return lhs.t1_.compare(rhs.t1_) < 0 || (lhs.t1_.compare(rhs.t1_) == 0 && lhs.t2_.compare(rhs.t2_) < 0);
}

std::vector<KbQuery> KbQuery::makeQueries(const std::vector<Object> &objs, const std::vector<PlaceGuess> &places){
  std::vector<KbQuery> queries;
  queries.reserve(objs.size() * places.size());
  for(const auto& o : objs){
    for(const auto& p : places){
      queries.push_back(KbQuery(o.class_, p.class_));
    }
  }
  return queries;
}

std::vector<KbQuery> KbQuery::makeQueries(const std::vector<std::string> &obj_names, const std::vector<std::string>& places_names){
  std::vector<KbQuery> queries;
  queries.reserve(obj_names.size() * places_names.size());
  for(const auto& o : obj_names){
    for(const auto& p : places_names){
      queries.push_back(KbQuery(o, p));
    }
  }
  return queries;
}


KnowledgeBase::KnowledgeBase(ros::NodeHandle& nh){
  reason_srv_client_ = nh.serviceClient<kb_query::KbQuerySrv>("kb_query_srv");
}

float KnowledgeBase::getValue(const KbQuery &query){
  auto it = cache_.find(query);
  if(it != cache_.end())
    return it->second;
  else {
    kb_query::KbQuerySrv srv;
    srv.request.terms1.push_back(query.t1_);
    srv.request.terms2.push_back(query.t2_);
    if(!reason_srv_client_.call(srv))
      return -1.f;
    cache_.insert(std::pair<KbQuery, float>(query, srv.response.similarities[0]));
  }
}

std::vector<float> KnowledgeBase::getValues(const std::vector<KbQuery> &queries){
  kb_query::KbQuerySrv srv;
  std::vector<int> missing_idx;
  std::vector<float> res(queries.size());
  for(int i=0; i<queries.size(); i++){
    auto it = cache_.find(queries[i]);
    if(it != cache_.end())
      res[i] = it->second;
    else {
      srv.request.terms1.push_back(queries[i].t1_);
      srv.request.terms2.push_back(queries[i].t2_);
      missing_idx.push_back(i);
    }
  }

  if(!srv.request.terms1.empty()){
//    for(int i=0; i<srv.request.terms1.size(); i++){
//      std::cout << "request: " << srv.request.terms1[i] << " " << srv.request.terms2[i] << std::endl;
//    }
    if(!reason_srv_client_.call(srv))
      return std::vector<float>();
  }

  for(int i=0; i<missing_idx.size(); i++){
    res[missing_idx[i]] = srv.response.similarities[i];
    cache_.insert(std::pair<KbQuery, float>(queries[missing_idx[i]], srv.response.similarities[i]));
  }

  return res;
}