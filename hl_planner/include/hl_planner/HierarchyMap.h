//
// Created by thomas on 31.12.17.
//

#ifndef HL_PLANNER_HIERARCHYMAP_H
#define HL_PLANNER_HIERARCHYMAP_H

#include <vector>
#include <geometry_msgs/Pose.h>
#include <semantic_mapping_v2/HierarchySrvResponse.h>

class HierarchyMap{
  public:
    const float UNEXPLORED_SEARCH_TIME_ESTIMATE = 1000.f;
    const float UNEXPLORED_QUICK_SEARCH_TIME_ESTIMATE = 100.f;
    const float UNEXPLORED_PROB_ESTIMATE = 0.5f;
    const float UNEXPLORED_QUICK_SEARCH_PROB_ESTIMATE = 0.001f;

    std::vector<float> search_times_;
    std::vector<float> expected_search_times_;
    std::vector<float> quick_search_times_;
    std::vector<std::vector<float>> travel_times_;
    std::vector<std::vector<std::vector<int>>> travel_path_;
    std::vector<std::vector<std::vector<geometry_msgs::Pose>>> travel_waypoints_;
    std::vector<bool> not_explored_;

    std::vector<float> search_prob_;
    std::vector<float> quick_search_prob_;

    HierarchyMap(const semantic_mapping_v2::HierarchySrvResponse& res, int obj);

    float getSearchSpeed(int curr_idx, int idx) const { return search_prob_[idx]/(search_times_[idx] + travel_times_[curr_idx][idx]); }
    float getFullSearchTime(int curr_idx, int idx) const { return search_times_[idx] + travel_times_[curr_idx][idx]; }
    float getExpectedSearchTime(int curr_idx, int idx) const { return expected_search_times_[idx] + travel_times_[curr_idx][idx]; }
    float getQuickSearchTime(int curr_idx, int idx) const { return quick_search_times_[idx] + travel_times_[curr_idx][idx]; }
};

#endif //HL_PLANNER_HIERARCHYMAP_H
