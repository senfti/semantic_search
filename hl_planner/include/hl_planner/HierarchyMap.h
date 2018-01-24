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
    static float UNEXPLORED_SEARCH_TIME_ESTIMATE;
    static float UNEXPLORED_QUICK_SEARCH_TIME_ESTIMATE;
    static float UNEXPLORED_PROB_ESTIMATE;
    static float UNEXPLORED_QUICK_SEARCH_PROB_ESTIMATE;

    int searched_obj_ = -1;

    std::vector<float> search_times_;
    std::vector<float> expected_search_times_;
    std::vector<float> quick_search_times_;
    std::vector<geometry_msgs::Pose> quick_search_poses_;
    std::vector<std::vector<float>> travel_times_;
    std::vector<std::vector<float>> search_speeds_;
    std::vector<std::vector<std::vector<int>>> travel_path_;
    std::vector<std::vector<std::vector<geometry_msgs::Pose>>> travel_waypoints_;
    std::vector<bool> not_visited_;

    std::vector<float> search_prob_;
    std::vector<float> quick_search_prob_;

    HierarchyMap(const semantic_mapping_v2::HierarchySrvResponse& res, int obj);

    float getFullSearchTime(int curr_idx, int idx) const { return search_times_[idx] + travel_times_[curr_idx][idx]; }
    float getExpectedSearchTime(int curr_idx, int idx) const { return expected_search_times_[idx] + travel_times_[curr_idx][idx]; }
    float getQuickSearchTime(int curr_idx, int idx) const { return quick_search_times_[idx] + travel_times_[curr_idx][idx]; }

    float getSearchSpeed(int curr_idx, int idx) const { return search_speeds_[curr_idx][idx]; }

    friend std::ostream& operator<<(std::ostream& os, const HierarchyMap& map);
};

std::ostream& operator<<(std::ostream& os, const HierarchyMap& map);

#endif //HL_PLANNER_HIERARCHYMAP_H
