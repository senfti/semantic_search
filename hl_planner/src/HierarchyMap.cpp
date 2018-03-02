//
// Created by thomas on 31.12.17.
//

#include <hl_planner/HierarchyMap.h>
#include <tf/transform_datatypes.h>
#include <limits>
#include <boost/math/special_functions/gamma.hpp>

float HierarchyMap::UNEXPLORED_SEARCH_TIME_ESTIMATE = 1000.f;
float HierarchyMap::UNEXPLORED_QUICK_SEARCH_TIME_ESTIMATE = 100.f;
float HierarchyMap::UNEXPLORED_PROB_ESTIMATE = 0.5f;
float HierarchyMap::UNEXPLORED_QUICK_SEARCH_PROB_ESTIMATE = 0.001f;

// based on wikipedia pseudocode: https://en.wikipedia.org/wiki/Floyd%E2%80%93Warshall_algorithm
void floydWarshall(const std::vector<std::vector<float>>& edges, std::vector<std::vector<float>>& dist, std::vector<std::vector<std::vector<int>>>& path){
  int num_nodes = edges.size();
  std::vector<std::vector<float>> next(num_nodes);
  for(auto& n : next)
    n.resize(num_nodes, 0.f);
  dist = edges;
  path.resize(num_nodes);
  for(auto& p : path)
    p.resize(num_nodes);

  for(int u=0; u<num_nodes; u++){
    for(int v=0; v<num_nodes; v++){
      dist[u][v] = edges[u][v];
      next[u][v] = v;
    }
  }

  for(int k=0; k<num_nodes; k++){
    for(int i=0; i<num_nodes; i++){
      for(int j=0; j<num_nodes; j++){
        if(dist[i][j] > dist[i][k] + dist[k][j]){
          dist[i][j] = dist[i][k] + dist[k][j];
          next[i][j] = next[i][k];
        }
      }
    }
  }

  for(int u=0; u<num_nodes; u++){
    for(int v=0; v<num_nodes; v++){
      int n = u;
      path[u][v].push_back(u);
      while(n != v){
        n = next[n][v];
        path[u][v].push_back(n);
      }
    }
  }
}


HierarchyMap::HierarchyMap(const semantic_mapping_v2::HierarchySrvResponse& res, int obj){
  semantic_mapping_v2::HierarchySrvResponse response = res;
  searched_obj_ = obj;
  search_times_.resize(response.rooms.size());
  expected_search_times_.resize(response.rooms.size());
  search_prob_.resize(response.rooms.size());
  not_visited_.resize(response.rooms.size(), false);

  std::cout << res.curr_room << " " << res.rooms.size() << " " << res.links.size() << std::endl;

  for(int i=0; i<response.rooms.size(); i++){
    search_times_[i] = response.rooms[i].search_time;
    search_prob_[i] = response.rooms[i].obj_probs[obj];
    expected_search_times_[i] = response.rooms[i].expected_search_time[obj];
  }

  int room_idx = search_times_.size();
  for(int i=0; i<response.links.size(); i++){
    if(response.links[i].room2 < 0 || response.links[i].room2 >= room_idx){
      search_times_.push_back(response.unknown_room.search_time);
      search_prob_.push_back(response.unknown_room.obj_probs[obj]);
      expected_search_times_.push_back(response.unknown_room.expected_search_time[obj]);
      not_visited_.push_back(true);
      response.rooms.push_back(response.unknown_room);
      response.rooms.back().links.push_back(i);
      response.rooms.back().id = room_idx;
      response.links[i].room2 = room_idx++;
    }
  }

  std::vector<std::vector<float>> edges;
  edges.resize(response.links.size());
  for(int i=0; i<response.links.size(); i++){
    edges[i] = response.links[i].dists;
  }
  std::vector<std::vector<float>> link_travel_times;
  std::vector<std::vector<std::vector<int>>> link_travel_path;
  floydWarshall(edges, link_travel_times, link_travel_path);

  travel_times_.resize(response.rooms.size(), std::vector<float>(response.rooms.size(), std::numeric_limits<float>::max()));
  search_speeds_.resize(response.rooms.size(), std::vector<float>(response.rooms.size(), 0.f));
  travel_path_.resize(response.rooms.size(), std::vector<std::vector<int>>(response.rooms.size()));
  travel_waypoints_.resize(response.rooms.size(), std::vector<std::vector<geometry_msgs::Pose>>(response.rooms.size()));
  for(int r1=0; r1<response.rooms.size(); r1++){
    for(int r2=0; r2<response.rooms.size(); r2++){
      int best_link = 0;
      std::vector<int> link_path;
      if(r1!=r2){
        for(int l1=0; l1<response.rooms[r1].links.size(); l1++){
          int link1 = response.rooms[r1].links[l1];
          for(int l2=0; l2<response.rooms[r2].links.size(); l2++){
            int link2 = response.rooms[r2].links[l2];
            if(link_travel_times[link1][link2] < travel_times_[r1][r2]){
              travel_times_[r1][r2] = link_travel_times[link1][link2];
              link_path = link_travel_path[link1][link2];
              best_link = l1;
            }
          }
        }
        travel_times_[r1][r2] += response.rooms[r1].to_link_travel_times[best_link];
      }
      else
        travel_times_[r1][r2] = 0.f;

      travel_path_[r1][r2].push_back(r1);
      travel_waypoints_[r1][r2].push_back(geometry_msgs::Pose());
      int curr_room = r1;
      for(int i=0; i<link_path.size(); i++){
        auto& link = response.links[link_path[i]];
        if(link.room1 == curr_room){
          travel_path_[r1][r2].push_back(link.room2);
          curr_room = link.room2;
          geometry_msgs::Pose waypoint = link.door1_pose;
//          tf::Transform tmp;
//          tf::poseMsgToTF(waypoint, tmp);
//          waypoint.position.x += 0.5*tmp.getBasis().getColumn(0).x();
//          waypoint.position.y += 0.5*tmp.getBasis().getColumn(0).y();
          travel_waypoints_[r1][r2].push_back(waypoint);
        }
        else{
          travel_path_[r1][r2].push_back(link.room1);
          curr_room = link.room1;
          geometry_msgs::Pose waypoint = link.door2_pose;
//          tf::Transform tmp;
//          tf::poseMsgToTF(waypoint, tmp);
//          waypoint.position.x += 0.5*tmp.getBasis().getColumn(0).x();
//          waypoint.position.y += 0.5*tmp.getBasis().getColumn(0).y();
          travel_waypoints_[r1][r2].push_back(waypoint);
        }
      }
      search_speeds_[r1][r2] = search_prob_[r2]/(search_times_[r2] + travel_times_[r1][r2]);
    }
  }
}

float func(float Tq, float Tf, float Pf, float b){
  float Tqb = std::pow(Tq,b);
  float Tfb = std::pow(Tf,b);
  return Tqb*Pf/(Tqb*Pf-Tfb*Pf+Tfb);
}

double fitBeta(float quick_search_time, float search_time, float quick_search_prob, float search_prob){
  double v1;
  double v2;
  double b1 = 0.000001;
  double b2 = 0.1;
  v1 = func(quick_search_time, search_time, search_prob, b1) - quick_search_prob;
  while(b2 < 999999.9){
    v2 = func(quick_search_time, search_time, search_prob, b2) - quick_search_prob;
    if(v1 * v2 <= 0.f)
      break;
    b1 = b2;
    b2 *= 1.5;
    v1 = v2;
  }
  if(v2 == 0.0)
    return b2;
  if(b2 >= 999999.9)
    return -1.0;
  double v_tmp;
  int count = 0;
  do{
    double tmp = b1 - (b2 - b1) / (v2 - v1) * v1;
    v_tmp = func(quick_search_time, search_time, search_prob, tmp) - quick_search_prob;
    if(v1 * v_tmp < 0){
      b2 = tmp;
      v2 = v_tmp;
    }
    else{
      b1 = tmp;
      v1 = v_tmp;
    }
  } while(std::abs(v_tmp) > 0.0000001f && std::abs(b1 - b2) > 0.0000000000001f && count < 100000);
  if(count >= 100000)
    return -1.0;
  return (std::abs(v2)/(std::abs(v2)+std::abs(v1)))*b1 + (std::abs(v1)/(std::abs(v1)+std::abs(v2)))*b2;
}


float HierarchyMap::calcExpectedSearchTime(float quick_search_time, float search_time, float quick_search_prob, float search_prob){
  double beta = fitBeta(quick_search_time, search_time, quick_search_prob, search_prob);
  double expected_search_time = 0.0;
  if(beta < 0.0){
    return search_prob*search_time/2 + (1-search_prob)*search_time;
  }
  double alpha_pow_beta = (1.0-search_prob)/search_prob*std::pow(search_time,beta);

  double step = 0.01;
  for(double t=step/2; t<search_time; t+=step){
    double gamma = beta*std::pow((t), beta-1.0)/alpha_pow_beta/std::pow(1.0+std::pow(t,beta)/alpha_pow_beta, 2);
    expected_search_time += gamma*t*step;
  }

  return expected_search_time + (1-search_prob)*search_time;
}


std::ostream& operator<<(std::ostream& os, const HierarchyMap& map){
  for(int i=0; i<map.search_times_.size(); i++){
    os << i << ":" << std::endl;
    os << "expected: " << map.expected_search_times_[i] << ",   full: " << map.search_times_[i] << std::endl;
    os << "probs: " << map.search_prob_[i] << std::endl;
    os << "travel: ";
    for(int j=0; j<map.travel_times_[i].size(); j++){
      os << j << ": ";
      for(int k=0; k<map.travel_path_[i][j].size(); k++)
        os << map.travel_path_[i][j][k] << " ";
      os << ", " << map.travel_times_[i][j] << "; ";
    }
    os << (map.not_visited_[i] ? "not visited" : "visited") << std::endl << std::endl;
  }
  os << std::endl;

  return os;
}
