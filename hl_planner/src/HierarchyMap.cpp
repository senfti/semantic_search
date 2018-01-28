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
  quick_search_times_.resize(response.rooms.size());
  expected_search_times_.resize(response.rooms.size());
  search_prob_.resize(response.rooms.size());
  quick_search_prob_.resize(response.rooms.size());
  not_visited_.resize(response.rooms.size(), false);
  quick_search_poses_.resize(response.rooms.size());

  for(int i=0; i<response.rooms.size(); i++){
    search_times_[i] = response.rooms[i].search_time;
    quick_search_times_[i] = response.rooms[i].single_view_search_time;
    quick_search_poses_[i].position = response.rooms[i].single_view_points[obj];
    quick_search_poses_[i].orientation.w = 1.0;
    search_prob_[i] = response.rooms[i].obj_probs[obj];
    quick_search_prob_[i] = response.rooms[i].single_view_obj_probs[obj];
//    expected_search_times_[i] = quick_search_times_[i]*quick_search_prob_[i]
//                                + (search_prob_[i]-quick_search_prob_[i])/(search_times_[i]-quick_search_times_[i])
//                                  * (search_times_[i]*search_times_[i]-quick_search_times_[i]*quick_search_times_[i])/2.f
//                                + (1.f-search_prob_[i])*search_times_[i];
    expected_search_times_[i] = calcExpectedSearchTime(quick_search_times_[i], search_times_[i], quick_search_prob_[i], search_prob_[i]);
  }

  int room_idx = search_times_.size();
  for(int i=0; i<response.links.size(); i++){
    if(response.links[i].room2 < 0 || response.links[i].room2 >= room_idx){
      search_times_.push_back(UNEXPLORED_SEARCH_TIME_ESTIMATE);
      quick_search_times_.push_back(UNEXPLORED_QUICK_SEARCH_TIME_ESTIMATE);
      quick_search_poses_.push_back(geometry_msgs::Pose());
      quick_search_poses_.back().orientation.w = 1.0;
      search_prob_.push_back(UNEXPLORED_PROB_ESTIMATE);
      quick_search_prob_.push_back(UNEXPLORED_QUICK_SEARCH_PROB_ESTIMATE);
//      expected_search_times_[room_idx] = quick_search_times_[room_idx]*quick_search_prob_[room_idx]
//                                  + (search_prob_[room_idx]-quick_search_prob_[room_idx])/(search_times_[room_idx]-quick_search_times_[room_idx])
//                                    * (search_times_[room_idx]*search_times_[room_idx]-quick_search_times_[room_idx]*quick_search_times_[room_idx])/2.f
//                                  + (1.f-search_prob_[room_idx])*search_times_[room_idx];
      expected_search_times_[room_idx] = calcExpectedSearchTime(quick_search_times_[room_idx], search_times_[room_idx], quick_search_prob_[room_idx], search_prob_[room_idx]);
      not_visited_.push_back(true);
      semantic_mapping_v2::RoomMsg unknown_room;
      unknown_room.links.push_back(i);
      unknown_room.to_link_travel_times.push_back(0.f);
      response.rooms.push_back(unknown_room);
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


float fitTheta(float k, float quick_search_time, float search_time, float quick_search_prob, float search_prob){
  double v1 = 0.000001;
  double v2 = 0.1;
  double t1 = 0.000001;
  double t2 = 0.1;
  v1 = boost::math::gamma_p(k, search_time / t1) - search_prob;
  while(true){
    v2 = boost::math::gamma_p(k, search_time / t2) - search_prob;
    if(v1 * v2 <= 0.f)
      break;
    t1 = t2;
    t2 *= 1.5;
    v1 = v2;
  }
  if(v2 == 0.0)
    return t2;
  double v_tmp;
  do{
    double tmp = t1 - (t2 - t1) / (v2 - v1) * v1;
    v_tmp = boost::math::gamma_p(k, search_time / tmp) - search_prob;
    if(v1 * v_tmp < 0){
      t2 = tmp;
      v2 = v_tmp;
    }
    else{
      t1 = tmp;
      v1 = v_tmp;
    }
  } while(std::abs(v_tmp) > 0.001f && std::abs(t1 - t2) > 0.0000000000001f);
  if(std::abs(v1) < std::abs(v2) / 2.0)
    return t1;
  else if(std::abs(v2) < std::abs(v1) / 2.0)
    return t2;
  else
    return (t1 + t2) / 2;
}


float fitSigma(float mu, float quick_search_time, float search_time, float quick_search_prob, float search_prob){
  double v1 = 0.000001;
  double v2 = 0.1;
  double t1 = 0.000001;
  double t2 = 0.1;
  v1 = 0.5*(1.0+std::erf((std::log(search_time)-mu)/(M_SQRT2*t1))) - search_prob;
  while(true){
    v2 = 0.5*(1.0+std::erf((std::log(search_time)-mu)/(M_SQRT2*t2))) - search_prob;
    if(v1 * v2 <= 0.f)
      break;
    t1 = t2;
    t2 *= 1.5;
    v1 = v2;
  }
  if(v2 == 0.0)
    return t2;
  double v_tmp;
  do{
    double tmp = t1 - (t2 - t1) / (v2 - v1) * v1;
    v_tmp = 0.5*(1.0+std::erf((std::log(search_time)-mu)/(M_SQRT2*tmp))) - search_prob;
    if(v1 * v_tmp < 0){
      t2 = tmp;
      v2 = v_tmp;
    }
    else{
      t1 = tmp;
      v1 = v_tmp;
    }
  } while(std::abs(v_tmp) > 0.001f && std::abs(t1 - t2) > 0.0000000000001f);

  return (std::abs(v2)/(std::abs(v2)+std::abs(v1)))*t1 + (std::abs(v1)/(std::abs(v1)+std::abs(v2)))*t2;
}


float func(float Tq, float Tf, float Pf, float b){
  float Tqb = std::pow(Tq,b);
  float Tfb = std::pow(Tf,b);
  return Tqb*Pf/(Tqb*Pf-Tfb*Pf+Tfb);
}

float fitBeta(float quick_search_time, float search_time, float quick_search_prob, float search_prob){
  double v1;
  double v2;
  double b1 = 0.000001;
  double b2 = 0.1;
  v1 = func(quick_search_time, search_time, search_prob, b1) - quick_search_prob;
  while(true){
    v2 = func(quick_search_time, search_time, search_prob, b2) - quick_search_prob;
    if(v1 * v2 <= 0.f)
      break;
    b1 = b2;
    b2 *= 1.5;
    v1 = v2;
  }
  if(v2 == 0.0)
    return b2;
  double v_tmp;
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
  } while(std::abs(v_tmp) > 0.0000001f && std::abs(b1 - b2) > 0.0000000000001f);
  return (std::abs(v2)/(std::abs(v2)+std::abs(v1)))*b1 + (std::abs(v1)/(std::abs(v1)+std::abs(v2)))*b2;
}


float HierarchyMap::calcExpectedSearchTime(float quick_search_time, float search_time, float quick_search_prob, float search_prob){
  double beta = fitBeta(quick_search_time, search_time, quick_search_prob, search_prob);
  double alpha_pow_beta = (1.0-search_prob)/search_prob*std::pow(search_time,beta);

  double expected_search_time = 0.0;
  double step = 0.01;
  for(double t=step/2; t<search_time; t+=step){
    double gamma = beta*std::pow((t), beta-1.0)/alpha_pow_beta/std::pow(1.0+std::pow(t,beta)/alpha_pow_beta, 2);
    expected_search_time += gamma*t*step;
  }

  return expected_search_time + (1-search_prob)*search_time;

//  double mu = quick_search_time;
//  double sigma;
//  double sigma_old = 0.1;
//  double diff_old = 0.0;
//  while(true){
//    sigma = fitSigma(mu, quick_search_time, search_time, quick_search_prob, search_prob);
//    double diff = 0.5*(1.0+std::erf((std::log(quick_search_time)-mu)/(M_SQRT2*sigma))) - quick_search_prob;
//    if(diff_old*diff < 0.0 || std::abs(diff-diff_old) < 0.00001){
//      sigma = (std::abs(diff)/(std::abs(diff)+std::abs(diff_old)))*sigma_old + (std::abs(diff_old)/(std::abs(diff)+std::abs(diff_old)))*sigma;
//      break;
//    }
//    diff_old = diff;
//    sigma_old = sigma;
//    if(diff > 0)
//      mu *= 1.01;
//    else
//      mu *= 0.99;
//  }
//  double expected_search_time = 0.0;
//  double full_prob = 0.0;
//  for(double t=0.5; t<quick_search_time; t+=1.0){
//    double gamma = 1.0/(t*sigma*std::sqrt(2*M_PI))*std::exp(-std::pow((std::log(t)-mu),2)/(2*sigma*sigma));
//    full_prob += gamma;
//  }
//  std::cout << "q " << quick_search_prob << " " << full_prob << std::endl;
//  full_prob = 0.0;
//  for(double t=0.5; t<search_time; t+=1.0){
//    double gamma = 1.0/(t*sigma*std::sqrt(2*M_PI))*std::exp(-std::pow((std::log(t)-mu),2)/(2*sigma*sigma));
//    expected_search_time += gamma*t;
//    full_prob += gamma;
//  }
//  std::cout << "f " << search_prob << " " << full_prob << std::endl;
//  std::cout << mu << " " << sigma << std::endl;
//
//  return expected_search_time + (1-search_prob)*search_time;

//  double k = 1.1;
//  double theta;
//  double theta_old = 0.1;
//  double diff_old = 0.0;
//  while(true){
//    theta = fitTheta(k, quick_search_time, search_time, quick_search_prob, search_prob);
//    double diff = boost::math::gamma_p(k, quick_search_time / theta) - quick_search_prob;
//    if(diff_old*diff < 0.0 || std::abs(diff-diff_old) < 0.00001){
//      theta = (std::abs(diff)/(std::abs(diff)+std::abs(diff_old)))*theta_old + (std::abs(diff_old)/(std::abs(diff)+std::abs(diff_old)))*theta;
//      break;
//    }
//    diff_old = diff;
//    theta_old = theta;
//    if(diff > 0)
//      k *= 1.01;
//    else
//      k *= 0.99;
//  }
//  double expected_search_time = 0.0;
//  double full_prob = 0.0;
//  for(double t=0.5; t<quick_search_time; t+=1.0){
//    double gamma = boost::math::gamma_p_derivative(k, t / theta) / theta;
//    full_prob += gamma;
//  }
//  std::cout << "q " << quick_search_prob << " " << full_prob << std::endl;
//  full_prob = 0.0;
//  for(double t=0.5; t<search_time; t+=1.0){
//    double gamma = boost::math::gamma_p_derivative(k, t / theta) / theta;
//    expected_search_time += gamma*t;
//    full_prob += gamma;
//  }
//  std::cout << "f " << search_prob << " " << full_prob << std::endl;
//
//  return expected_search_time + (1-search_prob)*search_time;
}


std::ostream& operator<<(std::ostream& os, const HierarchyMap& map){
  for(int i=0; i<map.search_times_.size(); i++){
    os << i << ":" << std::endl;
    os << "times: quick: " << map.quick_search_times_[i] << ",   expected: " << map.expected_search_times_[i] << ",   full: " << map.search_times_[i] << std::endl;
    os << "probs: quick: " << map.quick_search_prob_[i] << ",   full: " << map.search_prob_[i] << std::endl;
    os << "travel: ";
    for(int j=0; j<map.travel_times_[i].size(); j++){
      os << j << ": ";
      for(int k=0; k<map.travel_path_[i][j].size(); k++)
        os << map.travel_path_[i][j][k] << " ";
      os << ", " << map.travel_times_[i][j] << "; ";
    }
    os << std::endl << "quick pos: " << map.quick_search_poses_[i].position.x << " " << map.quick_search_poses_[i].position.y << std::endl;
    os << (map.not_visited_[i] ? "not visited" : "visited") << std::endl << std::endl;
  }
  os << std::endl;

  return os;
}
