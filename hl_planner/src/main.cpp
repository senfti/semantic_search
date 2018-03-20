//
// Created by thomas on 20.12.17.
//

#include <ros/ros.h>
#include <hl_planner/Planner.h>
#include <tf/LinearMath/Transform.h>

semantic_mapping_v2::RoomMsg getRoom(int id, float single_view_obj_probs, float obj_probs, float single_view_search_time, float search_time, std::vector<short> links, float to_link_travel_time){
  semantic_mapping_v2::RoomMsg room;
  room.id = id;
  room.obj_probs = std::vector<float>(80, obj_probs);
  room.search_time = search_time;
  room.links = links;
  room.to_link_travel_times = std::vector<float>(links.size(), to_link_travel_time);

  return room;
}

semantic_mapping_v2::HierarchyLinkMsg getLink(int r1, int r2, std::vector<float> dists){
  semantic_mapping_v2::HierarchyLinkMsg link;
  link.room1 = r1;
  link.room2 = r2;
  link.dists = dists;

  return link;
}

const std::vector<std::string> obj_names = { "person","bicycle","bird","cat","dog","backpack","umbrella","handbag","tie","suitcase","frisbee","ski","snowboard",
                                             "sport ball","kite","baseball bat","glove","skateboard","surf board","racket","bottle","wine glass","cup","fork",
                                             "knife","spoon","bowl","banana","apple","sandwich","orange","broccoli","carrot","hot dog","pizza","donut","cake",
                                             "chair","sofa","pot plant","bed","table","toilet","monitor","laptop","mouse","remote","keyboard","cell phone",
                                             "microwave","oven","toaster","sink","refrigerator","book","clock","vase","scissor","teddy bear","hair dryer","toothbrush"};
#include <tf/transform_datatypes.h>
int main(int argc, char** argv){
  ros::init(argc, argv, "hl_planner");
//  Planner planner;
//  planner.exploreAll();

//  float lim = std::numeric_limits<float>::max();
//  semantic_mapping_v2::HierarchySrvResponse res;
//  res.curr_room = 0;
//  res.rooms.push_back(getRoom(0, 0.1f, 0.5f, 20.f, 200.f, {0,1}, 19.f));
//  res.rooms.push_back(getRoom(1,0.04f,0.05f,20.f,200.f,{0,2}, 17.f));
//  res.rooms.push_back(getRoom(2,0.1f,0.5f,50.f,500.f,{1,4}, 16.f));
//  res.rooms.push_back(getRoom(3,0.25f,0.3f,10.f,10000.f,{2,3}, 15.f));
//  res.rooms.push_back(getRoom(4, 0.1f, 0.5f, 20.f, 200.f, {4,5,6,7,8}, 19.f));
//  res.rooms.push_back(getRoom(5,0.04f,0.05f,20.f,200.f,{5}, 17.f));
//  res.rooms.push_back(getRoom(6,0.1f,0.5f,50.f,500.f,{6}, 16.f));
//  res.rooms.push_back(getRoom(7,0.25f,0.3f,10.f,10000.f,{7}, 15.f));
//  res.rooms.push_back(getRoom(8, 0.1f, 0.5f, 20.f, 200.f, {8}, 19.f));
//
//  res.links.push_back(getLink(0,1,{0.f,20.f,30.f,lim,lim,lim,lim,lim,lim}));
//  res.links.push_back(getLink(0,2,{20.f,0.f,lim,lim, 25.f,lim,lim,lim,lim}));
//  res.links.push_back(getLink(1,3,{30.f,lim,0.f,10.f,lim,lim,lim,lim,lim}));
//  res.links.push_back(getLink(3,-1,{lim,lim,10.f,0.f,lim,lim,lim,lim,lim}));
//  res.links.push_back(getLink(2,4,{lim,15.f,lim,lim,0.f,10.f,20.f,30.f,40.f}));
//  res.links.push_back(getLink(4,5,{lim,lim,lim,lim,10.f,0.f,10.f,20.f,30.f}));
//  res.links.push_back(getLink(4,6,{lim,lim,lim,lim,20.f,10.f,0.f,10.f,20.f}));
//  res.links.push_back(getLink(4,7,{lim,lim,lim,lim,30.f,20.f,10.f,0.f,10.f}));
//  res.links.push_back(getLink(4,8,{lim,lim,lim,lim,40.f,30.f,20.f,10.f,0.f}));
//  res.curr_room = 0;
//  HierarchyMap m(res,0);
//  std::cout << m;
//  Planner p;
//  State s(m, res.curr_room);
//  p.generatePlan(m,s);

  Planner p;
  while(ros::ok()){
    std::cout << "-1:explore all | ";
    for(int i=0; i<obj_names.size(); i++){
      std::cout << i << ":" << obj_names[i] << " | ";
    }
    std::cout << std::endl << "Object Number: ";
    int n=-1;
    std::cin >> n;
    if(n>=0 && n<obj_names.size())
      p.run(n);
    if(n==-1)
      p.exploreAll();
  }

  return 0;
}
