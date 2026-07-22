//
// Created by thomas on 15.11.17.
//

#include "semantic_mapping_v2/HierarchyMapper.h"

int main(int argc, char** argv){
  ros::init(argc, argv, "hierarchy_mapping");
  HierarchyMapper mapper;
  mapper.run();

  return 0;
}