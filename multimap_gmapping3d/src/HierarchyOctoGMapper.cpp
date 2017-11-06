//
// Created by thomas on 05.11.17.
//

#include "multimap_gmapping3d/HierarchyOctoGMapper.h"

HierarchyOctoGMapper::HierarchyOctoGMapper(bool start){
  sub_mapper_.push_back(new OctoGMapper);
  if(start)
    startMapper(0);
}

void HierarchyOctoGMapper::addMapper(bool enable){
  sub_mapper_.push_back(new OctoGMapper);
  if(enable)
    startMapper(sub_mapper_.size() - 1);
}

void HierarchyOctoGMapper::startMapper(int mapper_idx){
  if(current_mapper_ >= 0)
    sub_mapper_[current_mapper_]->disable();
  current_mapper_ = mapper_idx;
  sub_mapper_[current_mapper_]->enable();
}
