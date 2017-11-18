//
// Created by thomas on 05.11.17.
//

#ifndef MULTIMAP_GMAPPING3D_HIERARCHYOCTOGMAPPER_H
#define MULTIMAP_GMAPPING3D_HIERARCHYOCTOGMAPPER_H

#include "multimap_gmapping3d/OctoGMapper.h"

class HierarchyOctoGMapper{
  protected:
    std::vector<OctoGMapper*> sub_mapper_;
    int current_mapper_ = -1;

  public:
    HierarchyOctoGMapper(bool start = true);

    void addMapper(bool enable = true);
    void startMapper(int mapper_idx);
};

#endif //MULTIMAP_GMAPPING3D_HIERARCHYOCTOGMAPPER_H
