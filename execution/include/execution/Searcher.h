//
// Created by thomas on 02.01.18.
//

#ifndef EXECUTION_SEARCHER_H
#define EXECUTION_SEARCHER_H

#include <execution/ObjectMapper.h>
#include <execution/OctoMapper.h>

class Searcher{
  private:
    OctoMapper* octo_mapper_;
    ObjectMap* obj_map_;

  public:
    Searcher();

};

#endif //EXECUTION_SEARCHER_H
