//
// Created by hjanott on 09.10.18.
//

#ifndef PROJECT_GMH_DEFINITIONS_H
#define PROJECT_GMH_DEFINITIONS_H

namespace grid_map_handler
{
constexpr double LETHAL_OBJECT = 100.0;
constexpr double LETHAL_THRESHOLD = 80.0;
constexpr double FREE = 0.0;
constexpr double NO_INFORMATION = -1.0;

constexpr double INFLATION_DISTANCE = 0.35; //meter

  //global layers
constexpr char STATIC[] = "static";
constexpr char STATIC_INF[] = "static_inflation";
constexpr char MERGE[] = "merge";

  //local layers
constexpr char DYNAMIC[] = "dynamic";
constexpr char INFLATION[] = "inflation";
}
#endif //PROJECT_GMH_DEFINITIONS_H
