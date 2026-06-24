#include "autonomy_state.h"

#include <iostream>

void runPlanner(AutonomyState state) {
  switch (state) {
    case AutonomyState::STAND_UP:
      std::cout << "Planner: prepare stable stand trajectory" << std::endl;
      break;
    case AutonomyState::WALK_TO_ZONE_1:
      std::cout << "Planner: prepare route to zone 1" << std::endl;
      break;
    case AutonomyState::DANCE:
      std::cout << "Planner: prepare dance sequence" << std::endl;
      break;
    case AutonomyState::DONE:
      break;
  }
}
