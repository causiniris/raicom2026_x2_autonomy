#include "autonomy_state.h"
#include "stability_controller.h"

#include <iostream>

void runController(AutonomyState state) {
  StabilityController stability_controller;
  StabilityState stability_state;
  const bool stable = stability_controller.isStable(stability_state);

  switch (state) {
    case AutonomyState::STAND_UP:
      std::cout << "Controller: official MC get-up/balance action active for STAND_UP" << std::endl;
      break;
    case AutonomyState::WALK_TO_ZONE_1:
      std::cout << "Controller: WALK command requires stability gate, stable="
                << (stable ? "true" : "false") << std::endl;
      break;
    case AutonomyState::DANCE:
      std::cout << "Controller: DANCE command requires stability gate, stable="
                << (stable ? "true" : "false") << std::endl;
      break;
    case AutonomyState::FALL_GUARD:
      std::cout << "Controller: fall guard active, commands saturated to recovery limits" << std::endl;
      break;
    case AutonomyState::DONE:
      break;
  }
}
