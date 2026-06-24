#include "autonomy_state.h"

#include <stdexcept>

const char* stateName(AutonomyState state) {
  switch (state) {
    case AutonomyState::STAND_UP:
      return "STAND_UP";
    case AutonomyState::WALK_TO_ZONE_1:
      return "WALK_TO_ZONE_1";
    case AutonomyState::DANCE:
      return "DANCE";
    case AutonomyState::DONE:
      return "DONE";
  }

  throw std::runtime_error("Unknown autonomy state");
}

AutonomyState nextState(AutonomyState state) {
  switch (state) {
    case AutonomyState::STAND_UP:
      return AutonomyState::WALK_TO_ZONE_1;
    case AutonomyState::WALK_TO_ZONE_1:
      return AutonomyState::DANCE;
    case AutonomyState::DANCE:
      return AutonomyState::DONE;
    case AutonomyState::DONE:
      return AutonomyState::DONE;
  }

  throw std::runtime_error("Unknown autonomy state");
}
