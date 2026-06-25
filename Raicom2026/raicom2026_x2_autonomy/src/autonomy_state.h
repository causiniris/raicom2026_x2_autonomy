#pragma once

enum class AutonomyState {
  STAND_UP,
  WALK_TO_ZONE_1,
  DANCE,
  FALL_GUARD,
  DONE
};

const char* stateName(AutonomyState state);
AutonomyState nextState(AutonomyState state);
