#pragma once

struct DifferentialTurnCommands {
  int left;
  int right;
};

// Both wheels move together in opposite directions for the entire turn.
inline DifferentialTurnCommands differentialTurnCommands(
    bool turnRight, int magnitude) {
  return turnRight ? DifferentialTurnCommands{magnitude, -magnitude}
                   : DifferentialTurnCommands{-magnitude, magnitude};
}
