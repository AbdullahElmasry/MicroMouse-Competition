#pragma once

inline float turnTargetError(float yawDegrees, float targetDegrees) {
  return targetDegrees-yawDegrees;
}

inline bool turnTargetReached(bool turnRight, float yawDegrees,
    float yawRateDps, float targetDegrees, float toleranceDegrees,
    float baseBrakeLeadDegrees, float brakeLookaheadSeconds,
    float &projectedStopDegrees) {
  const float directionalRate = turnRight
      ? (yawRateDps>0?yawRateDps:0)
      : (yawRateDps<0?yawRateDps:0);
  projectedStopDegrees = yawDegrees + directionalRate*brakeLookaheadSeconds +
      (turnRight?baseBrakeLeadDegrees:-baseBrakeLeadDegrees);
  return turnRight ? projectedStopDegrees>=targetDegrees-toleranceDegrees
                   : projectedStopDegrees<=targetDegrees+toleranceDegrees;
}
