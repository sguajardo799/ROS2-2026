#include "speed_estimator.h"

void SpeedEstimator::begin(int32_t count, int64_t nowUs) {
  for (int i = 0; i <= VEL_WINDOW_SAMPLES; i++) {
    counts_[i] = count;
    times_[i] = nowUs;
  }
  idx_ = 0;
  rpm_ = 0;
}

float SpeedEstimator::update(int32_t count, int64_t nowUs) {
  // Buffer circular de N+1 posiciones: la más nueva en idx_, la más antigua a continuación.
  idx_ = (idx_ + 1) % (VEL_WINDOW_SAMPLES + 1);
  counts_[idx_] = count;
  times_[idx_] = nowUs;
  uint8_t oldest = (idx_ + 1) % (VEL_WINDOW_SAMPLES + 1);

  int64_t dtUs = nowUs - times_[oldest];
  if (dtUs <= 0) return rpm_;  // sin tiempo transcurrido: conserva el valor anterior

  int32_t deltaCounts = count - counts_[oldest];
  rpm_ = (float)deltaCounts / kCountsPerWheelRev * (60.0e6f / (float)dtUs);
  return rpm_;
}
