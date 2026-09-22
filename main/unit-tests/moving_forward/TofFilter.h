#pragma once

// Median of three fresh samples: one isolated spike is rejected.
// Invalid/no-target samples bypass smoothing and discard old wall history.
class TofFilter {
 public:
  void reset() { count_ = next_ = 0; }
  int update(int mm, bool usable = true) {
    if (!usable || mm < 0) { reset(); return mm; }
    samples_[next_] = mm;
    next_ = (next_ + 1) % 3;
    if (count_ < 3) ++count_;
    if (count_ < 3) return mm;
    int a = samples_[0], b = samples_[1], c = samples_[2];
    if (a > b) { int t = a; a = b; b = t; }
    if (b > c) { int t = b; b = c; c = t; }
    return a > b ? a : b;
  }
 private:
  int samples_[3] = {};
  int count_ = 0, next_ = 0;
};
