// SPDX-License-Identifier: AGPL-3.0-only
#pragma once
#include <array>
#include <vector>

namespace raven::simulation {
struct MotionPoint {
  double seconds;
  std::array<double, 3> position;
  double yaw;
};

// Scene-fixture motion uses elapsed double seconds, never absolute float
// uptime. This does not supply observations or aircraft commands to the
// follower.
class Motion {
 public:
  explicit Motion(std::vector<MotionPoint> points);
  MotionPoint At(double elapsed_seconds) const;

 private:
  std::vector<MotionPoint> points_;
};
}  // namespace raven::simulation
