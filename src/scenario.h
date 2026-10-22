// SPDX-License-Identifier: AGPL-3.0-only
#pragma once
#include <filesystem>
#include <string>

namespace raven::simulation {
enum class FlightController { kSitl, kHitl };

struct ScenarioRuntime {
  FlightController controller;
  bool stream_video_to_board;
  int render_width;
  int render_height;
};

ScenarioRuntime GenerateScenario(const std::filesystem::path& profile,
                                 const std::filesystem::path& robot_template,
                                 const std::filesystem::path& directory,
                                 const std::string& host,
                                 const std::string& board,
                                 const std::string& serial_port);
void ValidateScenario(const std::filesystem::path& profile,
                      const std::filesystem::path& directory);
}  // namespace raven::simulation
