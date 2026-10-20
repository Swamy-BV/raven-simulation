// SPDX-License-Identifier: AGPL-3.0-only
#pragma once
#include <filesystem>
#include <string>

namespace raven::simulation {
void GenerateScenario(const std::filesystem::path& profile,
                      const std::filesystem::path& robot_template,
                      const std::filesystem::path& directory,
                      const std::string& host);
void ValidateScenario(const std::filesystem::path& profile,
                      const std::filesystem::path& directory);
}  // namespace raven::simulation
