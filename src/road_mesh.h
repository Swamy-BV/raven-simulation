// SPDX-License-Identifier: AGPL-3.0-only
#pragma once
#include <filesystem>
#include <json.hpp>

#include "route.h"

namespace raven::simulation {
// glTF is generated outside Git and imported through the standard simulator
// API.
void WriteRoadMesh(const std::vector<RoadPoint>& points,
                   const nlohmann::json& course,
                   const std::filesystem::path& directory,
                   bool include_textures);
nlohmann::json RoadScenery(const std::vector<RoadPoint>& points,
                           const nlohmann::json& course);
}  // namespace raven::simulation
