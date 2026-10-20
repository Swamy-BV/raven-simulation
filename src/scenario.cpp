// SPDX-License-Identifier: AGPL-3.0-only
#include "scenario.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <json.hpp>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>

namespace raven::simulation {
namespace {
using Json = nlohmann::json;
using Point = std::array<double, 2>;
constexpr double kPi = 3.14159265358979323846;

Json Read(const std::filesystem::path& path) {
  std::ifstream file(path);
  if (!file) throw std::runtime_error("Cannot read " + path.string());
  return Json::parse(file, nullptr, true, true);
}

void Write(const std::filesystem::path& path, const Json& value) {
  std::ofstream file(path);
  if (!file || !(file << value.dump(2) << '\n')) {
    throw std::runtime_error("Cannot write " + path.string());
  }
}

std::vector<Point> Route(const Json& route) {
  if (route.at("type") != "rounded_rectangle") {
    throw std::runtime_error("Unsupported route type");
  }
  const auto lower = route.at("min_xy_m").get<Point>();
  const auto upper = route.at("max_xy_m").get<Point>();
  const double radius = route.at("corner_radius_m").get<double>();
  if (!std::isfinite(radius) || radius <= 0 || !std::isfinite(lower[0]) ||
      !std::isfinite(lower[1]) || !std::isfinite(upper[0]) ||
      !std::isfinite(upper[1]) || upper[0] - lower[0] <= 2 * radius ||
      upper[1] - lower[1] <= 2 * radius || upper[0] - lower[0] > 10000 ||
      upper[1] - lower[1] > 10000) {
    throw std::runtime_error("Invalid rounded rectangle bounds or radius");
  }
  std::vector<Point> points;
  const auto line = [&](const Point& a, const Point& b) {
    const int count =
        static_cast<int>(std::ceil(std::hypot(b[0] - a[0], b[1] - a[1])));
    for (int i = 0; i < count; ++i) {
      const double t = static_cast<double>(i) / count;
      points.push_back({a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t});
    }
  };
  const auto arc = [&](const Point& center, const double a, const double b) {
    constexpr int kSamples = 12;
    for (int i = 0; i < kSamples; ++i) {
      const double angle = a + (b - a) * i / kSamples;
      points.push_back({center[0] + radius * std::cos(angle),
                        center[1] + radius * std::sin(angle)});
    }
  };
  const double left = lower[0], bottom = lower[1];
  const double right = upper[0], top = upper[1];
  line({left, bottom + radius}, {left, top - radius});
  arc({left + radius, top - radius}, kPi, kPi / 2);
  line({left + radius, top}, {right - radius, top});
  arc({right - radius, top - radius}, kPi / 2, 0);
  line({right, top - radius}, {right, bottom + radius});
  arc({right - radius, bottom + radius}, 0, -kPi / 2);
  line({right - radius, bottom}, {left + radius, bottom});
  arc({left + radius, bottom + radius}, -kPi / 2, -kPi);
  points.push_back(points.front());
  return points;
}

std::vector<Json> ActorProfiles(const Json& settings) {
  std::vector<Json> profiles{settings};
  const auto traffic = settings.value("traffic", Json::array());
  if (!traffic.is_array() || traffic.size() > 7) {
    throw std::runtime_error("Traffic must contain at most seven vehicles");
  }
  for (const auto& actor : traffic) {
    auto profile = settings;
    auto target = settings.at("target");
    target.update(actor);
    profile["target"] = target;
    profile["target_speed_mps"] =
        actor.value("speed_mps", settings.at("target_speed_mps").get<double>());
    profile["phase_fraction"] = actor.value("phase_fraction", 0.0);
    profile["reverse"] = actor.value("reverse", false);
    profile["route_offset_xy_m"] =
        actor.value("route_offset_xy_m", Point{0, 0});
    profiles.push_back(std::move(profile));
  }
  return profiles;
}

std::vector<double> Speeds(const std::vector<Point>& points,
                           const Json& profile, const double speed) {
  std::vector<double> result(points.size(), speed);
  if (!profile.contains("vehicle_acceleration_mps2")) return result;
  const auto acceleration =
      profile.at("vehicle_acceleration_mps2").get<double>();
  const auto lateral =
      profile.at("vehicle_lateral_acceleration_mps2").get<double>();
  if (!std::isfinite(acceleration) || acceleration <= 0 || acceleration > 10 ||
      !std::isfinite(lateral) || lateral <= 0 || lateral > 10) {
    throw std::runtime_error("Invalid vehicle acceleration limits");
  }
  for (std::size_t i = 1; i + 1 < points.size(); ++i) {
    const double ax = points[i][0] - points[i - 1][0];
    const double ay = points[i][1] - points[i - 1][1];
    const double bx = points[i + 1][0] - points[i][0];
    const double by = points[i + 1][1] - points[i][1];
    const auto angle =
        std::abs(std::atan2(ax * by - ay * bx, ax * bx + ay * by));
    const auto curvature =
        angle / ((std::hypot(ax, ay) + std::hypot(bx, by)) / 2);
    if (curvature > 1e-6)
      result[i] = std::min(speed, std::sqrt(lateral / curvature));
  }
  result.front() = 0;
  result.back() = 0;
  for (std::size_t i = 1; i < points.size(); ++i) {
    const auto distance = std::hypot(points[i][0] - points[i - 1][0],
                                     points[i][1] - points[i - 1][1]);
    result[i] = std::min(result[i], std::sqrt(result[i - 1] * result[i - 1] +
                                              2 * acceleration * distance));
  }
  for (std::size_t i = points.size() - 1; i > 0; --i) {
    const auto distance = std::hypot(points[i][0] - points[i - 1][0],
                                     points[i][1] - points[i - 1][1]);
    result[i - 1] = std::min(
        result[i - 1],
        std::sqrt(result[i] * result[i] + 2 * acceleration * distance));
  }
  return result;
}

Json Trajectory(const Json& profile) {
  const double speed = profile.at("target_speed_mps").get<double>();
  const double delay = profile.at("target_start_delay_s").get<double>();
  const double height = profile.at("target").at("height_ned_m").get<double>();
  if (!std::isfinite(speed) || speed <= 0 || speed > 30 ||
      !std::isfinite(delay) || delay <= 0 || delay > 300 ||
      !std::isfinite(height)) {
    throw std::runtime_error("Invalid target speed, start delay or height");
  }
  auto points = Route(profile.at("route"));
  const auto phase = profile.value("phase_fraction", 0.0);
  const auto offset = profile.value("route_offset_xy_m", Point{0, 0});
  if (!std::isfinite(phase) || phase < 0 || phase >= 1 ||
      !std::isfinite(offset[0]) || !std::isfinite(offset[1])) {
    throw std::runtime_error("Invalid traffic phase or lane offset");
  }
  points.pop_back();
  if (profile.value("reverse", false))
    std::reverse(points.begin(), points.end());
  const auto phase_index = static_cast<std::size_t>(phase * points.size());
  std::rotate(points.begin(), points.begin() + phase_index, points.end());
  for (auto& point : points) {
    point[0] += offset[0];
    point[1] += offset[1];
  }
  points.push_back(points.front());
  const auto speeds = Speeds(points, profile, speed);
  Json t = {{"name", profile.at("scene_id")}};
  double elapsed = delay;
  double heading =
      std::atan2(points[1][1] - points[0][1], points[1][0] - points[0][0]);
  const auto append = [&](const double time, const Point& point,
                          const double yaw, const double velocity) {
    t["time_sec"].push_back(time);
    t["pose_x"].push_back(point[0]);
    t["pose_y"].push_back(point[1]);
    t["pose_z"].push_back(height);
    t["pose_roll"].push_back(0);
    t["pose_pitch"].push_back(0);
    t["pose_yaw"].push_back(yaw);
    t["velocity_linear_x"].push_back(velocity * std::cos(yaw));
    t["velocity_linear_y"].push_back(velocity * std::sin(yaw));
    t["velocity_linear_z"].push_back(0);
  };
  append(0, points.front(), heading, 0);
  for (std::size_t i = 0; i < points.size(); ++i) {
    if (i + 1 < points.size()) {
      double next = std::atan2(points[i + 1][1] - points[i][1],
                               points[i + 1][0] - points[i][0]);
      while (next - heading > kPi) next -= 2 * kPi;
      while (next - heading < -kPi) next += 2 * kPi;
      heading = next;
    }
    append(elapsed, points[i], heading, speeds[i]);
    if (i + 1 < points.size())
      elapsed += std::hypot(points[i + 1][0] - points[i][0],
                            points[i + 1][1] - points[i][1]) /
                 ((speeds[i] + speeds[i + 1]) / 2);
  }
  return t;
}

Json Scene(const Json& profile) {
  const auto& aircraft = profile.at("aircraft");
  return {
      {"id", profile.at("scene_id")},
      {"actors",
       {{{"type", "robot"},
         {"name", aircraft.at("name")},
         {"origin",
          {{"xyz", aircraft.at("xyz")}, {"rpy-deg", aircraft.at("rpy_deg")}}},
         {"robot-config", "camera.jsonc"}}}},
      {"environment-actors",
       {{{"type", "env_actor"},
         {"name", profile.at("target").at("name")},
         {"origin", {{"xyz", "0 0 0"}, {"rpy-deg", "0 0 0"}}},
         {"env-actor-config", "target.jsonc"}}}},
      {"clock",
       {{"type", "steppable"},
        {"step-ns", 4000000},
        {"real-time-update-rate", 4000000},
        {"pause-on-start", false}}},
      {"home-geo-point",
       {{"latitude", 47.397742}, {"longitude", 8.545594}, {"altitude", 122}}},
      {"scene-type", "UnrealNative"}};
}

std::string ActorFile(const std::size_t index) {
  return index == 0 ? "target.jsonc"
                    : "traffic-" + std::to_string(index) + ".jsonc";
}

Json Camera() {
  return {{"id", "Front"},
          {"type", "camera"},
          {"enabled", true},
          {"parent-link", "Frame"},
          {"capture-interval", 1.0 / 30},
          {"origin", {{"xyz", "0.5 0 0.2"}, {"rpy-deg", "0 -35 0"}}},
          {"capture-settings",
           {{{"image-type", 0},
             {"width", 640},
             {"height", 360},
             {"fov-degrees", 80.2140913183},
             {"capture-enabled", true},
             {"streaming-enabled", true},
             {"pixels-as-float", false},
             {"compress", false},
             {"target-gamma", 2.2}}}}};
}
}  // namespace

void ValidateScenario(const std::filesystem::path& profile,
                      const std::filesystem::path& directory) {
  const auto settings = Read(profile);
  if (settings.at("schema_version") != 1) {
    throw std::runtime_error("Unsupported scenario schema version");
  }
  const auto profiles = ActorProfiles(settings);
  std::set<std::string> names{
      settings.at("aircraft").at("name").get<std::string>()};
  for (const auto& actor_profile : profiles) {
    const auto& actor = actor_profile.at("target");
    for (const auto* key : {"name", "asset", "display_name"}) {
      if (actor.at(key).get<std::string>().empty()) {
        throw std::runtime_error("Target identifiers must not be empty");
      }
    }
    for (const auto* key : {"name", "display_name"}) {
      if (!names.insert(actor.at(key).get<std::string>()).second) {
        throw std::runtime_error("Actor and display names must be unique");
      }
    }
    if (!std::isfinite(actor.at("visual_yaw_offset_deg").get<double>())) {
      throw std::runtime_error("Invalid target visual rotation");
    }
    Trajectory(actor_profile);
  }
  const auto trajectory = Trajectory(settings);
  const auto size = trajectory.at("time_sec").size();
  for (const auto& field : trajectory.items()) {
    if (field.key() != "name" && field.value().size() != size) {
      throw std::runtime_error("Mismatched trajectory arrays");
    }
  }
  if (trajectory.at("pose_x").front() != trajectory.at("pose_x").back() ||
      trajectory.at("pose_y").front() != trajectory.at("pose_y").back()) {
    throw std::runtime_error("Street route is not closed");
  }
  std::filesystem::create_directories(directory);
  Write(directory / "trajectory.json", trajectory);
  auto scene = Scene(settings);
  Json traffic = Json::array();
  for (std::size_t i = 1; i < profiles.size(); ++i) {
    const auto& actor = profiles[i].at("target");
    scene["environment-actors"].push_back(
        {{"type", "env_actor"},
         {"name", actor.at("name")},
         {"origin", {{"xyz", "0 0 0"}, {"rpy-deg", "0 0 0"}}},
         {"env-actor-config", ActorFile(i)}});
    traffic.push_back(actor);
    Write(directory / ("traffic-" + std::to_string(i) + "-trajectory.json"),
          Trajectory(profiles[i]));
  }
  Write(directory / "scene.jsonc", scene);
  Write(directory / "bindings.json",
        {{"aircraft", settings.at("aircraft").at("name")},
         {"camera", "Front"},
         {"target", settings.at("target")},
         {"traffic", traffic}});
}

void GenerateScenario(const std::filesystem::path& profile,
                      const std::filesystem::path& robot_template,
                      const std::filesystem::path& directory,
                      const std::string& host) {
  ValidateScenario(profile, directory);
  auto robot = Read(robot_template);
  auto& settings = robot["controller"]["px4-settings"];
  settings["local-host-ip"] = host;
  settings["tcp-port"] = 4560;
  settings["control-ip-address"] = "remote";
  settings["control-port"] = 14581;
  settings["control-port-remote"] = 14581;
  settings["control-port-local"] = 14580;
  settings["parameters"] = Json::object();
  Json sensors = Json::array({Camera()});
  for (const auto& sensor : robot.at("sensors")) {
    if (sensor.at("type") != "camera") sensors.push_back(sensor);
  }
  robot["sensors"] = sensors;
  Write(directory / "camera.jsonc", robot);
  auto rover =
      Read(robot_template.parent_path() / "robot_rover_fastphysics.jsonc");
  Json actor = {{"links", rover.at("links")}, {"joints", rover.at("joints")}};
  for (auto& link : actor["links"]) {
    link.erase("inertial");
    link["collision"] = {{"enabled", false}};
  }
  const auto profiles = ActorProfiles(Read(profile));
  for (std::size_t i = 0; i < profiles.size(); ++i) {
    actor["script"] = {{"loop", true},
                       {"auto_start", true},
                       {"trajectory", Trajectory(profiles[i])}};
    Write(directory / ActorFile(i), actor);
  }
}
}  // namespace raven::simulation
