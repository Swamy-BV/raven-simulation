// SPDX-License-Identifier: AGPL-3.0-only
#include "scenario.h"

#include <array>
#include <cmath>
#include <fstream>
#include <json.hpp>
#include <stdexcept>
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

std::vector<Point> Route() {
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
      points.push_back(
          {center[0] + 6 * std::cos(angle), center[1] + 6 * std::sin(angle)});
    }
  };
  line({2, -10}, {2, 106});
  arc({8, 106}, kPi, kPi / 2);
  line({8, 112}, {124, 112});
  arc({124, 106}, kPi / 2, 0);
  line({130, 106}, {130, -10});
  arc({124, -10}, 0, -kPi / 2);
  line({124, -16}, {8, -16});
  arc({8, -10}, -kPi / 2, -kPi);
  points.push_back(points.front());
  return points;
}

Json Trajectory(const Json& profile) {
  const double speed = profile.at("car_speed_mps").get<double>();
  const double delay = profile.at("car_start_delay_s").get<double>();
  if (!std::isfinite(speed) || speed <= 0 || speed > 10 ||
      !std::isfinite(delay) || delay <= 0 || delay > 300) {
    throw std::runtime_error("Invalid car speed or start delay");
  }
  const auto points = Route();
  Json t = {{"name", "neighborhood_street_loop"}};
  double elapsed = delay;
  double heading = kPi / 2;
  const auto append = [&](const double time, const Point& point,
                          const double yaw, const double velocity) {
    t["time_sec"].push_back(time);
    t["pose_x"].push_back(point[0]);
    t["pose_y"].push_back(point[1]);
    t["pose_z"].push_back(-0.75);
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
    append(elapsed, points[i], heading, speed);
    if (i + 1 < points.size())
      elapsed += std::hypot(points[i + 1][0] - points[i][0],
                            points[i + 1][1] - points[i][1]) /
                 speed;
  }
  return t;
}

Json Scene() {
  return {
      {"id", "CarLoop"},
      {"actors",
       {{{"type", "robot"},
         {"name", "CameraDrone"},
         {"origin", {{"xyz", "2 -18 -4"}, {"rpy-deg", "0 0 90"}}},
         {"robot-config", "camera.jsonc"}}}},
      {"environment-actors",
       {{{"type", "env_actor"},
         {"name", "TargetCar"},
         {"origin", {{"xyz", "0 0 0"}, {"rpy-deg", "0 0 0"}}},
         {"env-actor-config", "car-loop.jsonc"}}}},
      {"clock",
       {{"type", "steppable"},
        {"step-ns", 4000000},
        {"real-time-update-rate", 4000000},
        {"pause-on-start", false}}},
      {"home-geo-point",
       {{"latitude", 47.397742}, {"longitude", 8.545594}, {"altitude", 122}}},
      {"scene-type", "UnrealNative"}};
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
  const auto trajectory = Trajectory(Read(profile));
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
  Write(directory / "scene.jsonc", Scene());
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
  actor["script"] = {{"loop", true},
                     {"auto_start", true},
                     {"trajectory", Trajectory(Read(profile))}};
  Write(directory / "car-loop.jsonc", actor);
}
}  // namespace raven::simulation
