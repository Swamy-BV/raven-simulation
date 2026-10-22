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

#include "road_mesh.h"
#include "route.h"

namespace raven::simulation {
namespace {
using Json = nlohmann::json;
using Point = std::array<double, 2>;
constexpr double kPi = 3.14159265358979323846;

Json Read(const std::filesystem::path& path) {
  std::ifstream file(path);
  if (!file) {
    throw std::runtime_error("Cannot read " + path.string());
  }
  return Json::parse(file, nullptr, true, true);
}

void Write(const std::filesystem::path& path, const Json& value) {
  std::ofstream file(path);
  if (!file || !(file << value.dump(2) << '\n')) {
    throw std::runtime_error("Cannot write " + path.string());
  }
}

std::vector<Point> Route(const Json& route) {
  if (route.at("type") == "segments") {
    auto points = SegmentRoute(route);
    return OffsetPath(points, route.value("lane_offset_m", 0.0));
  }
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
    // Preserve the compact profiles, but keep wide high-speed bends near the
    // same metre-scale resolution as the straight sections.
    const auto samples =
        std::max(12, static_cast<int>(std::ceil(radius * std::abs(b - a))));
    for (int i = 0; i < samples; ++i) {
      const double angle = a + (b - a) * i / samples;
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

Json InsetRoute(Json route, const double inset) {
  if (!std::isfinite(inset)) {
    throw std::runtime_error("Invalid traffic route inset");
  }
  if (route.at("type") == "segments") {
    const double offset = route.value("lane_offset_m", 0.0) + inset;
    if (std::abs(offset) > 5) {
      throw std::runtime_error("Traffic lane offset exceeds five metres");
    }
    route["lane_offset_m"] = offset;
    return route;
  }
  auto minimum = route.at("min_xy_m").get<Point>();
  auto maximum = route.at("max_xy_m").get<Point>();
  for (std::size_t axis = 0; axis < minimum.size(); ++axis) {
    minimum[axis] += inset;
    maximum[axis] -= inset;
  }
  route["min_xy_m"] = minimum;
  route["max_xy_m"] = maximum;
  route["corner_radius_m"] = route.at("corner_radius_m").get<double>() - inset;
  return route;
}

std::vector<Json> ActorProfiles(const Json& settings) {
  if (!settings.contains("target")) {
    if (settings.contains("traffic") || settings.contains("route")) {
      throw std::runtime_error("Traffic and routes require a target");
    }
    return {};
  }
  std::vector<Json> profiles{settings};
  const auto traffic = settings.value("traffic", Json::array());
  if (!traffic.is_array() || traffic.size() > 24) {
    throw std::runtime_error(
        "Traffic must contain at most twenty-four vehicles");
  }
  for (const auto& actor : traffic) {
    auto profile = settings;
    auto target = settings.at("target");
    target.update(actor);
    profile["target"] = target;
    profile["route"] = InsetRoute(actor.value("route", settings.at("route")),
                                  actor.value("route_inset_m", 0.0));
    profile["lane_changes"] = actor.value("lane_changes", Json::array());
    profile["target_start_delay_s"] = actor.value(
        "start_delay_s", settings.at("target_start_delay_s").get<double>());
    profile.erase("stop_after_m");
    if (actor.contains("stop_after_m")) {
      profile["stop_after_m"] = actor.at("stop_after_m");
    }
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
  if (!profile.contains("vehicle_acceleration_mps2")) {
    return result;
  }
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
    if (curvature > 1e-6) {
      result[i] = std::min(speed, std::sqrt(lateral / curvature));
    }
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
    throw std::runtime_error("Invalid traffic phase or route translation");
  }
  points.pop_back();
  if (profile.value("reverse", false)) {
    std::reverse(points.begin(), points.end());
  }
  const auto phase_index = static_cast<std::size_t>(phase * points.size());
  std::rotate(points.begin(), points.begin() + phase_index, points.end());
  for (auto& point : points) {
    point[0] += offset[0];
    point[1] += offset[1];
  }
  points.push_back(points.front());
  points = ChangeLanes(points, profile.value("lane_changes", Json::array()));
  points.pop_back();
  const auto laps = profile.value("route_laps", 1U);
  if (laps == 0 || laps > 10) {
    throw std::runtime_error("Route laps must be 1..10");
  }
  const auto lap = points;
  for (unsigned int index = 1; index < laps; ++index) {
    points.insert(points.end(), lap.begin(), lap.end());
  }
  points.push_back(points.front());
  const bool repeat = !profile.contains("stop_after_m");
  if (!repeat) {
    StopAfterDistance(points, profile.at("stop_after_m").get<double>());
  }
  const auto speeds = Speeds(points, profile, speed);
  Json t = {{"name", profile.at("scene_id")}, {"loop", repeat}};
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
      while (next - heading > kPi) {
        next -= 2 * kPi;
      }
      while (next - heading < -kPi) {
        next += 2 * kPi;
      }
      heading = next;
    }
    append(elapsed, points[i], heading, speeds[i]);
    if (i + 1 < points.size()) {
      elapsed += std::hypot(points[i + 1][0] - points[i][0],
                            points[i + 1][1] - points[i][1]) /
                 ((speeds[i] + speeds[i + 1]) / 2);
    }
  }
  return t;
}

Json Course(const Json& profile) {
  Json objects = Json::array();
  if (!profile.contains("course")) {
    return objects;
  }
  const auto& course = profile.at("course");
  if (course.value("type", std::string{}) == "continuous") {
    return RoadScenery(Route(profile.at("route")), course);
  }
  const auto spacing = course.at("tile_spacing_m").get<double>();
  const auto width = course.at("road_width_m").get<double>();
  const auto height = course.at("surface_z_m").get<double>();
  if (!std::isfinite(spacing) || spacing < 2 || spacing > 16 ||
      !std::isfinite(width) || width < 8 || width > 40 ||
      !std::isfinite(height)) {
    throw std::runtime_error("Invalid course geometry");
  }
  auto minimum = profile.at("route").at("min_xy_m").get<Point>();
  auto maximum = profile.at("route").at("max_xy_m").get<Point>();
  const auto object = [&](const std::string& name, const std::string& asset,
                          const Json& position, double yaw, const Json& scale) {
    if (asset.empty()) {
      throw std::runtime_error("Course asset must not be empty");
    }
    objects.push_back({{"name", name},
                       {"asset", asset},
                       {"position", position},
                       {"yaw", yaw},
                       {"scale", scale}});
  };
  // The shared pad supplies a continuous collision surface above the existing
  // environment. Road meshes are visual tiles, independent of car trajectories.
  object("RaceSurface", course.at("surface_asset"),
         {(minimum[0] + maximum[0]) / 2, (minimum[1] + maximum[1]) / 2,
          height + 0.5},
         0, {maximum[0] - minimum[0] + 100, maximum[1] - minimum[1] + 100, 1});
  const auto points = Route(profile.at("route"));
  double accumulated = spacing;
  for (std::size_t i = 0; i + 1 < points.size(); ++i) {
    if (i > 0) {
      accumulated += std::hypot(points[i][0] - points[i - 1][0],
                                points[i][1] - points[i - 1][1]);
    }
    if (accumulated < spacing) {
      continue;
    }
    accumulated = 0;
    const auto yaw = std::atan2(points[i + 1][1] - points[i][1],
                                points[i + 1][0] - points[i][0]);
    // Neighborhood's Road_01New tile is 16 x 16 metres at unit scale.
    object("RaceRoad" + std::to_string(i), course.at("road_asset"),
           {points[i][0], points[i][1], height - 0.03}, yaw - kPi / 2,
           {(spacing + 6) / 16, width / 16, 1});
    if (objects.size() > 2000) {
      throw std::runtime_error("Course exceeds scenery object limit");
    }
  }
  return objects;
}

Json Scenery(const Json& profile) {
  auto objects = Course(profile);
  const auto extras = profile.value("scenery", Json::array());
  if (!extras.is_array() || extras.size() > 64) {
    throw std::runtime_error("Extra scenery must contain at most 64 objects");
  }
  std::set<std::string> names;
  for (const auto& object : objects) {
    names.insert(object.at("name").get<std::string>());
  }
  for (const auto& object : extras) {
    const auto name = object.at("name").get<std::string>();
    const auto position = object.at("position").get<std::array<double, 3>>();
    const auto scale = object.at("scale").get<std::array<double, 3>>();
    if (name.empty() || !names.insert(name).second ||
        object.at("asset").get<std::string>().empty() ||
        !std::isfinite(object.at("yaw").get<double>())) {
      throw std::runtime_error("Invalid extra scenery identity or heading");
    }
    for (std::size_t axis = 0; axis < position.size(); ++axis) {
      if (!std::isfinite(position[axis]) || std::abs(position[axis]) > 10000 ||
          !std::isfinite(scale[axis]) || scale[axis] <= 0 || scale[axis] > 10) {
        throw std::runtime_error("Invalid extra scenery position or scale");
      }
    }
    objects.push_back(object);
  }
  return objects;
}

bool IsHitl(const Json& profile) {
  const auto mode = profile.value("flight_controller", std::string("sitl"));
  if (mode != "sitl" && mode != "hitl") {
    throw std::runtime_error("Flight controller must be sitl or hitl");
  }
  return mode == "hitl";
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
         {"robot-config", "camera.jsonc"},
         {"start-landed", IsHitl(profile)}}}},
      {"environment-actors", Json::array()},
      {"clock",
       {{"type", IsHitl(profile) ? "real-time" : "steppable"},
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

int RenderWidth(const Json& profile) {
  const auto width =
      profile.value("camera", Json::object()).value("render_width", 640);
  if (width != 640 && width != 1280 && width != 1920) {
    throw std::runtime_error("Camera render width must be 640, 1280 or 1920");
  }
  return width;
}

int CaptureRate(const Json& profile) {
  const int rate =
      profile.value("camera", Json::object()).value("capture_rate_hz", 30);
  if (rate < 1 || rate > 60) {
    throw std::runtime_error("Camera capture rate must be 1 to 60 Hz");
  }
  return rate;
}

Json Camera(const Json& profile) {
  const auto camera = profile.value("camera", Json::object());
  const auto pitch = camera.value("pitch_down_deg", 35.0);
  const auto fov = camera.value("horizontal_fov_deg", 80.2140913183);
  if (!std::isfinite(pitch) || pitch < 0 || pitch > 80 || !std::isfinite(fov) ||
      fov < 30 || fov > 120) {
    throw std::runtime_error("Invalid camera pitch or field of view");
  }
  return {{"id", "Front"},
          {"type", "camera"},
          {"enabled", true},
          {"parent-link", "Frame"},
          {"capture-interval", 1.0 / CaptureRate(profile)},
          {"gimbal",
           {{"lock-roll", camera.value("stabilized", false)},
            {"lock-pitch", camera.value("stabilized", false)},
            {"lock-yaw", false}}},
          {"origin",
           {{"xyz", "0.5 0 0.2"},
            {"rpy-deg", "0 " + std::to_string(-pitch) + " 0"}}},
          {"capture-settings",
           {{{"image-type", 0},
             {"width", RenderWidth(profile)},
             {"height", RenderWidth(profile) * 9 / 16},
             {"fov-degrees", fov},
             {"capture-enabled", true},
             {"streaming-enabled", true},
             {"pixels-as-float", false},
             {"compress", false},
             {"target-gamma", 2.2}}}}};
}

void WriteFlightConfiguration(const Json& profile,
                              const std::filesystem::path& directory) {
  std::ofstream file(directory / "px4.env", std::ios::binary);
  const auto parameters = profile.value("px4_parameters", Json::object());
  if (!parameters.is_object()) {
    throw std::runtime_error("PX4 parameters must be an object");
  }
  for (const auto& entry : parameters.items()) {
    const auto& name = entry.key();
    if (name.rfind("MPC_", 0) != 0 ||
        name.find_first_not_of("ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_") !=
            std::string::npos ||
        !entry.value().is_number() ||
        !std::isfinite(entry.value().get<double>())) {
      throw std::runtime_error("Invalid scenario position-control parameter");
    }
    file << "PX4_PARAM_" << name << '=' << entry.value().dump() << '\n';
  }
  if (!file) {
    throw std::runtime_error("Cannot write PX4 scenario parameters");
  }
}

void WriteCameraConfiguration(const Json& profile,
                              const std::filesystem::path& directory) {
  Camera(profile);
  const auto camera = profile.value("camera", Json::object());
  const auto fov = camera.value("horizontal_fov_deg", 80.2140913183);
  constexpr double kHalfWidth = 320;
  constexpr double kHalfHeight = 180;
  const auto focal = kHalfWidth / std::tan(fov * kPi / 360);
  Write(
      directory / "camera.yaml",
      {{"image_width", 640},
       {"image_height", 360},
       {"camera_name", "front_camera"},
       {"camera_matrix",
        {{"rows", 3},
         {"cols", 3},
         {"data", {focal, 0, kHalfWidth, 0, focal, kHalfHeight, 0, 0, 1}}}},
       {"distortion_model", "plumb_bob"},
       {"distortion_coefficients",
        {{"rows", 1}, {"cols", 5}, {"data", {0, 0, 0, 0, 0}}}},
       {"rectification_matrix",
        {{"rows", 3}, {"cols", 3}, {"data", {1, 0, 0, 0, 1, 0, 0, 0, 1}}}},
       {"projection_matrix",
        {{"rows", 3},
         {"cols", 4},
         {"data",
          {focal, 0, kHalfWidth, 0, 0, focal, kHalfHeight, 0, 0, 0, 1, 0}}}}});
  auto following = profile.value("following_parameters", Json::object());
  if (profile.value("flight_controller", std::string("sitl")) == "hitl") {
    following["px4_namespace"] = "";
  }
  following["camera_pitch"] = camera.value("pitch_down_deg", 35.0) * kPi / 180;
  following["camera_stabilized"] = camera.value("stabilized", false);
  following["camera_forward"] = 0.5;
  following["camera_right"] = 0.0;
  following["camera_down"] = 0.2;
  Write(directory / "following.yaml", {{"/raven/vehicle_1/autonomy/guidance",
                                        {{"ros__parameters", following}}}});
  Write(directory / "tracking.yaml",
        {{"/raven/vehicle_1/perception/target_tracker",
          {{"ros__parameters",
            profile.value("tracking_parameters", Json::object())}}}});
}
}  // namespace

void ValidateScenario(const std::filesystem::path& profile,
                      const std::filesystem::path& directory) {
  const auto settings = Read(profile);
  if (settings.at("schema_version") != 1) {
    throw std::runtime_error("Unsupported scenario schema version");
  }
  const auto profiles = ActorProfiles(settings);
  if (settings.value("target_start_on_ready", false) &&
      (!IsHitl(settings) || profiles.empty())) {
    throw std::runtime_error(
        "Deferred target start requires HITL and a target");
  }
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
  std::filesystem::create_directories(directory);
  WriteCameraConfiguration(settings, directory);
  WriteFlightConfiguration(settings, directory);
  auto scene = Scene(settings);
  Json traffic = Json::array();
  for (std::size_t i = 0; i < profiles.size(); ++i) {
    const auto& actor = profiles[i].at("target");
    if (!IsHitl(settings)) {
      scene["environment-actors"].push_back(
          {{"type", "env_actor"},
           {"name", actor.at("name")},
           {"origin", {{"xyz", "0 0 0"}, {"rpy-deg", "0 0 0"}}},
           {"env-actor-config", ActorFile(i)}});
    }
    if (i > 0) {
      traffic.push_back(actor);
    }
    const auto trajectory = Trajectory(profiles[i]);
    const auto size = trajectory.at("time_sec").size();
    for (const auto& field : trajectory.items()) {
      if (field.key() != "name" && field.key() != "loop" &&
          field.value().size() != size) {
        throw std::runtime_error("Mismatched trajectory arrays");
      }
    }
    if (trajectory.value("loop", true) &&
        (trajectory.at("pose_x").front() != trajectory.at("pose_x").back() ||
         trajectory.at("pose_y").front() != trajectory.at("pose_y").back())) {
      throw std::runtime_error("Street route is not closed");
    }
    const auto filename =
        i == 0 ? "trajectory.json"
               : "traffic-" + std::to_string(i) + "-trajectory.json";
    Write(directory / filename, trajectory);
  }
  Json road_mesh = nullptr;
  if (settings.contains("course") &&
      settings.at("course").value("type", std::string{}) == "continuous") {
    const auto& course = settings.at("course");
    WriteRoadMesh(Route(settings.at("route")), course, directory, false);
    road_mesh = {{"file", "road.gltf"},
                 {"surface_z_m", course.at("surface_z_m")}};
  }
  Write(directory / "scene.jsonc", scene);
  Write(directory / "bindings.json",
        {{"real_time", IsHitl(settings)},
         {"road_mesh", road_mesh},
         {"target_start_on_ready",
          settings.value("target_start_on_ready", false)},
         {"aircraft", settings.at("aircraft").at("name")},
         {"camera", "Front"},
         {"target", settings.value("target", Json())},
         {"traffic", traffic},
         {"traffic_start_on_ready",
          settings.value("traffic_start_on_ready", false)},
         {"acceptance", settings.value("acceptance", Json::object())},
         {"scenery", Scenery(settings)}});
}

ScenarioRuntime GenerateScenario(const std::filesystem::path& profile,
                                 const std::filesystem::path& robot_template,
                                 const std::filesystem::path& directory,
                                 const std::string& host,
                                 const std::string& board,
                                 const std::string& serial_port) {
  ValidateScenario(profile, directory);
  const auto profile_settings = Read(profile);
  if (profile_settings.contains("course") &&
      profile_settings.at("course").value("type", std::string{}) ==
          "continuous") {
    // Offline validation builds geometry without needing downloaded textures.
    WriteRoadMesh(Route(profile_settings.at("route")),
                  profile_settings.at("course"), directory, true);
  }
  const bool hitl = IsHitl(profile_settings);
  const ScenarioRuntime runtime{
      hitl ? FlightController::kHitl : FlightController::kSitl,
      Read(profile).value("stream_video_to_board", true),
      RenderWidth(profile_settings), RenderWidth(profile_settings) * 9 / 16,
      CaptureRate(profile_settings)};
  auto robot = Read(robot_template);
  if (robot.value("physics-type", std::string{}) != "non-physics") {
    auto& settings = robot["controller"]["px4-settings"];
    settings["local-host-ip"] = hitl ? "0.0.0.0" : host;
    settings["lock-step"] = !hitl;
    const bool serial_hitl = !serial_port.empty();
    if (serial_hitl && !hitl) {
      throw std::runtime_error(
          "A physical serial controller requires a HITL scenario");
    }
    settings["use-serial"] = serial_hitl;
    settings["use-tcp"] = !serial_hitl;
    if (serial_hitl) {
      settings["serial-port"] = serial_port;
      settings["serial-baud-rate"] = 921600;
    }
    settings["tcp-port"] = 4560;
    settings["control-ip-address"] = hitl ? board : "remote";
    if (hitl) {
      settings["hil-gps-update-rate-hz"] = 10.0;
    }
    settings["control-port"] = 14581;
    settings["control-port-remote"] = 14581;
    settings["control-port-local"] = 14580;
    settings["parameters"] = Json::object();
  }
  Json sensors = Json::array({Camera(Read(profile))});
  for (const auto& sensor : robot.at("sensors")) {
    if (sensor.at("type") != "camera") {
      sensors.push_back(sensor);
    }
  }
  robot["sensors"] = sensors;
  Write(directory / "camera.jsonc", robot);
  const auto profiles = ActorProfiles(Read(profile));
  if (profiles.empty() || hitl) {
    return runtime;
  }
  auto rover =
      Read(robot_template.parent_path() / "robot_rover_fastphysics.jsonc");
  Json actor = {{"links", rover.at("links")}, {"joints", rover.at("joints")}};
  for (auto& link : actor["links"]) {
    link.erase("inertial");
    link["collision"] = {{"enabled", false}};
  }
  for (std::size_t i = 0; i < profiles.size(); ++i) {
    auto trajectory = Trajectory(profiles[i]);
    const bool repeat = trajectory.value("loop", true);
    trajectory.erase("loop");
    actor["script"] = {
        {"loop", repeat}, {"auto_start", true}, {"trajectory", trajectory}};
    Write(directory / ActorFile(i), actor);
  }
  return runtime;
}
}  // namespace raven::simulation
