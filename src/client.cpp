// SPDX-License-Identifier: AGPL-3.0-only
#include "client.h"

#include <ProjectAirSimMessage/response_message.hpp>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <initializer_list>
#include <stdexcept>
#include <utility>

namespace raven::simulation {
void Check(const sdk::Status status, const std::string& operation) {
  if (status != sdk::Status::OK) {
    throw std::runtime_error(operation + " failed: " +
                             std::to_string(static_cast<unsigned>(status)));
  }
}

Client::Client(const std::string& host)
    : client_(std::make_shared<sdk::Client>()),
      world_(std::make_shared<sdk::World>()) {
  Check(client_->Connect(host), "AirSim connection");
}

Client::~Client() {
  client_->UnsubscribeAll();
  client_->Disconnect();
}

Json Client::Request(const std::string& method, const Json& parameters) {
  sdk::Message message;
  Check(client_->Request(prefix_ + "/" + method, parameters, &message), method);
  microsoft::projectairsim::ResponseMessage response;
  response.Deserialize(static_cast<std::string>(message));
  if (response.GetErrorCode() != 0) {
    throw std::runtime_error(method + " rejected for " + parameters.dump() +
                             ": " + response.GetResult().dump());
  }
  return response.GetResult();
}

void Client::Load(const std::filesystem::path& directory) {
  std::ifstream bindings_file(directory / "bindings.json");
  if (!bindings_file) {
    throw std::runtime_error("Cannot read scene bindings");
  }
  bindings_file >> bindings_;
  Check(world_->Initialize(client_, (directory / "scene.jsonc").string(),
                           directory.string(), 1),
        "Scene loading");
  prefix_ = world_->GetParentTopic();
  actors_.clear();
  pending_traffic_routes_.clear();
  for (const auto& object : bindings_.value("scenery", Json::array())) {
    Request("ListAssets",
            {{"name", "^" + object.at("asset").get<std::string>() + "$"}});
    const auto& position = object.at("position");
    const auto yaw = object.at("yaw").get<double>();
    const Json pose = {
        {"frame_id", "DEFAULT_ID"},
        {"translation",
         {{"x", position[0]}, {"y", position[1]}, {"z", position[2]}}},
        {"rotation",
         {{"w", std::cos(yaw / 2)},
          {"x", 0},
          {"y", 0},
          {"z", std::sin(yaw / 2)}}}};
    if (Request("SpawnObject", {{"object_name", object.at("name")},
                                {"asset_path", object.at("asset")},
                                {"pose", pose},
                                {"scale", object.at("scale")},
                                {"enable_physics", false}})
            .get<std::string>()
            .empty()) {
      throw std::runtime_error("Course scenery could not be spawned");
    }
  }
  const auto scene_start_seconds =
      static_cast<double>(
          Request("GetSimTime", Json::object()).get<std::int64_t>()) /
      1000000000.0;
  const auto& target = bindings_.at("target");
  if (!target.is_null()) {
    if (bindings_.value("real_time", false)) {
      const auto wait = bindings_.value("target_start_on_ready", false);
      pending_target_route_ =
          wait ? directory / "trajectory.json" : std::filesystem::path{};
      StartRoute(directory / "trajectory.json", target.at("name"),
                 scene_start_seconds, wait);
    }
    actors_.push_back({target, Spawn(target), {}});
  }
  std::size_t traffic_index = 1;
  for (const auto& actor : bindings_.value("traffic", Json::array())) {
    if (bindings_.value("real_time", false)) {
      const auto route =
          directory /
          ("traffic-" + std::to_string(traffic_index) + "-trajectory.json");
      const auto wait = bindings_.value("traffic_start_on_ready", false);
      StartRoute(route, actor.at("name"), scene_start_seconds, wait);
      if (wait) {
        pending_traffic_routes_.emplace_back(
            route, actor.at("name").get<std::string>());
      }
    }
    ++traffic_index;
    actors_.push_back({actor, Spawn(actor), {}});
  }
}

void Client::StartRoute(const std::filesystem::path& trajectory,
                        const std::string& actor, double scene_start_seconds,
                        const bool stationary) {
  std::ifstream file(trajectory);
  if (!file) {
    throw std::runtime_error("Cannot read actor trajectory");
  }
  Json route;
  file >> route;
  if (stationary) {
    // A separate constant trajectory holds the fixture during flight setup.
    // Its moving route is anchored only after the operator/test signals ready.
    for (auto& field : route.items()) {
      if (field.value().is_array()) {
        const auto value = field.key().find("velocity_") == 0
                               ? Json(0.0)
                               : field.value().front();
        field.value() = Json::array({value, value});
      }
    }
    route["time_sec"] = {-1.0, 3600.0};
  }
  // TimeSec is float upstream: subtracting host uptime can round the first
  // update to exactly zero. Keep it inside an interval; a zero first knot
  // otherwise makes the upstream interpolator access index -1.
  route["time_sec"][0] = -1.0;
  const auto name = actor + (stationary ? "_hold" : "_route");
  if (!Request("ImportNEDTrajectory",
               {{"traj_name", name},
                {"time", route.at("time_sec")},
                {"pose_x", route.at("pose_x")},
                {"pose_y", route.at("pose_y")},
                {"pose_z", route.at("pose_z")},
                {"pose_roll", route.at("pose_roll")},
                {"pose_pitch", route.at("pose_pitch")},
                {"pose_yaw", route.at("pose_yaw")},
                {"vel_x_lin", route.at("velocity_linear_x")},
                {"vel_y_lin", route.at("velocity_linear_y")},
                {"vel_z_lin", route.at("velocity_linear_z")}})
           .get<bool>()) {
    throw std::runtime_error("Cannot import actor route");
  }
  // Real-time simulation uses host uptime, not zero-based scenario time.
  // Offset only environment actors; PX4 retains sole aircraft control.
  if (!Request("SetEnvActorTrajectory", {{"env_actor_name", actor},
                                         {"traj_name", name},
                                         {"time_offset", scene_start_seconds},
                                         {"x_offset", 0},
                                         {"y_offset", 0},
                                         {"z_offset", 0},
                                         {"roll_offset", 0},
                                         {"pitch_offset", 0},
                                         {"yaw_offset", 0},
                                         {"to_loop", true}})
           .get<bool>()) {
    throw std::runtime_error("Cannot anchor actor route to scene start");
  }
}

void Client::StartTarget() {
  if (pending_target_route_.empty()) {
    throw std::runtime_error("Target is not waiting for a start signal");
  }
  const auto stamp = Request("GetSimTime", Json::object()).get<std::int64_t>();
  StartRoute(pending_target_route_, bindings_.at("target").at("name"),
             static_cast<double>(stamp) / 1000000000.0);
  pending_target_route_.clear();
  for (const auto& route : pending_traffic_routes_) {
    StartRoute(route.first, route.second,
               static_cast<double>(stamp) / 1000000000.0);
  }
  pending_traffic_routes_.clear();
}

std::string Client::Spawn(const Json& target) {
  // ListAssets also populates the simulator's asset cache before spawning.
  std::string asset_filter = "^";
  for (const char character : target.at("asset").get<std::string>()) {
    if (std::string("\\.^$|()[]{}*+?").find(character) != std::string::npos) {
      asset_filter += '\\';
    }
    asset_filter += character;
  }
  asset_filter += '$';
  const auto assets = Request("ListAssets", {{"name", asset_filter}});
  if (!assets.is_array() || assets.empty()) {
    throw std::runtime_error("Scene assets are unavailable");
  }
  const auto pose =
      Request("GetObjectPose", {{"object_name", target.at("name")}});
  const auto display =
      Request("SpawnObject", {{"object_name", target.at("display_name")},
                              {"asset_path", target.at("asset")},
                              {"pose", pose},
                              {"scale", {1, 1, 1}},
                              {"enable_physics", false}})
          .get<std::string>();
  if (display.empty()) {
    throw std::runtime_error("No display target was spawned");
  }
  if (!Request("SetObjectScale",
               {{"object_name", target.at("name")}, {"scale", {0, 0, 0}}})
           .get<bool>()) {
    throw std::runtime_error("Could not hide trajectory fixture");
  }
  return display;
}

void Client::Subscribe(sdk::Client::TopicCallback callback) {
  const auto topic =
      prefix_ + "/robots/" + bindings_.at("aircraft").get<std::string>() +
      "/sensors/" + bindings_.at("camera").get<std::string>() + "/scene_camera";
  Check(client_->Subscribe(topic.c_str(), std::move(callback)),
        "Camera subscription");
}

void Client::Unsubscribe() {
  Check(client_->UnsubscribeAll(), "Camera unsubscribe");
}

Json Client::Sample() {
  const auto time = Request("GetSimTime", Json::object()).get<std::int64_t>();
  if (time < previous_time_) {
    throw std::runtime_error("Scene clock reset; restart the run");
  }
  previous_time_ = time;
  Json sample = {
      {"sim_ns", time},
      {"traffic", Json::array()},
      {"aircraft",
       Request("GetObjectPose", {{"object_name", bindings_.at("aircraft")}})}};
  for (std::size_t i = 0; i < actors_.size(); ++i) {
    auto& actor = actors_[i];
    const auto pose = UpdateDisplay(actor);
    if (i == 0) {
      sample["target"] = pose;
    } else {
      sample["traffic"].push_back(
          {{"name", actor.target.at("name")}, {"pose", pose}});
    }
  }
  return sample;
}

Json Client::UpdateDisplay(DisplayActor& actor) {
  const auto& target = actor.target;
  const auto& display = actor.display;
  const auto pose =
      Request("GetObjectPose", {{"object_name", target.at("name")}});
  for (const auto* axis : {"x", "y", "z"}) {
    if (!std::isfinite(pose.at("translation").at(axis).get<double>())) {
      throw std::runtime_error("Non-finite target fixture pose");
    }
  }
  const auto& q = pose.at("rotation");
  const double yaw =
      2 * std::atan2(q.at("z").get<double>(), q.at("w").get<double>());
  auto visual_pose = pose;
  const double visual_yaw =
      yaw + target.at("visual_yaw_offset_deg").get<double>() *
                3.14159265358979323846 / 180;
  visual_pose["rotation"] = {{"w", std::cos(visual_yaw / 2)},
                             {"x", 0},
                             {"y", 0},
                             {"z", std::sin(visual_yaw / 2)}};
  // Avoid asking Unreal to move a stationary mesh to its unchanged pose.
  if (actor.previous_pose == visual_pose) {
    return pose;
  }
  // Only the target's display mesh follows a script. PX4 controls the aircraft.
  if (!Request("SetObjectPose", {{"object_name", display},
                                 {"pose", visual_pose},
                                 {"teleport", true}})
           .get<bool>()) {
    throw std::runtime_error("Display target update failed");
  }
  actor.previous_pose = visual_pose;
  return pose;
}
}  // namespace raven::simulation
