// SPDX-License-Identifier: AGPL-3.0-only
#include "client.h"

#include <ProjectAirSimMessage/response_message.hpp>
#include <algorithm>
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
  const auto start_ns =
      Request("GetSimTime", Json::object()).get<std::int64_t>();
  const auto add = [&](const Json& target, const std::filesystem::path& route,
                       const bool waiting) {
    DisplayActor actor{target, {}, {}, std::nullopt, start_ns, waiting};
    if (bindings_.value("real_time", false)) {
      std::ifstream file(route);
      if (!file) {
        throw std::runtime_error("Cannot read fixture trajectory");
      }
      Json trajectory;
      file >> trajectory;
      std::vector<MotionPoint> points;
      for (std::size_t i = 0; i < trajectory.at("time_sec").size(); ++i) {
        points.push_back({trajectory.at("time_sec").at(i).get<double>(),
                          {trajectory.at("pose_x").at(i).get<double>(),
                           trajectory.at("pose_y").at(i).get<double>(),
                           trajectory.at("pose_z").at(i).get<double>()},
                          trajectory.at("pose_yaw").at(i).get<double>()});
      }
      actor.motion.emplace(std::move(points));
    }
    actor.display = Spawn(actor);
    actors_.push_back(std::move(actor));
  };
  const auto& target = bindings_.at("target");
  if (!target.is_null()) {
    add(target, directory / "trajectory.json",
        bindings_.value("target_start_on_ready", false));
  }
  std::size_t traffic_index = 1;
  for (const auto& actor : bindings_.value("traffic", Json::array())) {
    add(actor,
        directory /
            ("traffic-" + std::to_string(traffic_index++) + "-trajectory.json"),
        bindings_.value("traffic_start_on_ready", false));
  }
}

void Client::StartTarget() {
  const auto stamp = Request("GetSimTime", Json::object()).get<std::int64_t>();
  bool started = false;
  for (auto& actor : actors_) {
    if (actor.waiting && actor.motion) {
      actor.motion_start_ns = stamp;
      actor.waiting = false;
      started = true;
    }
  }
  if (!started) {
    throw std::runtime_error("No fixture is waiting to start");
  }
}

Json Client::FixturePose(const DisplayActor& actor,
                         const std::int64_t time_ns) {
  if (!actor.motion) {
    return Request("GetObjectPose", {{"object_name", actor.target.at("name")}});
  }
  // Subtract integer timestamps before converting to seconds: float uptime
  // loses 62.5 ms at eight days. Never feed this fixture state to guidance.
  const auto elapsed =
      actor.waiting
          ? 0.0
          : static_cast<double>(time_ns - actor.motion_start_ns) / 1000000000.0;
  const auto point = actor.motion->At(std::max(0.0, elapsed));
  return {{"frame_id", "DEFAULT_ID"},
          {"translation",
           {{"x", point.position[0]},
            {"y", point.position[1]},
            {"z", point.position[2]}}},
          {"rotation",
           {{"w", std::cos(point.yaw / 2)},
            {"x", 0},
            {"y", 0},
            {"z", std::sin(point.yaw / 2)}}}};
}

std::string Client::Spawn(const DisplayActor& actor) {
  const auto& target = actor.target;
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
  const auto pose = FixturePose(actor, actor.motion_start_ns);
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
  if (!actor.motion &&
      !Request("SetObjectScale",
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
    const auto pose = UpdateDisplay(actor, time);
    if (i == 0) {
      sample["target"] = pose;
    } else {
      sample["traffic"].push_back(
          {{"name", actor.target.at("name")}, {"pose", pose}});
    }
  }
  sample["fixture_pose_source"] = bindings_.value("real_time", false)
                                      ? "elapsed-time display command"
                                      : "simulator environment actor";
  return sample;
}

Json Client::UpdateDisplay(DisplayActor& actor, const std::int64_t time_ns) {
  const auto& target = actor.target;
  const auto& display = actor.display;
  const auto pose = FixturePose(actor, time_ns);
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
