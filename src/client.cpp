// SPDX-License-Identifier: AGPL-3.0-only
#include "client.h"

#include <ProjectAirSimMessage/response_message.hpp>
#include <cmath>
#include <fstream>
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
  if (response.GetErrorCode() != 0)
    throw std::runtime_error(method + " rejected");
  return response.GetResult();
}

void Client::Load(const std::filesystem::path& directory) {
  std::ifstream bindings_file(directory / "bindings.json");
  if (!bindings_file) throw std::runtime_error("Cannot read scene bindings");
  bindings_file >> bindings_;
  Check(world_->Initialize(client_, (directory / "scene.jsonc").string(),
                           directory.string(), 1),
        "Scene loading");
  prefix_ = world_->GetParentTopic();
  actors_.clear();
  const auto& target = bindings_.at("target");
  actors_.push_back({target, Spawn(target)});
  for (const auto& actor : bindings_.value("traffic", Json::array())) {
    actors_.push_back({actor, Spawn(actor)});
  }
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
  if (display.empty())
    throw std::runtime_error("No display target was spawned");
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
  if (time < previous_time_)
    throw std::runtime_error("Scene clock reset; restart the run");
  previous_time_ = time;
  Json sample = {
      {"sim_ns", time},
      {"traffic", Json::array()},
      {"aircraft",
       Request("GetObjectPose", {{"object_name", bindings_.at("aircraft")}})}};
  for (std::size_t i = 0; i < actors_.size(); ++i) {
    const auto& actor = actors_[i];
    const auto pose = UpdateDisplay(actor.target, actor.display);
    if (i == 0)
      sample["target"] = pose;
    else
      sample["traffic"].push_back(
          {{"name", actor.target.at("name")}, {"pose", pose}});
  }
  return sample;
}

Json Client::UpdateDisplay(const Json& target, const std::string& display) {
  const auto pose =
      Request("GetObjectPose", {{"object_name", target.at("name")}});
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
  // Only the target's display mesh follows a script. PX4 controls the aircraft.
  if (!Request("SetObjectPose", {{"object_name", display},
                                 {"pose", visual_pose},
                                 {"teleport", true}})
           .get<bool>()) {
    throw std::runtime_error("Display target update failed");
  }
  return pose;
}
}  // namespace raven::simulation
