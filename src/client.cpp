// SPDX-License-Identifier: AGPL-3.0-only
#include "client.h"

#include <ProjectAirSimMessage/response_message.hpp>
#include <cmath>
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
  Check(world_->Initialize(client_, (directory / "scene.jsonc").string(),
                           directory.string(), 1),
        "Scene loading");
  prefix_ = world_->GetParentTopic();
  const auto assets = Request("ListAssets", {{"name", "^Car_01$"}});
  if (!assets.is_array() || assets.empty()) {
    throw std::runtime_error("Neighborhood Car_01 asset is unavailable");
  }
  const auto pose = Request("GetObjectPose", {{"object_name", "TargetCar"}});
  display_car_ = Request("SpawnObject", {{"object_name", "ScenarioSedan"},
                                         {"asset_path", "Car_01"},
                                         {"pose", pose},
                                         {"scale", {1, 1, 1}},
                                         {"enable_physics", false}})
                     .get<std::string>();
  if (display_car_.empty())
    throw std::runtime_error("No display car was spawned");
  if (!Request("SetObjectScale",
               {{"object_name", "TargetCar"}, {"scale", {0, 0, 0}}})
           .get<bool>()) {
    throw std::runtime_error("Could not hide trajectory fixture");
  }
}

void Client::Subscribe(sdk::Client::TopicCallback callback) {
  const auto topic = prefix_ + "/robots/CameraDrone/sensors/Front/scene_camera";
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
  const auto pose = Request("GetObjectPose", {{"object_name", "TargetCar"}});
  const auto& q = pose.at("rotation");
  const double yaw =
      2 * std::atan2(q.at("z").get<double>(), q.at("w").get<double>());
  auto visual_pose = pose;
  const double visual_yaw = yaw - 1.5707963267948966;
  visual_pose["rotation"] = {{"w", std::cos(visual_yaw / 2)},
                             {"x", 0},
                             {"y", 0},
                             {"z", std::sin(visual_yaw / 2)}};
  // Only the car's display mesh follows a script. PX4 controls the aircraft.
  if (!Request("SetObjectPose", {{"object_name", display_car_},
                                 {"pose", visual_pose},
                                 {"teleport", true}})
           .get<bool>()) {
    throw std::runtime_error("Display car update failed");
  }
  return {
      {"sim_ns", time},
      {"car", pose},
      {"aircraft", Request("GetObjectPose", {{"object_name", "CameraDrone"}})}};
}
}  // namespace raven::simulation
