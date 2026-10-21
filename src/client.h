// SPDX-License-Identifier: AGPL-3.0-only
#pragma once
#include <ProjectAirsimClient/ProjectAirsimClient.h>

#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace raven::simulation {
namespace sdk = microsoft::projectairsim::client;
using Json = nlohmann::json;
void Check(sdk::Status status, const std::string& operation);

class Client {
 public:
  explicit Client(const std::string& host);
  ~Client();
  Client(const Client&) = delete;
  Client& operator=(const Client&) = delete;
  void Load(const std::filesystem::path& directory);
  void Subscribe(sdk::Client::TopicCallback callback);
  void Unsubscribe();
  Json Sample();
  // Start only the target fixture; aircraft motion remains owned by PX4.
  void StartTarget();

 private:
  struct DisplayActor {
    Json target;
    std::string display;
    Json previous_pose;
  };

  Json Request(const std::string& method, const Json& parameters);
  std::string Spawn(const Json& target);
  void StartRoute(const std::filesystem::path& trajectory,
                  const std::string& actor, double scene_start_seconds,
                  bool stationary = false);
  Json UpdateDisplay(DisplayActor& actor);
  std::shared_ptr<sdk::Client> client_;
  std::shared_ptr<sdk::World> world_;
  std::string prefix_;
  Json bindings_;
  std::filesystem::path pending_target_route_;
  std::vector<std::pair<std::filesystem::path, std::string>>
      pending_traffic_routes_;

  std::vector<DisplayActor> actors_;
  std::int64_t previous_time_ = 0;
};
}  // namespace raven::simulation
