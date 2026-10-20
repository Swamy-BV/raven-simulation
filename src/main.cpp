// SPDX-License-Identifier: AGPL-3.0-only
#include <chrono>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

#include "client.h"
#include "clock.h"
#include "scenario.h"
#include "video.h"

namespace {
volatile std::sig_atomic_t gStop = 0;

void Stop(int) { gStop = 1; }
}  // namespace

int main(int argc, char** argv) {
  using namespace raven::simulation;
  try {
    if (argc == 4 && std::string(argv[1]) == "--validate") {
      ValidateScenario(argv[2], argv[3]);
      std::cout << "Scenario validated\n";
      return 0;
    }
    if (argc != 7) {
      std::cerr
          << "Usage: airsim-bridge <sim-ip> <board-ip> <robot-template.jsonc> "
             "<scenario.jsonc> <run-dir> <seconds>\n";
      return 2;
    }
    const int seconds = std::stoi(argv[6]);
    if (seconds < 1 || seconds > 3600)
      throw std::runtime_error("Duration must be 1..3600 s");
    const auto directory = std::filesystem::absolute(argv[5]);
    // Do not overwrite recordings or regenerate a running scene.
    if (std::filesystem::exists(directory))
      throw std::runtime_error("Run directory already exists");
    std::filesystem::create_directories(directory);
    GenerateScenario(argv[4], argv[3], directory / "config", argv[1]);
    std::signal(SIGINT, Stop);
    std::signal(SIGTERM, Stop);
    Video video(argv[2], directory);
    Client client(argv[1]);
    ClockBridge clock(argv[2]);
    client.Load(directory / "config");
    client.Subscribe([&video](const std::string&, const std::string& bytes) {
      video.Receive(bytes);
    });
    std::ofstream poses(directory / "poses.jsonl");
    if (!poses) throw std::runtime_error("Cannot create pose log");
    const auto start = std::chrono::steady_clock::now();
    auto report = start;
    while (gStop == 0 && std::chrono::steady_clock::now() - start <
                             std::chrono::seconds(seconds)) {
      const auto sample = client.Sample();
      clock.Send(sample.at("sim_ns").get<std::int64_t>());
      poses << sample.dump() << '\n';
      if (!poses) throw std::runtime_error("Pose log failed");
      const auto now = std::chrono::steady_clock::now();
      if (now - report >= std::chrono::seconds(5)) {
        std::cout << "sim_ns=" << sample.at("sim_ns")
                  << " frames=" << video.Frames()
                  << " rejected=" << video.Rejected() << std::endl;
        report = now;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    client.Unsubscribe();
    video.Finish();
    std::cout << "Completed frames=" << video.Frames()
              << " rejected=" << video.Rejected() << '\n';
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
