// SPDX-License-Identifier: AGPL-3.0-only
#include <chrono>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
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

class OutputLog {
 public:
  explicit OutputLog(const std::filesystem::path& path) : file_(path) {
    if (!file_) {
      throw std::runtime_error("Cannot open bridge log");
    }
    output_ = std::cout.rdbuf(file_.rdbuf());
    error_ = std::cerr.rdbuf(file_.rdbuf());
  }

  ~OutputLog() {
    std::cout.rdbuf(output_);
    std::cerr.rdbuf(error_);
  }

  OutputLog(const OutputLog&) = delete;
  OutputLog& operator=(const OutputLog&) = delete;

 private:
  std::ofstream file_;
  std::streambuf* output_ = nullptr;
  std::streambuf* error_ = nullptr;
};
}  // namespace

int main(int argc, char** argv) {
  using namespace raven::simulation;
  std::unique_ptr<OutputLog> output_log;
  try {
    if (argc == 4 && std::string(argv[1]) == "--validate") {
      ValidateScenario(argv[2], argv[3]);
      std::cout << "Scenario validated\n";
      return 0;
    }
    if (argc < 7) {
      std::cerr
          << "Usage: airsim-bridge <sim-ip> <board-ip> <robot-template.jsonc> "
             "<scenario.jsonc> <run-dir> <seconds> [--log-to-run] "
             "[--serial-port <device>]\n";
      return 2;
    }
    bool log_to_run = false;
    std::string serial_port;
    for (int index = 7; index < argc; ++index) {
      const std::string option(argv[index]);
      if (option == "--log-to-run" && !log_to_run) {
        log_to_run = true;
      } else if (option == "--serial-port" && serial_port.empty() &&
                 index + 1 < argc) {
        serial_port = argv[++index];
        if (serial_port.empty()) {
          throw std::runtime_error("Serial device must not be empty");
        }
      } else {
        throw std::runtime_error("Unknown or repeated option: " + option);
      }
    }
    const int seconds = std::stoi(argv[6]);
    if (seconds < 1 || seconds > 3600) {
      throw std::runtime_error("Duration must be 1..3600 s");
    }
    const auto directory = std::filesystem::absolute(argv[5]);
    // Do not overwrite recordings or regenerate a running scene.
    if (std::filesystem::exists(directory)) {
      throw std::runtime_error("Run directory already exists");
    }
    std::filesystem::create_directories(directory);
    if (log_to_run) {
      output_log = std::make_unique<OutputLog>(directory / "bridge.log");
    }
    const auto runtime = GenerateScenario(
        argv[4], argv[3], directory / "config", argv[1], argv[2], serial_port);
    std::signal(SIGINT, Stop);
    std::signal(SIGTERM, Stop);
    std::unique_ptr<ClockBridge> clock;
    if (runtime.controller == FlightController::kSitl) {
      clock = std::make_unique<ClockBridge>(argv[2], directory);
    }
    const auto video_host =
        runtime.stream_video_to_board ? argv[2] : "127.0.0.1";
    // Real-time AirSim acquisition uses the host steady clock. Translate its
    // epoch once to UTC for the hardware-clock ROS graph; preserve elapsed
    // time. SITL retains its own simulated clock without translation.
    const auto capture_clock_offset_ns =
        runtime.controller == FlightController::kHitl
            ? std::chrono::duration_cast<std::chrono::nanoseconds>(
                  std::chrono::system_clock::now().time_since_epoch())
                      .count() -
                  std::chrono::duration_cast<std::chrono::nanoseconds>(
                      std::chrono::steady_clock::now().time_since_epoch())
                      .count()
            : 0;
    Video video(
        video_host, directory,
        [&clock](std::int64_t stamp) {
          if (clock) {
            clock->Send(stamp);
          }
        },
        capture_clock_offset_ns, runtime.render_width, runtime.render_height);
    Client client(argv[1]);
    client.Load(directory / "config");
    client.Subscribe([&video](const std::string&, const std::string& bytes) {
      video.Receive(bytes);
    });
    std::ofstream poses(directory / "poses.jsonl");
    if (!poses) {
      throw std::runtime_error("Cannot create pose log");
    }
    const auto start = std::chrono::steady_clock::now();
    auto report = start;
    bool target_started = false;
    // A stop marker also permits graceful shutdown from hidden Windows runs.
    while (gStop == 0 && !std::filesystem::exists(directory / "stop") &&
           std::chrono::steady_clock::now() - start <
               std::chrono::seconds(seconds)) {
      if (!target_started &&
          std::filesystem::exists(directory / "start-target")) {
        client.StartTarget();
        target_started = true;
        std::cout << "Target route started after readiness signal" << std::endl;
      }
      auto sample = client.Sample();
      sample["sample_steady_ns"] =
          std::chrono::duration_cast<std::chrono::nanoseconds>(
              std::chrono::steady_clock::now().time_since_epoch())
              .count();
      if (clock) {
        clock->Send(sample.at("sim_ns").get<std::int64_t>());
      }
      poses << sample.dump() << '\n';
      if (!poses) {
        throw std::runtime_error("Pose log failed");
      }
      const auto now = std::chrono::steady_clock::now();
      if (now - report >= std::chrono::seconds(5)) {
        std::cout << "sim_ns=" << sample.at("sim_ns")
                  << " frames=" << video.Frames()
                  << " rejected=" << video.Rejected() << std::endl;
        report = now;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(
          runtime.controller == FlightController::kHitl ? 2 : 20));
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
