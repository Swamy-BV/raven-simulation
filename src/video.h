// SPDX-License-Identifier: AGPL-3.0-only
#pragma once
#include <gst/gst.h>

#include <atomic>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <string>

namespace raven::simulation {
class Video {
 public:
  Video(const std::string& host, const std::filesystem::path& directory,
        std::function<void(std::int64_t)> source_clock,
        std::int64_t capture_clock_offset_ns);
  ~Video();
  Video(const Video&) = delete;
  Video& operator=(const Video&) = delete;
  void Receive(const std::string& bytes) noexcept;
  void Finish();

  unsigned Frames() const { return frames_.load(); }

  unsigned Rejected() const { return rejected_.load(); }

 private:
  static GstPadProbeReturn Encoded(GstPad*, GstPadProbeInfo* info,
                                   gpointer data);
  GstElement* pipeline_ = nullptr;
  GstElement* source_ = nullptr;
  std::ofstream frame_log_;
  std::ofstream encoded_log_;
  std::function<void(std::int64_t)> source_clock_;
  const std::int64_t capture_clock_offset_ns_;
  std::atomic<unsigned> frames_{0};
  std::atomic<unsigned> rejected_{0};
  bool finished_ = false;
};
}  // namespace raven::simulation
