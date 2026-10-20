// SPDX-License-Identifier: AGPL-3.0-only
#include "video.h"

#include <gst/app/gstappsrc.h>

#include <ProjectAirSimMessage/image_message.hpp>
#include <json.hpp>
#include <stdexcept>

namespace raven::simulation {
Video::Video(const std::string& host, const std::filesystem::path& directory)
    : frame_log_(directory / "frames.jsonl") {
  if (!frame_log_) throw std::runtime_error("Cannot create frame log");
  gst_init(nullptr, nullptr);
  GError* error = nullptr;
  pipeline_ = gst_parse_launch(
      "appsrc name=frames is-live=true format=time block=false "
      "max-bytes=1382400 leaky-type=downstream "
      "caps=video/x-raw,format=BGR,width=640,height=360,framerate=30/1 "
      "! queue max-size-buffers=2 leaky=downstream ! videoconvert "
      "! video/x-raw,format=I420 ! x264enc tune=zerolatency "
      "speed-preset=ultrafast "
      "bitrate=2000 key-int-max=30 threads=2 ! h264parse ! tee name=encoded "
      "encoded. ! queue ! rtph264pay pt=96 config-interval=1 "
      "! udpsink name=network port=5602 sync=false async=false "
      "encoded. ! queue ! mp4mux faststart=true "
      "! filesink name=record sync=false async=false",
      &error);
  if (error != nullptr || pipeline_ == nullptr) {
    const std::string message =
        error == nullptr ? "No video pipeline" : error->message;
    if (error != nullptr) g_error_free(error);
    if (pipeline_ != nullptr) gst_object_unref(pipeline_);
    pipeline_ = nullptr;
    throw std::runtime_error(message);
  }
  auto* network = gst_bin_get_by_name(GST_BIN(pipeline_), "network");
  auto* record = gst_bin_get_by_name(GST_BIN(pipeline_), "record");
  source_ = gst_bin_get_by_name(GST_BIN(pipeline_), "frames");
  const auto output = (directory / "camera.mp4").string();
  g_object_set(network, "host", host.c_str(), nullptr);
  g_object_set(record, "location", output.c_str(), nullptr);
  gst_object_unref(network);
  gst_object_unref(record);
  if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) ==
      GST_STATE_CHANGE_FAILURE) {
    gst_element_set_state(pipeline_, GST_STATE_NULL);
    gst_object_unref(source_);
    gst_object_unref(pipeline_);
    pipeline_ = nullptr;
    source_ = nullptr;
    throw std::runtime_error("Video pipeline could not start");
  }
}

Video::~Video() {
  // The client is destroyed first, so no camera callback can race finalization.
  if (!finished_ && pipeline_ != nullptr && source_ != nullptr &&
      Frames() > 0) {
    try {
      Finish();
    } catch (const std::exception&) {
      // Preserve the original run error while releasing the recording pipeline.
    }
  }
  if (pipeline_ != nullptr) gst_element_set_state(pipeline_, GST_STATE_NULL);
  if (source_ != nullptr) gst_object_unref(source_);
  if (pipeline_ != nullptr) gst_object_unref(pipeline_);
}

void Video::Receive(const std::string& bytes) noexcept {
  try {
    microsoft::projectairsim::ImageMessage image;
    image.Deserialize(bytes);
    auto& pixels = image.GetPixelVector();
    if (image.GetWidth() != 640 || image.GetHeight() != 360 ||
        image.GetEncoding() != "BGR" || pixels.size() != 640U * 360U * 3U) {
      ++rejected_;
      return;
    }
    auto* clock = gst_element_get_clock(pipeline_);
    if (clock == nullptr) {
      ++rejected_;
      return;
    }
    const auto pts =
        gst_clock_get_time(clock) - gst_element_get_base_time(pipeline_);
    gst_object_unref(clock);
    auto* buffer = gst_buffer_new_allocate(nullptr, pixels.size(), nullptr);
    if (buffer == nullptr) {
      ++rejected_;
      return;
    }
    gst_buffer_fill(buffer, 0, pixels.data(), pixels.size());
    GST_BUFFER_PTS(buffer) = pts;
    GST_BUFFER_DURATION(buffer) = GST_SECOND / 30;
    if (gst_app_src_push_buffer(GST_APP_SRC(source_), buffer) != GST_FLOW_OK) {
      ++rejected_;
      return;
    }
    frame_log_ << nlohmann::json{{"index", frames_.load()},
                                 {"pts_ns", pts},
                                 {"source_sim_ns", image.GetTimestamp()}}
                      .dump()
               << '\n';
    if (!frame_log_) throw std::runtime_error("Frame log failed");
    ++frames_;
  } catch (const std::exception&) {
    ++rejected_;
  }
}

void Video::Finish() {
  gst_app_src_end_of_stream(GST_APP_SRC(source_));
  auto* bus = gst_element_get_bus(pipeline_);
  auto* message = gst_bus_timed_pop_filtered(
      bus, 10 * GST_SECOND,
      static_cast<GstMessageType>(GST_MESSAGE_EOS | GST_MESSAGE_ERROR));
  const bool success =
      message != nullptr && GST_MESSAGE_TYPE(message) == GST_MESSAGE_EOS;
  if (message != nullptr) gst_message_unref(message);
  gst_object_unref(bus);
  if (!success) throw std::runtime_error("Recording did not finish");
  gst_element_set_state(pipeline_, GST_STATE_NULL);
  finished_ = true;
  if (Frames() == 0 || Rejected() != 0)
    throw std::runtime_error("Camera frames missing or rejected");
}
}  // namespace raven::simulation
