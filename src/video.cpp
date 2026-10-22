// SPDX-License-Identifier: AGPL-3.0-only
#include "video.h"

#include <gst/app/gstappsrc.h>

#include <ProjectAirSimMessage/image_message.hpp>
#include <chrono>
#include <json.hpp>
#include <memory>
#include <stdexcept>
#include <utility>

#include "rtp_time.h"

namespace raven::simulation {
Video::Video(const std::string& host, const std::filesystem::path& directory,
             std::function<void(std::int64_t)> source_clock,
             std::int64_t capture_clock_offset_ns, const int width,
             const int height, const int capture_rate_hz)
    : frame_log_(directory / "frames.jsonl"),
      encoded_log_(directory / "encoded.jsonl"),
      source_clock_(std::move(source_clock)),
      capture_clock_offset_ns_(capture_clock_offset_ns),
      width_(width),
      height_(height),
      capture_rate_hz_(capture_rate_hz) {
  if (!frame_log_ || !encoded_log_) {
    throw std::runtime_error("Cannot create frame logs");
  }
  gst_init(nullptr, nullptr);
  GError* error = nullptr;
#ifdef _WIN32
  constexpr auto kEncoder = "nvh264enc";
  constexpr auto kConvert =
      "cudaupload ! cudaconvertscale ! "
      "video/x-raw(memory:CUDAMemory),format=NV12";
  constexpr auto kScale =
      "cudascale ! "
      "video/x-raw(memory:CUDAMemory),format=NV12,width=640,height=360 ";
  constexpr auto kEncode =
      "nvh264enc preset=p4 tune=ultra-low-latency rc-mode=cbr "
      "gop-size=30 bframes=0 zerolatency=true ";
#else
  constexpr auto kEncoder = "x264enc";
  constexpr auto kConvert = "videoconvert ! video/x-raw,format=NV12";
  constexpr auto kScale =
      "videoscale ! video/x-raw,format=NV12,width=640,height=360 ";
  constexpr auto kEncode =
      "x264enc tune=zerolatency speed-preset=ultrafast "
      "key-int-max=30 threads=2 ";
#endif
  // Archive the rendered HD image. Perception receives the same acquisition
  // downscaled to its calibrated 640x360 grid; CUDA handles conversion/resize.
  const std::string pipeline =
      "appsrc name=frames is-live=true format=time block=false max-bytes=" +
      std::to_string(static_cast<std::size_t>(width) * height * 3U * 2U) +
      " leaky-type=downstream caps=video/x-raw,format=BGR,width=" +
      std::to_string(width) + ",height=" + std::to_string(height) +
      ",framerate=" + std::to_string(capture_rate_hz) +
      "/1 ! queue max-size-buffers=2 leaky=downstream ! " + kConvert +
      ",width=" + std::to_string(width) + ",height=" + std::to_string(height) +
      " ! tee name=raw "
      "raw. ! queue max-size-buffers=2 leaky=downstream ! " +
      kEncode +
      "bitrate=12000 ! h264parse name=archive ! mp4mux faststart=true "
      "! filesink name=record sync=false async=false "
      "raw. ! queue max-size-buffers=2 leaky=downstream ! " +
      kScale + " ! " + kEncode +
      "bitrate=6000 ! h264parse ! "
      "rtph264pay name=rtp pt=96 config-interval=1 "
      "! udpsink name=network port=5602 sync=false async=false";
  pipeline_ = gst_parse_launch(pipeline.c_str(), &error);
  if (error != nullptr || pipeline_ == nullptr) {
    const std::string message =
        error == nullptr ? "No video pipeline" : error->message;
    if (error != nullptr) {
      g_error_free(error);
    }
    if (pipeline_ != nullptr) {
      gst_object_unref(pipeline_);
    }
    pipeline_ = nullptr;
    throw std::runtime_error(message);
  }
  std::ofstream encoder_log(directory / "encoder.json");
  if (!encoder_log ||
      !(encoder_log << nlohmann::json{{"factory", kEncoder},
                                      {"gstreamer", gst_version_string()},
                                      {"render_width", width},
                                      {"render_height", height},
                                      {"requested_capture_rate_hz",
                                       capture_rate_hz},
                                      {"stream_width", 640},
                                      {"stream_height", 360},
                                      {"archive_bitrate_kbps", 12000},
                                      {"stream_bitrate_kbps", 6000}}
                           .dump(2)
                    << '\n')) {
    gst_object_unref(pipeline_);
    pipeline_ = nullptr;
    throw std::runtime_error("Cannot write encoder provenance");
  }
  auto* network = gst_bin_get_by_name(GST_BIN(pipeline_), "network");
  auto* record = gst_bin_get_by_name(GST_BIN(pipeline_), "record");
  auto* payloader = gst_bin_get_by_name(GST_BIN(pipeline_), "rtp");
  auto* archive = gst_bin_get_by_name(GST_BIN(pipeline_), "archive");
  auto* encoded_pad = gst_element_get_static_pad(archive, "src");
  gst_object_unref(archive);
  gst_pad_add_probe(encoded_pad, GST_PAD_PROBE_TYPE_BUFFER, Encoded, this,
                    nullptr);
  gst_object_unref(encoded_pad);
  ConfigureCaptureTime(payloader);
  gst_object_unref(payloader);
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
  if (pipeline_ != nullptr) {
    gst_element_set_state(pipeline_, GST_STATE_NULL);
  }
  if (source_ != nullptr) {
    gst_object_unref(source_);
  }
  if (pipeline_ != nullptr) {
    gst_object_unref(pipeline_);
  }
}

void Video::Receive(const std::string& bytes) noexcept {
  const auto received = std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::steady_clock::now().time_since_epoch())
                            .count();
  try {
    microsoft::projectairsim::ImageMessage image;
    image.Deserialize(bytes);
    auto& pixels = image.GetPixelVector();
    if (image.GetWidth() != static_cast<unsigned>(width_) ||
        image.GetHeight() != static_cast<unsigned>(height_) ||
        image.GetEncoding() != "BGR" ||
        pixels.size() != static_cast<std::size_t>(width_) * height_ * 3U) {
      ++rejected_;
      return;
    }
    // Publish the known acquisition time before its video can arrive. The
    // clock bridge serializes this with polled simulator time and never
    // regresses.
    source_clock_(image.GetTimestamp());
    auto* clock = gst_element_get_clock(pipeline_);
    if (clock == nullptr) {
      ++rejected_;
      return;
    }
    if (first_source_ns_ < 0) {
      first_source_ns_ = image.GetTimestamp();
      first_pts_ =
          gst_clock_get_time(clock) - gst_element_get_base_time(pipeline_);
    }
    const auto pts = first_pts_ + static_cast<GstClockTime>(
                                      image.GetTimestamp() - first_source_ns_);
    gst_object_unref(clock);
    std::unique_ptr<GstBuffer, decltype(&gst_buffer_unref)> buffer(
        gst_buffer_new_allocate(nullptr, pixels.size(), nullptr),
        gst_buffer_unref);
    if (buffer == nullptr) {
      ++rejected_;
      return;
    }
    gst_buffer_fill(buffer.get(), 0, pixels.data(), pixels.size());
    const auto capture_ns = image.GetTimestamp() + capture_clock_offset_ns_;
    AttachCaptureTime(buffer.get(), capture_ns);
    GST_BUFFER_PTS(buffer.get()) = pts;
    GST_BUFFER_DURATION(buffer.get()) = GST_SECOND / capture_rate_hz_;
    if (gst_app_src_push_buffer(GST_APP_SRC(source_), buffer.release()) !=
        GST_FLOW_OK) {
      ++rejected_;
      return;
    }
    frame_log_ << nlohmann::json{{"index", frames_.load()},
                                 {"pts_ns", pts},
                                 {"source_sim_ns", image.GetTimestamp()},
                                 {"capture_ns", capture_ns},
                                 {"receive_steady_ns", received},
                                 {"queued_bytes",
                                  gst_app_src_get_current_level_bytes(
                                      GST_APP_SRC(source_))}}
                      .dump()
               << '\n';
    if (!frame_log_) {
      throw std::runtime_error("Frame log failed");
    }
    ++frames_;
  } catch (const std::exception&) {
    ++rejected_;
  }
}

GstPadProbeReturn Video::Encoded(GstPad*, GstPadProbeInfo* info,
                                 gpointer data) {
  auto& self = *static_cast<Video*>(data);
  try {
    auto* caps = gst_caps_new_empty_simple("timestamp/x-ntp");
    auto* buffer = GST_PAD_PROBE_INFO_BUFFER(info);
    const auto* origin = gst_buffer_get_reference_timestamp_meta(buffer, caps);
    gst_caps_unref(caps);
    if (!origin || origin->timestamp < 2208988800ULL * GST_SECOND) {
      throw std::runtime_error("Encoded frame lost acquisition identity");
    }
    const auto time = std::chrono::duration_cast<std::chrono::nanoseconds>(
                          std::chrono::steady_clock::now().time_since_epoch())
                          .count();
    const auto capture_ns = static_cast<std::int64_t>(
        origin->timestamp - 2208988800ULL * GST_SECOND);
    self.encoded_log_ << nlohmann::json{{"source_sim_ns",
                                         capture_ns -
                                             self.capture_clock_offset_ns_},
                                        {"capture_ns", capture_ns},
                                        {"encoded_steady_ns", time},
                                        {"bytes", gst_buffer_get_size(buffer)}}
                             .dump()
                      << '\n';
    if (!self.encoded_log_) {
      throw std::runtime_error("Encoded frame log failed");
    }
    return GST_PAD_PROBE_OK;
  } catch (const std::exception&) {
    ++self.rejected_;
    return GST_PAD_PROBE_DROP;
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
  if (message != nullptr) {
    gst_message_unref(message);
  }
  gst_object_unref(bus);
  if (!success) {
    throw std::runtime_error("Recording did not finish");
  }
  gst_element_set_state(pipeline_, GST_STATE_NULL);
  finished_ = true;
  if (Frames() == 0 || Rejected() != 0) {
    throw std::runtime_error("Camera frames missing or rejected");
  }
}
}  // namespace raven::simulation
