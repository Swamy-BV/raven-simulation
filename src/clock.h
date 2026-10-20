#pragma once
// SPDX-License-Identifier: AGPL-3.0-only
// Publish the AirSim clock through the isolated simulation XRCE agent.

#include <ucdr/microcdr.h>
#include <uxr/client/client.h>

#include <array>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <thread>

// PX4's native XRCE library uses this platform hook for transport timeouts.
extern "C" std::uint64_t hrt_absolute_time() {
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

namespace {

constexpr std::uint32_t kClientKey = 0x52434c4b;
constexpr std::size_t kHistory = 8;
constexpr std::size_t kMtu = 512;
constexpr auto kTimeoutMs = 2000;
constexpr auto kDomain = 0;

class ClockBridge {
 public:
  explicit ClockBridge(const char* address) {
    if (!uxr_init_udp_transport(&transport_, UXR_IPv4, address, "8889")) {
      throw std::runtime_error("Clock XRCE transport could not open");
    }
    uxr_init_session(&session_, &transport_.comm, kClientKey);
    if (!uxr_create_session(&session_)) {
      uxr_close_udp_transport(&transport_);
      throw std::runtime_error("Simulation agent is unavailable");
    }
    reliable_ = uxr_create_output_reliable_stream(&session_, output_.data(),
                                                  output_.size(), kHistory);
    uxr_create_input_reliable_stream(&session_, input_.data(), input_.size(),
                                     kHistory);
    best_effort_ = uxr_create_output_best_effort_stream(
        &session_, samples_.data(), samples_.size());
    if (!CreateEntities()) {
      uxr_delete_session(&session_);
      uxr_close_udp_transport(&transport_);
      throw std::runtime_error(
          "Simulation clock DDS entities could not be created");
    }
  }

  ~ClockBridge() {
    uxr_delete_session(&session_);
    uxr_close_udp_transport(&transport_);
  }

  ClockBridge(const ClockBridge&) = delete;
  ClockBridge& operator=(const ClockBridge&) = delete;

  void Send(std::int64_t time_ns) {
    if (time_ns <= 0) return;
    ucdrBuffer buffer;
    if (uxr_prepare_output_stream(&session_, best_effort_, writer_, &buffer,
                                  8) == UXR_INVALID_REQUEST_ID ||
        !ucdr_serialize_int32_t(
            &buffer, static_cast<std::int32_t>(time_ns / 1000000000)) ||
        !ucdr_serialize_uint32_t(
            &buffer, static_cast<std::uint32_t>(time_ns % 1000000000))) {
      throw std::runtime_error("AirSim clock serialization failed");
    }
    uxr_flash_output_streams(&session_);
    uxr_run_session_timeout(&session_, 0);
  }

 private:
  bool CreateEntities() {
    const auto participant = uxr_object_id(1, UXR_PARTICIPANT_ID);
    const auto topic = uxr_object_id(1, UXR_TOPIC_ID);
    const auto publisher = uxr_object_id(1, UXR_PUBLISHER_ID);
    constexpr auto kParticipant =
        "<dds><profiles><transport_descriptors><transport_descriptor>"
        "<transport_id>clock_loopback</transport_id><type>UDPv4</type>"
        "<interfaceWhiteList><address>127.0.0.1</address></interfaceWhiteList>"
        "</transport_descriptor></transport_descriptors></profiles>"
        "<participant><rtps><name>gazebo_clock</"
        "name><useBuiltinTransports>false</useBuiltinTransports>"
        "<userTransports><transport_id>clock_loopback</transport_id></"
        "userTransports>"
        "</rtps></participant></dds>";
    constexpr auto kTopic =
        "<dds><topic><name>rt/clock</"
        "name><dataType>rosgraph_msgs::msg::dds_::Clock_</dataType></topic></"
        "dds>";
    constexpr auto kWriter =
        "<dds><data_writer><topic><kind>NO_KEY</kind><name>rt/clock</name>"
        "<dataType>rosgraph_msgs::msg::dds_::Clock_</dataType></topic>"
        "<qos><reliability><kind>BEST_EFFORT</kind></reliability></qos></"
        "data_writer></dds>";
    std::array<std::uint16_t, 4> requests{
        uxr_buffer_create_participant_xml(&session_, reliable_, participant,
                                          kDomain, kParticipant, UXR_REPLACE),
        uxr_buffer_create_topic_xml(&session_, reliable_, topic, participant,
                                    kTopic, UXR_REPLACE),
        uxr_buffer_create_publisher_xml(&session_, reliable_, publisher,
                                        participant, "", UXR_REPLACE),
        uxr_buffer_create_datawriter_xml(&session_, reliable_, writer_,
                                         publisher, kWriter, UXR_REPLACE)};
    std::array<std::uint8_t, 4> statuses{};
    return uxr_run_session_until_all_status(&session_, kTimeoutMs,
                                            requests.data(), statuses.data(),
                                            requests.size());
  }

  uxrUDPTransport transport_{};
  uxrSession session_{};
  uxrStreamId reliable_{};
  uxrStreamId best_effort_{};
  uxrObjectId writer_ = uxr_object_id(1, UXR_DATAWRITER_ID);
  std::array<std::uint8_t, kMtu * kHistory> output_{};
  std::array<std::uint8_t, kMtu * kHistory> input_{};
  std::array<std::uint8_t, kMtu> samples_{};
};
}  // namespace
