// SPDX-License-Identifier: AGPL-3.0-only
#pragma once
#include <gst/gst.h>

#include <cstdint>

namespace raven::simulation {
// RFC6051 NTP-64 capture timestamps. ROS simulation time uses the Unix epoch
// representation; the receiver must explicitly opt into this clock domain.
void ConfigureCaptureTime(GstElement* payloader);
void AttachCaptureTime(GstBuffer* buffer, std::int64_t source_ns);
}  // namespace raven::simulation
