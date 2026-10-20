// SPDX-License-Identifier: AGPL-3.0-only
#include "rtp_time.h"

#include <gst/rtp/gstrtphdrext.h>

#include <stdexcept>

namespace {
constexpr guint64 kNtpUnixOffset = 2208988800ULL * GST_SECOND;
constexpr auto kUri = "urn:ietf:params:rtp-hdrext:ntp-64";

struct CaptureTimeExtension {
  GstRTPHeaderExtension parent;
};

struct CaptureTimeExtensionClass {
  GstRTPHeaderExtensionClass parent;
};
G_DEFINE_TYPE(CaptureTimeExtension, capture_time_extension,
              GST_TYPE_RTP_HEADER_EXTENSION)

GstRTPHeaderExtensionFlags Flags(GstRTPHeaderExtension*) {
  return GST_RTP_HEADER_EXTENSION_ONE_BYTE;
}

gsize Size(GstRTPHeaderExtension*, const GstBuffer*) { return 8; }

gssize Write(GstRTPHeaderExtension*, const GstBuffer* input,
             GstRTPHeaderExtensionFlags, GstBuffer*, guint8* data, gsize size) {
  auto* caps = gst_caps_new_empty_simple("timestamp/x-ntp");
  const auto* meta = gst_buffer_get_reference_timestamp_meta(
      const_cast<GstBuffer*>(input), caps);
  gst_caps_unref(caps);
  if (!meta || size < 8) return 0;
  const auto ntp =
      gst_util_uint64_scale(meta->timestamp, 1ULL << 32, GST_SECOND);
  return gst_rtp_hdrext_set_ntp_64(data, size, ntp) ? 8 : -1;
}

void capture_time_extension_class_init(CaptureTimeExtensionClass* klass) {
  auto* base = GST_RTP_HEADER_EXTENSION_CLASS(klass);
  base->get_supported_flags = Flags;
  base->get_max_size = Size;
  base->write = Write;
  gst_rtp_header_extension_class_set_uri(base, kUri);
  gst_element_class_set_static_metadata(
      GST_ELEMENT_CLASS(klass), "Capture timestamp sender",
      GST_RTP_HDREXT_ELEMENT_CLASS,
      "RFC6051 capture timestamp writer for GStreamer 1.20 hosts", "Raven");
}

void capture_time_extension_init(CaptureTimeExtension*) {}
}  // namespace

namespace raven::simulation {
void ConfigureCaptureTime(GstElement* payloader) {
  // Prefer the upstream implementation on newer hosts. The small write-only
  // adapter supplies the same standard extension on
  // Ubuntu 22.04/GStreamer 1.20.
  auto* extension = gst_element_factory_make("rtphdrextntp64", nullptr);
  if (!extension)
    extension =
        GST_ELEMENT(g_object_new(capture_time_extension_get_type(), nullptr));
  gst_rtp_header_extension_set_id(GST_RTP_HEADER_EXTENSION(extension), 1);
  g_signal_emit_by_name(payloader, "add-extension", extension);
  gst_object_unref(extension);
}

void AttachCaptureTime(GstBuffer* buffer, std::int64_t source_ns) {
  if (source_ns <= 0)
    throw std::runtime_error("Invalid camera acquisition time");
  auto* caps = gst_caps_new_empty_simple("timestamp/x-ntp");
  const auto* meta = gst_buffer_add_reference_timestamp_meta(
      buffer, caps, kNtpUnixOffset + static_cast<guint64>(source_ns),
      GST_CLOCK_TIME_NONE);
  gst_caps_unref(caps);
  if (!meta) throw std::runtime_error("Cannot attach camera acquisition time");
}
}  // namespace raven::simulation
