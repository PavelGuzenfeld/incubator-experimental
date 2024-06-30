#include "video_pipeline.hpp"
#include <gst/app/gstappsink.h>
#include <gst/gst.h>
#include <stdexcept>
#include <chrono>

struct GstPipeline_Private
{
    FrameCallback frame_callback_;
    GstElement* pipeline_ = nullptr;
    GstElement* sink_ = nullptr;
    GstClockTime base_time_ = 0;
};

static GstPipeline_Private gst_pipeline_;

static GstFlowReturn on_new_sample(GstElement*)
{
    auto sample = gst_app_sink_pull_sample(GST_APP_SINK(gst_pipeline_.sink_));
    if (!sample)
    {
        return GST_FLOW_ERROR;
    }

    GstBuffer* buffer = gst_sample_get_buffer(sample);
    GstMapInfo map;
    gst_buffer_map(buffer, &map, GST_MAP_READ);

    GstClockTime buffer_pts = GST_BUFFER_PTS(buffer);
    GstClockTime monotonic_ts = buffer_pts + gst_pipeline_.base_time_;
    auto buffer_time = std::chrono::steady_clock::time_point(std::chrono::nanoseconds(monotonic_ts));

    gst_pipeline_.frame_callback_(buffer_time.time_since_epoch().count(), map.size, "i420", std::span<unsigned char>(map.data, map.size));

    gst_buffer_unmap(buffer, &map);
    gst_sample_unref(sample);

    return GST_FLOW_OK;
}


VideoPipeline::VideoPipeline(std::string_view pipeline_description, FrameCallback frame_callback)
{
    // Initialize GStreamer
    gst_init(nullptr, nullptr);

    gst_pipeline_.frame_callback_ = frame_callback;

    // Create the GStreamer pipeline
    gst_pipeline_.pipeline_ = gst_parse_launch(pipeline_description.data(), nullptr);
    if (!gst_pipeline_.pipeline_)
    {
        throw std::runtime_error("Failed to create GStreamer pipeline!");
    }

    // Get the appsink element
    gst_pipeline_.sink_ = gst_bin_get_by_name(GST_BIN(gst_pipeline_.pipeline_), "appsink");
    if (!gst_pipeline_.sink_)
    {
        gst_object_unref(gst_pipeline_.pipeline_);
        throw std::runtime_error("Failed to get appsink element from GStreamer pipeline!");
    }

    g_object_set(G_OBJECT(gst_pipeline_.sink_), "emit-signals", TRUE, "sync", FALSE, nullptr);
    g_signal_connect(gst_pipeline_.sink_, "new-sample", G_CALLBACK(on_new_sample), nullptr);

    // Set pipeline clock to system monotonic clock
    GstClock *clock = gst_system_clock_obtain();
    g_object_set(clock, "clock-type", GST_CLOCK_TYPE_MONOTONIC, nullptr);
    gst_pipeline_use_clock(GST_PIPELINE(gst_pipeline_.pipeline_), clock);
    gst_object_unref(clock);
}

VideoPipeline::~VideoPipeline()
{
    stop();
    if (gst_pipeline_.sink_)
    {
        gst_object_unref(gst_pipeline_.sink_);
    }
    if (gst_pipeline_.pipeline_)
    {
        gst_object_unref(gst_pipeline_.pipeline_);
    }
}

bool VideoPipeline::start()
{
    GstStateChangeReturn ret = gst_element_set_state(gst_pipeline_.pipeline_, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE)
    {
        return false;
    }

    // Get the base time of the source element
    GstElement *vsrc = gst_bin_get_by_name(GST_BIN(gst_pipeline_.pipeline_), "v4l2src0");
    if (vsrc)
    {
        gst_pipeline_.base_time_ = gst_element_get_base_time(vsrc);
        gst_object_unref(vsrc);
    }
    else
    {
        gst_pipeline_.base_time_ = gst_element_get_base_time(GST_ELEMENT(gst_pipeline_.pipeline_));
    }

    return true;
}

void VideoPipeline::stop()
{
    gst_element_set_state(gst_pipeline_.pipeline_, GST_STATE_NULL);
}

