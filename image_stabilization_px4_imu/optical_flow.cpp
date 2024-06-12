#include <gst/app/gstappsink.h>
#include <gst/gst.h>

#include <iostream>

static gboolean bus_call(GstBus *bus, GstMessage *msg, gpointer data)
{
    GMainLoop *loop = static_cast<GMainLoop *>(data);
    switch (GST_MESSAGE_TYPE(msg))
    {
    case GST_MESSAGE_EOS:
        g_print("End of stream\n");
        g_main_loop_quit(loop);
        break;
    case GST_MESSAGE_ERROR:
    {
        gchar *debug;
        GError *error;
        gst_message_parse_error(msg, &error, &debug);
        g_free(debug);
        g_printerr("Error: %s\n", error->message);
        g_error_free(error);
        g_main_loop_quit(loop);
        break;
    }
    default:
        break;
    }
    return TRUE;
}

void sink_cb(GstElement *sink, gpointer data)
{
    GstSample *sample = gst_app_sink_pull_sample(GST_APP_SINK(sink));
    GstBuffer *buffer = gst_sample_get_buffer(sample);
    GstClockTime pts = GST_BUFFER_PTS(buffer);
    g_print("PTS: %" GST_TIME_FORMAT "\n", GST_TIME_ARGS(pts));
    gst_sample_unref(sample);
}

int main(int argc, char *argv[])
{
    gst_init(&argc, &argv);
    GMainLoop *loop = g_main_loop_new(NULL, FALSE);

    // Create a pipeline
    GstElement *pipeline = gst_pipeline_new("camera-pipeline");

    // Create elements
    GstElement *src = gst_element_factory_make("v4l2src", "camera-source");
    GstElement *caps = gst_element_factory_make("capsfilter", "camera-caps");
    GstElement *sink = gst_element_factory_make("fakesink", "sink");

    if (!pipeline || !src || !caps || !sink)
    {
        g_printerr("Failed to create elements\n");
        return -1;
    }

    // Set properties
    g_object_set(G_OBJECT(src), "device", "/dev/video2", NULL);
    g_object_set(G_OBJECT(caps), "caps", gst_caps_from_string("video/x-raw, width=640, height=480, framerate=30/1"), NULL);

    // Add elements to the pipeline
    gst_bin_add_many(GST_BIN(pipeline), src, caps, sink, NULL);

    // Link elements
    if (!gst_element_link_many(src, caps, sink, NULL))
    {
        g_printerr("Failed to link elements\n");
        return -1;
    }

    // Attach a message handler to the pipeline bus
    GstBus *bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline));
    gst_bus_add_signal_watch(bus);
    g_signal_connect(bus, "message", G_CALLBACK(bus_call), loop);

    // auto cb = [](GstElement *sink, gpointer data) -> GstFlowReturn {
    //     GstSample *sample = gst_app_sink_pull_sample(GST_APP_SINK(sink));
    //     GstBuffer *buffer = gst_sample_get_buffer(sample);
    //     GstClockTime pts = GST_BUFFER_PTS(buffer);
    //     g_print("PTS: %" GST_TIME_FORMAT "\n", GST_TIME_ARGS(pts));
    //     gst_sample_unref(sample);
    //     return GST_FLOW_OK;
    // };

    // Print PTS for each buffer
    g_signal_connect(sink, "new-sample", G_CALLBACK(sink_cb), NULL);

    // Start the pipeline
    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    // Run the loop
    g_main_loop_run(loop);

    // Clean up
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);
    g_main_loop_unref(loop);

    return 0;
}
