// modified work from audio_play/src/audio_play.cpp

#include "audio_video_recorder/audio_video_recorder.h"

#include <boost/thread.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>

#include "audio_common_msgs/AudioData.h"
#include "sensor_msgs/Image.h"

namespace audio_video_recorder
{

  void AudioVideoRecorder::initialize()
  {
    GstCaps *caps;
    GstPad *audio_src_pad, *audio_mux_pad, *g_audio_mux_pad;
    GstPad *video_src_pad, *video_mux_pad, *g_video_mux_pad;

    std::string audio_format;
    std::string device;
    std::string file_location;
    std::string sample_format;
    int channels;
    int depth;
    int sample_rate;
    bool do_timestamp;

    int queue_size;
    bool use_async;

    // The destination of the audio
    ros::param::param<std::string>("~file_location", file_location, "/tmp/test.mp4");
    ros::param::param<std::string>("~device", device, std::string());
    ros::param::param<bool>("~do_timestamp", do_timestamp, true);
    ros::param::param<std::string>("~audio_format", audio_format, "mp3");
    ros::param::param<int>("~channels", channels, 1);
    ros::param::param<int>("~depth", depth, 16);
    ros::param::param<int>("~sample_rate", sample_rate, 16000);
    ros::param::param<std::string>("~sample_format", sample_format, "S16LE");
    ros::param::param<int>("~queue_size", queue_size, 10);

    _nh.reset (new ros::NodeHandle ("~"));
    _sub_image = _nh->subscribe("input/image", queue_size, &AudioVideoRecorder::callbackImage, this);
    _sub_audio = _nh->subscribe("input/audio", queue_size, &AudioVideoRecorder::callbackAudio, this);

    _loop = g_main_loop_new(NULL, false);
    _pipeline = gst_pipeline_new("app_pipeline");
    _audio_source = gst_element_factory_make("appsrc", "audio_source");
    g_object_set(G_OBJECT(_audio_source), "format", GST_FORMAT_TIME, NULL);
    g_object_set(G_OBJECT(_audio_source), "do-timestamp", (do_timestamp) ? TRUE : FALSE, NULL);

    // test video source
    _video_source = gst_element_factory_make("videotestsrc", "video_source");
    _video_filter = gst_element_factory_make("x264enc", "video_filter");
    video_src_pad = gst_element_get_static_pad(_video_filter, "src");
    _mux = gst_element_factory_make("mp4mux", "mux");
    audio_mux_pad = gst_element_get_request_pad(_mux, "audio_%u");
    video_mux_pad = gst_element_get_request_pad(_mux, "video_%u");

    _sink = gst_element_factory_make("filesink", "sink");
    g_object_set(G_OBJECT(_sink), "location", file_location.c_str(), NULL);

    _bin = gst_bin_new("bin");
    g_object_set(G_OBJECT(_bin), "message-forward", TRUE, NULL);
    gst_bin_add_many(GST_BIN(_bin), _mux, _sink, NULL);
    if (gst_element_link_many(_mux, _sink, NULL) != TRUE)
    {
      ROS_ERROR("failed to link gstreamer");
      return;
    }
    gst_bin_add(GST_BIN(_pipeline), _bin);

    g_audio_mux_pad = gst_ghost_pad_new("audio_sink", audio_mux_pad);
    g_video_mux_pad = gst_ghost_pad_new("video_sink", video_mux_pad);
    gst_element_add_pad(_bin, g_audio_mux_pad);
    gst_element_add_pad(_bin, g_video_mux_pad);
    gst_object_unref(GST_OBJECT(audio_mux_pad));
    gst_object_unref(GST_OBJECT(video_mux_pad));
    gst_element_sync_state_with_parent(_bin);

    ROS_INFO("Saving file to %s", file_location.c_str());
    if (audio_format == "mp3")
    {
      audio_src_pad = gst_element_get_static_pad(_audio_source, "src");
      // gst_bin_add_many(GST_BIN(_pipeline), _audio_source, _mux, _sink, NULL);
      // gst_bin_add_many(GST_BIN(_pipeline), _video_source, _video_filter, _mux, _sink, NULL);
      gst_bin_add_many(GST_BIN(_pipeline), _audio_source, _video_source, _video_filter, NULL);
      if (gst_element_link_many(_video_source, _video_filter, NULL) != TRUE ||
          gst_pad_link(audio_src_pad, g_audio_mux_pad) != GST_PAD_LINK_OK ||
          gst_pad_link(video_src_pad, g_video_mux_pad) != GST_PAD_LINK_OK)
      {
        ROS_ERROR("failed to link gstreamer");
        return;
      }
    }
    else if (audio_format == "wave")
    {
      caps = gst_caps_new_simple(
          "audio/x-raw",
          "format", G_TYPE_STRING, sample_format.c_str(),
          "rate", G_TYPE_INT, sample_rate,
          "channels", G_TYPE_INT, channels,
          "width",    G_TYPE_INT, depth,
          "depth",    G_TYPE_INT, depth,
          "signed",   G_TYPE_BOOLEAN, TRUE,
          "layout", G_TYPE_STRING, "interleaved",
          NULL);
      g_object_set( G_OBJECT(_audio_source), "caps", caps, NULL);
      g_object_set (G_OBJECT (_audio_source), "format", GST_FORMAT_TIME, NULL);
      gst_caps_unref(caps);
      _audio_filter = gst_element_factory_make("lamemp3enc", "audio_filter");
      audio_src_pad = gst_element_get_static_pad(_audio_filter, "src");

      gst_bin_add_many(GST_BIN(_pipeline), _audio_source, _audio_filter, _video_source, _video_filter, NULL);
      if (gst_element_link_many(_audio_source, _audio_filter, NULL) != TRUE ||
          gst_element_link_many(_video_source, _video_filter, NULL) != TRUE ||
          gst_pad_link(audio_src_pad, g_audio_mux_pad) != GST_PAD_LINK_OK ||
          gst_pad_link(video_src_pad, g_video_mux_pad) != GST_PAD_LINK_OK)
      {
        ROS_ERROR("failed to link gstreamer");
        return;
      }
    }
    else
    {
      ROS_ERROR("Unsupported format: %s", audio_format.c_str());
    }
    gst_element_set_state(GST_ELEMENT(_pipeline), GST_STATE_PLAYING);
    _gst_thread = boost::thread( boost::bind(g_main_loop_run, _loop) );
  }

  void AudioVideoRecorder::callbackImage(const sensor_msgs::ImageConstPtr &image_msg)
  {
    return;
  }

  void AudioVideoRecorder::callbackAudio(const audio_common_msgs::AudioDataConstPtr &audio_msg)
  {
    GstBuffer *buffer = gst_buffer_new_and_alloc(audio_msg->data.size());
    gst_buffer_fill(buffer, 0, &audio_msg->data[0], audio_msg->data.size());
    GstFlowReturn ret;

    g_signal_emit_by_name(_audio_source, "push-buffer", buffer, &ret);
  }

}


int main (int argc, char **argv)
{
  ros::init(argc, argv, "audio_video_recorder");
  gst_init(&argc, &argv);

  audio_video_recorder::AudioVideoRecorder client;
  client.initialize();
  ros::spin();
}
