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

    std::string file_location;
    std::string device;
    bool do_timestamp;
    std::string format;
    int channels;
    int depth;
    int sample_rate;
    std::string sample_format;

    int queue_size;
    bool use_async;

    // The destination of the audio
    ros::param::param<std::string>("~file_location", file_location, "/tmp/test.mp4");
    ros::param::param<std::string>("~device", device, std::string());
    ros::param::param<bool>("~do_timestamp", do_timestamp, true);
    ros::param::param<std::string>("~format", format, "mp3");
    ros::param::param<int>("~channels", channels, 1);
    ros::param::param<int>("~depth", depth, 16);
    ros::param::param<int>("~sample_rate", sample_rate, 16000);
    ros::param::param<std::string>("~sample_format", sample_format, "S16LE");
    ros::param::param<int>("~queue_size", queue_size, 10);

    _sub_image = _pnh->subscribe("input/image", queue_size, &AudioVideoRecorder::callbackImage, this);
    _sub_audio = _pnh->subscribe("input/audio", queue_size, &AudioVideoRecorder::callbackAudio, this);

    _loop = g_main_loop_new(NULL, false);
    _pipeline = gst_pipeline_new("app_pipeline");
    _source = gst_element_factory_make("appsrc", "app_source");
    g_object_set(G_OBJECT(_source), "do-timestamp", (do_timestamp) ? TRUE : FALSE, NULL);

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

    ROS_INFO("Saving file to %s", file_location.c_str());
    _sink = gst_element_factory_make("filesink", "sink");
    g_object_set(G_OBJECT(_sink), "location", file_location.c_str(), NULL);

    if (format == "mp3")
    {
      gst_bin_add_many(GST_BIN(_pipeline), _source, _sink, NULL);
      gst_element_link(_source, _sink);
    }
    else if (format == "wave")
    {
      g_object_set( G_OBJECT(_source), "caps", caps, NULL);
      g_object_set (G_OBJECT (_source), "format", GST_FORMAT_TIME, NULL);
      _filter = gst_element_factory_make("wavenc", "filter");
      gst_bin_add_many(GST_BIN(_pipeline), _source, _filter, _sink, NULL);
      gst_element_link_many( _source, _filter, _sink, NULL);
      gst_caps_unref(caps);
    }
    else
    {
      ROS_ERROR("Unsupported format: %s", format.c_str());
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

    g_signal_emit_by_name(_source, "push-buffer", buffer, &ret);
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
