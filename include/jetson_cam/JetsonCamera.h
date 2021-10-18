// Copyright (c) 2021 hs293go
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef JETSONCAMERA_H
#define JETSONCAMERA_H

#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <algorithm>
#include <memory>
#include <string>

namespace camera_info_manager {
class CameraInfoManager;
}

class gstCamera;
class imageConverter;

class JetsonCamera {
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::CameraPublisher camera_pub_;
  ros::Timer streaming_timer_;

  double frame_rate_;

  std::string camera_device_;
  std::string camera_device_type_;
  std::string camera_name_;
  std::string camera_info_url_;
  std::string image_topic_;
  std::unique_ptr<gstCamera> camera_;
  std::unique_ptr<imageConverter> camera_cvt_;

  sensor_msgs::Image img_;
  std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;

  void acquireFrame(const ros::TimerEvent &);

 public:
  JetsonCamera(ros::NodeHandle &nh);

  ~JetsonCamera();

  bool streamCamera();
};

#endif  // JETSONCAMERA_H
