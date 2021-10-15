// Copyright (c) 2021 hs293go
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include "JetsonCamera.h"

#include <camera_info_manager/camera_info_manager.h>
#include <jetson-utils/gstCamera.h>

#include "image_converter.h"

using namespace std::string_literals;

JetsonCamera::JetsonCamera(ros::NodeHandle &nh)
    : nh_(nh), it_(nh), camera_cvt_(new imageConverter()) {
  img_.header.frame_id = nh_.param("camera_frame_id", "head_camera"s);
  frame_rate_ = nh_.param("framerate", 30.0);
  camera_device_ = nh_.param("video_device", "0"s);
  camera_device_type_ = nh_.param("device_type", "default"s);
  camera_name_ = nh_.param("camera_name", "head_camera"s);
  camera_info_url_ = nh_.param("camera_info_url", ""s);
  image_topic_ = nh_.param("image", "/jetson_nano/image_raw"s);
  cinfo_.reset(new camera_info_manager::CameraInfoManager(nh_, camera_name_,
                                                          camera_info_url_));

  videoOptions opts;
  opts.resource = camera_device_;
  opts.frameRate = frame_rate_;
  opts.deviceType = opts.DeviceTypeFromStr(camera_device_type_.c_str());

  sensor_msgs::CameraInfo ci;

  if (!cinfo_->isCalibrated()) {
    ROS_INFO(
        "Camera calibration is not valid! Setting image width and height from "
        "rosparams.");
    cinfo_->setCameraName(camera_device_);
    ci.width = opts.width = nh.param("image_width"s, 640);
    ci.height = opts.height = nh.param("image_height"s, 480);
    ci.header.frame_id = img_.header.frame_id;
    cinfo_->setCameraInfo(ci);
  } else {
    ci = cinfo_->getCameraInfo();
    opts.width = ci.width;
    opts.height = ci.height;
    nh.setParam("image_width"s, static_cast<int>(ci.width));
    nh.setParam("image_height"s, static_cast<int>(ci.height));
  }
  camera_.reset(gstCamera::Create(opts));

  if (camera_) {
    ROS_INFO("opening camera device %s", camera_device_.c_str());
  } else {
    ROS_ERROR("failed to open camera device %s", camera_device_.c_str());
  }

  if (!camera_cvt_) {
    ROS_ERROR("failed to create imageConverter");
  }

  ROS_INFO("Publishing to: %s", image_topic_.c_str());
  camera_pub_ = it_.advertiseCamera(image_topic_, 1);
}

JetsonCamera::~JetsonCamera() {}

void JetsonCamera::acquireFrame(const ros::TimerEvent &event) {
  float4 *imgRGBA = nullptr;

  // get the latest frame
  if (!camera_->CaptureRGBA((float **)&imgRGBA, 1000)) {
    ROS_ERROR("failed to capture camera frame");
    return;
  }

  // assure correct image size
  if (!camera_cvt_->Resize(camera_->GetWidth(), camera_->GetHeight(),
                           IMAGE_RGBA32F)) {
    ROS_ERROR("failed to resize camera image converter");
    return;
  }

  // populate the message
  if (!camera_cvt_->Convert(img_, imageConverter::ROSOutputFormat, imgRGBA)) {
    ROS_ERROR("failed to convert camera frame to sensor_msgs::Image");
    return;
  }

  sensor_msgs::CameraInfoPtr ci(
      new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));
  ci->header.frame_id = img_.header.frame_id;
  ci->header.stamp = img_.header.stamp;

  // publish the message
  camera_pub_.publish(img_, *ci, event.current_real);
}

bool JetsonCamera::streamCamera() {
  if (!camera_->Open()) {
    ROS_ERROR("failed to start camera streaming");
    return false;
  }
  ros::Rate rate(frame_rate_);
  streaming_timer_ = nh_.createTimer(rate.expectedCycleTime(),
                                     &JetsonCamera::acquireFrame, this);
  return true;
}