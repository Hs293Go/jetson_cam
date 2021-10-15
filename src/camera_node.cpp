// Copyright (c) 2021 hs293go
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include "JetsonCamera.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "jetson_cam");
  ros::NodeHandle pnh("~");
  JetsonCamera camera(pnh);
  if (camera.streamCamera()) {
    ros::spin();
  } else {
    return 1;
  }
}
