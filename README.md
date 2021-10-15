# Jetson Cam

Jetson Nano camera driver extracted from the [jetbot_ros](https://github.com/dusty-nv/jetbot_ros) package.

## Dependencies

### System Dependencies

1. GStreamer
    - *Typically preinstalled in jetson Nano*
2. `jetson-utils`
    - Building `jetson-utils` is sufficient. Heavier dependencies like `jetson-inference` are not necessary
    - Use the following build commands

    ``` bash
    git clone https://github.com/dusty-nv/jetson-utils.git
    cd jetson-utils
    mkdir build && cd build
    cmake ..
    sudo make install
    ```

### ROS Dependencies

1. `camera_info_manager`
2. `image_transport`

**Warning:** ROS Melodic ships with Opencv-3.2. If you:

- Checked *Target Components: Jetson SDK Components* in SDK Manager, OR
- Flashed the Jetson Nano OS onto an SD card

then your Jetson Nano has OpenCV 4 installed. In this case, **DO NOT** install the dependent ROS packages using `sudo apt-get`. Build these packages from [source](https://github.com/ros-perception/image_common)  

## Building

Create a ROS Catkin workspace to contain our ROS packages:

```bash
# create the catkin workspace
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws
$ catkin build # or catkin_make / colcon build
```

## Running

To begin streaming the JetBot camera, start the `jetbot_camera` node:

```bash
rosrun jetson_cam camera_node
```

A CSI camera will be opened by default and video frames will be published to `/jetson_nano/image_raw` as [`sensor_msgs::Image`](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html) messages
