# siyi-ros-sdk

SIYI ROS SDK for communication with A8 Mini cameras

![HitCount](https://img.shields.io/endpoint?url=https%3A%2F%2Fhits.dwyl.com%2FHuaYuXiao%2Fsiyi-ros-sdk.json%3Fcolor%3Dpink)
![ros](https://img.shields.io/badge/ROS-noetic-22314E?logo=ros)
![ros](https://img.shields.io/badge/OpenCV-4.2.0-5C3EE8?logo=opencv)
![qt](https://img.shields.io/badge/Qt-5.12.8-41CD52?logo=qt)
![cplusplus](https://img.shields.io/badge/C%2B%2B-14-00599C?logo=cplusplus)
![python](https://img.shields.io/badge/Python-3.8.10-3776AB?logo=python)
![ubuntu](https://img.shields.io/badge/Ubuntu-20.04.6-E95420?logo=ubuntu)

Camera webpage: https://siyi.biz/en/index.php?id=22


## Features

- publish video stream to **ROS topic**, either **RTSP** or **HDMI** supported
- a panel to display video stream, and save frame **with Odometry info**
- combined with **YOLOv5** if launched
- tools to control gimbal angles


## Setup

If stream video with **HDMI**, nothing else to configure.

If stream video with **RTSP**,

* Connect the camera to PC or onboard computer using the ethernet cable that comes with it. The current implementation uses UDP communication.
* Do the PC wired network configuration. Make sure to assign a manual IP address to your computer
  * For example, IP `192.168.2.99`
  * Gateway `192.168.2.1`
  * Netmask `255.255.255.0`

### Requirements

- `OpenCV`: https://opencv.org/get-started/
- `ROS`: https://wiki.ros.org/noetic/Installation/Ubuntu
- `Qt5`: https://wiki.qt.io/Install_Qt_5_on_Ubuntu

### Installation

* Clone and build this package

```bash
git clone https://github.com/HuaYuXiao/siyi-ros-sdk.git ~/easondrone_ws/vision/siyi-ros-sdk
cd ~/easondrone_ws && catkin_make --source vision/siyi-ros-sdk --build vision/siyi-ros-sdk/build
```


## Usage

* Check the scripts in the `siyi_sdk/scripts` directory to learn how to use the SDK

* To import this module in your code, copy the `siyi_sdk.py` `siyi_message.py` `utility.py` `crc16_python.py` scripts in your code directory, and import as follows, and then follow the test examples

```python
from siyi_sdk import SIYISDK
```

### Video Streaming

```bash
roslaunch siyi_ros_sdk siyi_ros_sdk.launch
```

### YOLO detection

```bash
roslaunch siyi_ros_sdk yolov5.launch
```

The video will automatically switch to YOLO detection result once YOLO topics are available.

* An example of how to auto tracking with Yolov8 and CV2 tracket to0, see `examples/ml_object_tracker.py`
* An example of how to auto tracking with CV2 tracket, see `examples/object_tracker.py`

### Gimbal Control

* An example of gimbal control, see `examples/gimbal_control.py`

* Example: To run the `test_gimbal_rotation.py` run,

```bash
cd siyi_sdk/tests
python3 test_gimbal_rotation.py
```

## Acknowledgement

Thanks for the following packages:

- https://github.com/Innopolis-UAV-Team/siyi-python-sdk
- https://github.com/afdaniele/rtsp-ros-driver
- https://github.com/julianoes/siyi-a8-mini-camera-manager
