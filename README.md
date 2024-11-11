# siyi-ros-sdk

SIYI ROS SDK for communication with A8 Mini cameras

* Camera webpage: https://siyi.biz/en/index.php?id=22


## Features

- publish RTSP video stream to ROS topic
- a panel to display video stream and save frame
- tools to control gimbal angles


## Setup

### Requirements

* OpenCV 

```bash
sudo apt install python3-opencv -y
```

* imutils 

```bash
pip install imutils
```

* Gstreamer `https://gstreamer.freedesktop.org/documentation/installing/index.html?gi-language=c`

```bash
sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-doc gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio -y
```

- Deepstream (only for Nvidia Jetson boards)
  (https://docs.nvidia.com/metropolis/deepstream/dev-guide/text/DS_Quickstart.html#jetson-setup)
- For RTMP streaming

```bash
sudo apt install ffmpeg -y
pip install ffmpeg-python
```

* Clone and build this package

```bash
git clone https://github.com/HuaYuXiao/siyi-ros-sdk.git ~/easondrone_ws/vision/siyi-ros-sdk
cd ~/easondrone_ws && catkin_make --source vision/siyi-ros-sdk --build vision/siyi-ros-sdk/build
```

* Connect the camera to PC or onboard computer using the ethernet cable that comes with it. The current implementation uses UDP communication.
* Power on the camera
* Do the PC wired network configuration. Make sure to assign a manual IP address to your computer
  * For example, IP `192.168.2.99`
  * Gateway `192.168.2.1`
  * Netmask `255.255.255.0`
* Done. 


## Usage

* Check the scripts in the `siyi_sdk/scripts` directory to learn how to use the SDK

* To import this module in your code, copy the `siyi_sdk.py` `siyi_message.py` `utility.py` `crc16_python.py` scripts in your code directory, and import as follows, and then follow the test examples

```python
from siyi_sdk import SIYISDK
```

* Example: To run the `test_gimbal_rotation.py` run,

```bash
cd siyi_sdk/tests
python3 test_gimbal_rotation.py
```

### Video Streaming

```bash
rosrun siyi_ros_sdk rtsp_camera.launch
```

* An example of how to auto tracking with Yolov8 and CV2 tracket to0, see `examples/ml_object_tracker.py`
* An example of how to auto tracking with CV2 tracket, see `examples/object_tracker.py`

### Gimbal Control

* An example of gimbal control, see `examples/gimbal_control.py`

## Acknowledgement

Thanks for the following packages:

- https://github.com/Innopolis-UAV-Team/siyi-python-sdk
- https://github.com/afdaniele/rtsp-ros-driver
- https://github.com/julianoes/siyi-a8-mini-camera-manager
