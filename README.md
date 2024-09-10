# Confihure UAV Setup Guide

This guide provides step-by-step instructions for setting up Bird 3, including configuring various libraries and frameworks such as GStreamer, OpenCV, TensorRT with YOLO, PyMavlink, and others. **Note:** Replace `IP_configuration` as per your bird.

## Table of Contents
- [Pre-requisites](#pre-requisites)
- [Gstreamer](#gstreamer)
- [Opencv with Gstreamer](#opencv-with-gstreamer)
- [TensorRT and Yolo](#tensorrt-and-yolo)
- [Pymavlink](#pymavlink)
- [Mavproxy](#mavproxy)
- [Keyboard Input Library](#keyboard-input-library)
- [SIYI SDK (IP Cam SDK)](#siyi-sdk-ip-cam-sdk)
- [ROS](#ros)
- [MAVROS](#mavros)

## Pre-requisites

### Make Python3 Default:
```bash
sudo ln -sf /usr/bin/python3 /usr/bin/python
````
## Gstreamer
### Check if Gstreamer is Installed
To verify if Gstreamer is installed on your system, use the following commands:

```bash
gst-launch-1.0 --gst-version
````
This will display the version of Gstreamer installed.

Alternatively, you can use the following command to list all Gstreamer packages installed on your system:
```bash
dpkg -l | grep gstreamer
```
### Check for NVIDIA encoder/decoder
```
gst-inspect-1.0 | grep nv
gst-inspect-1.0 nvh264enc
```
