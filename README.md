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
