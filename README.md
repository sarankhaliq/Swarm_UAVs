# Configure UAV Setup Guide

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

### To check if a specific plugin is installed
```
gst-inspect-1.0 plugin_name
```
Example:
```
gst-inspect-1.0 pulsesink
```
### Check if you can get a stream
```
gst-launch-1.0 rtspsrc location=rtsp://192.168.1.23:8554/main.264 latency=0 ! decodebin ! videoconvert ! autovideosink
````
### Install Gstreamer and plugins
```bash
sudo apt-get install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio
```

### Check for NVIDIA encoder/decoder
```
gst-inspect-1.0 | grep nv
gst-inspect-1.0 nvh264enc
```

Available Plugins
Check for available NVIDIA encoder and decoder plugins with:

````bash
gst-inspect-1.0 | grep nv
````
The following plugins should be available:

- nvh264enc
- nvh265enc
- nvh264dec
- nvh265dec
- nvv4l2h264enc
- nvv4l2h265enc
- nvv4l2decoder

To inspect individual plugins, for example:

```bash
gst-inspect-1.0 nvh264enc
```

### Reference Links:
- [Installing GStreamer on Linux](https://gstreamer.freedesktop.org/documentation/installing/on-linux.html?gi-language=c)
- [NVIDIA Accelerated GStreamer](https://docs.nvidia.com/jetson/archives/r35.3.1/DeveloperGuide/text/SD/Multimedia/AcceleratedGstreamer.html)


## Opencv with Gstreamer

### Check if already Installed
````
import cv2
print(cv2.getBuildInformation())
````

### Remove other OpenCV installations
````
sudo apt-get purge *libopencv*
````
### Install requirements
````
sudo apt-get update
sudo apt-get install -y build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
sudo apt-get install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
sudo apt-get install -y python2.7-dev python3.6-dev python-dev python-numpy python3-numpy
sudo apt-get install -y libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev
sudo apt-get install -y libv4l-dev v4l-utils qv4l2 v4l2ucp
sudo apt-get install -y curl
````
### Download OpenCV
````
mkdir workspace
cd workspace
curl -L https://github.com/opencv/opencv/archive/4.5.0.zip -o opencv-4.5.0.zip
curl -L https://github.com/opencv/opencv_contrib/archive/4.5.0.zip -o opencv_contrib-4.5.0.zip
unzip opencv-4.5.0.zip
unzip opencv_contrib-4.5.0.zip
cd opencv-4.5.0/
````
### Start Building OpenCV
````
mkdir release
cd release/
cmake -D WITH_CUDA=ON -D WITH_CUDNN=ON -D CUDA_ARCH_BIN="5.3,6.2,7.2" -D CUDA_ARCH_PTX="" -D OPENCV_GENERATE_PKGCONFIG=ON -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-4.5.0/modules -D WITH_GSTREAMER=ON -D WITH_LIBV4L=ON -D BUILD_opencv_python2=ON -D BUILD_opencv_python3=ON -D BUILD_TESTS=OFF -D BUILD_PERF_TESTS=OFF -D BUILD_EXAMPLES=OFF -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local ..
make -j4
sudo make install
````

### Reference Link:
- [Installing OpenCV 4.5.0 on Jetson](https://github.com/AastaNV/JEP/blob/master/script/install_opencv4.5.0_Jetson.sh)


### TensorRT and Yolo
````
Clone the YOLOv8 TensorRT repository
git clone https://github.com/triple-Mu/YOLOv8-TensorRT.git
````

### Required Packages
- numpy <= 1.23.5
- Onnx
- Onnxsim
- Torch (Follow Instructions below)
- Torchvision (Follow Instructions below)
- Ultralytics

### Install required packages
````
pip3 install numpy==1.23.5 onnx onnxsim
pip3 install cmake --upgrade
````
### Install Torch and Torchvision
#### Follow the instructions from the NVIDIA documentation:

- [Install PyTorch on Jetson](https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/index.html)
- You must check [compatibility matrix](https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform-release-notes/pytorch-jetson-rel.html#pytorch-jetson-rel) as per your Jetson device, Jetpack and select Pytorch accordingly.

In our case __Jetpack=5.1.1__, __Pytochv2.1__

- [Install Torchvision on Jetson](https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048)

Once pytorch is installed, you must select torchvision as per your pytorch version

PyTorch and torchvision Version Compatibility
- PyTorch v1.0 - torchvision v0.2.2
- PyTorch v1.1 - torchvision v0.3.0
- PyTorch v1.2 - torchvision v0.4.0
- PyTorch v1.3 - torchvision v0.4.2
- PyTorch v1.4 - torchvision v0.5.0
- PyTorch v1.5 - torchvision v0.6.0
- PyTorch v1.6 - torchvision v0.7.0
- PyTorch v1.7 - torchvision v0.8.1
- PyTorch v1.8 - torchvision v0.9.0
- PyTorch v1.9 - torchvision v0.10.0
- PyTorch v1.10 - torchvision v0.11.1
- PyTorch v1.11 - torchvision v0.12.0
- PyTorch v1.12 - torchvision v0.13.0
- PyTorch v1.13 - torchvision v0.13.0
- PyTorch v1.14 - torchvision v0.14.1
- PyTorch v2.0 - torchvision v0.15.1
- __PyTorch v2.1 - torchvision v0.16.1 <===__
- PyTorch v2.2 - torchvision v0.17.1
- PyTorch v2.3 - torchvision v0.18.0

````
sudo apt-get install libjpeg-dev zlib1g-dev libpython3-dev libopenblas-dev libavcodec-dev libavformat-dev libswscale-dev
git clone --branch  https://github.com/pytorch/vision torchvision   
````
#### see below for version of torchvision to download
````
cd torchvision
export BUILD_VERSION=0.x.0  # where 0.x.0 is the torchvision version  
python3 setup.py install --user
cd ../  # attempting to load torchvision from build dir will result in import error
pip install 'pillow<7' # always needed for Python 2.7, not needed torchvision v0.5.0+ with Python 3.6
````

### Convert PyTorch model to ONNX
````
python3 export-det.py --weights yolov8s.pt --iou-thres 0.65 --conf-thres 0.25 --topk 100 --opset 11 --sim --input-shape 1 3 640 640 --device cuda:0
````

#### Convert ONNX to TensorRT Engine
````
python3 build.py --weights yolov8s.onnx --iou-thres 0.65 --conf-thres 0.25 --topk 100 --fp16 --device cuda:0
````
#### or
````
/usr/src/tensorrt/bin/trtexec --onnx=yolov8s.onnx --saveEngine=yolov8s.engine --fp16
````

#### Inference using TensorRT Engine
````
python3 infer-det.py --engine yolov8s.engine --imgs data --show --out-dir outputs --device cuda:0
````

## Pymavlink
````
pip3 install pymavlink
````

## Mavproxy
### Install dependencies
````
sudo apt-get install python3-dev python3-wxgtk4.0 python3-pip python3-matplotlib python3-lxml python3-pygame
````

### Install Mavproxy
````
pip3 install PyYAML mavproxy --user
echo 'export PATH="$PATH:$HOME/.local/bin"' >> ~/.bashrc
source ~/.bashrc
````

## Keyboard Input Library
````
pip3 install readchar
````

## SIYI SDK (IP Cam SDK)
### Install the SIYI SDK
````
pip3 install siyi-sdk==0.2.4
````

#### Note
This will forcefully install opencv-python==4.10.0, which might not be built with GStreamer. Uninstall this if you already have one built with GStreamer:
````
pip3 uninstall opencv-python==4.10.0
````

## ROS
(To be added)

## MAVROS
(To be added)
