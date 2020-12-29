# Introduction of ICT Vision Software
***
## Version Information

* 2020.06.02 Beta 0.0 Version: First Commit
* 2020.06.04 Beta 0.1 Version: Updated README.md and Minor Update
  * README.md edited
  * PWM channel info changed -> Ch.0 for pan & Ch.1 for tilt
  * Serial port reset rate changed -> 1/100 -> 1/30
* 2020.06.08 Beta 0.5 Version: Updated README.md and Detection Position Module update
  * README.md edited
  * CMakeLists.txt edited
  * Detect postition module updated
* 2020.06.09 Beta 1.0 Version: Main Header File and Detection Module update
  * CMakeLists.txt edited
  * ICT_2020.h created to use global variables as extern
  * Serial initiating and reset function re-declared
  * Detection result text file loggin modified
  * Detection module modifed for use serial data
  * Serial module modified for detected position data
* 2020.06.10 Beta 1.1 Version: Detection Module and Serial update
  * Serial communication with FCC for detection modified
  * Communication rate modified
  * Detection result file and print-out modified
* 2020.06.16 Beta 1.2 Version: Minor Update
  * All header and cpp files modified to statified the Google C++ Style Guide
* 2020.06.24 Beta 1.3 Version: Minor update
  * Communication delay problem modified
  * Header file brief modified
  * Detected object FPS information added
  * Serial communication variable modified
  * Video read check modified and video writer format modified
  * VidDraw function modified to display inference time
* 2020.08.03 Beta 1.4 Version: Minor update
  * Communication protocol size modified(Receive 22 to 26)
  * UNIX time included in protocol
  * Result logging Data modified, UNIX time added
  * Video writer frame rate modified 5 FPS to 1 FPS
  * End protocol (RECEV_BUF[4] == 99) checked
* 2020.08.03 Beta 1.5 Version: Minor update 2
  * Now possible to send detection state to FCC
  * GUI display modified, now displays mission state, lat and lon
* 2020.08.10 Beta 1.6 Version: Minor update
  * int cnt added for Global extern
  * README.md updated for new Jetson and PNUAV systems (Gimbal PWM calibrated)
* 2020.08.13 Beta 2.0 Version: Major update
  * Calculated position increasing problem modified
  * The problem was the local variable "output" in function "matrixmul"
* 2020.08.20 Beta 3.0 Version: Major update after FT
  * Calculated position error due to *1000 in DetecObj.cpp/double J modified
  * DetecObj.cpp had modified to increase the readability
* 2020.08.21 Beta 4.0 Version: Major update
  * Position estimation accuracy increased by modifying the algorithm
  * Now possible to detect multiple obstacles and target simultaneously
  * Also multiple position calculation and logging now available
* 2020.08.21 Beta 4.1 Version: Minor update after FT
  * Position estimation accuracy increased by modifying the algorithm
  * Calculation in x' modified (cos(tilt))
* 2020.08.27 Beta 4.2 Version: Minor update after FT
  * Position estimation accuracy increased by modifying the algorithm
  * Body-frame x and y bias problem modified
* 2020.09.04 Beta 4.3 Version: Minor update after FT
  * Position estimation accuracy increased by modifying the algorithm
  * Body-frame x and y bias problem modified (x direction bias 0.7m)
* 2020.09.11 Beta 4.4 Version: Minor update after FT
  * Modified for the YOLO working when UGV driving the path
* 2020.09.22 Beta 4.5 Version: Minor update after FT
  * Obstacle and target all tested
  * All functions working tested
  * Display of detected obstacle position modified

* 2020.10.12 Release 1.0 Version: Release version
  * Obstacle and target all tested
  * All functions working tested
  * Displayed information of detection modified
* 2020.10.13 Release 1.1 Version: Minor update after FT
  * Displayed information position modifed
  * Target position update modified
* 2020.10.19 Release 1.2 Version: Minor update after FT
  * Minor updates in DetecObj
* 2020.11.12 Release 1.2.1 Version: Alternative version of 1.2
  * Bug fixed -> counting fixed to address mission sequence failure


***
## 1. Software Introduction

 This is the software for the ICT Project 2020 flight tests. Main purposes of this software are detecting obstacles and target using YOLOv3 based on cuDNN and calculating the Geodetic coordinate position of those objects. This software had programmed to be worked properly on PNUAV-R2 and PNUAV-R3(Also any other platform which have same configuration).

**Developed by Samuel Sukkeun Kim**
**Flight Dynamics Lab., Dept. of Aerospace Eng., Pusan National University, Korea**
* Email: <samkim96@pusan.ac.kr>

## 2. Software Modules

 This software has 4 modules to conduct the mission as follow. Detailed function of the modules, see below.

1. SerialComm: Works for the serial communication(RS232) with FCC(Mbed) based on libserial.
2. VidProc: Works for the video input/output and several processing based on OpenCV.
3. DetecObj: Works for detecting objects and calculating the Geodetic coordinate position based on OpenCV and YOLOv3.
4. Gimbal: Works for gimbal control based on JHPWMDriver.

## 3. HW/SW Configuration and Requirements
### Hardware

* Nvidia Jetson TX2 with ConnectTech Inc. carrier board
  * ConnectTech Inc. Elroy (ASG002) for PNUAV-R2 (No longer)
  * ConnectTech Inc. Astro (ASG001) for PNUAV-R4~6
* Camera and gimbal system
  * Tarot Peeper HD 10X with TL10A00 gimbal for PNUAV-R3 (No longer)
  * GoPro 3 with Tarot T4-3D gimbal for PNUAV-R4~6
* AVerMedia DarkCrystal HD Capture Mini-PCIe C353
* SunFounder PCA9685 PWM servo driver

### Software

* Nvidia Jetson TX2 (Minimun)
  * JetPack 4.2.2
    * Ubuntu Kernel: L4T R32.2.1 (18.04)
    * GCC/G++: 7.5.0
    * CUDA: 10.0/cuDNN 7.5.0
* CMake 3.12.0 (Minimum)
* OpenCV 4.2.0 (Minimum)
* C353 Driver 1.5.0600 (for JetPack 4.2.2)
* Libserial 1.0.0
* JHPWMDriver

## 4. Gimbal Calibration(Tarot T4-3D)

 Tarot T4-3D gimbal gets the PWM signal from PCA9685 through the signal line and PCA9685 communicate with Jetson TX2 using I2C. For the detail, see below.
* Bus and Channel: Bus 0, Ox44
* Frequency: 60Hz
* PWM Channel Info (PNUAV-R4)
  * Pan: Channel 0 (300~440: -90 deg. to 90 deg.)
  * Tilt: Channel 1 (260~390: -90 deg. to 10 deg. and 335 for -30 deg.)
  * Neutral: 370
* PWM Channel Info (PNUAV-R5)
  * Pan: Channel 0 (320~460: -90 deg. to 90 deg.)
  * Tilt: Channel 1 (280~410: -90 deg. to 10 deg. and 355 for -30 deg.)
  * Neutral: 390
* Connection Info: Use righthandside input ports

## 5. To Run the Software

1. Make sure every hardware and software which had required are ready.
2. Make sure YOLOv3 names, cfg and weights are in the build directory.
3. Do CMake and make using the command as below in build directory. Use root permission to use the I2C bus.
	>$ cmake ..  
	>$ make -j4  
	>$ sudo ./ICT_2020
    
***
## Appendix 1: How to build required libraries on Jeston TX2

1. Flash Jetpack to Jetson TX2 
* [CTI-Flash](http://connecttech.com/resource-center/kdb373/ "CTI-Flash link")

2. After flashing update 
	>$ sudo apt-get update && upgrade

3. Install git & mc
	>$ sudo apt install git && mc

4. Install CMake 3.12.0
* [CMake 3.12.0](https://github.com/Kitware/CMake/releases/tag/v3.12.0/ "CMake 3.12.0 link")
	>$ ./bootstrap  
	>$ make  
	>$ sudo make install  

5. Install OpneCV 4.2.0
* [OpenCV 4.2.0](https://github.com/opencv/opencv/tree/4.2.0/ "OpenCV 4.2.0 link"),  [OpenCV CUDA](https://gist.github.com/raulqf/f42c718a658cddc16f9df07ecc627be7 "OpenCV CUDA link")
	>$ sudo apt-get update && upgrade
* [OpenCV 4.2.0 Contrib](https://github.com/opencv/opencv_contrib/releases/tag/4.2.0/ "OpenCV 4.2.0 Contrib link")

* Generic tools
	>$ sudo apt install build-essential cmake pkg-config unzip yasm git checkinstall  
	>$ sudo apt install g++-5

* Image I/O libs
	>$ sudo apt install libjpeg-dev libpng-dev libtiff-dev

* Video/Audio Libs - FFMPEG, GSTREAMER, x264 and so on
	>$ sudo apt install libavcodec-dev libavformat-dev libswscale-dev libavresample-dev  
	>$ sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev  
	>$ sudo apt install libxvidcore-dev x264 libx264-dev libfaac-dev libmp3lame-dev libtheora-dev   
	>$ sudo apt install libfaac-dev libmp3lame-dev libvorbis-dev  

* OpenCore - Adaptive Multi Rate Narrow Band (AMRNB) and Wide Band (AMRWB) speech codec
	>$ sudo apt install libopencore-amrnb-dev libopencore-amrwb-dev

* Cameras programming interface libs
	>$ sudo apt-get install libdc1394-22 libdc1394-22-dev libxine2-dev libv4l-dev v4l-utils  
	>$ cd /usr/include/linux  
	>$ sudo ln -s -f ../libv4l1-videodev.h videodev.h  
	>$ cd ~

* GTK lib for the graphical user functionalites coming from OpenCV highghui module
	>$ sudo apt-get install libgtk-3-dev

* Python libraries for python3
	>$ sudo apt-get install python3-dev python3-pip  
	>$ sudo -H pip3 install -U pip numpy  
	>$ sudo apt install python3-testresources  

* Parallelism library C++ for CPU
	>$ sudo apt-get install libtbb-dev

* Optimization libraries for OpenCV
	>$ sudo apt-get install libatlas-base-dev gfortran

* Optional libraries
	>$ sudo apt-get install libprotobuf-dev protobuf-compiler  
	>$ sudo apt-get install libgoogle-glog-dev libgflags-dev  
	>$ sudo apt-get install libgphoto2-dev libeigen3-dev libhdf5-dev doxygen  

* After installing libraries
	>$ mkdir build  
	>$ cd build  

	>$ cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DWITH_CUDA=ON \
&nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;-DWITH_CUDNN=ON -DOPENCV_DNN_CUDA=ON -DENABLE_FAST_MATH=1 -DCUDA_FAST_MATH=1 \
&nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;-DWITH_V4L=ON -DWITH_GSTREAMER=ON -DWITH_CUBLAS=1  -DCUDA_ARCH_BIN=6.2 \
&nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;-DOPENCV_EXTRA_MODULES_PATH=/home/nvidia/Programmes/opencv-4.2.0/opencv_contrib/modules \
&nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;-DCMAKE_C_COMPILER=/usr/bin/gcc-5 -DBUILD_opencv_xfeatures2d=OFF ..

	>$ make -j4  
	>$ sudo make install  
	>$ sudo /bin/bash -c 'echo "/usr/local/lib" >> /etc/ld.so.conf.d/opencv.conf'  
	>$ sudo ldconfig  
	>$ sudo apt-get install libopencv-dev

6. Install libserial
* [libserial](https://github.com/crayzeewulf/libserial "libserial link")
	>$ git clone https://github.com/crayzeewulf/libserial.git  
	>$ sudo apt update  
	>$ sudo apt install libserial-dev  

	>$ sudo apt install g++ git autogen autoconf build-essential cmake graphviz \
&nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; libboost-dev libboost-test-dev libgtest-dev libtool \
&nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; python3-sip-dev doxygen python3-sphinx pkg-config \
&nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; python3-sphinx-rtd-theme

	>$ 0001-dts-quill-common-enable-uartc-instance  
	>$ sudo usermod -a -G dialout $USER  
	>$ sudo usermod -a -G plugdev $USER
	>$ sudo apt-get install libboost-dev

* In the source folder,
	>$ ./compile.sh  
	>$ cd build  
	>$ sudo make install

7. Avermedia C353 Drvier (For the using of Capture board C353 only, R32.2.1)
* [C353 Driver](https://drive.google.com/open?id=1zV11Ghf7FFnK728mXTzi_FIrJZew-gO "C353 Driver link")
	>$ sudo make install  
	>$ sudo modprobe c353  
	>$ sudo apt-get install v4l-utils  
	>$ lspci  
	>$ v4l2-ctl --list-devices  

	>$ sudo apt-get install libgstreamer1.0-0 gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
&nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;  gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav \
&nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; gstreamer1.0-doc gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa \
&nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudi

	>$ gst-launch-1.0 -v v4l2src io-mode=2 device=/dev/video0 ! "video/x-raw,width=1920,height=1080,format=(string)YV12" ! xvimagesink

8. JHPWMDriver
* [JHPWMDriver](https://github.com/jetsonhacks/JHPWMDriver "JHPWMDriver link")
	>$ sudo apt-get install libi2c-dev i2c-tools  
	>$ sudo i2cdetect -y -r 0

* Insert below codes in the 28^th^ line of /JHPWMDriver/src/JHPWMPCA9685.h
```
	extern "C"{
	#include<i2c/smbus.h>
	}
```
* Edit Makefile as below to use example code
	> g++ servoExample.cpp ../src/JHPWMPCA9685.cpp -I../src -li2c -o servoExample



