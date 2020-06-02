#include <iostream>
#include <string>
#include <stdio.h>
#include "gst/gst.h"
#include "gst/app/gstappsink.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

void VidCap(cv::VideoCapture& cap, cv::VideoWriter& write);
void VidDraw();
void VidDisp(std::string WinName, cv::Mat& frame);
void VidWrite(cv::VideoWriter& write, cv::Mat& frame);
