/********************************************************************************
 * @file   VidProc.h								*
 * @date   3rd AUG 2020								*
 * @author Sukkeun Samuel Kim(samkim96@pusan.ac.kr)				*
 * @brief  Software for the ICT Project 2020 flight tests, video processing	*
 *******************************************************************************/

#ifndef ICT_VidProc_H_
#define ICT_VidProc_H_

#include <iostream>
#include <string>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "gst/gst.h"
#include "gst/app/gstappsink.h"

extern int RECEV_BUF_C[26];									// [SERIAL] New SerialComm class "Serial"

// Capture the video frome GoPro using Gstreamer pipeline
void VidCap( cv::VideoCapture &cap, cv::VideoWriter &write );
void VidDraw( double ts, double te, cv::Mat &frame );
void VidDisp( std::string WinName, cv::Mat &frame );
void VidWrite( cv::VideoWriter &write, cv::Mat &frame );

#endif	// ICT_VidProc_H_
