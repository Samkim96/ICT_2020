/********************************************************************************
 * @file   DetecObj.h								*
 * @date   16th JUN 2020							*
 * @author Sukkeun Samuel Kim(samkim96@pusan.ac.kr)				*
 * @brief  Software for the ICT Project 2020 flight tests, video processing	*
 *******************************************************************************/

#ifndef ICT_DetecObj_H_
#define ICT_DetecObj_H_

#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <string>

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "SerialComm.h"

#define CONFTHRES 	0.5									// confThreshold
#define NMSTHRES 	0.4									// nmsThreshold
#define INPW 		416									// inpWidth
#define INPH 		416									// inpHeight
#define PI 		3.14159265								// PI

extern unsigned char TRANS_BUF[33], RECEV_BUF[22];						// [SERIAL] Buffer init
extern SerialComm Serial;									// [SERIAL] New SerialComm class "Serial"
extern cv::Mat frame, blob;									// [ VIDEO] VideoMat init
extern std::ofstream outFile;									// [DETECT] Result file init

extern double lat_obs_1, lon_obs_1, lat_obs_2, lon_obs_2, lat_obs_3, lon_obs_3;			// [DETECT] Detected Obstacle Data

void DetecDnn( cv::dnn::Net &net, cv::Mat &frame, cv::Mat &blob, std::vector<std::string> classes );

// Remove the bounding boxes with low confidence using non-maxima suppression
void DetecProc( cv::Mat &frame, const std::vector<cv::Mat> &out, std::vector<std::string> classes );

// Draw the predicted bounding box
void DetecDraw( int classId, float conf, int left, int top, int right, int bottom, cv::Mat& frame, std::vector<std::string> classes );

// Detect the Geodetic(LLA) Postion of the obstacle
void DetecPosi( int left, int top, int right, int bottom );

// Calculators for coordinate conversion
double deg2rad( double degree );
double rad2deg( double radian );
double ( *transpose(double ( *input )[3] ) )[3];
double ( *matrixmul(double ( *input_1 )[3], double ( *input_2 )[3] ) )[3];

// Get the names of the output layers
std::vector<cv::String> getOutputsNames( const cv::dnn::Net &net );

#endif  // ICT_DetecObj_H_