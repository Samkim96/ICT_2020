/********************************************************************************
 * @file   DetecObj.h								*
 * @date   21st AUG 2020							*
 * @author Sukkeun Samuel Kim(samkim96@pusan.ac.kr)				*
 * @brief  Software for the ICT Project 2020 flight tests, Detecting Object	*
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

#define CONFTHRES 	0.3									// confThreshold Original: 0.5
#define NMSTHRES 	0.2									// nmsThreshold Original: 0.4
#define INPW 		416									// inpWidth
#define INPH 		416									// inpHeight
#define PI 		3.14159265								// PI

extern unsigned char TRANS_BUF[33], RECEV_BUF[26];						// [SERIAL] Buffer init
extern int RECEV_BUF_C[26];									// [SERIAL] New SerialComm class "Serial"
extern int detec_state, cnt;									// [SERIAL] Detection State
extern cv::Mat frame, blob;									// [ VIDEO] VideoMat init
extern std::ofstream outFile;									// [DETECT] Result file init

extern int left_old, top_old;									// [DETECT] Detected Obstacle No.
extern int obst;
extern double lat_obs_0, lon_obs_0, lat_obs_1, lon_obs_1, lat_obs_2, lon_obs_2;			// [DETECT] Detected Obstacle LLA

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
