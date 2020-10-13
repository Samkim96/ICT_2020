/********************************************************************************
 * @file   ICT_2020.h								*
 * @date   9th OCT 2020								*
 * @author Sukkeun Samuel Kim(samkim96@pusan.ac.kr)				*
 * @brief  Software for the ICT Project 2020 flight tests, main header		*
 *******************************************************************************/

#ifndef ICT_ICT_2020_H_
#define ICT_ICT_2020_H_

#include <iostream>
#include <fstream>
#include <sstream>
#include <memory>
#include <string>

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <SerialPort.h>
#include <SerialStream.h>
#include <JHPWMPCA9685.h>
#include "SerialComm.h"
#include "VidProc.h"
#include "DetecObj.h"

const int WRITE_BUFFER_SIZE = 33;							// [SERIAL] Size of Write Buffer
const int READ_BUFFER_SIZE  = 26;							// [SERIAL] Size of Read Buffer

int RECEV_BUF_C[26];
int detec_state;									// [SERIAL] Detection State
int cnt = 0;
int time_out = 0;

cv::VideoCapture cap;									// [ VIDEO] VideoCapture init
cv::VideoWriter video;									// [ VIDEO] VideoWriter init
cv::Mat frame, blob;									// [ VIDEO] VideoMat init
static const std::string WinName = "ICT_VISION";					// [ VIDEO] Window Name init

int obst;										// [DETECT] Detected Obstacle No.
int left_old, top_old;
double lat_obs_0, lon_obs_0, lat_obs_1, lon_obs_1, lat_obs_2, lon_obs_2;		// [DETECT] Detected Obstacle LLA
std::vector<std::string> classes;							// [DETECT] Class init
std::ofstream outFile( "Reuslt.txt" );							// [DETECT] Result file init

// [SERIAL] Serial port initial setting
void SerialInit( LibSerial::SerialStream &Stream )
{
    std::cout << "[SERIAL] Setting the Serial Port ttyTHS2" << std::endl;
    Stream.Open( "/dev/ttyTHS2" );							// [SERIAL] Open Serial port ttyTHS2
    Stream.SetBaudRate( LibSerial::BaudRate::BAUD_9600 );
    Stream.SetParity( LibSerial::Parity::PARITY_NONE );
    Stream.SetCharacterSize( LibSerial::CharacterSize::CHAR_SIZE_8 );
    Stream.SetStopBits( LibSerial::StopBits::STOP_BITS_DEFAULT );
    Stream.SetFlowControl( LibSerial::FlowControl::FLOW_CONTROL_NONE );

    if ( !Stream.good() )
    {
	std::cout << "[ERROR!] Could Not Set the Serial Port" << std::endl;
	exit( 1 );
    }
}

#endif  // ICT_ICT_2020_H_
