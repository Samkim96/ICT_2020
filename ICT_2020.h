/********************************************************************************
 * @file   ICT_2020.h								*
 * @date   16th JUN 2020							*
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

using namespace LibSerial;

const int Write_BUFFER_SIZE = 33;							// [SERIAL] Size of Write Buffer
const int Read_BUFFER_SIZE  = 22;							// [SERIAL] Size of Read Buffer

cv::VideoCapture cap;									// [ VIDEO] VideoCapture init
cv::VideoWriter video;									// [ VIDEO] VideoWriter init
cv::Mat frame, blob;									// [ VIDEO] VideoMat init
static const std::string WinName = "ICT_VISION";					// [ VIDEO] Window Name init

double lat_obs_1, lon_obs_1, lat_obs_2, lon_obs_2, lat_obs_3, lon_obs_3;		// [DETECT] Detected Obstacle Data
std::vector<std::string> classes;							// [DETECT] Class init
std::ofstream outFile( "Reuslt.txt" );							// [DETECT] Result file init

SerialComm Serial;									// [SERIAL] New SerialComm class "Serial"

// [SERIAL] Serial port initial setting
void SerialInit( SerialStream &Stream )
{
    Stream.Open( "/dev/ttyTHS2" );							// [SERIAL] Open Serial port ttyTHS2
    Stream.SetBaudRate( BaudRate::BAUD_115200 );
    Stream.SetParity( Parity::PARITY_NONE );
    Stream.SetCharacterSize( CharacterSize::CHAR_SIZE_8 );
    Stream.SetStopBits( StopBits::STOP_BITS_DEFAULT );
    Stream.SetFlowControl( FlowControl::FLOW_CONTROL_NONE );

    if ( !Stream.good() )
    {
	std::cout << "[ERROR!] Could Not Set the Serial Port" << std::endl;
	exit( 1 );
    }
}

#endif  // ICT_ICT_2020_H_
