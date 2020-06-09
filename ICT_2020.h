#ifndef _ICT_2020_H
#define _ICT_2020_H

#include <stdio.h>
#include <stdlib.h>
#include <memory>
#include <fstream>
#include <sstream>
#include <iostream>
#include <string>

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <SerialPort.h>
#include <SerialStream.h>

#include "SerialComm.h"
#include "VidProc.h"
#include "DetecObj.h"
#include <JHPWMPCA9685.h>

using namespace LibSerial;

const int Write_BUFFER_SIZE = 33;							// [SerialComm] Size of Write Buffer
const int Read_BUFFER_SIZE = 22;							// [SerialComm] Size of Read Buffer

cv::VideoCapture cap;									// [VidProc] VideoCapture init
cv::VideoWriter video;									// [VidProc] VideoWriter init
cv::Mat frame, blob;									// [VidProc] VideoMat init
static const std::string WinName = "ICT_VISION";					// [VidProc] Window Name init

double lat_obs_1, lon_obs_1, lat_obs_2, lon_obs_2, lat_obs_3, lon_obs_3;		// [DetecObj] Detected Obstacle Data
std::vector<std::string> classes;							// [DetecObj] Class init
std::ofstream outFile("Reuslt.txt");							// [DetecObj] Result file init

SerialComm Serial;									// [SerialComm] New SerialComm class "Serial"
void SerialInit(SerialStream &Stream){							// [SerialComm] Serial Reset if comm has problem
    Stream.Open("/dev/ttyTHS2");							// [SerialComm] Open Serial port

    // [SerialComm] Serial port initial setting
    Stream.SetBaudRate(BaudRate::BAUD_115200);
    Stream.SetParity(Parity::PARITY_NONE);
    Stream.SetCharacterSize(CharacterSize::CHAR_SIZE_8);
    Stream.SetStopBits(StopBits::STOP_BITS_DEFAULT);
    Stream.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);

   if (!Stream.good()){
	std::cout << "[ERROR!] Could Not Set the Serial Port" << std::endl;
	exit(1);
    }
}

#endif
