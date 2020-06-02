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

const int Write_BUFFER_SIZE = 33;						// [SerialComm] Size of Write Buffer
const int Read_BUFFER_SIZE = 22;						// [SerialComm] Size of Read Buffer
int n = 0;									// [SerialComm] integer for count

cv::VideoCapture cap;								// [VidProc] VideoCapture init
cv::VideoWriter video;								// [VidProc] VideoWriter init
static const std::string WinName = "ICT_VISION";				// [VidProc] Window Name init

std::vector<std::string> classes;						// [DetecObj] Class init 

void SerialRes(SerialStream &Res){						// [SerialComm] Serial Reset if comm has problem
    Res.Open("/dev/ttyTHS2");
    Res.SetBaudRate(BaudRate::BAUD_115200);
    Res.SetParity(Parity::PARITY_NONE);
    Res.SetCharacterSize(CharacterSize::CHAR_SIZE_8);
    Res.SetStopBits(StopBits::STOP_BITS_DEFAULT);
    Res.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);
    std::cout << "[SERIAL]Serial Port Restarted\n" << std::endl;
}

int main(int argc, char* argv[]){

//====================================SERIAL====================================//
    SerialComm Serial;								// [SerialComm] New SerialComm class "Serial"
    SerialStream Stream;							// [SerialComm] New SerialStream class "Stream"

    Stream.Open("/dev/ttyTHS2");						// [SerialComm] Open Serial port

    // [SerialComm] Serial port initial setting
    Stream.SetBaudRate(BaudRate::BAUD_115200);
    Stream.SetParity(Parity::PARITY_NONE);
    Stream.SetCharacterSize(CharacterSize::CHAR_SIZE_8);
    Stream.SetStopBits(StopBits::STOP_BITS_DEFAULT);
    Stream.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);

   if (!Stream.good()){
	std::cout << "[ERROR] Could Not Set the Serial Port" << std::endl;
	exit(1);
    }

//====================================GIMBAL====================================//

    PCA9685 *pca9685 = new PCA9685(0x44);
    pca9685->openPCA9685();
    
    printf("[GIMBAL] PCA9685 Device Address: 0x%02X\n",pca9685->kI2CAddress) ;
    pca9685->setAllPWM(0,0);
    pca9685->reset();
    pca9685->setPWMFrequency(60);

//====================================DETECT====================================//

    // [DetecObj] Load names of classes
    std::string classesFile = "ICT.names";
    std::ifstream ifs(classesFile.c_str());
    std::string line;
    while (getline(ifs, line)) classes.push_back(line);

    // [DetecObj] Give the configuration and weight files for the model
    cv::String modelConfiguration = "yolov3-ICT.cfg";
    cv::String modelWeights = "yolov3-ICT_best.weights";

    // [DetecObj] Load the network
    std::cout << "[DETECT] Loading Deep Learning Backend: CUDA\n" << std::endl;
    cv::dnn::Net net = cv::dnn::readNetFromDarknet(modelConfiguration, modelWeights);  
    net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);			// [DetecObj] Enable CUDA
    net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);

//====================================VIDEO====================================//

    VidCap(cap, video);								// [VideoProc] Video init and Capture video

    cv::Mat frame, blob;
    cv::namedWindow("ICT_VISION", cv::WINDOW_AUTOSIZE);

    //Stream.read((char*)Serial.RECEV_BUF, Read_BUFFER_SIZE);			// [SerialComm] Read Serial data
    //Serial.SerialRcv();

//====================================MLOOP====================================//

    while(1){ //Serial.RECEV_BUF[4] < 50){
	if(n == 0) std::cout << "\n[VIDEO] Video Starting! Press ESC to Exit" << std::endl;
	
	//Stream.read((char*)Serial.RECEV_BUF, Read_BUFFER_SIZE);		// [SerialComm] Read Serial Data
	//Serial.SerialRcv();
	/*
        printf("[SERIAL]Protocol: %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d \n", 
               int(Serial.RECEV_BUF[0]), int(Serial.RECEV_BUF[1]), int(Serial.RECEV_BUF[2]), int(Serial.RECEV_BUF[3]), int(Serial.RECEV_BUF[4]), int(Serial.RECEV_BUF[5]), 
               int(Serial.RECEV_BUF[6]), int(Serial.RECEV_BUF[7]), int(Serial.RECEV_BUF[8]), int(Serial.RECEV_BUF[9]), int(Serial.RECEV_BUF[10]), int(Serial.RECEV_BUF[11]), 
               int(Serial.RECEV_BUF[12]), int(Serial.RECEV_BUF[13]), int(Serial.RECEV_BUF[14]), int(Serial.RECEV_BUF[15]), int(Serial.RECEV_BUF[16]), int(Serial.RECEV_BUF[17]), 
               int(Serial.RECEV_BUF[18]), int(Serial.RECEV_BUF[19]), int(Serial.RECEV_BUF[20]), int(Serial.RECEV_BUF[21]));*/

/*	if(n%100 == 0 || Serial.Serial_Status != true){				// [SerialComm] For prevent Error, reset Serial port every 100 times
	    Stream.Close();
	    SerialRes(Stream);
	}*/

	cap >> frame;								// [VidProc] Push cap buffer to frame
	cv::cvtColor(frame, frame, cv::COLOR_YUV2BGR_YV12);

	//if(Serial.RECEV_BUF[4] == 11 || Serial.RECEV_BUF[4] == 21){		// Detecting Mode
	    pca9685->setPWM(0, 0, 350);
	    DetecDnn(net, frame, blob, classes);
	//}else{								// Normal Mode
	//    pca9685->setPWM(0, 0, 390);
	//    VidDraw();
	//}
	
	VidDisp(WinName, frame);	    
	VidWrite(video, frame);
	n++;
	
	Serial.SerialTrns();									// [SerialComm] Write Serial Data
	Stream.write(reinterpret_cast<const char *>(Serial.TRANS_BUF), Write_BUFFER_SIZE);

	if(cv::waitKey(1) == 27) //27 is ESC
	    break;	
    }

//====================================INITI====================================//

    Stream.Close();
    cap.release();
    pca9685->setPWM(0, 0, 390);
    pca9685->closePCA9685();
    std::cout<<"[SERIAL] End of Mission"<<std::endl;

    return 0;
}
