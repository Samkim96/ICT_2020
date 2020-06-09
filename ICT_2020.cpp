#include "ICT_2020.h"

int main(int argc, char* argv[]){

//====================================SERIAL====================================//
    int n = 0;									// [SerialComm] integer for count

    //SerialComm Serial;								// [SerialComm] New SerialComm class "Serial"
    SerialStream Stream;							// [SerialComm] New SerialStream class "Stream"
    SerialInit(Stream);

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
    std::cout << "[DETECT] Loading Deep Learning Backend: CUDA" << std::endl;
    cv::dnn::Net net = cv::dnn::readNetFromDarknet(modelConfiguration, modelWeights);  
    net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);			// [DetecObj] Enable CUDA
    net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);

//====================================VIDEO====================================//

    //VidCap(cap, video);							// [VideoProc] Video init and Capture video
    cv::VideoCapture cap{"ICT_20200608_160538.avi"};				// [VideoProc] Video init from the file
    cv::namedWindow("ICT_VISION", cv::WINDOW_AUTOSIZE);

    Stream.read((char*)Serial.RECEV_BUF, Read_BUFFER_SIZE);			// [SerialComm] Read Serial data
    Serial.SerialRcv();

//====================================MLOOP====================================//

    while(1){ //Serial.RECEV_BUF[4] < 50){
	if(n == 0) std::cout << "[VIDEO!] Video Starting! Press ESC to Exit" << std::endl;
	
	Stream.read((char*)Serial.RECEV_BUF, Read_BUFFER_SIZE);		// [SerialComm] Read Serial Data
	Serial.SerialRcv();
	
/*        printf("[SERIAL]Protocol: %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d \n", 
               int(Serial.RECEV_BUF[0]), int(Serial.RECEV_BUF[1]), int(Serial.RECEV_BUF[2]), int(Serial.RECEV_BUF[3]), int(Serial.RECEV_BUF[4]), int(Serial.RECEV_BUF[5]), 
               int(Serial.RECEV_BUF[6]), int(Serial.RECEV_BUF[7]), int(Serial.RECEV_BUF[8]), int(Serial.RECEV_BUF[9]), int(Serial.RECEV_BUF[10]), int(Serial.RECEV_BUF[11]), 
               int(Serial.RECEV_BUF[12]), int(Serial.RECEV_BUF[13]), int(Serial.RECEV_BUF[14]), int(Serial.RECEV_BUF[15]), int(Serial.RECEV_BUF[16]), int(Serial.RECEV_BUF[17]), 
               int(Serial.RECEV_BUF[18]), int(Serial.RECEV_BUF[19]), int(Serial.RECEV_BUF[20]), int(Serial.RECEV_BUF[21]));*/

	if(n%30 == 0 || Serial.Serial_Status != true){				// [SerialComm] For prevent Error, reset Serial port every 100 times
	    Stream.Close();
	    SerialInit(Stream);
   	    std::cout << "[SERIAL] Serial Port Restarted\n" << std::endl;
	}

	cap >> frame;								// [VidProc] Push cap buffer to frame
	//cv::cvtColor(frame, frame, cv::COLOR_YUV2BGR_YV12);			// [VidProc] Video colour convert for processing

	//if(Serial.RECEV_BUF[4] == 11 || Serial.RECEV_BUF[4] == 21){		// Detecting Mode
	    pca9685->setPWM(1, 0, 355);						// 30 deg. tilt down
	    DetecDnn(net, frame, blob, classes);
	//}else{								// Normal Mode
	//    pca9685->setPWM(1, 0, 390);
	//    VidDraw();
	//}

	VidDisp(WinName, frame);	    
	VidWrite(video, frame);
	//video.write(frame);
	n++;
	
	Serial.SerialTrns();							// [SerialComm] Write Serial Data
	Stream.write(reinterpret_cast<const char *>(Serial.TRANS_BUF), Write_BUFFER_SIZE);

	if(cv::waitKey(1) == 27) //27 is ESC
	    break;	
    }

//====================================INITI====================================//

    Stream.Close();
    outFile.close();
    cap.release();
    pca9685->setPWM(1, 0, 390);
    pca9685->closePCA9685();
    std::cout<<"[SERIAL] End of Mission"<<std::endl;

    return 0;
}
