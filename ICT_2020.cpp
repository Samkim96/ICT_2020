/********************************************************************************
 * @file   ICT_2020.cpp								*
 * @date   9th OCT 2020								*
 * @author Sukkeun Samuel Kim(samkim96@pusan.ac.kr)				*
 * @brief  Software for the ICT Project 2020 flight tests, main cpp		*
 *******************************************************************************/

#include "ICT_2020.h"

int main( int argc, char* argv[] )
{
/*************************************SERIAL************************************/

    // [SERIAL] New SerialStream class "Stream"
    SerialComm Serial;
    LibSerial::SerialStream Stream;
    SerialInit( Stream );

/*************************************GIMBAL************************************/

    // [GIMBAL] i2C initial setting
    PCA9685 *pca9685 = new PCA9685( 0x44 );
    pca9685->openPCA9685();

    printf( "[GIMBAL] PCA9685 Device Address: 0x%02X\n", pca9685->kI2CAddress );
    pca9685->setAllPWM( 0,0 );
    pca9685->reset();
    pca9685->setPWMFrequency( 60 );

/*************************************DETECT************************************/

    // [DETECT] Load names of classes
    std::string classesFile = "ICT.names";
    std::ifstream ifs( classesFile.c_str() );
    std::string line;
    while ( getline( ifs, line ) ) classes.push_back( line );

    // [DETECT] Give the configuration and weight files for the model
    cv::String modelConfiguration = "yolov3-ICT.cfg";
    cv::String modelWeights = "yolov3-ICT_best.weights";

    // [DETECT] Load the network
    std::cout << "[DETECT] Loading Deep Learning Backend: CUDA" << std::endl;
    cv::dnn::Net net = cv::dnn::readNetFromDarknet( modelConfiguration, modelWeights );  
    net.setPreferableBackend( cv::dnn::DNN_BACKEND_CUDA );
    net.setPreferableTarget( cv::dnn::DNN_TARGET_CUDA );

/*************************************VIDEO*************************************/

    // [ VIDEO] Video init and Capture video
    VidCap( cap, video );
    //cv::VideoCapture cap{ "ICT_20200608_160538.avi" };					// [ VIDEO] Video init from the file
    cv::namedWindow( "ICT_VISION", cv::WINDOW_AUTOSIZE );

    // [SERIAL] Read Serial data
    std::cout << "[SERIAL] Waiting for Protocol" << std::endl;
    Stream.read( (char*)Serial.RECEV_BUF, READ_BUFFER_SIZE );

/*************************************MLOOP*************************************/

    int n = 0;											// [SERIAL] integer for count
    double ts, te = 0;
    detec_state = 0;

    while ( Serial.RECEV_BUF[4] < 50 )
	{
	    ts = cv::getTickCount();

	    if ( n == 0 ) std::cout << "[ VIDEO] Starting Video! Press ESC to Exit" << std::endl;

	    // [SERIAL] Read Serial Data, Buffer parsing speed from FCC must slower than the main loop speed (5Hz)
	    Stream.read( (char*)Serial.RECEV_BUF, READ_BUFFER_SIZE );

	    for ( int i = 0; i < READ_BUFFER_SIZE; ++i )
	    {
		RECEV_BUF_C[i] = int( Serial.RECEV_BUF[i] );
	    }

/*
	    if ( n % 5 == 0 ) {
	        Stream.read( (char*)Serial.RECEV_BUF, READ_BUFFER_SIZE );

	        for ( int i = 0; i < READ_BUFFER_SIZE; ++i )
	        {
		    RECEV_BUF_C[i] = int( Serial.RECEV_BUF[i] );
	        }
	    }
*/
/*
	    printf( "[SERIAL] %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d \n", 
		    int( Serial.RECEV_BUF[ 0] ), int( Serial.RECEV_BUF[ 1] ), int( Serial.RECEV_BUF[ 2] ), int( Serial.RECEV_BUF[ 3] ), 
		    int( Serial.RECEV_BUF[ 4] ), int( Serial.RECEV_BUF[ 5] ), int( Serial.RECEV_BUF[ 6] ), int( Serial.RECEV_BUF[ 7] ), 
		    int( Serial.RECEV_BUF[ 8] ), int( Serial.RECEV_BUF[ 9] ), int( Serial.RECEV_BUF[10] ), int( Serial.RECEV_BUF[11] ), 
		    int( Serial.RECEV_BUF[12] ), int( Serial.RECEV_BUF[13] ), int( Serial.RECEV_BUF[14] ), int( Serial.RECEV_BUF[15] ), 
		    int( Serial.RECEV_BUF[16] ), int( Serial.RECEV_BUF[17] ), int( Serial.RECEV_BUF[18] ), int( Serial.RECEV_BUF[19] ),
		    int( Serial.RECEV_BUF[20] ), int( Serial.RECEV_BUF[21] ), int( Serial.RECEV_BUF[22] ), int( Serial.RECEV_BUF[23] ), 
		    int( Serial.RECEV_BUF[24] ), int( Serial.RECEV_BUF[25] ) );
*/
	    //std::cout << "Trigger: " << int ( Serial.RECEV_BUF[ 4] ) << std::endl;

	    // [ VIDEO] Push cap buffer to frame and break if frame is empty
	    cap >> frame;
	    if ( frame.empty() ) break;

	    // [ VIDEO] Video colour convert for processing
	    cv::cvtColor( frame, frame, cv::COLOR_YUV2BGR_YV12 );

	    frame.convertTo(frame, -1, 1, -20); // Arrange last int for make darker or brighter

	    // [DETECT] Detecting object mode
	    if ( Serial.RECEV_BUF[4] == 11 || Serial.RECEV_BUF[4] == 21 )
	    {
		pca9685->setPWM( 1, 0, 335 );							// 30 deg. tilt down
		DetecDnn( net, frame, blob, classes );
		++ time_out;

		if ( time_out == 29 && cnt == 0 ){
		    detec_state = 1;
		}
	    }
	    // [DETECT] Special just for today
	    else if ( Serial.RECEV_BUF[4] == 12 || Serial.RECEV_BUF[4] == 22 )
	    {
		pca9685->setPWM( 1, 0, 335 );							// 30 deg. tilt down
		DetecDnn( net, frame, blob, classes );
	    }
	    // [DETECT] Normal mode
	    else
	    {
		pca9685->setPWM( 1, 0, 370 );
		te = cv::getTickCount();
		VidDraw( ts, te, frame );
	    }

	    if ( Serial.RECEV_BUF[4] == 20 || Serial.RECEV_BUF[4] == 30 )
	    {
		time_out = 0;
    		detec_state = 0;

		if ( Serial.RECEV_BUF[4] == 30 ){
		    pca9685->setPWM( 1, 0, 300 );
		}
	    }

	    VidDisp( WinName, frame );
	    VidWrite( video, frame );
	
	    ++n;
	
	    // [SERIAL] Write Serial Data
	    Serial.SerialTrns();
	    Stream.write( reinterpret_cast<const char*>( Serial.TRANS_BUF ), WRITE_BUFFER_SIZE );

	    // [ VIDEO] Break if ESC key is entered
	    if ( cv::waitKey( 1 ) == 27 ) break;

	}

/*************************************INITI*************************************/

    Stream.Close();
    outFile.close();
    cap.release();
    pca9685->setPWM( 1, 0, 370 );
    pca9685->closePCA9685();
    std::cout << "[SERIAL] End of Mission" << std::endl;

    return 0;
}
