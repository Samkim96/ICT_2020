/********************************************************************************
 * @file   VidProc.cpp								*
 * @date   24th JUN 2020							*
 * @author Sukkeun Samuel Kim(samkim96@pusan.ac.kr)				*
 * @brief  Software for the ICT Project 2020 flight tests, video processing	*
 *******************************************************************************/

#include "VidProc.h"

const std::string currentDateTime()
{
    struct tm	tstruct;
    char	buf[80];

    // Save current time as type of time_t
    time_t now = time( 0 );
    tstruct    = *localtime( &now );
    //strftime(buf, sizeof(buf), "%Y%m%d_%X", &tstruct);						// YYYYMMDD_HH:mm:ss string
    strftime( buf, sizeof( buf ), "%Y%m%d_%H%M%S", &tstruct );						// YYYYMMDD_HHMMSS string

    return buf;
}

void VidCap( cv::VideoCapture &cap, cv::VideoWriter &video )
{ 
    //const char *pipe = "v4l2src device=/dev/video0 ! queue ! video/x-h264,width=1920,height=1080,framerate=30/1 ! avdec_h264 ! videoconvert ! appsink";
    const char *pipe = "v4l2src io-mode=2 device=/dev/video0 ! video/x-raw,width=1920,height=1080,framerate=30/1,format=(string)YV12 ! appsink";

    cap = cv::VideoCapture( pipe );
    const std::string output_name = "ICT_" + currentDateTime() + ".avi";
    //video = cv::VideoWriter( "ICT_Vision.avi", cv::VideoWriter::fourcc( 'x', 'v', 'i', 'd' ), 4.5, cv::Size( 1920, 1080 ) );
    video = cv::VideoWriter( output_name, cv::VideoWriter::fourcc( 'F', 'M', 'P', '4' ), 4.5, cv::Size( 1920, 1080 ) ); //videowriter ( name, codec, fps, size )
}

void VidDraw( double ts, double te, cv::Mat &frame )
{
    double time = ( ( te - ts ) / cv::getTickFrequency() ) * 1000;
    double fps = 1 / ( time / 1000 );

    std::string label = cv::format( "Inference time : %.2f ms (%.2f FPS)", time, fps );
    cv::putText( frame, label, cv::Point( 0, 15 ), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar( 0, 0, 255 ) );
}

void VidDisp( std::string WinName, cv::Mat &frame )
{
    cv::imshow( WinName, frame );
}

void VidWrite( cv::VideoWriter &video, cv::Mat &frame )
{
    video.write( frame );
}
