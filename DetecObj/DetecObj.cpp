/********************************************************************************
 * @file   DetecObj.cpp								*
 * @date   13th OCT 2020							*
 * @author Sukkeun Samuel Kim(samkim96@pusan.ac.kr)				*
 * @brief  Software for the ICT Project 2020 flight tests, Detecting Object	*
 *******************************************************************************/

#include "DetecObj.h"

void DetecDnn( cv::dnn::Net &net, cv::Mat &frame, cv::Mat &blob, std::vector<std::string> classes )
{
    // Create a 4D blob from a frame
    cv::dnn::blobFromImage( frame, blob, 1/255.0, cv::Size( INPW, INPH ), cv::Scalar( 0, 0, 0 ), true, false );

    // Sets the input to the network
    net.setInput( blob );

    // Runs the forward pass to get output of the output layers
    std::vector<cv::Mat> outs;
    net.forward( outs, getOutputsNames( net ) );

    // Remove the bounding boxes with low confidence
    DetecProc( frame, outs, classes );

    // Put efficiency information. The function getPerfProfile returns the overall time for inference(t) and the timings for each of the layers(in layersTimes)
    std::vector<double> layersTimes;
    double freq = cv::getTickFrequency() / 1000;
    double t = net.getPerfProfile( layersTimes ) / freq;
    double fps = 1 / ( t / 1000 );

    double latt = (double)( RECEV_BUF_C[5]*16777216 + RECEV_BUF_C[6]*65536 + RECEV_BUF_C[7]*256 + RECEV_BUF_C[8] - 90000000 )/1000000;
    double lonn = (double)( RECEV_BUF_C[9]*16777216 + RECEV_BUF_C[10]*65536 + RECEV_BUF_C[11]*256 + RECEV_BUF_C[12] - 180000000 )/1000000;

    std::string label = cv::format( "  Inference time: %.2f ms (%.2f FPS)", t, fps );
    std::string label_1 = cv::format( "  STATE        L A T            L O N    " );
    std::string label_2 = cv::format( "    %d      %.6f      %.6f    ", RECEV_BUF_C[4], latt, lonn );

    cv::putText( frame, label, cv::Point( 0, 25 ), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar( 0, 0, 255 ), 2 );
    cv::putText( frame, label_1, cv::Point( 0, 75 ), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar( 0, 0, 255 ), 2 );
    cv::putText( frame, label_2, cv::Point( 0, 100 ), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar( 0, 0, 255 ), 2 );
}

void DetecProc( cv::Mat &frame, const std::vector<cv::Mat> &outs, std::vector<std::string> classes )
{
    std::vector<int> classIds;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;

    for ( size_t i = 0; i < outs.size(); ++i )
    {
	// Scan through all the bounding boxes output from the network and keep only the
	// ones with high confidence scores. Assign the box's class label as the class
	// with the highest score for the box.
	float *data = (float*)outs[i].data; // (float*)outs
	for ( int j = 0; j < outs[i].rows; ++j, data += outs[i].cols )
	{
	    cv::Mat scores = outs[i].row( j ).colRange( 5, outs[i].cols );
	    cv::Point classIdPoint;
	    double confidence;

	    // Get the value and location of the maximum score
	    minMaxLoc( scores, 0, &confidence, 0, &classIdPoint );
	    if ( confidence > CONFTHRES )
	    {
		int centerX = (int)( data[0] * frame.cols );
		int centerY = (int)( data[1] * frame.rows );
		int width = (int)( data[2] * frame.cols );
		int height = (int)( data[3] * frame.rows );
		int left = centerX - width / 2;
		int top = centerY - height / 2;

		classIds.push_back( classIdPoint.x );
		confidences.push_back( (float)confidence );
		boxes.push_back( cv::Rect( left, top, width, height ) );
	    }
	}
    }

    // Perform non maximum suppression to eliminate redundant overlapping boxes with
    // lower confidences
    std::vector<int> indices;
    cv::dnn::NMSBoxes( boxes, confidences, CONFTHRES, NMSTHRES, indices );
    for ( size_t i = 0; i < indices.size(); ++i )
    {
	int idx = indices[i];
	cv::Rect box = boxes[idx];
	DetecDraw( classIds[idx], confidences[idx], box.x, box.y,
		   box.x + box.width, box.y + box.height, frame, classes );
    }
}

// Draw the predicted bounding box
void DetecDraw( int classId, float conf, int left, int top, int right, int bottom, cv::Mat &frame, std::vector<std::string> classes )
{
    //Draw a rectangle displaying the bounding box
    cv::rectangle( frame, cv::Point( left, top - 10 ), cv::Point( right, bottom ), cv::Scalar( 255, 178, 50 ), 3 );

    //Get the label for the class name and its confidence
    std::string label = cv::format( "%.2f", conf );
    if ( !classes.empty() )
    {
	CV_Assert( classId < (int)classes.size( ) );
	label = classes[classId] + ": " + label;
    }

    //Display the label at the top of the bounding box
    int baseLine;
    cv::Size labelSize = cv::getTextSize( label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine );
    top = std::max( top, labelSize.height );
    putText( frame, label, cv::Point( left, top - 30 ), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar( 0,0,0 ), 2 );
    
    if ( RECEV_BUF_C[4] == 11 || RECEV_BUF_C[4] == 21 )
    {
        if ( classId == 0 )
        {
	    if ( cnt == 0 )
	    {
	        obst = 0;
	        DetecPosi( left, top, right, bottom );
	        left_old = left;
	        top_old = top;
	    }
	    else if ( sqrt( ( left_old - left )*( left_old - left ) + ( top_old - top )*( top_old - top ) ) > 100 ) 
	    {
	        obst = 1;
	        DetecPosi( left, top, right, bottom );
	    }
	    else
	    {
	        obst = 0;
	        DetecPosi( left, top, right, bottom );
	        left_old = left;
	        top_old = top;
	    }
        }
        else
        {
	    obst = 2;
	    DetecPosi( left, top, right, bottom );
        }

        ++cnt;
/*8
        if ( cnt == 15 )
        {
            detec_state = 1;
	    cnt = 0;
        }*/
        if ( obst < 2 && cnt == 15 )
        {
            detec_state = 1;
            cnt = 0;
        }
    }
}

void DetecPosi( int left, int top, int right, int bottom )
{
    double R_0 = 6378137;	
    double E = 0.0818191908425;

    // Focal length, x and y in image frame, Edited
    double focal = 1665; //2500;
    double x_i = ( right + left )/2 - 960;
    double y_i = ( bottom + top )/2 - 540;

    // Calculate the position of PNUAV-R 
    double lat = deg2rad( (double)( RECEV_BUF_C[5]*16777216 + RECEV_BUF_C[6]*65536 + RECEV_BUF_C[7]*256 + RECEV_BUF_C[8] - 90000000 )/1000000 );
    double lon = deg2rad( (double)( RECEV_BUF_C[9]*16777216 + RECEV_BUF_C[10]*65536 + RECEV_BUF_C[11]*256 + RECEV_BUF_C[12] - 180000000 )/1000000 );
    double alt = (double)( RECEV_BUF_C[13]*256 + RECEV_BUF_C[14] )/100;
    double hdg = deg2rad( (double)( RECEV_BUF_C[15]*256 + RECEV_BUF_C[16] )/100 ); //+7
    double tilt = deg2rad( 33 );  //30

    //double lat = deg2rad( 35.321031 ), lon = deg2rad( 129.010656 ), alt = 15.670000, hdg = deg2rad( 23.250000 ), tilt = deg2rad( 35 ); 	// Lat, Lon, Hdg: deg., Alt: metre

    // Radius of the Earth at given position
    double R_E = R_0/sqrt( 1 - ( E*sin( lat ) )*( E*sin( lat ) ) );

    // ECEF position
    double r_ebe[3] = {( R_E + alt )*cos( lat )*cos( lon ), ( R_E + alt )*cos( lat )*sin( lon ), ( ( 1 - E*E )*R_E + alt )*sin( lat )};

    // Conversion matrix ECEF to NED
    double C_en[3][3] = {{-sin( lat )*cos( lon ), -sin( lat )*sin( lon ), cos( lat )}, {-sin( lon ), cos( lon ), 0}, {-cos( lat )*cos( lon ), -cos( lat )*sin( lon ), -sin( lat )}};
    double ( *Temp1 )[3] = {transpose( C_en )};
    double C_ne[3][3];

    for ( int i = 0; i < 3; ++i )
    {
	for ( int j = 0; j < 3; ++j )
	{
	    C_ne[i][j] = Temp1[i][j];
	}
    }

    // Conversion matrix NED to Body
    double C_nb[3][3] = {{cos( hdg ), sin( hdg ), 0}, {-sin( hdg ), cos( hdg ), 0}, {0, 0, 1}};
    double ( *Temp2 )[3] = {transpose( C_nb )};
    double C_bn[3][3];

    for ( int i = 0; i < 3; ++i )
    {
	for ( int j = 0; j < 3; ++j )
	{
	    C_bn[i][j] = Temp2[i][j];
	}
    }

    // Conversion matrix Body to ECEF
    double ( *Temp3 )[3] = {matrixmul( C_ne, C_bn )};
    double C_be[3][3];

    for ( int i = 0; i < 3; ++i )
    {
	for ( int j = 0; j < 3; ++j )
	{
	    C_be[i][j] = Temp3[i][j];
	}
    }

    // Conversion matrix ECEF to Body
    double ( *C_eb )[3] = {transpose( C_be )};

    // Position of UAV in UAV body frame
    double Body_UAV[3];
    for ( int i = 0; i < 3; ++i ) 
	Body_UAV[i] = C_eb[i][0]*r_ebe[0] + C_eb[i][1]*r_ebe[1] + C_eb[i][2]*r_ebe[2];

    double J = 	( alt - 0.6 )/( ( focal*cos( tilt )*cos( tilt )*tan( tilt ) )/y_i + 1 - sin( tilt )*cos( tilt) *tan( tilt ) );			
    double I = 	( x_i/focal )*( ( ( alt - ( J + 0.6 ) ) )/( tan( tilt )*cos( tilt ) ) + J*sin( tilt ) );		// Image frame y in World frmae
    double x =  ( alt - ( J + 0.6 ) )/tan( tilt );									// Image frame x in World frame

    if ( obst == 2)
    {
        I = I - 5;
    }

    // Position of obstacle in UAV body frame
    double Body_obs[3] = {Body_UAV[0] + x, Body_UAV[1] + I, Body_UAV[2] - alt + 0.6};

    // Position of obstacle in ECEF frame
    double ECEF_obs[3];
    for ( int i = 0; i < 3; ++i )
	ECEF_obs[i] = C_be[i][0]*Body_obs[0] + C_be[i][1]*Body_obs[1] + C_be[i][2]*Body_obs[2];

    // ECEF to geodetic(LLA) coordinate
    double f = 1.0/298.257223563;

    double b = R_0 - f*R_0;
    double lon_obs = atan2( ECEF_obs[1], ECEF_obs[0] );									// clambda
    double p = sqrt( ECEF_obs[0]*ECEF_obs[0] + ECEF_obs[1]*ECEF_obs[1] );
    double lat_obs = atan2( ECEF_obs[2], p*( 1.0-E*E ) );								// theta
    double cs = cos( lat_obs );
    double sn = sin( lat_obs );
    double N = ( R_0*R_0 )/sqrt( ( R_0*cs )*( R_0*cs ) + ( b*sn )*( b*sn ) );
    double alt_obs = p/cos( lat_obs )-N;

    if ( obst == 0 && lat_obs_0 < 10 )
    {
	lat_obs_0 = rad2deg( lat_obs );
	lon_obs_0 = rad2deg( lon_obs );
    }
    else if ( obst == 1 && lat_obs_1 < 10 )
    {
	lat_obs_1 = rad2deg( lat_obs );
	lon_obs_1 = rad2deg( lon_obs );
    }
    else if ( obst == 2 )//&& lat_obs_2 < 10 )
    {
	lat_obs_2 = rad2deg( lat_obs );
	lon_obs_2 = rad2deg( lon_obs );
    }
    
    // [ VIDEO] Display the calculated position on the video

    std::string label = cv::format( "  OBJCT        L A T            L O N    " );
    cv::putText( frame, label, cv::Point( 1300, 75 ), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar( 0, 0, 255 ), 2 );

    std::string label_1 = cv::format( "  OBST1     %.6f      %.6f", lat_obs_0, lon_obs_0 );
    cv::putText( frame, label_1, cv::Point( 1300, 100 ), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar( 0, 0, 255 ), 2 );

    std::string label_2 = cv::format( "  OBST2     %.6f      %.6f", lat_obs_1, lon_obs_1 );
    cv::putText( frame, label_2, cv::Point( 1300, 125 ), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar( 0, 0, 255 ), 2 );

    std::string label_3 = cv::format( "  TARGT     %.6f      %.6f", lat_obs_2, lon_obs_2 );
    cv::putText( frame, label_3, cv::Point( 1300, 150 ), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar( 0, 0, 255 ), 2 );

    // [ VIDEO] Display Detection information
    if ( detec_state == 0 )
    {
        std::string label_4 = cv::format( "  Detection count: %d", cnt + 1 );
        cv::putText( frame, label_4, cv::Point( 1300, 25 ), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar( 0, 0, 255 ), 2 );
    }

    int unix_time = RECEV_BUF_C[17]*16777216 + RECEV_BUF_C[18]*65536 + RECEV_BUF_C[19]*256 + RECEV_BUF_C[20];

    std::cout << "[DETECT] Obst_no: " << obst << " Lat_Obs: " << std::fixed << rad2deg( lat_obs ) << " Lon_Obs: " << std::fixed << rad2deg( lon_obs ) << std::endl;

    // Log the  calculated position display on the output file
    outFile << " UNIX_Time: " << unix_time
	    << " Lat_UAV: " << std::fixed << rad2deg( lat ) << " Lon_UAV: " << rad2deg( lon ) 
	    << " Alt_UAV: " << alt << " HDG_UAV: " << rad2deg( hdg ) << " Obst_no: " << obst
            << " Lat_Obs: " << rad2deg( lat_obs ) << " Lon_Obs: " << rad2deg( lon_obs ) << std::endl;
}

double deg2rad( double degree ){return degree*PI/180;}
double rad2deg( double radian ){return radian*180/PI;}

double ( *transpose ( double ( *input )[3] ) )[3]
{
    static double output[3][3];

    for ( int i = 0; i < 3; ++i )
    {
	for ( int j = 0; j < 3; ++j )
	{
	    output[i][j] = input[j][i];
	}
    }
    return output;
}

double ( *matrixmul ( double ( *input_1 )[3], double ( *input_2 )[3] ) )[3]
{
    static double output[3][3];

    for ( int i = 0; i < 3; ++i )
    {
	for ( int j = 0 ; j < 3; ++j )
	{
	    output[i][j] = 0;

	    for ( int k = 0; k < 3; ++k )
	    {
		output[i][j] += input_1[i][k]*input_2[k][j];
	    }
	}
    }

    return output;
}

// Get the names of the output layers
std::vector<cv::String> getOutputsNames( const cv::dnn::Net& net )
{
    static std::vector<cv::String> names;
    if ( names.empty() )
    {
        //Get the indices of the output layers, i.e. the layers with unconnected outputs
        std::vector<int> outLayers = net.getUnconnectedOutLayers();

        //get the names of all the layers in the network
        std::vector<cv::String> layersNames = net.getLayerNames();

        // Get the names of the output layers in names
        names.resize( outLayers.size() );
        for ( size_t i = 0; i < outLayers.size(); ++i )
	    names[i] = layersNames[outLayers[i] - 1];
    }
    return names;
}
