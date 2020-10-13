/********************************************************************************
 * @file   SerialComm.h								*
 * @date   21st AUG 2020							*
 * @author Sukkeun Samuel Kim(samkim96@pusan.ac.kr)				*
 * @brief  Software for the ICT Project 2020 flight tests, Serial Communication	*
 *******************************************************************************/

#ifndef ICT_SerialComm_H_
#define ICT_SerialComm_H_

#include <iostream>
#include <memory>
#include <cmath>

extern double lat_obs_0, lon_obs_0, lat_obs_1, lon_obs_1, lat_obs_2, lon_obs_2;			// [DETECT] Detected Obstacle Data
extern int detec_state;										// [SERIAL] Detection State

class SerialComm {

private:

public:

unsigned char TRANS_BUF[33], RECEV_BUF[26];
unsigned char data_length = 26;

int Serial_Flag, i = 0;
bool Serial_Status = true;

void SerialRcv()
{
    Serial_Flag = 0;

    if ( ( RECEV_BUF[1] == 252 ) & ( RECEV_BUF[0] == 252 ) )
    {
	if ( ( RECEV_BUF[21] == 253 ) & ( RECEV_BUF[20] == 253 ) )
	{
	    Serial_Flag = 1;
	    std::cout << "[SERIAL] Header matched\n" << std::endl;
        }
	else
	{
	    std::cout << "[ERROR!] Tail Unmatched\n" << std::endl;
	    Serial_Flag = 0;
	    Serial_Status = false;
	}
    }
    else
    {
	std::cout << "[ERROR!] Header Unmatched\n" << std::endl;
	Serial_Flag = 0;
	Serial_Status = false;
    }
}

void SerialTrns()
{
    signed long Lat_obs_0 = lat_obs_0*1000000 + 90000000;
    signed long Lon_obs_0 = lon_obs_0*1000000 + 180000000;
    signed long Lat_obs_1 = lat_obs_1*1000000 + 90000000;
    signed long Lon_obs_1 = lon_obs_1*1000000 + 180000000;
    signed long Lat_obs_2 = lat_obs_2*1000000 + 90000000;
    signed long Lon_obs_2 = lon_obs_2*1000000 + 180000000;

    //SOF
    TRANS_BUF[0] = 252;
    TRANS_BUF[1] = 252;

    //SYS. ID
    TRANS_BUF[2] = 1;

    //Data length
    TRANS_BUF[3] = data_length;

    //Main Data Frame
    //for (int i = 0; i < 14; i++){TRANS_BUF[i + 4] = Packet_DATA[i];}
    TRANS_BUF[4]  = detec_state;								// Detection State: True(1) or False(0)
    TRANS_BUF[5]  = Lat_obs_0 / 16777216;							// Point 1 Lat
    TRANS_BUF[6]  = Lat_obs_0 % 16777216 / 65536;
    TRANS_BUF[7]  = Lat_obs_0 % 65536 / 256;
    TRANS_BUF[8]  = Lat_obs_0 % 256;
    TRANS_BUF[9]  = Lon_obs_0 / 16777216;							// Point 1 Long
    TRANS_BUF[10] = Lon_obs_0 % 16777216 / 65536;
    TRANS_BUF[11] = Lon_obs_0 % 65536 / 256;
    TRANS_BUF[12] = Lon_obs_0 % 256;
    TRANS_BUF[13] = Lat_obs_1 / 16777216;							// Point 2 Lat
    TRANS_BUF[14] = Lat_obs_1 % 16777216 / 65536;
    TRANS_BUF[15] = Lat_obs_1 % 65536 / 256;
    TRANS_BUF[16] = Lat_obs_1 % 256;
    TRANS_BUF[17] = Lon_obs_1 / 16777216;							// Point 2 Long
    TRANS_BUF[18] = Lon_obs_1 % 16777216 / 65536;
    TRANS_BUF[19] = Lon_obs_1 % 65536 / 256;
    TRANS_BUF[20] = Lon_obs_1 % 256;
    TRANS_BUF[21] = Lat_obs_2 / 16777216;							// Point 3 Lat
    TRANS_BUF[22] = Lat_obs_2 % 16777216 / 65536;
    TRANS_BUF[23] = Lat_obs_2 % 65536 / 256;
    TRANS_BUF[24] = Lat_obs_2 % 256;
    TRANS_BUF[25] = Lon_obs_2 / 16777216;							// Point 3 Long
    TRANS_BUF[26] = Lon_obs_2 % 16777216 / 65536;
    TRANS_BUF[27] = Lon_obs_2 % 65536 / 256;
    TRANS_BUF[28] = Lon_obs_2 % 256;

    //CHKSUM
    unsigned int data_checksum = 0;
    for ( int i = 0; i < data_length; ++i )
    {
	data_checksum += TRANS_BUF[i+4];
    }

    TRANS_BUF[29] = data_checksum / 256;							// Check SUM SUM(Data)/2Bytes, Share
    TRANS_BUF[30] = data_checksum % 256;							// Check SUM SUM(Data)/2Bytes, Rest

    TRANS_BUF[31] = 253;
    TRANS_BUF[32] = 253;
}
};

#endif  // ICT_SerialComm_H_
