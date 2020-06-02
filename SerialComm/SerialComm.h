#pragma once
#include <math.h>
#include <memory>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>

class SerialComm {

private:

public:

//double Cal_position[2];
unsigned char TRANS_BUF[33], RECEV_BUF[22]; //, GCS_RX_buffer[50];
unsigned char data_length = 25;
int Serial_Flag, i = 0;
bool Serial_Status = true;

void SerialRcv(){
    Serial_Flag = 0;

    if((RECEV_BUF[1] == 252) & (RECEV_BUF[0] == 252))
    {
	if((RECEV_BUF[21] == 253) & (RECEV_BUF[20] == 253))
        {
            Serial_Flag = 1;
	    printf("[SERIAL]Header matched\n");
        }else{
	    printf("[ERROR]Tail Unmatched\n");
            Serial_Flag = 0;
	    Serial_Status = false;
	}
    }else{
	    printf("[ERROR]Header Unmatched\n");
            Serial_Flag = 0;
	    Serial_Status = false;
    }  
}

void SerialTrns(){

    signed long Lat_1 = 35.321111*1000000+90000000, Long_1 = 129.010111*1000000+180000000;
    signed long Lat_2 = 35.321333*1000000+90000000, Long_2 = 129.010333*1000000+180000000;
    signed long Lat_3 = 35.321555*1000000+90000000, Long_3 = 129.010555*1000000+180000000;

    //SOF
    TRANS_BUF[0] = 252;
    TRANS_BUF[1] = 252;

    //SYS. ID
    TRANS_BUF[2] = 1;

    //Data length
    TRANS_BUF[3] = data_length;

    //Main Data Frame
    //for (int i = 0; i < 14; i++){TRANS_BUF[i + 4] = Packet_DATA[i];}
    TRANS_BUF[4] = 0;	// Detection Status: True(1) or False(0)
    TRANS_BUF[5] = Lat_1 / 16777216;	// Point 1 Lat
    TRANS_BUF[6] = Lat_1 % 16777216 / 65536;
    TRANS_BUF[7] = Lat_1 % 65536 / 256;
    TRANS_BUF[8] = Lat_1 % 256;
    TRANS_BUF[9] = Long_1 / 16777216;	// Point 1 Long
    TRANS_BUF[10] = Long_1 % 16777216 / 65536;
    TRANS_BUF[11] = Long_1 % 65536 / 256;
    TRANS_BUF[12] = Long_1 % 256;
    TRANS_BUF[13] = Lat_2 / 16777216;	// Point 2 Lat
    TRANS_BUF[14] = Lat_2 % 16777216 / 65536;
    TRANS_BUF[15] = Lat_2 % 65536 / 256;
    TRANS_BUF[16] = Lat_2 % 256;
    TRANS_BUF[17] = Long_2 / 16777216;	// Point 2 Long
    TRANS_BUF[18] = Long_2 % 16777216 / 65536;
    TRANS_BUF[19] = Long_2 % 65536 / 256;
    TRANS_BUF[20] = Long_2 % 256;
    TRANS_BUF[21] = Lat_3 / 16777216;	// Point 3 Lat
    TRANS_BUF[22] = Lat_3 % 16777216 / 65536;
    TRANS_BUF[23] = Lat_3 % 65536 / 256;
    TRANS_BUF[24] = Lat_3 % 256;
    TRANS_BUF[25] = Long_3 / 16777216;	// Point 3 Long
    TRANS_BUF[26] = Long_3 % 16777216 / 65536;
    TRANS_BUF[27] = Long_3 % 65536 / 256;
    TRANS_BUF[28] = Long_3 % 256;

    //CHKSUM
    unsigned int data_checksum = 0;
    for (int i = 0; i < data_length; i++) {
	data_checksum += TRANS_BUF[i+4];
    }
    
    TRANS_BUF[29] = data_checksum / 256;	// Check SUM SUM(Data)/2Bytes, Share
    TRANS_BUF[30] = data_checksum % 256;	// Check SUM SUM(Data)/2Bytes, Rest

    TRANS_BUF[31] = 253;
    TRANS_BUF[32] = 253;
}

};

