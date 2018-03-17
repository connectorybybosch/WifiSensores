/*
* Licensee agrees that the example code provided to Licensee has been developed and released by Bosch solely as an example to be used as a potential reference for Licenseeï¿½s application development.
* Fitness and suitability of the example code for any use within Licenseeï¿½s applications need to be verified by Licensee on its own authority by taking appropriate state of the art actions and measures (e.g. by means of quality assurance measures).
* Licensee shall be responsible for conducting the development of its applications as well as integration of parts of the example code into such applications, taking into account the state of the art of technology and any statutory regulations and provisions applicable for such applications. Compliance with the functional system requirements and testing there of (including validation of information/data security aspects and functional safety) and release shall be solely incumbent upon Licensee.
* For the avoidance of doubt, Licensee shall be responsible and fully liable for the applications and any distribution of such applications into the market.
*
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are
* met:
*
*     (1) Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*
*     (2) Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in
*     the documentation and/or other materials provided with the
*     distribution.
*
*     (3)The name of the author may not be used to
*     endorse or promote products derived from this software without
*     specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
*  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
*  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
*  DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
*  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
*  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
*  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
*  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
*  IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*/
/*----------------------------------------------------------------------------*/



#include "BSP_BoardType.h"
#include "BCDS_BSP_LED.h"
#include "BCDS_BSP_Button.h"


#include <stdio.h>
#include "FreeRTOS.h"
#include "timers.h"
#include "WifiSensores.h"
#include "BCDS_CmdProcessor.h"
#include "BCDS_Assert.h"
#include "BCDS_LightSensor.h" // Light Sensor
#include "XdkSensorHandle.h" //all Sensor headers
// For WLAN
#include "BCDS_WlanConnect.h"
#include "BCDS_NetworkConfig.h"
#include "PAL_initialize_ih.h"
#include "PAL_socketMonitor_ih.h"
#include "PIp.h"
// For HTTP Requests
#include <Serval_HttpClient.h>
#include <Serval_Network.h>
// For HTTP Requests via REST
#include <Serval_RestClient.h>




/*HTTPS Beginning*/
#include <simplelink.h>

#define CA_FILE_NAME "digicert.der"
unsigned char digicert_root_crt[] = {
	0x30, 0x82, 0x03, 0xc5, 0x30, 0x82, 0x02, 0xad, 0xa0, 0x03, 0x02, 0x01,
0x02, 0x02, 0x10, 0x02, 0xac, 0x5c, 0x26, 0x6a, 0x0b, 0x40, 0x9b, 0x8f,
0x0b, 0x79, 0xf2, 0xae, 0x46, 0x25, 0x77, 0x30, 0x0d, 0x06, 0x09, 0x2a,
0x86, 0x48, 0x86, 0xf7, 0x0d, 0x01, 0x01, 0x05, 0x05, 0x00, 0x30, 0x6c,
0x31, 0x0b, 0x30, 0x09, 0x06, 0x03, 0x55, 0x04, 0x06, 0x13, 0x02, 0x55,
0x53, 0x31, 0x15, 0x30, 0x13, 0x06, 0x03, 0x55, 0x04, 0x0a, 0x13, 0x0c,
0x44, 0x69, 0x67, 0x69, 0x43, 0x65, 0x72, 0x74, 0x20, 0x49, 0x6e, 0x63,
0x31, 0x19, 0x30, 0x17, 0x06, 0x03, 0x55, 0x04, 0x0b, 0x13, 0x10, 0x77,
0x77, 0x77, 0x2e, 0x64, 0x69, 0x67, 0x69, 0x63, 0x65, 0x72, 0x74, 0x2e,
0x63, 0x6f, 0x6d, 0x31, 0x2b, 0x30, 0x29, 0x06, 0x03, 0x55, 0x04, 0x03,
0x13, 0x22, 0x44, 0x69, 0x67, 0x69, 0x43, 0x65, 0x72, 0x74, 0x20, 0x48,
0x69, 0x67, 0x68, 0x20, 0x41, 0x73, 0x73, 0x75, 0x72, 0x61, 0x6e, 0x63,
0x65, 0x20, 0x45, 0x56, 0x20, 0x52, 0x6f, 0x6f, 0x74, 0x20, 0x43, 0x41,
0x30, 0x1e, 0x17, 0x0d, 0x30, 0x36, 0x31, 0x31, 0x31, 0x30, 0x30, 0x30,
0x30, 0x30, 0x30, 0x30, 0x5a, 0x17, 0x0d, 0x33, 0x31, 0x31, 0x31, 0x31,
0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x5a, 0x30, 0x6c, 0x31, 0x0b,
0x30, 0x09, 0x06, 0x03, 0x55, 0x04, 0x06, 0x13, 0x02, 0x55, 0x53, 0x31,
0x15, 0x30, 0x13, 0x06, 0x03, 0x55, 0x04, 0x0a, 0x13, 0x0c, 0x44, 0x69,
0x67, 0x69, 0x43, 0x65, 0x72, 0x74, 0x20, 0x49, 0x6e, 0x63, 0x31, 0x19,
0x30, 0x17, 0x06, 0x03, 0x55, 0x04, 0x0b, 0x13, 0x10, 0x77, 0x77, 0x77,
0x2e, 0x64, 0x69, 0x67, 0x69, 0x63, 0x65, 0x72, 0x74, 0x2e, 0x63, 0x6f,
0x6d, 0x31, 0x2b, 0x30, 0x29, 0x06, 0x03, 0x55, 0x04, 0x03, 0x13, 0x22,
0x44, 0x69, 0x67, 0x69, 0x43, 0x65, 0x72, 0x74, 0x20, 0x48, 0x69, 0x67,
0x68, 0x20, 0x41, 0x73, 0x73, 0x75, 0x72, 0x61, 0x6e, 0x63, 0x65, 0x20,
0x45, 0x56, 0x20, 0x52, 0x6f, 0x6f, 0x74, 0x20, 0x43, 0x41, 0x30, 0x82,
0x01, 0x22, 0x30, 0x0d, 0x06, 0x09, 0x2a, 0x86, 0x48, 0x86, 0xf7, 0x0d,
0x01, 0x01, 0x01, 0x05, 0x00, 0x03, 0x82, 0x01, 0x0f, 0x00, 0x30, 0x82,
0x01, 0x0a, 0x02, 0x82, 0x01, 0x01, 0x00, 0xc6, 0xcc, 0xe5, 0x73, 0xe6,
0xfb, 0xd4, 0xbb, 0xe5, 0x2d, 0x2d, 0x32, 0xa6, 0xdf, 0xe5, 0x81, 0x3f,
0xc9, 0xcd, 0x25, 0x49, 0xb6, 0x71, 0x2a, 0xc3, 0xd5, 0x94, 0x34, 0x67,
0xa2, 0x0a, 0x1c, 0xb0, 0x5f, 0x69, 0xa6, 0x40, 0xb1, 0xc4, 0xb7, 0xb2,
0x8f, 0xd0, 0x98, 0xa4, 0xa9, 0x41, 0x59, 0x3a, 0xd3, 0xdc, 0x94, 0xd6,
0x3c, 0xdb, 0x74, 0x38, 0xa4, 0x4a, 0xcc, 0x4d, 0x25, 0x82, 0xf7, 0x4a,
0xa5, 0x53, 0x12, 0x38, 0xee, 0xf3, 0x49, 0x6d, 0x71, 0x91, 0x7e, 0x63,
0xb6, 0xab, 0xa6, 0x5f, 0xc3, 0xa4, 0x84, 0xf8, 0x4f, 0x62, 0x51, 0xbe,
0xf8, 0xc5, 0xec, 0xdb, 0x38, 0x92, 0xe3, 0x06, 0xe5, 0x08, 0x91, 0x0c,
0xc4, 0x28, 0x41, 0x55, 0xfb, 0xcb, 0x5a, 0x89, 0x15, 0x7e, 0x71, 0xe8,
0x35, 0xbf, 0x4d, 0x72, 0x09, 0x3d, 0xbe, 0x3a, 0x38, 0x50, 0x5b, 0x77,
0x31, 0x1b, 0x8d, 0xb3, 0xc7, 0x24, 0x45, 0x9a, 0xa7, 0xac, 0x6d, 0x00,
0x14, 0x5a, 0x04, 0xb7, 0xba, 0x13, 0xeb, 0x51, 0x0a, 0x98, 0x41, 0x41,
0x22, 0x4e, 0x65, 0x61, 0x87, 0x81, 0x41, 0x50, 0xa6, 0x79, 0x5c, 0x89,
0xde, 0x19, 0x4a, 0x57, 0xd5, 0x2e, 0xe6, 0x5d, 0x1c, 0x53, 0x2c, 0x7e,
0x98, 0xcd, 0x1a, 0x06, 0x16, 0xa4, 0x68, 0x73, 0xd0, 0x34, 0x04, 0x13,
0x5c, 0xa1, 0x71, 0xd3, 0x5a, 0x7c, 0x55, 0xdb, 0x5e, 0x64, 0xe1, 0x37,
0x87, 0x30, 0x56, 0x04, 0xe5, 0x11, 0xb4, 0x29, 0x80, 0x12, 0xf1, 0x79,
0x39, 0x88, 0xa2, 0x02, 0x11, 0x7c, 0x27, 0x66, 0xb7, 0x88, 0xb7, 0x78,
0xf2, 0xca, 0x0a, 0xa8, 0x38, 0xab, 0x0a, 0x64, 0xc2, 0xbf, 0x66, 0x5d,
0x95, 0x84, 0xc1, 0xa1, 0x25, 0x1e, 0x87, 0x5d, 0x1a, 0x50, 0x0b, 0x20,
0x12, 0xcc, 0x41, 0xbb, 0x6e, 0x0b, 0x51, 0x38, 0xb8, 0x4b, 0xcb, 0x02,
0x03, 0x01, 0x00, 0x01, 0xa3, 0x63, 0x30, 0x61, 0x30, 0x0e, 0x06, 0x03,
0x55, 0x1d, 0x0f, 0x01, 0x01, 0xff, 0x04, 0x04, 0x03, 0x02, 0x01, 0x86,
0x30, 0x0f, 0x06, 0x03, 0x55, 0x1d, 0x13, 0x01, 0x01, 0xff, 0x04, 0x05,
0x30, 0x03, 0x01, 0x01, 0xff, 0x30, 0x1d, 0x06, 0x03, 0x55, 0x1d, 0x0e,
0x04, 0x16, 0x04, 0x14, 0xb1, 0x3e, 0xc3, 0x69, 0x03, 0xf8, 0xbf, 0x47,
0x01, 0xd4, 0x98, 0x26, 0x1a, 0x08, 0x02, 0xef, 0x63, 0x64, 0x2b, 0xc3,
0x30, 0x1f, 0x06, 0x03, 0x55, 0x1d, 0x23, 0x04, 0x18, 0x30, 0x16, 0x80,
0x14, 0xb1, 0x3e, 0xc3, 0x69, 0x03, 0xf8, 0xbf, 0x47, 0x01, 0xd4, 0x98,
0x26, 0x1a, 0x08, 0x02, 0xef, 0x63, 0x64, 0x2b, 0xc3, 0x30, 0x0d, 0x06,
0x09, 0x2a, 0x86, 0x48, 0x86, 0xf7, 0x0d, 0x01, 0x01, 0x05, 0x05, 0x00,
0x03, 0x82, 0x01, 0x01, 0x00, 0x1c, 0x1a, 0x06, 0x97, 0xdc, 0xd7, 0x9c,
0x9f, 0x3c, 0x88, 0x66, 0x06, 0x08, 0x57, 0x21, 0xdb, 0x21, 0x47, 0xf8,
0x2a, 0x67, 0xaa, 0xbf, 0x18, 0x32, 0x76, 0x40, 0x10, 0x57, 0xc1, 0x8a,
0xf3, 0x7a, 0xd9, 0x11, 0x65, 0x8e, 0x35, 0xfa, 0x9e, 0xfc, 0x45, 0xb5,
0x9e, 0xd9, 0x4c, 0x31, 0x4b, 0xb8, 0x91, 0xe8, 0x43, 0x2c, 0x8e, 0xb3,
0x78, 0xce, 0xdb, 0xe3, 0x53, 0x79, 0x71, 0xd6, 0xe5, 0x21, 0x94, 0x01,
0xda, 0x55, 0x87, 0x9a, 0x24, 0x64, 0xf6, 0x8a, 0x66, 0xcc, 0xde, 0x9c,
0x37, 0xcd, 0xa8, 0x34, 0xb1, 0x69, 0x9b, 0x23, 0xc8, 0x9e, 0x78, 0x22,
0x2b, 0x70, 0x43, 0xe3, 0x55, 0x47, 0x31, 0x61, 0x19, 0xef, 0x58, 0xc5,
0x85, 0x2f, 0x4e, 0x30, 0xf6, 0xa0, 0x31, 0x16, 0x23, 0xc8, 0xe7, 0xe2,
0x65, 0x16, 0x33, 0xcb, 0xbf, 0x1a, 0x1b, 0xa0, 0x3d, 0xf8, 0xca, 0x5e,
0x8b, 0x31, 0x8b, 0x60, 0x08, 0x89, 0x2d, 0x0c, 0x06, 0x5c, 0x52, 0xb7,
0xc4, 0xf9, 0x0a, 0x98, 0xd1, 0x15, 0x5f, 0x9f, 0x12, 0xbe, 0x7c, 0x36,
0x63, 0x38, 0xbd, 0x44, 0xa4, 0x7f, 0xe4, 0x26, 0x2b, 0x0a, 0xc4, 0x97,
0x69, 0x0d, 0xe9, 0x8c, 0xe2, 0xc0, 0x10, 0x57, 0xb8, 0xc8, 0x76, 0x12,
0x91, 0x55, 0xf2, 0x48, 0x69, 0xd8, 0xbc, 0x2a, 0x02, 0x5b, 0x0f, 0x44,
0xd4, 0x20, 0x31, 0xdb, 0xf4, 0xba, 0x70, 0x26, 0x5d, 0x90, 0x60, 0x9e,
0xbc, 0x4b, 0x17, 0x09, 0x2f, 0xb4, 0xcb, 0x1e, 0x43, 0x68, 0xc9, 0x07,
0x27, 0xc1, 0xd2, 0x5c, 0xf7, 0xea, 0x21, 0xb9, 0x68, 0x12, 0x9c, 0x3c,
0x9c, 0xbf, 0x9e, 0xfc, 0x80, 0x5c, 0x9b, 0x63, 0xcd, 0xec, 0x47, 0xaa,
0x25, 0x27, 0x67, 0xa0, 0x37, 0xf3, 0x00, 0x82, 0x7d, 0x54, 0xd7, 0xa9,
0xf8, 0xe9, 0x2e, 0x13, 0xa3, 0x77, 0xe8, 0x1f, 0x4a
};
int digicert_root_crt_len = 969;







// Flash the Certificate BEFORE attempting to connect.
void flashCertificate(char *fileName, _u8* data, _u32 length) {
    // For the purpose of readability this code has no error handling.
    // The simplelink API provides return codes of the type _i32 that can be
    // checked for the value SL_RET_CODE_OK
    // The datetime is required for certificate validation:
	// Certificado de  https://thawing-waters-35250.herokuapp.com/xdk vÃ¡lido hasta el 22 de junio de 2020
    SlDateTime_t dateTime;
    dateTime.sl_tm_day =   (_u32)7;
    dateTime.sl_tm_mon =   (_u32)3;
    dateTime.sl_tm_year =  (_u32)2018;
    dateTime.sl_tm_hour =  (_u32)0;
    dateTime.sl_tm_min =   (_u32)0;
    dateTime.sl_tm_sec =   (_u32)0;
    sl_DevSet(SL_DEVICE_GENERAL_CONFIGURATION,
    		  		SL_DEVICE_GENERAL_CONFIGURATION_DATE_TIME,
			  			sizeof(SlDateTime_t),
			  			(_u8 *)(&dateTime));


    // If a file with the same name already exists, call this first:
    // sl_FsDel((_u8*) CA_FILE_NAME, 0)
    // The file handle should not be 0 after a file was successfully created:
    _i32 fileHandle = 0;
    sl_FsOpen((_u8*) fileName,
              FS_MODE_OPEN_CREATE(1024, _FS_FILE_PUBLIC_WRITE | _FS_FILE_PUBLIC_READ ),
              NULL,
              &fileHandle);


    // If the file is longer than 1024 bytes, you need to loop the writing.
    // "length" contains the length of the certificate
    // "writtenLength" contains the amount of actually written bytes.
    _i32 writtenLen = sl_FsWrite(fileHandle, 0, data, length);
    sl_FsClose(fileHandle, NULL, NULL, 0);
}


void connectServerSecure(void){


	// The simplelink API provides return codes of the type _i32 that can be // checked for the value SL_RET_CODE_OK
	// Getting the IP address of HOST_NAME:
	Ip_Address_T destAddr = 0;
	//printf("Trying to get the IP Address of: %s\n",DEST_SERVER_HOST);
	PAL_getIpaddress((uint8_t*) DEST_SERVER_HOST, &destAddr);
	// Creating a Socket (socketHandle â‰¦ 0 indicates an error):
	_i16 socketHandle = sl_Socket(SL_AF_INET, SL_SOCK_STREAM, SL_SEC_SOCKET);
	// Adding the certificate to the socket:
	sl_SetSockOpt(socketHandle,
				  SL_SOL_SOCKET,
				  SL_SO_SECURE_FILES_CA_FILE_NAME,
				  CA_FILE_NAME,
				  strlen(CA_FILE_NAME));
	// Configuration the connection settings (IP, Port, etc.):
	// printf("DESTINATION ADDRESS::: %s\n",destAddr);
	SlSockAddrIn_t addr;
	addr.sin_family = SL_AF_INET;
	addr.sin_port = sl_Htons(443);
	addr.sin_addr.s_addr = destAddr;
	// Connecting:
	sl_Connect(socketHandle, ( SlSockAddr_t *)&addr, sizeof(SlSockAddrIn_t)); // Implementation Placeholder
	// Do your request, etc. here
	printf("4. Send the POST Request\n");
  //sendGetRequest(socketHandle, DEST_SERVER_HOST, DEST_POST_PATH);
	taskSendPost(socketHandle, DEST_SERVER_HOST, DEST_POST_PATH);
	// Closing:
	//sl_Close(socketHandle); CIERRA EL SOCKET



}

void sendGetRequest(_i16 socketHandle, char* host, char* path)
{
	char outBuf[1024];
	char inBuf[1024];
	sprintf( outBuf,
			"GET https://%s%s HTTP/1.1\r\nHost: %s\r\n\r\n", host, path, host
			);
	sl_Send(socketHandle, (const void *) outBuf, strlen(outBuf), 0);
	sl_Recv(socketHandle, inBuf, 1024, 0);
	printf("%s\r\n", inBuf);
}


// --- HTTP Request ---
// POST /xdk HTTP/1.1
// Host: thawing-waters-35250.herokuapp.com
// Content-Type: application/json
// Content-Length: application/json
//
// {"status":true}


Retcode_T readLightSensor(void){

	uint32_t milliLuxData = UINT32_C(0);
	Retcode_T returnValue = RETCODE_FAILURE;

	returnValue = LightSensor_readLuxData(
			xdkLightSensor_MAX44009_Handle,&milliLuxData);

	if (RETCODE_OK == returnValue)
	{
		printf(" La luminosidad es: %d \n\r", (unsigned
				int) milliLuxData);

	}
	return milliLuxData;

}

Retcode_T readEnvironmentalTemperature(void){
	//(void) xTimer;
	Retcode_T returnValue = RETCODE_FAILURE;

	Environmental_Data_T bme280 = { INT32_C(0), UINT32_C(0), UINT32_C(0) };
	returnValue = Environmental_readData(xdkEnvironmental_BME280_Handle, &bme280);

	int32_t getTemperature = INT32_C(0);
	returnValue = Environmental_readTemperature(xdkEnvironmental_BME280_Handle,&getTemperature);

	printf("La Temperatura es  :=%ld mDeg\n\r", getTemperature);
	return getTemperature;
}
Retcode_T readEnvironmentalHumidity(void){
	Retcode_T returnValue = RETCODE_FAILURE;

	Environmental_Data_T bme280 = { INT32_C(0), UINT32_C(0), UINT32_C(0) };
	returnValue = Environmental_readData(xdkEnvironmental_BME280_Handle, &bme280);

	int32_t getHumidity = INT32_C(0);
	returnValue = Environmental_readHumidity(xdkEnvironmental_BME280_Handle,&getHumidity);

	printf("La Humedad es  :=%ld %%rh\n\r", getHumidity);
	return getHumidity;

}

Retcode_T readEnvironmentalPressure(void){
	Retcode_T returnValue = RETCODE_FAILURE;

	Environmental_Data_T bme280 = { INT32_C(0), UINT32_C(0), UINT32_C(0) };
	returnValue = Environmental_readData(xdkEnvironmental_BME280_Handle, &bme280);

	int32_t getPressure = INT32_C(0);
	returnValue = Environmental_readPressure(xdkEnvironmental_BME280_Handle,&getPressure);

	printf("La Presion es  :=%ld Pa\n\r", getPressure);
	return getPressure;

}

void sendPostRequest(_i16 socketHandle, char* host, char* path)
{
	char buffer[1024];
	_i16 bytesSent = 0;
	_i16 bytesReceived = 0;
	int resLight =readLightSensor();
	int resTemp = readEnvironmentalTemperature();
	int resHumed = readEnvironmentalHumidity();
	int resPres = readEnvironmentalPressure();
	char buffer2[300];
	//*********************//
	//  ojo !!! USER se debe cambiar para cada XDK !!!
	sprintf(buffer2, "{\"pressure\": %d, \"humidity\": %d, \"temperature\": %d, \"illumination\": %d, \"user\": 1}",resPres, resHumed, resTemp, resLight);

	char* body = buffer2;
	char* headers = "";
	int length = 0;

	length += sprintf(buffer+length, "POST %s HTTP/1.1\r\n",DEST_POST_PATH);
	length += sprintf(buffer+length, "Host: %s\r\n",DEST_SERVER_HOST);
	length += sprintf(buffer+length, "Content-Type: application/json\r\n");
	length += sprintf(buffer+length, "Content-Length: %d\r\n\r\n",strlen(body));
	length += sprintf(buffer+length, body);
	length += sprintf(buffer+length, "\r\n");


	printf("--- HTTPS Request --- \r\n");
	printf("%s", buffer);
	printf("\r\n---------------\r\n");
	printf("\r\n---------------\r\n");
	bytesSent = sl_Send(socketHandle, (const void *) buffer, strlen(buffer), 0 );
	if( bytesSent <= 0 ) {
		printf("sl_Send failed: %i\r\n", bytesSent);
		return;
	}

	printf("--- HTTPS Response --- \r\n");
	bytesReceived = sl_Recv(socketHandle, buffer, 1024, 0);
	printf("%s",buffer);
	printf("\r\n---------------\r\n");


}

void taskSendPost(_i16 socketHandle, char* host, char* path)
{
	uint32_t timerBlockTime = UINT32_MAX;
	//*********************//
	//  ojo !!! cambiar el valor de oneSecondDelay para el delay de la toma de sensores y el Request !!!
	uint32_t oneSecondDelay = UINT32_C(5000);
	uint32_t timerAutoReloadOn = pdTRUE;

	xTimerHandle LightSensorHandle = NULL;

    LightSensorHandle = xTimerCreate((const char *) "sendPostRequest",
		    		oneSecondDelay,timerAutoReloadOn, NULL, sendPostRequest);

    xTimerStart(LightSensorHandle,timerBlockTime);
}



// WLAN
static retcode_t WlanConnect(void){

    NetworkConfig_IpSettings_T myIpSettings;
    char ipAddress[PAL_IP_ADDRESS_SIZE] = { 0 };
    Ip_Address_T* IpaddressHex = Ip_getMyIpAddr();
    WlanConnect_SSID_T connectSSID;
    WlanConnect_PassPhrase_T connectPassPhrase;
    Retcode_T ReturnValue = (Retcode_T) RETCODE_FAILURE;
    int32_t Result = INT32_C(-1);

    if (RETCODE_OK != WlanConnect_Init()){
        return (RC_PLATFORM_ERROR);
    }

    printf("Connecting to WiFi network: %s  \n\r", WLAN_CONNECT_WPA_SSID);
    connectSSID = (WlanConnect_SSID_T) WLAN_CONNECT_WPA_SSID;
    connectPassPhrase = (WlanConnect_PassPhrase_T) WLAN_CONNECT_WPA_PASS;
    ReturnValue = NetworkConfig_SetIpDhcp(NULL);
    if (RETCODE_OK != ReturnValue){
        printf("Error in setting IP to DHCP  \n\r");
        return (RC_PLATFORM_ERROR);
    }
    if (RETCODE_OK == WlanConnect_WPA(connectSSID, connectPassPhrase, NULL)){
        ReturnValue = NetworkConfig_GetIpSettings(&myIpSettings);
        if (RETCODE_OK == ReturnValue){
            *IpaddressHex = Basics_htonl(myIpSettings.ipV4);
            Result = Ip_convertAddrToString(IpaddressHex, ipAddress);
            if (Result < 0){
                printf("Couldn't convert the IP address to string format  \n\r");
                return (RC_PLATFORM_ERROR);
            }
            printf("Connected to WPA network successfully  \n\r");
            printf(" Ip address of the device %s  \n\r", ipAddress);
            return (RC_OK);
        }else{
            printf("Error in getting IP settings  \n\r");
            return (RC_PLATFORM_ERROR);
        }
    }else{
        return (RC_PLATFORM_ERROR);
    }

}

static void setupWlan(void) {
	retcode_t rc = RC_OK;
	rc = WlanConnect();
	if (RC_OK != rc){ printf("Network init/connection failed %i  \n\r", rc); return; }
	rc = PAL_initialize();
	if (RC_OK != rc){ printf("PAL and network initialize %i \r\n", rc); return; }
	PAL_socketMonitorInit();
	rc = HttpClient_initialize();
	if (rc != RC_OK){ printf("Failed to initialize http client \r\n "); return; }
}

static void initLightSensor(void){
	Retcode_T returnValue = RETCODE_FAILURE;
	returnValue = LightSensor_init(xdkLightSensor_MAX44009_Handle);
	if(RETCODE_OK != returnValue){
		printf("Light Sensor initialization Failed");
	}
}
static void initEnviromentalSensor(void){
	Retcode_T returnValue = RETCODE_FAILURE;
	returnValue = Environmental_init(xdkEnvironmental_BME280_Handle);
	if(RETCODE_OK != returnValue){
		printf("Light Sensor initialization Failed");
	}
}

Retcode_T init(void)
{
    Retcode_T retval = RETCODE_OK;
    static_assert((portTICK_RATE_MS != 0), "Tick rate MS is zero");

    /*initializacion de sensores*/
    initLightSensor();
    initEnviromentalSensor();

    return retval;
}


// * - Main Function
void appInitSystem(void * CmdProcessorHandle, uint32_t param2)
{
	if (CmdProcessorHandle == NULL)
	    {
	        printf("Command processor handle is null \n\r");
	        assert(false);
	    }
	    BCDS_UNUSED(param2);
	    Retcode_T retVal = RETCODE_OK;


	    printf("1. Setup WLAN and connect to Wi-Fi\n");
	    setupWlan();

	    printf("2. Flashing certificate\n");
	  	flashCertificate(CA_FILE_NAME, digicert_root_crt, digicert_root_crt_len);

	  	retVal = init();
	  	    if (RETCODE_OK == retVal)
	  	    {
	  	        printf(" Initialization completed successfully \r\n ");
	  	    }
	  	    else
	  	    {
	  	        printf(" Initialization failed \r\n ");
	  	    }

	  	printf("3. Connect to server with certificate and send the POST request\n");
	  	connectServerSecure();
}

