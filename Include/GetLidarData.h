#pragma once
/******************** (C) COPYRIGHT 2023 Shenzhen LeiShen Intelligent System Co., Ltd. *****************

* File Name         : LSLidarDemo.h
* Author            : LSJ
* Version           : v2.1.0
* Date              : 2023/09
* Modify Record		: 
* Description       : Initial Version


*******************************************************************************
* Version           : v2.1.1
* Date              : 2023/10
* Modify Record		: C16 v4.0 bug
* Description       : C16 v4.0 fixed the incorrect display of coordinate conversions


*******************************************************************************
* Version           : v2.1.2
* Date              : 2024/02
* Modify Record		: Change the function that gets a frame of data
* Description       : std::shared_ptr<std::vector<MuchLidarData>> getLidarPerFrameDate();   
					  (Change) ->
                      bool getLidarPerFrameDate(std::shared_ptr<std::vector<MuchLidarData>>& preFrameData, std::string& Info);

*******************************************************************************
********************************************************************************
* Version           : v2.1.3
* Date              : 2024/03
* Modify Record		: All set functions add message feedback
* Description       : example:
					  bool setLidarRotateSpeed(int SpeedValue);			
					  (Change) ->
                     bool setLidarRotateSpeed(int SpeedValue, std::string& InfoString); 

*******************************************************************************
********************************************************************************
* Version           : v2.1.4
* Date              : 2024/04
* Modify Record		: 1：Add function to get Lidar parameter read-back
					  2：Add IO multiplexing
					  3：Add NTP IP settings
					  4：Add lidar filter display Settings
* Description       : bool getLidarParamState(LidarStateParam& mLidarStateParam, std::string& InfoString); 
					  bool setNTP_IP(std::string IPString, std::string& InfoString);	
					  bool  setLidarFilterdisplay(LidaFilterParamDisplay mLidaFilterParamDisplay);

*******************************************************************************
********************************************************************************
* Version           : v2.1.5
* Date              : 2024/05
* Modify Record		: 1：Change the CMakeLists.txt file 				
* Description       : 1：Change the file CMakeLists.txt to merge the 3 examples into
					  
					  
*******************************************************************************
********************************************************************************
* Version           : v2.1.6
* Date				: 2024 / 07
* Modify Record :	1：Incorporates the SDK for C_v4.0
* 					2：Add set frame rate mode
					3：Add set phase locked switch

* Description :		1：Incorporates the SDK for C_v4.0
*                   2: Increment function:  bool  setFrameRateMode(int StateValue, std::string& InfoString);
					3: Increment function:  bool  setPhaseLockedSwitch(int StateValue, std::string& InfoString);


* ******************************************************************************
********************************************************************************
* * Version           : v2.1.7
* Date				: 2024 / 08
* Modify Record :	1：Added LS fault code output
* Description :		2：Added fault code output for LS series Lidar

* ******************************************************************************
********************************************************************************
* * Version           : v2.1.8
* Date				: 2024 / 10
* Modify Record :	1：Added new LiDAR type MS01
					2：Add configuration parameters
* Description :		

* ******************************************************************************
* ******************************************************************************/

#include <iostream>
#include <vector>
#include <thread>
#include <queue>
#include <mutex>
#include <cmath>
#include <chrono>
#include <cstring>
#include <functional>
#include <string>
#include <regex>
#include <memory>

#include <sstream>  
#include <iomanip>
#ifdef LINUX
#include <sys/types.h>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#include <sys/epoll.h>
#include <fcntl.h>
#include <errno.h>
typedef uint64_t u_int64;
#else
#include <WinSock2.h>
#include <WS2tcpip.h>
#pragma comment(lib, "ws2_32.lib")

struct SocketContext {
    SOCKET socket;
    char buffer[1212];
    WSAOVERLAPPED overlapped;
    WSABUF dataBuf;
    SOCKADDR_IN remoteAddr;         //存储数据来源IP地址
};

typedef unsigned __int64 u_int64;

#define timegm _mkgmtime
#endif

#define PI 3.1415926
#define ConversionUnit	100.0			//unit conversion: convert the centimeter to meter

typedef struct _Point_XYZ
{
	float x1 = 0.0;
	float y1 = 0.0;
	float z1 = 0.0;

	float x2 = 0.0;
	float y2 = 0.0;
	float z2 = 0.0;
}m_PointXYZ;

typedef struct _MuchLidarData
{
	float X = 0.0;
	float Y = 0.0;
	float Z = 0.0;
	int ID = 0;
	float H_angle = 0.0;
	float V_angle = 0.0;
	float Distance = 0.0;
	int Intensity = 0;
	u_int64 Mtimestamp_nsce = 0;
}MuchLidarData;

typedef struct _UTC_Time
{
	int year = 2000;
	int month = 0;
	int day = 0;
	int hour = 0;
	int minute = 0;
	int second = 0;
}UTC_Time;

typedef struct _LidarStateParam
{
	std::string LidarIP = "-1,-1,-1,-1";			//Lidar IP
	std::string ComputerIP = "-1,-1,-1,-1";			//Lidar destination IP（computer IP）
	std::string NtpIP = "-1,-1,-1,-1";				//Lidar NTP IP
	std::string GatewayIP = "-1,-1,-1,-1";			//Lidar Gateway IP
	std::string SubnetMaskIP = "-1,-1,-1,-1";		//Lidar Subnet Mask IP
	std::string MacAddress = "-1,-1,-1,-1,-1,-1";	//Lidar Mac Address				50 3E 7C ** ** **
	int DataPort = -1;								//Lidar data pakcet port
	int DevPort = -1;								//Lidar device pakcet port

	float ReceiverTemperature = -1.0;				//Receiver plate temperature    Unit:°
	float ReceiverHighVoltage = -1.0;				//Receiving plate high voltage  Unit:volt
	float MotorSpeed = -1;							//Motor speed					Unit:revolution
	int   FrameRateMode = -1;						//Frame rate Mode				Value: 0; 1; 2

	int	  PTP_State = -1;							//PTP State:					1:Losed;	0: Not lost 
	int	  GPS_State = -1;							//GPS State:					1:Losed;	0: Not lost 
	int	  PPS_State = -1;							//PPS State:					1:Losed;	0: Not lost 
	int   StandbyMode = -1;							//Standby Mode					0:Normal	1: Standby
	int   Clock_Source = -1;						//Clock Source:					0:GPS;		1: PTP_L2;	 2:NTP;	3:PTP_UDPV4
	int   PhaseLockedSwitch = -1;					//Phase Locked Swich:			0:Closing;	1: Opening
	float PhaseLockedAngle = -1;					//Phase Locked Hanle:			Phase Locked Hanle Value
	int   PhaseLockedState = -1;					//Phase Locked State:			0:Unlocked; 1: Locked
	std::string FaultCode = "-1";					//Fault Code					Each one bit a fault state
	double RunningTime = -1;						//Running Time					Unit: hour
}LidarStateParam;

typedef struct _LidaFilterParamDisplay
{
	float mMin_HanleValue;		//Unit:°
	float mMax_HanleValue;		//Unit:°

	float mMin_VanleValue;		//Unit:°
	float mMax_VanleValue;		//Unit:°

	float mMin_Distance;		//Unit:m
	float mMax_Distance;		//Unit:m

	float mMin_Intensity;
	float mMax_Intensity;
	std::vector<int> mChannelVector;		//The value is 1 for display and 0 for filtering

	_LidaFilterParamDisplay()
	{
		mMin_HanleValue = -360;
		mMax_HanleValue = 360;

		mMin_VanleValue = -180;
		mMax_VanleValue = 180;

		mMin_Distance = 0;
		mMax_Distance = 1000;

		mMin_Intensity = 0;
		mMax_Intensity = 255;

		mChannelVector.resize(256);
		for (long int FF = 0; FF < 256; FF++)
		{
			mChannelVector[FF] = 1;
		}
	}

	//_LidaFilterParamDisplay(const _LidaFilterParamDisplay& obj)
	//{
	//	*this = obj;
	//}

	_LidaFilterParamDisplay& operator=(const _LidaFilterParamDisplay& obj)
	{
		if (&obj == this)
		{
			return *this;
		};

		mMin_HanleValue = obj.mMin_HanleValue;
		mMax_HanleValue = obj.mMax_HanleValue;
										 ;
		mMin_VanleValue = obj.mMin_VanleValue;
		mMax_VanleValue = obj.mMax_VanleValue;
										 ;
		mMin_Distance	= obj.mMin_Distance	 ;
		mMax_Distance	= obj.mMax_Distance	 ;
										 ;
		mMin_Intensity	= obj.mMin_Intensity ;
		mMax_Intensity	= obj.mMax_Intensity ;

		for (long int FF = 0; FF < obj.mChannelVector.size(); FF++)
		{
			mChannelVector[FF] = obj.mChannelVector[FF];
		}

		for (long int FF = obj.mChannelVector.size(); FF < 256; FF++)
		{
			mChannelVector[FF] = 1;
		}
		return *this;
	}

}LidaFilterParamDisplay;

typedef std::function<void(std::shared_ptr<std::vector<MuchLidarData>>, int, std::string)> FunDataPrt;

class GetLidarData
{
public:
	GetLidarData();
	~GetLidarData();
	int setIndex = -1;

	bool isQuit = false;												//a mark that lidar quits
	bool isFrameOK = false;
	int isDormant = 0;                                                   //whether lidar is in low power mode or not, 0: normal mode, 1:low power mode
	bool isFrames = true;                                               //frame synchronization judgement，true: normal
	bool isHighVoltage = true;                                          //whether voltage is normal, true: normal
	bool islidarDevCome = false;													//judge whether obtain lidar device packet

	double cosAngleValue[360000];										//Pre-calculate the Angle values of cos and sin	
	double sinAngleValue[360000];

	std::string mDataInfoString;										//Message feedback of data packet
	std::string mDevInfoString;											//Message feedback of dev packet
	bool isSuccessfulFlag;												//Whether the data is successfully obtained

	LidaFilterParamDisplay mLidaFilterParamDisplayValue;

	/*
		 Function: Get data packet status
		 parameter:none
		 return value: std::string：Returns the status information of the data packet 
	*/
	std::string getDataPacketState();

	/*
		 Function: Get dev packet status
		 parameter:none
		 return value: std::string：Returns the status information of the dev packet
	*/
	std::string getDevPacketState();		

	/*
	 Function: Obtain Lidar parameter status
	 parameter: LidarStateParam: Lidar status parameter feedback;			InfoString:Message feedback
	 return value: true, operation succeeded; false: operation failed, View InfoString
	*/
	virtual bool getLidarParamState(LidarStateParam& mLidarStateParam, std::string& InfoString);

	/*
		Function: Initializes the data packet port, device packet port, and destination IP address
		parameter 1: mDataPort		Data packet port		2368 (by default)
		parameter 2: mDevPort		Device packet port		2369 (by default)
		parameter 3: mDestIP		Destination IP			"192.168.1.102"(by default)
		parameter 4: mLidarIP		LidarIP IP				"192.168.1.200"(by default)
		parameter 5: mGroupIp		Multicast IP			"224.1.1.102"(by default)
		return value: none
		
	*/
	void setPortAndIP(uint16_t mDataPort = 2368, uint16_t mDevPort = 2369, std::string mDestIP = "192.168.1.102", std::string mLidarIP = "192.168.1.200",std::string mGroupIp = "226.1.1.102");
	

	/*
		Function: Set the radar fixed IP, radar IP, destination IP packet port and device port
		parameter 1:   mFixedLidarIP	Radar fixed IP			"10.10.10.254"   (by default)
		parameter 2:   mLidarIP		    Lidar IP				 "192.168.1.200"  (by default)
		parameter 3:   mDestIP		    Destination IP			 "192.168.1.102"  (by default)
		parameter 4:   mDataPort	    Data packet port	     2368			  (by default)
		parameter 5:   mDevPort		    Device packet port	     2369			  (by default)
		return: none
	**************  Only CX6S3 is currently available  *********/
	virtual void setChangeLidarIP(std::string mFixedLidarIP = "10.10.10.254", std::string mLidarIP = "192.168.1.200", std::string mDestIP = "192.168.1.102", uint16_t mDataPort = 2368, uint16_t mDevPort = 2369);

	/*
		Function: transmit callback function, obtain lidar data
		parameter 1: FunDataPrt*	 introduce callback function pointer, obtain lidar data
		return value: none
	*/
	void setCallbackFunction(FunDataPrt*);												//set data point callback function
	
	/*
		 Function: star parsing lidar data
		 parameter:none
		 return value: none
	 */
	void LidarStart();
	
	/*
	 Function: stop parsing lidar data 
	 parameter:none
	 return value: none
	*/
	void LidarStop();
	
	/*
		 Function: start offline data parsing of network data packet 
		 parameter:none
		 return value: none 
	*/
	void LidarOfflineDataStar();

	/*
	 Function:obtain lidar data of one frame 
	 parameter: preFrameData:  obtain point cloud data of one frame; Info:information
	 return value: true, operation succeeded; false: operation failed
	*/
	bool getLidarPerFrameDate(std::shared_ptr<std::vector<MuchLidarData>>& preFrameData, std::string& Info);
	
	virtual void LidarRun() = 0;
	void CollectionDataArrive(void* pData, uint16_t len);								//transmit UDP data here

	int NegativeToPositive(float);														//Negative numbers are converted to positive numbers
	
	std::string ucharToBinaryStr(const unsigned char);

public:

#pragma region //set lidar parameters
	 unsigned char dataDev[1206];														//receive device packet
	 unsigned char Rest_UCWP_buff[1206];												//modify configuration packet
	 std::string ip_sa;																	//send lidar IP
	 int m_SpeedValue = 0;																//set lidar rotate speed
	 bool is_speedFlag = false;															//whether enable rotating speed detection, true: enable
	 bool is_speedNormal = true;														//whether rotating speed is normal, true: normal

	std::string IntToHex(int value) {
		std::ostringstream ss;  
		ss << std::setw(2) << std::setfill('0') << std::hex << std::uppercase << value;
		return ss.str();
	}
	 /*
	 Function: mofify lidar rotating speed
	 parameter: SpeedValue				rotating speed value: support 300， 600，1200;  InfoString:Message feedback
	 return value: true, operation succeeded; false: operation failed

	 Attention: LS Lidar is not applicable
	 */
	virtual bool setLidarRotateSpeed(int SpeedValue, std::string& InfoString);						//set lidar rotating speed
	
	/*
	Function: mofify lidar IP
	parameter: IPString	input the IPv4 address to modify, i.e. "192.168.1.200";  InfoString:Message feedback
	return value: true, operation succeeded; false: operation failed
	*/
	virtual bool setLidarIP(std::string IPString, std::string& InfoString);							//set lidar IP
	
	/*
	Function: mofify lidar destination IP（computer IP）
	parameter: IPString	input the IPv4 address to modify, i.e. "192.168.1.102";  InfoString:Message feedback
	return value: true, operation succeeded; false: operation failed
	*/
	virtual bool setComputerIP(std::string IPString, std::string& InfoString);						//set computer IP

	/*
	Function: mofify lidar NTP IP
	parameter: IPString	input the IPv4 address to modify, i.e. "192.168.1.102";  InfoString:Message feedback
	return value: true, operation succeeded; false: operation failed
	*/
	virtual bool setNTP_IP(std::string IPString, std::string& InfoString);							//set NTP IP

	/*
	Function: mofify lidar Gateway IP（Gateway IP）
	parameter: IPString	input the IPv4 address to modify, i.e. "192.168.1.254";  InfoString:Message feedback
	return value: true, operation succeeded; false: operation failed
	*/
	virtual bool setGatewayIP(std::string IPString, std::string& InfoString);						//set Gateway IP

	/*
	Function: mofify lidar Subnet Mask IP（Subnet Mask IP）
	parameter: IPString	input the IPv4 address to modify, i.e. "255.255.255.0";  InfoString:Message feedback
	return value: true, operation succeeded; false: operation failed
	*/
	virtual bool setSubnetMaskIP(std::string IPString, std::string& InfoString);					//set Subnet Mask IP
	
	/*
	Function: modify lidar data pakcet port
	parameter: PortNum, input lidar data port,0~65536 can be input by default（a value larger than 2000 is recommended）;  InfoString:Message feedback
	return value: true, operation succeeded; false: operation failed
	*/
	virtual bool setDataPort(int PortNum, std::string& InfoString);									//set data pakcet port
	
	/*
	Function:modify the device packet port
	parameter: PortNum, input lidar data port,0~65536 can be input by default（a value larger than 2000 is recommended）;  InfoString:Message feedback
	return value: true, operation succeeded; false: operation failed
	*/
	virtual bool setDevPort(int PortNum, std::string& InfoString);									//set device packet
	
	/*
	Function: modify the Lidar's motor state
	Parameter: StateValue, input the mode of Lidar's motor, 0 means: running (by default) , 1: stationary;	InfoString:Message feedback
	return value: true, operation succeeded; false: operation failed
	*/
	virtual bool setLidarRotateState(int StateValue, std::string& InfoString);						//set lidar motor state: running or stationary 
	
	/*
	Function:modify Lidar's timing source selection
	Parameter:StateValue: input the source selection, by default, 0:GPS; 1: PTP_L2; 2:NTP;	3:PTP_UDPV4;	InfoString:Message feedback
	Returned value: true: operation succeeded; false: operation failed
	*/
	virtual bool setLidarSoureSelection(int StateValue, std::string& InfoString);					//set timing source

	/*
	Function:modify Lidar's low power mode, which means the lidar only sends device packet without data packet)
	Parameter:StateValue: input the mode, by default, 0:normal operation mode, 1:low power mode;			 InfoString:Message feedback
	Returned value: true: operation succeeded; false: operation failed
	*/
	virtual bool  setLidarWorkState(int StateValue, std::string& InfoString);						//lidar state: in low power mode or not
	
	/*
	Function: modify Lidar frame rate mode
	Parameter:StateValue: input the mode, by default, 0:normal frame rate; 1: 50% frame rate;	1: 25% frame rate;	 InfoString:Message feedback
	Returned value: true: operation succeeded; false: operation failed
	
	Attention: Only applicable to LS Lidar
	*/
	virtual bool  setFrameRateMode(int StateValue, std::string& InfoString);						//set lidar frame rate mode

	/*
	Function: modify Lidar phase locked switch
	Parameter:StateValue: input the mode, by default, 0:Closing;	1: Opening;	 InfoString:Message feedback
	Returned value: true: operation succeeded; false: operation failed

	Attention: Only applicable to LS Lidar
	*/
	virtual bool  setPhaseLockedSwitch(int StateValue, std::string& InfoString);						//set Lidar phase locked switch

	/*
	Function: mofify lidar index parameter values
	parameter: ControlValue: Index				ChangeValue: change value;	 InfoString:Message feedback
	return value: true, operation succeeded; false: operation failed
	*/
	virtual bool setLidarIndexParamValue(int ControlValue, int ChangeValue, std::string& InfoString);

	/*
	Function: send UDP packet, Lidar data
	Parameter: none
	Returned value: true: operation succeeded; false: operation failed
	*/
	virtual bool sendPackUDP();																		//set UDP packet
	
	/*
	Function:  The second IP control switch
	Parameter: 0 indicates enable and 1 indicates disable
	Returned value: true: operation succeeded; false: operation failed
	*/
	virtual bool sendSecondIP(int switchIP, std::string& InfoString);								 //The second IP control switch is available only to cx6s3

	/*
	Function:  Cue light control
	Parameter: 1 indicates enable and 0 indicates disable	;										InfoString:Message feedback
	Returned value: true: operation succeeded; false: operation failed
	*/
	virtual bool sendSecondCueLight(int switchIP, std::string& InfoString);							//Available only for CX6S3

	bool setLidarParam();																			//set package
	bool sendPacketToLidar(unsigned char* packet, const char* ip_data, u_short portdata);			//send packet via socket
#pragma endregion 
	
#pragma region//lidar filter display Settings
	/*
	Function:  Displays the filtered horizontal Angle	
	Parameter: LidaFilterParamDisplay : Input Lidar dar filter display parameters;	
	Returned value: true: operation succeeded; false: operation failed
	*/
	virtual bool  setLidarFilterdisplay(LidaFilterParamDisplay mLidaFilterParamDisplay);

	virtual bool isPointFilter(const MuchLidarData mPoint);						//Returned value : true : Filtration ;  false: Retention
#pragma endregion

public:
		u_int64 timestamp_nsce;																	//timestamp shorter than 1 second in the packet(unit: nanosecond)
		u_int64 allTimestamp;																	//save the timestamp in the packet
		u_int64 lastAllTimestamp = 0;															//save the timestamp of the previous packet
		double PointIntervalT_ns;																//time interval between each point in the packet, unit is nanosecond
		int m_DistanceIsNotZero = 0;
protected:
	std::queue<unsigned char *> allDataValue;													//data cache queue
	std::mutex m_mutex;																			//lock

	int m_messageCount = 0;
	std::shared_ptr<std::vector<MuchLidarData>> LidarPerFrameDatePrt_Get;						// obtain data
	std::shared_ptr<std::vector<MuchLidarData>> LidarPerFrameDatePrt_Send;						// send data
	
	UTC_Time m_UTC_Time;																		//obtain the GPS information of device packet
	void clearQueue(std::queue<unsigned char *>& m_queue);										//clear queue
	void sendLidarData();																		//packaging and send data
	FunDataPrt *callback = NULL;																//callback function pointer--callback data packet
	std::string time_service_mode = "gps";                                                     //timing method
	
	void messFunction(std::string strValue, int gValue);
	//Check whether the source IP matches or not
	bool checkDefaultIP(std::vector<std::string> m_DefaultIP, std::string& InfoString);
	//Check whether the destination IP matches or not
	bool checkDestIP(std::vector<std::string> m_DestIP, std::string& InfoString);
	
	//start rotating speed judgement after a while
	void sleepTime();
	//start rotating speed judgement 
	void startSleepThread();

private:
	std::shared_ptr<std::vector<MuchLidarData>> LidarPerFrameDatePer;
	bool isSendUDP = true;

	int dataPort = 2368;															//data packet port
	int devPort = 2369;																//device packet port
	std::string computerIP = "192.168.1.102";										//Lidar destination IP (computer IP)
	std::string lidarIP = "192.168.1.200";											//Lidar lidarIP IP (lidar IP)
	std::string groupIp = "226.1.1.102";											//Define Multicast IP

#ifdef LINUX
	int fd_;

	//start thread to obtain device packet
	int LinuxSockCreate();
#else
	SOCKET m_SockData;
	SOCKET m_SockDev;
	
	//start thread to obtain data packet
	void WindowSockCreate();
#endif
};
