#define _WINSOCK_DEPRECATED_NO_WARNINGS
#define UsageMethod_1 1

#include <IncludeFile.h>
//ConnectionParameter
int cDataPort = 2368;
int cDevPort = 2369;
std::string cLidarIP = "192.168.1.200";
std::string cDestIP = "192.168.1.102";
std::string cGroupIp = "226.1.1.102";


//ModifyParameter
int mDataPort = 2368;
int mDevPort = 2369;
std::string mLidarIP = "192.168.1.200";
std::string mDestIP = "192.168.1.102";
std::string mNtpIP = "";
std::string mGatewayIP = "";
std::string mSubnetMaskIP = "";

#if UsageMethod_1

#pragma region 		//method one, get one frame of data through flag bit
int main(int argc, char* argv[])
{
	std::cout << "The number of command line arguments:" << argc << std::endl;

	if (argc >= 3)
	{
		for (int i = 0; i < argc; ++i) {
			std::cout << "parameters:  " << i << ": " << argv[i] << std::endl;
		}

		m_GetLidarData->setIndex = atoi(argv[1]);

		m_GetLidarData->setPortAndIP(cDataPort, cDevPort, cDestIP, cLidarIP, cGroupIp);			//set the parameters of the Ethernet port which receives the lidar data（port number of data package, device package, destination IP)
		m_GetLidarData->LidarStart();											//start parsing lidar data

		std::this_thread::sleep_for(std::chrono::milliseconds(3000));			//wait for a while, get device package
		std::string InfoString;

		if (m_GetLidarData->setIndex > 0)
		{
			m_GetLidarData->setLidarIndexParamValue(atoi(argv[1]), atoi(argv[2]), InfoString);
		}
		else
		{
			m_GetLidarData->setLidarIP(mLidarIP, InfoString);
			m_GetLidarData->setComputerIP(mDestIP, InfoString);
			m_GetLidarData->setDataPort(mDataPort, InfoString);
			m_GetLidarData->setDevPort(mDevPort, InfoString);

			if (mNtpIP != "") m_GetLidarData->setNTP_IP(mNtpIP, InfoString);
			if (mGatewayIP != "") m_GetLidarData->setNTP_IP(mGatewayIP, InfoString);
			if (mSubnetMaskIP != "") m_GetLidarData->setNTP_IP(mSubnetMaskIP, InfoString);
		}
		m_GetLidarData->sendPackUDP();			 			//after setting parameters, "sendPackUDP" must be called to send UDP package

		std::this_thread::sleep_for(std::chrono::milliseconds(5000));
		m_GetLidarData->LidarStop();
		return 0;
	}

	m_GetLidarData->setPortAndIP(cDataPort, cDevPort, cDestIP, cLidarIP, cGroupIp);	
	//m_GetLidarData->setPortAndIP(2368, 2369, "192.168.1.102", "192.168.1.200", "226.1.1.102");			//set the parameters of the Ethernet port which receives the lidar data（port number of data package, device package, destination IP)
	m_GetLidarData->LidarStart();											//start parsing lidar data


#pragma region //modify lidar parameters and parameters can be modified individually 

	//std::string mInfo;
	//std::this_thread::sleep_for(std::chrono::milliseconds(2000));			//wait for a while, get device package
	//m_GetLidarData->setLidarRotateSpeed(600, mInfo);
	//m_GetLidarData->setLidarIP("192.168.1.200", mInfo);
	//m_GetLidarData->setComputerIP("192.168.1.102", mInfo);
	//m_GetLidarData->setDataPort(2368, mInfo);
	//m_GetLidarData->setDevPort(2369, mInfo);

	//m_GetLidarData->setLidarRotateState(0, mInfo);
	//m_GetLidarData->setLidarSoureSelection(0, mInfo);
	//m_GetLidarData->setLidarWorkState(0, mInfo);
	//m_GetLidarData->sendSecondCueLight(0, mInfo);                                   //CX6S3 is valid
	//m_GetLidarData->sendSecondIP(1, mInfo);                                        //The second IP control switch is available only to cx6s3
	//
	// m_GetLidarData->sendPackUDP();												//after setting parameters, "sendPackUDP" must be called to send UDP package
	//
	//m_GetLidarData->setChangeLidarIP("10.10.10.254","192.168.1.200", "192.168.1.102", 2368, 2369);   // Before changing the IP address, you need to enable the second IP control switch. Currently, only CX6S3 is available


	//LidarStateParam mLidarStateParam;
	//std::string mInfo1;
	//std::this_thread::sleep_for(std::chrono::milliseconds(2000));			//wait for a while, get device package
	//if (!m_GetLidarData->getLidarParamState(mLidarStateParam, mInfo1))				//Lidar parameter query
	//{
	//	std::cout << mInfo1 << std::endl;
	//}
	//else
	//{
	//	std::cout << "********** Lidar Parameters Display Start **********" << std::endl;

	//	std::cout.width(20); std::cout << "LidarIP	=	" << mLidarStateParam.LidarIP << std::endl;
	//	std::cout.width(20); std::cout << "ComputerIP	=	" << mLidarStateParam.ComputerIP << std::endl;
	//	std::cout.width(20); std::cout << "GatewayIP	=	" << mLidarStateParam.GatewayIP << std::endl;
	//	std::cout.width(20); std::cout << "SubnetMaskIP	=	" << mLidarStateParam.SubnetMaskIP << std::endl;
	//	std::cout.width(20); std::cout << "MacAddress	=	" << mLidarStateParam.MacAddress << std::endl;
	//	std::cout.width(20); std::cout << "DataPort	=	" << mLidarStateParam.DataPort << std::endl;
	//	std::cout.width(20); std::cout << "DevPort	=	" << mLidarStateParam.DevPort << std::endl;

	//	std::cout.width(20); std::cout << "ReceiverTemperature	=	" << mLidarStateParam.ReceiverTemperature << std::endl;
	//	std::cout.width(20); std::cout << "ReceiverHighVoltage	=	" << mLidarStateParam.ReceiverHighVoltage << std::endl;
	//	std::cout.width(20); std::cout << "MotorSpeed	=	" << mLidarStateParam.MotorSpeed << std::endl;
	//	std::cout.width(20); std::cout << "FrameRateMode	=	" << mLidarStateParam.FrameRateMode << std::endl;
		
	//	std::cout.width(20); std::cout << "PTP_State	=	" << mLidarStateParam.PTP_State << std::endl;
	//	std::cout.width(20); std::cout << "GPS_State	=	" << mLidarStateParam.GPS_State << std::endl;
	//	std::cout.width(20); std::cout << "PPS_State	=	" << mLidarStateParam.PPS_State << std::endl;
	//	std::cout.width(20); std::cout << "StandbyMode	=	" << mLidarStateParam.StandbyMode << std::endl;
	//	std::cout.width(20); std::cout << "Clock_Source	=	" << mLidarStateParam.Clock_Source << std::endl;
	//	std::cout.width(20); std::cout << "PhaseLockedSwitch	=	" << mLidarStateParam.PhaseLockedSwitch << std::endl;
	//	std::cout.width(20); std::cout << "PhaseLockedState	=	" << mLidarStateParam.PhaseLockedState << std::endl;
	//	std::cout.width(20); std::cout << "RunningTime	=	" << mLidarStateParam.RunningTime << std::endl;

	//	std::cout << "" << "********** Lidar Paramet<< std::cout.width(20)ers Display End **********" << std::endl << std::endl;
	//}
#pragma endregion 


	//get point cloud and process it with algorithm    //circular process
#pragma region   //get point cloud and process it with different algorithm,or display it
	while (true)
	{
		//method one, get one frame of data
		if (m_GetLidarData->isFrameOK)
		{
			std::shared_ptr<std::vector<MuchLidarData>> m_LidarData_temp;
			std::string mInfo;
			if (!m_GetLidarData->getLidarPerFrameDate(m_LidarData_temp, mInfo))
			{
				std::cout << mInfo << std::endl;
				std::this_thread::sleep_for(std::chrono::milliseconds(1));
				continue;
			}

			//output the number of point cloud
			std::cout << m_LidarData_temp->size() << std::endl;
		}
		else
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
	}
#pragma endregion

	m_GetLidarData->LidarStop();
}

#else

#pragma region 		//method two: get one frame data through callback function
std::mutex m_mutex;
std::shared_ptr<std::vector<MuchLidarData>> m_LidarData = nullptr;									//one frame data

//callback function
void callbackFunction_123(std::shared_ptr<std::vector<MuchLidarData>>, int, std::string);				//callback function, get data of each frame

//class function as callback function
class A
{
public:
	void callbackFunction(std::shared_ptr<std::vector<MuchLidarData>>, int, std::string);	//callback function, get data of each frame

	FunDataPrt fun;
	FunDataPrt* bind()
	{
		fun = std::bind(&A::callbackFunction, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
		return &fun;
	}
};

int main(int argc, char* argv[])
{
	std::cout << "The number of command line arguments:" << argc << std::endl;

	if (argc >= 3)
	{
		for (int i = 0; i < argc; ++i) {
			std::cout << "parameters:  " << i << ": " << argv[i] << std::endl;
		}

		m_GetLidarData->setIndex = atoi(argv[1]);

		m_GetLidarData->setPortAndIP(cDataPort, cDevPort, cDestIP, cLidarIP, cGroupIp);			//set the parameters of the Ethernet port which receives the lidar data（port number of data package, device package, destination IP)
		m_GetLidarData->LidarStart();											//start parsing lidar data

		std::this_thread::sleep_for(std::chrono::milliseconds(3000));			//wait for a while, get device package
		std::string InfoString;

		if (m_GetLidarData->setIndex > 0)
		{
			m_GetLidarData->setLidarIndexParamValue(atoi(argv[1]), atoi(argv[2]), InfoString);
		}
		else
		{
			m_GetLidarData->setLidarIP(mLidarIP, InfoString);
			m_GetLidarData->setComputerIP(mDestIP, InfoString);
			m_GetLidarData->setDataPort(mDataPort, InfoString);
			m_GetLidarData->setDevPort(mDevPort, InfoString);

			if (mNtpIP != "") m_GetLidarData->setNTP_IP(mNtpIP, InfoString);
			if (mGatewayIP != "") m_GetLidarData->setNTP_IP(mGatewayIP, InfoString);
			if (mSubnetMaskIP != "") m_GetLidarData->setNTP_IP(mSubnetMaskIP, InfoString);
		}
		m_GetLidarData->sendPackUDP();			 			//after setting parameters, "sendPackUDP" must be called to send UDP package

		std::this_thread::sleep_for(std::chrono::milliseconds(5000));
		m_GetLidarData->LidarStop();
		return 0;
	}

	//class member function as callback function
	A m_a;
	FunDataPrt fun = std::bind(&A::callbackFunction, m_a, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

	//ordinary callback function
	//FunDataPrt fun = std::bind(callbackFunction_123, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
	
	m_GetLidarData->setCallbackFunction(&fun);							//set callback function
	m_GetLidarData->setPortAndIP(cDataPort, cDevPort, cDestIP, cLidarIP, cGroupIp)		//set the parameters of the Ethernet port which receives the lidar data（port number of data package, device package, destination IP)
	m_GetLidarData->LidarStart();										//start parsing lidar data

#pragma region //modify lidar parameters and parameters can be modified individually 

	//std::string mInfo;
	//std::this_thread::sleep_for(std::chrono::milliseconds(2000));			//wait for a while, get device package
	//m_GetLidarData->setLidarRotateSpeed(600, mInfo);
	//m_GetLidarData->setLidarIP("192.168.1.200", mInfo);
	//m_GetLidarData->setComputerIP("192.168.1.102", mInfo);
	//m_GetLidarData->setDataPort(2368, mInfo);
	//m_GetLidarData->setDevPort(2369, mInfo);

	//m_GetLidarData->setLidarRotateState(0, mInfo);
	//m_GetLidarData->setLidarSoureSelection(0, mInfo);
	//m_GetLidarData->setLidarWorkState(0, mInfo);
	//m_GetLidarData->sendSecondCueLight(0, mInfo);                                   //CX6S3 is valid
	//m_GetLidarData->sendSecondIP(1, mInfo);                                        //The second IP control switch is available only to cx6s3
	//
	// m_GetLidarData->sendPackUDP();										//after setting parameters, "sendPackUDP" must be called to send UDP package

	//m_GetLidarData->setChangeLidarIP("10.10.10.254","192.168.1.200", "192.168.1.102", 2368, 2369);   // Before changing the IP address, you need to enable the second IP control switch. Currently, only CX6S3 is available

	//LidarStateParam mLidarStateParam;
	//std::string mInfo1;
	//std::this_thread::sleep_for(std::chrono::milliseconds(2000));			//wait for a while, get device package
	//if (!m_GetLidarData->getLidarParamState(mLidarStateParam, mInfo1))				//Lidar parameter query
	//{
	//	std::cout << mInfo1 << std::endl;
	//}
	//else
	//{
	//	std::cout << "********** Lidar Parameters Display Start **********" << std::endl;

	//	std::cout.width(20); std::cout << "LidarIP	=	" << mLidarStateParam.LidarIP << std::endl;
	//	std::cout.width(20); std::cout << "ComputerIP	=	" << mLidarStateParam.ComputerIP << std::endl;
	//	std::cout.width(20); std::cout << "GatewayIP	=	" << mLidarStateParam.GatewayIP << std::endl;
	//	std::cout.width(20); std::cout << "SubnetMaskIP	=	" << mLidarStateParam.SubnetMaskIP << std::endl;
	// 	std::cout.width(20); std::cout << "MacAddress	=	" << mLidarStateParam.MacAddress << std::endl;
	//	std::cout.width(20); std::cout << "DataPort	=	" << mLidarStateParam.DataPort << std::endl;
	//	std::cout.width(20); std::cout << "DevPort	=	" << mLidarStateParam.DevPort << std::endl;

	//	std::cout.width(20); std::cout << "ReceiverTemperature	=	" << mLidarStateParam.ReceiverTemperature << std::endl;
	//	std::cout.width(20); std::cout << "ReceiverHighVoltage	=	" << mLidarStateParam.ReceiverHighVoltage << std::endl;
	//	std::cout.width(20); std::cout << "MotorSpeed	=	" << mLidarStateParam.MotorSpeed << std::endl;
	//	std::cout.width(20); std::cout << "FrameRateMode	=	" << mLidarStateParam.FrameRateMode << std::endl;
	
	//	std::cout.width(20); std::cout << "PTP_State	=	" << mLidarStateParam.PTP_State << std::endl;
	//	std::cout.width(20); std::cout << "GPS_State	=	" << mLidarStateParam.GPS_State << std::endl;
	//	std::cout.width(20); std::cout << "PPS_State	=	" << mLidarStateParam.PPS_State << std::endl;
	//	std::cout.width(20); std::cout << "StandbyMode	=	" << mLidarStateParam.StandbyMode << std::endl;
	//	std::cout.width(20); std::cout << "Clock_Source	=	" << mLidarStateParam.Clock_Source << std::endl;
	//	std::cout.width(20); std::cout << "PhaseLockedSwitch	=	" << mLidarStateParam.PhaseLockedSwitch << std::endl;
	//	std::cout.width(20); std::cout << "PhaseLockedState	=	" << mLidarStateParam.PhaseLockedState << std::endl;
	//	std::cout.width(20); std::cout << "RunningTime	=	" << mLidarStateParam.RunningTime << std::endl;

	//	std::cout << "" << "********** Lidar Paramet<< std::cout.width(20)ers Display End **********" << std::endl << std::endl;
	//}
#pragma endregion 

#pragma region   //get point cloud and process it with different algorithm,or display it
	while (true)
	{
		if (m_LidarData != nullptr)
		{
			//output the number of point cloud
			std::cout << m_LidarData->size() << std::endl;
			m_LidarData.reset();
		}
		else
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
	}
#pragma endregion

	m_GetLidarData->LidarStop();
}

//callbacks pointer,  obtain point cloud data of each frame
void A::callbackFunction(std::shared_ptr<std::vector<MuchLidarData>> LidarDataValue, int flag, std::string mInfo)
{
	m_mutex.lock();
	m_LidarData = LidarDataValue;
	if (false == flag)
	{
		m_LidarData = nullptr;
		std::cout << "Error " + flag << ":" << mInfo << std::endl;
	}

	m_mutex.unlock();
}

void callbackFunction_123(std::shared_ptr<std::vector<MuchLidarData>> LidarDataValue, int flag, std::string mInfo)
{
	m_mutex.lock();
	m_LidarData = LidarDataValue;
	if (false == flag)
	{
		m_LidarData = nullptr;
		std::cout << "Error " + flag <<":" << mInfo << std::endl;
	}
}

#pragma endregion 

#endif

