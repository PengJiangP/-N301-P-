#define _WINSOCK_DEPRECATED_NO_WARNINGS

#define UsageMethod_1 1

#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>	//point cloud class header file
#include <IncludeFile.h>

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

//ConnectionParameter
int cDevPort = 2369;
std::string cLidarIP = "192.168.1.200";
std::string cDestIP = "192.168.1.102";
std::string cGroupIp = "226.1.1.102";
int cDataPort = 2368;

//ModifyParameter
int mDataPort = 2368;
int mDevPort = 2369;
std::string mLidarIP = "192.168.1.200";
std::string mDestIP = "192.168.1.102";
std::string mNtpIP = "";
std::string mGatewayIP = "";
std::string mSubnetMaskIP = "";

//Initialize PCL
void onInitPCL()
{
#pragma region	////Initialize PCL point cloud display
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZRGBA>(cloud, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	viewer->addCoordinateSystem(0.1);
	viewer->initCameraParameters();
	viewer->setCameraPosition(0, 0, 50, 0, 0, 0);

#pragma region 	//Add coordinates and grid 
	//Add coordinates 
	viewer->addCoordinateSystem(0.5, 0, 0, 0, "coordinate", 0);

	//Add grid and circle
	pcl::PointXYZ C1, C2;
	for (int radius = 1; radius < 11; radius = radius + 1)
	{
		for (int angle = 0; 360 > angle; angle = angle + 10)
		{
			C1.x = radius * cos(angle * PI / 180);
			C1.y = radius * sin(angle * PI / 180);
			C1.z = 0;
			C2.x = radius * cos((angle + 10) * PI / 180);
			C2.y = radius * sin((angle + 10) *PI / 180);
			C2.z = 0;
			viewer->addLine(C1, C2, 150, 150, 150, "Circle" + std::to_string(radius) + std::to_string(angle));
			viewer->setShapeRenderingProperties(1, 0.2, "Circle" + std::to_string(radius) + std::to_string(angle));
		}
	}
	
	//Add grid distance prompt 
	for (int radius = 1; radius < 11; radius = radius + 1)
	{
		int angle = 90;
		C1.x = radius * cos(angle * PI / 180);
		C1.y = radius * sin(angle * PI / 180);
		C1.z = 0;
		viewer->addText3D(std::to_string(radius) + std::string(" m"), C1, 0.4, 150, 150, 150, "CircleShow" + std::to_string(radius));
	}
	viewer->resetCamera();
#pragma endregion 
#pragma endregion
}

#if  UsageMethod_1

#pragma region 		//method one: get one frame data through flag bit

int main(int argc, char* argv[])
{
	std::cout << "The number of command line arguments:" << argc << std::endl;

	if (argc >= 3)
	{
		for (int i = 0; i < argc; ++i) {
			std::cout << "parameters:  " << i << ": " << argv[i] << std::endl;
		}

		m_GetLidarData->setIndex = atoi(argv[1]);

		m_GetLidarData->setPortAndIP(cDataPort, cDevPort, cDestIP, cLidarIP, cGroupIp);			//set the parameters of the Ethernet port which receives the lidar data£¨port number of data package, device package, destination IP)
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


	onInitPCL();

	m_GetLidarData->setPortAndIP(cDataPort, cDevPort, cDestIP, cLidarIP, cGroupIp);		//set the Ethernet port parameters that lidar receives data
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
	//			
	//	std::cout.width(20); std::cout << "LidarIP	=	"				 << mLidarStateParam.LidarIP << std::endl;
	//	std::cout.width(20); std::cout << "ComputerIP	=	"			 << mLidarStateParam.ComputerIP << std::endl;
	//	std::cout.width(20); std::cout << "GatewayIP	=	"			 << mLidarStateParam.GatewayIP << std::endl;
	//	std::cout.width(20); std::cout << "SubnetMaskIP	=	"			 << mLidarStateParam.SubnetMaskIP << std::endl;
	//	std::cout.width(20); std::cout << "MacAddress	=	"			 << mLidarStateParam.MacAddress << std::endl;
	//	std::cout.width(20); std::cout << "DataPort	=	"				 << mLidarStateParam.DataPort << std::endl;
	//	std::cout.width(20); std::cout << "DevPort	=	"				 << mLidarStateParam.DevPort << std::endl;

	//	std::cout.width(20); std::cout << "ReceiverTemperature	=	"	 << mLidarStateParam.ReceiverTemperature << std::endl;
	//	std::cout.width(20); std::cout << "ReceiverHighVoltage	=	"	 << mLidarStateParam.ReceiverHighVoltage << std::endl;
	//	std::cout.width(20); std::cout << "MotorSpeed	=	"			 << mLidarStateParam.MotorSpeed << std::endl;
	//	std::cout.width(20); std::cout << "FrameRateMode	=	" << mLidarStateParam.FrameRateMode << std::endl;
		
	//	std::cout.width(20); std::cout << "PTP_State	=	"				 << mLidarStateParam.PTP_State << std::endl;
	//	std::cout.width(20); std::cout << "GPS_State	=	"				 << mLidarStateParam.GPS_State << std::endl;
	//	std::cout.width(20); std::cout << "PPS_State	=	"				 << mLidarStateParam.PPS_State << std::endl;
	//	std::cout.width(20); std::cout << "StandbyMode	=	"			 << mLidarStateParam.StandbyMode << std::endl;
	//	std::cout.width(20); std::cout << "Clock_Source	=	"			 << mLidarStateParam.Clock_Source << std::endl;
	//	std::cout.width(20); std::cout << "PhaseLockedSwitch	=	"		 << mLidarStateParam.PhaseLockedSwitch << std::endl;
	//	std::cout.width(20); std::cout << "PhaseLockedState	=	"		 << mLidarStateParam.PhaseLockedState << std::endl;
	//	std::cout.width(20); std::cout << "RunningTime	=	"			 << mLidarStateParam.RunningTime << std::endl;

	//	std::cout << "" << "********** Lidar Paramet<< std::cout.width(20)ers Display End **********" << std::endl << std::endl;
	//}
#pragma endregion 

#pragma region   //Obtain Lidar parameter for other algorithm processing or point cloud display

	//method one: get one frame data
	while (true)
	{
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
			
			cloud->points.clear();
			for (u_int m_FF = 0; m_FF < m_LidarData_temp->size(); m_FF++)
			{
				pcl::PointXYZRGBA point;
				point.x = (*m_LidarData_temp)[m_FF].X;
				point.y = (*m_LidarData_temp)[m_FF].Y;
				point.z = (*m_LidarData_temp)[m_FF].Z;


				int m_Intensity = (*m_LidarData_temp)[m_FF].Intensity;
	
				if (m_Intensity < 16)
				{
					point.r = 0;
					point.g = 128 + m_Intensity * 8;
					point.b = 255;
				}
				else if (m_Intensity < 70 && m_Intensity >= 16)
				{
					point.r = 0;
					point.g = 255;
					point.b = 255 - (m_Intensity - 16) * 4;
				}
				else if (m_Intensity < 128 && m_Intensity >= 70)
				{
					point.r = 4 * (m_Intensity - 70);
					point.g = 255;
					point.b = 0;
				}
				else {
					point.r = 255;
					point.g = 255 - (m_Intensity - 128) * 2;
					point.b = 0;
				}
				point.a = 255;
				
				cloud->points.emplace_back(std::move(point));
	
			}
	
			//the data is stored in point cloud and can be processed by one's own algorithm
			viewer->removeAllPointClouds();
			viewer->addPointCloud(cloud);
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
			viewer->updatePointCloud(cloud);
			viewer->spinOnce(1);

			//viewer->removeAllShapes();
	
			if (viewer->wasStopped())
			{
				return 0;
			}
		}
		else
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
	}
	
	m_GetLidarData->LidarStop();
#pragma endregion  
}

#pragma endregion


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

		m_GetLidarData->setPortAndIP(cDataPort, cDevPort, cDestIP, cLidarIP, cGroupIp);			//set the parameters of the Ethernet port which receives the lidar data£¨port number of data package, device package, destination IP)
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

	onInitPCL();

	//class member function as callback function
	A m_a;
	FunDataPrt fun = std::bind(&A::callbackFunction, m_a, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
	
	//ordinary callback function
	//FunDataPrt fun = std::bind(callbackFunction_123, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
	
	m_GetLidarData->setCallbackFunction(&fun);				//set callback function
	
	m_GetLidarData->setPortAndIP(cDataPort, cDevPort, cDestIP, cLidarIP, cGroupIp);		//set the Ethernet port parameters that lidar receives data
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

#pragma region   //Obtain Lidar parameter for other algorithm processing or point cloud display
	while (true)
	{
		if (m_LidarData != nullptr)
		{
			//output the number of point cloud
			std::cout << m_LidarData->size() << std::endl;
			
			cloud->points.clear();
			for (u_int m_FF = 0; m_FF < m_LidarData->size(); m_FF++)
			{
				pcl::PointXYZRGBA point;
				point.x = (*m_LidarData)[m_FF].X;
				point.y = (*m_LidarData)[m_FF].Y;
				point.z = (*m_LidarData)[m_FF].Z;

				int m_Intensity = (*m_LidarData)[m_FF].Intensity;
	
				if (m_Intensity < 16)
				{
					point.r = 0;
					point.g = 128 + m_Intensity * 8;
					point.b = 255;
				}
				else if (m_Intensity < 70 && m_Intensity >= 16)
				{
					point.r = 0;
					point.g = 255;
					point.b = 255 - (m_Intensity - 16) * 4;
				}
				else if (m_Intensity < 128 && m_Intensity >= 70)
				{
					point.r = 4 * (m_Intensity - 70);
					point.g = 255;
					point.b = 0;
				}
				else {
					point.r = 255;
					point.g = 255 - (m_Intensity - 128) * 2;
					point.b = 0;
				}
				point.a = 255;
				cloud->points.emplace_back(std::move(point));
	
			}
	
			//the data is stored in point cloud and can be processed by one's own algorithm
			viewer->updatePointCloud(cloud);
			viewer->addPointCloud(cloud);
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
			viewer->spinOnce(1);
			viewer->removeAllPointClouds();
			//viewer->removeAllShapes();
	
			m_LidarData.reset();
			if (viewer->wasStopped())
			{
				return 0;
			}
	
		}
		else
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
	}
	m_GetLidarData->LidarStop();
#pragma endregion 
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
		std::cout << "Error " + flag << ":" << mInfo << std::endl;
	}
	m_mutex.unlock();
}
#pragma endregion 

#endif