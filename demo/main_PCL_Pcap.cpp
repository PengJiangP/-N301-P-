#define _WINSOCK_DEPRECATED_NO_WARNINGS

#define UsageMethod_1 1

#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>	//display point cloud class header file 
#include <IncludeFile.h>

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

//play Pcap file offline
#include <pcap.h>
#pragma comment(lib, "packet.lib")
#pragma comment(lib, "wpcap.lib")

//Offline packet path
#ifdef LINUX
	char* pcapFilePath = "../demo/PcapPacketPath/GetLidarDataType_LS.pcap";
#else
	char* pcapFilePath = "../../demo/PcapPacketPath/GetLidarDataType_LS.pcap";
	//char* pcapFilePath = "../demo/PcapPacketPath/GetLidarDataType_LS.pcap";
#endif

//off-line play
//离线播放
pcap_t *pfilePrt;													//file handle
bool openPcapFile(const char *fileName);							//open file and get the file handle
void readPcapFile();												//parse Pcap file and send it to library file 
int numTimes = 0;													//loop playback times
bool isPlayEnd = false;												//End of play marker

//open Pcap file and get the file handle
bool openPcapFile(const char *fileName)
{
	char errbuf[100];
	pfilePrt = pcap_open_offline(fileName, errbuf);
	if (NULL == pfilePrt) {
		std::cout << errbuf;
		return false;
	}
	return true;
}

//thread parsing pcap,  send to the object for parsing
void readPcapFile()
{
	int PacketDataLen = 1206;
	float lastPacketAngle_C = -1;

	pcap_pkthdr* pkthdr = 0;
	const u_char* pktdata = 0;
	char errbuf[100];
	
	//filter UDP and data port
	struct bpf_program filter_UDE;
	char packet_filter[] = "udp src port 2368 or udp src port 2369 ";
	bpf_u_int32 mask;
	bpf_u_int32 net;
	char* dev = pcap_lookupdev(errbuf);
	pcap_lookupnet(pcapFilePath, &net, &mask, errbuf);
	
	pcap_compile(pfilePrt, &filter_UDE, packet_filter, 0, net);
	pcap_setfilter(pfilePrt, &filter_UDE);
	
	u_char dataBuff[1212];					//*obtain device info from the devicde packet thread
	//the position where the offline play starts
	int offLinePacketStartIndex = 42;

	long int lastPacketTime = -1;
	long int timeInterval = -1;
	while (true)    //Obtain the size info of the offline file
	{
		if (pcap_next_ex(pfilePrt, &pkthdr, &pktdata) <= 0)
		{
			break;
		}

		int coutNum = 0;
		if (!(pktdata[12] == 0x08 && pktdata[13] == 0x00))
		{
			while (true)
			{
				coutNum++;
				if ((pktdata[12 + coutNum] == 0x08 && pktdata[13 + coutNum] == 0x00) || (12 + coutNum > pkthdr->len))
				{
					break;
				}
			}
		}
	
		offLinePacketStartIndex = 42 + coutNum;
	
		//(UDP length：([38] * 256 + [39])) - 8-byte UDP(source port+destination port+UDP length+UDP check)
		PacketDataLen = (pktdata[38 + coutNum] * 256 + pktdata[39 + coutNum]) - 8;
	
		pktdata = pktdata + offLinePacketStartIndex;
		memcpy(dataBuff, pktdata, PacketDataLen);
	
		if ((pkthdr->len >= 1248 && pkthdr->len <= 1254 + 50) &&
			(!(dataBuff[0] == 0xaa && dataBuff[1] == 0x00 && dataBuff[2] == 0xff && dataBuff[3] == 0x11)))
		{
			if ((dataBuff[0] == 0xa5 || dataBuff[0] == 0x00) && dataBuff[1] == 0xff && dataBuff[2] == 0x00 && dataBuff[3] == 0x5a)
			{
				m_GetLidarData->CollectionDataArrive(dataBuff, PacketDataLen);							
			}
			else
			{
				m_GetLidarData->CollectionDataArrive(dataBuff, PacketDataLen);										
			}

			if (lastPacketTime == -1)
			{
				lastPacketTime = pkthdr->ts.tv_sec * 1000000 + pkthdr->ts.tv_usec;
				continue;
			}

			timeInterval = (pkthdr->ts.tv_sec * 1000000 + pkthdr->ts.tv_usec) - lastPacketTime;
			if (timeInterval >= 50000)
			{
				std::this_thread::sleep_for(std::chrono::microseconds(timeInterval));
				lastPacketTime = pkthdr->ts.tv_sec * 1000000 + pkthdr->ts.tv_usec;
			}
		}


	}
	
	pcap_close(pfilePrt);
	isPlayEnd = true;
	return;
}
								
//initialize PCL 
void onInitPCL()
{
#pragma region	////initialize the point cloud display
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZRGBA>(cloud, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	viewer->addCoordinateSystem(0.1);
	viewer->initCameraParameters();
	viewer->setCameraPosition(0, 0, 50, 0, 0, 0);

#pragma region 	//add coordinates and griding
	//add coordinates
	viewer->addCoordinateSystem(0.5, 0, 0, 0, "coordinate", 0);

	//add gridding circle
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
	
	//add gridding distance reminder
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

int main()
{
	onInitPCL();

	m_GetLidarData->LidarOfflineDataStar();										//start parsing Lidar data

	//open pcap file
	if (!openPcapFile(pcapFilePath))
	{
		std::cout << "Pcap File Open Failed!" << std::endl;
		return 0;
	}
	isPlayEnd = false;
	std::thread m_ReadPcapT(&readPcapFile);
	m_ReadPcapT.detach();

//#pragma region //Set up a point cloud filtering test
//	LidaFilterParamDisplay mLidaFilterParamDisplay;
//
//	mLidaFilterParamDisplay.mMin_HanleValue = -60;
//	mLidaFilterParamDisplay.mMax_HanleValue = 0;
//
//	mLidaFilterParamDisplay.mMin_VanleValue = -60;
//	mLidaFilterParamDisplay.mMax_VanleValue = 0;
//
//	mLidaFilterParamDisplay.mMin_Distance = 0;
//	mLidaFilterParamDisplay.mMax_Distance = 10;
//
//	mLidaFilterParamDisplay.mMin_Intensity = 0;
//	mLidaFilterParamDisplay.mMax_Intensity = 255;
//
//	m_GetLidarData->setLidarFilterdisplay(mLidaFilterParamDisplay);
//
//#pragma endregion 

#pragma region   //obtain point cloud and process it with different algorithm or display it

	//method one: get one frame data 
	while (true)
	{
		if (true == isPlayEnd)
		{
			break;
		}
		if (m_GetLidarData->isFrameOK)
		{
			std::shared_ptr<std::vector<MuchLidarData>> m_LidarData_temp;
			std::string mInfo;
			if (!m_GetLidarData->getLidarPerFrameDate(m_LidarData_temp, mInfo))
			{
				std::cout << mInfo << std::endl;
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
	
			//the data is storaged in the point cloud and can be processed with one's own algorithm
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

	std::cout << "Pcap Playback Ends!" << std::endl;
	m_GetLidarData->LidarStop();									//finish and quit
#pragma endregion  
}

#pragma endregion


#else

#pragma region 		//method two: get one frame data through callback functions
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


int main()
{
	onInitPCL();

	//class member function as callback function
	A m_a;
	FunDataPrt fun = std::bind(&A::callbackFunction, m_a, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

	//ordinary callback function
	//FunDataPrt fun = std::bind(callbackFunction_123, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
	
	m_GetLidarData->setCallbackFunction(&fun);				//set callback function
	
	m_GetLidarData->LidarOfflineDataStar();										//start parsing data
	
	//open pcap file
	if (!openPcapFile(pcapFilePath))
	{
		std::cout << "Pcap File Open Failed!" << std::endl;
		return 0;
	}
	
	std::thread m_ReadPcapT(&readPcapFile);
	m_ReadPcapT.detach();

#pragma region   //get point cloud and process it with different algorithm, or display point cloud
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
	
			//the data is storaged in the point cloud and can be processed with one's own algorithm
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
