#pragma once
#include "GetLidarData_N301.h"

GetLidarData_N301::GetLidarData_N301(float  Single, float Double, int Protocol)
{

	BlockAngle.clear();
	all_BlockAngle.clear();
	endFrameAngle = 90.0;					//Fixed at 90 degree to parse the data


	IDistanceACC_N301_Single= Single/1000;   //convert mm to m
	IDistanceACC_N301_Double= Double/1000;    //convert mm to m
	N301Protocol = Protocol;
}

GetLidarData_N301::~GetLidarData_N301()
{
}

void GetLidarData_N301::LidarRun()
{
	while (true)
	{
		if (isQuit)
		{
			clearQueue(allDataValue);
			break;
		}
		if (!allDataValue.empty())
		{
			unsigned char data[1206] = { 0 };
			m_mutex.lock();
			memcpy(data, allDataValue.front(), 1206);
			delete allDataValue.front();
			allDataValue.pop();
			m_mutex.unlock();
			static int lidar_EchoModel;
			if (data[0] == 0xa5 && data[1] == 0xff && data[2] == 0x00 && data[3] == 0x5a)
			{
				lidar_EchoModel = data[185];
				continue;
			}
			timestamp_nsce = 16777216 * data[1203] + 65536 * data[1202] + 256 * data[1201] + data[1200];//timestamp; microsecond
		
			int angleBlock_0 = data[3] * 256 + data[2];
			int angleBlock_11 = data[3 + 100 * 11] * 256 + data[2 + 100 * 11];
			if (angleBlock_0 == angleBlock_11)
			{
				std::string str = "Frame synchronization failure!!!";
				messFunction(str, 10031);
				continue;
			}

#pragma region	//judge echo mode parsing

			if (0x00 == lidar_EchoModel)
			{
				handleSingleEcho(data);
			}
			else if (0x01 == lidar_EchoModel)
			{
				handleDoubleEcho(data);
			}

#pragma endregion
		}
		else
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
	}
}

void GetLidarData_N301::handleSingleEcho(unsigned char* data)
{
	if (data[0] == 0xff && data[1] == 0xee)
	{
		std::vector<MuchLidarData> lidardata_tmp;												//data point parsing structure
		lidardata_tmp.clear();
		bool isOneFrame = false;
		BlockAngle.clear();
		struct tm t;
		

		if (N301Protocol == 6)
		{
			//obatin GPS time in each block of 1.6 protocol
			m_UTC_Time.year = data[1194] + 2000;
			m_UTC_Time.month = data[1195];
			m_UTC_Time.day = data[1196];
			m_UTC_Time.hour = data[1197];
			m_UTC_Time.minute = data[1198];
			m_UTC_Time.second = data[1199];
		
			t.tm_sec = m_UTC_Time.second;
			t.tm_min = m_UTC_Time.minute;
			t.tm_hour = m_UTC_Time.hour;
			t.tm_mday = m_UTC_Time.day;
			t.tm_mon = m_UTC_Time.month - 1;
			t.tm_year = m_UTC_Time.year - 1900;
			t.tm_wday = 0;
			t.tm_yday = 0;
			t.tm_isdst = 0;
	
		}
		time_t _t = static_cast<uint64_t>(timegm(&t));
	
		for (int i = 0; i < 12; i++)
		{
			//Extract the value of azimuth of one circle
			float AngleBlock = (data[3 + 100 * i] * 256 + data[2 + 100 * i]) * 0.01;
	
			if (AngleBlock >= 360.0)
			{
				BlockAngle.emplace_back(std::move(AngleBlock - 360.0));
				all_BlockAngle.emplace_back(std::move(AngleBlock - 360.0));
			}
			else
			{
				BlockAngle.emplace_back(std::move(AngleBlock));
				all_BlockAngle.emplace_back(std::move(AngleBlock));
			}
			if(N301Protocol==6)
			{
				for (int j = 0, z = 0; j < 32; j++)
				{
					if (j == 0 || j == 16)
					{
						m_DataT[z].Distance = (data[5 + 3 * j + 100 * i] * 256 + data[4 + 3 * j + 100 * i]) * IDistanceACC_N301_Single;
						m_DataT[z].Intensity = data[6 + 3 * j + 100 * i];
						m_DataT[z].Mtimestamp_nsce = (timestamp_nsce + (2 * i + z)*3.125 - 596.875 + _t * 1000000 / 24)*1000;       //nanosecond
						z++;
						m_DistanceIsNotZero = m_DistanceIsNotZero > 30 ? m_DistanceIsNotZero : (abs(m_DataT[j].Distance - 0) > 0.00000001) ? ++m_DistanceIsNotZero : m_DistanceIsNotZero;
					}
				}
	
			}
			if (N301Protocol == 7)
			{
				//obatin GPS time in each block of 1.7 protocol
				m_UTC_Time.year = data[49 + i * 100] + 2000;
				m_UTC_Time.month = data[50 + i * 100];
				m_UTC_Time.day = data[51 + i * 100];
				m_UTC_Time.hour = data[97 + i * 100];
				m_UTC_Time.minute = data[98 + i * 100];
				m_UTC_Time.second = data[99 + i * 100];
	
				t.tm_sec = m_UTC_Time.second;
				t.tm_min = m_UTC_Time.minute;
				t.tm_hour = m_UTC_Time.hour;
				t.tm_mday = m_UTC_Time.day;
				t.tm_mon = m_UTC_Time.month - 1;
				t.tm_year = m_UTC_Time.year - 1900;
				t.tm_wday = 0;
				t.tm_yday = 0;
				t.tm_isdst = 0;
				
				time_t _t1 = static_cast<uint64_t>(timegm(&t));
	
				for (int j = 0, z = 0; j < 32; j++)
				{
					if (j == 15 || j == 31)
					{
						continue;
					}
					m_DataT[z].Distance = (data[5 + 3 * j + 100 * i] * 256 + data[4 + 3 * j + 100 * i]) * IDistanceACC_N301_Single;
					m_DataT[z].Intensity = data[6 + 3 * j + 100 * i];
					m_DataT[z].Mtimestamp_nsce = (timestamp_nsce + (30 * i + z) * 3.125 - 596.875 + _t1 * 1000000 / 30) * 1000;           //纳秒
					z++;
					m_DistanceIsNotZero = m_DistanceIsNotZero > 30 ? m_DistanceIsNotZero : (abs(m_DataT[j].Distance - 0) > 0.00000001) ? ++m_DistanceIsNotZero : m_DistanceIsNotZero;
				}
	
			}
	
			int firing_num = 0;
			if (N301Protocol == 6)
			{
				firing_num = 2;//2 data
			}
			else
			{
				firing_num = 30;//30 data
			}
	
			if (BlockAngle.size() >= 2)
			{
				for (int j = 0; j < firing_num; j++)
				{
					float mdiff = (BlockAngle.back() - BlockAngle[BlockAngle.size() - 2]);
					mdiff = mdiff >= 0 ? mdiff : mdiff + 360;
	
					double m_angle = (BlockAngle[BlockAngle.size() - 2] + (mdiff / firing_num) * j);
					if (m_angle < 360.0)
						m_DataT[j].H_angle = m_angle;
					else
						m_DataT[j].H_angle = m_angle - 360.0;
	
					m_PointXYZ m_point = XYZ_calculate(m_DataT[j].H_angle, m_DataT[j].Distance);
					m_DataT[j].X = m_point.x1;
					m_DataT[j].Y = m_point.y1;
					m_DataT[j].Z = m_point.z1;
	
					if (all_BlockAngle.size() > 100)
					{
						//if ((m_DataT[j].H_angle - endFrameAngle > 0) && abs(m_DataT[j].H_angle - endFrameAngle) < 20)
						
						if (j>0&&abs(m_DataT[j].H_angle - m_DataT[j - 1].H_angle) > 180)
						{
							if (isPointFilter(m_DataT[j]))
							{
								lidardata_tmp.emplace_back(std::move(m_DataT[j]));
							}
							isOneFrame = true;
						}
						else if (abs(m_DataT[j].H_angle - (*LidarPerFrameDatePrt_Get)[LidarPerFrameDatePrt_Get->size() - 1].H_angle) > 180)
						{
							if (isPointFilter(m_DataT[j]))
							{
								lidardata_tmp.emplace_back(std::move(m_DataT[j]));
							}
							isOneFrame = true;
						}
						else
						{
							if (isPointFilter(m_DataT[j]))
							{
								LidarPerFrameDatePrt_Get->emplace_back(std::move(m_DataT[j]));
							}
						}
					}
					else
					{
						if (isPointFilter(m_DataT[j]))
						{
							LidarPerFrameDatePrt_Get->emplace_back(std::move(m_DataT[j]));
						}
					}
				}
			}
		}
	
		if (isOneFrame)
		{
			sendLidarData();
			all_BlockAngle.clear();
	
			if (lidardata_tmp.size() > 0)
			{
				for (size_t SF = 0; SF < lidardata_tmp.size(); SF++)
				{
					LidarPerFrameDatePrt_Get->emplace_back(std::move(lidardata_tmp[SF]));
				}
			}
		}
	}
}

void GetLidarData_N301::handleDoubleEcho(unsigned char* data)
{
	if (data[0] == 0xff && data[1] == 0xee)
	{
		std::vector<MuchLidarData> lidardata_tmp, lidardata_tmp_d;												//data point parsing structure
		lidardata_tmp.clear();
		lidardata_tmp_d.clear();

		bool isOneFrame = false;
		BlockAngle.clear();
	
		struct tm t;
		if (N301Protocol == 6)
		{
			m_UTC_Time.year = data[1194] + 2000;
			m_UTC_Time.month = data[1195];
			m_UTC_Time.day = data[1196];
			m_UTC_Time.hour = data[1197];
			m_UTC_Time.minute = data[1198];
			m_UTC_Time.second = data[1199];
			
			t.tm_sec = m_UTC_Time.second;
			t.tm_min = m_UTC_Time.minute;
			t.tm_hour = m_UTC_Time.hour;
			t.tm_mday = m_UTC_Time.day;
			t.tm_mon = m_UTC_Time.month - 1;
			t.tm_year = m_UTC_Time.year - 1900;
			t.tm_wday = 0;
			t.tm_yday = 0;
			t.tm_isdst = 0;
		}
		time_t _t = static_cast<uint64_t>(timegm(&t));
	
		for (int i = 0; i < 12; i += 2)
		{
			//Extract the value of azimuth of one circle
			float AngleBlock = (data[3 + 100 * i] * 256 + data[2 + 100 * i]) * 0.01;
	
			if (AngleBlock >= 360.0)
			{
				BlockAngle.emplace_back(std::move(AngleBlock - 360.0));
				all_BlockAngle.emplace_back(std::move(AngleBlock - 360.0));
			}
			else
			{
				BlockAngle.emplace_back(std::move(AngleBlock));
				all_BlockAngle.emplace_back(std::move(AngleBlock));
			}
	
			if (N301Protocol == 6)
			{
				for (int j = 0, z = 0; j < 32; j++)
				{
					if (j == 0 || j == 16) 
					{
						m_DataT[z].Distance = (data[5 + 3 * j + 100 * i] * 256 + data[4 + 3 * j + 100 * i]) * IDistanceACC_N301_Double;
						m_DataT[z].Intensity = data[6 + 3 * j + 100 * i];
						m_DataT[z].Mtimestamp_nsce = (timestamp_nsce + (2 * i + z)*3.125 - 596.875 + _t * 1000000 / 24) * 1000;       //nanosecond
	
						m_DataT_d[z].Distance = (data[5 + 3 * j + 100 * (i + 1)] * 256 + data[4 + 3 * j + 100 * (i + 1)]) * IDistanceACC_N301_Double;
						m_DataT_d[z].Intensity = data[6 + 3 * j + 100 * (i + 1)];
						m_DataT[z].Mtimestamp_nsce = (timestamp_nsce + (2 * i + z)*3.125 - 596.875 + _t * 1000000 / 24) * 1000;       //nanosecond
						z++;
						m_DistanceIsNotZero = m_DistanceIsNotZero > 30 ? m_DistanceIsNotZero : (abs(m_DataT[j].Distance - 0) > 0.00000001) ? ++m_DistanceIsNotZero : m_DistanceIsNotZero;
					}
					
				}
			}
			if (N301Protocol == 7)
			{
				//obatin GPS time in each block of 1.7 protocol
				m_UTC_Time.year = data[49 + i * 100] + 2000;
				m_UTC_Time.month = data[50 + i * 100];
				m_UTC_Time.day = data[51 + i * 100];
				m_UTC_Time.hour = data[97 + i * 100];
				m_UTC_Time.minute = data[98 + i * 100];
				m_UTC_Time.second = data[99 + i * 100];
				
				t.tm_sec = m_UTC_Time.second;
				t.tm_min = m_UTC_Time.minute;
				t.tm_hour = m_UTC_Time.hour;
				t.tm_mday = m_UTC_Time.day;
				t.tm_mon = m_UTC_Time.month - 1;
				t.tm_year = m_UTC_Time.year - 1900;
				t.tm_wday = 0;
				t.tm_yday = 0;
				t.tm_isdst = 0;
	
				time_t _t1 = static_cast<uint64_t>(timegm(&t));
	
				for (int j = 0, z = 0; j < 32; j++)
				{
					if (j == 15 || j == 31) {
						continue;
					}
					m_DataT[z].Distance = (data[5 + 3 * j + 100 * i] * 256 + data[4 + 3 * j + 100 * i]) * IDistanceACC_N301_Double;
					m_DataT[z].Intensity = data[6 + 3 * j + 100 * i];
					m_DataT[z].Mtimestamp_nsce = (timestamp_nsce + (30 * i + z)*3.125 - 596.875 + _t1 * 1000000 / 30) * 1000;           //nanosecond
	
					m_DataT_d[z].Distance = (data[5 + 3 * j + 100 * (i + 1)] * 256 + data[4 + 3 * j + 100 * (i + 1)]) * IDistanceACC_N301_Double;
					m_DataT_d[z].Intensity = data[6 + 3 * j + 100 * (i + 1)];
					m_DataT[z].Mtimestamp_nsce = (timestamp_nsce + (30 * i + z)*3.125 - 596.875 + _t1 * 1000000 / 30) * 1000;           //nanosecond
					z++;
					m_DistanceIsNotZero = m_DistanceIsNotZero > 30 ? m_DistanceIsNotZero : (abs(m_DataT[j].Distance - 0) > 0.00000001) ? ++m_DistanceIsNotZero : m_DistanceIsNotZero;
				}
			}
	
			int firing_num = 0;
			if (N301Protocol == 6)
			{
				firing_num = 2;//2个 data
			}
			else
			{
				firing_num = 30;//30 data
			}
			if (BlockAngle.size() >= 2)
			{
				for (int j = 0; j < firing_num; j++)
				{
					float mdiff = (BlockAngle.back() - BlockAngle[BlockAngle.size() - 2]);
					mdiff = mdiff >= 0 ? mdiff : mdiff + 360;
	
					double m_angle = (BlockAngle[BlockAngle.size() - 2] + (mdiff / firing_num) * j);
					if (m_angle < 360.0)
					{
						m_DataT[j].H_angle = m_angle;
						m_DataT_d[j].H_angle = m_angle;
					}
					else
					{
						m_DataT[j].H_angle = m_angle - 360.0;
						m_DataT_d[j].H_angle = m_angle - 360.0;
					}
	
					m_PointXYZ m_point = XYZ_calculate(m_DataT[j].H_angle, m_DataT[j].Distance);
					m_DataT[j].X = m_point.x1;
					m_DataT[j].Y = m_point.y1;
					m_DataT[j].Z = m_point.z1;
	
					m_PointXYZ m_point_d = XYZ_calculate(m_DataT_d[j].H_angle, m_DataT_d[j].Distance);
					m_DataT_d[j].X = m_point_d.x1;
					m_DataT_d[j].Y = m_point_d.y1;
					m_DataT_d[j].Z = m_point_d.z1;
	
					if (all_BlockAngle.size() > 100)
					{
						//if ((m_DataT[j].H_angle - endFrameAngle > 0) && abs(m_DataT[j].H_angle - endFrameAngle) < 20)
						if (j>0 && abs(m_DataT[j].H_angle - m_DataT[j - 1].H_angle) > 180)
						{
							if (isPointFilter(m_DataT[j]))
							{
								lidardata_tmp.emplace_back(std::move(m_DataT[j]));
							}
							isOneFrame = true;
						}
						else if (abs(m_DataT[j].H_angle - (*LidarPerFrameDatePrt_Get)[LidarPerFrameDatePrt_Get->size() - 1].H_angle) > 180)
						{
							if (isPointFilter(m_DataT[j]))
							{
								lidardata_tmp.emplace_back(std::move(m_DataT[j]));
							}
							isOneFrame = true;
						}
						else
						{
							if (isPointFilter(m_DataT[j]))
							{
								LidarPerFrameDatePrt_Get->emplace_back(std::move(m_DataT[j]));
								LidarPerFrameDatePrt_Get->emplace_back(std::move(m_DataT_d[j]));
							}
						}
					}
					else
					{
						if (isPointFilter(m_DataT[j]))
						{
							LidarPerFrameDatePrt_Get->emplace_back(std::move(m_DataT[j]));
							LidarPerFrameDatePrt_Get->emplace_back(std::move(m_DataT_d[j]));
						}
					}
				}
			}
		}
	
		if (isOneFrame)
		{
			sendLidarData();
			all_BlockAngle.clear();
	
			if (lidardata_tmp.size() > 0)
			{
				for (size_t SF = 0; SF < lidardata_tmp.size(); SF++)
				{
					LidarPerFrameDatePrt_Get->emplace_back(std::move(lidardata_tmp[SF]));
					LidarPerFrameDatePrt_Get->emplace_back(std::move(lidardata_tmp_d[SF]));
				}
			}
		}
	}
}

m_PointXYZ GetLidarData_N301::XYZ_calculate(double H_angle, double Distance)
{
	m_PointXYZ point;

	point.x1 = float(Distance * sinAngleValue[NegativeToPositive(H_angle)]);

	point.y1 = float(Distance * cosAngleValue[NegativeToPositive(H_angle)]);
	
	//point.z1 = float(Distance);
	
	return point;
}

#pragma region //set lidar parameters

bool GetLidarData_N301::setLidarRotateState(int RotateState, std::string& InfoString)
{
	if (setLidarParam())
	{
		Rest_UCWP_buff[40] = 0x00;
		Rest_UCWP_buff[41] = RotateState;      //1: stationary ，0: rotating
		

		return true;
	}
	else
	{
		InfoString = "Equipment package is not update!!!";
		return false;
	}
}

bool GetLidarData_N301::setLidarWorkState(int LidarState, std::string& InfoString)
{
	if (setLidarParam())
	{
		Rest_UCWP_buff[44] = 0x00;
		Rest_UCWP_buff[45] = LidarState;   //1: im operation;0: sleep mode
		return true;
	}
	else
	{
		InfoString = "Equipment package is not update!!!";
		return false;
	}
}

#pragma endregion 