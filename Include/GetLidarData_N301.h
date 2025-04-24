#pragma once

#include "GetLidarData.h"

class GetLidarData_N301 : public GetLidarData
{
public:
	GetLidarData_N301(float Single, float Double, int Protocol);
	~GetLidarData_N301();

	void LidarRun() override;
	m_PointXYZ XYZ_calculate(double, double);
	
	bool setLidarRotateState(int StateValue, std::string& InfoString) override;							//Lidar rotate/stationary 
	bool setLidarWorkState(int StateValue, std::string& InfoString) override;							//Lidar state, low power mode or not

private:
	/*5.0 lidar
		V1.6 protocol
			single echo mode,  range resolution: 2MM
			double echo mode  range resolution: 1mm

		V1.7 protocol
			single echo mode, range resolution:4MM
			double echo mode, range resolution:1mm
	6.0 lidar
		V1.7 protocol
			single echo mode, range resolution:4MM
			double echo mode, range resolution:4mm
			*/
	float IDistanceACC_N301_Single;                                   //single echo, range resolution conversion: convert mm to m;
	float IDistanceACC_N301_Double;                                   //dopble echo, range resolution conversion: convert mm to m;
	int N301Protocol;                                               //Protocol，6:1.6 Protocol，7: 1.7 Protocol
	
	double cosTheta[8];
	double sinTheta[8];
	MuchLidarData m_DataT[30];
	MuchLidarData m_DataT_d[30];
	
	double endFrameAngle = 0;										//angle of the end frame which is also the starting angle
	std::vector<float>BlockAngle;									//angle of each block
	std::vector<float>all_BlockAngle;								//angles of all blocks in each frame
	
	void handleSingleEcho(unsigned char* data);						//single echo handling
	void handleDoubleEcho(unsigned char* data);						//double echo handling
};

