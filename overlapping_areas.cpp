#include "overlapping_areas.h"

//定义雷达的检测区域长度为L1
#define radarDetectLength 20000000 //m * 10^5
//上、下游相邻两台雷达的重叠检测区域长度为L2
#define overLappingLength 2000000
//雷达盲区距离为△L
#define blindAreaLength 2000000

//相邻上游雷达检测到目标车辆的X轴坐标为x1，相邻下游雷达检测到目标车辆的X轴坐标为x2；
//当△L≤x1≤L2 + △L，L1 - L2≤x2≤L1 + △L时，则判定该目标车辆为重叠检测区域内目标
void JudgmentOverlap(std::vector<RadarInfo_t> &radar1FrameData, std::vector<RadarInfo_t> &radar2FrameData, 
	std::vector<RadarInfo_t> &radar1OverlapOutput, std::vector<RadarInfo_t> &radar2OverlapOutput,
	std::vector<RadarInfo_t> &radar1OverlapExcept, std::vector<RadarInfo_t> &radar2OverlapExcept) {
	//获取位置
	vector<Eigen::Vector2d> radar1Points, radar2Points;
	ExtractPos(radar1FrameData, radar1Points);
	ExtractPos(radar2FrameData, radar2Points);

	//假设以雷达2为公共坐标系
	for (size_t idx = 0; idx < min(radar1Points.size(), radar2Points.size()); idx++) {
		if (radar1Points[idx][0] >= blindAreaLength && radar1Points[idx][0] <= overLappingLength + blindAreaLength){// && 
			//radar2Points[idx][0] >= radarDetectLength - overLappingLength && radar2Points[idx][0] <= radarDetectLength + blindAreaLength) {
			cout << "该目标车辆为重叠检测区域内目标" << endl;
			//cout << "radar1FrameData[idx].car_id : " << radar1FrameData[idx].car_id << endl;
			radar1OverlapOutput.push_back(radar1FrameData[idx]);
			radar2OverlapOutput.push_back(radar1FrameData[idx]);		
		}
		else {
			radar1OverlapExcept.push_back(radar1FrameData[idx]);
			radar2OverlapExcept.push_back(radar1FrameData[idx]);
		}
	}

	return;
}
