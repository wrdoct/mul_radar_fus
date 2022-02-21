#pragma once

/*将各个雷达采集的数据转换到同一个全局坐标系下之后先得出相邻雷达的重叠区域。
定义雷达的检测区域长度为L1，上、下游相邻两台雷达的重叠检测区域长度为L2，雷达盲区距离为△L，
相邻上游雷达检测到目标车辆的X轴坐标为x1，相邻下游雷达检测到目标车辆的X轴坐标为x2；
当△L≤x1≤L2+△L，L1-L2≤x2≤L1+△L时，则判定该目标车辆为重叠检测区域内目标。*/

#include "coordinate_transformation.h"

//重叠区域判断函数的输入应为两个雷达的数据，输出应为来自两个雷达重叠区域的数据，以及除了重叠区域之外两个雷达的数据；
//这样在用的时候可以：将重叠区域的数据中的一个进行坐标转换去做NNDA，将除了重叠区域之外的数据中的一个进行坐标转换以生成系统航迹
void JudgmentOverlap(std::vector<RadarInfo_t> &radar1FrameData, std::vector<RadarInfo_t> &radar2FrameData,
	std::vector<RadarInfo_t> &radar1OverlapOutput, std::vector<RadarInfo_t> &radar2OverlapOutput,
	std::vector<RadarInfo_t> &radar1OverlapExcept, std::vector<RadarInfo_t> &radar2OverlapExcept);
