#pragma once
#include "radar_json.h"
#include "tools.h"

//#define _ENABLE_EXTENDED_ALIGNED_STORAGE //预编译宏 不报错

//来自节点雷达A的航迹i和节点雷达B的航迹j在k时刻的状态估计差为Xij(k) = 雷达1在第k帧的第i个目标航迹状态 - 雷达2在第k帧的第j个目标航迹状态
//得到三维向量：距离维、速度维、车道号维；设置阈值向量e；若小于阈值则成功关联

using namespace std;
using namespace Eigen;

const double nnda_Pi = 3.1415926;

static double rangeThreshold = 200000; //距离阈值
static double velThreshold = 100000;//速度阈值

class NNDA {
public:
	NNDA();
	~NNDA();

public:

	//获得维度数据
	//void GetDimensionalData(const vector<RadarInfo_t> &radarInfos, vector<Eigen::Vector2d> &radarPoints, vector<Eigen::Vector2d> &radarVels, vector<int> &radarRoadId);
	void GetDimensionalData(const RadarInfo_t &radarInfos, Eigen::Vector2d &radarPoints, Eigen::Vector2d &radarVels, int &radarRoadId);

	//将数据变为 2行num列 的矩阵
	void TransMatrix(Eigen::Vector2d &radarData, vector<MatrixXd> &tmpData);

	void Fusion(vector<RadarInfo_t> &radar1_OverlapStatus, vector<RadarInfo_t> &radar2_OverlapStatus, vector<RadarInfo_t> &mul_radarFusion);
		
private:

	TOOLS *tools;

	int radar1_measureNum;//雷达1目标个数 / 航迹个数
	int radar2_measureNum;//雷达2目标个数 / 航迹个数

	Eigen::Vector2d radar1OverlapPoints;//雷达1重叠区域的位置
	Eigen::Vector2d radar1OverlapVels;//雷达1重叠区域的速度
	int radar1OverlapRoadId;//雷达1重叠区域的车道号

	Eigen::Vector2d radar2OverlapPoints;//雷达2重叠区域的位置
	Eigen::Vector2d radar2OverlapVels;//雷达2重叠区域的速度
	int radar2OverlapRoadId;//雷达2重叠区域的车道号

};
