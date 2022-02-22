#pragma once
#include "radar_json.h"
#include "tools.h"


//来自节点雷达A的航迹i和节点雷达B的航迹j在k时刻的状态估计差为Xij(k) = 雷达1在第k帧的第i个目标航迹状态 - 雷达2在第k帧的第j个目标航迹状态
//得到三维向量：距离维、速度维、车道号维；设置阈值向量e；若小于阈值则成功关联

using namespace std;
using namespace Eigen;

const double nnda_Pi = 3.1415926;

class NNDA {
public:
	NNDA();
	~NNDA();

public:

	void init();

	//获得维度数据
	//void GetDimensionalData(const vector<RadarInfo_t> &radarInfos, vector<Eigen::Vector2d> &radarPoints, vector<Eigen::Vector2d> &radarVels, vector<int> &radarRoadId);
	void GetDimensionalData(const RadarInfo_t &radarInfos, Eigen::Vector2d &radarPoints, Eigen::Vector2d &radarVels, int &radarRoadId);

	//将数据变为 2行num列 的矩阵
	void TransMatrix(Eigen::Vector2d &radarData, vector<MatrixXd> &tmpData);

	void Fusion(vector<RadarInfo_t> &radar1_OverlapStatus, vector<RadarInfo_t> &radar2_OverlapStatus, vector<RadarInfo_t> &mul_radarFusion);
		
private:

	TOOLS *tools;

	//距离阈值
	int rangeThreshold;
	//速度阈值
	int velThreshold;
};
