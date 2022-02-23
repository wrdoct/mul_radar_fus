#pragma once
#include "tools.h"

#include <graphics.h>		// 引用图形库头文件
#include <conio.h>

using namespace std;
using namespace Eigen;     // 改成这样亦可 using Eigen::MatrixXd; 

//画图 //验证程序使用
class GRAPH {
public:
	GRAPH();
	~GRAPH();

void radarPaint(vector<RadarInfo_t> &radarFrameInfos);

//显示的参数设置
void ParaConfig();

//画车道
void Road();

private:
	int carNum;
	IMAGE img;

	int x1;
	int y1;
	int road_w;
	int appear_length;
};
