#pragma once
#include <iostream>
#include <json.h>
#include <fstream>
#include <string>

#include <sys/timeb.h>
#include <time.h> 

#include <Eigen/Dense>

using namespace std;
using namespace Eigen;     // 改成这样亦可 using Eigen::MatrixXd; 

timeb StringToDatetime(string str);

double StringToDouble(string Input);
int StringToInt(string Input);

typedef struct {
	int timeStamp = 0;//时间戳//帧数 //double类型改为int型
	unsigned int radar_id = -1;
	unsigned int car_id = -1;//-1表示无符号整数的最大值65535
	unsigned int road_id = -1;//-1表示无符号整数的最大值65535
	int miss = 0;//丢失目标个数
	int life = 0;//成功检测目标个数
	Eigen::Vector2d position = { 0.0, 0.0 };//目标位置//Vector3d 3维向量类 默认为列向量
	Eigen::Vector2d velocity = { 0.0, 0.0 };//目标速度
}RadarInfo_t;

//加载雷达信息
void LoadRadarInfo(const std::string &path,   //输入
	std::vector<RadarInfo_t> &radarObjects); //输出

//数据按照时间帧率分类
void RadarFrame(std::vector<RadarInfo_t> &radarObjects, std::vector<RadarInfo_t> &radarFrameData);

//使用单个目标的测试
void SingleTarget(std::vector<RadarInfo_t> &radarObjects, vector<Eigen::Vector2d> &radarSinglePoints);

