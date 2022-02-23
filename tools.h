#pragma once
/*将各个雷达采集的数据转换到同一个全局坐标系下之后先得出相邻雷达的重叠区域。
定义雷达的检测区域长度为L1，上、下游相邻两台雷达的重叠检测区域长度为L2，雷达盲区距离为△L，
相邻上游雷达检测到目标车辆的X轴坐标为x1，相邻下游雷达检测到目标车辆的X轴坐标为x2；
当△L≤x1≤L2+△L，L1-L2≤x2≤L1+△L时，则判定该目标车辆为重叠检测区域内目标。*/


#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "radar_json.h"

//#include <math.h>
#define M_PI       3.14159265358979323846   // pi

using namespace std;
using namespace Eigen;

static double angle = M_PI / 4.0;
static double a = 1.0, b = 1.0;

class TOOLS {
public:
	TOOLS();
	virtual ~TOOLS();//基类中的析构函数必须为虚函数，否则会出现对象释放错误

	//重叠区域判断函数的输入应为两个雷达的数据，输出应为来自两个雷达重叠区域的数据，以及除了重叠区域之外两个雷达的数据；
	//这样在用的时候可以：将重叠区域的数据中的一个进行坐标转换去做NNDA，将除了重叠区域之外的数据中的一个进行坐标转换以生成系统航迹
	void JudgmentOverlap(std::vector<RadarInfo_t> &radar1FrameData, std::vector<RadarInfo_t> &radar2FrameData,
		std::vector<RadarInfo_t> &radar1OverlapOutput, std::vector<RadarInfo_t> &radar2OverlapOutput,
		std::vector<RadarInfo_t> &radar1OverlapExcept, std::vector<RadarInfo_t> &radar2OverlapExcept);

	//获取速度
	void ExtractVel(const vector<RadarInfo_t> &radarInfos, vector<Eigen::Vector2d> &radarVels);

	//获取位置
	void ExtractPos(const vector<RadarInfo_t> &radarInfos, vector<Eigen::Vector2d> &radarPoints);

	//空间坐标转换
	void SpaceTrans(Eigen::Vector2d &radar_pos, double &angle, double &a, double &b, Eigen::Vector2d &radartoshare_pos);

	//我们需要检查算法效果，来评估估算结果和真实结果差别有多大。最常见的检查标准叫做均方根误差(Root mean squared error)。
   //估算状态和真实状态之间的差值叫做残差。对这些残差先平方然后求均值就可以得到均方根误差。
   //计算均方根误差 //输入为 估算状态和真实状态
	Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, const std::vector<Eigen::VectorXd> &groundTruth);


	//计算雅可比矩阵
	Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& xState);

	
private:

};

//helper 函数来标准化角:
double AngleNormalization(double angle);

//4维向量变为3维向量  /(px,py,vx,vy) --> (rho,phi,rho_dot)
void FourVecToThreeVec(const VectorXd &z, Vector3d &z_);
