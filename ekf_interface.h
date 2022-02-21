#pragma once
//interface 接口

#include <Eigen/Dense>
#include <iostream>
#include <vector>

#include "ekf.h"
//#include "tools.hpp"
//#include "measurement.hpp"

//索引:MatrixXd矩阵只能用(),VectorXd不仅能用()还能用[]
using Eigen::MatrixXd;
using Eigen::VectorXd;

class EKF_API
{
public:
	EKF_API();
	~EKF_API();

	void process(std::vector<RadarInfo_t> measurementPackList);

private:
	// Create a Fusion EKF instance  //instance 实例
	FusionEKF *fusionEKF;

	// used to compute the RMSE later  使用这个估计值计算RMSE  //RMSE 均方根误差
	std::vector<VectorXd> estimations;
};
