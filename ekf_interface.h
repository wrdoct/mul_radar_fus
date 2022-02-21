#pragma once
//interface �ӿ�

#include <Eigen/Dense>
#include <iostream>
#include <vector>

#include "ekf.h"
//#include "tools.hpp"
//#include "measurement.hpp"

//����:MatrixXd����ֻ����(),VectorXd��������()������[]
using Eigen::MatrixXd;
using Eigen::VectorXd;

class EKF_API
{
public:
	EKF_API();
	~EKF_API();

	void process(std::vector<RadarInfo_t> measurementPackList);

private:
	// Create a Fusion EKF instance  //instance ʵ��
	FusionEKF *fusionEKF;

	// used to compute the RMSE later  ʹ���������ֵ����RMSE  //RMSE ���������
	std::vector<VectorXd> estimations;
};
