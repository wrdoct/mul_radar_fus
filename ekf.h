#pragma once

#include "kalmanFilter.h"

class FusionEKF {
public:
	FusionEKF(); //Constructor.
	virtual ~FusionEKF(); //Destructor. //基类中的析构函数必须为虚函数，否则会出现对象释放错误

	/*A helper method to calculate Jacobians.*/  //计算雅可比矩阵
	Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& xState);

	//从这里 运行卡尔曼滤波器的全部流程。  timestamp时间戳
	void ProcessMeasurement(float px, float py, float vx, float vy, long long timestamp);// Run the whole flow of the Kalman Filter from here.

	//卡尔曼滤波器更新和预测 在这里。  KalmanFilter为一个类
	KalmanFilter ekf;//Kalman Filter update and prediction math lives in here.

private:
	// check whether the tracking toolbox was initiallized or not (first measurement)
	bool isInitialized; //检查跟踪工具箱是否初始化(第一次测量)

	// previous timestamp
	long long previousTimestamp; //上一个时间戳

	// tool object used to compute Jacobian and RMSE   用于计算雅可比和 RMSE 的工具对象
	//Tools tools;  //用于计算雅可比和 RMSE 的工具对象

	Eigen::MatrixXd R_radar_;    // radar measurement noise   雷达测量噪声

	Eigen::MatrixXd H_jacobian;         // measurement function for radar  雷达测量函数

	//acceleration noise components   加速度噪声分量
	//（加速度ax， ay是一个均值为0，标准偏差为sigma_ax 和 sigma_ay的随机矢量）在本项目中我们给定sigma_ax=sigma_ay =3; noise_ax = sigma_ax * sigma_ax, noise_ay = sigma_ay * sigma_ay
	float noise_ax;
	float noise_ay;

};
