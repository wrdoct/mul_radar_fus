#pragma once

#include "KF.h"

//索引:MatrixXd矩阵只能用(),VectorXd不仅能用()还能用[]
using Eigen::MatrixXd;
using Eigen::VectorXd;


class EKF {
public:
	EKF();
	~EKF();


	/**
	* 利用扩展卡尔曼滤波方程更新状态
	* @param z The measurement at k+1    在k+1处的测量值
	*/
	void UpdateEKF(const VectorXd &z);


	void Process(std::vector<RadarInfo_t> &measurementPackList);

	//从这里 运行卡尔曼滤波器的全部流程。
	void ProcessMeasurement(RadarInfo_t &measurePackList);

	//卡尔曼滤波器更新和预测 在这里。  KalmanFilter为一个类
	KalmanFilter ekf;


	//保存上一帧数据
	void SavePreFrame();


private:

	bool isInitialized; //检查跟踪工具箱是否初始化(第一次测量)

	long long previousTimestamp; //上一个时间戳

	TOOLS *tools;  //用于计算雅可比和 RMSE 的工具对象

	Eigen::MatrixXd R_radar_;  //雷达测量噪声

	Eigen::MatrixXd H_jacobian;  // 雷达测量函数

	//加速度噪声分量
	//（加速度ax， ay是一个均值为0，标准偏差为sigma_ax 和 sigma_ay的随机矢量） 给定sigma_ax=sigma_ay =3; noise_ax = sigma_ax * sigma_ax, noise_ay = sigma_ay * sigma_ay
	float noise_ax;
	float noise_ay;

	bool flag;//一帧数据结束标志
	std::vector<RadarInfo_t> frameAllTar;
	std::vector<RadarInfo_t> radarPreFrameData;//保存上一帧数据的容器

	// 使用这个估计值计算RMSE  //RMSE 均方根误差
	std::vector<VectorXd> estimations;

};
