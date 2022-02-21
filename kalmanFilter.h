#pragma once
#include "radar_json.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class KalmanFilter {
public:
	KalmanFilter();	// Constructor
	virtual ~KalmanFilter();  // Destructor

	/**
	* Init Initializes Kalman filter
	* 初始化卡尔曼滤波器
	* @param x_in Initial state             初始状态
	* @param P_in Initial state covariance  初始状态协方差矩阵
	* @param F_in Transition matrix         状态转移矩阵
	* @param H_in Measurement matrix        量测矩阵
	* @param R_in Measurement covariance matrix   量测噪声协方差矩阵
	* @param Q_in Process covariance matrix       过程噪声协方差矩阵  /过程噪声是表示预测位置时，物体位置的不确定性
	**/
	void Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in, MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in);

	/**
	* Prediction Predicts the state and the state covariance
	* using the process model
	* 使用过程模型预测状态和状态协方差
	* @param delta_T Time between k and k+1 in s    k和k+1之间的时间，用s表示
	**/
	void Predict();

	/**
	* General kalman filter update operations
	* 一般卡尔曼滤波器更新操作
	* @param y the update prediction error  更新预测误差 （测量残差/新息 y = z - H * x ;z为第k帧量测）
	*/
	void UpdateRoutine(const VectorXd &y);

	/**
	* Updates the state by using standard Kalman Filter equations
	* 使用标准卡尔曼滤波方程更新状态
	* @param z The measurement at k+1    在k+1处的测量值
	*/
	void Update(const VectorXd &z);

	/**
	* Updates the state by using Extended Kalman Filter equations
	* 利用扩展卡尔曼滤波方程更新状态
	* @param z The measurement at k+1    在k+1处的测量值
	*/
	void UpdateEKF(const VectorXd &z);

public:
	VectorXd x_;		// state vector         /状态向量矩阵
	MatrixXd P_;		// state covariance matrix     /状态协方差矩阵
	MatrixXd F_;		// state transistion matrix    /状态转移矩阵
	MatrixXd H_;		// measurement matrix   /量测矩阵
	MatrixXd R_;		// measurement covariance matrix   /量测噪声协方差矩阵
	MatrixXd Q_;		// process covariance matrix   /过程噪声协方差矩阵	

	vector<RadarInfo_t> radarPreFrameData;

};
