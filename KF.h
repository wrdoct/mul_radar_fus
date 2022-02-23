#pragma once
#include "radar_json.h"
#include "tools.h"

//索引:MatrixXd矩阵只能用(),VectorXd不仅能用()还能用[]
using Eigen::MatrixXd;
using Eigen::VectorXd;

class KalmanFilter {
public:
	KalmanFilter();	// Constructor
	virtual ~KalmanFilter();  // Destructor  //基类中的析构函数必须为虚函数，否则会出现对象释放错误

	/**
	* Init Initializes Kalman filter
	* 初始化卡尔曼滤波器
	* @param x_in 初始状态
	* @param P_in 初始状态协方差矩阵
	* @param F_in 状态转移矩阵
	* @param H_in 量测矩阵
	* @param R_in 量测噪声协方差矩阵
	* @param Q_in 过程噪声协方差矩阵  /过程噪声是表示预测位置时，物体位置的不确定性
	**/
	void Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in, MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in);

	/**
	* 使用过程模型预测状态和状态协方差
	* @param delta_T Time between k and k+1 in s    k和k+1之间的时间，用s表示
	**/
	void Predict();

	/**
	* 一般卡尔曼滤波器更新操作
	* @param y the update prediction error  更新预测误差 （测量残差/新息 y = z - H * x ;z为第k帧量测）
	*/
	void UpdateRoutine(const VectorXd &y);

	/**
	* 使用标准卡尔曼滤波方程更新状态
	* @param z The measurement at k+1    在k+1处的测量值
	*/
	void Update(const VectorXd &z);

	/**
	* 利用扩展卡尔曼滤波方程更新状态
	* @param z The measurement at k+1    在k+1处的测量值
	*/
	void UpdateEKF(const VectorXd &z);

public:
	VectorXd x_;		// 状态向量矩阵
	MatrixXd P_;		// 状态协方差矩阵
	MatrixXd F_;		// 状态转移矩阵
	MatrixXd H_;		// 量测矩阵
	MatrixXd R_;		// 量测噪声协方差矩阵
	MatrixXd Q_;		// 过程噪声协方差矩阵	

};
