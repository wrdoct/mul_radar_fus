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
	* ��ʼ���������˲���
	* @param x_in Initial state             ��ʼ״̬
	* @param P_in Initial state covariance  ��ʼ״̬Э�������
	* @param F_in Transition matrix         ״̬ת�ƾ���
	* @param H_in Measurement matrix        �������
	* @param R_in Measurement covariance matrix   ��������Э�������
	* @param Q_in Process covariance matrix       ��������Э�������  /���������Ǳ�ʾԤ��λ��ʱ������λ�õĲ�ȷ����
	**/
	void Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in, MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in);

	/**
	* Prediction Predicts the state and the state covariance
	* using the process model
	* ʹ�ù���ģ��Ԥ��״̬��״̬Э����
	* @param delta_T Time between k and k+1 in s    k��k+1֮���ʱ�䣬��s��ʾ
	**/
	void Predict();

	/**
	* General kalman filter update operations
	* һ�㿨�����˲������²���
	* @param y the update prediction error  ����Ԥ����� �������в�/��Ϣ y = z - H * x ;zΪ��k֡���⣩
	*/
	void UpdateRoutine(const VectorXd &y);

	/**
	* Updates the state by using standard Kalman Filter equations
	* ʹ�ñ�׼�������˲����̸���״̬
	* @param z The measurement at k+1    ��k+1���Ĳ���ֵ
	*/
	void Update(const VectorXd &z);

	/**
	* Updates the state by using Extended Kalman Filter equations
	* ������չ�������˲����̸���״̬
	* @param z The measurement at k+1    ��k+1���Ĳ���ֵ
	*/
	void UpdateEKF(const VectorXd &z);

public:
	VectorXd x_;		// state vector         /״̬��������
	MatrixXd P_;		// state covariance matrix     /״̬Э�������
	MatrixXd F_;		// state transistion matrix    /״̬ת�ƾ���
	MatrixXd H_;		// measurement matrix   /�������
	MatrixXd R_;		// measurement covariance matrix   /��������Э�������
	MatrixXd Q_;		// process covariance matrix   /��������Э�������	

	vector<RadarInfo_t> radarPreFrameData;

};
