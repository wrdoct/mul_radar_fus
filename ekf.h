#pragma once

#include "kalmanFilter.h"

class FusionEKF {
public:
	FusionEKF(); //Constructor.
	virtual ~FusionEKF(); //Destructor. //�����е�������������Ϊ�麯�����������ֶ����ͷŴ���

	/*A helper method to calculate Jacobians.*/  //�����ſɱȾ���
	Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& xState);

	//������ ���п������˲�����ȫ�����̡�  timestampʱ���
	void ProcessMeasurement(float px, float py, float vx, float vy, long long timestamp);// Run the whole flow of the Kalman Filter from here.

	//�������˲������º�Ԥ�� �����  KalmanFilterΪһ����
	KalmanFilter ekf;//Kalman Filter update and prediction math lives in here.

private:
	// check whether the tracking toolbox was initiallized or not (first measurement)
	bool isInitialized; //�����ٹ������Ƿ��ʼ��(��һ�β���)

	// previous timestamp
	long long previousTimestamp; //��һ��ʱ���

	// tool object used to compute Jacobian and RMSE   ���ڼ����ſɱȺ� RMSE �Ĺ��߶���
	//Tools tools;  //���ڼ����ſɱȺ� RMSE �Ĺ��߶���

	Eigen::MatrixXd R_radar_;    // radar measurement noise   �״��������

	Eigen::MatrixXd H_jacobian;         // measurement function for radar  �״��������

	//acceleration noise components   ���ٶ���������
	//�����ٶ�ax�� ay��һ����ֵΪ0����׼ƫ��Ϊsigma_ax �� sigma_ay�����ʸ�����ڱ���Ŀ�����Ǹ���sigma_ax=sigma_ay =3; noise_ax = sigma_ax * sigma_ax, noise_ay = sigma_ay * sigma_ay
	float noise_ax;
	float noise_ay;

};
