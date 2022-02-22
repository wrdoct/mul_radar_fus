#pragma once

#include "KF.h"

//����:MatrixXd����ֻ����(),VectorXd��������()������[]
using Eigen::MatrixXd;
using Eigen::VectorXd;


class EKF {
public:
	EKF();
	~EKF();


	/**
	* ������չ�������˲����̸���״̬
	* @param z The measurement at k+1    ��k+1���Ĳ���ֵ
	*/
	void UpdateEKF(const VectorXd &z);


	void Process(std::vector<RadarInfo_t> &measurementPackList);

	//������ ���п������˲�����ȫ�����̡�
	void ProcessMeasurement(RadarInfo_t &measurePackList);

	//�������˲������º�Ԥ�� �����  KalmanFilterΪһ����
	KalmanFilter ekf;


	//������һ֡����
	void SavePreFrame();


private:

	bool isInitialized; //�����ٹ������Ƿ��ʼ��(��һ�β���)

	long long previousTimestamp; //��һ��ʱ���

	TOOLS *tools;  //���ڼ����ſɱȺ� RMSE �Ĺ��߶���

	Eigen::MatrixXd R_radar_;  //�״��������

	Eigen::MatrixXd H_jacobian;  // �״��������

	//���ٶ���������
	//�����ٶ�ax�� ay��һ����ֵΪ0����׼ƫ��Ϊsigma_ax �� sigma_ay�����ʸ���� ����sigma_ax=sigma_ay =3; noise_ax = sigma_ax * sigma_ax, noise_ay = sigma_ay * sigma_ay
	float noise_ax;
	float noise_ay;

	bool flag;//һ֡���ݽ�����־
	std::vector<RadarInfo_t> frameAllTar;
	std::vector<RadarInfo_t> radarPreFrameData;//������һ֡���ݵ�����

	// ʹ���������ֵ����RMSE  //RMSE ���������
	std::vector<VectorXd> estimations;

};
