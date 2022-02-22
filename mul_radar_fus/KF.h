#pragma once
#include "radar_json.h"
#include "tools.h"

//����:MatrixXd����ֻ����(),VectorXd��������()������[]
using Eigen::MatrixXd;
using Eigen::VectorXd;

class KalmanFilter {
public:
	KalmanFilter();	// Constructor
	virtual ~KalmanFilter();  // Destructor  //�����е�������������Ϊ�麯�����������ֶ����ͷŴ���

	/**
	* Init Initializes Kalman filter
	* ��ʼ���������˲���
	* @param x_in ��ʼ״̬
	* @param P_in ��ʼ״̬Э�������
	* @param F_in ״̬ת�ƾ���
	* @param H_in �������
	* @param R_in ��������Э�������
	* @param Q_in ��������Э�������  /���������Ǳ�ʾԤ��λ��ʱ������λ�õĲ�ȷ����
	**/
	void Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in, MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in);

	/**
	* ʹ�ù���ģ��Ԥ��״̬��״̬Э����
	* @param delta_T Time between k and k+1 in s    k��k+1֮���ʱ�䣬��s��ʾ
	**/
	void Predict();

	/**
	* һ�㿨�����˲������²���
	* @param y the update prediction error  ����Ԥ����� �������в�/��Ϣ y = z - H * x ;zΪ��k֡���⣩
	*/
	void UpdateRoutine(const VectorXd &y);

	/**
	* ʹ�ñ�׼�������˲����̸���״̬
	* @param z The measurement at k+1    ��k+1���Ĳ���ֵ
	*/
	void Update(const VectorXd &z);

	/**
	* ������չ�������˲����̸���״̬
	* @param z The measurement at k+1    ��k+1���Ĳ���ֵ
	*/
	void UpdateEKF(const VectorXd &z);

public:
	VectorXd x_;		// ״̬��������
	MatrixXd P_;		// ״̬Э�������
	MatrixXd F_;		// ״̬ת�ƾ���
	MatrixXd H_;		// �������
	MatrixXd R_;		// ��������Э�������
	MatrixXd Q_;		// ��������Э�������	

};
