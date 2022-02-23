#include "KF.h"

using namespace std;

KalmanFilter::KalmanFilter() {}   //���캯��
KalmanFilter::~KalmanFilter() {}  //��������

//��ʼ���������˲���
void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in, MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
	x_ = x_in;  //��ʼ״̬����
	P_ = P_in;  //״̬Э�������
	F_ = F_in;  //״̬ת�ƾ���
	H_ = H_in;  //�������
	R_ = R_in;  //��������Э�������
	Q_ = Q_in;  //��������Э�������
}

//ʹ�ù���ģ��Ԥ��״̬��״̬Э����
void KalmanFilter::Predict() {
	x_ = F_ * x_;   //KF�ĵ�һ��ʽ��//Ԥ��״ֵ̬
	//cout << "��Ԥ��һ֡x_��" << x_ << endl;
	MatrixXd Ft = F_.transpose(); //ת��
	P_ = F_ * P_ * Ft + Q_;  //KF�ĵڶ���ʽ��//Ԥ��ֵ����ʵֵ֮���Ԥ�����Э�������
}

//һ�㿨�����˲������²���   ����ϵͳ
void KalmanFilter::UpdateRoutine(const VectorXd &y) {
	MatrixXd Ht = H_.transpose();
	//cout << "Ht:" << Ht << endl;
	MatrixXd S = H_ * P_ * Ht + R_; //��ϢЭ���� S = H_ * P_ * Ht + R_
	//cout << "S:" << S << endl;
	MatrixXd Si = S.inverse(); //��
	//cout << "Si:" << Si << endl;

	MatrixXd K = P_ * Ht * Si; //KF�ĵ�����ʽ��//����������
	//cout << "K:" << K << endl;

	x_ = x_ + K * y;  //KF�ĵ��ĸ�ʽ��//����ֵ   //�������в�/��Ϣ y = z - H * x ;zΪ��k֡���⣩
	//cout << "x_" << x_ << endl;
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size); //x_size * x_size �ĵ�λ����
	P_ = (I - K * H_) * P_;  //KF�ĵ����ʽ��//����ֵ����ʵֵ֮������Э�������
	//cout << "P_:" << P_ << endl;
	std::cout << "UpdateRoutine:" << __LINE__ << " " << __FILE__ << std::endl; // __ LINE __ ��ʾ���뵱ǰԴ�����кŵ�����; __FILE__����ָʾ�����������Դ�ļ����ļ���

	return;
}

//ʹ�ñ�׼�������˲����̸���״̬
void KalmanFilter::Update(const VectorXd &z) {
	VectorXd z_pred = H_ * x_;  //��k+1����Ԥ��ֵ
	//cout << "z_pred:" << z_pred << endl;
	//cout << "z:" << z << endl;
	VectorXd y = z - z_pred; //�����в�
	//cout << "y:" << y << endl;
	UpdateRoutine(y);

	return;
}

//ʹ����չ�������˲����̸���״̬
void KalmanFilter::UpdateEKF(const VectorXd &z) {
	//h(x)�������Ի�����չ�������˲���(EKF)�Ĺؼ�
	VectorXd hx = VectorXd(4); //��ָ����ν�Ԥ���λ�ú��ٶ�ӳ�䵽���룬�ǶȺ;���仯�ʵļ�����  /�����Բ�������hx  //4 x 1 ����  //��hx��һ��4��С��������ͬ������ռ�δ��ʼ��Ԫ��

	float px = x_(0);
	float py = x_(1);
	float vx = x_(2);
	float vy = x_(3);

	//Ŀ����� �������ԭ�㵽Ŀ��ľ������
	float rho = sqrt(pow(px, 2) + pow(py, 2));  //pow(x,y)���� x �� y ���ݵ�ֵ������: x:���衣���������������֡� y:���衣����������������

	//��λ�� ����Ǧ�������x����ļн�
	float phi = 0.0; // Ĭ������� px ��0��
	if (fabs(px) > 0.0001) {  //fabs��ָ������ȡ����ֵ
		phi = atan2(py, px);  //double atan2(double y,double x) ���ص���ԭ������(x,y)�ķ�λ�ǣ����� x ��ļнǡ�����ֵ�ĵ�λΪ���ȣ�ȡֵ��ΧΪ��-��, ��]�����Ϊ����ʾ�� X ����ʱ����ת�ĽǶȣ����Ϊ����ʾ�� X ��˳ʱ����ת�ĽǶȡ���Ҫ�öȱ�ʾ������ֵ���뽫����ٳ��� 180/�С�����Ҫע����ǣ�����atan2(y,x)�в�����˳���ǵ��õģ�atan2(y,x)�����ֵ�൱�ڵ�(x,y)�ĽǶ�ֵ

	}
	//�����ٶ� ����仯�ʦ����ٶ�v����������L�����ϵ�ͶӰ
	float rho_dot = 0; // ��� �� = 0�������Ĭ����� 
	if (fabs(rho) > 0.0001) {  //fabs��ָ������ȡ����ֵ
		rho_dot = (px*vx + py * vy) / rho;  //dot ���(�ڻ�)����ӦԪ��������,�����һ������(��һ����)
	}
	hx << rho, phi, rho_dot; //����hx ����  �����Բ�������hx
	VectorXd y = z - hx;
	//Vector3d z_;
	//FourVecToThreeVec(z, z_);
	//VectorXd y = z_ - hx;

	// normalize result to -pi and pi
	y(1) = AngleNormalization(y(1)); //y(1)�Ƿ�λ��

	UpdateRoutine(y);

	return;
}

