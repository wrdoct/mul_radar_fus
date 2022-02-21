#include "ekf.h"

using namespace std;

//���캯��  ��ʼ��
FusionEKF::FusionEKF() {
	//isInitialized = false;
	isInitialized = true;
	previousTimestamp = 0;

	// initializing matrices
	R_radar_ = MatrixXd(4, 4);
	H_jacobian = MatrixXd(4, 4);

	//measurement covariance matrix - radar  ����Э�������-�״�
	R_radar_ << 0.09, 0, 0, 0,
		0, 0.0009, 0, 0,
		0, 0, 0.09, 0,
		0, 0, 0, 0.0009;

	// Radar - jacobian matrix  �״���ſɱȾ���
	H_jacobian << 0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 0;

	//����ѡ����ʵĲ�����ʼ��P��������˲���֪��׼ȷ�ĳ�ʼλ�ã������ǿ��Ը���һ�������Ϊ0�ĳ�ʼЭ�������
	//һ�������һ��Ԥ���ǻ������еĲ������������������������������¿��Գ�����Radar����Lidar�Ĳ���������������ʼ��P����
	// initialize the kalman filter variables   //��ʼ���������˲�����
	ekf.P_ = MatrixXd(4, 4);//�������Э������󣬻������� 
	ekf.P_ << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1000, 0,
		0, 0, 0, 1000;

	//��px;py;vx;vy�� = F * ��px;py;vx;vy��//״̬ת�ƾ���
	ekf.F_ = MatrixXd(4, 4);
	ekf.F_ << 1, 0, 1, 0,
		0, 1, 0, 1,
		0, 0, 1, 0,
		0, 0, 0, 1;

	// Initialize process noise covariance matrix   ��ʼ����������Э�������
	ekf.Q_ = MatrixXd(4, 4);
	ekf.Q_ << 0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 0;

	// Initialize ekf state   ��ʼ�� ekf ״̬
	ekf.x_ = VectorXd(4); //4 x 1 ����
	ekf.x_ << 1, 1, 1, 1;

	// set measurement noises  �̶���������  
	//�����ٶ�ax�� ay��һ����ֵΪ0����׼ƫ��Ϊsigma_ax �� sigma_ay�����ʸ�����ڱ���Ŀ�����Ǹ���sigma_ax=sigma_ay =3; 
	noise_ax = 9; //noise_ax = sigma_ax * sigma_ax
	noise_ay = 9; //noise_ay = sigma_ay * sigma_ay

}

//��������
FusionEKF::~FusionEKF() {}

//�����ſɱȾ���
MatrixXd FusionEKF::CalculateJacobian(const VectorXd& xState) {
	MatrixXd Hj(3, 4); //3 x 4 ����
	//recover state parameters  �ָ�״̬����
	float px = xState(0);
	float py = xState(1);
	float vx = xState(2);
	float vy = xState(3);

	// preparation of Jacobian terms  �ſɱ�������׼��
	//pre-compute a set of terms which recur in the Jacobian to avoid repeated calculation  Ԥ�ȼ���һ�����ſɱ����ظ����ֵ���Ա����ظ�����
	float c1 = px * px + py * py;
	float c2 = sqrt(c1);
	float c3 = (c1 * c2);

	if (fabs(c1) < 0.0001) {
		//cout << "ERROR - Division by Zero" << endl;
		cout << "CalculateJacobian () - Error - Division by Zero" << endl;
		Hj.fill(0.0); //��һ��������ֵ�������ʱ�����ǾͿ���ʹ��fill()����
		return Hj;
	}

	// compute jacobian matrix  �����ſɱȾ���
	Hj << (px / c2), (py / c2), 0, 0,
		-(py / c1), (px / c1), 0, 0,
		py * (vx * py - vy * px) / c3, px * (px * vy - py * vx) / c3, px / c2, py / c2;

	return Hj;

}

//������ ��ʼ ���п������˲�����ȫ�����̡�   timestampʱ���
void FusionEKF::ProcessMeasurement(float px, float py, float vx, float vy, long long timestamp) {
	Eigen::VectorXd rawMeasurements = Eigen::VectorXd(4);
	rawMeasurements << px, py, vx, vy;
	cout << "rawMeasurements: " << rawMeasurements << endl;
	//Initialization
	if (!isInitialized) {
		// first measurement
		ekf.x_ = VectorXd(4);
		ekf.x_ << px, py, vx, vy;  // px, py, vx, vy

		//previousTimestamp = timestamp;

		// done initializing, no need to predict or update
		isInitialized = true;  //��ʼ����ϣ�����Ԥ������
		return;
	}

	//  Prediction

	/**
	* Update the state transition matrix F according to the new elapsed time. �����µ�����ʱ�����״̬ת�ƾ���F
	* Time is measured in seconds.   ʱ���������
	* Update the process noise covariance matrix.  ���¹�������Э�������
	* Use noise_ax = 9 and noise_ay = 9 for your Q matrix.  ʹ�� noise _ ax = 9�� noise _ ay = 9��ʾQ����
	**/

	// compute the time elapsed between the current and previous measurements   //���㵱ǰ��������ǰ����֮�侭����ʱ��
	// float dt = (timestamp_ - previous_timestamp_) / 1000000.0;  //  in seconds
	float dt = timestamp / 1000.0;  //  in seconds
	// previousTimestamp = timestamp;

	float dt_2 = dt * dt;   //ƽ��
	float dt_3 = dt_2 * dt; //����
	float dt_4 = dt_3 * dt; //�Ĵη�

	// Modify the F matrix so that the time is integrated   �޸�F����ʹʱ��õ����� /integrated����//EKF��״̬ת�ƾ���
	/*����Ϊ��
	ekf.F_ <<  1, 0, dt, 0,
			   0, 1, 0, dt,
			   0, 0, 1, 0,
			   0, 0, 0, 1;*/
	ekf.F_(0, 2) = dt; //1��3��  dt--ʱ���
	ekf.F_(1, 3) = dt; //2��4��  dt--ʱ���

	//set the process covariance matrix Q   ���ù���Э�������Q�� ���������������ص�Э�������
	ekf.Q_ = MatrixXd(4, 4);
	ekf.Q_ << dt_4 / 4 * noise_ax, 0, dt_3 / 2 * noise_ax, 0,
		0, dt_4 / 4 * noise_ay, 0, dt_3 / 2 * noise_ay,
		dt_3 / 2 * noise_ax, 0, dt_2 * noise_ax, 0,
		0, dt_3 / 2 * noise_ay, 0, dt_2 * noise_ay;

	if (dt > 0.001)
		ekf.Predict();

	//Update
	/**
	* Use the sensor type to perform the update step.  ʹ�ô���������ִ�и��²��衣
	* Update the state and covariance matrices.  ����״̬��Э�������
	**/
	//H_jacobian = CalculateJacobian(ekf.x_);
	ekf.H_ = H_jacobian;
	ekf.R_ = R_radar_;
	ekf.UpdateEKF(rawMeasurements);
	//ekf.Update(rawMeasurements);

	return;
}
