#include "EKF.h"

using namespace std;

//���캯��  ��ʼ��
EKF::EKF() {
	isInitialized = false;//�����ٹ������Ƿ��ʼ��(��һ�β���)
	previousTimestamp = 0;//��һ��ʱ���

	tools = new TOOLS();

	// initializing matrices
	R_radar_ = MatrixXd(4, 4);
	H_jacobian = MatrixXd(4, 4);

	//measurement covariance matrix - radar  ����Э�������-�״�
	R_radar_ << 0.09, 0, 0, 0,
		0, 0.0009, 0, 0,
		0, 0, 0.09, 0,
		0, 0, 0, 0.0009;

	// Radar - jacobian matrix  �״���ſɱȾ���
	H_jacobian << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;
	
	//����ѡ����ʵĲ�����ʼ��P��������˲���֪��׼ȷ�ĳ�ʼλ�ã������ǿ��Ը���һ�������Ϊ0�ĳ�ʼЭ�������
	//һ�������һ��Ԥ���ǻ������еĲ������������������������������¿��Գ�����Radar�Ĳ���������������ʼ��P����
	//��ʼ���������˲�����
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

	//��ʼ����������Э�������
	ekf.Q_ = MatrixXd(4, 4);
	ekf.Q_ << 0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 0;

	//��ʼ�� ekf ״̬
	ekf.x_ = VectorXd(4); //4 x 1 ����
	ekf.x_ << 1, 
		1, 
		1, 
		1;

	//�̶���������  
	//�����ٶ�ax�� ay��һ����ֵΪ0����׼ƫ��Ϊsigma_ax �� sigma_ay�����ʸ�����ڱ���Ŀ�����Ǹ���sigma_ax=sigma_ay =3; 
	noise_ax = 9; //noise_ax = sigma_ax * sigma_ax
	noise_ay = 9; //noise_ay = sigma_ay * sigma_ay

	flag = false;
}

//��������
EKF::~EKF() {
	delete(tools);
}


void EKF::Process(std::vector<RadarInfo_t> &measurementPackList) {
	//Call the EKF-based fusion
	size_t N = measurementPackList.size();
	//cout << "N = " << N << endl;//��һ֡��Ŀ�����

	//��ȡ������Ϣ
	for (size_t k = 0; k < N; ++k) {
		/*cout << "radar_frame_infos[idx].car_id : " << measurementPackList[k].car_id << "   "
			<< "radarPoints:" << "(" << measurementPackList[k].position[0] << "," << measurementPackList[k].position[1] << ")" << "   "
			<< "radarVels:" << "(" << measurementPackList[k].velocity[0] << "," << measurementPackList[k].velocity[1] << ")" << endl;*/
		//cout << "radarPreFrameData.size():" << radarPreFrameData.size() << endl;

		//�ȸ�����һ֡�����г���id����һ֡������ĳ���id���бȶ��жϸ�Ŀ���Ƿ��ǵ�һ�γ��֣��Ծ����Ƿ���г�ʼ��
		for (size_t j = 0; j < radarPreFrameData.size(); j++) {
			if (measurementPackList[k].car_id == radarPreFrameData[j].car_id) {
				isInitialized = true;
				break;
			}
			else {
				isInitialized = false;
			}
		}

		ProcessMeasurement(measurementPackList[k]);

		estimations.push_back(ekf.x_); //���Ź���ֵ  VectorXd x_;/״̬��������/��Ϊȫ�ֱ��� ���ı仯��KF/EKF�ĵ��ĸ�ʽ��
		/*cout << "estimations.size():" << estimations.size() << endl;
		for (size_t idx = 0; idx < estimations.size(); idx++) {
			cout << "estimations:" << "(" << estimations[idx][0] << "," << estimations[idx][1] << ")" << endl;
		}*/
		//ekf.Predict();

		tools->CalculateRMSE(estimations, estimations);
		estimations.clear();
	}
	flag = true;//һ֡���ݽ���
	SavePreFrame();
}

void EKF::ProcessMeasurement(RadarInfo_t &measurePackList) {
	Eigen::VectorXd rawMeasurements = Eigen::VectorXd(4);
	rawMeasurements << measurePackList.position[0] / 100000,
		measurePackList.position[1] / 100000,
		measurePackList.velocity[0] / 100000,
		measurePackList.velocity[1] / 100000; //��mΪ��λ

	//Initialization
	if (!isInitialized) {
		// first measurement
		ekf.x_ = VectorXd(4); //��ʼ����Ŀ���һ�γ��ֵĳ�ʼ״̬
		ekf.x_ << measurePackList.position[0] / 100000,
			measurePackList.position[1] / 100000,
			measurePackList.velocity[0] / 100000,
			measurePackList.velocity[1] / 100000; //��mΪ��λ
		//cout << "��Ŀ��Ϊ��Ŀ�꣬ekf.x_:" << ekf.x_ << endl;

		isInitialized = true;  //��ʼ�����

		frameAllTar.push_back(measurePackList);

		return;//Ŀ���һ�γ��֣���ʼ��֮�󷵻�
	}

	//Prediction
	/**
	* �����µ�����ʱ�����״̬ת�ƾ���F
	* ʱ���������
	* ���¹�������Э�������
	* ʹ�� noise _ ax = 9�� noise _ ay = 9��ʾQ����
	**/

	//���㵱ǰ��������ǰ����֮�侭����ʱ��
	float dt = (measurePackList.timeStamp - radarPreFrameData[0].timeStamp) / 1000.0;  //  in seconds

	float dt_2 = dt * dt;   //ƽ��
	float dt_3 = dt_2 * dt; //����
	float dt_4 = dt_3 * dt; //�Ĵη�

	// �޸�F����ʹʱ��õ����� //EKF��״̬ת�ƾ���
	/*����Ϊ��
	ekf.F_ <<  1, 0, dt, 0,
			   0, 1, 0, dt,
			   0, 0, 1, 0,
			   0, 0, 0, 1;*/
	ekf.F_(0, 2) = dt; //1��3��  dt--ʱ���
	ekf.F_(1, 3) = dt; //2��4��  dt--ʱ���

	//���ù���Э�������Q�����������������ص�Э�������
	ekf.Q_ = MatrixXd(4, 4);
	ekf.Q_ << dt_4 / 4 * noise_ax, 0, dt_3 / 2 * noise_ax, 0,
		0, dt_4 / 4 * noise_ay, 0, dt_3 / 2 * noise_ay,
		dt_3 / 2 * noise_ax, 0, dt_2 * noise_ax, 0,
		0, dt_3 / 2 * noise_ay, 0, dt_2 * noise_ay;

	if (dt > 0.001)
		ekf.Predict();

	//Update
	/**����״̬��Э�������**/
	//H_jacobian = tools->CalculateJacobian(ekf.x_);
	//cout << "after ekf.x_ Jacobian = " << H_jacobian << endl;

	ekf.H_ = H_jacobian;
	ekf.R_ = R_radar_;
	//ekf.UpdateEKF(rawMeasurements);
	ekf.Update(rawMeasurements);
	
	frameAllTar.push_back(measurePackList);
	
	return;
}

void  EKF::SavePreFrame() {	
	if (flag) {
		radarPreFrameData.swap(frameAllTar);
		frameAllTar.clear();
		flag = false;
		//cout << "һ֡�����ѱ���" << endl;
	}	
	//cout << "radarPreFrameData.size():" << radarPreFrameData.size() << endl;
}