//�������˲��Ľӿ�  //��װ
#include "ekf_interface.h"

using namespace std;

//���캯��
EKF_API::EKF_API() {
	fusionEKF = new FusionEKF();
}
//��������
EKF_API::~EKF_API() {
	delete(fusionEKF);
}

void EKF_API::process(std::vector<RadarInfo_t> measurementPackList) {
	//Call the EKF-based fusion
	size_t N = measurementPackList.size();
	cout << "N = " << N << endl;//Ŀ�����

	//�ȸ�����һ֡�����г���id����һ֡������ĳ���id���бȶ��жϸ�Ŀ���Ƿ��ǵ�һ�γ��֣��Ѿ����Ƿ���г�ʼ��


	//��ȡ������Ϣ
	for (size_t k = 0; k < N; ++k) {

		cout << "radar_frame_infos[idx].car_id : " << measurementPackList[k].car_id << "   "
			<< "radarPoints:" << "(" << measurementPackList[k].position[0] << "," << measurementPackList[k].position[1] << ")" << "   "
			<< "radarVels:" << "(" << measurementPackList[k].velocity[0] << "," << measurementPackList[k].velocity[1] << ")" << endl;
		
		// start filtering from the second frame (the speed is unknown in the first frame)
		//�ӵڶ�֡��ʼ����(�ڵ�һ֡���ٶ���δ֪��)
		fusionEKF->ProcessMeasurement(measurementPackList[k].position[0], //px
			measurementPackList[k].position[1], //py
			measurementPackList[k].velocity[0],              //vx
			measurementPackList[k].velocity[1],         //vy
			measurementPackList[k].timeStamp);         //ʱ���  //ms//����֡��
				
		estimations.push_back(fusionEKF->ekf.x_); //���Ź���ֵ  VectorXd x_;/״̬��������/��Ϊȫ�ֱ��� ���ı仯��KF/EKF�ĵ��ĸ�ʽ��
		/*cout << "estimations.size():" << estimations.size() << endl;
		for (size_t idx = 0; idx < estimations.size(); idx++) {
			cout << "estimations:" << "(" << estimations[idx][0] << "," << estimations[idx][1] << ")" << endl;
		}*/
		
	}
	
	//EKF ����֮�������
	cout << "EKF Done!" << endl;

	return;
}