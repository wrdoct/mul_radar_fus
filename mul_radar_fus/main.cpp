#include "radar_json.h"
#include "tools.h"
#include "paint_EasyX.h"
#include "NNDA.h"
#include "EKF.h"

#include <thread>
#include <mutex>

//�״��ļ�·��
std::string radar1_path = "C:\\Users\\LLW\\Desktop\\mul_radar_fus\\radar.json";
std::string radar2_path = "C:\\Users\\LLW\\Desktop\\mul_radar_fus\\radar.json";
//����ļ�·��
std::string camera_path = "xxxxx";

mutex mtx1, mtx2;//������//���߳�����
condition_variable cv;// ������������(�������̼߳��ͬ��ͨ��)

//�״����ص�����ʱʹ��NNDA
std::shared_ptr<TOOLS> tool = std::make_shared<TOOLS>(); 
std::shared_ptr<NNDA> nnda = std::make_shared<NNDA>();

//�״�û���ص�����ʱʹ��EKF
//EKF *ekf_api = new EKF();//ʵ����EKFָ�� //�ú����ɾ��
//����ָ��:������Դ�Ĺ����Զ��ͷ�û��ָ�����õ���Դ
//std::shared_ptr<EKF> ekf_api(new EKF);
std::shared_ptr<EKF> ekf_api = std::make_shared<EKF>(); //make_sharedҪ����ʹ��new��make_shared����һ�ν���Ҫ�ڴ�����

//���干�����������������
vector<RadarInfo_t> radar_infos, radar_frame_infos;
void Radar1Thread() {

	// ��ȡmtx��������Դ
	unique_lock<std::mutex> lock1(mtx1);

	//radar
	//vector<RadarInfo_t> radar_infos, radar_frame_infos;//����RadarInfo_t��һ������
	LoadRadarInfo(radar1_path, radar_infos);
	/*for (auto iter = radar_infos.begin(); iter != radar_infos.end();) {
		std::cout << "iter->car_id = " << iter->car_id << std::endl;
		std::cout << "iter->road_id = " << iter->road_id << std::endl;
		std::cout << "iter->position[0] = " << iter->position[0] << "  "        //x
			<< "iter->position[1] = " << iter->position[1] << std::endl; //y
		std::cout << "iter->velocity[0] = " << iter->velocity[0] << "  "        //x
			<< "iter->velocity[1] = " << iter->velocity[1] << std::endl  //y
			<< std::endl;
		++iter;
	}*/

	//���ݰ���ʱ��֡�ʷ���
	//RadarFrame(radar_infos, radar_frame_infos);
	int frame_num = 1; int target_num = 1;
	std::vector<RadarInfo_t> radarFrameDataBefore;
	for (size_t idx = 1; idx < radar_infos.size(); idx++) {
		//cout << radarObjects[idx].timeStamp << endl;
		if (radar_infos[idx].timeStamp == radar_infos[idx - 1].timeStamp) {
			//cout << "ʱ������" << endl;
			target_num++;
			//radarFrameDataBefore[frame_num].life = target_num;//��һ֡���Ŀ�����
			radarFrameDataBefore.push_back(radar_infos[idx - 1]);
		}
		else {
			//cout << "ʱ�������ȣ�֡����һ" << endl;
			cout << "��" << frame_num << "֡�ڵ�Ŀ�����Ϊ��" << target_num << endl;	
			frame_num++;
			target_num = 1; //�ٴγ�ʼ�� ��λĿ�����

			radarFrameDataBefore.push_back(radar_infos[idx - 1]);
			radar_frame_infos.swap(radarFrameDataBefore);//radar_frame_infos��Ϊ֡����
			radarFrameDataBefore.clear();
			/*cout << "radar_frame_infos.size() = " << radar_frame_infos.size() << endl;
			for (size_t idx = 0; idx < radar_frame_infos.size(); idx++) {
				cout << "radar_frame_infos[idx].car_id:" << radar_frame_infos[idx].car_id << endl;
			}*/

			////��֡����������////
			while (!radar_frame_infos.empty()) {
				// �ж�������Ϊ�գ�����ȴ�����������״̬���ͷ�mtx����
				// ��RadarFusionThread�߳��������ܹ�ȥ��������
				cv.wait(lock1);
			}

			//֪ͨ�ȴ���cv���������ϵĴ����̣߳����Կ�ʼ���������ˣ�Ȼ���ͷ���mtx
			cv.notify_all();
			//����һ֡���ݣ�˯��100ms
			//std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
	}
	cout << "��֡��frame_num = " << frame_num - 1 << endl;//

	/*
	//ʹ�õ���Ŀ��Ĳ���
	vector<Eigen::Vector2d> radarSinglePoints;
	SingleTarget(radar_infos, radarSinglePoints);
	cout << "idΪ0��radarSinglePoints.size()=" << radarSinglePoints.size() << endl;
	for (size_t idx = 0; idx < radarSinglePoints.size(); idx++) {
		cout << "radarSinglePoints:" << "(" << radarSinglePoints[idx][0] / 100000 << "," << radarSinglePoints[idx][1] / 100000 << ")" << endl;
	}
	*/	

	return;

}

//���干�����������������
vector<RadarInfo_t> radar2_infos, radar2_frame_infos;
void Radar2Thread() {
	//mtx.lock();

	// ��ȡmtx��������Դ
	unique_lock<std::mutex> lock2(mtx2);
	//���������Ϊ�գ�����������δ����
	/*while (!radar2_frame_infos.empty()) {
		// �ж�������Ϊ�գ�����ȴ�����������״̬���ͷ�mtx����
		// ��RadarFusionThread�߳��������ܹ�ȥ��������
		cv.wait(lock);
	}*/

	//radar
	LoadRadarInfo(radar2_path, radar2_infos);

	//���ݰ���ʱ��֡�ʷ���
//RadarFrame(radar_infos, radar_frame_infos);
	int frame_num = 1; int target_num = 1;
	std::vector<RadarInfo_t> radar2FrameDataBefore;
	for (size_t idx = 1; idx < radar2_infos.size(); idx++) {
		//cout << radarObjects[idx].timeStamp << endl;
		if (radar2_infos[idx].timeStamp == radar2_infos[idx - 1].timeStamp) {
			//cout << "ʱ������" << endl;
			target_num++;
			//radarFrameDataBefore[frame_num].life = target_num;//��һ֡���Ŀ�����
			radar2FrameDataBefore.push_back(radar2_infos[idx - 1]);
		}
		else {
			//cout << "ʱ�������ȣ�֡����һ" << endl;
			//cout << "��" << frame_num << "֡�ڵ�Ŀ�����Ϊ��" << target_num << endl;			
			frame_num++;
			target_num = 1; //�ٴγ�ʼ�� ��λĿ�����

			radar2FrameDataBefore.push_back(radar2_infos[idx - 1]);
			radar2_frame_infos.swap(radar2FrameDataBefore);//radar_frame_infos��Ϊ֡����
			radar2FrameDataBefore.clear();
			/*cout << "radar2_frame_infos.size() = " << radar2_frame_infos.size() << endl;
			for (size_t idx = 0; idx < radar2_frame_infos.size(); idx++) {
				cout << "radar_frame_infos[idx].car_id:" << radar2_frame_infos[idx].car_id << endl;
			}*/

			////��֡����������////
			//���������Ϊ�գ�����������δ����
			while (!radar2_frame_infos.empty()) {
				// �ж�������Ϊ�գ�����ȴ�����������״̬���ͷ�mtx����
				// ��RadarFusionThread�߳��������ܹ�ȥ��������
				cv.wait(lock2);
			}

			//֪ͨ�ȴ���cv���������ϵĴ����̣߳����Կ�ʼ���������ˣ�Ȼ���ͷ���mtx
			cv.notify_all();
			//����һ֡���ݣ�˯��100ms
			//std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
	}
	//cout << "��֡��frame_num = " << frame_num - 1 << endl;//6628֡

	//mtx.unlock();
	
}

void RadarFusionThread() {
	//mtx.lock();

	//initgraph(1800, 900); //����ͼ�ν��棬����:ͼ�ο�ȣ�ͼ�θ߶� //��ͼ

	//ÿ����һ֡����֪ͨ�״��̲߳���һ֡
	while(1){
		// ��ȡmtx��������Դ
		unique_lock<std::mutex> lock1(mtx1), lock2(mtx2);
		//���������Ϊ�գ�����������δ����
		while (radar_frame_infos.empty()) {
			// �ж�����Ϊ�գ�����ȴ�����������״̬���ͷ�mtx����
			// ��Radar1Thread�߳��������ܹ�ȥ��������
			cv.wait(lock1);
		}
		while (radar2_frame_infos.empty()) {
			// �ж�����Ϊ�գ�����ȴ�����������״̬���ͷ�mtx����
			// ��Radar2Thread�߳��������ܹ�ȥ��������
			cv.wait(lock2);
		}

		/*
		cout << "radar_frame_infos.size() = " << radar_frame_infos.size() << endl; 
		for (size_t idx = 0; idx < radar_frame_infos.size(); idx++) {
			cout << "radar_frame_infos[idx].car_id:" << radar_frame_infos[idx].car_id << endl;
		}
		*/	

		/*
		cout << "radar2_frame_infos.size() = " << radar2_frame_infos.size() << endl;
		for (size_t idx = 0; idx < radar2_frame_infos.size(); idx++) {
			cout << "radar2_frame_infos[idx].car_id:" << radar2_frame_infos[idx].car_id << endl;
		}
		*/
				
		vector<RadarInfo_t> radar_fusion_result;	//������  //������
		/*----------------------------��һ��������״�֮������ص�����-------------------------*/
		//�״�֮���ص�������ж�
		vector<RadarInfo_t> radar1OverlapOutput, radar2OverlapOutput;
		vector<RadarInfo_t> radar1OverlapExcept, radar2OverlapExcept;
		tool->JudgmentOverlap(radar_frame_infos, radar2_frame_infos, radar1OverlapOutput, radar2OverlapOutput, radar1OverlapExcept, radar2OverlapExcept);
		/*
		cout << "��֡�������ص������ж�������radar1OverlapOutput.size() = " << radar1OverlapOutput.size() << endl;
		cout << "��֡���״�1�������ص������ж�������radar1OverlapExcept.size() = " << radar1OverlapExcept.size() << endl;
		cout << "��֡�������ص������ж�������radar2OverlapOutput.size() = " << radar2OverlapOutput.size() << endl;
		cout << "��֡���״�2�������ص������ж�������radar2OverlapExcept.size() = " << radar2OverlapExcept.size() << endl;
		*/
						
		if (!radar1OverlapOutput.empty() || !radar2OverlapOutput.empty()) {		
			//�������״�2Ϊ��������ϵ/���״�1�����ݽ�������ת��   
			
			////�ص�����֮������ݾ�������ת��֮��ֱ�ӽ�����ʾ���������״�2Ϊ��������ϵ��//���Ҹı�car_id
			for (size_t idx = 0; idx < radar1OverlapExcept.size(); idx++) {

				//cout << "before radar1OverlapExcept[idx].position:" << "(" << radar1OverlapExcept[idx].position[0] << "," << radar1OverlapExcept[idx].position[1] << ")" << endl;
				//�ռ�����ת��
				radar1OverlapExcept[idx].position[0] = radar1OverlapExcept[idx].position[0] * cos(angle) + radar1OverlapExcept[idx].position[1] * sin(angle) + a;
				radar1OverlapExcept[idx].position[1] = radar1OverlapExcept[idx].position[0] * (-sin(angle)) + radar1OverlapExcept[idx].position[1] * cos(angle) + b;
				radar_fusion_result.push_back(radar1OverlapExcept[idx]);//��һ����
			}
						
			//�жϾ����<������ֵ���ٶȲ�<�ٶ���ֵ����������� --> ����͹��ϼ�Ȩƽ���ĺ����ں�
			//NNDA
			vector<RadarInfo_t> mul_radar_fusion;
			nnda->Fusion(radar1OverlapOutput, radar2OverlapOutput, mul_radar_fusion);

			//�жϳ�ͬһ����֮��ѳ���������ϢҲ������ȥ

			for (size_t idx = 0; idx < mul_radar_fusion.size(); idx++) {
				radar_fusion_result.push_back(mul_radar_fusion[idx]);//�ڶ�����
			}
			for (size_t idx = 0; idx < radar2OverlapExcept.size(); idx++) {
				radar_fusion_result.push_back(radar2OverlapExcept[idx]);//��������
			}

			cout << "radar_fusion_result" << endl;

		}
		
		/*----------------------------�ڶ���������״�֮�䲻�����ص�����-------------------------*/
		else {
			//EKF				
			ekf_api->Process(radar_frame_infos);// EKF ����
		}
		
		/*
		//��ͼ
		//system("cls");  //��յ�ǰ����̨ ִ�л�ͼ
		vector<Eigen::Vector2d> testPoint;
		ExtractPos(radar_frame_infos, testPoint);
		radarPaint(testPoint);
		*/

		radar_frame_infos.clear(); radar2_frame_infos.clear();

		//֪ͨ�ȴ���cv���������ϵĲ����̣߳����Կ�ʼ���������ˣ�Ȼ���ͷ���mtx
		cv.notify_all();
		//����һ֡���ݣ�˯��100ms
		//std::this_thread::sleep_for(std::chrono::milliseconds(100));

	}
	//delete(ekf_api);//�ֶ��ͷ�EKF���ڴ�
	//mtx.unlock();

}

int main() {
	
	thread th1(Radar1Thread);//��һ������Ϊ���������ڶ�������Ϊ�ú����ĵ�һ������������ú������ն������������д�ں��档��ʱ�߳̿�ʼִ�С�
	//cout << "���߳�����ʾ���߳�idΪ" << th1.get_id() << endl;
	thread th2(Radar2Thread);	
	thread th3(RadarFusionThread);

	th1.join();//��ʱ���̱߳�����ֱ�����߳�ִ�н�����
	th2.join();
	th3.join();
		
	system("pause");//�밴���������...
	return 0;
}
