#include "NNDA.h"

NNDA::NNDA() {
	tools = new TOOLS();
}

NNDA::~NNDA() {
	delete(tools);
}

void NNDA::init() {

	rangeThreshold = 2; //
	velThreshold = 1;
		
}

//���ά������
void NNDA::GetDimensionalData(const RadarInfo_t &radarInfos, Eigen::Vector2d &radarPoints, Eigen::Vector2d &radarVels, int &radarRoadId) {
	//for (auto &iter : radarInfos) {
		//��ȡλ��
		Eigen::Vector2d radarPos;
		radarPos[0] = radarInfos.position[0];
		radarPos[1] = radarInfos.position[1];
		//radarPoints.emplace_back(radarPos);
		radarPoints << radarPos[0],
			radarPos[1];
		//��ȡ�ٶ�
		Eigen::Vector2d radarVel;
		radarVel[0] = radarInfos.velocity[0];
		radarVel[1] = radarInfos.velocity[1];
		//radarVels.emplace_back(radarVel);
		radarVels << radarVel[0],
			radarVel[1];
		//��ȡ������
		//radarRoadId.emplace_back(radarInfos.road_id);
		radarRoadId = radarInfos.road_id;
		//��ȡ����id����
		
	//}
	/*for (size_t idx = 0; idx < radarPoints.size(); idx++) {
		cout << "radarPoints:" << "(" << radarPoints[idx][0] << "," << radarPoints[idx][1] << ")" << endl;
	}*/
}

//�����ݱ�Ϊ 2��num�� �ľ���
void NNDA::TransMatrix(Eigen::Vector2d &radarData, vector<MatrixXd> &tmpData) {
	int num = radarData.size();
	//tmpData��Ϊÿ����ÿ���������۲�����ŷ�Χ�ڵ�Ŀ�����//2��num��	
	MatrixXd tmp;
	tmp.resize(2, num); //2��num��	
	/*for (int k = 0; k < radarData.size(); k++) {
		tmp(0, k) = radarData[k][0]; //��һ��Ϊxֵ
		tmp(1, k) = radarData[k][1]; //�ڶ���Ϊyֵ
	}*/
	tmp(0, 0) = radarData[0]; //��һ��Ϊxֵ
	tmp(1, 0) = radarData[1]; //�ڶ���Ϊyֵ
	tmpData.push_back(tmp);
	//cout << "tmpData.size():" << tmpData.size() << endl;//��Զ��1 //1֡����ֻѹ��һ��

	return;
}

void NNDA::Fusion(vector<RadarInfo_t> &radar1_OverlapStatus, vector<RadarInfo_t> &radar2_OverlapStatus, vector<RadarInfo_t> &mul_radarFusion) {
	int radar1_measureNum = radar1_OverlapStatus.size(); //�״�1Ŀ����� / ��������
	int radar2_measureNum = radar2_OverlapStatus.size(); //�״�2Ŀ����� / ��������
	//cout << "typeid(radar2_OverlapStatus).name() : " << typeid(radar2_OverlapStatus).name() << endl; //
	for (int i = 0; i < radar1_measureNum; i++) {
		Eigen::Vector2d radar1OverlapPoints;
		Eigen::Vector2d radar1OverlapVels;
		int radar1OverlapRoadId;
		NNDA::GetDimensionalData(radar1_OverlapStatus[i], radar1OverlapPoints, radar1OverlapVels, radar1OverlapRoadId);
		/*
		cout << "radar2Points.size()=" << radar1OverlapPoints.size() << endl;//2
		cout << "radar1OverlapPoints:" << "(" << radar1OverlapPoints[0] << "," << radar1OverlapPoints[1] << ")" << endl;
		cout << "radar1OverlapVels.size()=" << radar1OverlapVels.size() << endl;//2
		cout << "radar1OverlapVels:" << "(" << radar1OverlapVels[0] << "," << radar1OverlapVels[1] << ")" << endl;
		cout << "radar1OverlapRoadId:" << radar1OverlapRoadId << endl;
		*/
			
		//�����״�2������
		for (int j = 0; j < radar2_measureNum; j++) {
			Eigen::Vector2d radar2OverlapPoints;
			Eigen::Vector2d radar2OverlapVels;
			int radar2OverlapRoadId;				
			NNDA::GetDimensionalData(radar2_OverlapStatus[j], radar2OverlapPoints, radar2OverlapVels, radar2OverlapRoadId);		
									
			Eigen::Vector2d overlap_radartoshare_pos;
			tools->SpaceTrans(radar1OverlapPoints, angle, a, b, overlap_radartoshare_pos);//�ص���������ݾ�������ת��֮�����NNDA���������״�2Ϊ��������ϵ��
			
			//�����ݱ�Ϊ 2��num�� �ľ���
			vector<MatrixXd> radar1_tmpDataPos, radar1_tmpDataVel;
			//TransMatrix(overlap_radartoshare_pos, radar1_tmpDataPos); TransMatrix(radar1OverlapVels, radar1_tmpDataVel);
			TransMatrix(radar1OverlapPoints, radar1_tmpDataPos); TransMatrix(radar1OverlapVels, radar1_tmpDataVel);
			vector<MatrixXd> radar2_tmpDataPos, radar2_tmpDataVel;
			TransMatrix(radar2OverlapPoints, radar2_tmpDataPos); TransMatrix(radar2OverlapVels, radar2_tmpDataVel);

			//ִ��nnda�Ĳ���
			MatrixXd d = radar1_tmpDataPos.at(0).col(0) - radar2_tmpDataPos.at(0).col(0); //�״�1��ÿ�м�ȥ�״�2��ÿ��
			//cout << "d = " << d << endl;
			MatrixXd D = d.transpose() * d;
			double dd = sqrt(D.determinant());//�����ֵ

			MatrixXd v = radar1_tmpDataVel.at(0).col(0) - radar2_tmpDataVel.at(0).col(0); //�״�1��ÿ�м�ȥ�״�2��ÿ��
			//cout << "v = " << v << endl;
			MatrixXd V = v.transpose() * v;
			double vv = sqrt(V.determinant());//�ٶȲ�ֵ

			//cout << "radar1OverlapRoadId:" << radar1OverlapRoadId << "    " << "radar2OverlapRoadId:" << radar2OverlapRoadId << endl;
			//cout << "dd:" << dd << "    " << "vv:" << vv << endl;
			if (radar1OverlapRoadId == radar2OverlapRoadId) {
				if (dd <= rangeThreshold && vv <= velThreshold) {
					//cout << "���ճɹ������ĺ���" << endl;
					//cout << "radar1_OverlapStatus[i].car_id:" << radar1_OverlapStatus[i].car_id << endl;
					//cout << "radar2_OverlapStatus[j].car_id:" << radar2_OverlapStatus[j].car_id << endl;

					//͹��ϼ�Ȩ�����ں�
					//radar2_OverlapStatus[j].car_id = radar1_OverlapStatus[i].car_id; //id��Ϊ�״�1��id
					cout << "radar2_OverlapStatus[j].car_id �� radar1_OverlapStatus[i].car_id Ϊͬһ����" << endl;						
					radar2_OverlapStatus[j].position[0] = (radar1_OverlapStatus[i].position[0] + radar2_OverlapStatus[j].position[0]) / 2;
					radar2_OverlapStatus[j].position[1] = (radar1_OverlapStatus[i].position[1] + radar2_OverlapStatus[j].position[1]) / 2;
					radar2_OverlapStatus[j].velocity[0] = (radar1_OverlapStatus[i].position[0] + radar2_OverlapStatus[j].position[0]) / 2;
					radar2_OverlapStatus[j].velocity[1] = (radar1_OverlapStatus[i].velocity[1] + radar2_OverlapStatus[j].velocity[1]) / 2;
					mul_radarFusion.emplace_back(radar2_OverlapStatus[j]);												
				}
			}
							
		}
	}

	return;
}

