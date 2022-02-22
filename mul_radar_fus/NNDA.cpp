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

//获得维度数据
void NNDA::GetDimensionalData(const RadarInfo_t &radarInfos, Eigen::Vector2d &radarPoints, Eigen::Vector2d &radarVels, int &radarRoadId) {
	//for (auto &iter : radarInfos) {
		//获取位置
		Eigen::Vector2d radarPos;
		radarPos[0] = radarInfos.position[0];
		radarPos[1] = radarInfos.position[1];
		//radarPoints.emplace_back(radarPos);
		radarPoints << radarPos[0],
			radarPos[1];
		//获取速度
		Eigen::Vector2d radarVel;
		radarVel[0] = radarInfos.velocity[0];
		radarVel[1] = radarInfos.velocity[1];
		//radarVels.emplace_back(radarVel);
		radarVels << radarVel[0],
			radarVel[1];
		//获取车道号
		//radarRoadId.emplace_back(radarInfos.road_id);
		radarRoadId = radarInfos.road_id;
		//获取车辆id，即
		
	//}
	/*for (size_t idx = 0; idx < radarPoints.size(); idx++) {
		cout << "radarPoints:" << "(" << radarPoints[idx][0] << "," << radarPoints[idx][1] << ")" << endl;
	}*/
}

//将数据变为 2行num列 的矩阵
void NNDA::TransMatrix(Eigen::Vector2d &radarData, vector<MatrixXd> &tmpData) {
	int num = radarData.size();
	//tmpData中为每次在每个航迹，观测矩阵波门范围内的目标个数//2行num列	
	MatrixXd tmp;
	tmp.resize(2, num); //2行num列	
	/*for (int k = 0; k < radarData.size(); k++) {
		tmp(0, k) = radarData[k][0]; //第一行为x值
		tmp(1, k) = radarData[k][1]; //第二行为y值
	}*/
	tmp(0, 0) = radarData[0]; //第一行为x值
	tmp(1, 0) = radarData[1]; //第二行为y值
	tmpData.push_back(tmp);
	//cout << "tmpData.size():" << tmpData.size() << endl;//永远是1 //1帧数据只压入一次

	return;
}

void NNDA::Fusion(vector<RadarInfo_t> &radar1_OverlapStatus, vector<RadarInfo_t> &radar2_OverlapStatus, vector<RadarInfo_t> &mul_radarFusion) {
	int radar1_measureNum = radar1_OverlapStatus.size(); //雷达1目标个数 / 航迹个数
	int radar2_measureNum = radar2_OverlapStatus.size(); //雷达2目标个数 / 航迹个数
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
			
		//遍历雷达2的数据
		for (int j = 0; j < radar2_measureNum; j++) {
			Eigen::Vector2d radar2OverlapPoints;
			Eigen::Vector2d radar2OverlapVels;
			int radar2OverlapRoadId;				
			NNDA::GetDimensionalData(radar2_OverlapStatus[j], radar2OverlapPoints, radar2OverlapVels, radar2OverlapRoadId);		
									
			Eigen::Vector2d overlap_radartoshare_pos;
			tools->SpaceTrans(radar1OverlapPoints, angle, a, b, overlap_radartoshare_pos);//重叠区域的数据经过坐标转换之后进行NNDA（假设以雷达2为公共坐标系）
			
			//将数据变为 2行num列 的矩阵
			vector<MatrixXd> radar1_tmpDataPos, radar1_tmpDataVel;
			//TransMatrix(overlap_radartoshare_pos, radar1_tmpDataPos); TransMatrix(radar1OverlapVels, radar1_tmpDataVel);
			TransMatrix(radar1OverlapPoints, radar1_tmpDataPos); TransMatrix(radar1OverlapVels, radar1_tmpDataVel);
			vector<MatrixXd> radar2_tmpDataPos, radar2_tmpDataVel;
			TransMatrix(radar2OverlapPoints, radar2_tmpDataPos); TransMatrix(radar2OverlapVels, radar2_tmpDataVel);

			//执行nnda的操作
			MatrixXd d = radar1_tmpDataPos.at(0).col(0) - radar2_tmpDataPos.at(0).col(0); //雷达1的每列减去雷达2的每列
			//cout << "d = " << d << endl;
			MatrixXd D = d.transpose() * d;
			double dd = sqrt(D.determinant());//距离差值

			MatrixXd v = radar1_tmpDataVel.at(0).col(0) - radar2_tmpDataVel.at(0).col(0); //雷达1的每列减去雷达2的每列
			//cout << "v = " << v << endl;
			MatrixXd V = v.transpose() * v;
			double vv = sqrt(V.determinant());//速度差值

			//cout << "radar1OverlapRoadId:" << radar1OverlapRoadId << "    " << "radar2OverlapRoadId:" << radar2OverlapRoadId << endl;
			//cout << "dd:" << dd << "    " << "vv:" << vv << endl;
			if (radar1OverlapRoadId == radar2OverlapRoadId) {
				if (dd <= rangeThreshold && vv <= velThreshold) {
					//cout << "最终成功关联的航迹" << endl;
					//cout << "radar1_OverlapStatus[i].car_id:" << radar1_OverlapStatus[i].car_id << endl;
					//cout << "radar2_OverlapStatus[j].car_id:" << radar2_OverlapStatus[j].car_id << endl;

					//凸组合加权航迹融合
					//radar2_OverlapStatus[j].car_id = radar1_OverlapStatus[i].car_id; //id变为雷达1的id
					cout << "radar2_OverlapStatus[j].car_id 和 radar1_OverlapStatus[i].car_id 为同一辆车" << endl;						
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

