#include "coordinate_transformation.h"
/*
Eigen::Vector2d radar1_pos;
Eigen::Vector2d radar1_vel;
Eigen::Vector2d radar2_pos;
Eigen::Vector2d radar2_vel;
*/

void ExtractVel(const vector<RadarInfo_t> &radarInfos, vector<Eigen::Vector2d> &radarVels) {
	for (auto &iter : radarInfos) {
		Eigen::Vector2d radarVel;
		radarVel[0] = iter.velocity[0];
		radarVel[1] = iter.velocity[1];
		radarVels.emplace_back(radarVel);
	}
}

void ExtractPos(const vector<RadarInfo_t> &radarInfos, vector<Eigen::Vector2d> &radarPoints) {
	/*for (auto iter = radarInfos.begin(); iter != radarInfos.end();) {
		cout << "iter->position[0] = " << iter->position[0] << "  "        //x
			<< "iter->position[1] = " << iter->position[1] << endl; //y

		radarPos << iter->position[0],
			iter->position[1];
		++iter;
	}*/
	for (auto &iter : radarInfos) {
		Eigen::Vector2d radarPos;
		radarPos[0] = iter.position[0];
		radarPos[1] = iter.position[1];
		radarPoints.emplace_back(radarPos);
	}
	/*for (size_t idx = 0; idx < radarPoints.size(); idx++) {
		cout << "radarPoints:" << "(" << radarPoints[idx][0] << "," << radarPoints[idx][1] << ")" << endl;
	}*/
}

//将数据中的Px,Py进行坐标转换
void SpaceTrans(Eigen::Vector2d &radar_pos, double &angle, double &a, double &b, Eigen::Vector2d &radartoshare_pos) {
	//旋转矩阵
	Eigen::Matrix2d R; //定义
	//赋值
	R << cos(angle), sin(angle),
		-sin(angle), cos(angle);

	//平移矩阵
	Eigen::Vector2d T; //定义
	//赋值
	T << a,
		b;

	/*
	for (size_t idx = 0; idx < radar_pos.size(); idx++) {
		Eigen::Vector2d radarPos_input, radarPos_output;
		radarPos_input << radar_pos[idx][0],
			radar_pos[idx][1];
		radarPos_output = R * radarPos_input + T;
		radartoshare_pos.emplace_back(radarPos_output);
	}
	*/
	
		Eigen::Vector2d radarPos_input, radarPos_output;
		radarPos_input << radar_pos[0],
			radar_pos[1];
		radarPos_output = R * radarPos_input + T;

	return;
}

