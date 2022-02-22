#pragma once
#include "radar_json.h"
#include "tools.h"


//���Խڵ��״�A�ĺ���i�ͽڵ��״�B�ĺ���j��kʱ�̵�״̬���Ʋ�ΪXij(k) = �״�1�ڵ�k֡�ĵ�i��Ŀ�꺽��״̬ - �״�2�ڵ�k֡�ĵ�j��Ŀ�꺽��״̬
//�õ���ά����������ά���ٶ�ά��������ά��������ֵ����e����С����ֵ��ɹ�����

using namespace std;
using namespace Eigen;

const double nnda_Pi = 3.1415926;

class NNDA {
public:
	NNDA();
	~NNDA();

public:

	void init();

	//���ά������
	//void GetDimensionalData(const vector<RadarInfo_t> &radarInfos, vector<Eigen::Vector2d> &radarPoints, vector<Eigen::Vector2d> &radarVels, vector<int> &radarRoadId);
	void GetDimensionalData(const RadarInfo_t &radarInfos, Eigen::Vector2d &radarPoints, Eigen::Vector2d &radarVels, int &radarRoadId);

	//�����ݱ�Ϊ 2��num�� �ľ���
	void TransMatrix(Eigen::Vector2d &radarData, vector<MatrixXd> &tmpData);

	void Fusion(vector<RadarInfo_t> &radar1_OverlapStatus, vector<RadarInfo_t> &radar2_OverlapStatus, vector<RadarInfo_t> &mul_radarFusion);
		
private:

	TOOLS *tools;

	//������ֵ
	int rangeThreshold;
	//�ٶ���ֵ
	int velThreshold;
};
