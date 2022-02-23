#pragma once
#include "radar_json.h"
#include "tools.h"

//#define _ENABLE_EXTENDED_ALIGNED_STORAGE //Ԥ����� ������

//���Խڵ��״�A�ĺ���i�ͽڵ��״�B�ĺ���j��kʱ�̵�״̬���Ʋ�ΪXij(k) = �״�1�ڵ�k֡�ĵ�i��Ŀ�꺽��״̬ - �״�2�ڵ�k֡�ĵ�j��Ŀ�꺽��״̬
//�õ���ά����������ά���ٶ�ά��������ά��������ֵ����e����С����ֵ��ɹ�����

using namespace std;
using namespace Eigen;

const double nnda_Pi = 3.1415926;

static double rangeThreshold = 200000; //������ֵ
static double velThreshold = 100000;//�ٶ���ֵ

class NNDA {
public:
	NNDA();
	~NNDA();

public:

	//���ά������
	//void GetDimensionalData(const vector<RadarInfo_t> &radarInfos, vector<Eigen::Vector2d> &radarPoints, vector<Eigen::Vector2d> &radarVels, vector<int> &radarRoadId);
	void GetDimensionalData(const RadarInfo_t &radarInfos, Eigen::Vector2d &radarPoints, Eigen::Vector2d &radarVels, int &radarRoadId);

	//�����ݱ�Ϊ 2��num�� �ľ���
	void TransMatrix(Eigen::Vector2d &radarData, vector<MatrixXd> &tmpData);

	void Fusion(vector<RadarInfo_t> &radar1_OverlapStatus, vector<RadarInfo_t> &radar2_OverlapStatus, vector<RadarInfo_t> &mul_radarFusion);
		
private:

	TOOLS *tools;

	int radar1_measureNum;//�״�1Ŀ����� / ��������
	int radar2_measureNum;//�״�2Ŀ����� / ��������

	Eigen::Vector2d radar1OverlapPoints;//�״�1�ص������λ��
	Eigen::Vector2d radar1OverlapVels;//�״�1�ص�������ٶ�
	int radar1OverlapRoadId;//�״�1�ص�����ĳ�����

	Eigen::Vector2d radar2OverlapPoints;//�״�2�ص������λ��
	Eigen::Vector2d radar2OverlapVels;//�״�2�ص�������ٶ�
	int radar2OverlapRoadId;//�״�2�ص�����ĳ�����

};
