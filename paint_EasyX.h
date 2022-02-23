#pragma once
#include "tools.h"

#include <graphics.h>		// ����ͼ�ο�ͷ�ļ�
#include <conio.h>

using namespace std;
using namespace Eigen;     // �ĳ�������� using Eigen::MatrixXd; 

//��ͼ //��֤����ʹ��
class GRAPH {
public:
	GRAPH();
	~GRAPH();

void radarPaint(vector<RadarInfo_t> &radarFrameInfos);

//��ʾ�Ĳ�������
void ParaConfig();

//������
void Road();

private:
	int carNum;
	IMAGE img;

	int x1;
	int y1;
	int road_w;
	int appear_length;
};
