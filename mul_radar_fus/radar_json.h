#pragma once
#include <iostream>
#include <json.h>
#include <fstream>
#include <string>

#include <sys/timeb.h>
#include <time.h> 

#include <Eigen/Dense>

using namespace std;
using namespace Eigen;     // �ĳ�������� using Eigen::MatrixXd; 

timeb StringToDatetime(string str);

double StringToDouble(string Input);
int StringToInt(string Input);

typedef struct {
	int timeStamp = 0;//ʱ���//֡�� //double���͸�Ϊint��
	unsigned int radar_id = -1;
	unsigned int car_id = -1;//-1��ʾ�޷������������ֵ65535
	unsigned int road_id = -1;//-1��ʾ�޷������������ֵ65535
	int miss = 0;//��ʧĿ�����
	int life = 0;//�ɹ����Ŀ�����
	Eigen::Vector2d position = { 0.0, 0.0 };//Ŀ��λ��//Vector3d 3ά������ Ĭ��Ϊ������
	Eigen::Vector2d velocity = { 0.0, 0.0 };//Ŀ���ٶ�
}RadarInfo_t;

//�����״���Ϣ
void LoadRadarInfo(const std::string &path,   //����
	std::vector<RadarInfo_t> &radarObjects); //���

//���ݰ���ʱ��֡�ʷ���
void RadarFrame(std::vector<RadarInfo_t> &radarObjects, std::vector<RadarInfo_t> &radarFrameData);

//ʹ�õ���Ŀ��Ĳ���
void SingleTarget(std::vector<RadarInfo_t> &radarObjects, vector<Eigen::Vector2d> &radarSinglePoints);

