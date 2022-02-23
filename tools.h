#pragma once
/*�������״�ɼ�������ת����ͬһ��ȫ������ϵ��֮���ȵó������״���ص�����
�����״�ļ�����򳤶�ΪL1���ϡ�����������̨�״���ص�������򳤶�ΪL2���״�ä������Ϊ��L��
���������״��⵽Ŀ�공����X������Ϊx1�����������״��⵽Ŀ�공����X������Ϊx2��
����L��x1��L2+��L��L1-L2��x2��L1+��Lʱ�����ж���Ŀ�공��Ϊ�ص����������Ŀ�ꡣ*/


#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "radar_json.h"

//#include <math.h>
#define M_PI       3.14159265358979323846   // pi

using namespace std;
using namespace Eigen;

static double angle = M_PI / 4.0;
static double a = 1.0, b = 1.0;

class TOOLS {
public:
	TOOLS();
	virtual ~TOOLS();//�����е�������������Ϊ�麯�����������ֶ����ͷŴ���

	//�ص������жϺ���������ӦΪ�����״�����ݣ����ӦΪ���������״��ص���������ݣ��Լ������ص�����֮�������״�����ݣ�
	//�������õ�ʱ����ԣ����ص�����������е�һ����������ת��ȥ��NNDA���������ص�����֮��������е�һ����������ת��������ϵͳ����
	void JudgmentOverlap(std::vector<RadarInfo_t> &radar1FrameData, std::vector<RadarInfo_t> &radar2FrameData,
		std::vector<RadarInfo_t> &radar1OverlapOutput, std::vector<RadarInfo_t> &radar2OverlapOutput,
		std::vector<RadarInfo_t> &radar1OverlapExcept, std::vector<RadarInfo_t> &radar2OverlapExcept);

	//��ȡ�ٶ�
	void ExtractVel(const vector<RadarInfo_t> &radarInfos, vector<Eigen::Vector2d> &radarVels);

	//��ȡλ��
	void ExtractPos(const vector<RadarInfo_t> &radarInfos, vector<Eigen::Vector2d> &radarPoints);

	//�ռ�����ת��
	void SpaceTrans(Eigen::Vector2d &radar_pos, double &angle, double &a, double &b, Eigen::Vector2d &radartoshare_pos);

	//������Ҫ����㷨Ч��������������������ʵ�������ж������ļ���׼�������������(Root mean squared error)��
   //����״̬����ʵ״̬֮��Ĳ�ֵ�����в����Щ�в���ƽ��Ȼ�����ֵ�Ϳ��Եõ���������
   //������������ //����Ϊ ����״̬����ʵ״̬
	Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, const std::vector<Eigen::VectorXd> &groundTruth);


	//�����ſɱȾ���
	Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& xState);

	
private:

};

//helper ��������׼����:
double AngleNormalization(double angle);

//4ά������Ϊ3ά����  /(px,py,vx,vy) --> (rho,phi,rho_dot)
void FourVecToThreeVec(const VectorXd &z, Vector3d &z_);
