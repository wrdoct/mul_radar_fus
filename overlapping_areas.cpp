#include "overlapping_areas.h"

//�����״�ļ�����򳤶�ΪL1
#define radarDetectLength 20000000 //m * 10^5
//�ϡ�����������̨�״���ص�������򳤶�ΪL2
#define overLappingLength 2000000
//�״�ä������Ϊ��L
#define blindAreaLength 2000000

//���������״��⵽Ŀ�공����X������Ϊx1�����������״��⵽Ŀ�공����X������Ϊx2��
//����L��x1��L2 + ��L��L1 - L2��x2��L1 + ��Lʱ�����ж���Ŀ�공��Ϊ�ص����������Ŀ��
void JudgmentOverlap(std::vector<RadarInfo_t> &radar1FrameData, std::vector<RadarInfo_t> &radar2FrameData, 
	std::vector<RadarInfo_t> &radar1OverlapOutput, std::vector<RadarInfo_t> &radar2OverlapOutput,
	std::vector<RadarInfo_t> &radar1OverlapExcept, std::vector<RadarInfo_t> &radar2OverlapExcept) {
	//��ȡλ��
	vector<Eigen::Vector2d> radar1Points, radar2Points;
	ExtractPos(radar1FrameData, radar1Points);
	ExtractPos(radar2FrameData, radar2Points);

	//�������״�2Ϊ��������ϵ
	for (size_t idx = 0; idx < min(radar1Points.size(), radar2Points.size()); idx++) {
		if (radar1Points[idx][0] >= blindAreaLength && radar1Points[idx][0] <= overLappingLength + blindAreaLength){// && 
			//radar2Points[idx][0] >= radarDetectLength - overLappingLength && radar2Points[idx][0] <= radarDetectLength + blindAreaLength) {
			cout << "��Ŀ�공��Ϊ�ص����������Ŀ��" << endl;
			//cout << "radar1FrameData[idx].car_id : " << radar1FrameData[idx].car_id << endl;
			radar1OverlapOutput.push_back(radar1FrameData[idx]);
			radar2OverlapOutput.push_back(radar1FrameData[idx]);		
		}
		else {
			radar1OverlapExcept.push_back(radar1FrameData[idx]);
			radar2OverlapExcept.push_back(radar1FrameData[idx]);
		}
	}

	return;
}
