#pragma once

/*�������״�ɼ�������ת����ͬһ��ȫ������ϵ��֮���ȵó������״���ص�����
�����״�ļ�����򳤶�ΪL1���ϡ�����������̨�״���ص�������򳤶�ΪL2���״�ä������Ϊ��L��
���������״��⵽Ŀ�공����X������Ϊx1�����������״��⵽Ŀ�공����X������Ϊx2��
����L��x1��L2+��L��L1-L2��x2��L1+��Lʱ�����ж���Ŀ�공��Ϊ�ص����������Ŀ�ꡣ*/

#include "coordinate_transformation.h"

//�ص������жϺ���������ӦΪ�����״�����ݣ����ӦΪ���������״��ص���������ݣ��Լ������ص�����֮�������״�����ݣ�
//�������õ�ʱ����ԣ����ص�����������е�һ����������ת��ȥ��NNDA���������ص�����֮��������е�һ����������ת��������ϵͳ����
void JudgmentOverlap(std::vector<RadarInfo_t> &radar1FrameData, std::vector<RadarInfo_t> &radar2FrameData,
	std::vector<RadarInfo_t> &radar1OverlapOutput, std::vector<RadarInfo_t> &radar2OverlapOutput,
	std::vector<RadarInfo_t> &radar1OverlapExcept, std::vector<RadarInfo_t> &radar2OverlapExcept);
