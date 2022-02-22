#include "tools.h"

TOOLS::TOOLS() {}

TOOLS::~TOOLS() {}

/*--------------------�ص������ж�------------------------*/

//�����״�ļ�����򳤶�ΪL1
#define radarDetectLength 20000000 //m * 10^5
//�ϡ�����������̨�״���ص�������򳤶�ΪL2
#define overLappingLength 2000000
//�״�ä������Ϊ��L
#define blindAreaLength 2000000

//���������״��⵽Ŀ�공����X������Ϊx1�����������״��⵽Ŀ�공����X������Ϊx2��
//����L��x1��L2 + ��L��L1 - L2��x2��L1 + ��Lʱ�����ж���Ŀ�공��Ϊ�ص����������Ŀ��
void TOOLS::JudgmentOverlap(std::vector<RadarInfo_t> &radar1FrameData, std::vector<RadarInfo_t> &radar2FrameData,
	std::vector<RadarInfo_t> &radar1OverlapOutput, std::vector<RadarInfo_t> &radar2OverlapOutput,
	std::vector<RadarInfo_t> &radar1OverlapExcept, std::vector<RadarInfo_t> &radar2OverlapExcept) {
	//��ȡλ��
	vector<Eigen::Vector2d> radar1Points, radar2Points;
	ExtractPos(radar1FrameData, radar1Points);
	ExtractPos(radar2FrameData, radar2Points);

	//�������״�2Ϊ��������ϵ
	for (size_t idx = 0; idx < min(radar1Points.size(), radar2Points.size()); idx++) {
		if (radar1Points[idx][0] >= blindAreaLength && radar1Points[idx][0] <= overLappingLength + blindAreaLength) {// && 
			//radar2Points[idx][0] >= radarDetectLength - overLappingLength && radar2Points[idx][0] <= radarDetectLength + blindAreaLength) {			
			//cout << "��Ŀ�공��Ϊ�ص����������Ŀ��" << endl;
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


/*--------------------�ռ�����ת��------------------------*/

void TOOLS::ExtractVel(const vector<RadarInfo_t> &radarInfos, vector<Eigen::Vector2d> &radarVels) {
	for (auto &iter : radarInfos) {
		Eigen::Vector2d radarVel;
		radarVel[0] = iter.velocity[0];
		radarVel[1] = iter.velocity[1];
		radarVels.emplace_back(radarVel);
	}
}

void TOOLS::ExtractPos(const vector<RadarInfo_t> &radarInfos, vector<Eigen::Vector2d> &radarPoints) {
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

//�������е�Px,Py��������ת��
void TOOLS::SpaceTrans(Eigen::Vector2d &radar_pos, double &angle, double &a, double &b, Eigen::Vector2d &radartoshare_pos) {
	//��ת����
	Eigen::Matrix2d R; //����
	//��ֵ
	R << cos(angle), sin(angle),
		-sin(angle), cos(angle);

	//ƽ�ƾ���
	Eigen::Vector2d T; //����
	//��ֵ
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

/*--------------------EKF------------------------*/

//RMSE ���������  //����Ϊ ����״̬����ʵ״̬
VectorXd  TOOLS::CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, const std::vector<Eigen::VectorXd> &groundTruth) {
	size_t num_estimations = estimations.size();
	size_t num_groundTruths = groundTruth.size();

	VectorXd rmse(4);
	rmse << 0, 0, 0, 0;

	// sanity check of input validity  //���������Ч��
	// the estimation vector size should not be zero  //����������С��ӦΪ��
	// the estimation vector size should equal ground truth vector size  //����������СӦ���ڻ���ֵ������С
	if (estimations.size() != groundTruth.size() || estimations.size() < 1) {
		std::cout << "Cannot compute RMSE metric. Invalid input size." << std::endl;
		return rmse;
	}

	//����Ϊ ����״̬����ʵ״̬������֮��Ĳ�ֵ�����в����Щ�в���ƽ��Ȼ�����ֵ�Ϳ��Եõ����������
	// accumulate residuals   ���۲в�
	for (size_t i = 0; i < estimations.size(); ++i) {
		VectorXd residual = estimations[i] - groundTruth[i]; //����֮��Ĳ�ֵ�����в�

		//coefficient-wise multiplication  ϵ���˷�
		residual = residual.array() * residual.array(); //����Щ�в���ƽ��  //residual.array()��4��1�е�����
		rmse += residual;
	}

	// compute mean  ����ƽ��ֵ
	rmse /= estimations.size(); //rmse = rmse / estimations.size();

	// compute squared root  ������
	rmse = rmse.array().sqrt();

	return rmse;

}


//�����ſɱȾ���
MatrixXd TOOLS::CalculateJacobian(const VectorXd& xState) {
	MatrixXd Hj(3, 4); //3 x 4 ����
	//recover state parameters  �ָ�״̬����
	float px = xState(0);
	float py = xState(1);
	float vx = xState(2);
	float vy = xState(3);

	// preparation of Jacobian terms  �ſɱ�������׼��
	//Ԥ�ȼ���һ�����ſɱ����ظ����ֵ���Ա����ظ�����
	float c1 = px * px + py * py;
	float c2 = sqrt(c1);
	float c3 = (c1 * c2);

	if (fabs(c1) < 0.0001) {
		//cout << "ERROR - Division by Zero" << endl;
		cout << "CalculateJacobian () - Error - Division by Zero" << endl;
		Hj.fill(0.0); //��һ��������ֵ�������ʱ�����ǾͿ���ʹ��fill()����
		return Hj;
	}

	// compute jacobian matrix  �����ſɱȾ���
	Hj << (px / c2), (py / c2), 0, 0,
		-(py / c1), (px / c1), 0, 0,
		py * (vx * py - vy * px) / c3, px * (px * vy - py * vx) / c3, px / c2, py / c2;

	return Hj;
}

//helper ��������׼����:
double AngleNormalization(double angle) {
	// Constrain to less than pi  
	while (angle > M_PI) angle -= 2.0 * M_PI;

	// Constrain to greater than -pi
	while (angle < -M_PI) angle += 2.0 * M_PI;

	return angle;
}

//4ά������Ϊ3ά����  /(px,py,vx,vy) --> (rho,phi,rho_dot)
void FourVecToThreeVec(const VectorXd &z, Vector3d &z_) {	
	float px = z(0);
	float py = z(1);
	float vx = z(2);
	float vy = z(3);

	//Ŀ����� �������ԭ�㵽Ŀ��ľ������
	float rho = sqrt(pow(px, 2) + pow(py, 2));  //pow(x,y)���� x �� y ���ݵ�ֵ������: x:���衣���������������֡� y:���衣����������������

	//��λ�� ����Ǧ�������x����ļн�
	float phi = 0.0; // this will be default case if px is 0   �⽫��Ĭ������£���� px ��0
	if (fabs(px) > 0.0001) {  //fabs��ָ������ȡ����ֵ
		phi = atan2(py, px);  //double atan2(double y,double x) ���ص���ԭ������(x,y)�ķ�λ�ǣ����� x ��ļнǡ�����ֵ�ĵ�λΪ���ȣ�ȡֵ��ΧΪ��-��, ��]�����Ϊ����ʾ�� X ����ʱ����ת�ĽǶȣ����Ϊ����ʾ�� X ��˳ʱ����ת�ĽǶȡ���Ҫ�öȱ�ʾ������ֵ���뽫����ٳ��� 180/�С�����Ҫע����ǣ�����atan2(y,x)�в�����˳���ǵ��õģ�atan2(y,x)�����ֵ�൱�ڵ�(x,y)�ĽǶ�ֵ

	}
	//�����ٶ� ����仯�ʦ����ٶ�v����������L�����ϵ�ͶӰ
	float rho_dot = 0; // ��� �� = 0�������Ĭ����� 
	if (fabs(rho) > 0.0001) {  //fabs��ָ������ȡ����ֵ
		rho_dot = (px*vx + py * vy) / rho;  //dot ���(�ڻ�)����ӦԪ��������,�����һ������(��һ����)
	}
	z_ << rho, phi, rho_dot; 
}