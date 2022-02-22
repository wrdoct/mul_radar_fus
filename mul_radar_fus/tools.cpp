#include "tools.h"

TOOLS::TOOLS() {}

TOOLS::~TOOLS() {}

/*--------------------重叠区域判断------------------------*/

//定义雷达的检测区域长度为L1
#define radarDetectLength 20000000 //m * 10^5
//上、下游相邻两台雷达的重叠检测区域长度为L2
#define overLappingLength 2000000
//雷达盲区距离为△L
#define blindAreaLength 2000000

//相邻上游雷达检测到目标车辆的X轴坐标为x1，相邻下游雷达检测到目标车辆的X轴坐标为x2；
//当△L≤x1≤L2 + △L，L1 - L2≤x2≤L1 + △L时，则判定该目标车辆为重叠检测区域内目标
void TOOLS::JudgmentOverlap(std::vector<RadarInfo_t> &radar1FrameData, std::vector<RadarInfo_t> &radar2FrameData,
	std::vector<RadarInfo_t> &radar1OverlapOutput, std::vector<RadarInfo_t> &radar2OverlapOutput,
	std::vector<RadarInfo_t> &radar1OverlapExcept, std::vector<RadarInfo_t> &radar2OverlapExcept) {
	//获取位置
	vector<Eigen::Vector2d> radar1Points, radar2Points;
	ExtractPos(radar1FrameData, radar1Points);
	ExtractPos(radar2FrameData, radar2Points);

	//假设以雷达2为公共坐标系
	for (size_t idx = 0; idx < min(radar1Points.size(), radar2Points.size()); idx++) {
		if (radar1Points[idx][0] >= blindAreaLength && radar1Points[idx][0] <= overLappingLength + blindAreaLength) {// && 
			//radar2Points[idx][0] >= radarDetectLength - overLappingLength && radar2Points[idx][0] <= radarDetectLength + blindAreaLength) {			
			//cout << "该目标车辆为重叠检测区域内目标" << endl;
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


/*--------------------空间坐标转换------------------------*/

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

//将数据中的Px,Py进行坐标转换
void TOOLS::SpaceTrans(Eigen::Vector2d &radar_pos, double &angle, double &a, double &b, Eigen::Vector2d &radartoshare_pos) {
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

/*--------------------EKF------------------------*/

//RMSE 均方根误差  //输入为 估算状态和真实状态
VectorXd  TOOLS::CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, const std::vector<Eigen::VectorXd> &groundTruth) {
	size_t num_estimations = estimations.size();
	size_t num_groundTruths = groundTruth.size();

	VectorXd rmse(4);
	rmse << 0, 0, 0, 0;

	// sanity check of input validity  //检查输入有效性
	// the estimation vector size should not be zero  //估计向量大小不应为零
	// the estimation vector size should equal ground truth vector size  //估计向量大小应等于基真值向量大小
	if (estimations.size() != groundTruth.size() || estimations.size() < 1) {
		std::cout << "Cannot compute RMSE metric. Invalid input size." << std::endl;
		return rmse;
	}

	//输入为 估算状态和真实状态，两者之间的差值叫做残差，对这些残差先平方然后求均值就可以得到均方根误差
	// accumulate residuals   积累残差
	for (size_t i = 0; i < estimations.size(); ++i) {
		VectorXd residual = estimations[i] - groundTruth[i]; //两者之间的差值叫做残差

		//coefficient-wise multiplication  系数乘法
		residual = residual.array() * residual.array(); //对这些残差先平方  //residual.array()是4行1列的数组
		rmse += residual;
	}

	// compute mean  计算平均值
	rmse /= estimations.size(); //rmse = rmse / estimations.size();

	// compute squared root  开根号
	rmse = rmse.array().sqrt();

	return rmse;

}


//计算雅可比矩阵
MatrixXd TOOLS::CalculateJacobian(const VectorXd& xState) {
	MatrixXd Hj(3, 4); //3 x 4 矩阵
	//recover state parameters  恢复状态参数
	float px = xState(0);
	float py = xState(1);
	float vx = xState(2);
	float vy = xState(3);

	// preparation of Jacobian terms  雅可比条件的准备
	//预先计算一组在雅可比里重复出现的项，以避免重复计算
	float c1 = px * px + py * py;
	float c2 = sqrt(c1);
	float c3 = (c1 * c2);

	if (fabs(c1) < 0.0001) {
		//cout << "ERROR - Division by Zero" << endl;
		cout << "CalculateJacobian () - Error - Division by Zero" << endl;
		Hj.fill(0.0); //对一个容器的值进行填充时，我们就可以使用fill()函数
		return Hj;
	}

	// compute jacobian matrix  计算雅可比矩阵
	Hj << (px / c2), (py / c2), 0, 0,
		-(py / c1), (px / c1), 0, 0,
		py * (vx * py - vy * px) / c3, px * (px * vy - py * vx) / c3, px / c2, py / c2;

	return Hj;
}

//helper 函数来标准化角:
double AngleNormalization(double angle) {
	// Constrain to less than pi  
	while (angle > M_PI) angle -= 2.0 * M_PI;

	// Constrain to greater than -pi
	while (angle < -M_PI) angle += 2.0 * M_PI;

	return angle;
}

//4维向量变为3维向量  /(px,py,vx,vy) --> (rho,phi,rho_dot)
void FourVecToThreeVec(const VectorXd &z, Vector3d &z_) {	
	float px = z(0);
	float py = z(1);
	float vx = z(2);
	float vy = z(3);

	//目标距离 距离ρ是原点到目标的径向距离
	float rho = sqrt(pow(px, 2) + pow(py, 2));  //pow(x,y)返回 x 的 y 次幂的值。参数: x:必需。底数。必须是数字。 y:必需。幂数。必须是数字

	//方位角 方向角φ射线与x方向的夹角
	float phi = 0.0; // this will be default case if px is 0   这将是默认情况下，如果 px 是0
	if (fabs(px) > 0.0001) {  //fabs是指浮点数取绝对值
		phi = atan2(py, px);  //double atan2(double y,double x) 返回的是原点至点(x,y)的方位角，即与 x 轴的夹角。返回值的单位为弧度，取值范围为（-π, π]。结果为正表示从 X 轴逆时针旋转的角度，结果为负表示从 X 轴顺时针旋转的角度。若要用度表示反正切值，请将结果再乘以 180/π。另外要注意的是，函数atan2(y,x)中参数的顺序是倒置的，atan2(y,x)计算的值相当于点(x,y)的角度值

	}
	//径向速度 距离变化率ρ是速度v在沿着射线L方向上的投影
	float rho_dot = 0; // 如果 ρ = 0，这就是默认情况 
	if (fabs(rho) > 0.0001) {  //fabs是指浮点数取绝对值
		rho_dot = (px*vx + py * vy) / rho;  //dot 点积(内积)：对应元素相乘相加,结果是一个标量(即一个数)
	}
	z_ << rho, phi, rho_dot; 
}