#include "EKF.h"

using namespace std;

//构造函数  初始化
EKF::EKF() {
	isInitialized = false;//检查跟踪工具箱是否初始化(第一次测量)
	previousTimestamp = 0;//上一个时间戳

	tools = new TOOLS();

	// initializing matrices
	R_radar_ = MatrixXd(4, 4);
	H_jacobian = MatrixXd(4, 4);

	//measurement covariance matrix - radar  量测协方差矩阵-雷达
	R_radar_ << 0.09, 0, 0, 0,
		0, 0.0009, 0, 0,
		0, 0, 0.09, 0,
		0, 0, 0, 0.0009;

	// Radar - jacobian matrix  雷达的雅可比矩阵
	H_jacobian << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;
	
	//可以选择合适的参数初始化P矩阵。如果滤波器知道准确的初始位置，则我们可以给出一个各项均为0的初始协方差矩阵
	//一般情况第一次预测是基于已有的测量结果，往往带有噪声。这种情况下可以尝试用Radar的测量噪声方差来初始化P矩阵
	//初始化卡尔曼滤波变量
	ekf.P_ = MatrixXd(4, 4);//估计误差协方差矩阵，会逐渐收敛 
	ekf.P_ << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1000, 0,
		0, 0, 0, 1000;

	//（px;py;vx;vy） = F * （px;py;vx;vy）//状态转移矩阵
	ekf.F_ = MatrixXd(4, 4);
	ekf.F_ << 1, 0, 1, 0,
		0, 1, 0, 1,
		0, 0, 1, 0,
		0, 0, 0, 1;

	//初始化过程噪声协方差矩阵
	ekf.Q_ = MatrixXd(4, 4);
	ekf.Q_ << 0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 0;

	//初始化 ekf 状态
	ekf.x_ = VectorXd(4); //4 x 1 向量
	ekf.x_ << 1, 
		1, 
		1, 
		1;

	//固定测量噪声  
	//（加速度ax， ay是一个均值为0，标准偏差为sigma_ax 和 sigma_ay的随机矢量）在本项目中我们给定sigma_ax=sigma_ay =3; 
	noise_ax = 9; //noise_ax = sigma_ax * sigma_ax
	noise_ay = 9; //noise_ay = sigma_ay * sigma_ay

	flag = false;
}

//析构函数
EKF::~EKF() {
	delete(tools);
}


void EKF::Process(std::vector<RadarInfo_t> &measurementPackList) {
	//Call the EKF-based fusion
	size_t N = measurementPackList.size();
	//cout << "N = " << N << endl;//这一帧的目标个数

	//获取数据信息
	for (size_t k = 0; k < N; ++k) {
		/*cout << "radar_frame_infos[idx].car_id : " << measurementPackList[k].car_id << "   "
			<< "radarPoints:" << "(" << measurementPackList[k].position[0] << "," << measurementPackList[k].position[1] << ")" << "   "
			<< "radarVels:" << "(" << measurementPackList[k].velocity[0] << "," << measurementPackList[k].velocity[1] << ")" << endl;*/
		//cout << "radarPreFrameData.size():" << radarPreFrameData.size() << endl;

		//先根据上一帧的所有车辆id和这一帧的输入的车辆id进行比对判断该目标是否是第一次出现，以决定是否进行初始化
		for (size_t j = 0; j < radarPreFrameData.size(); j++) {
			if (measurementPackList[k].car_id == radarPreFrameData[j].car_id) {
				isInitialized = true;
				break;
			}
			else {
				isInitialized = false;
			}
		}

		ProcessMeasurement(measurementPackList[k]);

		estimations.push_back(ekf.x_); //最优估计值  VectorXd x_;/状态向量矩阵/作为全局变量 最后的变化在KF/EKF的第四个式子
		/*cout << "estimations.size():" << estimations.size() << endl;
		for (size_t idx = 0; idx < estimations.size(); idx++) {
			cout << "estimations:" << "(" << estimations[idx][0] << "," << estimations[idx][1] << ")" << endl;
		}*/
		//ekf.Predict();

		tools->CalculateRMSE(estimations, estimations);
		estimations.clear();
	}
	flag = true;//一帧数据结束
	SavePreFrame();
}

void EKF::ProcessMeasurement(RadarInfo_t &measurePackList) {
	Eigen::VectorXd rawMeasurements = Eigen::VectorXd(4);
	rawMeasurements << measurePackList.position[0] / 100000,
		measurePackList.position[1] / 100000,
		measurePackList.velocity[0] / 100000,
		measurePackList.velocity[1] / 100000; //以m为单位

	//Initialization
	if (!isInitialized) {
		// first measurement
		ekf.x_ = VectorXd(4); //初始化该目标第一次出现的初始状态
		ekf.x_ << measurePackList.position[0] / 100000,
			measurePackList.position[1] / 100000,
			measurePackList.velocity[0] / 100000,
			measurePackList.velocity[1] / 100000; //以m为单位
		//cout << "该目标为新目标，ekf.x_:" << ekf.x_ << endl;

		isInitialized = true;  //初始化完毕

		frameAllTar.push_back(measurePackList);

		return;//目标第一次出现，初始化之后返回
	}

	//Prediction
	/**
	* 根据新的运行时间更新状态转移矩阵F
	* 时间以秒计算
	* 更新过程噪声协方差矩阵
	* 使用 noise _ ax = 9和 noise _ ay = 9表示Q矩阵
	**/

	//计算当前测量和以前测量之间经过的时间
	float dt = (measurePackList.timeStamp - radarPreFrameData[0].timeStamp) / 1000.0;  //  in seconds

	float dt_2 = dt * dt;   //平方
	float dt_3 = dt_2 * dt; //立方
	float dt_4 = dt_3 * dt; //四次方

	// 修改F矩阵，使时间得到积分 //EKF的状态转移矩阵
	/*更改为：
	ekf.F_ <<  1, 0, dt, 0,
			   0, 1, 0, dt,
			   0, 0, 1, 0,
			   0, 0, 0, 1;*/
	ekf.F_(0, 2) = dt; //1行3列  dt--时间差
	ekf.F_(1, 3) = dt; //2行4列  dt--时间差

	//设置过程协方差矩阵Q，它是与过程噪声相关的协方差矩阵
	ekf.Q_ = MatrixXd(4, 4);
	ekf.Q_ << dt_4 / 4 * noise_ax, 0, dt_3 / 2 * noise_ax, 0,
		0, dt_4 / 4 * noise_ay, 0, dt_3 / 2 * noise_ay,
		dt_3 / 2 * noise_ax, 0, dt_2 * noise_ax, 0,
		0, dt_3 / 2 * noise_ay, 0, dt_2 * noise_ay;

	if (dt > 0.001)
		ekf.Predict();

	//Update
	/**更新状态和协方差矩阵**/
	//H_jacobian = tools->CalculateJacobian(ekf.x_);
	//cout << "after ekf.x_ Jacobian = " << H_jacobian << endl;

	ekf.H_ = H_jacobian;
	ekf.R_ = R_radar_;
	//ekf.UpdateEKF(rawMeasurements);
	ekf.Update(rawMeasurements);
	
	frameAllTar.push_back(measurePackList);
	
	return;
}

void  EKF::SavePreFrame() {	
	if (flag) {
		radarPreFrameData.swap(frameAllTar);
		frameAllTar.clear();
		flag = false;
		//cout << "一帧数据已保存" << endl;
	}	
	//cout << "radarPreFrameData.size():" << radarPreFrameData.size() << endl;
}