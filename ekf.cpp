#include "ekf.h"

using namespace std;

//构造函数  初始化
FusionEKF::FusionEKF() {
	//isInitialized = false;
	isInitialized = true;
	previousTimestamp = 0;

	// initializing matrices
	R_radar_ = MatrixXd(4, 4);
	H_jacobian = MatrixXd(4, 4);

	//measurement covariance matrix - radar  量测协方差矩阵-雷达
	R_radar_ << 0.09, 0, 0, 0,
		0, 0.0009, 0, 0,
		0, 0, 0.09, 0,
		0, 0, 0, 0.0009;

	// Radar - jacobian matrix  雷达的雅可比矩阵
	H_jacobian << 0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 0;

	//可以选择合适的参数初始化P矩阵。如果滤波器知道准确的初始位置，则我们可以给出一个各项均为0的初始协方差矩阵
	//一般情况第一次预测是基于已有的测量结果，往往带有噪声。这种情况下可以尝试用Radar或者Lidar的测量噪声方差来初始化P矩阵
	// initialize the kalman filter variables   //初始化卡尔曼滤波变量
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

	// Initialize process noise covariance matrix   初始化过程噪声协方差矩阵
	ekf.Q_ = MatrixXd(4, 4);
	ekf.Q_ << 0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 0;

	// Initialize ekf state   初始化 ekf 状态
	ekf.x_ = VectorXd(4); //4 x 1 向量
	ekf.x_ << 1, 1, 1, 1;

	// set measurement noises  固定测量噪声  
	//（加速度ax， ay是一个均值为0，标准偏差为sigma_ax 和 sigma_ay的随机矢量）在本项目中我们给定sigma_ax=sigma_ay =3; 
	noise_ax = 9; //noise_ax = sigma_ax * sigma_ax
	noise_ay = 9; //noise_ay = sigma_ay * sigma_ay

}

//析构函数
FusionEKF::~FusionEKF() {}

//计算雅可比矩阵
MatrixXd FusionEKF::CalculateJacobian(const VectorXd& xState) {
	MatrixXd Hj(3, 4); //3 x 4 矩阵
	//recover state parameters  恢复状态参数
	float px = xState(0);
	float py = xState(1);
	float vx = xState(2);
	float vy = xState(3);

	// preparation of Jacobian terms  雅可比条件的准备
	//pre-compute a set of terms which recur in the Jacobian to avoid repeated calculation  预先计算一组在雅可比里重复出现的项，以避免重复计算
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

//从这里 开始 运行卡尔曼滤波器的全部流程。   timestamp时间戳
void FusionEKF::ProcessMeasurement(float px, float py, float vx, float vy, long long timestamp) {
	Eigen::VectorXd rawMeasurements = Eigen::VectorXd(4);
	rawMeasurements << px, py, vx, vy;
	cout << "rawMeasurements: " << rawMeasurements << endl;
	//Initialization
	if (!isInitialized) {
		// first measurement
		ekf.x_ = VectorXd(4);
		ekf.x_ << px, py, vx, vy;  // px, py, vx, vy

		//previousTimestamp = timestamp;

		// done initializing, no need to predict or update
		isInitialized = true;  //初始化完毕，无需预测或更新
		return;
	}

	//  Prediction

	/**
	* Update the state transition matrix F according to the new elapsed time. 根据新的运行时间更新状态转移矩阵F
	* Time is measured in seconds.   时间以秒计算
	* Update the process noise covariance matrix.  更新过程噪声协方差矩阵
	* Use noise_ax = 9 and noise_ay = 9 for your Q matrix.  使用 noise _ ax = 9和 noise _ ay = 9表示Q矩阵
	**/

	// compute the time elapsed between the current and previous measurements   //计算当前测量和以前测量之间经过的时间
	// float dt = (timestamp_ - previous_timestamp_) / 1000000.0;  //  in seconds
	float dt = timestamp / 1000.0;  //  in seconds
	// previousTimestamp = timestamp;

	float dt_2 = dt * dt;   //平方
	float dt_3 = dt_2 * dt; //立方
	float dt_4 = dt_3 * dt; //四次方

	// Modify the F matrix so that the time is integrated   修改F矩阵，使时间得到积分 /integrated整合//EKF的状态转移矩阵
	/*更改为：
	ekf.F_ <<  1, 0, dt, 0,
			   0, 1, 0, dt,
			   0, 0, 1, 0,
			   0, 0, 0, 1;*/
	ekf.F_(0, 2) = dt; //1行3列  dt--时间差
	ekf.F_(1, 3) = dt; //2行4列  dt--时间差

	//set the process covariance matrix Q   设置过程协方差矩阵Q， 它是与过程噪声相关的协方差矩阵
	ekf.Q_ = MatrixXd(4, 4);
	ekf.Q_ << dt_4 / 4 * noise_ax, 0, dt_3 / 2 * noise_ax, 0,
		0, dt_4 / 4 * noise_ay, 0, dt_3 / 2 * noise_ay,
		dt_3 / 2 * noise_ax, 0, dt_2 * noise_ax, 0,
		0, dt_3 / 2 * noise_ay, 0, dt_2 * noise_ay;

	if (dt > 0.001)
		ekf.Predict();

	//Update
	/**
	* Use the sensor type to perform the update step.  使用传感器类型执行更新步骤。
	* Update the state and covariance matrices.  更新状态和协方差矩阵
	**/
	//H_jacobian = CalculateJacobian(ekf.x_);
	ekf.H_ = H_jacobian;
	ekf.R_ = R_radar_;
	ekf.UpdateEKF(rawMeasurements);
	//ekf.Update(rawMeasurements);

	return;
}
