#include "KF.h"

using namespace std;

KalmanFilter::KalmanFilter() {}   //构造函数
KalmanFilter::~KalmanFilter() {}  //析构函数

//初始化卡尔曼滤波器
void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in, MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
	x_ = x_in;  //初始状态向量
	P_ = P_in;  //状态协方差矩阵
	F_ = F_in;  //状态转移矩阵
	H_ = H_in;  //量测矩阵
	R_ = R_in;  //量测噪声协方差矩阵
	Q_ = Q_in;  //过程噪声协方差矩阵
}

//使用过程模型预测状态和状态协方差
void KalmanFilter::Predict() {
	x_ = F_ * x_;   //KF的第一个式子//预测状态值
	//cout << "再预测一帧x_：" << x_ << endl;
	MatrixXd Ft = F_.transpose(); //转置
	P_ = F_ * P_ * Ft + Q_;  //KF的第二个式子//预测值和真实值之间的预测误差协方差矩阵
}

//一般卡尔曼滤波器更新操作   更新系统
void KalmanFilter::UpdateRoutine(const VectorXd &y) {
	MatrixXd Ht = H_.transpose();
	//cout << "Ht:" << Ht << endl;
	MatrixXd S = H_ * P_ * Ht + R_; //新息协方差 S = H_ * P_ * Ht + R_
	//cout << "S:" << S << endl;
	MatrixXd Si = S.inverse(); //逆
	//cout << "Si:" << Si << endl;

	MatrixXd K = P_ * Ht * Si; //KF的第三个式子//卡尔曼增益
	//cout << "K:" << K << endl;

	x_ = x_ + K * y;  //KF的第四个式子//估计值   //（测量残差/新息 y = z - H * x ;z为第k帧量测）
	//cout << "x_" << x_ << endl;
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size); //x_size * x_size 的单位矩阵
	P_ = (I - K * H_) * P_;  //KF的第五个式子//估计值和真实值之间的误差协方差矩阵
	//cout << "P_:" << P_ << endl;
	std::cout << "UpdateRoutine:" << __LINE__ << " " << __FILE__ << std::endl; // __ LINE __ 表示插入当前源代码行号的整数; __FILE__用以指示本行语句所在源文件的文件名

	return;
}

//使用标准卡尔曼滤波方程更新状态
void KalmanFilter::Update(const VectorXd &z) {
	VectorXd z_pred = H_ * x_;  //在k+1处的预测值
	//cout << "z_pred:" << z_pred << endl;
	//cout << "z:" << z << endl;
	VectorXd y = z - z_pred; //测量残差
	//cout << "y:" << y << endl;
	UpdateRoutine(y);

	return;
}

//使用扩展卡尔曼滤波方程更新状态
void KalmanFilter::UpdateEKF(const VectorXd &z) {
	//h(x)函数线性化是扩展卡尔曼滤波器(EKF)的关键
	VectorXd hx = VectorXd(4); //它指定如何将预测的位置和速度映射到距离，角度和距离变化率的极坐标  /非线性测量函数hx  //4 x 1 向量  //；hx是一个4大小的向量，同样分配空间未初始化元素

	float px = x_(0);
	float py = x_(1);
	float vx = x_(2);
	float vy = x_(3);

	//目标距离 距离ρ是原点到目标的径向距离
	float rho = sqrt(pow(px, 2) + pow(py, 2));  //pow(x,y)返回 x 的 y 次幂的值。参数: x:必需。底数。必须是数字。 y:必需。幂数。必须是数字

	//方位角 方向角φ射线与x方向的夹角
	float phi = 0.0; // 默认情况下 px 是0，
	if (fabs(px) > 0.0001) {  //fabs是指浮点数取绝对值
		phi = atan2(py, px);  //double atan2(double y,double x) 返回的是原点至点(x,y)的方位角，即与 x 轴的夹角。返回值的单位为弧度，取值范围为（-π, π]。结果为正表示从 X 轴逆时针旋转的角度，结果为负表示从 X 轴顺时针旋转的角度。若要用度表示反正切值，请将结果再乘以 180/π。另外要注意的是，函数atan2(y,x)中参数的顺序是倒置的，atan2(y,x)计算的值相当于点(x,y)的角度值

	}
	//径向速度 距离变化率ρ是速度v在沿着射线L方向上的投影
	float rho_dot = 0; // 如果 ρ = 0，这就是默认情况 
	if (fabs(rho) > 0.0001) {  //fabs是指浮点数取绝对值
		rho_dot = (px*vx + py * vy) / rho;  //dot 点积(内积)：对应元素相乘相加,结果是一个标量(即一个数)
	}
	hx << rho, phi, rho_dot; //存入hx 向量  非线性测量函数hx
	VectorXd y = z - hx;
	//Vector3d z_;
	//FourVecToThreeVec(z, z_);
	//VectorXd y = z_ - hx;

	// normalize result to -pi and pi
	y(1) = AngleNormalization(y(1)); //y(1)是方位角

	UpdateRoutine(y);

	return;
}

