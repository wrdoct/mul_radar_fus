//卡尔曼滤波的接口  //封装
#include "ekf_interface.h"

using namespace std;

//构造函数
EKF_API::EKF_API() {
	fusionEKF = new FusionEKF();
}
//析构函数
EKF_API::~EKF_API() {
	delete(fusionEKF);
}

void EKF_API::process(std::vector<RadarInfo_t> measurementPackList) {
	//Call the EKF-based fusion
	size_t N = measurementPackList.size();
	cout << "N = " << N << endl;//目标个数

	//先根据上一帧的所有车辆id和这一帧的输入的车辆id进行比对判断该目标是否是第一次出现，已决定是否进行初始化


	//获取数据信息
	for (size_t k = 0; k < N; ++k) {

		cout << "radar_frame_infos[idx].car_id : " << measurementPackList[k].car_id << "   "
			<< "radarPoints:" << "(" << measurementPackList[k].position[0] << "," << measurementPackList[k].position[1] << ")" << "   "
			<< "radarVels:" << "(" << measurementPackList[k].velocity[0] << "," << measurementPackList[k].velocity[1] << ")" << endl;
		
		// start filtering from the second frame (the speed is unknown in the first frame)
		//从第二帧开始过滤(在第一帧中速度是未知的)
		fusionEKF->ProcessMeasurement(measurementPackList[k].position[0], //px
			measurementPackList[k].position[1], //py
			measurementPackList[k].velocity[0],              //vx
			measurementPackList[k].velocity[1],         //vy
			measurementPackList[k].timeStamp);         //时间戳  //ms//采样帧率
				
		estimations.push_back(fusionEKF->ekf.x_); //最优估计值  VectorXd x_;/状态向量矩阵/作为全局变量 最后的变化在KF/EKF的第四个式子
		/*cout << "estimations.size():" << estimations.size() << endl;
		for (size_t idx = 0; idx < estimations.size(); idx++) {
			cout << "estimations:" << "(" << estimations[idx][0] << "," << estimations[idx][1] << ")" << endl;
		}*/
		
	}
	
	//EKF 结束之后输出：
	cout << "EKF Done!" << endl;

	return;
}