#include "radar_json.h"
#include "tools.h"
#include "paint_EasyX.h"
#include "NNDA.h"
#include "EKF.h"

#include <thread>
#include <mutex>

//雷达文件路径
std::string radar1_path = "C:\\Users\\LLW\\Desktop\\mul_radar_fus\\radar.json";
std::string radar2_path = "C:\\Users\\LLW\\Desktop\\mul_radar_fus\\radar.json";
//相机文件路径
std::string camera_path = "xxxxx";

mutex mtx1, mtx2;//互斥量//给线程上锁
condition_variable cv;// 定义条件变量(用来做线程间的同步通信)

//雷达有重叠区域时使用NNDA
std::shared_ptr<TOOLS> tool = std::make_shared<TOOLS>(); 
std::shared_ptr<NNDA> nnda = std::make_shared<NNDA>();

//雷达没有重叠区域时使用EKF
//EKF *ekf_api = new EKF();//实例化EKF指针 //用后必须删除
//智能指针:方便资源的管理，自动释放没有指针引用的资源
//std::shared_ptr<EKF> ekf_api(new EKF);
std::shared_ptr<EKF> ekf_api = std::make_shared<EKF>(); //make_shared要优于使用new，make_shared可以一次将需要内存分配好

//定义共享容器，共享变量等
vector<RadarInfo_t> radar_infos, radar_frame_infos;
void Radar1Thread() {

	// 获取mtx互斥锁资源
	unique_lock<std::mutex> lock1(mtx1);

	//radar
	//vector<RadarInfo_t> radar_infos, radar_frame_infos;//定义RadarInfo_t的一个类型
	LoadRadarInfo(radar1_path, radar_infos);
	/*for (auto iter = radar_infos.begin(); iter != radar_infos.end();) {
		std::cout << "iter->car_id = " << iter->car_id << std::endl;
		std::cout << "iter->road_id = " << iter->road_id << std::endl;
		std::cout << "iter->position[0] = " << iter->position[0] << "  "        //x
			<< "iter->position[1] = " << iter->position[1] << std::endl; //y
		std::cout << "iter->velocity[0] = " << iter->velocity[0] << "  "        //x
			<< "iter->velocity[1] = " << iter->velocity[1] << std::endl  //y
			<< std::endl;
		++iter;
	}*/

	//数据按照时间帧率分类
	//RadarFrame(radar_infos, radar_frame_infos);
	int frame_num = 1; int target_num = 1;
	std::vector<RadarInfo_t> radarFrameDataBefore;
	for (size_t idx = 1; idx < radar_infos.size(); idx++) {
		//cout << radarObjects[idx].timeStamp << endl;
		if (radar_infos[idx].timeStamp == radar_infos[idx - 1].timeStamp) {
			//cout << "时间戳相等" << endl;
			target_num++;
			//radarFrameDataBefore[frame_num].life = target_num;//这一帧里的目标个数
			radarFrameDataBefore.push_back(radar_infos[idx - 1]);
		}
		else {
			//cout << "时间戳不相等，帧数加一" << endl;
			cout << "第" << frame_num << "帧内的目标个数为：" << target_num << endl;	
			frame_num++;
			target_num = 1; //再次初始化 复位目标个数

			radarFrameDataBefore.push_back(radar_infos[idx - 1]);
			radar_frame_infos.swap(radarFrameDataBefore);//radar_frame_infos即为帧数据
			radarFrameDataBefore.clear();
			/*cout << "radar_frame_infos.size() = " << radar_frame_infos.size() << endl;
			for (size_t idx = 0; idx < radar_frame_infos.size(); idx++) {
				cout << "radar_frame_infos[idx].car_id:" << radar_frame_infos[idx].car_id << endl;
			}*/

			////对帧数据做处理////
			while (!radar_frame_infos.empty()) {
				// 判断容器不为空，进入等待条件变量的状态，释放mtx锁，
				// 让RadarFusionThread线程抢到锁能够去处理数据
				cv.wait(lock1);
			}

			//通知等待在cv条件变量上的处理线程，可以开始处理数据了，然后释放锁mtx
			cv.notify_all();
			//生成一帧数据，睡眠100ms
			//std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
	}
	cout << "总帧数frame_num = " << frame_num - 1 << endl;//

	/*
	//使用单个目标的测试
	vector<Eigen::Vector2d> radarSinglePoints;
	SingleTarget(radar_infos, radarSinglePoints);
	cout << "id为0的radarSinglePoints.size()=" << radarSinglePoints.size() << endl;
	for (size_t idx = 0; idx < radarSinglePoints.size(); idx++) {
		cout << "radarSinglePoints:" << "(" << radarSinglePoints[idx][0] / 100000 << "," << radarSinglePoints[idx][1] / 100000 << ")" << endl;
	}
	*/	

	return;

}

//定义共享容器，共享变量等
vector<RadarInfo_t> radar2_infos, radar2_frame_infos;
void Radar2Thread() {
	//mtx.lock();

	// 获取mtx互斥锁资源
	unique_lock<std::mutex> lock2(mtx2);
	//如果容器不为空，代表还有数据未处理
	/*while (!radar2_frame_infos.empty()) {
		// 判断容器不为空，进入等待条件变量的状态，释放mtx锁，
		// 让RadarFusionThread线程抢到锁能够去处理数据
		cv.wait(lock);
	}*/

	//radar
	LoadRadarInfo(radar2_path, radar2_infos);

	//数据按照时间帧率分类
//RadarFrame(radar_infos, radar_frame_infos);
	int frame_num = 1; int target_num = 1;
	std::vector<RadarInfo_t> radar2FrameDataBefore;
	for (size_t idx = 1; idx < radar2_infos.size(); idx++) {
		//cout << radarObjects[idx].timeStamp << endl;
		if (radar2_infos[idx].timeStamp == radar2_infos[idx - 1].timeStamp) {
			//cout << "时间戳相等" << endl;
			target_num++;
			//radarFrameDataBefore[frame_num].life = target_num;//这一帧里的目标个数
			radar2FrameDataBefore.push_back(radar2_infos[idx - 1]);
		}
		else {
			//cout << "时间戳不相等，帧数加一" << endl;
			//cout << "第" << frame_num << "帧内的目标个数为：" << target_num << endl;			
			frame_num++;
			target_num = 1; //再次初始化 复位目标个数

			radar2FrameDataBefore.push_back(radar2_infos[idx - 1]);
			radar2_frame_infos.swap(radar2FrameDataBefore);//radar_frame_infos即为帧数据
			radar2FrameDataBefore.clear();
			/*cout << "radar2_frame_infos.size() = " << radar2_frame_infos.size() << endl;
			for (size_t idx = 0; idx < radar2_frame_infos.size(); idx++) {
				cout << "radar_frame_infos[idx].car_id:" << radar2_frame_infos[idx].car_id << endl;
			}*/

			////对帧数据做处理////
			//如果容器不为空，代表还有数据未处理
			while (!radar2_frame_infos.empty()) {
				// 判断容器不为空，进入等待条件变量的状态，释放mtx锁，
				// 让RadarFusionThread线程抢到锁能够去处理数据
				cv.wait(lock2);
			}

			//通知等待在cv条件变量上的处理线程，可以开始处理数据了，然后释放锁mtx
			cv.notify_all();
			//生成一帧数据，睡眠100ms
			//std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
	}
	//cout << "总帧数frame_num = " << frame_num - 1 << endl;//6628帧

	//mtx.unlock();
	
}

void RadarFusionThread() {
	//mtx.lock();

	//initgraph(1800, 900); //创建图形界面，参数:图形宽度，图形高度 //画图

	//每处理一帧，就通知雷达线程产生一帧
	while(1){
		// 获取mtx互斥锁资源
		unique_lock<std::mutex> lock1(mtx1), lock2(mtx2);
		//如果容器不为空，代表还有数据未处理
		while (radar_frame_infos.empty()) {
			// 判断容器为空，进入等待条件变量的状态，释放mtx锁，
			// 让Radar1Thread线程抢到锁能够去产生数据
			cv.wait(lock1);
		}
		while (radar2_frame_infos.empty()) {
			// 判断容器为空，进入等待条件变量的状态，释放mtx锁，
			// 让Radar2Thread线程抢到锁能够去产生数据
			cv.wait(lock2);
		}

		/*
		cout << "radar_frame_infos.size() = " << radar_frame_infos.size() << endl; 
		for (size_t idx = 0; idx < radar_frame_infos.size(); idx++) {
			cout << "radar_frame_infos[idx].car_id:" << radar_frame_infos[idx].car_id << endl;
		}
		*/	

		/*
		cout << "radar2_frame_infos.size() = " << radar2_frame_infos.size() << endl;
		for (size_t idx = 0; idx < radar2_frame_infos.size(); idx++) {
			cout << "radar2_frame_infos[idx].car_id:" << radar2_frame_infos[idx].car_id << endl;
		}
		*/
				
		vector<RadarInfo_t> radar_fusion_result;	//输出结果  //三部分
		/*----------------------------第一种情况：雷达之间存在重叠区域-------------------------*/
		//雷达之间重叠区域的判断
		vector<RadarInfo_t> radar1OverlapOutput, radar2OverlapOutput;
		vector<RadarInfo_t> radar1OverlapExcept, radar2OverlapExcept;
		tool->JudgmentOverlap(radar_frame_infos, radar2_frame_infos, radar1OverlapOutput, radar2OverlapOutput, radar1OverlapExcept, radar2OverlapExcept);
		/*
		cout << "该帧里满足重叠区域判断条件，radar1OverlapOutput.size() = " << radar1OverlapOutput.size() << endl;
		cout << "该帧里雷达1不满足重叠区域判断条件，radar1OverlapExcept.size() = " << radar1OverlapExcept.size() << endl;
		cout << "该帧里满足重叠区域判断条件，radar2OverlapOutput.size() = " << radar2OverlapOutput.size() << endl;
		cout << "该帧里雷达2不满足重叠区域判断条件，radar2OverlapExcept.size() = " << radar2OverlapExcept.size() << endl;
		*/
						
		if (!radar1OverlapOutput.empty() || !radar2OverlapOutput.empty()) {		
			//假设以雷达2为公共坐标系/对雷达1的数据进行坐标转换   
			
			////重叠区域之外的数据经过坐标转换之后直接进行显示（假设以雷达2为公共坐标系）//并且改变car_id
			for (size_t idx = 0; idx < radar1OverlapExcept.size(); idx++) {

				//cout << "before radar1OverlapExcept[idx].position:" << "(" << radar1OverlapExcept[idx].position[0] << "," << radar1OverlapExcept[idx].position[1] << ")" << endl;
				//空间坐标转换
				radar1OverlapExcept[idx].position[0] = radar1OverlapExcept[idx].position[0] * cos(angle) + radar1OverlapExcept[idx].position[1] * sin(angle) + a;
				radar1OverlapExcept[idx].position[1] = radar1OverlapExcept[idx].position[0] * (-sin(angle)) + radar1OverlapExcept[idx].position[1] * cos(angle) + b;
				radar_fusion_result.push_back(radar1OverlapExcept[idx]);//第一部分
			}
						
			//判断距离差<距离阈值，速度差<速度阈值，车道号相等 --> 进行凸组合加权平均的航迹融合
			//NNDA
			vector<RadarInfo_t> mul_radar_fusion;
			nnda->Fusion(radar1OverlapOutput, radar2OverlapOutput, mul_radar_fusion);

			//判断出同一辆车之后把车辆特征信息也叠加上去

			for (size_t idx = 0; idx < mul_radar_fusion.size(); idx++) {
				radar_fusion_result.push_back(mul_radar_fusion[idx]);//第二部分
			}
			for (size_t idx = 0; idx < radar2OverlapExcept.size(); idx++) {
				radar_fusion_result.push_back(radar2OverlapExcept[idx]);//第三部分
			}

			cout << "radar_fusion_result" << endl;

		}
		
		/*----------------------------第二种情况：雷达之间不存在重叠区域-------------------------*/
		else {
			//EKF				
			ekf_api->Process(radar_frame_infos);// EKF 过程
		}
		
		/*
		//画图
		//system("cls");  //清空当前控制台 执行画图
		vector<Eigen::Vector2d> testPoint;
		ExtractPos(radar_frame_infos, testPoint);
		radarPaint(testPoint);
		*/

		radar_frame_infos.clear(); radar2_frame_infos.clear();

		//通知等待在cv条件变量上的产生线程，可以开始产生数据了，然后释放锁mtx
		cv.notify_all();
		//处理一帧数据，睡眠100ms
		//std::this_thread::sleep_for(std::chrono::milliseconds(100));

	}
	//delete(ekf_api);//手动释放EKF的内存
	//mtx.unlock();

}

int main() {
	
	thread th1(Radar1Thread);//第一个参数为函数名，第二个参数为该函数的第一个参数，如果该函数接收多个参数就依次写在后面。此时线程开始执行。
	//cout << "主线程中显示子线程id为" << th1.get_id() << endl;
	thread th2(Radar2Thread);	
	thread th3(RadarFusionThread);

	th1.join();//此时主线程被阻塞直至子线程执行结束。
	th2.join();
	th3.join();
		
	system("pause");//请按任意键继续...
	return 0;
}
