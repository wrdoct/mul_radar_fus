#include "radar_json.h"
#include <cstdlib> //string转double

//字符形式的时间转换成时间戳  //time_t类型，这本质上是一个长整数
timeb StringToDatetime(string str) {
	timeb time_b;                                    
	tm tm_;							// 定义tm结构体。
	int year, month, day, hour, minute, second, millisecond;// 定义时间的各个int临时变量。
	year = atoi((str.substr(0, 4)).c_str());
	month = atoi((str.substr(5, 2)).c_str());
	day = atoi((str.substr(8, 2)).c_str());
	hour = atoi((str.substr(11, 2)).c_str());
	minute = atoi((str.substr(14, 2)).c_str());
	second = atoi((str.substr(17, 2)).c_str());
	millisecond = atoi((str.substr(20, 3)).c_str());

	tm_.tm_year = year - 1900;                 // 年，由于tm结构体存储的是从1900年开始的时间，所以tm_year为int临时变量减去1900。      
	tm_.tm_mon = month - 1;                    // 月，由于tm结构体的月份存储范围为0-11，所以tm_mon为int临时变量减去1。
	tm_.tm_mday = day;                         // 日。
	tm_.tm_hour = hour;                        // 时。
	tm_.tm_min = minute;                       // 分。
	tm_.tm_sec = second;                       // 秒。
	tm_.tm_isdst = 0;                          // 非夏令时。
	time_b.time = mktime(&tm_);                  // 将tm结构体转换成time_t格式  //mktime将时间结构数据转换成经过的秒数 
	time_b.millitm = millisecond;
	return time_b;                                 // 返回值。
}

double StringToDouble(string Input)
{
	double Result;
	stringstream Oss;
	Oss << Input;
	Oss >> Result;
	return Result;
}
int StringToInt(string Input)
{
	int ans;
	stringstream ss(Input);//构造字符串流对象，初始化为Input
	ss >> ans;//输出流的内容到ans
	return ans;
}

void LoadRadarInfo(const std::string &path,   //输入
	std::vector<RadarInfo_t> &radarObjects) {

	radarObjects.clear();

	//从文件中读取
	ifstream in(path, ios::binary);

	Json::Reader reader;
	Json::Value root;

	if (!in.is_open())
	{
		cout << "Error opening file\n";
		return;
	}
	reader.parse(in, root); //reader将Json字符串解析到root，root将包含Json里所有子元素
	//cout << "root.size()=" << root.size();//root.size()=35234
	
	for (int i = 0; i < root.size(); i++) {
		RadarInfo_t object;
		Json::Value value = root[i];
		//读取根节点信息 
		if (value["RecordTime"].isString()) {
			string RecordTime = value["RecordTime"].asString();  // 访问节点 
			//cout << "RecordTime: " << RecordTime << endl;
			timeb radarTime = StringToDatetime(RecordTime);
			//cout << "radarTime.time = " << radarTime.time << endl; //从1970年开始经过的秒数 
			//cout << "radarTime.millitm = " << radarTime.millitm << endl;//数据的毫秒数			
			object.timeStamp = radarTime.time * 1000 + radarTime.millitm; //时间戳//帧率
		}
		//QDateTime radarTime = QDateTime::fromString(RecordTime, " yyyy-MM-ddThh:mm:ss:zzz");//提取时间戳
		if (value["RadarID"].isString()) {
			string RadarID = value["RadarID"].asString();    // 访问节点
			//cout << "RadarID: " << RadarID << endl;
			object.radar_id = StringToDouble(RadarID);
		}
		if (value["Px"].isString()) {
			string Px = value["Px"].asString();    // 访问节点
			//cout << "Px: " << Px << endl;
			object.position[0] = StringToDouble(Px);
		}
		if (value["Py"].isString()) {
			string Py = value["Py"].asString();    // 访问节点
			//cout << "Py: " << Py << endl;
			object.position[1] = StringToDouble(Py);
		}
		if (value["RoadId"].isString()) {
			string RoadId = value["RoadId"].asString();    // 访问节点
			//cout << "RoadId: " << RoadId << endl;
			object.road_id = StringToInt(RoadId);
		}
		if (value["Vx"].isString()) {
			string Vx = value["Vx"].asString();    // 访问节点
			//cout << "Vx: " << Vx << endl;
			object.velocity[0] = StringToDouble(Vx);
		}
		if (value["Vy"].isString()) {
			string Vy = value["Vy"].asString();    // 访问节点
			//cout << "Vy: " << Vy << endl;
			object.velocity[1] = StringToDouble(Vy);
		}
		if (value["CarID"].isString()) {
			string CarID = value["CarID"].asString();    // 访问节点
			//cout << "CarID: " << CarID << endl; cout << endl;
			object.car_id = StringToInt(CarID);
		}
		//读取子节点信息  
		//string friend_name = root["friends"]["friend_name"].asString();

		radarObjects.push_back(object);//id 距离 速度 信息压入 radarObjects
		/*for (auto iter = radarObjects.begin(); iter != radarObjects.end();){
			std::cout << "iter->car_id = " << iter->car_id << std::endl;
			std::cout << "iter->road_id = " << iter->road_id << std::endl;
			std::cout << "iter->position[0] = " << iter->position[0] <<"  "        //x
						<< "iter->position[1] = " << iter->position[1] << std::endl; //y
			std::cout << "iter->velocity[0] = " << iter->velocity[0] <<"  "        //x
						<< "iter->velocity[1] = " << iter->velocity[1] << std::endl  //y
						<< std::endl;
			++iter;
		}*/
	}

	in.close();

	return;

}

//数据按照时间帧率分类
void RadarFrame(std::vector<RadarInfo_t> &radarObjects, std::vector<RadarInfo_t> &radarFrameData) {
	//cout << radarObjects.size() << endl;//35234
	int frame_num = 1; int target_num = 1;
	std::vector<RadarInfo_t> radarFrameDataBefore;
	for (size_t idx = 1; idx < radarObjects.size(); idx++) {
		//cout << radarObjects[idx].timeStamp << endl;
		if (radarObjects[idx].timeStamp == radarObjects[idx - 1].timeStamp) {
			//cout << "时间戳相等" << endl;
			target_num++;	
			//radarFrameDataBefore[frame_num].life = target_num;//这一帧里的目标个数
			radarFrameDataBefore.push_back(radarObjects[idx - 1]);								
		}
		else {
			//cout << "时间戳不相等，帧数加一" << endl;
			//cout << "第" << frame_num << "帧内的目标个数为：" << target_num << endl;
			radarFrameDataBefore.push_back(radarObjects[idx - 1]);
			radarFrameData.swap(radarFrameDataBefore);
			radarFrameDataBefore.clear();
			frame_num++;					
			target_num = 1; //再次初始化 复位目标个数
			/*cout << "radarFrameData.size() = " << radarFrameData.size() << endl;
			for (size_t idx = 0; idx < radarFrameData.size(); idx++) {
				cout << "radarFrameData[idx].car_id:" << radarFrameData[idx].car_id << endl;
			}*/
		}
	}
	//cout << "总帧数frame_num = " << frame_num - 1 << endl;//6628帧

	return;
}

//使用单个目标的测试
void SingleTarget(std::vector<RadarInfo_t> &radarObjects, vector<Eigen::Vector2d> &radarSinglePoints) {
	for (size_t idx = 0; idx < radarObjects.size(); idx++) {
		//cout << radarObjects[idx].car_id << endl;
		if (radarObjects[idx].car_id == 0 && radarObjects[idx].timeStamp - 2048062926 < 25000) {
			Eigen::Vector2d radarSinglePos;
			radarSinglePos[0] = radarObjects[idx].position[0];
			radarSinglePos[1] = radarObjects[idx].position[1];
			radarSinglePoints.emplace_back(radarSinglePos);
		}
	}
}