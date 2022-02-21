#include "radar_json.h"
#include <cstdlib> //stringתdouble

//�ַ���ʽ��ʱ��ת����ʱ���  //time_t���ͣ��Ȿ������һ��������
timeb StringToDatetime(string str) {
	timeb time_b;                                    
	tm tm_;							// ����tm�ṹ�塣
	int year, month, day, hour, minute, second, millisecond;// ����ʱ��ĸ���int��ʱ������
	year = atoi((str.substr(0, 4)).c_str());
	month = atoi((str.substr(5, 2)).c_str());
	day = atoi((str.substr(8, 2)).c_str());
	hour = atoi((str.substr(11, 2)).c_str());
	minute = atoi((str.substr(14, 2)).c_str());
	second = atoi((str.substr(17, 2)).c_str());
	millisecond = atoi((str.substr(20, 3)).c_str());

	tm_.tm_year = year - 1900;                 // �꣬����tm�ṹ��洢���Ǵ�1900�꿪ʼ��ʱ�䣬����tm_yearΪint��ʱ������ȥ1900��      
	tm_.tm_mon = month - 1;                    // �£�����tm�ṹ����·ݴ洢��ΧΪ0-11������tm_monΪint��ʱ������ȥ1��
	tm_.tm_mday = day;                         // �ա�
	tm_.tm_hour = hour;                        // ʱ��
	tm_.tm_min = minute;                       // �֡�
	tm_.tm_sec = second;                       // �롣
	tm_.tm_isdst = 0;                          // ������ʱ��
	time_b.time = mktime(&tm_);                  // ��tm�ṹ��ת����time_t��ʽ  //mktime��ʱ��ṹ����ת���ɾ��������� 
	time_b.millitm = millisecond;
	return time_b;                                 // ����ֵ��
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
	stringstream ss(Input);//�����ַ��������󣬳�ʼ��ΪInput
	ss >> ans;//����������ݵ�ans
	return ans;
}

void LoadRadarInfo(const std::string &path,   //����
	std::vector<RadarInfo_t> &radarObjects) {

	radarObjects.clear();

	//���ļ��ж�ȡ
	ifstream in(path, ios::binary);

	Json::Reader reader;
	Json::Value root;

	if (!in.is_open())
	{
		cout << "Error opening file\n";
		return;
	}
	reader.parse(in, root); //reader��Json�ַ���������root��root������Json��������Ԫ��
	//cout << "root.size()=" << root.size();//root.size()=35234
	
	for (int i = 0; i < root.size(); i++) {
		RadarInfo_t object;
		Json::Value value = root[i];
		//��ȡ���ڵ���Ϣ 
		if (value["RecordTime"].isString()) {
			string RecordTime = value["RecordTime"].asString();  // ���ʽڵ� 
			//cout << "RecordTime: " << RecordTime << endl;
			timeb radarTime = StringToDatetime(RecordTime);
			//cout << "radarTime.time = " << radarTime.time << endl; //��1970�꿪ʼ���������� 
			//cout << "radarTime.millitm = " << radarTime.millitm << endl;//���ݵĺ�����			
			object.timeStamp = radarTime.time * 1000 + radarTime.millitm; //ʱ���//֡��
		}
		//QDateTime radarTime = QDateTime::fromString(RecordTime, " yyyy-MM-ddThh:mm:ss:zzz");//��ȡʱ���
		if (value["RadarID"].isString()) {
			string RadarID = value["RadarID"].asString();    // ���ʽڵ�
			//cout << "RadarID: " << RadarID << endl;
			object.radar_id = StringToDouble(RadarID);
		}
		if (value["Px"].isString()) {
			string Px = value["Px"].asString();    // ���ʽڵ�
			//cout << "Px: " << Px << endl;
			object.position[0] = StringToDouble(Px);
		}
		if (value["Py"].isString()) {
			string Py = value["Py"].asString();    // ���ʽڵ�
			//cout << "Py: " << Py << endl;
			object.position[1] = StringToDouble(Py);
		}
		if (value["RoadId"].isString()) {
			string RoadId = value["RoadId"].asString();    // ���ʽڵ�
			//cout << "RoadId: " << RoadId << endl;
			object.road_id = StringToInt(RoadId);
		}
		if (value["Vx"].isString()) {
			string Vx = value["Vx"].asString();    // ���ʽڵ�
			//cout << "Vx: " << Vx << endl;
			object.velocity[0] = StringToDouble(Vx);
		}
		if (value["Vy"].isString()) {
			string Vy = value["Vy"].asString();    // ���ʽڵ�
			//cout << "Vy: " << Vy << endl;
			object.velocity[1] = StringToDouble(Vy);
		}
		if (value["CarID"].isString()) {
			string CarID = value["CarID"].asString();    // ���ʽڵ�
			//cout << "CarID: " << CarID << endl; cout << endl;
			object.car_id = StringToInt(CarID);
		}
		//��ȡ�ӽڵ���Ϣ  
		//string friend_name = root["friends"]["friend_name"].asString();

		radarObjects.push_back(object);//id ���� �ٶ� ��Ϣѹ�� radarObjects
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

//���ݰ���ʱ��֡�ʷ���
void RadarFrame(std::vector<RadarInfo_t> &radarObjects, std::vector<RadarInfo_t> &radarFrameData) {
	//cout << radarObjects.size() << endl;//35234
	int frame_num = 1; int target_num = 1;
	std::vector<RadarInfo_t> radarFrameDataBefore;
	for (size_t idx = 1; idx < radarObjects.size(); idx++) {
		//cout << radarObjects[idx].timeStamp << endl;
		if (radarObjects[idx].timeStamp == radarObjects[idx - 1].timeStamp) {
			//cout << "ʱ������" << endl;
			target_num++;	
			//radarFrameDataBefore[frame_num].life = target_num;//��һ֡���Ŀ�����
			radarFrameDataBefore.push_back(radarObjects[idx - 1]);								
		}
		else {
			//cout << "ʱ�������ȣ�֡����һ" << endl;
			//cout << "��" << frame_num << "֡�ڵ�Ŀ�����Ϊ��" << target_num << endl;
			radarFrameDataBefore.push_back(radarObjects[idx - 1]);
			radarFrameData.swap(radarFrameDataBefore);
			radarFrameDataBefore.clear();
			frame_num++;					
			target_num = 1; //�ٴγ�ʼ�� ��λĿ�����
			/*cout << "radarFrameData.size() = " << radarFrameData.size() << endl;
			for (size_t idx = 0; idx < radarFrameData.size(); idx++) {
				cout << "radarFrameData[idx].car_id:" << radarFrameData[idx].car_id << endl;
			}*/
		}
	}
	//cout << "��֡��frame_num = " << frame_num - 1 << endl;//6628֡

	return;
}

//ʹ�õ���Ŀ��Ĳ���
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