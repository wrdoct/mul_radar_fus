#include "paint_EasyX.h"

GRAPH::GRAPH() {
	carNum = 0;

	x1 = 30;
	y1 = 30;
	road_w = 35;//3.5m
	appear_length = 1300;//130m
}

GRAPH::~GRAPH() {}

//验证程序使用
void GRAPH::radarPaint(vector<RadarInfo_t> &radarFrameInfos) {
	carNum = radarFrameInfos.size();

	Road();
	ParaConfig();
	outtextxy(100, 200, to_string(carNum).c_str());//将字符串输出到界面上
		
	loadimage(&img, "images\\white_car.png");

	//int a = 0;
	for (size_t idx = 0; idx < carNum; idx++) {		
		putimage((int)radarFrameInfos[idx].position[0] / 10000, (int)radarFrameInfos[idx].position[1] / 10000 + 100, &img);		
		outtextxy((int)radarFrameInfos[idx].position[0] / 10000, (int)radarFrameInfos[idx].position[1] / 10000 + 100 - 16, to_string(radarFrameInfos[idx].car_id).c_str());
		outtextxy((int)radarFrameInfos[idx].position[0] / 10000 + 40, (int)radarFrameInfos[idx].position[1] / 10000 + 100 - 16, to_string(radarFrameInfos[idx].road_id).c_str());
		//outtextxy(100 + a, 230, to_string(radarFrameInfos[idx].car_id).c_str());
		//a += 50;
		//cleardevice();//清除屏幕内容
	}
	Sleep(50); //等待50ms
	for (size_t idx = 0; idx < carNum; idx++) {
		setfillcolor(BLACK);;//填充色要是黑色
		solidrectangle((int)radarFrameInfos[idx].position[0] / 10000, (int)radarFrameInfos[idx].position[1] / 10000 + 100, (int)radarFrameInfos[idx].position[0] / 10000 + img.getwidth(), (int)radarFrameInfos[idx].position[1] / 10000 + 100 + img.getheight());
		solidrectangle((int)radarFrameInfos[idx].position[0] / 10000, (int)radarFrameInfos[idx].position[1] / 10000 + 100 - 16, (int)radarFrameInfos[idx].position[0] / 10000 + img.getwidth() + 20, (int)radarFrameInfos[idx].position[1] / 10000 + 100 + img.getheight());

	}
	
	// 按任意键退出
	//_getch();
	//closegraph(); //关闭图形界面

	return;
}

//显示的参数设置
void GRAPH::ParaConfig() {
	settextstyle(16, 0, _T("黑体"));
	settextcolor(RED);
	outtextxy(30, 200, "车辆个数：");
	//outtextxy(30, 230, "车 辆 id：");
	outtextxy(x1, y1 + 5, "车道1");
	outtextxy(x1, y1 + road_w +5, "车道2");
	outtextxy(x1, y1 + 2 * road_w + 5, "车道3");
	outtextxy(x1, y1 + 3 * road_w + 5, "车道4");

	setfillcolor(WHITE);
	solidcircle(x1, y1 + 2 * road_w, 10);//雷达位置

	return;
}

//画车道
void GRAPH::Road() {
	line(x1, y1, x1 + appear_length, y1);
	line(x1, y1 + road_w, x1 + appear_length, y1 + road_w);
	line(x1, y1 + 2 * road_w, x1 + appear_length, y1 + 2 * road_w);
	line(x1, y1 + 3 * road_w, x1 + appear_length, y1 + 3 * road_w);
	line(x1, y1 + 4 * road_w, x1 + appear_length, y1 + 4 * road_w);
	return;
}