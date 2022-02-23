#include "paint_EasyX.h"

GRAPH::GRAPH() {
	carNum = 0;

	x1 = 30;
	y1 = 30;
	road_w = 35;//3.5m
	appear_length = 1300;//130m
}

GRAPH::~GRAPH() {}

//��֤����ʹ��
void GRAPH::radarPaint(vector<RadarInfo_t> &radarFrameInfos) {
	carNum = radarFrameInfos.size();

	Road();
	ParaConfig();
	outtextxy(100, 200, to_string(carNum).c_str());//���ַ��������������
		
	loadimage(&img, "images\\white_car.png");

	//int a = 0;
	for (size_t idx = 0; idx < carNum; idx++) {		
		putimage((int)radarFrameInfos[idx].position[0] / 10000, (int)radarFrameInfos[idx].position[1] / 10000 + 100, &img);		
		outtextxy((int)radarFrameInfos[idx].position[0] / 10000, (int)radarFrameInfos[idx].position[1] / 10000 + 100 - 16, to_string(radarFrameInfos[idx].car_id).c_str());
		outtextxy((int)radarFrameInfos[idx].position[0] / 10000 + 40, (int)radarFrameInfos[idx].position[1] / 10000 + 100 - 16, to_string(radarFrameInfos[idx].road_id).c_str());
		//outtextxy(100 + a, 230, to_string(radarFrameInfos[idx].car_id).c_str());
		//a += 50;
		//cleardevice();//�����Ļ����
	}
	Sleep(50); //�ȴ�50ms
	for (size_t idx = 0; idx < carNum; idx++) {
		setfillcolor(BLACK);;//���ɫҪ�Ǻ�ɫ
		solidrectangle((int)radarFrameInfos[idx].position[0] / 10000, (int)radarFrameInfos[idx].position[1] / 10000 + 100, (int)radarFrameInfos[idx].position[0] / 10000 + img.getwidth(), (int)radarFrameInfos[idx].position[1] / 10000 + 100 + img.getheight());
		solidrectangle((int)radarFrameInfos[idx].position[0] / 10000, (int)radarFrameInfos[idx].position[1] / 10000 + 100 - 16, (int)radarFrameInfos[idx].position[0] / 10000 + img.getwidth() + 20, (int)radarFrameInfos[idx].position[1] / 10000 + 100 + img.getheight());

	}
	
	// ��������˳�
	//_getch();
	//closegraph(); //�ر�ͼ�ν���

	return;
}

//��ʾ�Ĳ�������
void GRAPH::ParaConfig() {
	settextstyle(16, 0, _T("����"));
	settextcolor(RED);
	outtextxy(30, 200, "����������");
	//outtextxy(30, 230, "�� �� id��");
	outtextxy(x1, y1 + 5, "����1");
	outtextxy(x1, y1 + road_w +5, "����2");
	outtextxy(x1, y1 + 2 * road_w + 5, "����3");
	outtextxy(x1, y1 + 3 * road_w + 5, "����4");

	setfillcolor(WHITE);
	solidcircle(x1, y1 + 2 * road_w, 10);//�״�λ��

	return;
}

//������
void GRAPH::Road() {
	line(x1, y1, x1 + appear_length, y1);
	line(x1, y1 + road_w, x1 + appear_length, y1 + road_w);
	line(x1, y1 + 2 * road_w, x1 + appear_length, y1 + 2 * road_w);
	line(x1, y1 + 3 * road_w, x1 + appear_length, y1 + 3 * road_w);
	line(x1, y1 + 4 * road_w, x1 + appear_length, y1 + 4 * road_w);
	return;
}