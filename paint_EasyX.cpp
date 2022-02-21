#include "paint_EasyX.h"

//验证程序使用
void radarPaint(vector<Eigen::Vector2d> &radarPoints) {
	//initgraph(1800, 900); //创建图形界面，参数:图形宽度，图形高度
	//画圆，参数：圆心点x坐标，圆心点y坐标,半径mm
	//circle(200, 200, 100);

	//画直线，代表从点(400，400)到点(200,200)的直线
	//line(400, 400, 200, 200);

	/*画矩形，参数依次为矩形左部x坐标,上部y坐标,右部x坐标,下部y坐标
	也可以理解为以点(100,200)为左上角点，以点（300，400)为右下角点画矩形。*/
	//rectangle(100, 200, 300, 400);

	//画椭圆，参数为其外接矩形的四个坐标(和rectangle参数一样)
	//ellipse(100, 200, 300, 400);

	for (size_t idx = 0; idx < radarPoints.size(); idx++) {
		putpixel((int)radarPoints[idx][0] / 10000, (int)radarPoints[idx][1] / 10000, 255);
		Sleep(50); //等待50ms
	}

	//closegraph(); //关闭图形界面

	return;
}