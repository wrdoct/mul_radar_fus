#include "paint_EasyX.h"

//��֤����ʹ��
void radarPaint(vector<Eigen::Vector2d> &radarPoints) {
	//initgraph(1800, 900); //����ͼ�ν��棬����:ͼ�ο�ȣ�ͼ�θ߶�
	//��Բ��������Բ�ĵ�x���꣬Բ�ĵ�y����,�뾶mm
	//circle(200, 200, 100);

	//��ֱ�ߣ�����ӵ�(400��400)����(200,200)��ֱ��
	//line(400, 400, 200, 200);

	/*�����Σ���������Ϊ������x����,�ϲ�y����,�Ҳ�x����,�²�y����
	Ҳ�������Ϊ�Ե�(100,200)Ϊ���Ͻǵ㣬�Ե㣨300��400)Ϊ���½ǵ㻭���Ρ�*/
	//rectangle(100, 200, 300, 400);

	//����Բ������Ϊ����Ӿ��ε��ĸ�����(��rectangle����һ��)
	//ellipse(100, 200, 300, 400);

	for (size_t idx = 0; idx < radarPoints.size(); idx++) {
		putpixel((int)radarPoints[idx][0] / 10000, (int)radarPoints[idx][1] / 10000, 255);
		Sleep(50); //�ȴ�50ms
	}

	//closegraph(); //�ر�ͼ�ν���

	return;
}