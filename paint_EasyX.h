#pragma once
#include "coordinate_transformation.h"

#include <graphics.h>		// 引用图形库头文件
#include <conio.h>

using namespace std;
using namespace Eigen;     // 改成这样亦可 using Eigen::MatrixXd; 

void radarPaint(vector<Eigen::Vector2d> &radarPoints);