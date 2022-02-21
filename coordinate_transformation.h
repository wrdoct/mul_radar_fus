#pragma once
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "radar_json.h"

using namespace std;
using namespace Eigen;

#define PI 3.1415926
static double angle = PI / 4.0;
static double a = 1.0, b = 1.0;

/*
class SpaceTrans {
public:
	//Æ½ÒÆ

	//Ðý×ª

private:

};
*/
void ExtractVel(const vector<RadarInfo_t> &radarInfos, vector<Eigen::Vector2d> &radarVels);

void ExtractPos(const vector<RadarInfo_t> &radarInfos, vector<Eigen::Vector2d> &radarPoints);

void SpaceTrans(Eigen::Vector2d &radar_pos, double &angle, double &a, double &b, Eigen::Vector2d &radartoshare_pos);