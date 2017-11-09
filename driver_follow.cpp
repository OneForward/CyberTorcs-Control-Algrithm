/***************************************************************************

file                 : user2.cpp
author            : Xuangui Huang
email              : stslxg@gmail.com
description    :  user module for CyberFollower

***************************************************************************/

/*
WARNING !

DO NOT MODIFY CODES BELOW!
*/

#ifdef _WIN32
#include <windows.h>
#include <cmath>
#endif

#include "driver_follow.h"
#include <vector>


static void userDriverGetParam(float LeaderXY[2], float midline[200][2], float yaw,
	float yawrate, float speed, float acc, float width, int gearbox, float rpm);
static void userDriverSetParam(float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear);
static int InitFuncPt(int index, void *pt);

// Module Entry Point
extern "C" int driver_follow(tModInfo *modInfo)
{
	memset(modInfo, 0, 10 * sizeof(tModInfo));
	modInfo[0].name = "driver_follow";	// name of the module (short).
	modInfo[0].desc = "user module for CyberFollower";	// Description of the module (can be long).
	modInfo[0].fctInit = InitFuncPt;			// Init function.
	modInfo[0].gfId = 0;
	modInfo[0].index = 0;
	return 0;
}

// Module interface initialization.
static int InitFuncPt(int, void *pt)
{
	tUserItf *itf = (tUserItf *)pt;
	itf->userDriverGetParam = userDriverGetParam;
	itf->userDriverSetParam = userDriverSetParam;
	return 0;
}

/*
WARNING!

DO NOT MODIFY CODES ABOVE!
*/

/*
define your variables here.
following are just examples
*/



/*******************************My Codes*****************************************/

typedef struct Circle
{
	double r;
	int sign;
};
typedef struct XY { // 前车的坐标
	float x;
	float y;
};
std::vector<XY> savedXY; // 将前车的坐标保存在一个容量为6的队列向量里，
						 // 主函数每调用一次就压入队列中，并弹出最初的值
XY XYcord;

static float _midline[200][2];
static float _yaw, _yawrate, _speed, _acc, _width, _rpm;
static int _gearbox;
static float _Leader_X, _Leader_Y;

static int counter = 0;
static Circle CArray[95];

static FILE* f = fopen("C:/torcs_data_velocity.txt", "w");


static float dist[5], v_err[5], diff_v_err[5];//距离、相对速度、相对加速度
float sum_dist_err = 0; // 距离误差积分
float her_yaw;	//前车速度方向在我坐标系下的角度，朝左为负，朝右为正
float her_real_yaw; // 前车偏离道路中心线的角度

float border_X; // 车身边界点
float safe_dist;
float V_Steer;
float a, b;



//******************* Declaration of My Functions ******************************//
double constrain(double lowerBoundary, double upperBoundary, double input);		//
Circle getC(int p1, int p2, int p3);											//				
float length(float x1, float x2, float y1, float y2);							//		
void curveDetect();																//
void push_back(float arr[], float value);										//
float getMean(float arr[]);														//
void gear(int* cmdGear, float* cmdAcc, float* cmdBrake);						//
float getDiff(float arr[]);														//
void writedata(float* cmdacc, float* cmdbrake, float* cmdsteer);				//
																				//
float getAcc(float k1, float k2, float k3, float k4, float k5, float k6);	//
float curveCrash();																//
float getSteer(float k1, float k2);												//
float getSafeDist();															//
float getHerDirection();														//




/********************* Implementation of My Functions ***************************/
/**************************** GET PARAM *************************************/
static void userDriverGetParam(float LeaderXY[2], float midline[200][2], float yaw,
	float yawrate, float speed, float acc, float width, int gearbox, float rpm) {

	_Leader_X = LeaderXY[0];
	_Leader_Y = LeaderXY[1];
	for (int i = 0; i< 200; ++i) _midline[i][0] = midline[i][0], _midline[i][1] = midline[i][1];
	_yaw = yaw;
	_yawrate = yawrate;
	_speed = speed;
	_acc = acc;
	_width = width;
	_rpm = rpm;
	_gearbox = gearbox;
	//printf("speed %.3f Leader XY(%.3f, %.3f)\n", _speed, _Leader_X, _Leader_Y);
}



/******************************写数据的函数************************************/
void writedata(float* cmdAcc, float* cmdBrake, float* cmdSteer) {
	counter++;
	fprintf(f, "ct:%d, ", counter);
	fprintf(f, "her_X:%.3f,Y:%.3f,_yaw:%.3f,V_err:%.3f,Acc:%.3f,my_X:%.3f,border_X:%.3f ",
		_Leader_X - _midline[0][0], _Leader_Y, her_real_yaw, v_err[4], diff_v_err[4], -_midline[0][0], border_X);
	fprintf(f,
		"V %.3f, _yaw:%.3f, Acc:%.3f, Brake:%.3f, Steer:%.3f\n", _speed, _yaw, *cmdAcc, *cmdBrake, *cmdSteer);
}

/***************Core Parameters: 油门刹车的PID控制参数************************/
float kOfV = 2.9;
float kOfAcc = 0.5;
float kOfDist1 = 0.6;
float kOfDist2 = 0.8;
float kOfSumDist = 0.105;


/**************************** MAIN FUNC *************************************/
static void userDriverSetParam(float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear) {
	/* write your own code here */
	gear(cmdGear, cmdAcc, cmdBrake);


	border_X = -_midline[0][0] + 0.5 * (4.90 * fabs(sin(_yaw)) + 1.92 * fabs(cos(_yaw)));
	safe_dist = getSafeDist();

	//前车速度方向在我坐标系下的角度，朝左为负，朝右为正
	her_yaw = getHerDirection();
	// 前车偏离道路中心线的角度
	her_real_yaw = her_yaw + _yaw;

	// 压入距离信息
	push_back(dist, length(0, _Leader_X, 0, _Leader_Y));

	// 压入相对速度信息
	push_back(v_err, getDiff(dist));

	// 压入相对加速度信息
	push_back(diff_v_err, getDiff(v_err));

	// 计算偏差距离的积分
	if (counter > 100) {
		sum_dist_err += (dist[4] - 16) / counter;
	}

	curveDetect();
		
	*cmdSteer = getSteer(250 / dist[4], 2);
	a = 0.2 + 0.01 * curveCrash();
	b = getAcc(kOfV, kOfAcc, kOfDist1, kOfDist2, 0, kOfSumDist)
		* 4.4 / (kOfV + kOfAcc + kOfDist1 + kOfDist2 + kOfSumDist) / 3.5;
	V_Steer = _speed * fabs(*cmdSteer);


	if (CArray[20].r < 150 && curveCrash() > 0) {//弯道中减速避免撞墙
		*cmdAcc = 0.2 - 0.01 * curveCrash();
		*cmdBrake = (fabs(a) > fabs(b)) ? fabs(a) : fabs(b);
		*cmdSteer = (_yaw - 8 * atan2(float(_midline[35][0] - 0.25 * _width), float(_midline[35][1]))) / 3.14;
		fprintf(f, "0 lalaalalalal\n");
	}
	else {
		float offset = *cmdAcc * 0.4;
		*cmdAcc = b; //油门 刹车PID控制
		*cmdBrake = -b;
		if (V_Steer > 50) //防止打滑
			fprintf(f, "5 lalaalalalal\n"),
			*cmdAcc = getAcc(2, 0, 0.6, 0.8, 0.04, 0);
		if (*cmdBrake >= 0.8) *cmdSteer = 0;

		// 转向角很大时油门不能乱踩，会飘出
		if (fabs(*cmdSteer) > 0.70) {
			*cmdAcc = 0.005 + offset;
			*cmdBrake = 0;

		}
		else if (fabs(*cmdSteer) > 0.60) {
			*cmdAcc = 0.11 + offset;
			*cmdBrake = 0;
			fprintf(f, "6 lalaalalalal\n");
		}
	}

	/*********************下面是一些想法，以全局安全为首要目标*********************************/
	if (CArray[1].r < 150) { // 弯道时避免撞墙，将预瞄点设置在道路四分之一线处
							 
		if (_Leader_X - _midline[0][0] > 0.25 * _width)fprintf(f, "1 lalaalalalal\n"),
			*cmdSteer = (_yaw - 8 * atan2(float(_midline[35][0] + 0.0 * _width), float(_midline[35][1]))) / 3.14;
		else if (_Leader_X - _midline[0][0] < -0.25 * _width)fprintf(f, "2 lalaalalalal\n"),
			*cmdSteer = (_yaw - 8 * atan2(float(_midline[35][0] - 0.3 * _width),float( _midline[35][1]))) / 3.14;
	}


	// 针对17组的调整-突然横向
	if (fabs(her_real_yaw) > 1.57 && v_err[4] < 0 && _Leader_Y < 14) {
		*cmdBrake = 1.0; *cmdAcc = 0;
	}

	// 考虑太近时的刹车
	if (_Leader_Y < 12 && v_err[4] < -2.0) {
		*cmdBrake = 1.0; *cmdAcc = 0;
	}

	// 考虑被拉开时的加速
	if (_Leader_Y > 20 && v_err[4] > 1.0 && CArray[15].r > 150) {
		*cmdAcc = 1.0; *cmdBrake = 0;
	}

	// 针对第7组右上角弯道被甩开


	// 写一写边界函数
	




	*cmdAcc = constrain(0, 1.0, *cmdAcc);
	*cmdBrake = constrain(0, 1.0, *cmdBrake);
	*cmdSteer = constrain(-1.0, 1.0, *cmdSteer);

	writedata(cmdAcc, cmdBrake, cmdSteer);
}


/*****************************控制函数**********************************/
float getAcc(float k1, float k2, float k3, float k4, float k5, float k6) {
	return k1 * v_err[4] + k2 * diff_v_err[4] + k3 * (getMean(dist) - safe_dist)
		+ k4 * (dist[4] - safe_dist) + k5 * V_Steer + k6 * sum_dist_err;
}
float getSteer(float k1, float k2) {
	float  p1 = 0.02;
	return (_yaw - k1 * atan2(_Leader_X, _Leader_Y) -
		k2 * atan2(_midline[35][0], _midline[35][1])) / 3.14 + p1*her_real_yaw;
}
float getSafeDist() { // 安全距离随速度的函数关系
	float dist[3] = { 10.0, 12.5, 14.0 };
	float _safeDist;
	if (_speed < 100)
		_safeDist = dist[0];
	else if (_speed < 150)
		_safeDist = (dist[1] - dist[0]) * 0.02 * (_speed - 100) + dist[0]-1;
	else if (_speed < 200)
		_safeDist = (dist[2] - dist[1]) * 0.02 * (_speed - 150) + dist[1]-1;
	else
		_safeDist = 0.06 * (_speed - 200) + dist[2];

	return _safeDist * 1.2;
}
float getHerDirection() { //获取前车速度的方向

	XYcord.x = _Leader_X;
	XYcord.y = _Leader_Y;
	savedXY.push_back(XYcord);
	if (savedXY.size() >= 6) {
		savedXY.erase(savedXY.begin());
	}
	float dx = 0, dy = 0;
	for (int i = 0; i < savedXY.size() - 1; ++i) {
		dx += (savedXY[savedXY.size() - 1].x - savedXY[i].x) / (0.02*3.6*(savedXY.size() - i));
		dy += (savedXY[savedXY.size() - 1].y - savedXY[i].y) / (0.02*3.6*(savedXY.size() - i));
	}

	return atan2(dx, dy);
}


/*****************************弯道类型检测函数**********************************/
void curveDetect() {//判断弯道类型
	int i;
	i = 20;
	CArray[i] = getC(2 * i, 2 * i + 5, 2 * i + 9);
	i = 15;
	CArray[i] = getC(2 * i, 2 * i + 5, 2 * i + 9);
	i = 1;
	CArray[i] = getC(2 * i, 2 * i + 5, 2 * i + 9);
}
Circle getC(int p1, int p2, int p3)
{
	Circle tmp;
	float r, s, a, b, c, x1, x2, x3, y1, y2, y3;
	x1 = _midline[p1][0]; y1 = _midline[p1][1];
	x2 = _midline[p2][0]; y2 = _midline[p2][1];
	x3 = _midline[p3][0]; y3 = _midline[p3][1];

	s = ((x2 - x1)*(y3 - y1) - (x3 - x1)*(y2 - y1)) / 2.0;
	a = length(x1, x2, y1, y2);
	b = length(x1, x3, y1, y3);
	c = length(x3, x2, y3, y2);
	r = fabs((a*b*c) / (4.0 * fabs(s) + 0.0001));

	tmp.r = (r > 250.0) ? 250.0 : r;
	tmp.sign = (s > 0) ? 1 : -1;
	return tmp;
}
float curveCrash() {
	return _speed - sqrt(250 * CArray[20].r);
}


/****************************一些辅助函数****************************/
double constrain(double lowerBoundary, double upperBoundary, double input)
{
	float tmp;
	if (lowerBoundary > upperBoundary)
	{
		tmp = lowerBoundary;
		lowerBoundary = upperBoundary;
		upperBoundary = tmp;
	}
	if (input > upperBoundary)
		return upperBoundary;
	else if (input < lowerBoundary)
		return lowerBoundary;
	else
		return input;
}
float length(float x1, float x2, float y1, float y2)
{
	return sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1));
}
void push_back(float arr[], float value) {
	for (int i = 0; i<4; i++)
		arr[i] = arr[i + 1];
	arr[4] = value;
}
float getDiff(float arr[]) {
	float sum = 0;
	for (int i = 3; i >= 0; i = i - 1)
		sum = sum + (arr[4] - arr[i]) / (0.02*(4 - i));
	return sum / 4;
}
float getMean(float arr[]) {
	float sum = 0;
	for (int i = 4; i >= 0; i = i - 1)
		sum = sum + arr[i];
	return sum / 5;
}
void gear(int* cmdGear, float* cmdAcc, float* cmdBrake) {
	if (_speed <= 45)*cmdGear = 1;
	if (_speed>45 && _speed <= 90 && *cmdGear == 1 && _rpm>650) *cmdGear = *cmdGear + 1;
	if (_speed>90 && _speed <= 145 && *cmdGear == 2 && _rpm>650) *cmdGear = *cmdGear + 1;
	if (_speed>145 && _speed <= 185 && *cmdGear == 3 && _rpm>650) *cmdGear = *cmdGear + 1;
	if (_speed>185 && _speed <= 230 && *cmdGear == 4 && _rpm>650)*cmdGear = *cmdGear + 1;
	if (_speed>230 && *cmdGear == 5 && _rpm>600)*cmdGear = *cmdGear + 1;
	if (_speed <= 45 && *cmdGear == 2)*cmdGear = *cmdGear - 1;
	if (_speed>45 && _speed <= 90 && *cmdGear == 3 && _rpm<600) *cmdGear = *cmdGear - 1;
	if (_speed>90 && _speed <= 145 && *cmdGear == 4 && _rpm<600) *cmdGear = *cmdGear - 1;
	if (_speed>145 && _speed <= 185 && *cmdGear == 5 && _rpm<600) *cmdGear = *cmdGear - 1;
	if (_speed>185 && _speed <= 230 && *cmdGear == 6 && _rpm<600)*cmdGear = *cmdGear - 1;
}