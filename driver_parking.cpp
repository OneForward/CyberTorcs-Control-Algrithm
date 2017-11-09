/***************************************************************************

file : user3.cpp
author : Xuangui Huang
email : stslxg@gmail.com
description : user module for CyberParking

***************************************************************************/

/*
WARNING !

DO NOT MODIFY CODES BELOW!
*/

#ifdef _WIN32
#include <windows.h>
#endif
#include <cmath>
#include <math.h>
#include "driver_parking.h"

static void userDriverGetParam(float lotX, float lotY, float lotAngle, bool bFrontIn, float carX, float carY, float caryaw, float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm);
static void userDriverSetParam(bool* bFinished, float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear);
static int InitFuncPt(int index, void *pt);

// Module Entry Point
extern "C" int driver_parking(tModInfo *modInfo)
{
	memset(modInfo, 0, 10 * sizeof(tModInfo));
	modInfo[0].name = "driver_parking";	// name of the module (short).
	modInfo[0].desc = "user module for CyberParking";	// Description of the module (can be long).
	modInfo[0].fctInit = InitFuncPt;	 // Init function.
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
	printf("OK!\n");
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

static FILE* f = fopen("D:\\park_data.txt", "w");

static float _midline[200][2];
static float _yaw, _yawrate, _speed, _acc, _width, _rpm, _lotX, _lotY, _lotAngle, _carX, _carY, _caryaw;
static int _gearbox;
static bool _bFrontIn;
static float haltX, haltY, haltDist, parkdist, distance, angle;
float
X1 = 168.39, Y1 = 138.56,
X2 = 149.52, Y2 = 138.56,
X3 = 31.65, Y3 = 183.46,
X4 = 29.44, Y4 = 346.91,
X5 = 44.69, Y5 = 397.09;

void push_back(float[], float);
void prepareData();

static void userDriverGetParam(float lotX, float lotY, float lotAngle,
	bool bFrontIn, float carX, float carY, float caryaw, float midline[200][2],
	float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm) {
	/* write your own code here */
	float a, b;
	_lotX = lotX;
	_lotY = lotY;
	_lotAngle = lotAngle;
	_bFrontIn = bFrontIn;
	_carX = carX;
	_carY = carY;
	_caryaw = caryaw;
	for (int i = 0; i< 200; ++i) _midline[i][0] = midline[i][0], _midline[i][1] = midline[i][1];
	_yaw = yaw;
	_yawrate = yawrate;
	_speed = speed;
	_acc = acc;
	_width = width;
	_rpm = rpm;
	_gearbox = gearbox;
}

enum STATE { CRUISE, LEFT_SHIFT, RIGHT_TURN, FIRST_HALT, BACK_CAR, SECOND_HALT, STEER_OUT };
char* string[8] = { "巡线", "接近泊车点，移至道路左侧", "漂移右转", "第一次停下",
"倒车", "第二次停下", "漂移出车位" };

static float parkDist[5], avgPark, parkAngle[5], avgAngle;
static int state = 0;
static bool startBackCar = false;
static bool haltFlag = false, startTurnRight = false, startFirstHalt = false, startLeftShift = false;


static void userDriverSetParam(bool* bFinished, float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear) {
	/* write your own code here */
	// 采集、处理泊车会用到的数据
	prepareData();

	// 以下泊车代码倒着写，顺序为：
	// 泊车位终点停下（第二次停下）-慢速倒车-第一次停下-漂移右转-接近泊车点移至道路左侧-巡线
	if (!*bFinished) {
		if (haltFlag) {
			*cmdSteer = -33.6 * angle / 3.14 - 2.16 * parkdist;
			*cmdBrake = 1.0;
			*cmdGear = -1;
			*cmdAcc = 0.0;
			if (fabs(_speed) < 0.2)
				*bFinished = true;
			state = SECOND_HALT;
		}
		else if (startBackCar) {

			float k1 = 20.18 / 3.14, k2 = 26.88, k3 = 4, k4 = 0.2;
			if (fabs(_lotX - X1) < 1 && fabs(_lotY - Y1) < 1) k1 = 20.19 / 3.14;//#1 
			if (fabs(_lotX - X2) < 1 && fabs(_lotY - Y2) < 1) k1 = 20.19 / 3.14;//#2 
			if (fabs(_lotX - X3) < 1 && fabs(_lotY - Y3) < 1) k1 = 20.186 / 3.14, k4 = 0.4; //#3
			if (fabs(_lotX - X4) < 1 && fabs(_lotY - Y4) < 1) k1 = 20.19 / 3.14;//#4 
			if (fabs(_lotX - X5) < 1 && fabs(_lotY - Y5) < 1) k2 = 26.92, k3 = 4.6;//#5

			*cmdSteer = -k1 * angle - k2*avgAngle / 3.14 - 1.404*(parkdist)-1.872*avgPark;
			if (fabs(_speed) > k3 * distance + 5) {//维持倒车速度在 k3 * distance + 5
				*cmdBrake = k4;
				*cmdGear = -1;
				*cmdAcc = 0;
			}
			else {
				*cmdBrake = 0.0;
				*cmdGear = -1;
				*cmdAcc = 0.6;
				if (fabs(_lotX - X3) < 1 && fabs(_lotY - Y3) < 1) *cmdAcc = 0.55;//#3
				if (fabs(_lotX - X5) < 1 && fabs(_lotY - Y5) < 1) *cmdAcc = 0.553; //#5
			}

			state = BACK_CAR;
		}
		else if (startTurnRight) {
			//漂移右转至垂直于车位
			if (!startBackCar && !startFirstHalt) {// 漂移右转
				*cmdSteer = -0.5*fabs(atan2(haltX - _carX, haltY - _carY));
				if (_speed<haltDist) *cmdAcc = 0.2, *cmdBrake = 0;
				else *cmdAcc = 0, *cmdBrake = 0.2;

				state = RIGHT_TURN;
			}
			if (!startBackCar && startFirstHalt) {// 刹车减速第一次停下
				float k1 = 1.0, k2 = 0.1, k3 = 0.04;
				if (fabs(_lotX - X2) < 1 && fabs(_lotY - Y2) < 1) k1 = 0.95, k2 = 0.2; //#2 
				if (fabs(_lotX - X4) < 1 && fabs(_lotY - Y4) < 1) k2 = 0.2;  //#4
				if (fabs(_lotX - X5) < 1 && fabs(_lotY - Y5) < 1) k1 = 1.05, k2 = 0.2; //#5

				*cmdSteer = (k1*_caryaw - (_lotAngle + 0.61)) / 3.14;
				*cmdBrake = k2*_speed + k3*haltDist + 0.2;
				*cmdAcc = 0;

				if (_speed < 1) startBackCar = true;
				state = FIRST_HALT;
			}
		}
		else if (startLeftShift) {
			// 接近泊车点，移至道路左侧
			float k1 = 1, k2 = 0.0, k3 = 4;
			if (fabs(_lotX - X2) < 1 && fabs(_lotY - Y2) < 1) k1 = 1.1, k2 = 0.28;
			if (fabs(_lotX - X3) < 1 && fabs(_lotY - Y3) < 1) k2 = 0.002;
			if (fabs(_lotX - X4) < 1 && fabs(_lotY - Y4) < 1) k1 = 1.1, k2 = 0.28;
			if (fabs(_lotX - X5) < 1 && fabs(_lotY - Y5) < 1) k1 = 0.95, k3 = 4.1;

			*cmdSteer = (k1 * _yaw - k3 * atan2(_midline[20][0] - _width * k2, _midline[20][1])) / 3.14;
			*cmdGear = 1;
			*cmdAcc = 0.2;
			*cmdBrake = 0;
			state = LEFT_SHIFT;
		}
		else {	 //其它路段按巡线方式行驶
			*cmdAcc = 1;
			*cmdBrake = 0;
			*cmdSteer = (_yaw - 8 * atan2(_midline[30][0], _midline[30][1])) / 3.14;
			*cmdGear = 1;
			state = CRUISE;
		}
	}
	// 以下为停稳车之后出去
	if (*bFinished) { //漂移出去
		float k1 = 1.6;
		if (fabs(_lotX - X3) < 1 && fabs(_lotY - Y3) < 1)  k1 = 2; //#3
		if (fabs(_lotX - X4) < 1 && fabs(_lotY - Y4) < 1)  k1 = 2; //#4

		*cmdSteer = (distance > 10) ? 0 : (_yaw - 8 * atan2(_midline[30][0] + k1 * _width, _midline[30][1])) / 3.14;
		*cmdAcc = 1;
		*cmdBrake = 0;
		*cmdGear = 1;

		state = STEER_OUT;
	}

	// 以下写数据
	if (distance < 40) {

		fprintf(f, "Steer:%.2f\tstate:%s\tspeed:%.2f\tlotAngle:%.2f\tcaryaw:%.2f\tishalt:%d\n",
			*cmdSteer, string[state], _speed, _lotAngle, _caryaw, *bFinished);
		fprintf(f, "油门%.2f 刹车%.2f\n车位坐标(X,Y):(%.2f,%.2f)\t车辆坐标(X,Y): (%.2f,%.2f)\n",
			*cmdAcc, *cmdBrake, _lotX, _lotY, _carX, _carY);
		fprintf(f, "停车坐标(X,Y):(%.2f,%.2f)\t停车距离:%.2f\t距离:%.2f\t横向偏移距离:%.2f\n\n",
			haltX, haltY, haltDist, distance, parkdist);
	}
}

void push_back(float arr[], float value) {
	const int arrSize = 4;
	for (int i = 0; i < arrSize; i++)
		arr[i] = arr[i + 1];
	arr[arrSize] = value;
}
float getMean(float arr[]) {
	float sum = 0;
	const int arrSize = 4;
	for (int i = arrSize; i >= 0; i = i - 1)
		sum = sum + arr[i];
	return sum / (1 + arrSize);
}
void prepareData() {
	distance = sqrt((_carX - _lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY));
	// haltX, haltY 为第一个停车点坐标
	haltX = _lotX + 10.8*cos(_lotAngle + 0.15);
	haltY = _lotY + 10.8*sin(_lotAngle + 0.15);
	//haltDist计算第一个停车点与泊车中心点距离, 目标10.8m
	haltDist = sqrt((_carX - haltX) * (_carX - haltX) + (_carY - haltY) * (_carY - haltY));

	//parkdist，计算车辆中心到泊车绝对朝向所在直线的距离，目标0.0
	parkdist = (tan(_lotAngle)*(_carX - _lotX) - (_carY - _lotY)) / (sqrt(tan(_lotAngle)*tan(_lotAngle) + 1));
	if (cos(_lotAngle) >= 0) parkdist = -parkdist;

	//angle计算车与车位的角度差，目标为0.0
	angle = _lotAngle - _caryaw;
	if (angle > PI) angle -= 2 * PI;
	if (angle < -PI) angle += 2 * PI;

	// 准备一些别的数据
	push_back(parkDist, parkdist);
	push_back(parkAngle, angle);
	avgPark = getMean(parkDist);
	avgAngle = getMean(parkAngle);

	// 子动作判定
	if (distance < sqrt(0.008)) haltFlag = true;
	if (distance < sqrt(4000.0)) startLeftShift = true;
	if (haltDist < 15) startTurnRight = true;
	if (haltDist < 5)  startFirstHalt = true;
}

