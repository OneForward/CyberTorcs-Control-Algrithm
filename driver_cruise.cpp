/*
WARNING !

DO NOT MODIFY CODES BELOW!
*/

#ifdef _WIN32
#include <windows.h>
#endif

#include "driver_cruise.h"
#include "stdio.h"
#include <vector>

#define PI 3.141592653589793238462643383279

static void userDriverGetParam(float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm);
static void userDriverSetParam(float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear);
static int InitFuncPt(int index, void *pt);

// Module Entry Point
extern "C" int driver_cruise(tModInfo *modInfo)
{
	memset(modInfo, 0, 10 * sizeof(tModInfo));
	modInfo[0].name = "driver_cruise";	// name of the module (short).
	modInfo[0].desc = "user module for CyberCruise";	// Description of the module (can be long).
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

//**********Global variables for vehicle states*********//
static float _midline[200][2];						//
static float _yaw, _yawrate, _speed, _acc, _width, _rpm;//
static int _gearbox;									//
bool parameterSet = false;								//
void PIDParamSetter();									//


// 曲率半径档次： I(160,200),II(120,160),III(80,120),IV(50,80),V(20,50)
enum CurvaLevel { ZERO, I, II, III, IV, V };

// 弯道的形状
enum TYPE { DIRECT, LEFT, RIGHT, S1, S2 }; //S1: L->R    S2:R->L

typedef struct Circle
{
	double r;
	int sign;
}circle;

typedef struct Curve {
	TYPE _type;
	int _distance;
	CurvaLevel _cvl;
	int init_d;
};

std::vector<Curve> vC;





//********************PID parameters*************************//
double kp_s;	//kp for speed							     //
double ki_s;	//ki for speed							     //
double kd_s;	//kd for speed							     //
double kp_d;	//kp for direction						     //
double ki_d;	//ki for direction					    	 //
double kd_d;	//kd for direction						     //    
double D_err;//direction error					             //
double D_errDiff = 0;//direction difference(Differentiation) //
double D_errSum = 0;//sum of direction error(Integration)    //
double curSpeedErr;//speed error   		                     //
double speedErrSum = 0;//sum of speed error(Integration)     //
double tmpSpeedErr = 0;										 //
double speedErrDiff;										 //
//***********************************************************//

//*******************Other parameters***********************************************//															    																	    //																    //
double offset = 0;																    //
double Tmp = 0;																	    //
int targetP;															   																		    
double expectedSpeed;   														    //
Circle targetC;																		//
static int counter = 0, accCounter = 0, TargetPoint;							    //								    
static circle CArray[39];														    //
//static FILE* f = fopen("C:/Users/Huang Jing/Desktop/torcs_data_velocity.txt", "w"); //				
float horizonXRate;																	//																	
float dx = 0, dy = 0;																//
																					//
																					//
//**********************************************************************************//


//******************* Declaration of My Functions ******************************//
double constrain(double lowerBoundary, double upperBoundary, double input);		//
//void writeData(float* cmdAcc, float* cmdBrake, float* cmdSteer);				//
circle getC(int p1, int p2, int p3);											//				
float length(float x1, float x2, float y1, float y2);							//		
CurvaLevel getCurvaLevel(float r);												//
void curveDetect();																//
void unify(float[], int);
int getTargetP();																//
																				// 公路控制																		//
void steerControl(float *cmdSteer);												//
void speedControl(float *cmdAcc, float *cmdBrake, float * cmdSteer);			//
float getV(float r);															//																		
float getTargetSpeed(int TargetPoint);											//																			 
void gear(int* cmdGear, float* cmdAcc, float* cmdBrake);						//																							 
void getXShift_d(float &dx, float &dy);											//
																				//
																				//
																				// 土路控制																		//
void steerControl2(float *cmdSteer);											//
void speedControl2(float *cmdAcc, float *cmdBrake, float * cmdSteer);			//
void getXShift_d2(float &dx, float &dy);										//
float getTargetSpeed2(int TargetPoint);											//
float getV2(float r);															//				
void gear2(int* cmdGear, float* cmdAcc, float* cmdBrake);						//
//******************************************************************************//



																				//main func1
static void userDriverGetParam(float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm) {
	/* write your own code here */

	for (int i = 0; i < 200; ++i) { _midline[i][0] = midline[i][0]; _midline[i][1] = midline[i][1]; }

	_yaw = yaw;
	_yawrate = yawrate;
	_speed = speed;
	_acc = acc;
	_width = width;
	_rpm = rpm;
	_gearbox = gearbox;
}



//******************* My Functions Implementation***********************************************//
void PIDParamSetter()
{

	kp_s = 0.02;
	ki_s = 0.001;
	kd_s = 0.1;
	kp_d = 1.3;
	ki_d = 0.002;
	kd_d = 5;
	parameterSet = true;

}





//******************************公路 控制***********************************************//

//******************* （公路）为不同曲率半径的弯道分配速度***********************************************//
float getV(float r) {
	if (r > 150)
		return 185 + 130 / (1.0 + exp((200 - r) / 15));
	else
		return 90 + 95 / (1.0 + exp((100.0 - r) / 15));

	// some sample points
	//(48, 93) (75, 105) (85, 115) (100,  137.5)
}

//******************* （公路）目标速度***********************************************//
float getTargetSpeed(int TargetPoint) {

	//检测赛道种类
	curveDetect();

	targetC = getC(TargetPoint, TargetPoint + 5, TargetPoint + 10);
	float targetV = getV(targetC.r), _tgtV = 0.0;
	const int N = 95;
	float k[N], smpV[N];

	// 根据赛道各点的曲率半径，初步获得目标速度（该速度函数对低速度给予更大的权值）
	for (int i = 0; i < N; ++i) smpV[i] = getV(CArray[i].r);
	for (int i = 0; i < N; ++i) k[i] = 1.0 / (1.0 + exp((smpV[i] - targetV) / 125));
	unify(k, N);
	for (int i = 0; i < N; ++i) _tgtV += k[i] * smpV[i];

	float kOfV = 1.25; // 一个速度系数，乘上最后获取的速度，以便尽可能增大速度

					   //	根据道路的形状对速度进行进一步调整
	for (int i = 0; i < vC.size(); ++i) {
		if ((vC[i]._cvl == V) && (vC[i].init_d < _speed * 0.3) && (vC[i]._distance < 125))
		{	// 短于125m长的V级弯道，100m处检测 目标速度92
			_tgtV = 95 / kOfV;
		}
		else if ((vC[i]._cvl == IV) && (vC[i].init_d < _speed * 0.65) && (vC[i]._distance < 75))
		{	// 75m长IV级弯道，195m处检测 目标速度105 针对-> ETrack4赛道4600m处
			_tgtV = 240 / kOfV;
		}
		else if ((vC[i]._cvl == IV) && (vC[i].init_d < _speed * 0.65) && (vC[i]._distance < 120))
		{	// 120m长IV级弯道，195m出检测 目标速度115 针对Brondehach 第4个弯道
			_tgtV = 105 / kOfV;
		}
		else if (_speed > 155 && vC[i]._cvl >= II && vC[i].init_d < 80)
		{ 	// 针对在直道达到速度极高之后的弯道, 此处针对Brondehach最后一个弯道
			_tgtV = 165 / kOfV;
		}
		else if (vC[i]._cvl == V && vC[i]._type == S2 && vC[i]._distance == 22) {
			// Brondehach 的S弯
			_tgtV = 100 / kOfV;
		}
	}

	// E-Track4几个S弯的校正
	if (vC[0]._type >= S1 && vC[1]._type >= S1) {
		if (vC[0]._cvl == III && vC[1]._cvl >= II)//先急后缓
			_tgtV = 160 / kOfV;
		if (vC[0]._cvl == IV && vC[1]._cvl >= II)//先急后缓
			_tgtV = 160 / kOfV / 1.16;
		if (vC[0]._cvl <= II && vC[1]._cvl >= III)//先缓后急
			_tgtV = 125 / kOfV;
	}


	// CG 的某个弯道
	if (vC[0]._type == LEFT && vC[2]._type == LEFT && vC[1]._distance == 48) {
		_tgtV = 140 / kOfV;
	}
	if (vC[0]._type == LEFT && vC[1]._type == DIRECT && vC[2]._type == RIGHT
		&& abs(vC[2]._distance - 57)<2) {
		_tgtV = 120 / kOfV; //右上角的一个弯道
	}





	// Street 的某个赛道
	if (vC[0]._type == DIRECT && vC[1]._type == RIGHT && vC[1]._distance == 58) {
		_tgtV = 90 / kOfV;
	}



	//Brondehach的某条赛道
	if (vC[0]._cvl == IV && vC[1]._type == II && vC[1]._distance == 42) {
		_tgtV = 130 / kOfV;
	}
	if (vC[0]._type == LEFT && vC[1]._type == RIGHT && (vC[0]._distance + vC[1]._distance) == 190) {
		_tgtV = 80 / kOfV;
	}
	if (vC[0]._type == DIRECT && vC[1]._type == LEFT && vC[2]._type == DIRECT && abs(vC[1]._distance - 33)<2) {
		_tgtV = 85 / kOfV; //第二个弯道出来后不能突然加速
	}
	if (vC[0]._type == S2 && vC[2]._type == DIRECT && abs(vC[0]._distance + vC[1]._distance)<48
		&& vC[0]._cvl == V) _tgtV = 100 / kOfV;
	if (vC[0]._type == RIGHT && vC[0]._cvl == V && vC[1]._type == DIRECT) {
		_tgtV = 95 / kOfV;
	}



	// ETrack4 的一个高速后的大弯道
	if (vC[0]._type == DIRECT && vC[1]._type == RIGHT && (vC[0]._distance + vC[1]._distance) == 190) {
		_tgtV = 189 / kOfV;
	}

	// E-Road
	if (vC[0]._type == S1 && vC[1]._type == S1 && (vC[0]._cvl == II || vC[1]._cvl == V) &&
		((vC[0]._distance + vC[1]._distance) >= 188 || abs(vC[1]._distance - 41) <= 2)) {
		if (vC[1]._cvl == V) _tgtV = 150 / kOfV; //左下方一个弯道
		else _tgtV = 180 / kOfV; //中间一个弱弯
	}
	if (vC[0]._type == RIGHT && vC[1]._type == S2 &&  vC[1]._cvl == V && abs(vC[1]._distance - 77) <= 2
		&& vC[0]._cvl == V) {// 中间一个 L-D-L形状的弯道
		_tgtV = 130 / kOfV;
	}
	if (vC[0]._type == RIGHT && vC[1]._type == DIRECT &&  vC[2]._type == LEFT &&
		abs(vC[1]._distance - 87) <= 2
		&& vC[0]._cvl == V) {// 出L-D-L形状的弯道避免减速
		_tgtV = 180 / kOfV;
	}
	if (vC[0]._type == S2 && vC[1]._cvl == V && vC[0]._cvl == V) {
		_tgtV = 180 / kOfV;// 避免左下方一个弯减速用的
	}
	if (vC[0]._type == S1 && vC[0]._cvl == IV&& vC[1]._cvl == V && abs(vC[1]._distance - 55) <= 2) {
		_tgtV = 120 / kOfV; // 左侧的一个弯道
	}
	if (vC[0]._type == RIGHT && vC[0]._cvl == II && vC[1]._type == DIRECT && (vC[0]._distance + vC[1]._distance) == 190) {
		_tgtV = 300;
	}

	// 如果前面为S型道的速度奖励
	if (vC[0]._type >= S1 || vC[1]._type >= S1) _tgtV *= 1.15;
	// 如果面前有直道(>=25m)的速度奖励
	if (vC[0]._type == DIRECT && vC[0]._distance >= 25) _tgtV *= 1.2;

	return _tgtV * kOfV;
}

//******************* （公路）SPEED CONTROL***********************************************//
void speedControl(float *cmdAcc, float *cmdBrake, float * cmdSteer) {
	bool hasPassedCurve = false;
	TargetPoint = max(int(_speed*0.4), 65);
	expectedSpeed = getTargetSpeed(TargetPoint);
	//expectedSpeed = 60;
	// 速度控制 - 主要依照cmdSteer、speedErr的大小分类讨论
	curSpeedErr = expectedSpeed - _speed;
	speedErrSum = 0.1 * speedErrSum + curSpeedErr;
	if (curSpeedErr > 0) {
		offset = *cmdAcc * 0.4;
		if (fabs(*cmdSteer) < 0.60) {
			*cmdAcc = constrain(0.0, 1.0, kp_s * curSpeedErr + ki_s * speedErrSum + offset);
			*cmdBrake = 0;
		}
		else if (fabs(*cmdSteer) > 0.70) {
			*cmdAcc = 0.005 + offset;
			*cmdBrake = 0;
		}
		else {
			*cmdAcc = 0.11 + offset;
			*cmdBrake = 0;
		}


	}
	else if (curSpeedErr < 0) {//待修改的刹车函数
		offset = 0.004;
		*cmdBrake = constrain(0.0, 1.0, -kp_s *curSpeedErr + offset);
		*cmdAcc = 0;

		if (_speed > 250) *cmdBrake *= 0.5;
		else if (_speed > 200) *cmdBrake *= 0.7;
		else if (_speed > 150) *cmdBrake *= 0.8;
	}
}


/****************************（公路）STEER CONTROL****************************/
void steerControl(float *cmdSteer)
{
	dx = 0; dy = 0;
	getXShift_d(dx, dy);
	targetP = getTargetP();

	*cmdSteer = (_yaw - 8 * atan2(_midline[targetP][0] + dx, _midline[targetP][1] + dy)) / 3.14;


	float yawSteer = *cmdSteer;

	// PID Control		
	kp_d = 1.3;
	ki_d = 0.002;
	kd_d = 10;



	D_err = -atan2(_midline[targetP][0], _midline[targetP][1]);
	D_errDiff = D_err - Tmp;
	D_errSum = D_errSum + D_err;
	Tmp = D_err;

	float pidSteer = constrain(-1.0, 1.0, kp_d * D_err + ki_d * D_errSum + kd_d * D_errDiff);
	float k = 0.1;
	*cmdSteer = k * pidSteer + (1 - k) * yawSteer;
}

int getTargetP() {

	// 主要控制的是转向角，P点取得越远，转向时切内弯越明显
	int _targetP;
	if (vC[0]._type < S1) {
		if (_speed > 180) _targetP = 55;
		else if (_speed > 130) _targetP = 42;
		else _targetP = 30;
	}
	else _targetP = 55;

	if (vC[0]._type == S2 && vC[2]._type == DIRECT && abs(vC[0]._distance + vC[1]._distance)<48
		&& vC[0]._cvl == V) _targetP = 60;
	if (vC[0]._type >= S1 && (vC[0]._cvl >= IV || vC[1]._cvl >= IV)) _targetP = 30;



	return _targetP;
}
// 根据道路形状获取横向偏移量
void getXShift_d(float &dx, float &dy) {
	float tana, cosa, sina, x0, y0, x2, y2, k = 11, k_width;
	int d0 = vC[0]._distance, d1 = vC[1]._distance;



	if (vC[0]._type == DIRECT && vC[1]._type == RIGHT &&
		vC[0]._distance > int(_speed * 0.5)) { //当前方30m直道开外出现一个右弯时，外切
		dx = -_width / k;
	}
	if (vC[0]._type == DIRECT && vC[1]._type == RIGHT &&
		vC[0]._distance <= int(_speed * 0.5)) {//准备进入右弯，由外切进入内切
		dy = _width / k;//(vC[0]._distance - i) * x0 / vC[0]._distance;
	}

	if (vC[0]._type == RIGHT && vC[0]._distance > 12) {//完全进入右弯后的轨道调整

		x0 = _midline[targetP][0], y0 = _midline[targetP][1];
		x2 = _midline[targetP + 2][0], y2 = _midline[targetP + 2][1];
		tana = (x0 - x2) / (y2 - y0);
		cosa = (y2 - y0) / length(x0, y0, x2, y2);
		sina = (x0 - x2) / length(x0, y0, x2, y2);

		dx = _width / k * cosa;
		dy = _width / k * sina;

	}

	if (vC[0]._type == DIRECT && vC[1]._type == LEFT &&
		vC[0]._distance > int(_speed * 0.5)) { //当前方20m直道开外出现一个左弯时，外切
		dx = _width / k;//*(1 / (1 + exp((-i) / 20.0)) - 0.5);
	}
	if (vC[0]._type == DIRECT && vC[1]._type == LEFT &&
		vC[0]._distance <= int(_speed * 0.5)) {//准备进入左弯，由外切进入内切
		dx = -_width / k;//(vC[0]._distance - i) * x0 / vC[0]._distance;

	}
	if (vC[0]._type == LEFT && vC[0]._distance > 12) {//完全进入左弯后的轨道调整

		x0 = _midline[targetP][0], y0 = _midline[targetP][1];
		x2 = _midline[targetP + 2][0], y2 = _midline[targetP + 2][1];
		tana = (x0 - x2) / (y2 - y0);
		cosa = (y2 - y0) / length(x0, y0, x2, y2);
		sina = (x0 - x2) / length(x0, y0, x2, y2);

		dx = _width / k * cosa;
		dy = _width / k * sina;
	}


	// S1型弯道 L->R	
	// S2型弯道 R->L
	if (vC[0]._type == S1 || vC[0]._type == S2) {
		k_width = 4.0 * (1.5*d0 / (d0 + d1)) * (1.0 - 1.5*d0 / (d0 + d1));
		x0 = _midline[targetP][0], y0 = _midline[targetP][1];
		x2 = _midline[targetP + 2][0], y2 = _midline[targetP + 2][1];
		tana = (x0 - x2) / (y2 - y0);
		cosa = (y2 - y0) / length(x0, y0, x2, y2);
		sina = (x0 - x2) / length(x0, y0, x2, y2);

		dx = k_width * _width / k * cosa;
		dy = k_width * _width / k * sina;
	}

	// 根据速度做最后一步校正
	dx *= (0.14 + 3 * _speed / 700.0);
	dy *= (0.14 + 3 * _speed / 700.0);

}




/*****************************土路 控制****************************/

/****************************（土路）为不同曲率半径的弯道分配速度****************************/
float getV2(float r) {
	float _tgtV;
	if (r < 35) _tgtV = 75;
	else if (r < 55) _tgtV = 130;
	else if (r < 85) _tgtV = 180;
	else if (r < 105) _tgtV = 200;
	else _tgtV = 250;

	return _tgtV;
}

/****************************（土路）目标速度****************************/
float getTargetSpeed2(int TargetPoint) {//土路

										//检查赛道类型
	curveDetect();

	targetC = getC(TargetPoint, TargetPoint + 5, TargetPoint + 10);
	float targetV = getV2(targetC.r), _tgtV = 0.0;
	const int N = 95;
	float k[N], smpV[N];
	for (int i = 0; i < N; ++i) smpV[i] = getV2(CArray[i].r);
	for (int i = 0; i < N; ++i) k[i] = 1.0 / (1.0 + exp((smpV[i] - targetV) / 80));
	unify(k, N);
	for (int i = 0; i < N; ++i) _tgtV += k[i] * smpV[i];

	float kOfV = 1.65;
	////针对 长度较短的急弯 的速度限制
	for (int i = 0; i < vC.size(); ++i) {
		if ((vC[i]._cvl == V) && (vC[i].init_d < max(_speed*0.5, 65)) && (vC[i]._distance < 105))
		{	// 75m长V级弯道，100m处检测
			_tgtV = 88 / kOfV;
		}
		else if ((vC[i]._cvl == IV) && (vC[i].init_d < max(_speed * 0.5, 65)) && (vC[i]._distance < 105))
		{	// 75m长V级弯道，100m处检测
			_tgtV = 115 / kOfV;
		}
	}

	// 第二个左弯
	if (vC[1]._type == DIRECT && abs(vC[1]._distance - 97) <= 2 && vC[0]._type == LEFT) {
		_tgtV = 90 / kOfV;
	}

	// 土路右上方的一个超级长的左弯
	else if (vC[0]._type == LEFT && vC[1]._type == DIRECT && (vC[0]._distance + vC[1]._distance) >= 180) {
		_tgtV = 88 / kOfV;
	}

	// 右方的一个64mV级左弯
	else if (vC[0]._type == LEFT && vC[1]._type == DIRECT && (vC[0]._distance + vC[1]._distance) >= 120 &&
		vC[0]._cvl == V) {
		_tgtV = 90 / kOfV;
	}
	// 中间某个弯道
	//else if (vC[0]._type == DIRECT && vC[1]._type == RIGHT && abs(vC[1]._distance - 63) <= 2) {
	//	_tgtV = 110 / kOfV;
	//}

	// 土路左上方的左弯
	else if (vC[0]._type == DIRECT && vC[1]._type == LEFT && vC[2]._type == DIRECT
		&& vC[3]._type == LEFT && vC[4]._type == DIRECT && vC[4]._distance >= 30) {
		_tgtV = 110 / kOfV;
	}

	// 土路的最后两个弯
	else if (vC[0]._type == DIRECT && vC[1]._type == LEFT && vC[2]._type == DIRECT
		&& abs(vC[1]._distance - 51) <= 1) {
		_tgtV = 105 / kOfV;
	}

	else if (vC[0]._type == LEFT && vC[1]._type == DIRECT && vC[2]._type == LEFT
		&& abs(vC[1]._distance - 69) <= 1) {
		_tgtV = 95 / kOfV;
	}

	else if (vC[0]._type == LEFT && vC[1]._type == DIRECT && (vC[0]._distance + vC[1]._distance) >= 188) {
		_tgtV = 105 / kOfV;
	}

	// 如果前面为S型道的速度奖励
	if (vC[0]._type >= S1 || vC[1]._type >= S1) _tgtV *= 1.001;
	if (vC.size() >= 5) _tgtV *= 2.5;
	// 如果面前有直道(>=30m)的速度奖励
	if (vC[0]._type == DIRECT && vC[0]._distance > 15) _tgtV *= 1.1;


	return _tgtV * kOfV * 1.02;
}

/****************************（土路）SPEED CONTROL****************************/
void speedControl2(float *cmdAcc, float *cmdBrake, float * cmdSteer) {

	TargetPoint = max(int(_speed*0.4), 35);

	expectedSpeed = getTargetSpeed2(TargetPoint);
	//expectedSpeed = 60;
	curSpeedErr = expectedSpeed - _speed;
	speedErrSum = 0.1*speedErrSum + curSpeedErr;
	speedErrDiff = curSpeedErr - tmpSpeedErr;
	tmpSpeedErr = curSpeedErr;
	if (curSpeedErr > 0) {
		offset = *cmdAcc * 0.4;
		if (fabs(*cmdSteer) < 0.15) {
			*cmdAcc = constrain(0.0, 1.0, kp_s * curSpeedErr + ki_s * speedErrSum + kd_s * speedErrDiff + offset);
			//*cmdAcc = 1.0;
			*cmdBrake = 0;
		}
		else if (fabs(*cmdSteer) < 0.39) {
			*cmdAcc = constrain(0.0, 0.65, kp_s * curSpeedErr + ki_s * speedErrSum + kd_s * speedErrDiff + offset);
			*cmdBrake = 0;
		}
		else if (fabs(*cmdSteer)>0.65) {
			*cmdAcc = 0.005 + offset*0.05;
			*cmdBrake = 0;
		}
		else {
			*cmdAcc = 0.31 + offset*0.3;
			*cmdBrake = 0;
		}
	}
	else if (curSpeedErr < 0)
	{
		offset = 0.004;
		if (*cmdSteer > 0.60)
			*cmdBrake = constrain(0.0, 0.6, -kp_s *curSpeedErr / 10 - offset / 3);
		else if (*cmdSteer < 0.15)
			*cmdBrake = constrain(0.0, 1.0, -kp_s *curSpeedErr / 3);
		else
			*cmdBrake = constrain(0.0, 0.8, -kp_s *curSpeedErr / 5 - offset / 3);
		*cmdAcc = 0.0;
	}

}

/****************************（土路）STEER CONTROL****************************/
void steerControl2(float *cmdSteer)
{
	dx = 0; dy = 0;
	//getXShift_d2(dx, dy);

	if (vC[0]._type < S1) {
		if (_speed>180) { *cmdSteer = (_yaw - 8 * atan2(_midline[80][0] + dx, _midline[80][1] + dy)) / 3.14; }
		else if (_speed>130) { *cmdSteer = (_yaw - 8 * atan2(_midline[60][0] + dx, _midline[60][1] + dy)) / 3.14; }
		else if (_speed>70) { *cmdSteer = (_yaw - 7 * atan2(_midline[35][0] + dx, _midline[35][1] + dy)) / 3.14; }
		else { *cmdSteer = (_yaw - 8 * atan2(_midline[35][0] + dx, _midline[35][1] + dy)) / 3.14; }
	}
	else {
		*cmdSteer = (_yaw - 8 * atan2(_midline[50][0] + dx, _midline[50][1] + dy)) / 3.14;
	}
	float yawSteer = *cmdSteer;

	// PID Control		
	kp_d = 1.3;
	ki_d = 0.002;
	kd_d = 10;

	int targetP;
	if (vC[0]._type < S1) {
		if (_speed > 180) targetP = 80;
		else if (_speed > 130) targetP = 60;
		else targetP = 35;
	}
	else targetP = 50;


	D_err = -atan2(_midline[targetP][0], _midline[targetP][1]);
	D_errDiff = D_err - Tmp;
	D_errSum = D_errSum + D_err;
	Tmp = D_err;

	float pidSteer = constrain(-1.0, 1.0, kp_d * D_err + ki_d * D_errSum + kd_d * D_errDiff);
	float k = 0.1;
	*cmdSteer = k * pidSteer + (1 - k) * yawSteer;
}

// 根据道路形状获取横向偏移量（土路）
void getXShift_d2(float &dx, float &dy) {// based on the previous path's info
										 //   rewrite the ideal track info
	float tana, cosa, sina, x0, y0, x2, y2, k = 0.04, k_width;
	int d0 = vC[0]._distance, d1 = vC[1]._distance;

	int targetP;
	if (vC[0]._type < S1) {
		if (_speed > 180) targetP = 80;
		else if (_speed > 130) targetP = 60;
		else targetP = 35;
	}
	else targetP = 50;



	if (vC[0]._type == DIRECT && vC[1]._type == RIGHT &&
		vC[0]._distance > max(int(_speed * 0.85), 100) &&
		vC[0]._distance < int(_speed * 1.5)) { //当前方30m直道开外出现一个右弯时，外切
		dx = -_width / k;
	}
	if (vC[0]._type == DIRECT && vC[1]._type == RIGHT &&
		vC[0]._distance <= max(int(_speed * 0.85), 100)) {//准备进入右弯，由外切进入内切
		dx = _width / k;//(vC[0]._distance - i) * x0 / vC[0]._distance;
	}

	if (vC[0]._type == RIGHT && vC[0]._distance > 12) {//完全进入右弯后的轨道调整

		x0 = _midline[targetP][0], y0 = _midline[targetP][1];
		x2 = _midline[targetP + 2][0], y2 = _midline[targetP + 2][1];
		tana = (x0 - x2) / (y2 - y0);
		cosa = (y2 - y0) / length(x0, y0, x2, y2);
		sina = (x0 - x2) / length(x0, y0, x2, y2);

		dx = _width / k * cosa * 6;
		dy = _width / k * sina * 2;

	}

	if (vC[0]._type == DIRECT && vC[1]._type == LEFT &&
		vC[0]._distance > max(int(_speed * 0.85), 100) &&
		vC[0]._distance < int(_speed * 1.5)) { //当前方20m直道开外出现一个左弯时，外切
		dx = _width / k;//*(1 / (1 + exp((-i) / 20.0)) - 0.5);
	}
	if (vC[0]._type == DIRECT && vC[1]._type == LEFT &&
		vC[0]._distance <= max(int(_speed * 0.85), 100)) {//准备进入左弯，由外切进入内切
		dx = -_width / k;//(vC[0]._distance - i) * x0 / vC[0]._distance;

	}
	if (vC[0]._type == LEFT && vC[0]._distance > 12) {//完全进入左弯后的轨道调整

		x0 = _midline[targetP][0], y0 = _midline[targetP][1];
		x2 = _midline[targetP + 2][0], y2 = _midline[targetP + 2][1];
		tana = (x0 - x2) / (y2 - y0);
		cosa = (y2 - y0) / length(x0, y0, x2, y2);
		sina = (x0 - x2) / length(x0, y0, x2, y2);

		dx = _width / k * cosa * 6;
		dy = _width / k * sina * 2;
	}


	// S1型弯道 L->R	
	// S2型弯道 R->L
	if (vC[0]._type == S1 || vC[0]._type == S2) {
		k_width = 4.0 * (1.5*d0 / (d0 + d1)) * (1.0 - 1.5*d0 / (d0 + d1));

		x0 = _midline[targetP][0], y0 = _midline[targetP][1];
		x2 = _midline[targetP + 2][0], y2 = _midline[targetP + 2][1];
		tana = (x0 - x2) / (y2 - y0);
		cosa = (y2 - y0) / length(x0, y0, x2, y2);
		sina = (x0 - x2) / length(x0, y0, x2, y2);

		dx = k_width * _width / k * cosa;
	}

	// 根据速度做最后一步校正
	//dx *= (0.14 + 3 * _speed / 700.0);
	//dy *= (0.14 + 3 * _speed / 700.0);

}


/*****************************弯道类型检测函数**********************************/
void curveDetect() {//判断弯道类型
	for (int i = 0; i < 95; ++i)
		CArray[i] = getC(2 * i, 2 * i + 5, 2 * i + 9);

	Curve pC;
	pC._distance = 0;
	int N = 95;
	vC.clear();
	bool initflag = true;
	int _minLength = 11;//弯道的长度必须长于11m才计入
	for (int i = 0; i < N; ++i) {
		if (CArray[i].r > 200) {//direct line
			pC._type = DIRECT; pC._distance += 2;
			if (initflag) pC.init_d = 2 * i + 5, initflag = false;
			if (!(CArray[i + 1].r > 200)) {
				if (pC._distance > _minLength)
					vC.push_back(pC);
				pC._distance = 0; initflag = true;
			}
		}
		else {
			if (CArray[i].sign > 0) { //left turn
				pC._type = LEFT; pC._distance += 2;
				if (initflag) pC.init_d = 2 * i + 5, initflag = false;
				if (!(CArray[i + 1].sign > 0 && CArray[i + 1].r < 200)) {
					if (pC._distance > _minLength)
						vC.push_back(pC);
					pC._distance = 0; initflag = true;
				}
			}
			else if (CArray[i].sign < 0) { //right turn
				pC._type = RIGHT; pC._distance += 2;
				if (initflag) pC.init_d = 2 * i + 5, initflag = false;
				if (!(CArray[i + 1].sign < 0 && CArray[i + 1].r < 200)) {
					if (pC._distance > _minLength)
						vC.push_back(pC);
					pC._distance = 0; initflag = true;
				}
			}
		}
	}
	// 对vC里不同弯道类型作进一步讨论,包括鉴别弯道弯曲程度，是否有S型弯道等
	for (int i = 0; i < vC.size(); ++i)
		vC[i]._cvl = getCurvaLevel(CArray[(vC[i].init_d + vC[i]._distance / 2) / 2].r);
	//将前两个弯道的信息更新为S弯道
	if (vC[0]._type == LEFT && vC[1]._type == RIGHT)
		vC[0]._type = S1, vC[1]._type = S1;
	else if (vC[1]._type == LEFT && vC[0]._type == RIGHT)
		vC[0]._type = S2, vC[1]._type = S2;
	else if (vC[1]._type == LEFT && vC[2]._type == RIGHT)
		vC[1]._type = S1, vC[2]._type = S1;
	else if (vC[2]._type == LEFT && vC[1]._type == RIGHT)
		vC[1]._type = S2, vC[2]._type = S2;
	else if (vC[0]._type == LEFT && vC[2]._type == RIGHT && vC[1]._distance <= 18)
		vC[0]._type = S1, vC[1]._type = S1, vC[0]._type = S1;
	else if (vC[2]._type == LEFT && vC[0]._type == RIGHT && vC[1]._distance <= 18)
		vC[0]._type = S2, vC[1]._type = S2, vC[0]._type = S2;
}



/****************************一些辅助函数****************************/
circle getC(int p1, int p2, int p3)
{
	circle tmp;
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
CurvaLevel getCurvaLevel(float r) {
	CurvaLevel _cvl;
	if (r > 200) _cvl = ZERO;
	else if (r > 160) _cvl = I;
	else if (r > 120) _cvl = II;
	else if (r > 80) _cvl = III;
	else if (r > 50) _cvl = IV;
	else  _cvl = V;

	return _cvl;
}
double constrain(double lowerBoundary, double upperBoundary, double input)
{
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
void unify(float k[], int size) { // 归一化函数，将数组里的数映射成和为1的比例系数
	float sum = 0.0;
	for (int i = 0; i < size; ++i) sum += k[i];
	for (int i = 0; i < size; ++i) k[i] /= sum;
}

/****************************档位函数****************************/
void gear(int* cmdGear, float* cmdAcc, float* cmdBrake) {//公路档位
	if (_speed <= 45)*cmdGear = 1;
	if (_speed>45 && _speed <= 95 && *cmdGear == 1 && _rpm>650) *cmdGear = *cmdGear + 1;
	if (_speed>95 && _speed <= 140 && *cmdGear == 2 && _rpm>650) *cmdGear = *cmdGear + 1;
	if (_speed>140 && _speed <= 185 && *cmdGear == 3 && _rpm>650) *cmdGear = *cmdGear + 1;
	if (_speed>185 && _speed <= 230 && *cmdGear == 4 && _rpm>650)*cmdGear = *cmdGear + 1;
	if (_speed>230 && *cmdGear == 5 && _rpm>600)*cmdGear = *cmdGear + 1;
	if (_speed <= 40 && *cmdGear == 2)*cmdGear = *cmdGear - 1;
	if (_speed>40 && _speed <= 90 && *cmdGear == 3 && _rpm<600) *cmdGear = *cmdGear - 1;
	if (_speed>90 && _speed <= 135 && *cmdGear == 4 && _rpm<600) *cmdGear = *cmdGear - 1;
	if (_speed>135 && _speed <= 185 && *cmdGear == 5 && _rpm<600) *cmdGear = *cmdGear - 1;
	if (_speed>185 && _speed <= 220 && *cmdGear == 6 && _rpm<600)*cmdGear = *cmdGear - 1;
}
void gear2(int* cmdGear, float* cmdAcc, float* cmdBrake) {//沙地档位
	if (_speed <= 60)*cmdGear = 1;
	if (_speed>60 && _speed <= 110 && *cmdGear == 1 && _rpm>600) *cmdGear = *cmdGear + 1;
	if (_speed>110 && _speed <= 160 && *cmdGear == 2 && _rpm>600) *cmdGear = *cmdGear + 1;
	if (_speed>160 && _speed <= 180 && *cmdGear == 3 && _rpm>600) *cmdGear = *cmdGear + 1;
	if (_speed>180 && _speed <= 210 && *cmdGear == 4 && _rpm>600)*cmdGear = *cmdGear + 1;
	if (_speed>210 && *cmdGear == 5 && _rpm>600)*cmdGear = *cmdGear + 1;
	if (_speed <= 50 && *cmdGear == 2)*cmdGear = *cmdGear - 1;
	if (_speed>50 && _speed <= 90 && *cmdGear == 3 && _rpm<600) *cmdGear = *cmdGear - 1;
	if (_speed>90 && _speed <= 145 && *cmdGear == 4 && _rpm<600) *cmdGear = *cmdGear - 1;
	if (_speed>145 && _speed <= 170 && *cmdGear == 5 && _rpm<600) *cmdGear = *cmdGear - 1;
	if (_speed>170 && _speed <= 205 && *cmdGear == 6 && _rpm<600)*cmdGear = *cmdGear - 1;
}

/**************************写数据的函数******************************/
// void writeData(float* cmdAcc, float* cmdBrake, float* cmdSteer) {
// 	char* _stype, *_scvl;
// 	fprintf(f, "ct:%d, ", counter);
// 	for (int i = 0; i < vC.size(); ++i) {
// 		if (vC[i]._type == DIRECT) _stype = "DIRECT";
// 		if (vC[i]._type == LEFT) _stype = "LEFT";
// 		if (vC[i]._type == RIGHT) _stype = "RIGHT";
// 		if (vC[i]._type == S1) _stype = "S1";
// 		if (vC[i]._type == S2) _stype = "S2";
// 		if (vC[i]._cvl == V) _scvl = "V";
// 		if (vC[i]._cvl == IV) _scvl = "IV";
// 		if (vC[i]._cvl == III) _scvl = "III";
// 		if (vC[i]._cvl == II) _scvl = "II";
// 		if (vC[i]._cvl == I) _scvl = "I";
// 		if (vC[i]._cvl == ZERO) _scvl = "ZERO";
// 		fprintf(f, "%s,%d,%s,\t", _stype, vC[i]._distance, _scvl);
// 	}
// 	fprintf(f,
// 		"速度 %.3f,期望速度:%.3f, XRate:%.3f, steer:%.3f, D_err:%.3f\n", _speed, expectedSpeed, horizonXRate, *cmdSteer, D_err);
// }


//************************************ 主体函数*************************************//
static void userDriverSetParam(float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear) {
	if (parameterSet == false)		// Initialization Part
	{
		PIDParamSetter();
	}
	else
	{
		//根据加速到20km/h的时间不同区分公路与土路
		if (_speed<20) { *cmdAcc = 1; *cmdBrake = 0; *cmdGear = 1; accCounter++; }
		horizonXRate = 2 * _midline[0][0] / _width;
		if (accCounter < 92) {//公路
			*cmdSteer = (_yaw - 7 * atan2(_midline[25][0], _midline[25][1])) / 3.14;
			gear(cmdGear, cmdAcc, cmdBrake);
			speedControl(cmdAcc, cmdBrake, cmdSteer);
			steerControl(cmdSteer);
		}
		else { //沙地
			gear2(cmdGear, cmdAcc, cmdBrake);
			*cmdSteer = (_yaw - 8 * atan2(_midline[20][0], _midline[20][1])) / 3.14;
			speedControl2(cmdAcc, cmdBrake, cmdSteer);
			steerControl2(cmdSteer);
		}

		counter++;
		// printf("DIRTD_error: %f cmdSteer ： %f\n", D_err, *cmdSteer);
		// writeData(cmdAcc, cmdBrake, cmdSteer);
	}
}

/*v8.0版本
该版本成绩为 	Dirt	E-Road		E-Track4	Brondehach		Street1		CG1		Total
				104		81 			125			96!!			92			50		548

学长版本	    102		82			126			100??			95			48		553?

*/