#include <stdio.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <regex>
#include "XPLMDataAccess.h"
#include "XPLMMenus.h"
#include "XPLMProcessing.h"
//#include "json.hpp"

using namespace std;
//using json = nlohmann::json;

static float TEKLoopCallback(float inElapsedSinceLastCall, float inElapsedTimeSinceLastFlightLoop, int inCounter, void* inRefcon);

//===========================================定义各DataRef================================================
static XPLMDataRef _aircraftName = NULL;
static XPLMDataRef _onGrass = NULL;//是否在草地上
static XPLMDataRef _onGround = NULL;//是否在地面上
static XPLMDataRef _liveryPath = NULL;//涂装路径
//=============模拟器直接数据起始==============
static XPLMDataRef _latitude = NULL;//飞机纬度
static XPLMDataRef _longitude = NULL;//飞机经度
static XPLMDataRef _elevation = NULL;//飞机高度
static XPLMDataRef _aboveGroundLevel = NULL;//飞机离地高度
static XPLMDataRef _vpath = NULL;
//飞机空速
//飞机地速
static XPLMDataRef _heading = NULL;//飞机航向
//飞机磁航向
//飞机俯仰
//飞机横滚
//飞机总重
static XPLMDataRef _fuelWeight = NULL;//燃油重量
static XPLMDataRef _payloadWeight = NULL;//商载重量
static XPLMDataRef _distanceFlown = NULL;//飞行距离
static XPLMDataRef _zuluTime = NULL;//自00：00开始计算的秒数
static XPLMDataRef _strobeLights = NULL;//频闪灯状态
//GPWS警告
//=============模拟器直接数据结尾==============

//=============仪表显示数据起始===============
static XPLMDataRef _idc_airspeed = NULL;//指示空速
static XPLMDataRef _idc_elevation = NULL;//指示高度
static XPLMDataRef _idc_altimeter = NULL;//高度表拨正值
static XPLMDataRef _idc_ldgLight = NULL;//着陆灯
static XPLMDataRef _idc_pitch = NULL;//俯仰
static XPLMDataRef _idc_roll = NULL;//横滚
static XPLMDataRef _idc_heading = NULL;//航向
//应答机模式
static XPLMDataRef _idc_transponderCode = NULL;//应答机代码
static XPLMDataRef _idc_fuelOnBoard = NULL;//机载燃油
static XPLMDataRef _idc_parkingBrake = NULL;//停机刹车
//襟翼位置
//减速板
//起落架
static XPLMDataRef _idc_com1 = NULL;//COM1频率
//static XPLMDataRef _idc_engine_fuelPump = NULL;//油泵
static XPLMDataRef _idc_engine_N1 = NULL;//发动机N1参数
static XPLMDataRef _idc_engine_N2 = NULL;//发动机N2参数
static XPLMDataRef _idc_engine_EGT = NULL;//发动机排气温度
static XPLMDataRef _idc_taxiLight = NULL;//滑行灯
static XPLMDataRef _idc_verticalSpeed = NULL;//垂直速度
static XPLMDataRef _idc_masterWarning = NULL;//主警告
//自动驾驶模式
static XPLMDataRef _idc_trueAirspeed = NULL;//真空速
static XPLMDataRef _idc_autoThrottle = NULL;//自动油门状态
//=============仪表显示数据结尾===============

//=============质量监测数据起始===============
static XPLMDataRef _groundSpeed = NULL;// 地速
static XPLMDataRef _magheading = NULL;// 磁航向
static XPLMDataRef _totalWeight = NULL;// 全重
static XPLMDataRef _airspeed = NULL;// 空速
static XPLMDataRef _pitch = NULL;// 俯仰角
// 俯仰率
static XPLMDataRef _roll = NULL;// 滚转角
// 马赫数
//static XPLMDataRef _idc_gear = NULL;// 起落架位置
static XPLMDataRef _idc_flaps = NULL;// 襟翼位置
// 起落架状态
static XPLMDataRef _gForce = NULL;// 垂直过载
static XPLMDataRef _GPWS = NULL;// 近地警告
static XPLMDataRef _idc_speedBrake = NULL;// 减速板
static XPLMDataRef _reverse = NULL;// 反推
static XPLMDataRef _idc_transponder = NULL;// TCAS
static XPLMDataRef _idc_autopilotMode = NULL;// 自动驾驶
static bool readlivery = true;
//=============质量监测数据结尾===============

string outputPath = "./Output/TEKConnect/TEKConnect_Query.json";

ofstream output(outputPath, ios::trunc);

PLUGIN_API int XPluginStart(char* outName, char* outSignature, char* outDescription) {
	strcpy(outName, "TEKConnect");
	strcpy(outSignature, "peercat.connector.vatek");
	strcpy(outDescription, "Peercat plugin for VATEK to get data from simulator.(X-Plane)");

	_aircraftName = XPLMFindDataRef("sim/aircraft/view/acf_ICAO");
	_gForce = XPLMFindDataRef("sim/flightmodel/forces/g_nrml");
	_reverse = XPLMFindDataRef("sim/cockpit/warnings/annunciators/reverse");
	_liveryPath = XPLMFindDataRef("sim/aircraft/view/acf_livery_path");
	//初始化dataref
	//===直接数据===
	_vpath = XPLMFindDataRef("sim/flightmodel/position/vpath");//单精度
	_GPWS = XPLMFindDataRef("sim/cockpit2/annunciators/GPWS");//整数，布尔值
	_latitude = XPLMFindDataRef("sim/flightmodel/position/latitude");//双精度
	_longitude = XPLMFindDataRef("sim/flightmodel/position/longitude");//双精度
	_elevation = XPLMFindDataRef("sim/flightmodel/position/elevation");//双精度，米
	_airspeed = XPLMFindDataRef("sim/flightmodel/position/true_airspeed");//单精度，米每秒
	_groundSpeed = XPLMFindDataRef("sim/flightmodel/position/groundspeed");//单精度，米每秒
	_heading = XPLMFindDataRef("sim/flightmodel/position/true_psi");//单精度，度
	_magheading = XPLMFindDataRef("sim/flightmodel/position/magpsi");//单精度，度
	_pitch = XPLMFindDataRef("sim/flightmodel/position/true_theta");//单精度，度
	_roll = XPLMFindDataRef("sim/flightmodel/position/true_phi");//单精度，度
	_totalWeight = XPLMFindDataRef("sim/flightmodel/weight/m_total");//单精度，千克
	_fuelWeight = XPLMFindDataRef("sim/flightmodel/weight/m_fuel_total");//单精度，千克
	_payloadWeight = XPLMFindDataRef("sim/flightmodel/weight/m_fixed");//单精度，千克
	_distanceFlown = XPLMFindDataRef("sim/flightmodel/controls/dist");//单精度，米
	_zuluTime = XPLMFindDataRef("sim/time/zulu_time_sec");//单精度，秒
	_strobeLights = XPLMFindDataRef("sim/cockpit/electrical/strobe_lights_on");//整型，布尔值
	_aboveGroundLevel = XPLMFindDataRef("sim/flightmodel/position/y_agl");//单精度，米

	//===仪表数据===
	_idc_airspeed = XPLMFindDataRef("sim/flightmodel/position/indicated_airspeed");//单精度，节
	_idc_trueAirspeed = XPLMFindDataRef("sim/flightmodel/position/true_airspeed");//单精度，节
	_idc_elevation = XPLMFindDataRef("sim/cockpit2/gauges/indicators/altitude_ft_pilot");//单精度，英尺
	_idc_altimeter = XPLMFindDataRef("sim/cockpit/misc/barometer_setting");//单精度
	_idc_ldgLight = XPLMFindDataRef("sim/cockpit2/switches/landing_lights_on");//整形，布尔运算值
	_idc_pitch = XPLMFindDataRef("sim/cockpit/gyros/the_ind_elec_pilot_deg");//单精度，度
	_idc_roll = XPLMFindDataRef("sim/cockpit/gyros/phi_ind_elec_pilot_deg");//单精度，度
	_idc_heading = XPLMFindDataRef("sim/cockpit/gyros/psi_ind_elec_pilot_degm");//单精度，度
	_idc_transponder = XPLMFindDataRef("sim/cockpit2/radios/actuators/transponder_mode");//整型，下同
	_idc_transponderCode = XPLMFindDataRef("sim/cockpit2/radios/actuators/transponder_code");
	_idc_fuelOnBoard = XPLMFindDataRef("sim/cockpit2/fuel/fuel_quantity");//单精度数组[9]，千克
	_idc_parkingBrake = XPLMFindDataRef("sim/flightmodel/controls/parkbrake");//单精度
	_idc_flaps = XPLMFindDataRef("sim/flightmodel2/controls/flap_handle_deploy_ratio");//单精度，ratio
	_idc_speedBrake = XPLMFindDataRef("sim/flightmodel/controls/sbrkrqst");//单精度，0~1.5
	//_idc_gear = XPLMFindDataRef("sim/flightmodel2/gear/deploy_ratio");//单精度，ratio
	_idc_com1 = XPLMFindDataRef("sim/cockpit/radios/com1_freq_hz");//整型，10Hz
	//_idc_engine_fuelPump = XPLMFindDataRef("sim/cockpit/engine/fuel_pump_on");//整型数组[8]，布尔运算值
	_idc_engine_N1 = XPLMFindDataRef("sim/flightmodel/engine/ENGN_N1_");//单精度数组[8]，百分比
	_idc_engine_N2 = XPLMFindDataRef("sim/flightmodel/engine/ENGN_N2_");//单精度数组[8]，百分比
	_idc_engine_EGT = XPLMFindDataRef("sim/flightmodel/engine/ENGN_EGT");//单精度数组[8]，百分比
	_idc_taxiLight = XPLMFindDataRef("sim/cockpit/electrical/taxi_light_on");//整型，布尔值
	_idc_verticalSpeed = XPLMFindDataRef("sim/cockpit2/gauges/indicators/vvi_fpm_pilot");//单精度，fpm
	_idc_masterWarning = XPLMFindDataRef("sim/cockpit/warnings/annunciators/master_warning");//单精度，布尔值
	_idc_autopilotMode = XPLMFindDataRef("sim/cockpit2/autopilot/autopilot_on");//整型，0-off 1-FD 2-ON
	_idc_autoThrottle = XPLMFindDataRef("sim/cockpit2/autopilot/autothrottle_on");//整型，布尔值
	//注册TekLoopCallback循环事件，每0.5秒一次
	XPLMRegisterFlightLoopCallback(TEKLoopCallback, 0.5, NULL);

	return (_latitude != NULL) ? 1 : 0;
}

PLUGIN_API void	XPluginStop(void){
	XPLMUnregisterFlightLoopCallback(TEKLoopCallback, NULL);

	output.close();
}

PLUGIN_API void XPluginDisable(void){
	output.flush();
}

PLUGIN_API int XPluginEnable(void){
	return 1;
}

PLUGIN_API void XPluginReceiveMessage(XPLMPluginID inFromWho,int inMessage,void * inParam){
}
                            
float TEKLoopCallback(float inElapsedSinceLastCall, float inElapsedTimeSinceLastFlightLoop, int inCounter, void* inRefcon) {
	float elapsed = XPLMGetElapsedTime();
	double __latitude = XPLMGetDatad(_latitude);
	double __longitude = XPLMGetDatad(_longitude);
	double __elevation = XPLMGetDatad(_elevation);//Elevation
	float __airspeed = XPLMGetDataf(_airspeed);//Airspeed
	float __groundSpeed = XPLMGetDataf(_groundSpeed);
	float __heading = XPLMGetDataf(_heading);//heading
	float __magHeading = XPLMGetDataf(_magheading);//magheading
	float __pitch = XPLMGetDataf(_pitch);//pitch
	float __roll = XPLMGetDataf(_roll);//roll
	float __totalWeight = XPLMGetDataf(_totalWeight);//totalWeight
	float __fuelWeight = XPLMGetDataf(_fuelWeight);
	float __payloadWeight = XPLMGetDataf(_payloadWeight);//payloadWeight
	float __distanceFlown = XPLMGetDataf(_distanceFlown);//distanceFlown
	float __zuluTime = XPLMGetDataf(_zuluTime);
	int __strobeLights = XPLMGetDatai(_strobeLights);
	char __aircraftName[40];
	XPLMGetDatab(_aircraftName, __aircraftName, 0, 39);
	//==========================_idc_data======================
	float __idc_airspeed = XPLMGetDataf(_idc_airspeed);
	float __idc_trueAirspeed = XPLMGetDataf(_idc_trueAirspeed);//_idc_trueAirspeed
	float __idc_elevation = XPLMGetDataf(_idc_elevation);
	float __idc_altimeter = XPLMGetDataf(_idc_altimeter);
	int __idc_ldgLight = XPLMGetDatai(_idc_ldgLight);
	float __idc_pitch = XPLMGetDataf(_idc_pitch);
	float __idc_roll = XPLMGetDataf(_idc_roll);
	float __idc_heading = XPLMGetDataf(_idc_heading);
	int __idc_transponder = XPLMGetDatai(_idc_transponder);
	int __idc_transponderCode = XPLMGetDatai(_idc_transponderCode);
	float __idc_fuelOnBoard[9];
	XPLMGetDatavf(_idc_fuelOnBoard, __idc_fuelOnBoard, 0, 7);//fuel on board
	float __idc_parkingBrake = XPLMGetDataf(_idc_parkingBrake);
	float __idc_flaps = XPLMGetDataf(_idc_flaps);
	float __idc_speedBrake = XPLMGetDataf(_idc_speedBrake);
	//float __idc_gear[2];//gear
	//XPLMGetDatavf(_idc_gear, __idc_gear, 0, 99);
	int __idc_com1 = XPLMGetDatai(_idc_com1);//com1
	//int __idc_engineFuelPump[8];//engine fuel pump
	//XPLMGetDatavi(_idc_engine_fuelPump, __idc_engineFuelPump, 0, 7);
	float __idc_engine_N1[8];//engine n1
	XPLMGetDatavf(_idc_engine_N1, __idc_engine_N1, 0, 7);
	float __idc_engine_N2[8];//engine n2
	XPLMGetDatavf(_idc_engine_N2, __idc_engine_N2, 0, 7);
	float __idc_engine_EGT[8];//engine EGT
	XPLMGetDatavf(_idc_engine_EGT, __idc_engine_EGT, 0, 7);
	int __idc_taxiLight = XPLMGetDatai(_idc_taxiLight);
	float __idc_verticalSpeed = XPLMGetDataf(_idc_verticalSpeed);
	float __idc_masterWarning = XPLMGetDataf(_idc_masterWarning);
	int __idc_autopilotMode = XPLMGetDatai(_idc_autopilotMode);
	int __idc_autoThrottle = XPLMGetDatai(_idc_autoThrottle);
	float __gForce = XPLMGetDataf(_gForce);
	int __GPWS = XPLMGetDatai(_GPWS);
	int __onGrass[10];
	XPLMGetDatavi(_onGrass, __onGrass, 0, 9);
	int __onGround[10];
	XPLMGetDatavi(_onGround, __onGround, 0, 9);
	int __reverse = XPLMGetDatai(_reverse);
	char __liveryPath[1024];
	string aircraftName = __aircraftName;
	string liveryPath = __liveryPath;
	float __aboveGroundLevel = XPLMGetDataf(_aboveGroundLevel);
	float __vpath = XPLMGetDataf(_vpath);
	if (readlivery) {
		XPLMGetDatab(_liveryPath, __liveryPath, 0, 1023);
		
		readlivery = false;
	}
	if (aircraftName == "A321" && __liveryPath == strtok(__liveryPath, "NEO")) {
		__aircraftName[1] = '2';
		__aircraftName[2] = '1';
		__aircraftName[3] = 'N';
	}
	else if (aircraftName == "A320" && __liveryPath == strtok(__liveryPath, "NEO")) {
		__aircraftName[1] = '2';
		__aircraftName[2] = '0';
		__aircraftName[3] = 'N';
	}


	output.open(outputPath);
	/*json Joutput = {
		{"纬度",__latitude},
		{"经度",__longitude},
		{"地速",__groundSpeed},
		{"磁航向",__magHeading},
		{"全重",__totalWeight},
		{"空速",__airspeed},
		{"俯仰角",__pitch},
		{"滚转角",__roll},
		{"襟翼位置",__idc_flaps},
		{"垂直过载",__gForce},
		{"减速板",__idc_speedBrake},
		{"TCAS",__idc_transponder},
		{"自动驾驶",__idc_autopilotMode}
	};*/

	//ofstream o(outputPath);

	//o <<Joutput;
	//输出数据
	output <<
		//直接数据=================
		"{\n"<<
		"\"AircraftName\":\"" << __aircraftName << "\", \n" <<
		"\"AGL\":" << __aboveGroundLevel << ", \n" <<
		"\"vpath\":" << __vpath << ", \n" <<
		"\"LiveryPath\":\"" << __liveryPath << "\", \n" <<
		"\"Latitude\":" << __latitude << ", \n" <<
		"\"Longitude\":" << __longitude << ", \n" <<
		"\"Elevation\":" << __elevation << ", \n" <<
		"\"Airspeed\":" << __airspeed << ", \n" <<
		"\"GroundSpeed\":" << __groundSpeed << ", \n" <<
		"\"Heading\":" << __heading << ", \n" <<
		"\"MagHeading\":" << __magHeading << ", \n" <<
		"\"Pitch\":" << __pitch << ", \n" <<
		"\"Roll\":" << __roll << ", \n" <<
		"\"totalWeight\":" << __totalWeight << ", \n" <<
		"\"FuelWeight\":" << __fuelWeight << ", \n" <<
		"\"PayloadWeight\":" << __payloadWeight << ", \n" <<
		"\"DistanceFlown\":" << __distanceFlown << ", \n" <<
		"\"ZuluTime\":" << __zuluTime << ", \n" <<
		"\"StrobeLight\":" << __strobeLights << ", \n" <<
		//仪表数据==================
		"\"_idc_Airspeed\":" << __idc_airspeed << ", \n" <<
		"\"_idc_TrueAirspeed\":" << __idc_trueAirspeed << ", \n" <<
		"\"_idc_Elevation\":" << __idc_elevation << ", \n" <<
		"\"_idc_Altimeter\":" << __idc_altimeter << ", \n" <<
		"\"_idc_LandingLight\":" << __idc_ldgLight << ", \n" <<
		"\"_idc_Pitch\":" << __idc_pitch << ", \n" <<
		"\"_idc_Heading\":" << __idc_heading << ", \n" <<
		"\"_idc_Transponder\":" << __idc_transponder << ", \n" <<
		"\"_idc_TransponderCode\":" << __idc_transponderCode << ", \n" <<
		//FuelOnBoard
		"\"_idc_ParkingBrake\":" << __idc_parkingBrake << ", \n" <<
		"\"_idc_Flaps\":" << __idc_flaps << ", \n" <<
		//"\"_idc_Gear\":" << __idc_gear[0] << ", \n" <<
		"\"_idc_COM1\":" << __idc_com1 << ", \n" <<
		//"\"EngineFuelPump\":{\n"
		//"\t\"0\":" << __idc_engineFuelPump[0] << ", \n" <<
		//"\t\"1\":" << __idc_engineFuelPump[1] << ", \n" <<
		//"\t\"2\":" << __idc_engineFuelPump[2] << ", \n" <<
		//"\t\"3\":" << __idc_engineFuelPump[3] << " \n" <<
		//"},\n"<<
		"\"Engine_N1\":{\n"<<
		"\t\"0\":" << __idc_engine_N1[0] << ", \n" <<
		"\t\"1\":" << __idc_engine_N1[1] << ", \n" <<
		"\t\"2\":" << __idc_engine_N1[2] << ", \n" <<
		"\t\"3\":" << __idc_engine_N1[3] << " \n" <<
		"},\n"<<
		"\"Engine_N2\":{\n"<<
		"\t\"0\":" << __idc_engine_N2[0] << ", \n" <<
		"\t\"1\":" << __idc_engine_N2[1] << ", \n" <<
		"\t\"2\":" << __idc_engine_N2[2] << ", \n" <<
		"\t\"3\":" << __idc_engine_N2[3] << " \n" <<
		"},\n"<<
		"\"Engine_EGT\":{\n"<<
		"\t\"0\":" << __idc_engine_EGT[0] << ", \n" <<
		"\t\"1\":" << __idc_engine_EGT[1] << ", \n" <<
		"\t\"2\":" << __idc_engine_EGT[2] << ", \n" <<
		"\t\"3\":" << __idc_engine_EGT[3] << " \n" <<
		"},\n" <<
		"\"_idc_VerticalSpeed\":" << __idc_verticalSpeed << ", \n" <<
		"\"MasterWarning\":" << __idc_masterWarning << ", \n" <<
		"\"AutopilotMode\":" << __idc_autopilotMode << ", \n" <<
		"\"autoThrottle\":" << __idc_autoThrottle << ", \n"<<
		"\"gForce\":"<< __gForce<<", \n"<<
		"\"GPWS\":"<<__GPWS<<", \n"<<
		"\"reverse\":" << __reverse << ", \n" <<
		"\"onGrass?\":" << __onGrass[0] << ", \n" <<
		"\"onGround?\":" << __onGround[0] << " \n" <<
		"}";
	output.close();
	//o.close();
	
	return 0.5;
}


