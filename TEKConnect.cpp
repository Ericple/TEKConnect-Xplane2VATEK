#include <stdio.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include "XPLMDataAccess.h"
#include "XPLMMenus.h"
#include "XPLMProcessing.h"

using namespace std;

static float TEKLoopCallback(float inElapsedSinceLastCall, float inElapsedTimeSinceLastFlightLoop, int inCounter, void* inRefcon);

//===========================================定义各DataRef================================================

//=============模拟器直接数据起始==============
static XPLMDataRef _latitude = NULL;//飞机纬度
static XPLMDataRef _longitude = NULL;//飞机经度
static XPLMDataRef _elevation = NULL;//飞机高度
static XPLMDataRef _airspeed = NULL;//飞机空速
static XPLMDataRef _groundSpeed = NULL;//飞机地速
static XPLMDataRef _heading = NULL;//飞机航向
static XPLMDataRef _magheading = NULL;//飞机磁航向
static XPLMDataRef _pitch = NULL;//飞机俯仰
static XPLMDataRef _roll = NULL;//飞机横滚
static XPLMDataRef _totalWeight = NULL;//飞机总重
static XPLMDataRef _fuelWeight = NULL;//燃油重量
static XPLMDataRef _payloadWeight = NULL;//商载重量
static XPLMDataRef _distanceFlown = NULL;//飞行距离
static XPLMDataRef _zuluTime = NULL;//自00：00开始计算的秒数
static XPLMDataRef _strobeLights = NULL;//频闪灯状态
//=============模拟器直接数据结尾==============

//=============仪表显示数据起始===============
static XPLMDataRef _idc_airspeed = NULL;//指示空速
static XPLMDataRef _idc_elevation = NULL;//指示高度
static XPLMDataRef _idc_altimeter = NULL;//高度表拨正值
static XPLMDataRef _idc_ldgLight = NULL;//着陆灯
static XPLMDataRef _idc_pitch = NULL;//俯仰
static XPLMDataRef _idc_roll = NULL;//横滚
static XPLMDataRef _idc_heading = NULL;//航向
static XPLMDataRef _idc_transponder = NULL;//应答机模式
static XPLMDataRef _idc_transponderCode = NULL;//应答机代码
static XPLMDataRef _idc_fuelOnBoard = NULL;//机载燃油
static XPLMDataRef _idc_parkingBrake = NULL;//停机刹车
static XPLMDataRef _idc_flaps = NULL;//襟翼位置
static XPLMDataRef _idc_speedBrake = NULL;//减速板
static XPLMDataRef _idc_gear = NULL;//起落架
static XPLMDataRef _idc_com1 = NULL;//COM1频率
static XPLMDataRef _idc_engine_fuelPump = NULL;//油泵
static XPLMDataRef _idc_engine_N1 = NULL;//发动机N1参数
static XPLMDataRef _idc_engine_N2 = NULL;//发动机N2参数
static XPLMDataRef _idc_engine_EGT = NULL;//发动机排气温度
static XPLMDataRef _idc_taxiLight = NULL;//滑行灯
static XPLMDataRef _idc_verticalSpeed = NULL;//垂直速度
static XPLMDataRef _idc_masterWarning = NULL;//主警告
static XPLMDataRef _idc_autopilotMode = NULL;//自动驾驶模式
static XPLMDataRef _idc_trueAirspeed = NULL;//真空速
static XPLMDataRef _idc_autoThrottle = NULL;//自动油门状态
//=============仪表显示数据结尾===============

string outputPath = "./Output/TEKConnect/TEKConnect_Query.dat";

ofstream output(outputPath, ios::trunc);

PLUGIN_API int XPluginStart(char* outName, char* outSignature, char* outDescription) {
	strcpy(outName, "TEKConnect");
	strcpy(outSignature, "peercat.connector.vatek");
	strcpy(outDescription, "Peercat plugin for VATEK to get data from simulator.(X-Plane)");

	//初始化dataref
	//===直接数据===
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
	_idc_gear = XPLMFindDataRef("sim/flightmodel2/gear/deploy_ratio");//单精度，ratio
	_idc_com1 = XPLMFindDataRef("sim/cockpit/radios/com1_freq_hz");//整型，10Hz
	_idc_engine_fuelPump = XPLMFindDataRef("sim/cockpit/engine/fuel_pump_on");//整型数组[8]，布尔运算值
	_idc_engine_N1 = XPLMFindDataRef("sim/flightmodel/engine/ENGN_N1_");//单精度数组[8]，百分比
	_idc_engine_N2 = XPLMFindDataRef("sim/flightmodel/engine/ENGN_N2_");//单精度数组[8]，百分比
	_idc_engine_EGT = XPLMFindDataRef("sim/flightmodel/engine/ENGN_EGT");//单精度数组[8]，百分比
	_idc_taxiLight = XPLMFindDataRef("sim/cockpit/electrical/taxi_light_on");//整型，布尔值
	_idc_verticalSpeed = XPLMFindDataRef("sim/cockpit2/gauges/indicators/vvi_fpm_pilot");//单精度，fpm
	_idc_masterWarning = XPLMFindDataRef("sim/cockpit/warnings/master_warning_on");//单精度，布尔值
	_idc_autopilotMode = XPLMFindDataRef("sim/cockpit2/autopilot/autopilot_on");//整型，0-off 1-FD 2-ON
	_idc_autoThrottle = XPLMFindDataRef("sim/cockpit2/autopilot/autothrottle_on");//整型，布尔值
	//注册TekLoopCallback循环事件，每0.1秒一次
	XPLMRegisterFlightLoopCallback(TEKLoopCallback, 0.1, NULL);

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
	XPLMGetDatavf(_idc_fuelOnBoard, __idc_fuelOnBoard, 0, 9);//fuel on board
	float __idc_parkingBrake = XPLMGetDataf(_idc_parkingBrake);
	float __idc_flaps = XPLMGetDataf(_idc_flaps);
	float __idc_speedBrake = XPLMGetDataf(_idc_speedBrake);
	float __idc_gear = XPLMGetDataf(_idc_gear);//gear
	int __idc_com1 = XPLMGetDatai(_idc_com1);//com1
	float __idc_engineFuelPump[8];//engine fuel pump
	XPLMGetDatavf(_idc_engine_fuelPump, __idc_engineFuelPump, 0, 8);
	float __idc_engine_N1[8];//engine n1
	XPLMGetDatavf(_idc_engine_N1, __idc_engine_N1, 0, 8);
	float __idc_engine_N2[8];//engine n2
	XPLMGetDatavf(_idc_engine_N2, __idc_engine_N2, 0, 8);
	//engine EGT
	int __idc_taxiLight = XPLMGetDatai(_idc_taxiLight);
	float __idc_verticalSpeed = XPLMGetDataf(_idc_verticalSpeed);
	float __idc_masterWarning = XPLMGetDataf(_idc_masterWarning);
	int __idc_autopilotMode = XPLMGetDatai(_idc_autopilotMode);
	int __idc_autoThrottle = XPLMGetDatai(_idc_autoThrottle);


	output.open(outputPath);
	//输出数据
	output <<
		//直接数据=================
		"\nLatitude " << __latitude <<
		"\nLongitude " << __longitude <<
		"\nElevation " << __elevation <<
		"\nAirspeed " << __airspeed <<
		"\nGroundSpeed " << __groundSpeed <<
		"\nHeading " << __heading <<
		"\nMagHeading " << __magHeading <<
		"\nPitch " << __pitch <<
		"\nRoll " << __roll <<
		"\ntotalWegiht " << __totalWeight <<
		"\nFuelWeight " << __fuelWeight <<
		"\nPayloadWeight " << __payloadWeight <<
		"\nDistanceFlown " << __distanceFlown <<
		"\nZuluTime " << __zuluTime <<
		"\nStrobeLight " << __strobeLights <<
		//仪表数据==================
		"\n_idc_Airspeed " << __idc_airspeed <<
		"\n_idc_TrueAirspeed " << __idc_trueAirspeed <<
		"\n_idc_Elevation " << __idc_elevation <<
		"\n_idc_Altimeter " << __idc_altimeter <<
		"\n_idc_LandingLight " << __idc_ldgLight <<
		"\n_idc_Pitch " << __idc_pitch <<
		"\n_idc_Heading " << __idc_heading <<
		"\n_idc_Transponder " << __idc_transponder <<
		"\n_idc_TransponderCode " << __idc_transponderCode <<
		//FuelOnBoard
		"\n_idc_ParkingBrake " << __idc_parkingBrake <<
		"\n_idc_Flaps " << __idc_flaps <<
		"\n_idc_Gear " << __idc_gear <<
		"\n_idc_COM1 " << __idc_com1 <<
		"\n_idc_VerticalSpeed " << __idc_verticalSpeed <<
		"\n_idc_MasterWarning " << __idc_masterWarning <<
		"\n_idc_AutopilotMode " << __idc_autopilotMode <<
		"\n_idc_autoThrottle " << __idc_autoThrottle;
	output.close();
	return 0.1;
}


