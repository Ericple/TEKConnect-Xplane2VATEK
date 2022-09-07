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

//===========================================�����DataRef================================================
static XPLMDataRef _aircraftName = NULL;
static XPLMDataRef _onGrass = NULL;//�Ƿ��ڲݵ���
static XPLMDataRef _onGround = NULL;//�Ƿ��ڵ�����
static XPLMDataRef _liveryPath = NULL;//Ϳװ·��
//=============ģ����ֱ��������ʼ==============
static XPLMDataRef _latitude = NULL;//�ɻ�γ��
static XPLMDataRef _longitude = NULL;//�ɻ�����
static XPLMDataRef _elevation = NULL;//�ɻ��߶�
static XPLMDataRef _aboveGroundLevel = NULL;//�ɻ���ظ߶�
static XPLMDataRef _vpath = NULL;
//�ɻ�����
//�ɻ�����
static XPLMDataRef _heading = NULL;//�ɻ�����
//�ɻ��ź���
//�ɻ�����
//�ɻ����
//�ɻ�����
static XPLMDataRef _fuelWeight = NULL;//ȼ������
static XPLMDataRef _payloadWeight = NULL;//��������
static XPLMDataRef _distanceFlown = NULL;//���о���
static XPLMDataRef _zuluTime = NULL;//��00��00��ʼ���������
static XPLMDataRef _strobeLights = NULL;//Ƶ����״̬
//GPWS����
//=============ģ����ֱ�����ݽ�β==============

//=============�Ǳ���ʾ������ʼ===============
static XPLMDataRef _idc_airspeed = NULL;//ָʾ����
static XPLMDataRef _idc_elevation = NULL;//ָʾ�߶�
static XPLMDataRef _idc_altimeter = NULL;//�߶ȱ���ֵ
static XPLMDataRef _idc_ldgLight = NULL;//��½��
static XPLMDataRef _idc_pitch = NULL;//����
static XPLMDataRef _idc_roll = NULL;//���
static XPLMDataRef _idc_heading = NULL;//����
//Ӧ���ģʽ
static XPLMDataRef _idc_transponderCode = NULL;//Ӧ�������
static XPLMDataRef _idc_fuelOnBoard = NULL;//����ȼ��
static XPLMDataRef _idc_parkingBrake = NULL;//ͣ��ɲ��
//����λ��
//���ٰ�
//�����
static XPLMDataRef _idc_com1 = NULL;//COM1Ƶ��
//static XPLMDataRef _idc_engine_fuelPump = NULL;//�ͱ�
static XPLMDataRef _idc_engine_N1 = NULL;//������N1����
static XPLMDataRef _idc_engine_N2 = NULL;//������N2����
static XPLMDataRef _idc_engine_EGT = NULL;//�����������¶�
static XPLMDataRef _idc_taxiLight = NULL;//���е�
static XPLMDataRef _idc_verticalSpeed = NULL;//��ֱ�ٶ�
static XPLMDataRef _idc_masterWarning = NULL;//������
//�Զ���ʻģʽ
static XPLMDataRef _idc_trueAirspeed = NULL;//�����
static XPLMDataRef _idc_autoThrottle = NULL;//�Զ�����״̬
//=============�Ǳ���ʾ���ݽ�β===============

//=============�������������ʼ===============
static XPLMDataRef _groundSpeed = NULL;// ����
static XPLMDataRef _magheading = NULL;// �ź���
static XPLMDataRef _totalWeight = NULL;// ȫ��
static XPLMDataRef _airspeed = NULL;// ����
static XPLMDataRef _pitch = NULL;// ������
// ������
static XPLMDataRef _roll = NULL;// ��ת��
// �����
//static XPLMDataRef _idc_gear = NULL;// �����λ��
static XPLMDataRef _idc_flaps = NULL;// ����λ��
// �����״̬
static XPLMDataRef _gForce = NULL;// ��ֱ����
static XPLMDataRef _GPWS = NULL;// ���ؾ���
static XPLMDataRef _idc_speedBrake = NULL;// ���ٰ�
static XPLMDataRef _reverse = NULL;// ����
static XPLMDataRef _idc_transponder = NULL;// TCAS
static XPLMDataRef _idc_autopilotMode = NULL;// �Զ���ʻ
static bool readlivery = true;
//=============����������ݽ�β===============

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
	//��ʼ��dataref
	//===ֱ������===
	_vpath = XPLMFindDataRef("sim/flightmodel/position/vpath");//������
	_GPWS = XPLMFindDataRef("sim/cockpit2/annunciators/GPWS");//����������ֵ
	_latitude = XPLMFindDataRef("sim/flightmodel/position/latitude");//˫����
	_longitude = XPLMFindDataRef("sim/flightmodel/position/longitude");//˫����
	_elevation = XPLMFindDataRef("sim/flightmodel/position/elevation");//˫���ȣ���
	_airspeed = XPLMFindDataRef("sim/flightmodel/position/true_airspeed");//�����ȣ���ÿ��
	_groundSpeed = XPLMFindDataRef("sim/flightmodel/position/groundspeed");//�����ȣ���ÿ��
	_heading = XPLMFindDataRef("sim/flightmodel/position/true_psi");//�����ȣ���
	_magheading = XPLMFindDataRef("sim/flightmodel/position/magpsi");//�����ȣ���
	_pitch = XPLMFindDataRef("sim/flightmodel/position/true_theta");//�����ȣ���
	_roll = XPLMFindDataRef("sim/flightmodel/position/true_phi");//�����ȣ���
	_totalWeight = XPLMFindDataRef("sim/flightmodel/weight/m_total");//�����ȣ�ǧ��
	_fuelWeight = XPLMFindDataRef("sim/flightmodel/weight/m_fuel_total");//�����ȣ�ǧ��
	_payloadWeight = XPLMFindDataRef("sim/flightmodel/weight/m_fixed");//�����ȣ�ǧ��
	_distanceFlown = XPLMFindDataRef("sim/flightmodel/controls/dist");//�����ȣ���
	_zuluTime = XPLMFindDataRef("sim/time/zulu_time_sec");//�����ȣ���
	_strobeLights = XPLMFindDataRef("sim/cockpit/electrical/strobe_lights_on");//���ͣ�����ֵ
	_aboveGroundLevel = XPLMFindDataRef("sim/flightmodel/position/y_agl");//�����ȣ���

	//===�Ǳ�����===
	_idc_airspeed = XPLMFindDataRef("sim/flightmodel/position/indicated_airspeed");//�����ȣ���
	_idc_trueAirspeed = XPLMFindDataRef("sim/flightmodel/position/true_airspeed");//�����ȣ���
	_idc_elevation = XPLMFindDataRef("sim/cockpit2/gauges/indicators/altitude_ft_pilot");//�����ȣ�Ӣ��
	_idc_altimeter = XPLMFindDataRef("sim/cockpit/misc/barometer_setting");//������
	_idc_ldgLight = XPLMFindDataRef("sim/cockpit2/switches/landing_lights_on");//���Σ���������ֵ
	_idc_pitch = XPLMFindDataRef("sim/cockpit/gyros/the_ind_elec_pilot_deg");//�����ȣ���
	_idc_roll = XPLMFindDataRef("sim/cockpit/gyros/phi_ind_elec_pilot_deg");//�����ȣ���
	_idc_heading = XPLMFindDataRef("sim/cockpit/gyros/psi_ind_elec_pilot_degm");//�����ȣ���
	_idc_transponder = XPLMFindDataRef("sim/cockpit2/radios/actuators/transponder_mode");//���ͣ���ͬ
	_idc_transponderCode = XPLMFindDataRef("sim/cockpit2/radios/actuators/transponder_code");
	_idc_fuelOnBoard = XPLMFindDataRef("sim/cockpit2/fuel/fuel_quantity");//����������[9]��ǧ��
	_idc_parkingBrake = XPLMFindDataRef("sim/flightmodel/controls/parkbrake");//������
	_idc_flaps = XPLMFindDataRef("sim/flightmodel2/controls/flap_handle_deploy_ratio");//�����ȣ�ratio
	_idc_speedBrake = XPLMFindDataRef("sim/flightmodel/controls/sbrkrqst");//�����ȣ�0~1.5
	//_idc_gear = XPLMFindDataRef("sim/flightmodel2/gear/deploy_ratio");//�����ȣ�ratio
	_idc_com1 = XPLMFindDataRef("sim/cockpit/radios/com1_freq_hz");//���ͣ�10Hz
	//_idc_engine_fuelPump = XPLMFindDataRef("sim/cockpit/engine/fuel_pump_on");//��������[8]����������ֵ
	_idc_engine_N1 = XPLMFindDataRef("sim/flightmodel/engine/ENGN_N1_");//����������[8]���ٷֱ�
	_idc_engine_N2 = XPLMFindDataRef("sim/flightmodel/engine/ENGN_N2_");//����������[8]���ٷֱ�
	_idc_engine_EGT = XPLMFindDataRef("sim/flightmodel/engine/ENGN_EGT");//����������[8]���ٷֱ�
	_idc_taxiLight = XPLMFindDataRef("sim/cockpit/electrical/taxi_light_on");//���ͣ�����ֵ
	_idc_verticalSpeed = XPLMFindDataRef("sim/cockpit2/gauges/indicators/vvi_fpm_pilot");//�����ȣ�fpm
	_idc_masterWarning = XPLMFindDataRef("sim/cockpit/warnings/annunciators/master_warning");//�����ȣ�����ֵ
	_idc_autopilotMode = XPLMFindDataRef("sim/cockpit2/autopilot/autopilot_on");//���ͣ�0-off 1-FD 2-ON
	_idc_autoThrottle = XPLMFindDataRef("sim/cockpit2/autopilot/autothrottle_on");//���ͣ�����ֵ
	//ע��TekLoopCallbackѭ���¼���ÿ0.5��һ��
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
		{"γ��",__latitude},
		{"����",__longitude},
		{"����",__groundSpeed},
		{"�ź���",__magHeading},
		{"ȫ��",__totalWeight},
		{"����",__airspeed},
		{"������",__pitch},
		{"��ת��",__roll},
		{"����λ��",__idc_flaps},
		{"��ֱ����",__gForce},
		{"���ٰ�",__idc_speedBrake},
		{"TCAS",__idc_transponder},
		{"�Զ���ʻ",__idc_autopilotMode}
	};*/

	//ofstream o(outputPath);

	//o <<Joutput;
	//�������
	output <<
		//ֱ������=================
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
		//�Ǳ�����==================
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


