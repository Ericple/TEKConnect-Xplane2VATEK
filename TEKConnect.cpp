#include <stdio.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include "XPLMDataAccess.h"
#include "XPLMMenus.h"
#include "XPLMProcessing.h"

using namespace std;

static float TEKLoopCallback(float inElapsedSinceLastCall, float inElapsedTimeSinceLastFlightLoop, int inCounter, void* inRefcon);

//===========================================�����DataRef================================================

//=============ģ����ֱ��������ʼ==============
static XPLMDataRef _latitude = NULL;//�ɻ�γ��
static XPLMDataRef _longitude = NULL;//�ɻ�����
static XPLMDataRef _elevation = NULL;//�ɻ��߶�
static XPLMDataRef _airspeed = NULL;//�ɻ�����
static XPLMDataRef _groundSpeed = NULL;//�ɻ�����
static XPLMDataRef _heading = NULL;//�ɻ�����
static XPLMDataRef _magheading = NULL;//�ɻ��ź���
static XPLMDataRef _pitch = NULL;//�ɻ�����
static XPLMDataRef _roll = NULL;//�ɻ����
static XPLMDataRef _totalWeight = NULL;//�ɻ�����
static XPLMDataRef _fuelWeight = NULL;//ȼ������
static XPLMDataRef _payloadWeight = NULL;//��������
static XPLMDataRef _distanceFlown = NULL;//���о���
static XPLMDataRef _zuluTime = NULL;//��00��00��ʼ���������
static XPLMDataRef _strobeLights = NULL;//Ƶ����״̬
//=============ģ����ֱ�����ݽ�β==============

//=============�Ǳ���ʾ������ʼ===============
static XPLMDataRef _idc_airspeed = NULL;//ָʾ����
static XPLMDataRef _idc_elevation = NULL;//ָʾ�߶�
static XPLMDataRef _idc_altimeter = NULL;//�߶ȱ���ֵ
static XPLMDataRef _idc_ldgLight = NULL;//��½��
static XPLMDataRef _idc_pitch = NULL;//����
static XPLMDataRef _idc_roll = NULL;//���
static XPLMDataRef _idc_heading = NULL;//����
static XPLMDataRef _idc_transponder = NULL;//Ӧ���ģʽ
static XPLMDataRef _idc_transponderCode = NULL;//Ӧ�������
static XPLMDataRef _idc_fuelOnBoard = NULL;//����ȼ��
static XPLMDataRef _idc_parkingBrake = NULL;//ͣ��ɲ��
static XPLMDataRef _idc_flaps = NULL;//����λ��
static XPLMDataRef _idc_speedBrake = NULL;//���ٰ�
static XPLMDataRef _idc_gear = NULL;//�����
static XPLMDataRef _idc_com1 = NULL;//COM1Ƶ��
static XPLMDataRef _idc_engine_fuelPump = NULL;//�ͱ�
static XPLMDataRef _idc_engine_N1 = NULL;//������N1����
static XPLMDataRef _idc_engine_N2 = NULL;//������N2����
static XPLMDataRef _idc_engine_EGT = NULL;//�����������¶�
static XPLMDataRef _idc_taxiLight = NULL;//���е�
static XPLMDataRef _idc_verticalSpeed = NULL;//��ֱ�ٶ�
static XPLMDataRef _idc_masterWarning = NULL;//������
static XPLMDataRef _idc_autopilotMode = NULL;//�Զ���ʻģʽ
static XPLMDataRef _idc_trueAirspeed = NULL;//�����
static XPLMDataRef _idc_autoThrottle = NULL;//�Զ�����״̬
//=============�Ǳ���ʾ���ݽ�β===============

string outputPath = "./Output/TEKConnect/TEKConnect_Query.dat";

ofstream output(outputPath, ios::trunc);

PLUGIN_API int XPluginStart(char* outName, char* outSignature, char* outDescription) {
	strcpy(outName, "TEKConnect");
	strcpy(outSignature, "peercat.connector.vatek");
	strcpy(outDescription, "Peercat plugin for VATEK to get data from simulator.(X-Plane)");

	//��ʼ��dataref
	//===ֱ������===
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
	_idc_gear = XPLMFindDataRef("sim/flightmodel2/gear/deploy_ratio");//�����ȣ�ratio
	_idc_com1 = XPLMFindDataRef("sim/cockpit/radios/com1_freq_hz");//���ͣ�10Hz
	_idc_engine_fuelPump = XPLMFindDataRef("sim/cockpit/engine/fuel_pump_on");//��������[8]����������ֵ
	_idc_engine_N1 = XPLMFindDataRef("sim/flightmodel/engine/ENGN_N1_");//����������[8]���ٷֱ�
	_idc_engine_N2 = XPLMFindDataRef("sim/flightmodel/engine/ENGN_N2_");//����������[8]���ٷֱ�
	_idc_engine_EGT = XPLMFindDataRef("sim/flightmodel/engine/ENGN_EGT");//����������[8]���ٷֱ�
	_idc_taxiLight = XPLMFindDataRef("sim/cockpit/electrical/taxi_light_on");//���ͣ�����ֵ
	_idc_verticalSpeed = XPLMFindDataRef("sim/cockpit2/gauges/indicators/vvi_fpm_pilot");//�����ȣ�fpm
	_idc_masterWarning = XPLMFindDataRef("sim/cockpit/warnings/master_warning_on");//�����ȣ�����ֵ
	_idc_autopilotMode = XPLMFindDataRef("sim/cockpit2/autopilot/autopilot_on");//���ͣ�0-off 1-FD 2-ON
	_idc_autoThrottle = XPLMFindDataRef("sim/cockpit2/autopilot/autothrottle_on");//���ͣ�����ֵ
	//ע��TekLoopCallbackѭ���¼���ÿ0.1��һ��
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
	//�������
	output <<
		//ֱ������=================
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
		//�Ǳ�����==================
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


