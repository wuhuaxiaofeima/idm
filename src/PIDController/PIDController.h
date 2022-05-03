
#pragma once
#include<iostream>
#include <list>
#include <string>
#include "../../3rd/rapidjson/filereadstream.h"
#include "../../3rd/rapidjson/document.h"
using namespace std;

struct PIDParaSet
{
	double kp;
	double ki;
	double kd;
};

struct PIDDeadZone
{
	double dzone_speed_error;
	double dzone_steer_error;
	double dzone_steer;
	double dzone_throttle;
	double dzone_brake;
};

struct PIDMax
{
	double max_throttle;
	double max_brake;
	double max_steer;
};

struct Control
{
	double steer;
	double throttle;
	double brake;
};

class PIDController
{
public:
	// frequency means the frequency of the dynamics model,
	// to ensure precision, I set a 20Hz lower bound.
	PIDController(const std::string &filename);
	void tick(double aim_speed, double aim_steer, double veh_speed, double veh_steer);

public:
	Control control;

private:
	int _integr_update_period;
	double _time_step;
	static int _step_count;
	PIDParaSet _para_steer;
	PIDParaSet _para_throttle;
	PIDParaSet _para_brake;
	PIDDeadZone _para_dzone;
	PIDMax _para_max;
	std::list<double> _error_speed;
	std::list<double> _error_steer;
	double _integr_error_speed;
	double _integr_error_steer;

private:
	void pull_parameters(const std::string &filename);
	void integration(double &res, double val);
	double differential(const std::list<double> &vals);
	void process_speed(double aim_speed, double veh_speed);
	void process_steer(double aim_steer, double veh_steer);
};
// struct Control
// {
//     double steer;
//     double throttle;
//     double brake;
// };

// class PIDController
// {
//     public:
//         // frequency means the frequency of the dynamics model,
//         // to ensure precision, I set a 20Hz lower bound.
//         PIDController();
//         void tick(double aim_speed, double aim_steer, double veh_speed, double veh_steer);

//     public:
//         Control control;

//     private:
//         static int _step_count;
       
//         std::list<double> _error_speed;
//         std::list<double> _error_steer;
//          double kp_steer ;
//          double ki_steer ;
//          double kd_steer;
//          double deadzone_steer ;
//          double max_steer ;
//          double kp_throttle  ;
//          double ki_throt;
//          double kd_throttle ;
//          double deadzone_throttle ;
//          double max_throttle ;
//          double kp_brake ;
//          double ki_brake ;
//          double kd_brake ;
//          double deadzone_brake ;
//          double max_brake;
//          double deadzone_speed_error;
//          double deadzone_steer_error ;
//          double _integr_error_speed;
//          double _integr_error_steer ;
//          double frequency ;
//         double _time_step ;
//         double _integr_update_period ;


//     private:
//         void pull_parameters(const std::string &filename);
//         void integration(double &res, double val);
//         double differential(const std::list<double> &vals);
//         void process_speed(double aim_speed, double veh_speed);
//         void process_steer(double aim_steer, double veh_steer);
// };

