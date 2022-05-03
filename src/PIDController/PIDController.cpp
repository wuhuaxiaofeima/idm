#include "PIDController.h"
#include <stdio.h>
#include <math.h>
#include <algorithm>

int PIDController::_step_count;


PIDController::PIDController(const std::string &filename)
{
	pull_parameters(filename);
	control.brake = _para_max.max_brake;
	control.throttle = 0;
	control.steer = 0;
}
void PIDController::tick(double aim_speed, double aim_steer, double veh_speed, double veh_steer)
{
	// process_speed(aim_speed, veh_speed);
	process_steer(aim_steer, veh_steer);
	_step_count += 1;
    
}

void PIDController::pull_parameters(const std::string &filename)
{
	FILE *fp = fopen(filename.c_str(), "r");
	if (fp == 0)
	{
		printf("cannot open pid_parameters.json.");
		return;
	}
	printf("controller use pid_parameters.json.");
	char readBuffer[1024];
	rapidjson::FileReadStream is(fp, readBuffer, sizeof(readBuffer));
	rapidjson::Document d;
	d.ParseStream(is);
	int freq = d["frequency"].GetInt();
	freq = freq <= 20 ? 20 : freq;
	_time_step = 1.0 / (double)freq;
	_step_count = 0;
	_integr_error_speed = 0;
	_integr_error_steer = 0;
	_integr_update_period = d["integration_update_period"].GetInt();
	_para_brake.kp = d["kp_brake"].GetDouble();
	_para_brake.ki = d["ki_brake"].GetDouble();
	_para_brake.kd = d["kd_brake"].GetDouble();
	_para_steer.kp = d["kp_steer"].GetDouble();
	_para_steer.ki = d["ki_steer"].GetDouble();
	_para_steer.kd = d["kd_steer"].GetDouble();
	_para_throttle.kp = d["kp_throttle"].GetDouble();
	_para_throttle.ki = d["ki_throttle"].GetDouble();
	_para_throttle.kd = d["kd_throttle"].GetDouble();
	_para_dzone.dzone_brake = d["deadzone_brake"].GetDouble();
	_para_dzone.dzone_speed_error = d["deadzone_speed_error"].GetDouble();
	_para_dzone.dzone_steer_error = d["deadzone_steer_error"].GetDouble();
	_para_dzone.dzone_steer = d["deadzone_steer"].GetDouble();
	_para_dzone.dzone_throttle = d["deadzone_throttle"].GetDouble();
	_para_max.max_brake = d["max_brake"].GetDouble();
	_para_max.max_throttle = d["max_throttle"].GetDouble();
	_para_max.max_steer = d["max_steer"].GetDouble();
	fprintf(stderr, "_para_max.max_steer =%lf\n" , _para_max.max_steer);
	fclose(fp);
}

void PIDController::process_speed(double aim_speed, double veh_speed)
{    
	
	double error = aim_speed - veh_speed;
	_error_speed.push_back(error);
	while (_error_speed.size() > 3)
    
	{
		_error_speed.pop_front();
	}
	integration(_integr_error_speed, error);
	// double diff = differential(_error_speed);

	if (aim_speed == 0)
	{
		control.throttle = 0;
		control.brake = _para_max.max_brake;

		return;
	}

	if (fabs(error) > _para_dzone.dzone_speed_error)
	{
		if (error > 0)
		{
			control.throttle = _para_throttle.kp * error + _para_throttle.ki * _integr_error_speed + _para_throttle.kd * 0;
			control.throttle = control.throttle > _para_dzone.dzone_throttle ? control.throttle : 0;
			control.throttle = control.throttle < _para_max.max_throttle ? control.throttle : _para_max.max_throttle;
			control.brake = 0;
            fprintf(stderr, "throttle =%lf\n" , control.throttle);

		}
		else
		{
			control.throttle = 0;
			control.brake = -_para_brake.kp * error - _para_brake.ki * _integr_error_speed - _para_brake.kd * 0;
			control.brake = control.brake > _para_dzone.dzone_brake ? control.brake : 0;
			control.brake = control.brake < _para_max.max_brake ? control.brake : _para_max.max_brake;
		}
	}
}
void PIDController::process_steer(double aim_steer, double veh_steer)
{
	double error =atan(aim_steer - veh_steer) * 180/M_PI;
	_error_steer.push_back(error);
	while (_error_steer.size() > 3)
	{_error_steer.pop_front();}
	integration(_integr_error_steer, error);
	// double diff = differential(_error_steer);
	if (fabs(error) > _para_dzone.dzone_steer_error)
	{   control.steer = _para_steer.kp * error + _para_steer.ki * _integr_error_steer + _para_steer.kd * 0;
		control.steer = abs(control.steer) > _para_dzone.dzone_steer ? control.steer : 0;
		if (abs(control.steer) > _para_max.max_steer)
		{control.steer = control.steer > 0 ? _para_max.max_steer : -_para_max.max_steer;}
	}
}

void PIDController::integration(double &res, double val)
{
    double frequency = 60;
    double _time_step = 1.0 / (double)frequency;
    double _integr_update_period =  30;
    //  if (_step_count % static_cast<int>(_integr_update_period / _time_step) == 0)
    if (static_cast<int>(_integr_update_period / _time_step) != 0)//此代码修改，由于%0会报错，上行代码注释掉
       res = 0;
	else
    {
	// {fprintf(stderr, "d222222\n");  目前只走else！
        res += val * _time_step;
        fprintf(stderr, "res = %lf\n" , res);
    }
	
    
    
	return;
}

double PIDController::differential(const std::list<double> &vals)
{
	double result = 0;
	int len = vals.size();
	double vals_array[3];
	std::list<double>::const_iterator it = vals.begin();
	std::list<double>::const_iterator end_it = vals.end();
	for (int i = 0; it != end_it; ++it, ++i)
	{vals_array[i] = *it;}
	if (len >= 3)
	{// 3-point backwards
		result = (vals_array[len - 3] - 4 * vals_array[len - 2] + 3 * vals_array[len - 1]) / (2 * _time_step);
	}
	else if (len >= 2)
	{// 2-point backwards
		result = (vals_array[len - 1] - vals_array[len - 2]) / _time_step;
	}
	else
	{// no enough points
		result = 0;
	}
    
    
	return result;
}
