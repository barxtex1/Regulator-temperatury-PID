/*
 * pid.c
 *
 *  Created on: Jan 15, 2022
 *      Author: mackop
 */
#include "pid.h"

//float calculate_discrete_pid(PID_regulator* pid, float setpoint, float measured){
//	float u=0, P, I, D, error, integral, derivative;
//	pid->p.Kp=0.11071524461;;
//	pid->p.Ki=0.000387312937110031;
//	pid->p.Kd=0,
//	pid->p.dt=0.1;
//	pid->previous_error=0;
//	pid->previous_integral=0;
//	error = setpoint-measured;
//
//	//proportional part
//	P = pid->p.Kp * error;
//
//	//integral part
//	integral = pid->previous_integral + (error+pid->previous_error) ; //numerical integrator without anti-windup
//	pid->previous_integral = integral;
//	I = pid->p.Ki*integral*(pid->p.dt/2.0);
//
//	//derivative part
//	derivative = (error - pid->previous_error)/pid->p.dt; //numerical derivative without filter
//	pid->previous_error = error;
//	D = pid->p.Kd*derivative;
//
//	//sum of all parts
//	u = P  + I + D; //without saturation
//
//	return u;
//}
void PID_Regulator_Init(PID_regulator *pid){
    pid->param.Kp = 0.11071524461;
    pid->param.Ki = 0.000387312937110031;
    pid->param.Kd = 1.5023432595749;
    pid->param.dt = 0.1;
    pid->param.limMax = 900;
    pid->param.limMin = 0;
    pid->tim_counter = 1000;
    pid->prev_error =0;
    pid->Differentiator =0;
}
float Limit_PID_Signal(PID_regulator *PID){
	PID->signal_output = PID->signal_output * PID->tim_counter; // pulse = pulse / ( counter + 1 )
    if (PID->signal_output > PID->param.limMax){
        PID->signal_output = PID->param.limMax;
    }
    if (PID->signal_output < PID->param.limMin) {
        PID->signal_output = PID->param.limMin;
    }
    return PID->signal_output;
}
float PID_Output_Signal(PID_regulator *PID, float setpoint, float measured){
    float error = setpoint - measured;
    float P = PID->param.Kp * error;
    PID->Integrator = PID->prev_Integrator + ( error + PID->prev_error );
    PID->prev_Integrator = PID->Integrator;
    float I = PID->param.Ki * PID->Integrator * ( PID->param.dt / 2.0 );
    PID->Differentiator = (error - PID->prev_error)/PID->param.dt;
    PID->signal_output = P + I + PID->Differentiator;
    PID->signal_output = Limit_PID_Signal(PID);
    PID->prev_error = error;
    return PID->signal_output;
}



