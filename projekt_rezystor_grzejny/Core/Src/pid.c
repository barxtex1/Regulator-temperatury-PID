/*
 * pid.c
 *
 *  Created on: Jan 15, 2022
 *      Author: mackop
 */
#include "pid.h"

void PID_Regulator_Init(PID_regulator * pid) {
  pid -> param.Kp = 0.11071524461; // K gain
  pid -> param.Ki = 0.000387312937110031; // I gain 
  pid -> param.Kd = 1.5023432595749; // D gain 
  pid -> param.Kc = 0.8 ;// antiwindup corrector
  pid -> param.dt = 0.1; // sample time
  pid -> param.limMax = 900; // saturation max 
  pid -> param.limMin = 0; // saturation min 
  pid -> tim_counter = 1000; // counter period to set pulse
  pid -> prev_error = 0;
  pid -> Differentiator = 0;
}

// saturation block 
float PID_Limit_Signal(PID_regulator * PID) {
  PID -> signal_output = PID -> signal_output; // pulse = pulse / ( counter + 1 )
  if (PID -> signal_output > PID -> param.limMax) {
    PID -> signal_output = PID -> param.limMax;
  }
  if (PID -> signal_output < PID -> param.limMin) {
    PID -> signal_output = PID -> param.limMin;
  }
  return PID -> signal_output;
}
// PID signal without memory of setpoint 
float PID_Output_Signal_M(PID_regulator * PID, float setpoint, float measured) {
  float error = setpoint - measured;
  float u_pid;
  // calc P * e
  float P = PID -> param.Kp * error;

  // calc KI * d/dt e 
  PID -> Integrator = PID -> prev_Integrator + (error + PID -> prev_error);
  float I = PID -> param.Ki * PID -> Integrator * (PID -> param.dt / 2.0);

  // calc D * sum(e)
  PID -> Differentiator = (error - PID -> prev_error) / PID -> param.dt;

  // calc PID
  PID -> signal_output =( P + I + PID -> Differentiator)* PID -> tim_counter;
  u_pid = PID -> signal_output;

  // saturation
  PID -> signal_output = PID_Limit_Signal(PID);

  // anti wind up
  if (u_pid != PID -> signal_output) {
	  I = ( PID -> param.Ki )/( PID -> param.Kc ) * PID -> Integrator * (PID -> param.dt / 2.0);
	  PID -> signal_output = (P + I + PID -> Differentiator)* PID -> tim_counter;
  }

  // PID memory 
  PID -> prev_error = error;
  PID -> prev_Integrator = PID -> Integrator;
  return PID -> signal_output;
}
// PID signal with setpoint 
float PID_Output_Signal(PID_regulator * PID, float measured) {
	if (PID -> setpoint){
	  float error = PID -> setpoint - measured;
	  float u_pid;
	  // calc P * e
	  float P = PID -> param.Kp * error;

	  // calc KI * d/dt e
	  PID -> Integrator = PID -> prev_Integrator + (error + PID -> prev_error);
	  float I = PID -> param.Ki * PID -> Integrator * (PID -> param.dt / 2.0);

	  // calc D * sum(e)
	  PID -> Differentiator = (error - PID -> prev_error) / PID -> param.dt;

	  // calc PID
	  PID -> signal_output =( P + I + PID -> Differentiator)* PID -> tim_counter;
	  u_pid = PID -> signal_output;

	  // saturation
	  PID -> signal_output = PID_Limit_Signal(PID);

	  // anti wind up
	  if (u_pid != PID -> signal_output) {
		  I = ( PID -> param.Ki )/( PID -> param.Kc ) * PID -> Integrator * (PID -> param.dt / 2.0);
		  PID -> signal_output = (P + I + PID -> Differentiator)* PID -> tim_counter;
	  }

	  // PID memory
	  PID -> prev_error = error;
	  PID -> prev_Integrator = PID -> Integrator;
	}
	return PID -> signal_output;
}
// setting setpoint via UART
void PID_SET_SETPOINT(PID_regulator * PID, float spoint) {
  if (spoint < 21.3 && spoint > 71.1) {
    PID -> setpoint = 32.0;
  } else {
    PID -> setpoint = spoint;
  }
}
