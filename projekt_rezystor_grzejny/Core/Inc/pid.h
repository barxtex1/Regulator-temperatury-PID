/*
 * pid.h
 *
 *  Created on: Jan 15, 2022
 *      Author: mackop
 */

#ifndef INC_PID_H_
#define INC_PID_H_
typedef struct {
  // Regulator Gain
  float Kp;
  float Ki;
  float Kd;

  // anit wind - up 
  float Kc;
  // sampling time
  float dt;

  // Signal limits
  float limMax;
  float limMin;

  //
}
pid_parameters;

typedef struct {
  pid_parameters param;
  float setpoint;
  // memory
  float prev_error;
  float Differentiator;
  float Integrator;
  float prev_Integrator;
  float tim_counter;
  // output
  float signal_output;
}
PID_regulator;
void PID_Regulator_Init(PID_regulator * pid);
void PID_SET_SETPOINT(PID_regulator * PID, float spoint);
float PID_Limit_Signal(PID_regulator * PID);
float PID_Output_Signal_M(PID_regulator * PID, float setpoint, float measured);
float PID_Output_Signal(PID_regulator * PID, float measured);


PID_regulator pid;
#endif /* INC_PID_H_ */
