/*
 * pid.h
 *
 *  Created on: Jan 15, 2022
 *      Author: mackop
 */

#ifndef INC_PID_H_
#define INC_PID_H_
typedef struct{
    // Regulator Gain
    float Kp;
    float Ki;
    float Kd;

    // sampling time
    float dt;

    // Signal limits
    float limMax;
    float limMin;

    //
}pid_parameters;

typedef struct{
    pid_parameters param;
    float prev_error;
    float Differentiator;
    float Integrator;
    float prev_Integrator;
    float tim_counter;
    // output
    float signal_output;
} PID_regulator;
void PID_Regulator_Init(PID_regulator *pid);
float Limit_PID_Signal(PID_regulator *PID);
float PID_Output_Signal(PID_regulator *PID, float setpoint, float measured);
//float PID_Output_Signal(PID_regulator *PID, float setpoint, float measured);
//typedef struct{
//	float Kp;
//	float Ki;
//	float Kd;
//	float dt;
//}pid_parameters_t;
//
//typedef struct{
//	pid_parameters_t p;
//	float previous_error, previous_integral;
//}PID_regulator;
PID_regulator pid;
//float calculate_discrete_pid(PID_regulator* pid, float setpoint, float measured);
#endif /* INC_PID_H_ */
