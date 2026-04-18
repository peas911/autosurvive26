#ifndef APP_CHASSIS_H
#define APP_CHASSIS_H

#include "stm32f4xx_hal.h"

// M2006 Motor Struct
typedef struct {
    uint16_t can_id;
    int16_t set_current;
    
    int16_t rotor_angle;
    int16_t rotor_speed; // rpm
    int16_t torque_current;
    uint8_t temp;
} motor_measure_t;

// Speed PID control struct
typedef struct {
    float Kp, Ki, Kd;
    float max_out, max_iout;
    float set, fdb;
    float out, Pout, Iout, Dout;
    float Dbuf[3];
    float error[3];
} pid_type_def;

void Chassis_Task(void * argument);
void Motor_PID_Init(void);
void Chassis_Kinematics(int16_t vx, int16_t vy, int16_t vw, int16_t wheel_speed[4]);
void CAN_Send_Drive_Currents(int16_t current1, int16_t current2, int16_t current3, int16_t current4);

#endif