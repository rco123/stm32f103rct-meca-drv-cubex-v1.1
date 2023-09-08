#ifndef __APP_MOTION_H__
#define __APP_MOTION_H__

#include "stdint.h"

// rco meca model size
#define ROBOT_WIDTH                  (180.0f)
#define ROBOT_LENGTH                 (170.0f)
#define MECANUM_APB                  ((ROBOT_WIDTH + ROBOT_LENGTH)/2.0f)  // 177.5
#define MECANUM_CIRCLE_MM            (204.203f)    // length =  2 * pi * R(65mm / 2 )

#define ENCODER_JGA25_370_130  (2024.0f) //46(ratio) * 11 * 4
#define ENCODER_JGA25_370_60  (4532.0f)  //103(ratio) * 11 * 4



typedef struct _car_data
{
    int16_t car_Vx;
    int16_t car_Vy;
    int16_t car_Vz;
} car_data_t;



void Motion_Stop();
void Motion_Set_Pwm(int16_t Motor_1, int16_t Motor_2, int16_t Motor_3, int16_t Motor_4);
void Motion_Ctrl(int16_t V_x, int16_t V_y, int16_t V_z);


void Motion_Get_Encoder(void);
void Motion_Set_Speed(float speed_m1, float speed_m2, float speed_m3, float speed_m4);


void Motion_Handle(void);

void Motion_Send_Data(void);
void Motion_Get_Speed(car_data_t* car);

uint8_t Motion_Get_Car_Type(void);

float Motion_Get_Circle_MM(void);
float Motion_Get_APB(void);


void Motion_Get_Motor_Speed(float* speed);

#endif
