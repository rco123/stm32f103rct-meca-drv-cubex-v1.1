#include "app_motion.h"
#include "app_pid.h"
//#include "app_bat.h"

//#include "app_ackerman.h"
//#include "app_flash.h"
#include "protocol.h"

#include "bsp_motor.h"
#include "bsp_encoder.h"
#include "bsp_usart.h"

#include "math.h"


// motor   1 -------- 1
//              ||
//              ||
//              ||
//         3 -------- 4
//

int g_Encoder_All_Now[MAX_MOTOR] = {0};
int g_Encoder_All_Last[MAX_MOTOR] = {0};
int g_Encoder_All_Offset[MAX_MOTOR] = {0};


car_data_t car_data;
motor_data_t motor_data;

uint8_t g_yaw_adjust = 0;

static float Motion_Get_Circle_Pulse(void)
{
    float temp = 0;
    temp = ENCODER_JGA25_370_130; // 46 * 11 * 4
    return temp;
}


void* Motion_Get_Data(uint8_t index)
{
    if (index == 1) return (int*)g_Encoder_All_Now;
    if (index == 2) return (int*)g_Encoder_All_Last;
    if (index == 3) return (int*)g_Encoder_All_Offset;
    return NULL;
}

void Motion_Get_Motor_Speed(float* speed)
{
    for (int i = 0; i < 4; i++)
    {
        speed[i] = motor_data.speed_mm_s[i];
        
    }
}


void Motion_Set_Pwm(int16_t Motor_1, int16_t Motor_2, int16_t Motor_3, int16_t Motor_4)
{
    if (Motor_1 >= -MOTOR_MAX_PULSE && Motor_1 <= MOTOR_MAX_PULSE)
    {
        Motor_Set_Pwm(MOTOR_ID_M1, Motor_1);
    }
    if (Motor_2 >= -MOTOR_MAX_PULSE && Motor_2 <= MOTOR_MAX_PULSE)
    {
        Motor_Set_Pwm(MOTOR_ID_M2, Motor_2);
    }
    if (Motor_3 >= -MOTOR_MAX_PULSE && Motor_3 <= MOTOR_MAX_PULSE)
    {
        Motor_Set_Pwm(MOTOR_ID_M3, Motor_3);
    }
    if (Motor_4 >= -MOTOR_MAX_PULSE && Motor_4 <= MOTOR_MAX_PULSE)
    {
        Motor_Set_Pwm(MOTOR_ID_M4, Motor_4);
    }
}


void Motion_Stop()
{
    Motion_Set_Speed(0, 0, 0, 0);
    PID_Clear_Motor(MAX_MOTOR);
    Motor_Stop();
}


// speed_mX=[-1000, 1000], 单位为：mm/s
void Motion_Set_Speed(float speed_m1, float speed_m2, float speed_m3, float speed_m4)
{
    motor_data.speed_set[0] = speed_m1;
    motor_data.speed_set[1] = speed_m2;
    motor_data.speed_set[2] = speed_m3;
    motor_data.speed_set[3] = speed_m4;

    for (uint8_t i = 0; i < MAX_MOTOR; i++)
    {
        PID_Set_Motor_Target(i, motor_data.speed_set[i] );
    }
}



void Motion_Get_Speed(car_data_t* car)
{
    int i = 0;
    float speed_mm[MAX_MOTOR] = {0};
    float circle_mm = Motion_Get_Circle_MM();
    float circle_pulse = ENCODER_JGA25_370_130;  // 11 * 46 * 4
    float robot_APB = Motion_Get_APB();

    Motion_Get_Encoder(); //get encode off set

    for (i = 0; i < 4; i++)
    {
		speed_mm[i] = (g_Encoder_All_Offset[i]) * 100 * circle_mm / circle_pulse;
		//  100 comes from because we use 10ms loop check
    }

	car->car_Vx = (speed_mm[0] + speed_mm[1] + speed_mm[2] + speed_mm[3]) / 4;
	car->car_Vy = -(speed_mm[0] - speed_mm[1] - speed_mm[2] + speed_mm[3]) / 4;
	car->car_Vz = -(speed_mm[0] - speed_mm[1] + speed_mm[2] - speed_mm[3]) / 4.0f / robot_APB * 1000;


	for (i = 0; i < MAX_MOTOR; i++)
	{
		motor_data.speed_mm_s[i] = speed_mm[i];  // real speed mm speed
	}
	PID_Calc_Motor(&motor_data);


}

float Motion_Get_APB(void)
{
    return MECANUM_APB;
}

float Motion_Get_Circle_MM(void)
{
    return MECANUM_CIRCLE_MM; // 204.203f
}


void Motion_Get_Encoder(void)
{

	Encoder_Get_ALL(g_Encoder_All_Now);
    for(uint8_t i = 0; i < MAX_MOTOR; i++)
    {
        g_Encoder_All_Offset[i] = g_Encoder_All_Now[i] - g_Encoder_All_Last[i];
	    g_Encoder_All_Last[i] = g_Encoder_All_Now[i];

    }

}

float speed_lr = 0;
float speed_fb = 0;
float speed_spin = 0;

float speed_L1_setup = 0;
float speed_L2_setup = 0;
float speed_R1_setup = 0;
float speed_R2_setup = 0;

// 控制小车运动
void Motion_Ctrl(int16_t V_x, int16_t V_y, int16_t V_z)
{



	// rco meca model size
//	#define ROBOT_WIDTH                  (180.0f)
//	#define ROBOT_LENGTH                 (170.0f)
//	#define MECANUM_APB                  ((ROBOT_WIDTH + ROBOT_LENGTH)/2.0f)  // 177.5
//	#define MECANUM_CIRCLE_MM            (204.203f)    // length =  2 * pi * R(65mm / 2 )
//
//	#define ENCODER_JGA25_370_130  (2024.0f) //46(ratio) * 11 * 4
//	#define ENCODER_JGA25_370_60  (4532.0f)  //103(ratio) * 11 * 4


		float robot_APB = MECANUM_APB; // 177.5 (rotation ratio)

		speed_lr = V_y;
        speed_fb = V_x;

        float rot = V_z;
        speed_spin = ( ( rot / 180.0f ) * 3.14159 ) * robot_APB;

        float theta = ( ( rot / 180.0f ) * 3.14159 );

        if (V_x == 0 && V_y == 0 && V_z == 0)
        {
            Motion_Stop();
            return;
        }
//
//        speed_L1_setup = speed_fb - speed_lr - speed_spin;
//		speed_R1_setup = speed_fb + speed_lr + speed_spin;
//		speed_L2_setup = speed_fb + speed_lr - speed_spin;
//		speed_R2_setup = speed_fb - speed_lr + speed_spin;
//
//
//    	if (speed_L1_setup > 1000) speed_L1_setup = 1000;
//    	if (speed_L1_setup < -1000) speed_L1_setup = -1000;
//    	if (speed_L2_setup > 1000) speed_L2_setup = 1000;
//    	if (speed_L2_setup < -1000) speed_L2_setup = -1000;
//    	if (speed_R1_setup > 1000) speed_R1_setup = 1000;
//    	if (speed_R1_setup < -1000) speed_R1_setup = -1000;
//    	if (speed_R2_setup > 1000) speed_R2_setup = 1000;
//    	if (speed_R2_setup < -1000) speed_R2_setup = -1000;
//
//    	float motor_1_speed = speed_L1_setup;
//    	float motor_2_speed = speed_R1_setup;
//    	float motor_3_speed = speed_L2_setup;
//    	float motor_4_speed = speed_R2_setup;


        float motor_1_speed = speed_fb * ( 1 + (1/cos(theta))*(ROBOT_WIDTH * tan(theta) / ( 2* ROBOT_LENGTH) ) );
        float motor_2_speed = speed_fb * ( 1 - (1/cos(theta))*(ROBOT_WIDTH * tan(theta) / ( 2* ROBOT_LENGTH) ) );



        float motor_3_speed = speed_fb * ( 1 + (ROBOT_WIDTH * tan(theta) / ( 2* ROBOT_LENGTH) ) );
        float motor_4_speed = speed_fb * ( 1 - (ROBOT_WIDTH * tan(theta) / ( 2* ROBOT_LENGTH) ) );



        //Motion_Set_Speed(motor_1_speed, motor_2_speed, motor_3_speed, motor_4_speed);
        //Motion_Set_Speed(motor_3_speed, motor_4_speed, motor_3_speed, motor_4_speed);
        Motion_Set_Speed(motor_1_speed, motor_2_speed, motor_3_speed, motor_4_speed);
        //Motion_Set_Speed(0, 0, 20.0, -20.0);


}





void Motion_Send_Data(void)
{
	;
}


void Motion_Handle(void)
{
    Motion_Get_Speed(&car_data);
    Motion_Set_Pwm(motor_data.speed_pwm[0], motor_data.speed_pwm[1], motor_data.speed_pwm[2], motor_data.speed_pwm[3]);
}

