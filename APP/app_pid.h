#ifndef __APP_PID_H__
#define __APP_PID_H__

#include "stdint.h"


//#define PID_DEF_KP      (1.5f)
//#define PID_DEF_KI      (0.08f)
//#define PID_DEF_KD      (0.5f)


//#define PID_DEF_KP      (0.8f)
//#define PID_DEF_KI      (0.06f)
//#define PID_DEF_KD      (0.5f)

#define PID_DEF_KP      (5.8f)
#define PID_DEF_KI      (0.2f)
#define PID_DEF_KD      (0.5f)



#define PID_YAW_DEF_KP  (0.4)
#define PID_YAW_DEF_KI  (0.0)
#define PID_YAW_DEF_KD  (0.1)

typedef struct _pid
{
    float target_val;               //target value
    float output_val;               //output value
    float pwm_output;        		//PWM out
    float Kp,Ki,Kd;
    float err;
    float err_last;

    float err_next;
    float integral;
} pidx_t;

typedef struct _motor_data_t
{
    float speed_mm_s[4];        // current motor speed
    float speed_pwm[4];         // calcurated pid motor pwm value
    float speed_set[4];       // setting speed value
} motor_data_t;


typedef struct
{
    float SetPoint;   // Desired value
    float Proportion; // Proportional Const
    float Integral;   // ntegral Const
    float Derivative; // Derivative Const
    float LastError;  // Error[-1]
    float PrevError;  // Error[-2]
    float SumError;   // Sums of Errors
} PID;


void PID_Param_Init(void);

float PID_Location_Calc(pidx_t *pid, float actual_val);
void PID_Calc_Motor(motor_data_t* motor);
float PID_Calc_One_Motor(uint8_t motor_id, float now_speed);
void PID_Set_Motor_Target(uint8_t motor_id, float target);
void PID_Clear_Motor(uint8_t motor_id);
void PID_Set_Motor_Parm(uint8_t motor_id, float kp, float ki, float kd);
void PID_Send_Parm_Active(uint8_t index);


void PID_Yaw_Reset(float yaw);
float PID_Yaw_Calc(float NextPoint);
void PID_Yaw_Set_Parm(float kp, float ki, float kd);

#endif /* __APP_PID_H__ */
