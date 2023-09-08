#include "main.h"
#include <bsp_motor.h>


uint8_t motor_enable = 0;


static int16_t Motor_Ignore_Dead_Zone(int16_t pulse)
{
    if (pulse > 0) return pulse + MOTOR_IGNORE_PULSE;
    if (pulse < 0) return pulse - MOTOR_IGNORE_PULSE;
    return 0;
}



void Motor_Stop()
{
	PWM_M1 =0;
	PWM_M2 =0;
	PWM_M3 = 0;
	PWM_M4 = 0;
}


void Motor_Set_Pwm(uint8_t id, int16_t pwm)
{
    //int16_t pulse = Motor_Ignore_Dead_Zone(pwm);

	int16_t pulse = pwm;


    if (pulse >= MOTOR_MAX_PULSE)
        pulse = MOTOR_MAX_PULSE;
    if (pulse <= -MOTOR_MAX_PULSE)
        pulse = -MOTOR_MAX_PULSE;

    switch (id)
    {
    case MOTOR_ID_M1:
    {
        if (pulse >= 0)  // + value direction
        {
            PWM_M1 = pulse;   //PWM PC6
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);

        }
        else              // - value direction
        {
        	PWM_M1 = -pulse;
        	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
        }
        break;
    }

    case MOTOR_ID_M2:
    {
    	 if (pulse >= 0)  // + value direction
		{
			PWM_M2 = pulse;   //PWM PC7

			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);

		}
		else              // - value direction
		{
			PWM_M2 = -pulse;
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
		}
		break;
    }


    case MOTOR_ID_M3:
    {
    	if (pulse >= 0)  // + value direction
		{
			PWM_M3 = pulse;   //PWM PC8

			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

		}
		else              // - value direction
		{
			PWM_M3 = -pulse;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
		}
        break;
    }
    case MOTOR_ID_M4:
    {
    	if (pulse >= 0)  // + value direction
		{
			PWM_M4 = pulse;   //PWM PC9
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

		}
		else              // - value direction
		{
			PWM_M4 = -pulse;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

		}
        break;
    }

    default:
        break;
    }
}

// ��ʼ���������
void MOTOR_GPIO_Init(void)
{
    /*����һ��GPIO_InitTypeDef���͵Ľṹ��*/
    //GPIO_InitTypeDef GPIO_InitStructure;
    /* ��ʼ�����Žṹ�� */
//    gpio_t pwm[] = {
//        {M1A_PORT, M1A_PIN, M1A_CLK},
//        {M2A_PORT, M2A_PIN, M2A_CLK},
//        {M3A_PORT, M3A_PIN, M3A_CLK},
//        {M4A_PORT, M4A_PIN, M4A_CLK},
//
//        {M1B_PORT, M1B_PIN, M1B_CLK},
//        {M2B_PORT, M2B_PIN, M2B_CLK},
//        {M3B_PORT, M3B_PIN, M3B_CLK},
//        {M4B_PORT, M4B_PIN, M4B_CLK},
//    };
    
//    // ��ʼ��PWM����
//    for (int i = 0; i < MAX_MOTOR*2; i++)
//    {
//        /* PWM������� */
//        RCC_APB2PeriphClockCmd(pwm[i].clock, ENABLE);
//        GPIO_InitStructure.GPIO_Pin = pwm[i].pin;
//        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //�����������
//        GPIO_Init(pwm[i].port, &GPIO_InitStructure);
//    }

    motor_enable = MOTOR_ENABLE_A | MOTOR_ENABLE_B | MOTOR_ENABLE_C | MOTOR_ENABLE_D;
}

// �жϵ���Ƿ��ʼ�����Ƿ���1���񷵻�0
uint8_t Motor_Get_Enable_State(uint8_t enable_m)
{
    return motor_enable & enable_m;
}
