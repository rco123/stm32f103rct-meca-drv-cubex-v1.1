#include "bsp_encoder.h"
#include "bsp_motor.h"


//#include "protocol.h"
//#include "app_motion.h"

int g_Encoder_M1_Now = 0;
int g_Encoder_M2_Now = 0;
int g_Encoder_M3_Now = 0;
int g_Encoder_M4_Now = 0;


/**
 * @Brief: 10�������һ�Σ���ȡ����������
 * @Note: 
 * @Parm: �����ID��:MOTOR_ID_M1, MOTOR_ID_M2, MOTOR_ID_M3, MOTOR_ID_M4
 * @Retval: ���ر�������������
 */
static int16_t Encoder_Read_CNT(uint8_t Motor_id)
{
	int16_t Encoder_TIM = 0;
	switch(Motor_id)
	{
	case MOTOR_ID_M1:  Encoder_TIM = (short)TIM2 -> CNT; TIM2 -> CNT = 0x0000; break;
	case MOTOR_ID_M2:  Encoder_TIM = (short)TIM3 -> CNT; TIM3 -> CNT = 0x0000; break;
	case MOTOR_ID_M3:  Encoder_TIM = (short)TIM4 -> CNT; TIM4 -> CNT = 0x0000; break;
	case MOTOR_ID_M4:  Encoder_TIM = (short)TIM5 -> CNT; TIM5 -> CNT = 0x0000; break;
	default:  break;
	}
	return Encoder_TIM;
}

// ���ؿ����������ܹ�ͳ�Ƶı������ļ�������·����
int Encoder_Get_Count_Now(uint8_t Motor_id)
{
	if (Motor_id == MOTOR_ID_M1) return g_Encoder_M1_Now;
	if (Motor_id == MOTOR_ID_M2) return g_Encoder_M2_Now;
	if (Motor_id == MOTOR_ID_M3) return g_Encoder_M3_Now;
	if (Motor_id == MOTOR_ID_M4) return g_Encoder_M4_Now;
	return 0;
}

// ��ȡ�����������ܹ�����·������������
void Encoder_Get_ALL(int* Encoder_all)
{
	Encoder_all[0] = g_Encoder_M1_Now;
	Encoder_all[1] = g_Encoder_M2_Now;
	Encoder_all[2] = g_Encoder_M3_Now;
	Encoder_all[3] = g_Encoder_M4_Now;
}

// ���±������ļ�����ֵ��
void Encoder_Update_Count(void)
{
		g_Encoder_M1_Now -= Encoder_Read_CNT(MOTOR_ID_M1);

		g_Encoder_M2_Now += Encoder_Read_CNT(MOTOR_ID_M2);

		g_Encoder_M3_Now -= Encoder_Read_CNT(MOTOR_ID_M3);

		g_Encoder_M4_Now += Encoder_Read_CNT(MOTOR_ID_M4);

}
	


// ���͵�ǰ�ı��������ݵ�������
void Encoder_Send_Count_Now(void)
{
    #define LEN        21
	uint8_t data_buffer[LEN] = {0};
	uint8_t i, checknum = 0;
	//data_buffer[0] = PTO_HEAD;
	//data_buffer[1] = PTO_DEVICE_ID-1;
	data_buffer[2] = LEN-2; // ����
	//data_buffer[3] = FUNC_REPORT_ENCODER; // ����λ
	data_buffer[4] = g_Encoder_M1_Now & 0xff;
	data_buffer[5] = (g_Encoder_M1_Now >> 8) & 0xff;
	data_buffer[6] = (g_Encoder_M1_Now >> 16) & 0xff;
	data_buffer[7] = (g_Encoder_M1_Now >> 24) & 0xff;
	data_buffer[8] = g_Encoder_M2_Now & 0xff;
	data_buffer[9] = (g_Encoder_M2_Now >> 8) & 0xff;
	data_buffer[10] = (g_Encoder_M2_Now >> 16) & 0xff;
	data_buffer[11] = (g_Encoder_M2_Now >> 24) & 0xff;
	data_buffer[12] = g_Encoder_M3_Now & 0xff;
	data_buffer[13] = (g_Encoder_M3_Now >> 8) & 0xff;
	data_buffer[14] = (g_Encoder_M3_Now >> 16) & 0xff;
	data_buffer[15] = (g_Encoder_M3_Now >> 24) & 0xff;
	data_buffer[16] = g_Encoder_M4_Now & 0xff;
	data_buffer[17] = (g_Encoder_M4_Now >> 8) & 0xff;
	data_buffer[18] = (g_Encoder_M4_Now >> 16) & 0xff;
	data_buffer[19] = (g_Encoder_M4_Now >> 24) & 0xff;

	for (i = 2; i < LEN-1; i++)
	{
		checknum += data_buffer[i];
	}
	data_buffer[LEN-1] = checknum;
	//USART1_Send_ArrayU8(data_buffer, sizeof(data_buffer));
}
