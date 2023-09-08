#include "protocol.h"
#include "config.h"
#include "debug.h"


#include "app_motion.h"
#include "app_pid.h"
#include "cJSON.h"
#include "string.h"



/* 命令接收缓存 */
uint8_t RxBuffer[PTO_MAX_BUF_LEN];
/* 接收数据下标 */
uint8_t RxIndex = 0;
/* 接收状态机 */
uint8_t RxFlag = 0;
/* 新命令接收标志 */
uint8_t New_CMD_flag;
/* 新命令数据长度 */
uint8_t New_CMD_length;

uint8_t g_Request_Flag = 0;
uint8_t g_Request_Parm = 0;


struct _sercmd {
	int cmd_no;
	int vx;
	int vy;
	int angle;
	int rot;
} sercmd;


void Request_Data(uint8_t request, uint8_t parm)
{


}

void Send_Request_Data(void)
{
	static uint16_t request_count = 0;

	switch (g_Request_Flag)
	{

	case FUNC_AUTO_REPORT:
	{
		Motion_Send_Data();
		g_Request_Flag = 0;
		break;
	}

	case FUNC_SET_MOTOR_PID:
	{
		if (g_Request_Parm > 0 && g_Request_Parm < 5)
		{
			PID_Send_Parm_Active(g_Request_Parm);
		}
		g_Request_Flag = 0;
		break;
	}

	case FUNC_SET_YAW_PID:
	{
		if (g_Request_Parm == 5)
		{
			PID_Send_Parm_Active(5);
		}
		g_Request_Flag = 0;
		break;
	}

	default:
		g_Request_Flag = 0;
		break;
	}

}


uint8_t Get_Request_Flag(void)
{
	return g_Request_Flag;
}

// 获取接收的数据
uint8_t* Get_RxBuffer(void)
{
	return (uint8_t*)RxBuffer;
}

uint8_t Get_CMD_Length(void)
{
	return New_CMD_length;
}


uint8_t Get_CMD_Flag(void)
{
	return New_CMD_flag;
}

void Set_CMD_Flag(void)
{
	New_CMD_flag = 1;
}

void Clear_CMD_Flag(void)
{
	New_CMD_flag = 0;
}


void Clear_RxBuffer(void)
{
	for (uint8_t i = 0; i < PTO_MAX_BUF_LEN; i++)
	{
		RxBuffer[i] = 0;
	}
}

#define CMD_SPEED 0
#define CMD_LED 1

void u_Data_Parse(uint8_t *data_buf, uint8_t num)
{

	char buff[200];
	memset(buff,0,sizeof(buff));
	memcpy(buff, data_buf, num);

	// json parsing.
	cJSON *ser_json = cJSON_Parse(buff);
	cJSON *name = NULL;

	if( ser_json )
	{
		name = cJSON_GetObjectItemCaseSensitive(ser_json, "vx");
		if (cJSON_IsNumber(name) )
		{
			sercmd.vx = name->valueint;
		}
		name = cJSON_GetObjectItemCaseSensitive(ser_json, "vy");
		if (cJSON_IsNumber(name) )
		{
			sercmd.vy = name->valueint;
		}
		name = cJSON_GetObjectItemCaseSensitive(ser_json, "angle");
		if (cJSON_IsNumber(name) )
		{
			sercmd.angle = name->valueint;
		}
		name = cJSON_GetObjectItemCaseSensitive(ser_json, "rot");
		if (cJSON_IsNumber(name) )
		{
			sercmd.rot = name->valueint;
		}
		name = cJSON_GetObjectItemCaseSensitive(ser_json, "cmd");
		if (cJSON_IsNumber(name) )
		{
			sercmd.cmd_no = name->valueint;
		}
		else sercmd.cmd_no = 999;
	}
	cJSON_Delete(ser_json);
	cJSON_Delete(name);

		//end of parsing

	//speed command
	if(sercmd.cmd_no == CMD_SPEED)
	{

		int16_t Vx_recv = sercmd.vx;
		int16_t Vy_recv = 0;
		int16_t Vz_recv = sercmd.angle;

		DEBUG("motion: %d, %d, %d\n", Vx_recv, Vy_recv, Vz_recv);

		if (Vx_recv == 0 && Vy_recv == 0 && Vz_recv == 0)
		{
			Motion_Stop();
		}
		else
		{
			Motion_Ctrl(Vx_recv, Vy_recv, Vz_recv);
		}
	}
	else if( sercmd.cmd_no == CMD_LED )
	{
		if(sercmd.vx == 0)
		{
			if(sercmd.vy == 0)
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
		}
		else
		{
			if(sercmd.vy == 0)
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);

		}
	}
	else if( sercmd.cmd_no == 3 )
	{
		if(sercmd.vx == 0)
			Motion_Set_Speed(sercmd.vy, 0, 0, 0);
		else if(sercmd.vx == 1)
			Motion_Set_Speed(0, sercmd.vy, 0, 0);
		else if(sercmd.vx == 2)
			Motion_Set_Speed(0, 0, sercmd.vy, 0);
		else if(sercmd.vx == 3)
			Motion_Set_Speed(0, 0, 0, sercmd.vy);
		else
			;
	}

}




