#ifndef __MPUMPU_IIC_H
#define __MPUMPU_IIC_H

#include "main.h"
#include "bsp_common.h"

#define ENABLE_I2C_PB10_PB11      1

#if ENABLE_I2C_PB10_PB11
#define MPU_SDA_IN()  	{GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=8<<12;}
#define MPU_SDA_OUT() 	{GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=3<<12;}

//IO��������	 
#define MPU_IIC_SCL    PBout(10) //SCL
#define MPU_IIC_SDA    PBout(11) //SDA	 
#define MPU_READ_SDA   PBin(11)  //����SDA 
#else
///*PB13 PB15 OK*/
//IO��������
#define MPU_SDA_IN()  	{GPIOB->CRH&=0X0FFFFFFF;GPIOB->CRH|=(u32)8<<28;}
#define MPU_SDA_OUT() 	{GPIOB->CRH&=0X0FFFFFFF;GPIOB->CRH|=(u32)3<<28;}

//IO��������	 
#define MPU_IIC_SCL    PBout(13) //SCL
#define MPU_IIC_SDA    PBout(15) //SDA	 
#define MPU_READ_SDA   PBin(15)  //����SDA 

#endif


//IIC���в�������
void MPU_IIC_Delay(void);				//MPU IIC��ʱ����
void MPU_IIC_Init(void);                //��ʼ��IIC��IO��				 
void MPU_IIC_Start(void);				//����IIC��ʼ�ź�
void MPU_IIC_Stop(void);	  			//����IICֹͣ�ź�
void MPU_IIC_Send_Byte(uint8_t txd);			//IIC����һ���ֽ�
uint8_t MPU_IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
uint8_t MPU_IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void MPU_IIC_Ack(void);					//IIC����ACK�ź�
void MPU_IIC_NAck(void);				//IIC������ACK�ź�

uint8_t MPU_Write_Byte(uint8_t devaddr,uint8_t reg,uint8_t data);
uint8_t MPU_Read_Byte(uint8_t devaddr,uint8_t reg);
uint8_t MPU_Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf);
uint8_t MPU_Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf);

#endif
















