#include "bsp_mpuiic.h"

static void Delay_For_Pin(uint8_t nCount)
{
    uint8_t i = 0;
    for(; nCount != 0; nCount--)
    {
        for (i = 0; i < 10; i++); 
    }
}

#define delay_us  Delay_For_Pin


//����IIC��ʼ�ź�
void MPU_IIC_Start(void)
{
	MPU_SDA_OUT();     //sda�����
	MPU_IIC_SDA=1;	  	  
	MPU_IIC_SCL=1;
	delay_us(4);
 	MPU_IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	MPU_IIC_SCL=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
}	  
//����IICֹͣ�ź�
void MPU_IIC_Stop(void)
{
	MPU_SDA_OUT();//sda�����
	MPU_IIC_SCL=0;
	MPU_IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	MPU_IIC_SCL=1;  
	MPU_IIC_SDA=1;//����I2C���߽����ź�
	delay_us(4);							   	
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
uint8_t MPU_IIC_Wait_Ack(void)
{
	uint8_t ucErrTime=0;
	MPU_SDA_IN();      //SDA����Ϊ����  
	MPU_IIC_SDA=1;delay_us(1);   
	MPU_IIC_SCL=1;delay_us(1);	 
	while(MPU_READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			MPU_IIC_Stop();
			return 1;
		}
	}
	MPU_IIC_SCL=0;//ʱ�����0 	   
	return 0;  
} 
//����ACKӦ��
void MPU_IIC_Ack(void)
{
	MPU_IIC_SCL=0;
	MPU_SDA_OUT();
	MPU_IIC_SDA=0;
	delay_us(2);
	MPU_IIC_SCL=1;
	delay_us(2);
	MPU_IIC_SCL=0;
}
//������ACKӦ��		    
void MPU_IIC_NAck(void)
{
	MPU_IIC_SCL=0;
	MPU_SDA_OUT();
	MPU_IIC_SDA=1;
	delay_us(2);
	MPU_IIC_SCL=1;
	delay_us(2);
	MPU_IIC_SCL=0;
}					 				     
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void MPU_IIC_Send_Byte(uint8_t txd)
{                        
    uint8_t t;
	MPU_SDA_OUT(); 	    
    MPU_IIC_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        MPU_IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(2);
		MPU_IIC_SCL=1;
		delay_us(2); 
		MPU_IIC_SCL=0;	
		delay_us(2);
    }	 
} 	    
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
uint8_t MPU_IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	MPU_SDA_IN();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	{
        MPU_IIC_SCL=0; 
        delay_us(2); 
		MPU_IIC_SCL=1;
        receive<<=1;
        if(MPU_READ_SDA)receive++;   
		delay_us(1); 
    }					 
    if (!ack)
        MPU_IIC_NAck();//����nACK
    else
        MPU_IIC_Ack(); //����ACK   
    return receive;
}


// IIC����д, ����ֵ:0,���� , ����,�������
uint8_t MPU_Write_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
	uint8_t i;
	MPU_IIC_Start();
	MPU_IIC_Send_Byte((addr << 1) | 0); //����������ַ+д����
	if (MPU_IIC_Wait_Ack())				//�ȴ�Ӧ��
	{
		MPU_IIC_Stop();
		return 1;
	}
	MPU_IIC_Send_Byte(reg); //д�Ĵ�����ַ
	MPU_IIC_Wait_Ack();		//�ȴ�Ӧ��
	for (i = 0; i < len; i++)
	{
		MPU_IIC_Send_Byte(buf[i]); //��������
		if (MPU_IIC_Wait_Ack())	   //�ȴ�ACK
		{
			MPU_IIC_Stop();
			return 1;
		}
	}
	MPU_IIC_Stop();
	return 0;
}

// IIC������, ����ֵ:0,���� , ����,�������
uint8_t MPU_Read_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
	MPU_IIC_Start();
	MPU_IIC_Send_Byte((addr << 1) | 0); //����������ַ+д����
	if (MPU_IIC_Wait_Ack())				//�ȴ�Ӧ��
	{
		MPU_IIC_Stop();
		return 1;
	}
	MPU_IIC_Send_Byte(reg); //д�Ĵ�����ַ
	MPU_IIC_Wait_Ack();		//�ȴ�Ӧ��
	MPU_IIC_Start();
	MPU_IIC_Send_Byte((addr << 1) | 1); //����������ַ+������
	MPU_IIC_Wait_Ack();					//�ȴ�Ӧ��
	while (len)
	{
		if (len == 1)
			*buf = MPU_IIC_Read_Byte(0); //������,����nACK
		else
			*buf = MPU_IIC_Read_Byte(1); //������,����ACK
		len--;
		buf++;
	}
	MPU_IIC_Stop(); //����һ��ֹͣ����
	return 0;
}

// IICдһ���ֽ�, ����ֵ:0,���� , ����,�������
uint8_t MPU_Write_Byte(uint8_t addr, uint8_t reg, uint8_t data)
{
	MPU_IIC_Start();
	MPU_IIC_Send_Byte((addr << 1) | 0); //����������ַ+д����
	if (MPU_IIC_Wait_Ack())				//�ȴ�Ӧ��
	{
		MPU_IIC_Stop();
		return 1;
	}
	MPU_IIC_Send_Byte(reg);	 //д�Ĵ�����ַ
	MPU_IIC_Wait_Ack();		 //�ȴ�Ӧ��
	MPU_IIC_Send_Byte(data); //��������
	if (MPU_IIC_Wait_Ack())	 //�ȴ�ACK
	{
		MPU_IIC_Stop();
		return 1;
	}
	MPU_IIC_Stop();
	return 0;
}

// IIC��һ���ֽ�, ����ֵ:0,���� , ����,�������
uint8_t MPU_Read_Byte(uint8_t addr, uint8_t reg)
{
	uint8_t res;
	MPU_IIC_Start();
	MPU_IIC_Send_Byte((addr << 1) | 0); //����������ַ+д����
	MPU_IIC_Wait_Ack();					//�ȴ�Ӧ��
	MPU_IIC_Send_Byte(reg);				//д�Ĵ�����ַ
	MPU_IIC_Wait_Ack();					//�ȴ�Ӧ��
	MPU_IIC_Start();
	MPU_IIC_Send_Byte((addr << 1) | 1); //����������ַ+������
	MPU_IIC_Wait_Ack();					//�ȴ�Ӧ��
	res = MPU_IIC_Read_Byte(0);			//������,����nACK
	MPU_IIC_Stop();						//����һ��ֹͣ����
	return res;
}


