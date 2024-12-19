/*
 * Copyright (c)  2024, suse Development Team
 *
 * SPDX-License-Identifier:  LGPLv2.1
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-12-10     xuzhao       the first version
 */
#ifndef _nst175_H_
#define _nst175_H_

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

//����nst175�豸��i2c�����ϵĵ�ַ
#define NST175_I2CADDR_DEF1 0x48
#define NST175_I2CADDR_DEF2 0x49
#define NST175_I2CADDR_DEF3 0x4a

#define nst175_reg_temp 0x00 //�����ֽڶ�ȡ,����12bit��Ч,��ѡ��ʹ��9,10,11,12bit,����bit����0
#define nst175_reg_config 0x01 //���üĴ��� 1Byte
#define nst175_reg_temp_low 0x02
#define nst175_reg_temp_high 0x03
#define nst175_reg_id 0x07			//��Ʒid�Ĵ���


typedef union _nst175_info
{
	rt_uint8_t byte;
	struct _id_bits
    {
        rt_uint8_t die_revision : 4 ;			//�̶�ֵΪ1h
        rt_uint8_t product_identification : 4 ;	//�̶�ֵΪah
    } id_bits;
		
    struct _config_bits
      {
          rt_uint8_t sd : 1 ;//shut_down mode
					//SD=1:���ùر�ģʽ,��ת�����ʱ�豸�ػ�
					//SD=0:����ģʽ,ת�����ʱ������һ��ת��
				
					rt_uint8_t tm : 1;//Thermostat Mode nst175ģʽѡ��
					//TM=0:�Ƚ�ģʽ
					//TM=1:�ж�ģʽ
          rt_uint8_t pol : 1 ;//ALERT pin����ļ���ѡ��
					//POL=0:�����¶ȳ����������ƼĴ�����ֵʱ,�����������0
					//POL=1:�����¶ȳ����������ƼĴ�����ֵʱ,�����������1
				
          rt_uint8_t f1f0 : 2 ;//Fault Queue ���϶���
					//F1 F2=0 0:����1��
					//F1 F2=0 1:��������2��
					//F1 F2=1 0:��������4��
					//F1 F2=1 1:��������6��
					
          rt_uint8_t r1r0 : 2 ;//�¶ȷֱ���
					//R1 R2 = 0 0:�ֱ���Ϊ9bit,0.5��
          //R1 R2 = 0 1:�ֱ���Ϊ10bit��0.25��
          //R1 R2 = 1 0:�ֱ���Ϊ11bit��0.125��
          //R1 R2 = 1 1:�ֱ���Ϊ12bit��0.0625��
					rt_uint8_t os :1;		//һ���Բ���ģʽ
      } config_bits;
	

}nst175_info;


typedef union _nst175_temperature
{
    rt_int32_t int_temerature;
		float f_temerature;

}nst175_temperature;

struct nst175_device
{
    struct rt_i2c_bus_device *i2c;		//i2c�����豸
    rt_uint8_t nst175_addr ;
    rt_mutex_t lock;
    nst175_temperature temperature ;
    nst175_info id;//оƬid
    nst175_info config;//���üĴ���
};
typedef struct nst175_device *nst175_device_t;
nst175_device_t nst175_init(const char *i2c_bus_name, rt_uint8_t nst175_addr);
rt_err_t nst175_read_temperature(nst175_device_t dev);
rt_err_t nst175_read_id(nst175_device_t dev);
void nst175_deinit(nst175_device_t dev);
rt_err_t nst175_read_config(nst175_device_t dev);
rt_err_t nst175_write_config(nst175_device_t dev,rt_uint8_t data);
#endif /* _nst175_H_ */