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

//三个nst175设备在i2c总线上的地址
#define NST175_I2CADDR_DEF1 0x48
#define NST175_I2CADDR_DEF2 0x49
#define NST175_I2CADDR_DEF3 0x4a

#define nst175_reg_temp 0x00 //两个字节读取,最大高12bit有效,可选择使用9,10,11,12bit,无用bit等于0
#define nst175_reg_config 0x01 //配置寄存器 1Byte
#define nst175_reg_temp_low 0x02
#define nst175_reg_temp_high 0x03
#define nst175_reg_id 0x07			//产品id寄存器


typedef union _nst175_info
{
	rt_uint8_t byte;
	struct _id_bits
    {
        rt_uint8_t die_revision : 4 ;			//固定值为1h
        rt_uint8_t product_identification : 4 ;	//固定值为ah
    } id_bits;
		
    struct _config_bits
      {
          rt_uint8_t sd : 1 ;//shut_down mode
					//SD=1:启用关闭模式,当转换完成时设备关机
					//SD=0:连续模式,转换完成时进入下一次转换
				
					rt_uint8_t tm : 1;//Thermostat Mode nst175模式选择
					//TM=0:比较模式
					//TM=1:中断模式
          rt_uint8_t pol : 1 ;//ALERT pin输出的极性选择
					//POL=0:测量温度超过高温限制寄存器的值时,警报引脚输出0
					//POL=1:测量温度超过高温限制寄存器的值时,警报引脚输出1
				
          rt_uint8_t f1f0 : 2 ;//Fault Queue 故障队列
					//F1 F2=0 0:测量1次
					//F1 F2=0 1:连续测量2次
					//F1 F2=1 0:连续测量4次
					//F1 F2=1 1:连续测量6次
					
          rt_uint8_t r1r0 : 2 ;//温度分辨率
					//R1 R2 = 0 0:分辨率为9bit,0.5℃
          //R1 R2 = 0 1:分辨率为10bit，0.25℃
          //R1 R2 = 1 0:分辨率为11bit，0.125℃
          //R1 R2 = 1 1:分辨率为12bit，0.0625℃
					rt_uint8_t os :1;		//一次性测量模式
      } config_bits;
	

}nst175_info;


typedef union _nst175_temperature
{
    rt_int32_t int_temerature;
		float f_temerature;

}nst175_temperature;

struct nst175_device
{
    struct rt_i2c_bus_device *i2c;		//i2c总线设备
    rt_uint8_t nst175_addr ;
    rt_mutex_t lock;
    nst175_temperature temperature ;
    nst175_info id;//芯片id
    nst175_info config;//配置寄存器
};
typedef struct nst175_device *nst175_device_t;
nst175_device_t nst175_init(const char *i2c_bus_name, rt_uint8_t nst175_addr);
rt_err_t nst175_read_temperature(nst175_device_t dev);
rt_err_t nst175_read_id(nst175_device_t dev);
void nst175_deinit(nst175_device_t dev);
rt_err_t nst175_read_config(nst175_device_t dev);
rt_err_t nst175_write_config(nst175_device_t dev,rt_uint8_t data);
#endif /* _nst175_H_ */