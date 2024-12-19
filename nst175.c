/*
 * Copyright (c)  2024, suse Development Team
 *
 * SPDX-License-Identifier:  LGPLv2.1
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-12-10     xuzhao       the first version
 */
#include <string.h>
#include <rtdbg.h>
#include "nst175.h"



/*******************
*功能:nst175设备创建并初始化										
*输入参数:i2c_bus_name:i2c总线设备名,nst175_addr:nst175 i2c设备地址
*返回值:成功 nst175设备指针,失败 空指针
********************/
nst175_device_t nst175_init(const char *i2c_bus_name, rt_uint8_t nst175_addr)
{
    nst175_device_t dev;
    RT_ASSERT(i2c_bus_name);

    dev = rt_calloc(1, sizeof(struct nst175_device));
    if (dev == RT_NULL)
    {
        rt_kprintf("can't allocate memory for nst175 device on '%s' ", i2c_bus_name);
        return RT_NULL;
    }

    if (nst175_addr == NST175_I2CADDR_DEF1 || nst175_addr == NST175_I2CADDR_DEF2 || nst175_addr == NST175_I2CADDR_DEF3)
    {
        dev->nst175_addr = nst175_addr;
    }
    else
    {
        rt_kprintf("Illegal nst175 address:'%x'", nst175_addr);
        rt_free(dev);
        return RT_NULL;
    }

    dev->i2c = rt_i2c_bus_device_find(i2c_bus_name);

    if (dev->i2c == RT_NULL)
    {
        rt_kprintf("can't find nst175_addr device on '%s' ", i2c_bus_name);
        rt_free(dev);
        return RT_NULL;
    }

    dev->lock = rt_mutex_create("mutex_nst175_addr", RT_IPC_FLAG_FIFO);
    if (dev->lock == RT_NULL)
    {
        rt_kprintf("can't create mutex for nst175_addr device on '%s' ", i2c_bus_name);
        rt_free(dev);
        return RT_NULL;
    }

    // write config
		nst175_write_config(dev,0x60);		//默认配置12bit分辨率
		
    nst175_read_config(dev);
		
		nst175_read_id(dev);
    return dev;
}

/*******************
*功能:nst175设备释放										
*输入参数:dev:nst175设备指针
*返回值:无
********************/
void nst175_deinit(nst175_device_t dev)
{
    RT_ASSERT(dev);

    rt_mutex_delete(dev->lock);

    rt_free(dev);
}

/*******************
*功能:nst175写寄存器										
*输入参数:dev:nst175设备指针,reg:nst175设备可写寄存器地址,buf:写入数据头指针,len:写入数据长度
*返回值:RT_EOK 正常 ;RT_ERROR 错误代码
********************/
static rt_err_t nst175_write_reg(nst175_device_t dev, rt_uint8_t reg, rt_uint8_t *buf,rt_uint8_t len)
{
    struct rt_i2c_msg msgs;
		rt_uint8_t tr_buf[3] = {0};
		
		tr_buf[0] = reg;
		rt_memmove(tr_buf + 1,buf,len);

    msgs.addr = dev->nst175_addr; /*从机地址*/
    msgs.flags = RT_I2C_WR; /*写标志 */
    msgs.buf = tr_buf; 			/*发送数据指针 */
    msgs.len = len + 1;			/*数据长度加上寄存器的那一个字节*/

    if (rt_i2c_transfer(dev->i2c, &msgs, 1) == 1)
    {
        return RT_EOK;
    }
    else
    {
        return RT_ERROR;
    }
}

/*******************
*功能:nst175读寄存器										
*输入参数:dev:nst175设备指针,reg:nst175设备可读寄存器地址,buf:数据缓存头指针,len:读出数据长度
*返回值:RT_EOK 正常 ;RT_ERROR 错误代码
********************/
static rt_err_t nst175_read_reg(nst175_device_t dev, rt_uint8_t reg, rt_uint8_t *buf, rt_uint8_t len)
{
    struct rt_i2c_msg msgs[2];

    msgs[0].addr = dev->nst175_addr;  /* 从机地址 */
    msgs[0].flags = RT_I2C_WR;  /* 写标志 */
    msgs[0].buf = &reg;  /* 从机寄存器地址 */
    msgs[0].len = 1; /* 发送数据字节数 */

    msgs[1].addr = dev->nst175_addr; /*从机地址*/
    msgs[1].flags = RT_I2C_RD; /* 读标志 */
    msgs[1].buf = buf;  /* 读取数据指针 */
    msgs[1].len = len; /* 读取数据字节数 */

    if (rt_i2c_transfer(dev->i2c, msgs, 2) == 2)
    {
        return RT_EOK;
    }
    else
    {
        return RT_ERROR;
    }
}


/*******************
*功能:nst175读芯片id										
*输入参数:dev:nst175设备指针
*返回值:RT_EOK 正常 ;RT_ERROR 错误代码
********************/
rt_err_t nst175_read_id(nst175_device_t dev)
{
    rt_uint8_t buf[2] = { 0 };
    rt_err_t ret = RT_ERROR;

    if (nst175_read_reg(dev, nst175_reg_id, (rt_uint8_t*) &buf, 1) == RT_EOK)
    {
        dev->id.byte = buf[0];
				ret = RT_EOK;
    }
    return ret;
}

/*******************
*功能:nst175读配置寄存器										
*输入参数:dev:nst175设备指针
*返回值:RT_EOK 正常 ;RT_ERROR 错误代码
********************/
rt_err_t nst175_read_config(nst175_device_t dev)
{
    rt_uint8_t buf[2] = { 0 };
    rt_err_t ret = RT_ERROR;

    if (nst175_read_reg(dev, nst175_reg_config, (rt_uint8_t*) &buf, 1) == RT_EOK)
    {

        dev->config.byte = buf[0];
				ret = RT_EOK;

    }
    return ret;
}

/*******************
*功能:nst175写配置寄存器										
*输入参数:dev:nst175设备指针,data:写入数据
*返回值:RT_EOK 正常 ;RT_ERROR 错误代码
********************/
rt_err_t nst175_write_config(nst175_device_t dev,rt_uint8_t data)
{
    rt_uint8_t buf[1] = { 0 };
    rt_err_t ret = RT_ERROR;
		buf[0] = data;
    if (nst175_write_reg(dev, nst175_reg_config, (rt_uint8_t*) &buf, 1) == RT_EOK)
		{
			ret = RT_EOK;
		}
		
		return ret;
}


//大小端交换
static rt_uint16_t swapendianness(rt_uint16_t value)
{
    return ((value & 0x00FF) << 8) | ((value & 0xFF00) >> 8);
}


/*******************
*功能:nst175温度读取										
*输入参数:dev:nst175设备指针,data:写入数据
*返回值:RT_EOK 正常 ;RT_ERROR 错误代码
********************/
rt_err_t nst175_read_temperature(nst175_device_t dev)
{
    rt_err_t ret = RT_ERROR;
    RT_ASSERT(dev);
    rt_uint16_t temp = 0;
    rt_uint16_t temp1 = 0;
		rt_int16_t temp2 = 0;
     float temp_c = 0;
    if (nst175_read_reg(dev, nst175_reg_temp, (rt_uint8_t*) &temp1, 2) == RT_EOK)
    {
        temp=swapendianness(temp1);
       //rt_kprintf("adt74xx_read_temperature:%d res=%d\n",temp,dev->config.config_bits.res);
				if(temp <= 0x8000){
					temp_c = (temp >> 4)*0.0625f;
				}else if(temp > 0x8000){
					temp2 = (temp >> 4) - 0x1000;
					temp_c = temp2*0.0625f;
				}
        dev->temperature.f_temerature = temp_c;
        ret = RT_EOK;
    }
    return ret;
}


void nst175_sensor(int argc, char *argv[])
{
	static nst175_device_t dev = RT_NULL;
	rt_int32_t val = 0;
	rt_uint8_t nst175_addr = 0;
	rt_uint8_t config_val = 0;
	rt_uint8_t reg  = 0;
	if (argc > 2)
  {
			if (strcmp(argv[1], "init") == 0){
				if (dev == RT_NULL || strcmp(dev->i2c->parent.parent.name, argv[2]) == 0){
						if(dev){
							rt_kprintf("deinit nst175dev\r\n");
							nst175_deinit(dev);
						}
						
						val = atoi(argv[3]);
						nst175_addr = (rt_uint8_t)val;//强制取最低字节数据无符号数
						
						if(nst175_addr < NST175_I2CADDR_VPX ||  nst175_addr > NST175_I2CADDR_CPU){
							rt_kprintf("input nst175_addr is not valid,the valid addr is 72,73,74;please check input args\r\n");
							return;
						}
						
						dev = nst175_init(argv[2], nst175_addr);
						
						if(dev == RT_NULL){
							rt_kprintf("nst175 dev init failed\r\n");
						}else{
							rt_kprintf("dev.config:%x\r\n",dev->config.byte);
							rt_kprintf("dev.id:%x\r\n",dev->id.byte);
							rt_kprintf("nst175 dev init successed\r\n");
						}
					}					
			}else if(strcmp(argv[1], "read") == 0){
					
					if(dev != RT_NULL){
					 if(strcmp(argv[2], "temperature") == 0){
							if(nst175_read_temperature(dev) == RT_EOK){
								rt_kprintf("nst175_dev->temperature:%d.%d\r\n",(int)(dev->temperature.f_temerature*100)/100,(int)(dev->temperature.f_temerature*100.0)%100);
							}else{
								rt_kprintf("failed get nst175 temperature\r\n");
							}

					 }else if(strcmp(argv[2], "config") == 0){
							if(nst175_read_config(dev) == RT_EOK){
								rt_kprintf("dev.config:%x\r\n",dev->config.byte);
							}else{
								rt_kprintf("failed read nst175 config register\r\n");
							}						
					 }			
				 }else{
					rt_kprintf("please init nst175 dev first\r\n");
				 
				 }					
			}else if(strcmp(argv[1], "write") == 0){
					
					if(dev != RT_NULL){					
						if(strcmp(argv[2], "config") == 0){
								val = atoi(argv[3]);
								config_val = (rt_uint8_t)val;//强制取最低字节数据无符号数
								rt_kprintf("input config val is %x\r\n",config_val);											
								if(nst175_write_config(dev,config_val) == RT_ERROR){ //
									rt_kprintf("failed write nst175 config register\r\n");
								}
							}else{
								rt_kprintf("nst175 write format is nst175_sensor write config xxx\r\n");
							}							
					}else{
						rt_kprintf("Please init nst175 dev first\r\n");
					}				
				}
	}
	else
	{
		rt_kprintf("Instructions for use:\n");
		rt_kprintf("1.nst175_sensor init <i2c bus name> <i2caddr> -- init sensor by i2c dev name and nst175 i2caddr(decimal)\r\n");
		rt_kprintf("2.nst175_sensor read temperature/config -- read sensor nst175 temperature/config\r\n");
		rt_kprintf("3.nst175_sensor write config <config_value> -- write config register of nst175\r\n");
	}
	
}

MSH_CMD_EXPORT(nst175_sensor, nst175 sensor);






