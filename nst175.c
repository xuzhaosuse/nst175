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
*����:nst175�豸��������ʼ��										
*�������:i2c_bus_name:i2c�����豸��,nst175_addr:nst175 i2c�豸��ַ
*����ֵ:�ɹ� nst175�豸ָ��,ʧ�� ��ָ��
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
		nst175_write_config(dev,0x60);		//Ĭ������12bit�ֱ���
		
    nst175_read_config(dev);
		
		nst175_read_id(dev);
    return dev;
}

/*******************
*����:nst175�豸�ͷ�										
*�������:dev:nst175�豸ָ��
*����ֵ:��
********************/
void nst175_deinit(nst175_device_t dev)
{
    RT_ASSERT(dev);

    rt_mutex_delete(dev->lock);

    rt_free(dev);
}

/*******************
*����:nst175д�Ĵ���										
*�������:dev:nst175�豸ָ��,reg:nst175�豸��д�Ĵ�����ַ,buf:д������ͷָ��,len:д�����ݳ���
*����ֵ:RT_EOK ���� ;RT_ERROR �������
********************/
static rt_err_t nst175_write_reg(nst175_device_t dev, rt_uint8_t reg, rt_uint8_t *buf,rt_uint8_t len)
{
    struct rt_i2c_msg msgs;
		rt_uint8_t tr_buf[3] = {0};
		
		tr_buf[0] = reg;
		rt_memmove(tr_buf + 1,buf,len);

    msgs.addr = dev->nst175_addr; /*�ӻ���ַ*/
    msgs.flags = RT_I2C_WR; /*д��־ */
    msgs.buf = tr_buf; 			/*��������ָ�� */
    msgs.len = len + 1;			/*���ݳ��ȼ��ϼĴ�������һ���ֽ�*/

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
*����:nst175���Ĵ���										
*�������:dev:nst175�豸ָ��,reg:nst175�豸�ɶ��Ĵ�����ַ,buf:���ݻ���ͷָ��,len:�������ݳ���
*����ֵ:RT_EOK ���� ;RT_ERROR �������
********************/
static rt_err_t nst175_read_reg(nst175_device_t dev, rt_uint8_t reg, rt_uint8_t *buf, rt_uint8_t len)
{
    struct rt_i2c_msg msgs[2];

    msgs[0].addr = dev->nst175_addr;  /* �ӻ���ַ */
    msgs[0].flags = RT_I2C_WR;  /* д��־ */
    msgs[0].buf = &reg;  /* �ӻ��Ĵ�����ַ */
    msgs[0].len = 1; /* ���������ֽ��� */

    msgs[1].addr = dev->nst175_addr; /*�ӻ���ַ*/
    msgs[1].flags = RT_I2C_RD; /* ����־ */
    msgs[1].buf = buf;  /* ��ȡ����ָ�� */
    msgs[1].len = len; /* ��ȡ�����ֽ��� */

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
*����:nst175��оƬid										
*�������:dev:nst175�豸ָ��
*����ֵ:RT_EOK ���� ;RT_ERROR �������
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
*����:nst175�����üĴ���										
*�������:dev:nst175�豸ָ��
*����ֵ:RT_EOK ���� ;RT_ERROR �������
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
*����:nst175д���üĴ���										
*�������:dev:nst175�豸ָ��,data:д������
*����ֵ:RT_EOK ���� ;RT_ERROR �������
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


//��С�˽���
static rt_uint16_t swapendianness(rt_uint16_t value)
{
    return ((value & 0x00FF) << 8) | ((value & 0xFF00) >> 8);
}


/*******************
*����:nst175�¶ȶ�ȡ										
*�������:dev:nst175�豸ָ��,data:д������
*����ֵ:RT_EOK ���� ;RT_ERROR �������
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
						nst175_addr = (rt_uint8_t)val;//ǿ��ȡ����ֽ������޷�����
						
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
								config_val = (rt_uint8_t)val;//ǿ��ȡ����ֽ������޷�����
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






