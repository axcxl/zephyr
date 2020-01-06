/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <drivers/i2c.h>
#include <init.h>
#include <sys/__assert.h>
#include <sys/byteorder.h>
#include <drivers/sensor.h>
#include <string.h>
#include <logging/log.h>

#include "htu21d.h"

LOG_MODULE_REGISTER(HTU21D, CONFIG_SENSOR_LOG_LEVEL);

static int htu21d_channel_get(struct device *dev,
			       enum sensor_channel chan,
			       struct sensor_value *val)
{
	struct htu21d_data *drv_data = dev->driver_data;
    float tmp;

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_AMBIENT_TEMP ||
			chan == SENSOR_CHAN_HUMIDITY);

	if (chan == SENSOR_CHAN_AMBIENT_TEMP) {
        tmp = (float)drv_data->temp_out * TEMPERATURE_COEFF_MUL / (1UL<<16) + TEMPERATURE_COEFF_ADD;
	} else { /* SENSOR_CHAN_HUMIDITY */
        tmp = (float)drv_data->humidity_out * HUMIDITY_COEFF_MUL / (1UL<<16) + HUMIDITY_COEFF_ADD;
	}

    val->val1 = (int)tmp;
    val->val2 = (int)(1000000 * (tmp - (float)val->val1));

	return 0;
}

static int htu21d_crc_check(s16_t value, u8_t crc)
{
	u32_t polynom = 0x988000; // x^8 + x^5 + x^4 + 1
	u32_t msb     = 0x800000;
	u32_t mask    = 0xFF8000;
	u32_t result  = (u32_t)value<<8; // Pad with zeros as specified in spec
	
	while( msb != 0x80 ) {
		
		// Check if msb of current value is 1 and apply XOR mask
		if( result & msb )
			result = ((result ^ polynom) & mask) | ( result & ~mask);
			
		// Shift by one
		msb >>= 1;
		mask >>= 1;
		polynom >>=1;
	}
	if( result == crc )
		return 	0;
	else
		return -EIO;
}


static int htu21d_get_measurement(struct device *dev, u8_t cmd, u8_t meas_time, s16_t *result)
{
	struct i2c_msg msg;
    u8_t res[3];
    u8_t crc;
    int ret;

    /* Send command to start measurement */
	msg.buf = &cmd;
	msg.len = 1U;
	msg.flags = I2C_MSG_WRITE;

	ret = i2c_transfer(dev, &msg, 1, DT_INST_0_MS_HTU21D_BASE_ADDRESS);
    if(ret != 0)
    {
        printk("Failed to start %d measurement\n", cmd);
        return -EIO;
    }

    /* Wait for completion, driver blocks, but bus if free to be used */
    k_sleep(K_MSEC(meas_time));

    /* Read the result (2 bytes) and the CRC (1 byte) */
    msg.buf = res;
    msg.len = 3U;
    msg.flags = I2C_MSG_RESTART | I2C_MSG_READ | I2C_MSG_STOP;
	ret = i2c_transfer(dev, &msg, 1, DT_INST_0_MS_HTU21D_BASE_ADDRESS);
    if(ret != 0)
    {
        printk("Failed to read %d measurement\n", cmd);
        return -EIO;
    }
    
    *result =  sys_le16_to_cpu((res[0] << 8) | res[1]);
    crc = res[2];

    return htu21d_crc_check(*result, crc);
}

static int htu21d_sample_fetch(struct device *dev, enum sensor_channel chan)
{
	struct htu21d_data *drv_data = dev->driver_data;
    int ret;

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL);

    /* Start temp measurement */
    ret = htu21d_get_measurement(drv_data->i2c, HTU21D_READ_TEMPERATURE, 50, &drv_data->temp_out);
    if(ret != 0)
    {
        return ret;
    }

    /* Start humidity measurement */
    ret = htu21d_get_measurement(drv_data->i2c, HTU21D_READ_HUMIDITY, 16, &drv_data->humidity_out);
    if(ret != 0)
    {
        return ret;
    }

	return 0;
}

static const struct sensor_driver_api htu21d_driver_api = {
	.sample_fetch = htu21d_sample_fetch,
	.channel_get = htu21d_channel_get,
};

int htu21d_init(struct device *dev)
{
	struct htu21d_data *drv_data = dev->driver_data;
    u8_t buf, result = 69;
    int ret;

	drv_data->i2c = device_get_binding(DT_INST_0_MS_HTU21D_BUS_NAME);
	if (drv_data->i2c == NULL) {
		LOG_ERR("Could not get pointer to %s device.",
			    DT_INST_0_MS_HTU21D_BUS_NAME);
		return -EINVAL;
	}

    printk("Wait start\n");
    /* Device needs 15ms to init */
	k_sleep(K_MSEC(15));

    printk("Wait end, doing a soft reset\n");

    /* Do a soft reset - recommended by datasheet */
    buf = HTU21D_RESET_COMMAND;
    ret = i2c_write(drv_data->i2c, &buf, 
            1, DT_INST_0_MS_HTU21D_BASE_ADDRESS);
    if(ret != 0)
    {
        printk("Failed to reset device!\n");
        return -EIO;
    }

    /* Wait until soft reset is finished */
	k_sleep(K_MSEC(15));

    printk("Soft reset end\n");

	/* check chip is present - read user register */
    buf = HTU21D_READ_USER_REG_COMMAND;
    ret = i2c_write_read(drv_data->i2c, DT_INST_0_MS_HTU21D_BASE_ADDRESS,
            &buf, 1, &result, 1);
    if(ret != 0)
    {
        printk("Failed to read user reg!");
        return -EIO;
    }

    printk("RESULT = %d\n", result);

	return 0;
}

struct htu21d_data htu21d_driver;

DEVICE_AND_API_INIT(htu21d, DT_INST_0_MS_HTU21D_LABEL, htu21d_init, &htu21d_driver,
		    NULL, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,
		    &htu21d_driver_api);
