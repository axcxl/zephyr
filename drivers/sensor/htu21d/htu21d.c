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
	s32_t conv_val;

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_AMBIENT_TEMP ||
			chan == SENSOR_CHAN_HUMIDITY);

	if (chan == SENSOR_CHAN_AMBIENT_TEMP) {
        conv_val = 0;

		/* convert temperature x8 to degrees Celsius */
		val->val1 = conv_val / 8;
		val->val2 = (conv_val % 8) * (1000000 / 8);
	} else { /* SENSOR_CHAN_HUMIDITY */
        conv_val = 0;

		/* convert humidity x2 to percent */
		val->val1 = conv_val / 2;
		val->val2 = (conv_val % 2) * 500000;
	}

	return 0;
}

static int htu21d_sample_fetch(struct device *dev, enum sensor_channel chan)
{
	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL);

	return 0;
}

static const struct sensor_driver_api htu21d_driver_api = {
	.sample_fetch = htu21d_sample_fetch,
	.channel_get = htu21d_channel_get,
};

int htu21d_init(struct device *dev)
{
	struct htu21d_data *drv_data = dev->driver_data;

	drv_data->i2c = device_get_binding(DT_INST_0_MS_HTU21D_BUS_NAME);
	if (drv_data->i2c == NULL) {
		LOG_ERR("Could not get pointer to %s device.",
			    DT_INST_0_MS_HTU21D_BUS_NAME);
		return -EINVAL;
	}

	return 0;
}

struct htu21d_data htu21d_driver;

DEVICE_AND_API_INIT(htu21d, DT_INST_0_MS_HTU21D_LABEL, htu21d_init, &htu21d_driver,
		    NULL, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,
		    &htu21d_driver_api);
