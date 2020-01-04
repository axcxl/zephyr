/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_HTU21D_HTU21D_H_
#define ZEPHYR_DRIVERS_SENSOR_HTU21D_HTU21D_H_

#include <device.h>
#include <sys/util.h>
#include <zephyr/types.h>
#include <drivers/gpio.h>

struct htu21d_data {
	struct device *i2c;

	s16_t humidity_out;
	s16_t temp_out;
};

#endif /* __SENSOR_HTU21D__ */
