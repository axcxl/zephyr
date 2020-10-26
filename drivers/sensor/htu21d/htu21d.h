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

/* HTU21D device commands */
#define HTU21D_RESET_COMMAND				0xFE
#define HTU21D_READ_TEMPERATURE				0xF3
#define HTU21D_READ_HUMIDITY				0xF5
#define HTU21D_WRITE_USER_REG_COMMAND		0xE6
#define HTU21D_READ_USER_REG_COMMAND		0xE7

// Coefficients for temperature computation
#define TEMPERATURE_COEFF_MUL								(175.72)
#define TEMPERATURE_COEFF_ADD								(-46.85)

// Coefficients for relative humidity computation
#define HUMIDITY_COEFF_MUL									(125)
#define HUMIDITY_COEFF_ADD									(-6)


struct htu21d_data {
	struct device *i2c;

	int16_t humidity_out;
	int16_t temp_out;
};

struct htu21d_config {
    const char *i2c_bus;
    uint16_t i2c_addr;
};

#endif /* __SENSOR_HTU21D__ */
