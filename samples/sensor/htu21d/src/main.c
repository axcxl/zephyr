/*
 * Copyright (c) 2017 Linaro Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <drivers/sensor.h>
#include <stdio.h>
#include <sys/util.h>

static void process_sample(struct device *dev)
{
	static unsigned int obs;
	struct sensor_value temp, hum;
	if (sensor_sample_fetch(dev) < 0) {
		printf("Sensor sample update error\n");
		return;
	}

	if (sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp) < 0) {
		printf("Cannot read HTU21D temperature channel\n");
		return;
	}

	if (sensor_channel_get(dev, SENSOR_CHAN_HUMIDITY, &hum) < 0) {
		printf("Cannot read HTU21D humidity channel\n");
		return;
	}

	++obs;
	printf("Observation:%u\n", obs);

	/* display temperature */
	printf("Temperature:%.1f C\n", sensor_value_to_double(&temp));

	/* display humidity */
	printf("Relative Humidity:%.1f%%\n",
	       sensor_value_to_double(&hum));
}

void main(void)
{
	struct device *dev = device_get_binding("HTU21D");

	if (dev == NULL) {
		printf("Could not get HTU21D device\n");
		return;
	}

	while (1) {
		process_sample(dev);
		k_sleep(K_MSEC(2000));
	}
	k_sleep(K_FOREVER);
}
