/*
 * A V4L2 driver for Asus Sensor(OV5647/IMX219) cameras.
 *
 * Based on Samsung S5K6AAFX SXGA 1/6" 1.3M CMOS Image Sensor driver
 * Copyright (C) 2011 Sylwester Nawrocki <s.nawrocki@samsung.com>
 *
 * Based on Omnivision OV7670 Camera Driver
 * Copyright (C) 2006-7 Jonathan Corbet <corbet@lwn.net>
 *
 * Based on OmniVision OV5647 Camera Driver
 * Copyright (C) 2016, Synopsys, Inc.
 *
 * Based on Sony IMX219 Camera Driver
 * Copyright (C) 2014, Andrew Chew <achew@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed .as is. WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "cam_sensor.h"

static int sCameraId = 0;

static int cam_sensor_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct gpio_desc	*enable_gpio;
	int ret;

	enable_gpio = devm_gpiod_get(dev, "enable", GPIOD_OUT_HIGH);
	if (IS_ERR(enable_gpio))
		dev_warn(dev, "Failed to get enable_gpios\n");

	ret = ov5647_detect(client);
	if (ret < 0) {
		client->addr = 0x10;
		ret = imx219_detect(client);
		if(ret < 0) return ret;
		sCameraId = CAMERA_IMX219;
	} else {
		sCameraId = CAMERA_OV5647;
	}

	if(sCameraId == CAMERA_OV5647) {
		ret = ov5647_probe(client, enable_gpio);
	} else if(sCameraId == CAMERA_IMX219) {
		ret = imx219_probe(client, enable_gpio);
	}

	return ret;
}

static int sensor_remove(struct i2c_client *client)
{
	if(sCameraId == CAMERA_OV5647) {
		ov5647_remove(client);
	} else if(sCameraId == CAMERA_IMX219) {
		imx219_remove(client);
	}
	return 0;
}

static const struct i2c_device_id sensor_id[] = {
	{"cam_sensor", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, sensor_id);

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id sensor_of_match[] = {
	{.compatible = "asus,cam_sensor"},
	{ /* sentinel */ },
};

MODULE_DEVICE_TABLE(of, sensor_of_match);
#endif

static struct i2c_driver sensor_driver = {
	.driver = {
		   .of_match_table = of_match_ptr(sensor_of_match),
		   .name = "cam_sensor",
		   },
	.probe = cam_sensor_probe,
	.remove = sensor_remove,
	.id_table = sensor_id,
};

module_i2c_driver(sensor_driver);

MODULE_AUTHOR("Xuanhao Zhang <xuanhao_zhang@asus.com>");
MODULE_DESCRIPTION("A low-level driver for Asus camera sensors");
MODULE_LICENSE("GPL v2");
