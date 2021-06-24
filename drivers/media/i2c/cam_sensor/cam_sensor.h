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
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_graph.h>
#include <linux/rk-camera-module.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <linux/version.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-image-sizes.h>
#include <media/v4l2-mediabus.h>

#ifndef _CAM_SENSOR_CORE_H_
#define _CAM_SENSOR_CORE_H_

enum {
	CAMERA_OV5647,
	CAMERA_IMX219,
	CAMERA_SIZE
};

int ov5647_detect(struct i2c_client *client);
int ov5647_probe(struct i2c_client *client, struct gpio_desc	*enable_gpio);
int ov5647_remove(struct i2c_client *client);

int imx219_detect(struct i2c_client *client);
int imx219_probe(struct i2c_client *client, struct gpio_desc	*enable_gpio);
int imx219_remove(struct i2c_client *client);
#endif /* _CAM_SENSOR_CORE_H_ */
