/*
 * LED Kernel Blinker Trigger
 *
 * Copyright 2005-2006 Openedhand Ltd.
 *
 * Author: Richard Purdie <rpurdie@openedhand.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/device.h>
//#include <linux/sysdev.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/ctype.h>
#include <linux/leds.h>
#include "../leds.h"

#define LED_MODE_OFF 			0
#define LED_MODE_ON  			1
#define LED_MODE_BLINK 			2
#define LED_MODE_REVERSE_BLINK	3

struct blinker_trig_data {
	unsigned long delay_on;		/* milliseconds on */
	unsigned long delay_off;	/* milliseconds off */
	unsigned long cnt;
	unsigned long state;
	unsigned long mode;
	unsigned long level;
	unsigned long level_off;
	struct timer_list timer;
};

static void led_blinker_function(unsigned long data)
{
	struct led_classdev *led_cdev = (struct led_classdev *) data;
	struct blinker_trig_data *timer_data = led_cdev->trigger_data;
	unsigned long brightness = timer_data->level_off;
	unsigned long delay = timer_data->delay_off;
	int mode = timer_data->mode;
	int cnt = timer_data->cnt;

	if(mode == LED_MODE_OFF) {
	  led_set_brightness(led_cdev, timer_data->level_off);
	}else if(mode == LED_MODE_ON) {
	  led_set_brightness(led_cdev, timer_data->level);
	}else {
	  if(mode == LED_MODE_BLINK)
	  {
		if(cnt&1)
		{//next on state
		  delay = timer_data->delay_on;
		  brightness = timer_data->level;
		}else
		{//next off state
		  delay = timer_data->delay_off;
		  brightness = timer_data->level_off;
		}
	  }else { //LED_MODE_REVERSE_BLINK 
		if(cnt&1)
		{//next off state
		  delay = timer_data->delay_off;
		  brightness = timer_data->level_off;
		}else
		{//next on state
		  delay = timer_data->delay_on;
		  brightness = timer_data->level;
		}
	  }
	  led_set_brightness(led_cdev,brightness);
	  if(delay < 10)
		delay = 10;
	  if(cnt > 0)
		{
		  mod_timer(&timer_data->timer, jiffies + msecs_to_jiffies(delay));
		  timer_data->cnt = cnt-1;
		}
	}
}

static ssize_t led_delay_on_show(struct device *dev, 
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct blinker_trig_data *timer_data = led_cdev->trigger_data;

	sprintf(buf, "%lu\n", timer_data->delay_on);

	return strlen(buf) + 1;
}

static ssize_t led_delay_on_store(struct device *dev, 
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct blinker_trig_data *timer_data = led_cdev->trigger_data;
	int ret = -EINVAL;
	char *after;
	unsigned long state = simple_strtoul(buf, &after, 10);
	size_t count = after - buf;

	if (*after && isspace(*after))
		count++;

	if (count == size) {
		timer_data->delay_on = state;
//		mod_timer(&timer_data->timer, jiffies + 1);
		ret = count;
	}

	return ret;
}

static ssize_t led_delay_off_show(struct device *dev, 
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct blinker_trig_data *timer_data = led_cdev->trigger_data;

	sprintf(buf, "%lu\n", timer_data->delay_off);

	return strlen(buf) + 1;
}

static ssize_t led_delay_off_store(struct device *dev, 
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct blinker_trig_data *timer_data = led_cdev->trigger_data;
	int ret = -EINVAL;
	char *after;
	unsigned long state = simple_strtoul(buf, &after, 10);
	size_t count = after - buf;

	if (*after && isspace(*after))
		count++;

	if (count == size) {
		timer_data->delay_off = state;
		ret = count;
	}

	return ret;
}
static ssize_t led_cnt_show(struct device *dev, 
		struct device_attribute *attr, char *buf)
{
	sprintf(buf, "%lu\n", 0L);

	return strlen(buf) + 1;
}

static ssize_t led_cnt_store(struct device *dev, 
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct blinker_trig_data *timer_data = led_cdev->trigger_data;
	int ret = -EINVAL;
	char *after;
	unsigned long cnt = simple_strtoul(buf, &after, 10);
	size_t count = after - buf;

	if (*after && isspace(*after))
		count++;

	if (count == size) {
		cnt = cnt<<1;
		if(cnt > 0)
		  cnt = cnt - 1;
		timer_data->cnt = cnt;
		mod_timer(&timer_data->timer, jiffies + 1);
		ret = count;
	}
	return ret;
}
static ssize_t led_mode_show(struct device *dev, 
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct blinker_trig_data *timer_data = led_cdev->trigger_data;

	sprintf(buf, "%lu\n", timer_data->mode);
	return strlen(buf) + 1;
}

static ssize_t led_mode_store(struct device *dev, 
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct blinker_trig_data *timer_data = led_cdev->trigger_data;
	int ret = -EINVAL;
	char *after;
	unsigned long mode = simple_strtoul(buf, &after, 10);
	size_t count = after - buf;

	if (*after && isspace(*after))
		count++;

	if (count == size) {
		timer_data->mode = mode;
		timer_data->cnt = 0;
//		if(mode ==  || mode == 1)
//		  led_set_brightness(led_cdev, LED_OFF);
//		else
//		  led_set_brightness(led_cdev, LED_FULL);
		mod_timer(&timer_data->timer, jiffies + 1);
		ret = count;
	}
	return ret;
}
static ssize_t led_level_show(struct device *dev, 
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct blinker_trig_data *timer_data = led_cdev->trigger_data;

	sprintf(buf, "%lu\n", timer_data->level);
	return strlen(buf) + 1;
}

static ssize_t led_level_store(struct device *dev, 
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct blinker_trig_data *timer_data = led_cdev->trigger_data;
	int ret = -EINVAL;
	char *after;
	unsigned long level = simple_strtoul(buf, &after, 10);
	size_t count = after - buf;

	if (*after && isspace(*after))
		count++;

	if (count == size) {
		if(level > LED_FULL)
		  level = LED_FULL;
		timer_data->level = level;
		mod_timer(&timer_data->timer, jiffies + 1);
		ret = count;
	}
	return ret;
}
static ssize_t led_level_off_show(struct device *dev, 
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct blinker_trig_data *timer_data = led_cdev->trigger_data;

	sprintf(buf, "%lu\n", timer_data->level_off);
	return strlen(buf) + 1;
}

static ssize_t led_level_off_store(struct device *dev, 
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct blinker_trig_data *timer_data = led_cdev->trigger_data;
	int ret = -EINVAL;
	char *after;
	unsigned long level = simple_strtoul(buf, &after, 10);
	size_t count = after - buf;

	if (*after && isspace(*after))
		count++;

	if (count == size) {
		if(level > LED_FULL)
		  level = LED_FULL;
		timer_data->level_off = level;
		mod_timer(&timer_data->timer, jiffies + 1);
		ret = count;
	}
	return ret;
}
static DEVICE_ATTR(delay_on, 0644, led_delay_on_show, led_delay_on_store);
static DEVICE_ATTR(delay_off, 0644, led_delay_off_show, led_delay_off_store);
static DEVICE_ATTR(cnt, 0644, led_cnt_show, led_cnt_store);
static DEVICE_ATTR(mode, 0644, led_mode_show, led_mode_store);
static DEVICE_ATTR(level, 0644, led_level_show, led_level_store);
static DEVICE_ATTR(level_off, 0644, led_level_off_show, led_level_off_store);

static void blinker_trig_activate(struct led_classdev *led_cdev)
{
	struct blinker_trig_data *timer_data;
	int rc;

	timer_data = kzalloc(sizeof(struct blinker_trig_data), GFP_KERNEL);
	if (!timer_data)
		return;

	led_cdev->trigger_data = timer_data;

	init_timer(&timer_data->timer);
	timer_data->timer.function = led_blinker_function;
	timer_data->timer.data = (unsigned long) led_cdev;
	timer_data->cnt = 0;
	timer_data->mode = LED_MODE_ON;
	timer_data->state = 0;
	timer_data->delay_off = 50;
	timer_data->delay_on = 50;
	timer_data->level = LED_FULL;
	timer_data->level_off = LED_OFF;
	rc = device_create_file(led_cdev->dev, &dev_attr_delay_on);
	if (rc)
		goto err_out;
	rc = device_create_file(led_cdev->dev, &dev_attr_delay_off);
	if (rc)
		goto err_out_delayon;
	rc = device_create_file(led_cdev->dev, &dev_attr_cnt);
	if (rc)
		goto err_out_cnt;
	rc = device_create_file(led_cdev->dev, &dev_attr_mode);
	if (rc)
		goto err_out_mode;
	rc = device_create_file(led_cdev->dev, &dev_attr_level);
	if (rc)
		goto err_out_level;
	rc = device_create_file(led_cdev->dev, &dev_attr_level_off);
	if (rc)
		goto err_out_level_off;

//	led_set_brightness(led_cdev, LED_FULL);
	mod_timer(&timer_data->timer, jiffies + 1);
	return;

err_out_level_off:
	device_remove_file(led_cdev->dev, &dev_attr_level);
err_out_level:
	device_remove_file(led_cdev->dev, &dev_attr_mode);
err_out_mode:
	device_remove_file(led_cdev->dev, &dev_attr_cnt);
err_out_cnt:
	device_remove_file(led_cdev->dev, &dev_attr_delay_off);
err_out_delayon:
	device_remove_file(led_cdev->dev, &dev_attr_delay_on);
err_out:
	led_cdev->trigger_data = NULL;
	kfree(timer_data);
}

static void blinker_trig_deactivate(struct led_classdev *led_cdev)
{
	struct blinker_trig_data *timer_data = led_cdev->trigger_data;

	if (timer_data) {
		device_remove_file(led_cdev->dev, &dev_attr_delay_on);
		device_remove_file(led_cdev->dev, &dev_attr_delay_off);
		device_remove_file(led_cdev->dev, &dev_attr_cnt);
		device_remove_file(led_cdev->dev, &dev_attr_mode);
		device_remove_file(led_cdev->dev, &dev_attr_level);
		device_remove_file(led_cdev->dev, &dev_attr_level_off);
		del_timer_sync(&timer_data->timer);
		kfree(timer_data);
	}
}

static struct led_trigger blinker_led_trigger = {
	.name     = "blinker",
	.activate = blinker_trig_activate,
	.deactivate = blinker_trig_deactivate,
};

static int __init blinker_trig_init(void)
{
	return led_trigger_register(&blinker_led_trigger);
}

static void __exit blinker_trig_exit(void)
{
	led_trigger_unregister(&blinker_led_trigger);
}

module_init(blinker_trig_init);
module_exit(blinker_trig_exit);

MODULE_AUTHOR("Dmitry Grib");
MODULE_DESCRIPTION("Timer Cnt LED trigger");
MODULE_LICENSE("GPL");
