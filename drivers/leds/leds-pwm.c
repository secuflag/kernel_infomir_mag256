/*
 * linux/drivers/leds-pwm.c
 *
 * simple PWM based LED control
 *
 * Copyright 2009 Luotao Fu @ Pengutronix (l.fu@pengutronix.de)
 *
 * based on leds-gpio.c by Raphael Assenat <raph@8d.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/fb.h>
#include <linux/leds.h>
#include <linux/err.h>
#include <linux/pwm.h>
#include <linux/leds_pwm.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/ctype.h>
#include <linux/of_gpio.h>

#include <asm/io.h>

struct led_pwm_data {
	struct led_classdev	cdev;
	struct pwm_device	*pwm;
	struct work_struct	work;
	unsigned int		active_low;
	unsigned int		period;
	int			duty;
	bool			can_sleep;
    bool            has_alt_color;
    unsigned long   use_alt_color;
    int             alt_pio;
};

struct led_pwm_priv {
	int num_leds;
	struct led_pwm_data leds[0];
};

static void __led_pwm_set(struct led_pwm_data *led_dat)
{
	int new_duty = led_dat->duty;

	pwm_config(led_dat->pwm, new_duty, led_dat->period);

	if (new_duty == 0)
		pwm_disable(led_dat->pwm);
	else
		pwm_enable(led_dat->pwm);
}

static void led_pwm_work(struct work_struct *work)
{
	struct led_pwm_data *led_dat =
		container_of(work, struct led_pwm_data, work);

	__led_pwm_set(led_dat);
}

static void led_pwm_set(struct led_classdev *led_cdev,
	enum led_brightness brightness)
{
	struct led_pwm_data *led_dat =
		container_of(led_cdev, struct led_pwm_data, cdev);
	unsigned int max = led_dat->cdev.max_brightness;
	unsigned int period =  led_dat->period;
	if( led_cdev->flags & LED_SUSPENDED )
	    return;
	led_dat->duty = brightness * period / max;

	if (led_dat->can_sleep)
		schedule_work(&led_dat->work);
	else
		__led_pwm_set(led_dat);
}

static inline size_t sizeof_pwm_leds_priv(int num_leds)
{
	return sizeof(struct led_pwm_priv) +
		      (sizeof(struct led_pwm_data) * num_leds);
}

static struct led_pwm_data *cdev_to_led_pwm_data(struct led_classdev *cdev)
{
        return container_of(cdev, struct led_pwm_data, cdev);
}

static struct led_pwm_data *dev_to_led_pwm_data(struct device *dev)
{
        return cdev_to_led_pwm_data(dev_get_drvdata(dev));
}

static void led_pwm_cleanup(struct led_pwm_priv *priv)
{
	while (priv->num_leds--) {
		led_classdev_unregister(&priv->leds[priv->num_leds].cdev);
		if (priv->leds[priv->num_leds].can_sleep)
			cancel_work_sync(&priv->leds[priv->num_leds].work);
        if (priv->leds[priv->num_leds].has_alt_color){
            if(priv->leds[priv->num_leds].use_alt_color){
                gpio_direction_output(priv->leds[priv->num_leds].alt_pio, 0);
            }
            gpio_free(priv->leds[priv->num_leds].alt_pio);
            priv->leds[priv->num_leds].alt_pio = -1;
        }
	}
}

static ssize_t alt_color_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
        struct led_classdev *led_cdev = dev_get_drvdata(dev);
        struct led_pwm_data *led_pwm  = cdev_to_led_pwm_data(led_cdev);

        sprintf(buf, "%lu\n", led_pwm->use_alt_color);
        return strlen(buf) + 1;
}

static ssize_t alt_color_store(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t size)
{
        struct led_classdev *led_cdev = dev_get_drvdata(dev);
        struct led_pwm_data *led_pwm  = cdev_to_led_pwm_data(led_cdev);
        int ret = -EINVAL;
        char *after;
        unsigned long use_alt = simple_strtoul(buf, &after, 10);
        size_t count = after - buf;

        if (*after && isspace(*after))
                count++;
        if (count == size) {
            if( use_alt != led_pwm->use_alt_color && led_pwm->has_alt_color)
            {
                led_pwm->use_alt_color = use_alt;
                gpio_direction_output(led_pwm->alt_pio, use_alt);
            }
            ret = count;
        }
        return ret;
}

static DEVICE_ATTR(alt_color, 644, alt_color_show, alt_color_store);

static struct led_pwm_priv *led_pwm_create_of(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct device_node *child;
	struct led_pwm_priv *priv;
	int count, ret;

	/* count LEDs in this device, so we know how much to allocate */
	count = of_get_child_count(node);
	if (!count)
		return NULL;

	priv = devm_kzalloc(&pdev->dev, sizeof_pwm_leds_priv(count),
			    GFP_KERNEL);
	if (!priv)
		return NULL;

	for_each_child_of_node(node, child) {
		struct led_pwm_data *led_dat = &priv->leds[priv->num_leds];

		led_dat->cdev.name = of_get_property(child, "label",
						     NULL) ? : child->name;

		led_dat->pwm = devm_of_pwm_get(&pdev->dev, child, NULL);
		if (IS_ERR(led_dat->pwm)) {
			dev_err(&pdev->dev, "unable to request PWM for %s\n",
				led_dat->cdev.name);
			goto err;
		}
		/* Get the period from PWM core when n*/
		led_dat->period = pwm_get_period(led_dat->pwm);

		led_dat->cdev.default_trigger = of_get_property(child,
						"linux,default-trigger", NULL);
		of_property_read_u32(child, "max-brightness",
				     &led_dat->cdev.max_brightness);

        of_property_read_u32(child, "max-brightness",
				     &led_dat->cdev.max_brightness);

		led_dat->cdev.brightness_set = led_pwm_set;
		led_dat->cdev.brightness = LED_OFF;
		led_dat->cdev.flags |= LED_CORE_SUSPENDRESUME;

		led_dat->can_sleep = pwm_can_sleep(led_dat->pwm);
		if (led_dat->can_sleep)
			INIT_WORK(&led_dat->work, led_pwm_work);

		ret = led_classdev_register(&pdev->dev, &led_dat->cdev);
		if (ret < 0) {
			dev_err(&pdev->dev, "failed to register for %s\n",
				led_dat->cdev.name);
			of_node_put(child);
			goto err;
		}

        led_dat->alt_pio = of_get_named_gpio(child, "alt-color-pio", 0);
        led_dat->has_alt_color = false;
        if (gpio_is_valid(led_dat->alt_pio)) {
            if (!gpio_request(led_dat->alt_pio, "alt-led-color-sw"))
            {
                gpio_direction_output(led_dat->alt_pio, 0);
                led_dat->has_alt_color = true;
                led_dat->use_alt_color = false;
                if(device_create_file(led_dat->cdev.dev, &dev_attr_alt_color) < 0){
                    gpio_free(led_dat->alt_pio);
                    led_dat->has_alt_color = false;
                    led_dat->use_alt_color = false;
                }
            }
        }
		priv->num_leds++;
	}

	return priv;
err:
	led_pwm_cleanup(priv);

	return NULL;
}

static int led_pwm_probe(struct platform_device *pdev)
{
	struct led_pwm_platform_data *pdata = pdev->dev.platform_data;
	struct led_pwm_priv *priv;
	int i, ret = 0;

	if (pdata && pdata->num_leds) {
		priv = devm_kzalloc(&pdev->dev,
				    sizeof_pwm_leds_priv(pdata->num_leds),
				    GFP_KERNEL);
		if (!priv)
			return -ENOMEM;

		for (i = 0; i < pdata->num_leds; i++) {
			struct led_pwm *cur_led = &pdata->leds[i];
			struct led_pwm_data *led_dat = &priv->leds[i];

			led_dat->pwm = devm_pwm_get(&pdev->dev, cur_led->name);
			if (IS_ERR(led_dat->pwm)) {
				ret = PTR_ERR(led_dat->pwm);
				dev_err(&pdev->dev,
					"unable to request PWM for %s\n",
					cur_led->name);
				goto err;
			}

			led_dat->cdev.name = cur_led->name;
			led_dat->cdev.default_trigger = cur_led->default_trigger;
			led_dat->active_low = cur_led->active_low;
			led_dat->period = cur_led->pwm_period_ns;
			led_dat->cdev.brightness_set = led_pwm_set;
			led_dat->cdev.brightness = LED_OFF;
			led_dat->cdev.max_brightness = cur_led->max_brightness;
			led_dat->cdev.flags |= LED_CORE_SUSPENDRESUME;

			led_dat->can_sleep = pwm_can_sleep(led_dat->pwm);
			if (led_dat->can_sleep)
				INIT_WORK(&led_dat->work, led_pwm_work);

			ret = led_classdev_register(&pdev->dev, &led_dat->cdev);
			if (ret < 0)
				goto err;
		}
		priv->num_leds = pdata->num_leds;
	} else {
		priv = led_pwm_create_of(pdev);
		if (!priv)
			return -ENODEV;
	}

	platform_set_drvdata(pdev, priv);

	return 0;

err:
	priv->num_leds = i;
	led_pwm_cleanup(priv);

	return ret;
}

static int led_pwm_remove(struct platform_device *pdev)
{
	struct led_pwm_priv *priv = platform_get_drvdata(pdev);

	led_pwm_cleanup(priv);

	return 0;
}

static const struct of_device_id of_pwm_leds_match[] = {
	{ .compatible = "pwm-leds", },
	{},
};
MODULE_DEVICE_TABLE(of, of_pwm_leds_match);

static struct platform_driver led_pwm_driver = {
	.probe		= led_pwm_probe,
	.remove		= led_pwm_remove,
	.driver		= {
		.name	= "leds_pwm",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(of_pwm_leds_match),
	},
};

module_platform_driver(led_pwm_driver);

MODULE_AUTHOR("Luotao Fu <l.fu@pengutronix.de>");
MODULE_DESCRIPTION("PWM LED driver for PXA");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:leds-pwm");
