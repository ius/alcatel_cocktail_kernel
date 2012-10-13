/* 
 *
 * pmic8058 led notification driver
 *
 * Copyright (C) 2008-2020 ThunderSoft Technology Co.,Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/pwm.h>
#ifdef CONFIG_PMIC8058_PWM
#include <linux/leds-pmic8058.h>
#include <linux/pmic8058-pwm.h>
#endif
#include <linux/err.h>
#include <linux/mfd/pmic8058.h>
#include <linux/debugfs.h>
#include <linux/slab.h>
#include <linux/leds.h>
#include <linux/sysfs.h>
#include <linux/ctype.h>


/* PMIC8058 PIN */
#define LED_DRV0_N		4                       /* PMIC LED_DRV0_N */
#define LED_NOTIFICATION_NAME 	"led_notification"


/* set the duty time and period time */ 
#define PWM_DUTY_ON     7
#define PWM_PERIOD_ON   7

#define PWM_DUTY_OFF    0
#define PWM_PERIOD_OFF  7

#define PWM_DUTY_FAST   50000
#define PWM_PERIOD_FAST 800000

#define PWM_DUTY_SLOW	70000
#define PWM_PERIOD_SLOW 2000000 

/*
 * for breathing mode,
 * read the duty and period value 
 * will always be zero
 */

#define PWM_DUTY_BREATH		0
#define PWM_PERIOD_BREATH	0

/* for breathing light */
#define BREATH_PERIOD_US     80
#define DUTY_TIME_MS         80
#define START_IDX            0
#define IDX_LEN              64
#define PAUSE_LO             1000
#define PAUSE_HI             1000


#define ON_MODE  	0
#define OFF_MODE        1
#define FAST_MODE       2
#define SLOW_MODE       3
#define BREATH_MODE     4

struct led_notification_struct {
	struct led_classdev led_notification_dev;
	spinlock_t spin_lock;
	unsigned int mode;
	unsigned int duty;
	unsigned int period;
};

static struct led_notification_struct led_notif_info;
static struct pwm_device *led_notif;


static int duty_pct[] = { 
	1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
	11, 12, 13, 14, 15, 16, 23, 28, 31, 42,
  	47, 56, 63, 83, 94, 111, 125, 167, 188, 
  	250, 375, 500, 667, 500, 375, 250,
  	188, 167, 125, 111, 94, 83, 63, 56, 47, 42,
        31, 28, 23, 16, 15, 14, 13, 12, 11, 10, 
        9, 8, 7, 6, 5, 4, 3, 2, 1,
};


static int led_notification_control(int mode)
{
	int ret = 0;
	unsigned long flags;

	switch (mode) {
	case ON_MODE:
		pr_info("%s: led notification on\n", __func__);
		ret = pwm_config(led_notif, PWM_DUTY_ON, PWM_PERIOD_ON);
        	if (ret) {
              		pr_err("%s: duty=%d, period=%d\n", __func__, PWM_DUTY_ON, PWM_PERIOD_ON);
              		pr_err("%s: pwm_config on pwm failed %d\n", __func__, ret);
              		return ret;
        	}
		ret = pwm_enable(led_notif);
		if (ret) {
              		pr_err("%s: pwm_enable on pwm failed %d\n", __func__, ret);
              		return ret;
        	}
		spin_lock_irqsave(&led_notif_info.spin_lock, flags);
		led_notif_info.duty =  PWM_DUTY_ON;
		led_notif_info.period = PWM_PERIOD_ON;
		spin_unlock_irqrestore(&led_notif_info.spin_lock, flags);
		break;
		
	case OFF_MODE:
		pr_info("%s: led notification off\n", __func__);
		ret = pwm_config(led_notif, PWM_DUTY_OFF, PWM_PERIOD_OFF);
        	if (ret) {
             		pr_err("%s: duty=%d, period=%d\n", __func__, PWM_DUTY_OFF, PWM_PERIOD_OFF);
              		pr_err("%s: pwm_config on pwm failed %d\n", __func__, ret);
              		return ret;
        	}
		pwm_disable(led_notif);
		spin_lock_irqsave(&led_notif_info.spin_lock, flags);
		led_notif_info.duty = PWM_DUTY_OFF;
		led_notif_info.period = PWM_PERIOD_OFF;
		spin_unlock_irqrestore(&led_notif_info.spin_lock, flags);
		break;
		
	case FAST_MODE:
		pr_info("%s: led notification fast mode\n", __func__);
		ret = pwm_config(led_notif, PWM_DUTY_FAST, PWM_PERIOD_FAST);
        	if (ret) {
              		pr_err("%s: duty=%d, period=%d\n", __func__, PWM_DUTY_FAST, PWM_PERIOD_FAST);
              		pr_err("%s: pwm_config on pwm failed %d\n", __func__, ret);
              		return ret;
        	}
		pwm_enable(led_notif);
		spin_lock_irqsave(&led_notif_info.spin_lock, flags);
		led_notif_info.duty = PWM_DUTY_FAST;
		led_notif_info.period = PWM_PERIOD_FAST;
		spin_unlock_irqrestore(&led_notif_info.spin_lock, flags);	
		break;
	case SLOW_MODE:
		
		pr_info("%s: led notification slow mode\n", __func__);
		ret = pwm_config(led_notif, PWM_DUTY_SLOW, PWM_PERIOD_SLOW);
        	if (ret) {
              		pr_err("%s: duty=%d, period=%d\n", __func__, PWM_DUTY_SLOW, PWM_PERIOD_SLOW);
              		pr_err("%s: pwm_config on pwm failed %d\n", __func__, ret);
              		return ret;
        	}
		pwm_enable(led_notif);
		spin_lock_irqsave(&led_notif_info.spin_lock, flags);
		led_notif_info.duty = PWM_DUTY_SLOW;
		led_notif_info.period = PWM_PERIOD_SLOW;
		spin_unlock_irqrestore(&led_notif_info.spin_lock, flags);	
		break;
	case BREATH_MODE:
		pr_info("%s: led notification breathing mode\n", __func__);
		ret = pm8058_pwm_lut_config(led_notif, BREATH_PERIOD_US, duty_pct, DUTY_TIME_MS, START_IDX, IDX_LEN,  PAUSE_LO, PAUSE_HI, PM_PWM_LUT_LOOP);
		if (ret) {
              		pr_err("%s: pwm_lut configure failed %d\n", __func__, ret);
              		return ret;
        	}
        	ret = pm8058_pwm_lut_enable(led_notif, 1);
		if (ret) {
              		pr_err("%s: pwm_lut enable failed %d\n", __func__, ret);
              		return ret;
        	}
		spin_lock_irqsave(&led_notif_info.spin_lock, flags);
		led_notif_info.duty = PWM_DUTY_BREATH;
		led_notif_info.period = PWM_PERIOD_BREATH;
		spin_unlock_irqrestore(&led_notif_info.spin_lock, flags);	
		break;
	default:
		pr_err("%s: unknown flags: %d\n", __func__, mode);
		return -EINVAL;
	}
	
    	spin_lock_irqsave(&led_notif_info.spin_lock, flags);
	led_notif_info.mode = led_notif_info.led_notification_dev.brightness = mode;
	spin_unlock_irqrestore(&led_notif_info.spin_lock, flags);
	return ret;
}

static ssize_t led_notif_mode_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
        struct led_classdev *led_dev = dev_get_drvdata(dev);
		
        return sprintf(buf, "%u\n", led_dev->brightness);
}

static ssize_t led_notif_mode_store(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t size)
{
        struct led_classdev *led_dev = dev_get_drvdata(dev);
        char *tmp;
        ssize_t ret = -EINVAL;
        unsigned long mode = simple_strtoul(buf, &tmp, 10);
        size_t num = tmp - buf;

        if (isspace(*tmp))
                num++;

        if (num == size) {
                ret = num;
                if (mode > led_dev->max_brightness)
                	mode = led_dev->max_brightness;
	        led_notification_control(mode);
        }

        return ret;
}

static ssize_t led_notif_duty_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", led_notif_info.duty);
}

static ssize_t led_notif_duty_store(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t size)
{
        char *tmp;
        ssize_t ret ;
	unsigned long flags;
        unsigned int duty_val = simple_strtoul(buf, &tmp, 10);
        size_t num = tmp - buf;

        if (isspace(*tmp))
                num++;

        if (num == size) {
	            ret = pwm_config(led_notif, duty_val, led_notif_info.period);
		if (ret) {
              		pr_err("%s: duty=%u, period=%d\n", __func__, duty_val, led_notif_info.period);
              		pr_err("%s: pwm_config on pwm duty failed %d\n", __func__, ret);
              		return ret;
    		}
        }
	spin_lock_irqsave(&led_notif_info.spin_lock, flags);
	led_notif_info.duty = duty_val;
        spin_unlock_irqrestore(&led_notif_info.spin_lock, flags);
		
        return num;
}

static ssize_t led_notif_period_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{		
        return sprintf(buf, "%u\n", led_notif_info.period);
}

static ssize_t led_notif_period_store(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t size)
{
        char *tmp;
        ssize_t ret ;
	unsigned long flags;
        unsigned int period_val = simple_strtoul(buf, &tmp, 10);
        size_t num = tmp - buf;

        if (isspace(*tmp))
                num++;

        if (num == size) {
		ret = pwm_config(led_notif, led_notif_info.duty, period_val);
		if (ret) {
              		pr_err("%s: duty=%u, period=%d\n", __func__, led_notif_info.duty, period_val);
              		pr_err("%s: pwm_config on pwm period failed %d\n", __func__, ret);
              		return ret;
    		}
        }
	spin_lock_irqsave(&led_notif_info.spin_lock, flags);
        led_notif_info.period = period_val;
	spin_unlock_irqrestore(&led_notif_info.spin_lock, flags);
		
        return num;
}

static struct device_attribute led_notification_attrs[] = {
        __ATTR(mode, 0664, led_notif_mode_show, led_notif_mode_store),
	__ATTR(duty, 0664, led_notif_duty_show, led_notif_duty_store),
	__ATTR(period, 0664, led_notif_period_show, led_notif_period_store),
        __ATTR_NULL,
};

static int led_notification_probe(struct platform_device *pdev)
{
	int ret = 0;

	printk("enter led notification probe\n");
	
	led_notif = pwm_request(LED_DRV0_N, "notification");

	if (led_notif == NULL || IS_ERR(led_notif)) {
           	pr_err("%s pwm_request() failed\n", __func__);
           	led_notif = NULL;
	   	goto err1;
    	}

   	ret = pm8058_pwm_config_led(led_notif, PM_PWM_LED_0, PM_PWM_CONF_PWM1, 5);
   	if (ret)
   	{
		pr_err("%s: pm8058_pwm_config_led failed\n", __func__);
		goto err1;
   	}

  
	spin_lock_init(&led_notif_info.spin_lock);
	led_notif_info.duty = PWM_DUTY_OFF;
	led_notif_info.period = PWM_PERIOD_OFF;
	led_notif_info.led_notification_dev.name = pdev->name;
	led_notif_info.led_notification_dev.brightness_set = NULL;
	led_notif_info.mode = led_notif_info.led_notification_dev.brightness = 1;
	ret = led_classdev_register(&pdev->dev, &led_notif_info.led_notification_dev);
	if (ret < 0) {
		pr_err("failed on led_notification\n");
		goto err1;
	}

	/* create "mode" attribute file */
	ret = device_create_file(led_notif_info.led_notification_dev.dev, &led_notification_attrs[0]);
	if (ret) {
		pr_err("device create file for mode error!\n");
		goto err1;
	}
	
        /* create "duty" attribute file */
	ret = device_create_file(led_notif_info.led_notification_dev.dev, &led_notification_attrs[1]);
	if (ret) {
		pr_err("device create file for duty  error!\n");
		goto err1;
	}
	
    	/* create "period" attribute file */
	ret = device_create_file(led_notif_info.led_notification_dev.dev, &led_notification_attrs[2]);
	if (ret) {
		pr_err("device create file for period error!\n");
		goto err1;
	}

	return 0;
err1:
	pwm_free(led_notif);
 	return ret;
}

static int led_notification_remove(struct platform_device *pdev)
{
	pwm_free(led_notif);
	led_classdev_unregister(&led_notif_info.led_notification_dev);
	return 0;
}
 

static struct platform_device led_notification_device = 
{
	.name 	= LED_NOTIFICATION_NAME,
};

static struct platform_driver led_notification_driver = {
	.probe		= led_notification_probe,
	.remove		= led_notification_remove,
	.driver		= {
		.name		= LED_NOTIFICATION_NAME,
		.owner		= THIS_MODULE,
	},
};

static int __init led_notification_init(void)
{
	int ret;
 	printk("enter led notification init\n");

	ret = platform_device_register(&led_notification_device);
	if (ret < 0)
	{
		pr_err("%s: Register device LED notification error!\n", __func__);
		return ret;
	}

	return platform_driver_register(&led_notification_driver);
}

static void __exit led_notification_exit(void)
{
	platform_device_unregister(&led_notification_device);
	platform_driver_unregister(&led_notification_driver);
}

module_init(led_notification_init);
module_exit(led_notification_exit);

MODULE_AUTHOR("Thunderst");
MODULE_DESCRIPTION("LED notification driver");
MODULE_LICENSE("GPL");
