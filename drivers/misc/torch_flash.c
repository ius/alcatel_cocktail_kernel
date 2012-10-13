/* 
 *
 * SGM3140 torch&flash　driver
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
#include <linux/leds-pmic8058.h>
#include <linux/pwm.h>
#include <linux/pmic8058-pwm.h>
#include <linux/err.h>
#include <linux/mfd/pmic8058.h>
#include <linux/mfd/pm8xxx/gpio.h>
#include <linux/debugfs.h>
#include <linux/slab.h>
#include <linux/leds.h>
#include <linux/sysfs.h>
#include <linux/ctype.h>



/* PMIC8058 PIN */
#define TORCH_FLASH	 	16       /* PMIC GPIO Number 17 */
#define CAM_FLASH_DRV_EN 	25       /* PMIC GPIO Number 26 */
#define TORCH_FLASH_PWM       	2        /* PMIC GPIO  Number 26, for pwm is 2*/

#define TORCH_FLASH_NAME 		"SGM3140_torch_flash"
#define TORCH_FLASH_GPIO_LABEL  	"torch_flash_select"
#define CAM_FLASH_DRV_EN_GPIO_LABEL  	"cam_flash_drv_en"        
#define TORCH_PWM_LABEL                 "torch_pwm"

#define FLASH_MODE        	4

#define TORCH_MODE_SLIGHT    	3 
#define TORCH_MODE_NORMAL  	2
#define TORCH_MODE_HIGH        	1
#define TORCH_FLASH_OFF   	0


#define PWM_PERIOD_DEF  	80
#define PWM_DUTY_DEF       	7

#define PWM_PERIOD_SLIGHT   	80
#define PWM_DUTY_SLIGHT       	20

#define PWM_PERIOD_NORMAL  	80
#define PWM_DUTY_NORMAL      	40

#define PWM_PERIOD_HIGH        	80
#define PWM_DUTY_HIGH           80

#define PM8058_GPIO_PM_TO_SYS(pm_gpio)     (pm_gpio + NR_GPIO_IRQS)
#define PM8058_GPIO_SYS_TO_PM(sys_gpio)    (sys_gpio - NR_GPIO_IRQS)

struct torch_flash_struct {
	struct led_classdev torch_flash_dev;
	spinlock_t spin_lock;
	int mode;
};

static struct pm_gpio flash_drv_enable = {
	.direction      = PM_GPIO_DIR_OUT,
	.pull           = PM_GPIO_PULL_NO,
	.out_strength   = PM_GPIO_STRENGTH_HIGH,
	.function       = PM_GPIO_FUNC_NORMAL,
	.inv_int_pol    = 0,
	.vin_sel        = 2,
	.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
	.output_value   = 0,
};

static struct pm_gpio torch_flash_enable = {
	.direction      = PM_GPIO_DIR_OUT,
	.pull           = PM_GPIO_PULL_NO,
	.out_strength   = PM_GPIO_STRENGTH_HIGH,
	.function       = PM_GPIO_FUNC_NORMAL,
	.inv_int_pol    = 0,
	.vin_sel        = 2,
	.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
	.output_value   = 0,
};

static struct torch_flash_struct sgm3140_info;
static struct pwm_device *torch_pwm;

int torch_flash_control(int mode)
{
	int ret = 0;
	unsigned long flags;

	spin_lock_irqsave(&sgm3140_info.spin_lock, flags);

	switch (mode) {
	case TORCH_MODE_SLIGHT:
		pr_info("%s: torch  slightly light\n", __func__);
               	gpio_set_value(PM8058_GPIO_PM_TO_SYS(TORCH_FLASH), 0);
	       	gpio_set_value(PM8058_GPIO_PM_TO_SYS(CAM_FLASH_DRV_EN), 0);
	       	pwm_request(TORCH_FLASH_PWM,  TORCH_PWM_LABEL);

	       	ret = pwm_config(torch_pwm,  PWM_DUTY_SLIGHT,  PWM_PERIOD_SLIGHT);
               	if (ret) {
				pr_err("%s: duty=%d, period=%d\n",
									__func__, PWM_DUTY_SLIGHT, PWM_PERIOD_SLIGHT);
				pr_err("%s: pwm_config on pwm failed %d\n",
									__func__, ret);
				goto done;
		}
		pwm_enable(torch_pwm);
		break;
       case TORCH_MODE_NORMAL:
	   	pr_info("%s: torch normal light\n ",  __func__);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(TORCH_FLASH), 0);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(CAM_FLASH_DRV_EN), 0);
		pwm_request(TORCH_FLASH_PWM,  TORCH_PWM_LABEL);

		pwm_config(torch_pwm,  PWM_DUTY_NORMAL,  PWM_PERIOD_NORMAL);
		if (ret) {
				pr_err("%s: duty=%d, period=%d\n",
									__func__, PWM_DUTY_NORMAL, PWM_PERIOD_NORMAL);
				pr_err("%s: pwm_config on pwm failed %d\n",
									__func__, ret);
				goto done;
		}
	
		pwm_enable(torch_pwm);
		break;
	case TORCH_MODE_HIGH:
	   	pr_info("%s: torch high light\n ",  __func__);
		/*modify by jinlin.Hu for change pwm to high always.START*/
#if 0
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(TORCH_FLASH), 0);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(CAM_FLASH_DRV_EN), 0);
		pwm_request(TORCH_FLASH_PWM,  TORCH_PWM_LABEL);

		ret = pwm_config(torch_pwm,  PWM_DUTY_HIGH,  PWM_PERIOD_HIGH);
		if (ret) {
				pr_err("%s: duty=%d, period=%d\n",
									__func__, PWM_DUTY_HIGH, PWM_PERIOD_HIGH);
				pr_err("%s: pwm_config on pwm failed %d\n",
									__func__, ret);
				goto done;
		}	

		pwm_enable(torch_pwm);
#endif
		pwm_disable(torch_pwm);
		pwm_free(torch_pwm);

		ret = pm8xxx_gpio_config(PM8058_GPIO_PM_TO_SYS(CAM_FLASH_DRV_EN), &flash_drv_enable);     
     		if (ret) {
           		pr_err("%s: PMIC GPIO %d set failed\n", __func__,(CAM_FLASH_DRV_EN + 1));
             		goto done;
     		}

		gpio_set_value(PM8058_GPIO_PM_TO_SYS(TORCH_FLASH), 0);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(CAM_FLASH_DRV_EN), 1);
		/*modify by jinlin.Hu for change pwm to high always.END*/
		break;
	case FLASH_MODE:
		pr_info("%s: flash mode\n", __func__);
		pwm_disable(torch_pwm);
		pwm_free(torch_pwm);

		ret = pm8xxx_gpio_config(PM8058_GPIO_PM_TO_SYS(CAM_FLASH_DRV_EN), &flash_drv_enable);     
     		if (ret) {
           		pr_err("%s: PMIC GPIO %d set failed\n", __func__,(CAM_FLASH_DRV_EN + 1));
             		goto done;
     		}
			
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(TORCH_FLASH), 0);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(TORCH_FLASH), 1);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(CAM_FLASH_DRV_EN), 1);
		break;
    	case TORCH_FLASH_OFF:
		pr_info("%s: torch&flash off\n", __func__);
		pwm_disable(torch_pwm);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(CAM_FLASH_DRV_EN), 0);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(TORCH_FLASH), 0);
		break;
	default:
		pr_err("%s: unknown flags: %d\n", __func__, mode);
		ret = -EINVAL;
		goto done;
	}

	sgm3140_info.mode = sgm3140_info.torch_flash_dev.brightness = mode;
done:
	spin_unlock_irqrestore(&sgm3140_info.spin_lock, flags);
	return ret;
}
EXPORT_SYMBOL(torch_flash_control);

static void torch_flash_brightness_set(struct led_classdev *led_cdev,
		enum led_brightness brightness)
{
	int value;
	switch (brightness) {
	case LED_HALF:
		value = TORCH_MODE_NORMAL;
		break;
	case LED_FULL:
		value = TORCH_MODE_HIGH;
		break;
	case LED_OFF:
	default:
		value = TORCH_MODE_SLIGHT;
	};

	torch_flash_control(value);
}

static ssize_t led_mode_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
      //struct led_classdev *led_dev = dev_get_drvdata(dev);
		
        return sprintf(buf, "%u\n", sgm3140_info.mode);
}

static ssize_t led_mode_store(struct device *dev,
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
                if (!(led_dev->flags & LED_SUSPENDED)) {
	            	torch_flash_control(mode);
                }
        }

        return ret;
}


static struct device_attribute led_mode_attrs[] = {
        __ATTR(mode, 0664, led_mode_show, led_mode_store),
        		__ATTR_NULL,
};


static int torch_flash_probe(struct platform_device *pdev)
{
	int ret = 0;

        ret = gpio_request(PM8058_GPIO_PM_TO_SYS(TORCH_FLASH), TORCH_FLASH_GPIO_LABEL);
	if (ret) {
		pr_err("gpio_request(%d) <%s> failed: %d\n", 
			TORCH_FLASH, TORCH_FLASH_GPIO_LABEL ?: "?", ret);
		goto err2;
	}

	ret = pm8xxx_gpio_config(PM8058_GPIO_PM_TO_SYS(TORCH_FLASH), &torch_flash_enable); 
     	if (ret) {
       		pr_err("%s: PMIC GPIO %d set failed\n", __func__, (TORCH_FLASH + 1));
             	goto err1;
     	}

        //for pwm

	torch_pwm = pwm_request(TORCH_FLASH_PWM,  TORCH_PWM_LABEL);
	
	if (torch_pwm== NULL || IS_ERR(torch_pwm)) {
			   pr_err("%s pwm_request() failed\n", __func__);
			   torch_pwm = NULL;
			   goto err1;
	}	
	
	 ret = pwm_config(torch_pwm, PWM_DUTY_DEF, PWM_PERIOD_DEF);
	 if (ret) {
				pr_err("%s: duty=%d, period=%d\n",
									__func__, PWM_DUTY_DEF, PWM_PERIOD_DEF);
				pr_err("%s: pwm_config on pwm failed %d\n",
									__func__, ret);
				goto err1;
	}	
	pr_err("%s: duty=%d, period=%d\n", __func__, PWM_DUTY_DEF, PWM_PERIOD_DEF);
	
	pwm_disable(torch_pwm);
              
	spin_lock_init(&sgm3140_info.spin_lock);
	sgm3140_info.torch_flash_dev.name = pdev->name;
	sgm3140_info.torch_flash_dev.brightness_set = torch_flash_brightness_set;
	sgm3140_info.mode = sgm3140_info.torch_flash_dev.brightness = TORCH_FLASH_OFF;
	ret = led_classdev_register(&pdev->dev, &sgm3140_info.torch_flash_dev);
	if (ret < 0) {
		pr_err("failed on led_classdev_register\n");
		goto err1;
	}

	//create "mode" attribute file
	ret = device_create_file(sgm3140_info.torch_flash_dev.dev,	&led_mode_attrs[0]);
	if (ret) {
			pr_err("device create file for mode error!\n");
			goto err1;
	}

	return 0;
err1:
	pwm_free(torch_pwm);
err2:
	gpio_free(PM8058_GPIO_PM_TO_SYS(TORCH_FLASH));
 	return ret;
}

static int torch_flash_remove(struct platform_device *pdev)
{
	torch_flash_control(TORCH_FLASH_OFF);
	pwm_disable(torch_pwm);
	led_classdev_unregister(&sgm3140_info.torch_flash_dev);
	gpio_free(PM8058_GPIO_PM_TO_SYS(CAM_FLASH_DRV_EN));
	gpio_free(PM8058_GPIO_PM_TO_SYS(TORCH_FLASH));
	pwm_free(torch_pwm);
	return 0;
}
 

static struct platform_device torch_flash_device = 
{
	.name 	= TORCH_FLASH_NAME,
};

static struct platform_driver torch_flash_driver = {
	.probe		= torch_flash_probe,
	.remove		= torch_flash_remove,
	.driver		= {
		.name		= TORCH_FLASH_NAME,
		.owner		= THIS_MODULE,
	},
};

static int __init torch_flash_init(void)
{
	int ret;

	ret = platform_device_register(&torch_flash_device);
	if (ret < 0)
	{
		pr_err("%s: Register device SGM3140 error!\n", __func__);
		return ret;
	}
	return platform_driver_register(&torch_flash_driver);
}

static void __exit torch_flash_exit(void)
{
	platform_device_unregister(&torch_flash_device);
	platform_driver_unregister(&torch_flash_driver);
}

module_init(torch_flash_init);
module_exit(torch_flash_exit);

MODULE_DESCRIPTION("SGM3140 torch&flash driver");
MODULE_LICENSE("GPL v2");
