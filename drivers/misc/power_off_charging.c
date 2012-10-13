/* 
 *
 * misc driver for power off charging
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
#include <linux/miscdevice.h>
#include <linux/sched.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include "../../../arch/arm/mach-msm/proc_comm.h"
#include "../../../arch/arm/mach-msm/include/mach/msm_smsm.h"
int boot_mode;
//#define SMEM_PROC_COMM_GET_POWER_ON_STATUS 6
static int power_on_reason = 0;
//extern int boot_reason;
volatile char power_off_charging_state = 0;
EXPORT_SYMBOL(power_off_charging_state);
ssize_t power_off_charging_read(struct file *filp,char __user *buf,size_t count, loff_t *f_pos)
{	
    if(copy_to_user(buf,(void *)&power_on_reason,sizeof(buf) )){
        return -EFAULT;
    }
    return sizeof(buf);
}

long power_off_charging_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	switch(cmd) {
		case 0:
			power_off_charging_state = 0;
			break;
		case 1:
			power_off_charging_state = 1;
			break;
		default:
			return -ENOTTY;
	}
	return 0;
}

struct file_operations power_off_charging_fops =
{
    .owner = THIS_MODULE,
    .read  = &power_off_charging_read,
	.unlocked_ioctl = &power_off_charging_ioctl,
};
struct miscdevice  power_off_charging_driver =
{
    .minor = MISC_DYNAMIC_MINOR,
    .name = "power_off_charging",
    .fops = &power_off_charging_fops,
};

ssize_t apps_boot_mode_read(struct file *filp,char __user *buf,size_t count, loff_t *f_pos)
{
    if(copy_to_user(buf,(void *)&boot_mode,sizeof(buf) )){
        return -EFAULT;
    }
    return sizeof(buf);
}

struct file_operations apps_boot_mode_fops =
{
    .owner = THIS_MODULE,
    .read  = &apps_boot_mode_read,
};

struct miscdevice  apps_boot_mode_driver =
{
    .minor = 255,
    .name = "apps_boot_mode",
    .fops = &apps_boot_mode_fops,
};

static int __init power_off_charging_init(void)
{
    int rc = 0;
    unsigned boot_mode_len;
    pr_debug("%s: enter\n", __func__);

    boot_mode=*(unsigned int *)smem_get_entry(SMEM_APPS_BOOT_MODE,&boot_mode_len);

    if (boot_mode == 0x776655DD)  //SYX added for restart cmd from dloadarm.c 
    {
      power_on_reason = 1;
      boot_mode = 0x776655AA;
    }
    else
    {
	rc = msm_proc_comm(PCOM_GET_POWER_ON_STATUS,&power_on_reason,NULL);
    }
    rc = misc_register(&apps_boot_mode_driver); 
    rc = misc_register(&power_off_charging_driver);

    printk("%s: power_on_reason:%x ,boot_mode:%x\n",__func__,power_on_reason,boot_mode);
    return rc;
}

static void __exit power_off_charging_exit(void)
{
    misc_deregister(&power_off_charging_driver);
    misc_deregister(&apps_boot_mode_driver);
}


module_init(power_off_charging_init);
module_exit(power_off_charging_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Thunderst");
MODULE_DESCRIPTION("used for power off chagring.");
MODULE_VERSION("1.0");
MODULE_ALIAS("platform:power_off_charging");
