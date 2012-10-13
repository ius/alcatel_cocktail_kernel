#include <linux/delay.h>
#include <linux/pwm.h>
#ifdef CONFIG_PMIC8058_PWM
#include <linux/mfd/pmic8058.h>
#include <linux/pmic8058-pwm.h>
#endif
#include <mach/gpio.h>
#include "msm_fb.h"
#include "mddihost.h"
#include "mddihosti.h"

#ifdef CONFIG_DEBUG_FS
#include "mddi_ry002z.h"
#endif

static struct pwm_device *bl_pwm;

#define PWM_FREQ_HZ 20000  //20K
#define PWM_LEVEL 255
#define PWM_PERIOD_USEC (USEC_PER_SEC / PWM_FREQ_HZ)

static struct msm_panel_common_pdata *mddi_ry002z_pdata;
static struct platform_device *ry002z_fbpdev;

static int led_pwm;             /* pm8058 gpio 24, channel 0 */
static int gpio_reset;          /* msm8255 gpio 100 */
static int high = 1;
static int low = 0;
static int lcdid=0;

#define LCDID_TRULY 2
#define LCDID_SEIKO 0
#define LCDID_TDT 	1

#ifdef CONFIG_DEBUG_FS
static struct driver_state_type lcd_driver_state = { 
	.display_on = 1,
	.is_sleep = 0,
	.bl_level =0
};
#endif

static void mddi_gpio_init(void)
{
	uint32 ret = 0;

	pr_info("%s:config gpio\n", __func__);
	ret = gpio_request(gpio_reset, "mddi reset");

	gpio_tlmm_config( GPIO_CFG(gpio_reset,  0, GPIO_CFG_OUTPUT, 
				GPIO_CFG_PULL_UP, GPIO_CFG_16MA), GPIO_CFG_ENABLE);

	if (!ret) {
		gpio_set_value(gpio_reset, high);
	} else {
		pr_err(KERN_ERR "failed to request gpio\n");
		pr_err(KERN_ERR "force a set to high\n");
		gpio_set_value(gpio_reset, high);
	}

	return;
}

static void mddi_lcdid_read(void)
{
	uint gpio_lcdid0=149;
	uint gpio_lcdid1=150;
	int lcdid0=0;
	int lcdid1=0;
	
	gpio_tlmm_config( GPIO_CFG(gpio_lcdid0,  0, GPIO_CFG_INPUT, 
				GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	lcdid0 = gpio_get_value(gpio_lcdid0);
	
	gpio_tlmm_config( GPIO_CFG(gpio_lcdid1,  0, GPIO_CFG_INPUT, 
				GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	lcdid1 = gpio_get_value(gpio_lcdid1);

	lcdid=lcdid1*2+lcdid0;
	printk("%s-----lcdid=%d\n", __func__, lcdid);
}

static void write_client_reg(uint16 reg, uint8 data)
{
	int ret;
	ret = mddi_queue_register_write(reg, data, TRUE, 0);
	if (ret == -EBUSY)
		pr_err("%s:write_client_reg failed,reg = %x,data = %c\n", __func__, reg, data);
}

static void mddi_ry002z_lcd_ic_init(void)
{
	pr_info("%s begin\n", __func__);
	
	mddi_host_disable_hibernation(FALSE);
	
	gpio_set_value(gpio_reset, low);
	mddi_wait(10);	
	gpio_set_value(gpio_reset, high);
	mddi_wait(120);

	mddi_host_disable_hibernation(TRUE);

	if((lcdid == LCDID_TRULY) ||(lcdid == LCDID_TDT)){
		write_client_reg(0xB9,0xFF); 
		write_client_reg(0xFD,0x83);
		write_client_reg(0xFD,0x69);
		
		write_client_reg(0x36, 0x08);

		write_client_reg(0xD5,0x00);
		write_client_reg(0xFD,0x05);
		write_client_reg(0xFD,0x03);
		write_client_reg(0xFD,0x00);
		write_client_reg(0xFD,0x01);
		write_client_reg(0xFD,0x09);
		write_client_reg(0xFD,0x10);
		write_client_reg(0xFD,0x80);
		write_client_reg(0xFD,0x37);
		write_client_reg(0xFD,0x37);
		write_client_reg(0xFD,0x20);
		write_client_reg(0xFD,0x31);
		write_client_reg(0xFD,0x46);
		write_client_reg(0xFD,0x8A);
		write_client_reg(0xFD,0x57);
		write_client_reg(0xFD,0x9B);
		write_client_reg(0xFD,0x20);
		write_client_reg(0xFD,0x31);
		write_client_reg(0xFD,0x46);
		write_client_reg(0xFD,0x8A);
		write_client_reg(0xFD,0x57);
		write_client_reg(0xFD,0x9B);
		write_client_reg(0xFD,0x07);
		write_client_reg(0xFD,0x0F);
		write_client_reg(0xFD,0x02);
		write_client_reg(0xFD,0x00);

		write_client_reg(0xB1,0x01); 
		write_client_reg(0xFD,0x00);
		write_client_reg(0xFD,0x34);
		write_client_reg(0xFD,0x06);
		write_client_reg(0xFD,0x00);
		write_client_reg(0xFD,0x0F);
		write_client_reg(0xFD,0x0F);
		write_client_reg(0xFD,0x2A);
		write_client_reg(0xFD,0x32);
		write_client_reg(0xFD,0x3F);
		write_client_reg(0xFD,0x3F);
		write_client_reg(0xFD,0x07);
		write_client_reg(0xFD,0x23);
		write_client_reg(0xFD,0x01);
		write_client_reg(0xFD,0xE6);
		write_client_reg(0xFD,0xE6);
		write_client_reg(0xFD,0xE6);
		write_client_reg(0xFD,0xE6);
		write_client_reg(0xFD,0xE6);

		write_client_reg(0xB2,0x00); 
		write_client_reg(0xFD,0x20);
		write_client_reg(0xFD,0x0A);
		write_client_reg(0xFD,0x0A);
		write_client_reg(0xFD,0x70);
		write_client_reg(0xFD,0x00);
		write_client_reg(0xFD,0xFF);
		write_client_reg(0xFD,0x00);
		write_client_reg(0xFD,0x0a); // 11 frames one Vsync Signal
		write_client_reg(0xFD,0x00);
		write_client_reg(0xFD,0x00);
		write_client_reg(0xFD,0x03);
		write_client_reg(0xFD,0x03);
		write_client_reg(0xFD,0x00);
		write_client_reg(0xFD,0x01);
		 
		write_client_reg(0xB4,0x00);
		write_client_reg(0xFD,0x0C);
		write_client_reg(0xFD,0xA0);
		write_client_reg(0xFD,0x0E);
		write_client_reg(0xFD,0x06);	

		write_client_reg(0x35,0x00); // tearing on

		write_client_reg(0xCC, 0x03);
		write_client_reg(0xB6,0x2C);
		write_client_reg(0xFD,0x2C);
		 
		write_client_reg(0x3A,0x77);

		write_client_reg(0xBA,0x00);
		write_client_reg(0xFD,0xA0);
		write_client_reg(0xFD,0xC6);
		write_client_reg(0xFD,0x00);
		write_client_reg(0xFD,0x0A);
		write_client_reg(0xFD,0x00);
		write_client_reg(0xFD,0x10);
		write_client_reg(0xFD,0x30);
		write_client_reg(0xFD,0x6F);
		write_client_reg(0xFD,0x02);
		write_client_reg(0xFD,0x11);
		write_client_reg(0xFD,0x18);	 
				 
		write_client_reg(0xE0,0x00);
		write_client_reg(0xFD,0x08);
		write_client_reg(0xFD,0x0D);
		write_client_reg(0xFD,0x2D);
		write_client_reg(0xFD,0x34);
		write_client_reg(0xFD,0x3F);
		write_client_reg(0xFD,0x19);
		write_client_reg(0xFD,0x38);
		write_client_reg(0xFD,0x09);
		write_client_reg(0xFD,0x0E);
		write_client_reg(0xFD,0x0E);
		write_client_reg(0xFD,0x12);
		write_client_reg(0xFD,0x14);
		write_client_reg(0xFD,0x12);
		write_client_reg(0xFD,0x14);
		write_client_reg(0xFD,0x13);
		write_client_reg(0xFD,0x19);
		write_client_reg(0xFD,0x00);
		write_client_reg(0xFD,0x08);
		write_client_reg(0xFD,0x0D);
		write_client_reg(0xFD,0x2D);
		write_client_reg(0xFD,0x34);
		write_client_reg(0xFD,0x3F);
		write_client_reg(0xFD,0x19);
		write_client_reg(0xFD,0x38);
		write_client_reg(0xFD,0x09);
		write_client_reg(0xFD,0x0E);
		write_client_reg(0xFD,0x0E);
		write_client_reg(0xFD,0x12);
		write_client_reg(0xFD,0x14);
		write_client_reg(0xFD,0x12);
		write_client_reg(0xFD,0x14);
		write_client_reg(0xFD,0x13);
		write_client_reg(0xFD,0x19);		
		
		write_client_reg(0x29,0x00);
		write_client_reg(0x11,0x00);
		mddi_wait(120);                 
	}else {
		write_client_reg(0xB9, 0xff);
		write_client_reg(0xFD, 0x83);
		write_client_reg(0xFD, 0x69);

		write_client_reg(0xB1, 0x85);
		write_client_reg(0xFD, 0x00);
		write_client_reg(0xFD, 0x34);
		write_client_reg(0xFD, 0x06);
		write_client_reg(0xFD, 0x00);
		write_client_reg(0xFD, 0x0f);
		write_client_reg(0xFD, 0x0f);
		write_client_reg(0xFD, 0x2a);
		write_client_reg(0xFD, 0x32);
		write_client_reg(0xFD, 0x3f);
		write_client_reg(0xFD, 0x3f);
		write_client_reg(0xFD, 0x07);
		write_client_reg(0xFD, 0x23);
		write_client_reg(0xFD, 0x01);
		write_client_reg(0xFD, 0xe6);
		write_client_reg(0xFD, 0xe6);
		write_client_reg(0xFD, 0xe6);
		write_client_reg(0xFD, 0xe6);
		write_client_reg(0xFD, 0xe6);

		write_client_reg(0xB2, 0x00);
		write_client_reg(0xFD, 0x20);
		write_client_reg(0xFD, 0x0a);
		write_client_reg(0xFD, 0x03);
		write_client_reg(0xFD, 0x70);
		write_client_reg(0xFD, 0x00);
		write_client_reg(0xFD, 0xff);		
		write_client_reg(0xFD, 0x06); 
		write_client_reg(0xFD, 0x0a); 
		write_client_reg(0xFD, 0x00);
		write_client_reg(0xFD, 0x00);
		write_client_reg(0xFD, 0x03);
		write_client_reg(0xFD, 0x03);
		write_client_reg(0xFD, 0x00);
		write_client_reg(0xFD, 0x01);

		write_client_reg(0xB4, 0x00);  
		write_client_reg(0xFD, 0x0f);  
		write_client_reg(0xFD, 0x82);  
		write_client_reg(0xFD, 0x0c);  
		write_client_reg(0xFD, 0x03);  
	

		write_client_reg(0xD5, 0x00);
		write_client_reg(0xFD, 0x05);
		write_client_reg(0xFD, 0x03);
		write_client_reg(0xFD, 0x00);
		write_client_reg(0xFD, 0x01);
		write_client_reg(0xFD, 0x09);
		write_client_reg(0xFD, 0x10);
		write_client_reg(0xFD, 0x80);
		write_client_reg(0xFD, 0x37);
		write_client_reg(0xFD, 0x37);
		write_client_reg(0xFD, 0x20);
		write_client_reg(0xFD, 0x31);
		write_client_reg(0xFD, 0x46);
		write_client_reg(0xFD, 0x8a);
		write_client_reg(0xFD, 0x57);
		write_client_reg(0xFD, 0x9b);
		write_client_reg(0xFD, 0x20);
		write_client_reg(0xFD, 0x31);
		write_client_reg(0xFD, 0x46);
		write_client_reg(0xFD, 0x8a);
		write_client_reg(0xFD, 0x57);
		write_client_reg(0xFD, 0x9b);
		write_client_reg(0xFD, 0x07);
		write_client_reg(0xFD, 0x0f);
		write_client_reg(0xFD, 0x02);
		write_client_reg(0xFD, 0x00);

		write_client_reg(0xCC, 0x02);
		write_client_reg(0xFD, 0x01);

		write_client_reg(0x35, 0x00);

		write_client_reg(0xB6, 0x39);
		write_client_reg(0xFD, 0x39);

		write_client_reg(0xE0, 0x00);
		write_client_reg(0xFD, 0x08);
		write_client_reg(0xFD, 0x0d);
		write_client_reg(0xFD, 0x2d);
		write_client_reg(0xFD, 0x34);
		write_client_reg(0xFD, 0x3f);
		write_client_reg(0xFD, 0x19);
		write_client_reg(0xFD, 0x38);
		write_client_reg(0xFD, 0x09);
		write_client_reg(0xFD, 0x0e);
		write_client_reg(0xFD, 0x0e);
		write_client_reg(0xFD, 0x12);
		write_client_reg(0xFD, 0x14);
		write_client_reg(0xFD, 0x12);
		write_client_reg(0xFD, 0x14);
		write_client_reg(0xFD, 0x13);
		write_client_reg(0xFD, 0x19);
		write_client_reg(0xFD, 0x00);
		write_client_reg(0xFD, 0x08);
		write_client_reg(0xFD, 0x0d);
		write_client_reg(0xFD, 0x2d);
		write_client_reg(0xFD, 0x34);
		write_client_reg(0xFD, 0x3f);
		write_client_reg(0xFD, 0x19);
		write_client_reg(0xFD, 0x38);
		write_client_reg(0xFD, 0x09);
		write_client_reg(0xFD, 0x0e);
		write_client_reg(0xFD, 0x0e);
		write_client_reg(0xFD, 0x12);
		write_client_reg(0xFD, 0x14);
		write_client_reg(0xFD, 0x12);
		write_client_reg(0xFD, 0x14);
		write_client_reg(0xFD, 0x13);
		write_client_reg(0xFD, 0x19);

		write_client_reg(0x29, 0x00);	
		write_client_reg(0x11, 0x00);
		mddi_wait(120);			
	}

#ifndef CONFIG_MDDI_HIMAX_24BIT_WORKROUND
	mddi_host_disable_hibernation(FALSE);
#endif

}

#ifdef CONFIG_MDDI_HIMAX_24BIT_WORKROUND
void mddi_himax_ic_init_again(void)
{
	static int init = 0;

	if(init)
		return;
	else
		init = 1;
		
	pr_info("%s begin\n", __func__);
	
	mddi_host_disable_hibernation(FALSE);	
	write_client_reg(0x01,0x00); 
	mddi_wait(120);
	mddi_host_disable_hibernation(TRUE);

	if((lcdid == LCDID_TRULY) ||(lcdid == LCDID_TDT)){
		write_client_reg(0xB9,0xFF); 
		write_client_reg(0xFD,0x83);
		write_client_reg(0xFD,0x69);
		
		write_client_reg(0x36, 0x08);

		write_client_reg(0xD5,0x00);
		write_client_reg(0xFD,0x05);
		write_client_reg(0xFD,0x03);
		write_client_reg(0xFD,0x00);
		write_client_reg(0xFD,0x01);
		write_client_reg(0xFD,0x09);
		write_client_reg(0xFD,0x10);
		write_client_reg(0xFD,0x80);
		write_client_reg(0xFD,0x37);
		write_client_reg(0xFD,0x37);
		write_client_reg(0xFD,0x20);
		write_client_reg(0xFD,0x31);
		write_client_reg(0xFD,0x46);
		write_client_reg(0xFD,0x8A);
		write_client_reg(0xFD,0x57);
		write_client_reg(0xFD,0x9B);
		write_client_reg(0xFD,0x20);
		write_client_reg(0xFD,0x31);
		write_client_reg(0xFD,0x46);
		write_client_reg(0xFD,0x8A);
		write_client_reg(0xFD,0x57);
		write_client_reg(0xFD,0x9B);
		write_client_reg(0xFD,0x07);
		write_client_reg(0xFD,0x0F);
		write_client_reg(0xFD,0x02);
		write_client_reg(0xFD,0x00);

		write_client_reg(0xB1,0x01); 
		write_client_reg(0xFD,0x00);
		write_client_reg(0xFD,0x34);
		write_client_reg(0xFD,0x06);
		write_client_reg(0xFD,0x00);
		write_client_reg(0xFD,0x0F);
		write_client_reg(0xFD,0x0F);
		write_client_reg(0xFD,0x2A);
		write_client_reg(0xFD,0x32);
		write_client_reg(0xFD,0x3F);
		write_client_reg(0xFD,0x3F);
		write_client_reg(0xFD,0x07);
		write_client_reg(0xFD,0x23);
		write_client_reg(0xFD,0x01);
		write_client_reg(0xFD,0xE6);
		write_client_reg(0xFD,0xE6);
		write_client_reg(0xFD,0xE6);
		write_client_reg(0xFD,0xE6);
		write_client_reg(0xFD,0xE6);

		write_client_reg(0xB2,0x00); 
		write_client_reg(0xFD,0x20);
		write_client_reg(0xFD,0x0A);
		write_client_reg(0xFD,0x0A);
		write_client_reg(0xFD,0x70);
		write_client_reg(0xFD,0x00);
		write_client_reg(0xFD,0xFF);
		write_client_reg(0xFD,0x00);
		write_client_reg(0xFD,0x0a); // 11 frames one Vsync Signal
		write_client_reg(0xFD,0x00);
		write_client_reg(0xFD,0x00);
		write_client_reg(0xFD,0x03);
		write_client_reg(0xFD,0x03);
		write_client_reg(0xFD,0x00);
		write_client_reg(0xFD,0x01);
		 
		write_client_reg(0xB4,0x00);
		write_client_reg(0xFD,0x0C);
		write_client_reg(0xFD,0xA0);
		write_client_reg(0xFD,0x0E);
		write_client_reg(0xFD,0x06);	

		write_client_reg(0x35,0x00); // tearing on

		write_client_reg(0xCC, 0x03);
		write_client_reg(0xB6,0x2C);
		write_client_reg(0xFD,0x2C);
		 
		write_client_reg(0x3A,0x77);

		write_client_reg(0xBA,0x00);
		write_client_reg(0xFD,0xA0);
		write_client_reg(0xFD,0xC6);
		write_client_reg(0xFD,0x00);
		write_client_reg(0xFD,0x0A);
		write_client_reg(0xFD,0x00);
		write_client_reg(0xFD,0x10);
		write_client_reg(0xFD,0x30);
		write_client_reg(0xFD,0x6F);
		write_client_reg(0xFD,0x02);
		write_client_reg(0xFD,0x11);
		write_client_reg(0xFD,0x18);	 
				 
		write_client_reg(0xE0,0x00);
		write_client_reg(0xFD,0x08);
		write_client_reg(0xFD,0x0D);
		write_client_reg(0xFD,0x2D);
		write_client_reg(0xFD,0x34);
		write_client_reg(0xFD,0x3F);
		write_client_reg(0xFD,0x19);
		write_client_reg(0xFD,0x38);
		write_client_reg(0xFD,0x09);
		write_client_reg(0xFD,0x0E);
		write_client_reg(0xFD,0x0E);
		write_client_reg(0xFD,0x12);
		write_client_reg(0xFD,0x14);
		write_client_reg(0xFD,0x12);
		write_client_reg(0xFD,0x14);
		write_client_reg(0xFD,0x13);
		write_client_reg(0xFD,0x19);
		write_client_reg(0xFD,0x00);
		write_client_reg(0xFD,0x08);
		write_client_reg(0xFD,0x0D);
		write_client_reg(0xFD,0x2D);
		write_client_reg(0xFD,0x34);
		write_client_reg(0xFD,0x3F);
		write_client_reg(0xFD,0x19);
		write_client_reg(0xFD,0x38);
		write_client_reg(0xFD,0x09);
		write_client_reg(0xFD,0x0E);
		write_client_reg(0xFD,0x0E);
		write_client_reg(0xFD,0x12);
		write_client_reg(0xFD,0x14);
		write_client_reg(0xFD,0x12);
		write_client_reg(0xFD,0x14);
		write_client_reg(0xFD,0x13);
		write_client_reg(0xFD,0x19);		
		
		write_client_reg(0x29,0x00);
		write_client_reg(0x11,0x00);
		mddi_wait(120);                 
	}else {
		write_client_reg(0xB9, 0xff);
		write_client_reg(0xFD, 0x83);
		write_client_reg(0xFD, 0x69);

		write_client_reg(0xB1, 0x85);
		write_client_reg(0xFD, 0x00);
		write_client_reg(0xFD, 0x34);
		write_client_reg(0xFD, 0x06);
		write_client_reg(0xFD, 0x00);
		write_client_reg(0xFD, 0x0f);
		write_client_reg(0xFD, 0x0f);
		write_client_reg(0xFD, 0x2a);
		write_client_reg(0xFD, 0x32);
		write_client_reg(0xFD, 0x3f);
		write_client_reg(0xFD, 0x3f);
		write_client_reg(0xFD, 0x07);
		write_client_reg(0xFD, 0x23);
		write_client_reg(0xFD, 0x01);
		write_client_reg(0xFD, 0xe6);
		write_client_reg(0xFD, 0xe6);
		write_client_reg(0xFD, 0xe6);
		write_client_reg(0xFD, 0xe6);
		write_client_reg(0xFD, 0xe6);

		write_client_reg(0xB2, 0x00);
		write_client_reg(0xFD, 0x20);
		write_client_reg(0xFD, 0x0a);
		write_client_reg(0xFD, 0x03);
		write_client_reg(0xFD, 0x70);
		write_client_reg(0xFD, 0x00);
		write_client_reg(0xFD, 0xff);		
		write_client_reg(0xFD, 0x06); 
		write_client_reg(0xFD, 0x0a); 
		write_client_reg(0xFD, 0x00);
		write_client_reg(0xFD, 0x00);
		write_client_reg(0xFD, 0x03);
		write_client_reg(0xFD, 0x03);
		write_client_reg(0xFD, 0x00);
		write_client_reg(0xFD, 0x01);

		write_client_reg(0xB4, 0x00);  
		write_client_reg(0xFD, 0x0f);  
		write_client_reg(0xFD, 0x82);  
		write_client_reg(0xFD, 0x0c);  
		write_client_reg(0xFD, 0x03);  
	

		write_client_reg(0xD5, 0x00);
		write_client_reg(0xFD, 0x05);
		write_client_reg(0xFD, 0x03);
		write_client_reg(0xFD, 0x00);
		write_client_reg(0xFD, 0x01);
		write_client_reg(0xFD, 0x09);
		write_client_reg(0xFD, 0x10);
		write_client_reg(0xFD, 0x80);
		write_client_reg(0xFD, 0x37);
		write_client_reg(0xFD, 0x37);
		write_client_reg(0xFD, 0x20);
		write_client_reg(0xFD, 0x31);
		write_client_reg(0xFD, 0x46);
		write_client_reg(0xFD, 0x8a);
		write_client_reg(0xFD, 0x57);
		write_client_reg(0xFD, 0x9b);
		write_client_reg(0xFD, 0x20);
		write_client_reg(0xFD, 0x31);
		write_client_reg(0xFD, 0x46);
		write_client_reg(0xFD, 0x8a);
		write_client_reg(0xFD, 0x57);
		write_client_reg(0xFD, 0x9b);
		write_client_reg(0xFD, 0x07);
		write_client_reg(0xFD, 0x0f);
		write_client_reg(0xFD, 0x02);
		write_client_reg(0xFD, 0x00);

		write_client_reg(0xCC, 0x02);
		write_client_reg(0xFD, 0x01);

		write_client_reg(0x35, 0x00);

		write_client_reg(0xB6, 0x39);
		write_client_reg(0xFD, 0x39);

		write_client_reg(0xE0, 0x00);
		write_client_reg(0xFD, 0x08);
		write_client_reg(0xFD, 0x0d);
		write_client_reg(0xFD, 0x2d);
		write_client_reg(0xFD, 0x34);
		write_client_reg(0xFD, 0x3f);
		write_client_reg(0xFD, 0x19);
		write_client_reg(0xFD, 0x38);
		write_client_reg(0xFD, 0x09);
		write_client_reg(0xFD, 0x0e);
		write_client_reg(0xFD, 0x0e);
		write_client_reg(0xFD, 0x12);
		write_client_reg(0xFD, 0x14);
		write_client_reg(0xFD, 0x12);
		write_client_reg(0xFD, 0x14);
		write_client_reg(0xFD, 0x13);
		write_client_reg(0xFD, 0x19);
		write_client_reg(0xFD, 0x00);
		write_client_reg(0xFD, 0x08);
		write_client_reg(0xFD, 0x0d);
		write_client_reg(0xFD, 0x2d);
		write_client_reg(0xFD, 0x34);
		write_client_reg(0xFD, 0x3f);
		write_client_reg(0xFD, 0x19);
		write_client_reg(0xFD, 0x38);
		write_client_reg(0xFD, 0x09);
		write_client_reg(0xFD, 0x0e);
		write_client_reg(0xFD, 0x0e);
		write_client_reg(0xFD, 0x12);
		write_client_reg(0xFD, 0x14);
		write_client_reg(0xFD, 0x12);
		write_client_reg(0xFD, 0x14);
		write_client_reg(0xFD, 0x13);
		write_client_reg(0xFD, 0x19);

		mddi_wait(120);
		write_client_reg(0x11, 0x00);
		mddi_wait(140);
		write_client_reg(0x29, 0x00);		
	}
}
EXPORT_SYMBOL(mddi_himax_ic_init_again);

int mddi_himax_workround_enable(void)
{
	if(lcdid == LCDID_TRULY)
		return 1;
	else
		return 0;
}
EXPORT_SYMBOL(mddi_himax_workround_enable);
#endif

static void pmic_set_lcd_intensity(int level)
{
	int ret;

	if (bl_pwm) {
		ret = pwm_config(bl_pwm, (PWM_PERIOD_USEC* level) / PWM_LEVEL ,
				PWM_PERIOD_USEC);
		if (ret) {
			pr_err("%s: level=%d, duty=%ld, period=%ld\n",
					__func__, level, (PWM_PERIOD_USEC* level) / PWM_LEVEL, PWM_PERIOD_USEC);
			pr_err("%s: pwm_config on pwm failed %d\n",
					__func__, ret);
			return;
		}

		ret = pwm_enable(bl_pwm);
		if (ret) {
			pr_err("%s: pwm_enable on pwm failed %d\n",
					__func__, ret);
			return;
		}
	}

#ifdef CONFIG_DEBUG_FS
	lcd_driver_state.bl_level = level;
#endif
}

static void mddi_ry002z_lcd_set_backlight(struct msm_fb_data_type *mfd)
{
	pr_info("%s %d\n", __func__, mfd->bl_level);
	pmic_set_lcd_intensity(mfd->bl_level);
}

static int mddi_ry002z_lcd_on(struct platform_device *pdev)
{
	mddi_ry002z_lcd_ic_init();

#ifdef CONFIG_DEBUG_FS
	lcd_driver_state.display_on = 1;
	lcd_driver_state.is_sleep = 0;
#endif
	pr_info("%s\n", __func__);

	return 0;
}

static int mddi_ry002z_lcd_off(struct platform_device *pdev)
{
	write_client_reg(0x28, 0x00);
	mddi_wait(5);
	write_client_reg(0x10, 0x00);
	mddi_wait(120);

	gpio_set_value(gpio_reset, low);
	mddi_wait(130);

#ifdef CONFIG_DEBUG_FS
	lcd_driver_state.display_on = 0;
	lcd_driver_state.is_sleep = 1;
#endif
	pr_info("%s\n", __func__);

	return 0;
}

static int __devinit mddi_ry002z_probe(struct platform_device *pdev)
{
	int rc = 0;

	pr_info("%s\n", __func__);
	if (pdev->id == 0) {
		mddi_ry002z_pdata = pdev->dev.platform_data;
		if (mddi_ry002z_pdata == NULL) {
			pr_err("%s: no PWM gpio specified\n", __func__);
			return 0;
		}
		led_pwm = mddi_ry002z_pdata->gpio_num[0];
		gpio_reset = mddi_ry002z_pdata->gpio_num[1];
		pr_info("%s: led_pwm=%d, gpio_reset=%d\n", __func__, led_pwm, gpio_reset);
		return 0;
	}
	if (mddi_ry002z_pdata == NULL)
		return -ENODEV;

	bl_pwm = pwm_request(led_pwm, "backlight");

	if (bl_pwm == NULL || IS_ERR(bl_pwm)) {
		pr_err("%s pwm_request() failed\n", __func__);
		bl_pwm = NULL;
	}
#ifdef CONFIG_FIX_BOOTUP_BLINK
        pmic_set_lcd_intensity(255);
#endif
	ry002z_fbpdev = msm_fb_add_device(pdev);
	if (!ry002z_fbpdev) {
		dev_err(&pdev->dev, "failed to add msm_fb device\n");
		rc = -ENODEV;
		goto probe_exit;
	}

probe_exit:
	return rc;

}

static struct platform_driver this_driver = {
	.probe  = mddi_ry002z_probe,
	.driver = {
		.name   = "mddi_ry002z",
	},
};

static struct msm_fb_panel_data mddi_ry002z_panel_data = {
	.on = mddi_ry002z_lcd_on,
	.off = mddi_ry002z_lcd_off,
	.set_backlight = mddi_ry002z_lcd_set_backlight,
};

static struct platform_device this_device = {
	.name   = "mddi_ry002z",
	.id	= 1,
	.dev	= {
		.platform_data = &mddi_ry002z_panel_data,
	}
};

static int __init mddi_ry002z_init(void)
{
	int ret;
	struct msm_panel_info *pinfo;

	mddi_lcdid_read();
	ret = platform_driver_register(&this_driver);
	if (ret)
		return ret;
	pinfo = &mddi_ry002z_panel_data.panel_info;
	pinfo->xres = 480;
	pinfo->yres = 800;
	MSM_FB_SINGLE_MODE_PANEL(pinfo);
	if((lcdid == LCDID_TRULY) ||(lcdid == LCDID_TDT))
		pinfo->bpp = 24;
	else
		pinfo->bpp = 18;
	pinfo->type = MDDI_PANEL;
	pinfo->wait_cycle = 0;
	pinfo->pdest = DISPLAY_1;
	pinfo->bl_max = PWM_LEVEL;
	pinfo->bl_min = 1;
	pinfo->fb_num = 3;
	pinfo->clk_rate = 245760000;
	
	if((lcdid == LCDID_TRULY) ||(lcdid == LCDID_TDT))
		pinfo->clk_min = 192000000;
	else if(lcdid == LCDID_SEIKO)
		pinfo->clk_min = 122880000;
	else
		pinfo->clk_min = 122880000;
		
	pinfo->clk_max = 445500000;

	pinfo->mddi.vdopkt = MDDI_DEFAULT_PRIM_PIX_ATTR;
       
	if ((lcdid == LCDID_TRULY) ||(lcdid == LCDID_TDT)) {
		pinfo->lcd.vsync_enable = TRUE;//Prevent tearing
		pinfo->lcd.refx100 = 6100;//adjust for tearing
		pinfo->lcd.v_back_porch = 12;
		pinfo->lcd.v_front_porch = 12;
		pinfo->lcd.v_pulse_width = 4;
	}
	else {
		pinfo->lcd.vsync_enable = FALSE;
		pinfo->lcd.refx100 = 6300;
		pinfo->lcd.v_back_porch = 12;
		pinfo->lcd.v_front_porch = 5;
		pinfo->lcd.v_pulse_width = 4;
	}
	pinfo->lcd.hw_vsync_mode = TRUE;
	pinfo->lcd.vsync_notifier_period = (1 / 60) * 1000000;
	pinfo->lcd.rev = 2;

	ret = platform_device_register(&this_device);

	if (ret)
		platform_driver_unregister(&this_driver);

	mddi_gpio_init();

	return ret;
}

module_init(mddi_ry002z_init);

#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>

static int lcd_sleep_set_value(void *data, u64 val)
{
	if (val){
		if (lcd_driver_state.is_sleep){
			printk("the lcd is in sleep status originally\n");
			return 0;
		}
		write_client_reg(0x28, 0x00);  
		mddi_wait(5);       
		write_client_reg(0x10, 0x00);  
		mddi_wait(120);       

		lcd_driver_state.display_on = 0;
		lcd_driver_state.is_sleep = 1;
	}else {
		if (!lcd_driver_state.is_sleep){
			printk("the lcd is not in sleep status originally\n");
			return 0;
		}
		write_client_reg(0x11, 0x00);
		mddi_wait(140);
		write_client_reg(0x29, 0x00);

		lcd_driver_state.display_on = 1;
		lcd_driver_state.is_sleep = 0;
	}
	return 0;
}

static int lcd_sleep_get_value(void *data, u64 *val)
{
	*val = lcd_driver_state.is_sleep;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(Tcl_lcd_sleep_fops, lcd_sleep_get_value, 
		lcd_sleep_set_value, "%llu\n");

static int lcd_disp_set_value(void *data, u64 val)
{
	if (val){
		if (lcd_driver_state.display_on){
			printk("the lcd is in display on mode originally\n");
			return 0;
		}
		write_client_reg(0x29, 0x00);

		lcd_driver_state.display_on = 1;
	}else{
		if (!lcd_driver_state.display_on){
			printk("the lcd is in display off mode originally\n");
			return 0;
		}
		write_client_reg(0x28, 0x00);

		lcd_driver_state.display_on = 0;
	}

	return 0;
}

static int lcd_disp_get_value(void *data, u64 *val)
{
	*val = lcd_driver_state.display_on;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(Tcl_lcd_disp_fops, lcd_disp_get_value, 
		lcd_disp_set_value, "%llu\n");

static int lcd_bl_set_value(void *data, u64 val)
{
	if (val < 30 || val > 255){
		printk("the value should in 30 ~ 255\n");
		return 0;
	}
	pmic_set_lcd_intensity(val);
	return 0;
}

static int lcd_bl_get_value(void *data, u64 *val)
{
	*val = lcd_driver_state.bl_level;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(Tcl_lcd_bl_fops, lcd_bl_get_value,
		lcd_bl_set_value, "%llu\n");

static int lcd_init_again(void *data, u64 val)
{
	if (val){
		mddi_ry002z_lcd_ic_init();
	}
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(Tcl_lcd_init_status, NULL,
		lcd_init_again, "%llu\n");

static int __init Tcl_lcd_debugfs_init(void)
{
	struct dentry *dent = NULL;
	dent = debugfs_create_dir("Tcl_lcd", 0);
	if (IS_ERR(dent))
		return 0;
	debugfs_create_file("sleep", 0644, dent, 0, &Tcl_lcd_sleep_fops);
	debugfs_create_file("display", 0644, dent, 0, &Tcl_lcd_disp_fops);
	debugfs_create_file("bl", 0644, dent, 0, &Tcl_lcd_bl_fops);
	debugfs_create_file("re_init", 0644, dent, 0, &Tcl_lcd_init_status);

	return 0;
}

device_initcall(Tcl_lcd_debugfs_init);

#endif

MODULE_DESCRIPTION("Driver for Seiko ry002z");
MODULE_LICENSE("GPL");
