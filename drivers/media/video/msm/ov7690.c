/* kernel/drivers/media/video/msm/ov7690.c
 *
 * XXX　devices driver
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

/*
 * 2010-04-13 Inited based driver by LiuLi 
 */

#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <media/msm_camera.h>
#include <mach/gpio.h>
#include <mach/camera.h>
#include "ov7690.h"
#include "ov7690_init_reg.c"


/*=========================== MACRO        ======================*/
#define DBG_ENABLE	1
#if DBG_ENABLE
#define DBG(fmt, args...)	printk(KERN_INFO "msm_camera ov7690: " fmt, ##args)
#define DBG_IN	DBG("%s enter \n",  __FUNCTION__);
#define DBG_OUT	DBG("%s leavel \n", __FUNCTION__);
#define DBG_OUT_RC	DBG("%s leavel rc =%d  \n", __FUNCTION__, rc);

#else
#define DBG(fmd, args...)	do {} while(0)
#define DBG_IN	
#define DBG_OUT
#define DBG_OUT_RC
#endif

#define	Q8	0x00000100

#define REG_OV7690_MODEL_ID_MSB			0x0A	
#define REG_OV7690_MODEL_ID_LSB			0x0B
#define OV7690_MODEL_ID				0x7691
/*============================ Data struct ======================*/
struct ov7690_work_t {
	struct work_struct work;
};
static struct ov7690_work_t *ov7690_sensorw;

struct ov7690_ctrl_t {
	const struct 	msm_camera_sensor_info *sensordata;
	uint32_t	sensormode;
	uint32_t	fps_divider;
	uint32_t	pict_fps_divider;
	uint32_t	fps;
	uint32_t	curr_lens_pos;
	uint32_t	curr_step_pos;
	uint32_t	my_reg_gain;
	uint32_t	my_reg_line_count;
	uint32_t	total_lines_per_frame;
	enum ov7690_resolution_t	prev_res; //preview resolution
	enum ov7690_resolution_t	pict_res;
	enum ov7690_resolution_t	curr_res;
	enum ov7690_resolution_t	set_test; //just used for test mode
	unsigned short	imgaddr;
};
static struct ov7690_ctrl_t * ov7690_ctrl;
static struct i2c_client    *ov7690_client;

DEFINE_MUTEX(ov7690_mutex);
/*================================ FUNCTION ========================*/
/*------------- I2C FUNCTION ------------------------------*/
static const struct i2c_device_id ov7690_i2c_id[] = {
		{"ov7690", 0},
		{}
};

static int ov7690_i2c_probe(struct i2c_client *client, const struct i2c_device_id* id)
{
	int rc = 0;
DBG_IN
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)){
		DBG("i2c_check_functionality failed \n");
		rc = -EFAULT;	
		goto probe_failure;
	}

	ov7690_sensorw = kzalloc(sizeof(struct ov7690_work_t), GFP_KERNEL);
	if (NULL == ov7690_sensorw){
		DBG("kzalloc ov7690 failed\n");
		rc = -ENOMEM;
		goto probe_failure;
	}
	
	i2c_set_clientdata(client, ov7690_sensorw);
	ov7690_client = client;
	DBG(" probe success !\n");
probe_failure:
	
DBG_OUT_RC	
	return rc;
}

static int __exit ox7690_remove(struct i2c_client *client)
{
	struct ov7690_work_t *sensorw = i2c_get_clientdata(client);

	free_irq(client->irq, sensorw);
	ov7690_client = NULL;
	kfree(sensorw);
	return 0;
}

static struct i2c_driver ov7690_i2c_driver = {
	.id_table	= ov7690_i2c_id,
	.probe		= ov7690_i2c_probe,
	.remove		= __exit_p(ov7690_i2c_remove),
	.driver		= {
				.name = "ov7690",
			},
};

static int ov7690_i2c_rxdata(unsigned short saddr, unsigned char* rxdata, int length)
{
        struct i2c_msg msgs[] = {
                {
                        .addr  = saddr,
                        .flags = 0,
                        .len   = 1,
                        .buf   = rxdata,
                },
                {
                        .addr  = saddr,
                        .flags = I2C_M_RD,
                        .len   = 1,
                        .buf   = rxdata,
                },
        };
        if (i2c_transfer(ov7690_client->adapter, msgs, 2) < 0) {
                DBG("ov7690_i2c_rxdata failed!\n");
                return -EIO;
        }
        return 0;
}

static int32_t ov7690_i2c_txdata(unsigned short saddr,
                                unsigned char *txdata, int length)
{
        struct i2c_msg msg[] = {
                {
                        .addr = saddr,
                        .flags = 0,
                        .len = 2,
                        .buf = txdata,
                 },
        };
        if (i2c_transfer(ov7690_client->adapter, msg, 1) < 0) {
                DBG("ov7690_i2c_txdata faild 0x%x\n", ov7690_client->addr);
                return -EIO;
        }

        return 0;
}

static int32_t ov7690_i2c_read(uint8_t raddr,
        uint8_t *rdata, int rlen)
{
        int32_t rc = 0;
        unsigned char buf[1];
        if (!rdata)
                return -EIO;
        memset(buf, 0, sizeof(buf));
        buf[0] = raddr;
        rc = ov7690_i2c_rxdata(ov7690_client->addr >> 1, buf, rlen);
        if (rc < 0) {
                DBG("ov7690_i2c_read 0x%x failed!\n", raddr);
                return rc;
        }
        *rdata = buf[0];
        return rc;
}

static int32_t ov7690_i2c_write_b_sensor(uint8_t waddr, uint8_t bdata)
{
        int32_t rc = -EFAULT;
        unsigned char buf[2];
        memset(buf, 0, sizeof(buf));
        buf[0] = waddr;
        buf[1] = bdata;
//    DBG("i2c_write_b addr = 0x%x, val = 0x%x\n", waddr, bdata);
        rc = ov7690_i2c_txdata(ov7690_client->addr >>1, buf, 2);
        if (rc < 0)
                DBG("i2c_write_b failed, addr = 0x%x, val = 0x%x\n",
                        waddr, bdata);
        return rc;
}

static int32_t ov7690_i2c_write_table(struct ov7690_i2c_reg_conf * reg_cfg_tbl, int num)
{
	int32_t rc = -EIO;
	int i;
	for (i = 0; i<num; i++){
		if (reg_cfg_tbl->baddr == 0xee){
			mdelay(reg_cfg_tbl->bdata);
		       reg_cfg_tbl++;			
                    continue;
		}
		
		rc = ov7690_i2c_write_b_sensor(reg_cfg_tbl->baddr, reg_cfg_tbl->bdata);
		if (rc < 0)
			break;
		reg_cfg_tbl++;
	}
	return rc;
}
/*------------------- Sensor -------------------*/
static int32_t ov7690_probe_init_done(const struct msm_camera_sensor_info *data)
{
	/*power down active high with internel pull down resistor
	PWDN should be connected to ground outside of module if unused*/
	gpio_direction_output(data->sensor_pwd, 1);
	mdelay(20);
	gpio_direction_output(data->sensor_pwd, 0);
	return 0;
}

static int ov7690_probe_init_sensor(const struct msm_camera_sensor_info *data)
{
	uint8_t  model_id_msb = 0;
	uint8_t  model_id_lsb = 0;
	uint16_t model_id = 0;
	int32_t	 rc = 0;
DBG_IN
	//reset sensor
	gpio_direction_output(data->sensor_pwd, 0);
	mdelay(20);

	//read sensor model id
	rc = ov7690_i2c_read(REG_OV7690_MODEL_ID_MSB, &model_id_msb, 1);
	if (rc < 0)
		goto init_probe_fail;
	rc = ov7690_i2c_read(REG_OV7690_MODEL_ID_LSB, &model_id_lsb, 1);
	if (rc < 0)
		goto init_probe_fail;
	model_id = (model_id_msb << 8) | (model_id_lsb & 0x00FF);
	DBG("%s modle_id 0x%x  msb 0x%x  lsb 0x%x\n", __func__, model_id, model_id_msb, model_id_lsb);
	//compare sensor id to OV7690_ID
	if (OV7690_MODEL_ID != model_id){
		rc = -ENODEV;
		goto init_probe_fail;
	}
	goto init_probe_done;

init_probe_fail:
	DBG(" ov7690_prove_init_sensor fail \n");
	ov7690_probe_init_done(data);
init_probe_done:
	DBG(" ov7690_probe_init_sensor done \n");
DBG_OUT_RC
	return rc;	
}

static int32_t ov7690_sensor_setting(enum ov7690_reg_update update_type, enum ov7690_setting rt){
	int32_t rc = 0;

DBG_IN	
	switch (update_type){
	case UPDATE_PERIODIC:
	  if (rt == RES_PREVIEW || rt == RES_CAPTURE){
		rc = ov7690_i2c_write_table(&tbl_1[0], ARRAY_SIZE(tbl_1));
		if (rc < 0)
			return rc;
	  }
	  break;
	case REG_INIT:
//          ov7690_i2c_write_b_sensor(0x0e, 0x08);
	  break;
	default:
	  rc = -EINVAL;
          break;
	}
DBG_OUT_RC
	return rc;
}

static int ov7690_sensor_open_init(const struct msm_camera_sensor_info *data)
{
	int rc = 0;
	
DBG_IN
	ov7690_ctrl = kzalloc(sizeof(struct ov7690_ctrl_t), GFP_KERNEL);
	if (NULL == ov7690_ctrl){
		DBG("ov7690_ctrl kzalloc failed\n");
		rc = -ENOMEM;
		goto init_done;
	}
	ov7690_ctrl->fps_divider = 1 * 0x00000400;
	ov7690_ctrl->pict_fps_divider = 1 * 0x00000400;
	ov7690_ctrl->set_test = TEST_OFF;
	ov7690_ctrl->prev_res = QTR_SIZE;
	ov7690_ctrl->pict_res = FULL_SIZE;
	ov7690_ctrl->curr_res = QTR_SIZE;

	if (NULL != data)
		ov7690_ctrl->sensordata = data;

	//enable MCLK
	msm_camio_clk_rate_set(24000000);
	mdelay(20);
        //reset
	msm_camio_camif_pad_reg_reset();
	mdelay(20);
	
	rc = ov7690_probe_init_sensor(data);
	if (rc < 0){
		DBG("ov7690 failed for probe_init_sersnor");
		goto init_fail;
	}
	rc = ov7690_sensor_setting(REG_INIT, RES_PREVIEW);
	if (rc < 0){
		goto init_fail;
	}
	else
		goto init_done;

init_fail:
	DBG("%s fail!\n", __func__);
	ov7690_probe_init_done(data);
	kfree(ov7690_ctrl);
	ov7690_ctrl = NULL;

init_done:
	DBG("%s done !", __func__);
DBG_OUT_RC
	return rc;
}

static int32_t ov7690_test_SCCB(void){
	int rc = 0;
DBG_IN
	{
		uint8_t old13;
		uint8_t temp;

		ov7690_i2c_read(0x13, &old13, 1);
		ov7690_i2c_write_b_sensor(0x13, 0xf0);
		ov7690_i2c_write_b_sensor(0x00, 0x55);
		ov7690_i2c_read(0x00, &temp, 1);
		if (temp != 0x55){
			DBG("%s test read error \n", __func__);
			return rc;
		}
		ov7690_i2c_write_b_sensor(0x00, 0xaa);
		ov7690_i2c_read(0x00, &temp, 1);
		if (temp != 0xaa){
			DBG("%s test error!", __func__);
			return rc;
		}
		ov7690_i2c_write_b_sensor(0x13, old13);
		DBG("%s test result is ok\n", __func__);
	}
DBG_OUT_RC
	return 0;
}

static int32_t ov7690_test_HREF(void)
{
	int rc = 0;
#define HREF  13 //GPIO13 for HSYNC

DBG_IN
	ov7690_i2c_write_b_sensor(0X12, 0X80);
	mdelay(3);
	ov7690_i2c_write_b_sensor(0x18, 0x00);
	mdelay(40);
	if (gpio_get_value(HREF) != 0){
		DBG("%s test href is error for high \n", __func__);
		return -1;
	}
	ov7690_i2c_write_b_sensor(0x28, 0x10);
	mdelay(40);
	if (gpio_get_value(HREF) != 1){
		DBG("%s test href is error for low \n", __func__);
		return -1;
	}
DBG_OUT_RC
	return rc;
}
static int32_t ov7690_test_PCLK(void){
	int rc = 0;
#define PCLK	12 //GPIO 12 for PCLK
DBG_IN
	ov7690_i2c_write_b_sensor(0x12, 0x80);
	mdelay(3);
	ov7690_i2c_write_b_sensor(0x18, 0x00);
	ov7690_i2c_write_b_sensor(0x3e, 0x40);
	mdelay(40);
	if (gpio_get_value(PCLK) != 1){
		DBG("%s test PCLK error for high\n", __func__);
		return -1;
	}
	ov7690_i2c_write_b_sensor(0x3f, 0x00);
	mdelay(40);
	if (gpio_get_value(PCLK) != 0){
		DBG("%s test PCLK error for low\n", __func__);
		return -1;
	}
DBG_OUT_RC
	return rc;
}
static int32_t ov7690_test_VSYNC(void){
	int rc = 0;
	int i = 0;
	int temp = 0;
#define VSYNC  14 //gpio 14 for vsync
DBG_IN
	ov7690_i2c_write_b_sensor(0x12, 0x80);
	mdelay(3);
	for (i=0; i<5; i++){
		if (gpio_get_value(VSYNC) == 0)
		temp++;
	}
	if (temp <4){
		DBG("%s test vsync error for high", __func__);
		return -1;
	}
	ov7690_i2c_write_b_sensor(0x28, 0x02);
	mdelay(40);
	temp = 0;
	for(i=0; i<5; i++){
		if (gpio_get_value(VSYNC)==1)
			temp++;
	}
	if (temp <4){
		DBG("%s test vsync error for low", __func__);
		return -1;
	}	
DBG_OUT_RC
	return rc;
}
static int32_t ov7690_test(enum ov7690_test_mode_t teston)
{
	int rc = 0;
DBG_IN
	if (teston == TEST_OFF)
		return rc;
	else{
		rc = ov7690_test_SCCB();
		rc += ov7690_test_HREF();
		rc += ov7690_test_PCLK();
		rc += ov7690_test_VSYNC();
		ov7690_ctrl->set_test = TEST_OFF;
	}
DBG_OUT_RC
	return rc;		

}
static int32_t ov7690_video_config(int mode, int res)
{
	int32_t rc = 0;
DBG_IN
	DBG("%s mode %d res %d \n", __func__, mode, res);
	switch (res){
	case S_QTR_SIZE:
		rc = ov7690_sensor_setting(UPDATE_PERIODIC, RES_PREVIEW);
		if (rc < 0){
			DBG("%s config preview failed \n", __func__);
			return rc;
		}
		break;
	case S_FULL_SIZE:
		rc = ov7690_sensor_setting(UPDATE_PERIODIC, RES_CAPTURE);
		if (rc < 0){
			DBG("%s config capture failed \n", __func__);
			return rc;
		}
		break;
	default:
		return 0;
	}
	if (ov7690_ctrl->set_test) {
		if (ov7690_test(ov7690_ctrl->set_test) < 0){
			DBG("%s test failed, exit\n", __func__);
			return  rc;
		}
	}	
	ov7690_ctrl->prev_res = res;
	ov7690_ctrl->curr_res = res;
	ov7690_ctrl->sensormode = mode;
DBG_OUT_RC
	return rc;
}
static int32_t ov7690_snapshot_config(int mode)
{
DBG_IN
	ov7690_ctrl->curr_res = ov7690_ctrl->pict_res;
	ov7690_ctrl->sensormode = mode;
DBG_OUT
	return 0;
}

static int32_t ov7690_raw_snapshot_config(int mode)
{
DBG_IN
	ov7690_ctrl->curr_res = ov7690_ctrl->pict_res;
	ov7690_ctrl->sensormode = mode;
DBG_OUT
	return 0;
}

static int32_t ov7690_set_sensor_mode(int mode, int res)
{
	int32_t rc = 0;
DBG_IN
	switch (mode){
	case SENSOR_PREVIEW_MODE:
		rc = ov7690_video_config(mode, res);
		break;
	case SENSOR_SNAPSHOT_MODE:
		rc = ov7690_snapshot_config(mode);
		break;
	case SENSOR_RAW_SNAPSHOT_MODE:
		rc = ov7690_raw_snapshot_config(mode);
		break;
	default:
		rc = - EINVAL;
		break;
	}
DBG_OUT_RC
	return rc;	
}
static int32_t ov7690_power_down(void)
{
DBG_IN
DBG_OUT
	return 0;
}
static int ov7690_sensor_config(void __user *argp)
{
	int rc = 0;
	struct sensor_cfg_data cdata;

DBG_IN
	if (copy_from_user(&cdata, (void *)argp, sizeof(struct sensor_cfg_data))){
		DBG("%s copy from user fail\n", __func__);
		return -EFAULT;
	}
	
	mutex_lock(&ov7690_mutex);
	DBG("%s cfgtype = %d \n", __func__, cdata.cfgtype);
	switch (cdata.cfgtype){
	case CFG_SET_MODE:
		rc = ov7690_set_sensor_mode(cdata.mode, cdata.rs);
		break;
	case CFG_PWR_DOWN:
		rc = ov7690_power_down();
		break;
	default:
		rc = -EFAULT;
		break;
	}
	mutex_unlock(&ov7690_mutex);
DBG_OUT_RC
	return rc;
}

static int ov7690_sensor_release(void)
{
	int rc = 0;
DBG_IN
DBG_OUT_RC
	return rc;
}

static int ov7690_sensor_probe(const struct msm_camera_sensor_info *info, struct msm_sensor_ctrl *s)
{
	int rc = 0;
DBG_IN
	rc = i2c_add_driver(&ov7690_i2c_driver);
	if (rc < 0 || (NULL == ov7690_client)){
		rc = -ENOTSUPP;
		goto probe_fail;
	}
	//set MCLK
	msm_camio_clk_rate_set(24000000);
	//init sensor
	rc = ov7690_probe_init_sensor(info);
	if(rc < 0)
		goto probe_fail;
	s->s_init        = ov7690_sensor_open_init;
	s->s_release     = ov7690_sensor_release;
	s->s_config  	 = ov7690_sensor_config;
	s->s_camera_type = FRONT_CAMERA_2D;
    //pr 252661, gao chao, modified to meet requirement of framework to solove 3th app( such as skype) preview problem
    s->s_mount_angle = 270;
	goto probe_done;
	DBG("%s probe success!\n", __func__);

probe_fail:
	DBG("%s ov7690_sensor_probe fail! \n", __func__);
	i2c_del_driver(&ov7690_i2c_driver);
probe_done:
DBG_OUT_RC
	return rc;
}

static int __ov7690_probe(struct platform_device *pdev)
{
	DBG_IN
	return msm_camera_drv_start(pdev, ov7690_sensor_probe);
}
static int __ov7690_remove(struct platform_device *pdev)
{
	DBG_IN
	return 0;
}
static void __ov7690_shutdown(struct platform_device *pdev)
{
	DBG_IN
}
static int __ov7690_suspend(struct platform_device *pdev, pm_message_t state)
{
	DBG_IN
	return 0;
}
static int __ov7690_resume(struct platform_device *pdev)
{
	DBG_IN
	return 0;
}
static struct platform_driver msm_camera_ov7690_driver = {
	.probe	= __ov7690_probe,
	.remove = __ov7690_remove,
	.shutdown = __ov7690_shutdown,
	.suspend = __ov7690_suspend,
	.resume  = __ov7690_resume,
	.driver	= {
			.name  = "msm_camera_ov7690",
			.owner = THIS_MODULE, 
		},
};
static int __init ov7690_init(void){
	return platform_driver_register(&msm_camera_ov7690_driver);
}

module_init(ov7690_init);
MODULE_DESCRIPTION("OMNI VGA YUV sensor driver");
MODULE_LICENSE("GPL V2");
