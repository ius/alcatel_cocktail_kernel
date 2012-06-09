/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */

#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <media/msm_camera.h>
#include <mach/gpio.h>
#include <mach/camera.h>
#include <mach/vreg.h>
#include <linux/slab.h>
#include "ov8820.h"

/*=============================================================
	SENSOR REGISTER DEFINES
==============================================================*/
#define Q8    0x00000100

/* Omnivision8820 product ID register address */
#define OV8820_PIDH_REG                       0x300A
#define OV8820_PIDL_REG                       0x300B
/* Omnivision8820 product ID */
#define OV8820_PID                    0x88
/* Omnivision8820 version */
#define OV8820_VER                    0x10
/* Time in milisecs for waiting for the sensor to reset */
#define OV8820_RESET_DELAY_MSECS    66
#define OV8820_DEFAULT_CLOCK_RATE   12000000
/* Registers*/
#define OV8820_GAIN         0x3000
#define OV8820_AEC_MSB      0x3002
#define OV8820_AEC_LSB      0x3003
#define OV8820_AF_MSB       0x30EC
#define OV8820_AF_LSB       0x30ED
/* AF Total steps parameters */
#define OV8820_STEPS_NEAR_TO_CLOSEST_INF  43
#define OV8820_TOTAL_STEPS_NEAR_TO_FAR    43
/*Test pattern*/
/* Color bar pattern selection */
#define OV8820_COLOR_BAR_PATTERN_SEL_REG      0x307B
/* Color bar enabling control */
#define OV8820_COLOR_BAR_ENABLE_REG           0x307D
/* Time in milisecs for waiting for the sensor to reset*/
#define OV8820_RESET_DELAY_MSECS    66
/* I2C Address of the Sensor */
#define OV8820_I2C_SLAVE_ID    0x6c

#define DBG(fmt, args...)       printk(KERN_INFO "msm_camera ov8820: " fmt, ##args)

#define DBG_IN	printk(KERN_ERR "%s enter \n", __func__);
#define DBG_OUT printk(KERN_ERR "%s level \n",__func__);
#define DBG_OUT_RC printk(KERN_ERR "%s level rc %d \n", __func__, rc);

#ifdef CDBG
#undef CDBG
#define CDBG	DBG
#endif 
/*============================================================================
							 TYPE DECLARATIONS
============================================================================*/

typedef unsigned char byte;
/* 16bit address - 8 bit context register structure */
typedef struct reg_addr_val_pair_struct
{
	uint16_t  reg_addr;
	byte      reg_val;
}reg_struct_type;

enum ov8820_test_mode_t {
	TEST_OFF,
	TEST_1,
	TEST_2,
	TEST_3
};

enum ov8820_resolution_t {
	QTR_SIZE,
	FULL_SIZE,
	INVALID_SIZE
};


/*============================================================================
							DATA DECLARATIONS
============================================================================*/
/*  24M MCLK 480M MipiClk 9.76fps 10bit/2lane */
//static int counter = 0;
static reg_struct_type ov8820_init_settings_array[] =
{
	{0x3012, 0x80},
	{0x360b, 0x0c},
	{0x3100, 0x06},
	{0x30fa, 0x00},
	{0x3302, 0x20},
	{0x30b2, 0x10},
	{0x30a0, 0x40},
	{0x3098, 0x24},
	{0x3099, 0x81},
	{0x309a, 0x64},
	{0x309b, 0x00},
	{0x309d, 0x64},
	{0x3321, 0x02},
	{0x3322, 0x04},
	{0x3331, 0x08},
	{0x3332, 0x08},
	{0x3306, 0x00},
	{0x3316, 0x03},
	{0x3058, 0x01},
	{0x3059, 0xa0},
	{0x306b, 0x00},
	{0x3067, 0x40},
	{0x3069, 0x80},
	{0x3334, 0x02},
	{0x3333, 0x41},
	{0x3091, 0x00},
	{0x3006, 0x00},
	{0x331e, 0x94},
	{0x331f, 0x6e},
	{0x3094, 0x01},
	{0x30ab, 0x44},
	{0x3095, 0x0a},
	{0x3082, 0x00},
	{0x3080, 0x40},
	{0x30aa, 0x59},
	{0x30a9, 0x00},
	{0x30be, 0x08},
	{0x309f, 0x23},
	{0x3065, 0x40},
	{0x3068, 0x00},
	{0x30bf, 0x80},
	{0x309c, 0x00},
	{0x3084, 0x44},
	{0x3016, 0x03},
	{0x30e9, 0x09},
	{0x3075, 0x29},
	{0x3076, 0x29},
	{0x3077, 0x29},
	{0x3078, 0x29},
	{0x306a, 0x05},
	{0x333e, 0x00},
	{0x3087, 0x41},
	{0x30e3, 0x0e},
	{0x30f0, 0x00},
	{0x30f2, 0x00},
	{0x30f4, 0x90},
	{0x3092, 0x08},
	{0x3090, 0x97},
	{0x30fb, 0xc9},
	{0x3100, 0x88},
	{0x3101, 0x77},
	{0x3601, 0x16},
	{0x3300, 0xef}, //manul awb and ae,close lenc,open wbc and blc(BLC target 08)
	{0x3320, 0xc2}, //manual awb
	{0x3324, 0x40},
	{0x3325, 0x40},
	{0x3326, 0x40},
	{0x3327, 0x00},
	{0x3328, 0x00},
	{0x3013, 0xc0}, //manual AE
	{0x3000, 0x7f}, //gain
	{0x3002, 0x01}, //exp
	{0x3003, 0x10},
	{0x30fa, 0x01}
};

/*1632x1224; 24M MCLK 480M MipiClk 9.76fps 10bit/2lane*/
static reg_struct_type ov8820_qtr_settings_array[] =
{
	{0x3329, 0xe3},
	{0x3079, 0x0a},
	{0x30b3, 0x08},
	{0x33e5, 0x00},
	{0x30f8, 0x45},
	{0x3020, 0x04},
	{0x3021, 0xf4},
	{0x3022, 0x09},
	{0x3023, 0xD8},
	{0x3024, 0x00},
	{0x3025, 0x04},
	{0x3026, 0x00},
	{0x3027, 0x00},
	{0x3028, 0x0c},
	{0x3029, 0xDB},
	{0x302a, 0x09},
	{0x302b, 0x9F},
	{0x302c, 0x06},
	{0x302d, 0x60},
	{0x302e, 0x04},
	{0x302f, 0xC8},
	{0x307e, 0x00},
	{0x3301, 0x0b},
	{0x331c, 0x00},
	{0x331d, 0x00},
	{0x308a, 0x02},
	{0x3015, 0x33},
	{0x3018, 0x44},
	{0x3019, 0x38},
	{0x301a, 0x82},
	{0x3305, 0x80},
	{0x3072, 0x0d},
	{0x3319, 0x04},
	{0x309e, 0x09},
	{0x3071, 0x40},
	{0x3347, 0x00},
	{0x308d, 0x02},
	{0x300e, 0x15},
	{0x300F, 0x14},
	{0x3010, 0x28},
	{0x3011, 0x22}
};
/* 3280x2456 Sensor Raw; 24M MCLK 480M MipiClk 9.76fps 10bit/2lane*/
static reg_struct_type ov8820_full_settings_array[] =
{
	{0x3329, 0xe3},
	{0x3079, 0x0a},
	{0x30b3, 0x08},
	{0x33e5, 0x00},
	{0x30f8, 0x40},
	{0x3020, 0x09},
	{0x3021, 0xc4},
	{0x3022, 0x0f},
	{0x3023, 0x88},
	{0x3024, 0x00},
	{0x3025, 0x00},
	{0x3026, 0x00},
	{0x3027, 0x00},
	{0x3028, 0x0c},
	{0x3029, 0xdf},
	{0x302a, 0x09},
	{0x302b, 0x9f},
	{0x302c, 0x0c},
	{0x302d, 0xd0},
	{0x302e, 0x09},
	{0x302f, 0x98},
	{0x307e, 0x00},
	{0x3301, 0x0b},
	{0x331c, 0x28},
	{0x331d, 0x21},
	{0x308a, 0x01},
	{0x3015, 0x33},
	{0x3018, 0x44},
	{0x3019, 0x38},
	{0x301a, 0x82},
	{0x3305, 0x80},
	{0x3072, 0x01},
	{0x3319, 0x08},
	{0x309e, 0x1b},
	{0x3071, 0x40},
	{0x3347, 0x00},
	{0x308d, 0x02},
	{0x300e, 0x15},
	{0x300F, 0x14},
	{0x3010, 0x28},
	{0x3011, 0x22}
};

uint32_t OV8820_FULL_SIZE_DUMMY_PIXELS =    0;
uint32_t OV8820_FULL_SIZE_DUMMY_LINES  =    0;
uint32_t OV8820_FULL_SIZE_WIDTH        = 3280; //3280;
uint32_t OV8820_FULL_SIZE_HEIGHT       = 2456; //2456;

uint32_t OV8820_QTR_SIZE_DUMMY_PIXELS  =    0;
uint32_t OV8820_QTR_SIZE_DUMMY_LINES   =    0;
uint32_t  OV8820_QTR_SIZE_WIDTH         = 1632;
uint32_t OV8820_QTR_SIZE_HEIGHT        = 1224;

uint32_t OV8820_HRZ_FULL_BLK_PIXELS    = 696; 
uint32_t OV8820_VER_FULL_BLK_LINES     =  44; //44;
uint32_t OV8820_HRZ_QTR_BLK_PIXELS     = 888; //890;
uint32_t OV8820_VER_QTR_BLK_LINES      =  44;

/* AF Tuning Parameters */
static uint16_t ov8820_step_position_table[OV8820_TOTAL_STEPS_NEAR_TO_FAR+1];
static uint8_t ov8820_damping_threshold = 10;
static uint8_t ov8820_damping_course_step = 4;
static uint8_t ov8820_damping_fine_step = 10;
static uint16_t ov8820_focus_debug = 0;
static uint16_t ov8820_use_default_damping = 1;
static uint16_t ov8820_use_threshold_damping = 1;//set to FALSE if too slow
static uint32_t stored_line_length_ratio = 1 * Q8;
static uint8_t ov8820_damping_time_wait;
uint8_t S3_to_0 = 0x1; 
/* static Variables*/
static uint16_t step_position_table[OV8820_TOTAL_STEPS_NEAR_TO_FAR+1];
static uint16_t csi_config;
/* FIXME: Changes from here */
struct ov8820_work_t {
	struct work_struct work;
};
static struct  ov8820_work_t *ov8820_sensorw;
static struct  i2c_client *ov8820_client;
struct ov8820_ctrl_t {
	const struct  msm_camera_sensor_info *sensordata;
	uint32_t sensormode;
	uint32_t fps_divider; 		/* init to 1 * 0x00000400 */
	uint32_t pict_fps_divider; 	/* init to 1 * 0x00000400 */
	uint16_t fps;
	int16_t  curr_lens_pos;
	uint16_t curr_step_pos;
	uint16_t my_reg_gain;
	uint32_t my_reg_line_count;
	uint16_t total_lines_per_frame;
	enum ov8820_resolution_t prev_res;
	enum ov8820_resolution_t pict_res;
	enum ov8820_resolution_t curr_res;
	enum ov8820_test_mode_t  set_test;
	unsigned short imgaddr;
};
static struct ov8820_ctrl_t *ov8820_ctrl;
static DECLARE_WAIT_QUEUE_HEAD(ov8820_wait_queue);
DECLARE_MUTEX(ov8820_sem);

/*=============================================================*/
static int ov8820_i2c_rxdata(unsigned short saddr,
	unsigned char *rxdata, int length)
{
	struct i2c_msg msgs[] = {
	{
//		.addr  = saddr << 1,
		.addr  = saddr,
		.flags = 0,
		.len   = 2,
		.buf   = rxdata,
	},
	{
//		.addr  = saddr << 1,
		.addr  = saddr,
		.flags = I2C_M_RD,
		.len   = length,
		.buf   = rxdata,
	},
	};

	if (i2c_transfer(ov8820_client->adapter, msgs, 2) < 0) {
		CDBG("ov8820_i2c_rxdata failed!\n");
		return -EIO;
	}

	return 0;
}
static int32_t ov8820_i2c_txdata(unsigned short saddr,
				unsigned char *txdata, int length)
{
	struct i2c_msg msg[] = {
		{
		 .addr = saddr << 1,
		 .flags = 0,
		 .len = length,
		 .buf = txdata,
		 },
	};

	if (i2c_transfer(ov8820_client->adapter, msg, 1) < 0) {
		CDBG("ov8820_i2c_txdata faild 0x%x\n", ov8820_client->addr);
		return -EIO;
	}

	return 0;
}


static int32_t ov8820_i2c_read(unsigned short raddr,
				unsigned short *rdata, int rlen)
{
	int32_t rc = 0;
	unsigned char buf[2];

	if (!rdata)
		return -EIO;

	memset(buf, 0, sizeof(buf));

	buf[0] = (raddr & 0xFF00) >> 8;
	buf[1] = (raddr & 0x00FF);
DBG("%s client->addr 0x%x\n", __func__, ov8820_client->addr);
//	rc = ov8820_i2c_rxdata(ov8820_client->addr, buf, rlen);
	rc = ov8820_i2c_rxdata(0x6c>>1, buf, rlen);
	if (rc < 0) {
		CDBG("ov8820_i2c_read 0x%x failed!\n", raddr);
		return rc;
	}

	*rdata = (rlen == 2 ? buf[0] << 8 | buf[1] : buf[0]);

	return rc;

}
static int32_t ov8820_i2c_write_b(unsigned short saddr,unsigned short waddr, uint8_t bdata)
{
	int32_t rc = -EFAULT;
	unsigned char buf[3];

	memset(buf, 0, sizeof(buf));
	buf[0] = (waddr & 0xFF00) >> 8;
	buf[1] = (waddr & 0x00FF);
	buf[2] = bdata;

	CDBG("i2c_write_b addr = 0x%x, val = 0x%x\n", waddr, bdata);
	rc = ov8820_i2c_txdata(saddr, buf, 3);

	if (rc < 0) {
		CDBG("i2c_write_b failed, addr = 0x%x, val = 0x%x!\n",
			 waddr, bdata);
	}

	return rc;
}
static int32_t ov8820_af_i2c_write(uint16_t data)
{
	uint8_t code_val_msb, code_val_lsb;//, S3_to_0;
	uint32_t rc = 0;
	//S3_to_0 = 0x9; /* S[3:0] */
	code_val_msb = data >> 4; /* D[9:4] */
	code_val_lsb = ((data & 0x000F) << 4) | S3_to_0;
	CDBG("code value = %d ,D[9:4] = %d ,D[3:0] = %d", data, code_val_msb, code_val_lsb);
	rc = ov8820_i2c_write_b(0x6C >>2 , OV8820_AF_MSB, code_val_msb);
	if (rc < 0) {
		CDBG("Unable to write code_val_msb = %d\n", code_val_msb);
		return rc;
	}
	rc = ov8820_i2c_write_b( 0x6C >>2, OV8820_AF_LSB, code_val_lsb);
	if (rc < 0) {
		CDBG("Unable to write code_val_lsb = %disclaimer\n", code_val_lsb);
		return rc;
	}

	return rc;
} /* ov8820_af_i2c_write */
static struct vreg *vreg_vcm_vdd   = NULL;   //which match vreg_L-13    wlan1

static int32_t ov8820_poweron_af(void)
{
	int32_t rc = 0;

	CDBG("%s enter \n", __func__);

	if (NULL == vreg_vcm_vdd){
		vreg_vcm_vdd = vreg_get(NULL, "wlan1");
		if (IS_ERR(vreg_vcm_vdd)){
			CDBG("%s get vreg_vcm_vdd faile\n", __func__);
			return -1;
		}
	}
	rc = vreg_enable(vreg_vcm_vdd);
	return rc;
}
static int32_t ov8820_poweroff_af(void)
{
	int32_t rc = 0;

	CDBG("%s enter \n", __func__);
	if (NULL == vreg_vcm_vdd){
		vreg_vcm_vdd = vreg_get(NULL, "wlan1");
		if (IS_ERR(vreg_vcm_vdd)){
			CDBG("%s get vreg_vcm_vdd faile\n", __func__);
			return -1;
		}
	}
	rc = vreg_disable(vreg_vcm_vdd);

	return rc;

}
static int32_t ov8820_move_focus( int direction,
	int32_t num_steps)
{

	int8_t step_direction;
	int8_t dest_step_position;
	uint16_t dest_lens_position, target_dist, small_step;
	int16_t next_lens_position;
	int32_t rc = 0;

DBG_IN
	if (num_steps == 0) {
		return rc;
	}

	if ( direction == MOVE_NEAR ) {
		step_direction = 1;
	}
	else if ( direction == MOVE_FAR) {
		step_direction = -1;
	}
	else {
		CDBG("Illegal focus direction\n");
		return -EINVAL;;//CAMERA_INVALID_PARM;
	}

	CDBG("interpolate/n");
	dest_step_position = ov8820_ctrl->curr_step_pos + (step_direction * num_steps);
	if (dest_step_position < 0) {
		dest_step_position = 0;
	}
	else if (dest_step_position > OV8820_TOTAL_STEPS_NEAR_TO_FAR) {
		dest_step_position = OV8820_TOTAL_STEPS_NEAR_TO_FAR;
	}

	dest_lens_position = ov8820_step_position_table[dest_step_position];

	/* Taking small damping steps */
	target_dist = step_direction * (dest_lens_position - ov8820_ctrl->curr_lens_pos);

	if (target_dist == 0) {
		return rc;
	}
	if(ov8820_use_threshold_damping && (step_direction < 0)
			&& (target_dist >= ov8820_step_position_table[ov8820_damping_threshold]))//change to variable
	{
		small_step = (uint16_t)((target_dist/ov8820_damping_fine_step));
		if (small_step == 0)
			small_step = 1;
		ov8820_damping_time_wait = 1;
	}
	else {
		small_step = (uint16_t)(target_dist/ov8820_damping_course_step);
		if (small_step == 0)
			small_step = 1;
		ov8820_damping_time_wait = 4;
	}
	for (next_lens_position = ov8820_ctrl->curr_lens_pos + (step_direction * small_step);
	(step_direction * next_lens_position) <= (step_direction * dest_lens_position);
	next_lens_position += (step_direction * small_step))
	{
		if(ov8820_af_i2c_write(next_lens_position) < 0)
			return -EBUSY;;
		ov8820_ctrl->curr_lens_pos = next_lens_position;
		if(ov8820_ctrl->curr_lens_pos != dest_lens_position){
			mdelay(10);//mdelay(ov8820_damping_time_wait);//*1000
		}
	}

	if(ov8820_ctrl->curr_lens_pos != dest_lens_position) {
		if(ov8820_af_i2c_write(dest_lens_position) < 0) {
			return -EBUSY;
		}
				mdelay(10);
	}
	/* Storing the current lens Position */
	ov8820_ctrl->curr_lens_pos = dest_lens_position;
	ov8820_ctrl->curr_step_pos = dest_step_position;
	CDBG("done\n");
	return rc;
}
static int32_t ov8820_set_default_focus(uint8_t af_step)
{

	int16_t position;
	int32_t rc = 0;
	ov8820_damping_time_wait= 4;

DBG_IN
	if(ov8820_use_default_damping) {
		/* when lens is uninitialized */
		if(ov8820_ctrl->curr_lens_pos == -1 || (ov8820_focus_debug == 1) ) {
			position = ov8820_step_position_table[ov8820_damping_threshold];
			rc =  ov8820_af_i2c_write(position);
			if (rc < 0)
				return rc;
			ov8820_ctrl->curr_step_pos = ov8820_damping_threshold;
			ov8820_ctrl->curr_lens_pos = position;
			mdelay(ov8820_damping_time_wait);;
		}
		rc = ov8820_move_focus(MOVE_FAR, ov8820_ctrl->curr_step_pos);
		if(rc < 0)
		return rc;
	}
	else {
	rc = ov8820_af_i2c_write(ov8820_step_position_table[0]);
	if ( rc < 0)
		return rc;
	ov8820_ctrl->curr_step_pos = 0;
	ov8820_ctrl->curr_lens_pos = ov8820_step_position_table[0];
	}
	return rc;
}
static void ov8820_get_pict_fps(uint16_t fps, uint16_t *pfps)
{
    uint32_t divider;	/*Q10 */
	uint32_t d1;
	uint32_t d2;
	uint16_t snapshot_height, preview_height, preview_width,snapshot_width;
DBG_IN
    if (ov8820_ctrl->prev_res == QTR_SIZE) {
		preview_width = OV8820_QTR_SIZE_WIDTH  + OV8820_HRZ_QTR_BLK_PIXELS ;
		preview_height= OV8820_QTR_SIZE_HEIGHT + OV8820_VER_QTR_BLK_LINES ;
	}
	else {
		/* full size resolution used for preview. */
		preview_width = OV8820_FULL_SIZE_WIDTH + OV8820_HRZ_FULL_BLK_PIXELS ;
		preview_height= OV8820_FULL_SIZE_HEIGHT + OV8820_VER_FULL_BLK_LINES ;
	}
	if (ov8820_ctrl->pict_res == QTR_SIZE){
		snapshot_width  = OV8820_QTR_SIZE_WIDTH + OV8820_HRZ_QTR_BLK_PIXELS ;
		snapshot_height = OV8820_QTR_SIZE_HEIGHT + OV8820_VER_QTR_BLK_LINES ;
	}
	else {
		snapshot_width  = OV8820_FULL_SIZE_WIDTH + OV8820_HRZ_FULL_BLK_PIXELS;
		snapshot_height = OV8820_FULL_SIZE_HEIGHT + OV8820_VER_FULL_BLK_LINES;
	}

	d1 =
		(uint32_t)(
		(preview_height *
		0x00000400) /
		snapshot_height);

	d2 =
		(uint32_t)(
		(preview_width *
		0x00000400) /
		 snapshot_width);


	divider = (uint32_t) (d1 * d2) / 0x00000400;
	/* Verify PCLK settings and frame sizes. */
	*pfps = (uint16_t)(fps * divider / 0x00000400);
	/* input fps is preview fps in Q8 format */


}/*endof ov8820_get_pict_fps*/

static uint16_t ov8820_get_prev_lines_pf(void)
{
DBG_IN
	if (ov8820_ctrl->prev_res == QTR_SIZE) {
		return (OV8820_QTR_SIZE_HEIGHT + OV8820_VER_QTR_BLK_LINES);
	} else {
		return (OV8820_FULL_SIZE_HEIGHT + OV8820_VER_FULL_BLK_LINES);
	}
}

static uint16_t ov8820_get_prev_pixels_pl(void)
{
DBG_IN
	if (ov8820_ctrl->prev_res == QTR_SIZE) {
		return (OV8820_QTR_SIZE_WIDTH + OV8820_HRZ_QTR_BLK_PIXELS);
	}
	else{
		return (OV8820_FULL_SIZE_WIDTH + OV8820_HRZ_FULL_BLK_PIXELS);
	}
}
static uint16_t ov8820_get_pict_lines_pf(void)
{
DBG_IN
	if (ov8820_ctrl->pict_res == QTR_SIZE) {
		return (OV8820_QTR_SIZE_HEIGHT + OV8820_VER_QTR_BLK_LINES);
	} else {
		return (OV8820_FULL_SIZE_HEIGHT + OV8820_VER_FULL_BLK_LINES);
	}
}
static uint16_t ov8820_get_pict_pixels_pl(void)
{
DBG_IN
	if (ov8820_ctrl->pict_res == QTR_SIZE) {
		return (OV8820_QTR_SIZE_WIDTH + OV8820_HRZ_QTR_BLK_PIXELS);
	} else {
		return (OV8820_FULL_SIZE_WIDTH + OV8820_HRZ_FULL_BLK_PIXELS);
	}
}

static uint32_t ov8820_get_pict_max_exp_lc(void)
{
DBG_IN
	if (ov8820_ctrl->pict_res == QTR_SIZE) {
		return (OV8820_QTR_SIZE_HEIGHT + OV8820_VER_QTR_BLK_LINES)*24;
	} else {
		return (OV8820_FULL_SIZE_HEIGHT + OV8820_VER_FULL_BLK_LINES)*24;
	}
}

static int32_t ov8820_set_fps(struct fps_cfg	*fps)
{
	int32_t rc = 0;
DBG_IN
	ov8820_ctrl->fps_divider = fps->fps_div;
	ov8820_ctrl->pict_fps_divider = fps->pict_fps_div;
	ov8820_ctrl->fps = fps->f_mult;
	return rc;
}
static int32_t ov8820_write_exp_gain(uint16_t gain, uint32_t line)
{
	uint16_t aec_msb;
	uint16_t aec_lsb;
	int32_t rc = 0;
	uint32_t total_lines_per_frame;
	uint32_t total_pixels_per_line;
	uint32_t line_length_ratio = 1 * Q8;
	uint8_t ov8820_offset = 2;
	CDBG("%s,%d: gain: %d: Linecount: %d\n",__func__,__LINE__,gain,line);

	if (ov8820_ctrl->curr_res == QTR_SIZE) {
		total_lines_per_frame = (OV8820_QTR_SIZE_HEIGHT+ OV8820_VER_QTR_BLK_LINES);
		total_pixels_per_line= OV8820_QTR_SIZE_WIDTH + OV8820_HRZ_QTR_BLK_PIXELS;

	}
	else {
		total_lines_per_frame = (OV8820_FULL_SIZE_HEIGHT + OV8820_VER_FULL_BLK_LINES);
		total_pixels_per_line= OV8820_FULL_SIZE_WIDTH + OV8820_HRZ_FULL_BLK_PIXELS;

	}

	CDBG("%s,%dline length ratio is %d\n",__func__,__LINE__,line_length_ratio);
	aec_msb = (uint16_t)(line & 0xFF00) >> 8;
	aec_lsb = (uint16_t)(line & 0x00FF);

	if (line_length_ratio != stored_line_length_ratio || (ov8820_ctrl->sensormode == SENSOR_SNAPSHOT_MODE)) {
		rc = ov8820_i2c_write_b(ov8820_client->addr, 0x3020, (((total_lines_per_frame+ov8820_offset) & 0xFF00) >> 8));
		if (rc < 0) {
			return rc;
		}
		rc = ov8820_i2c_write_b(ov8820_client->addr,0x3021, ((total_lines_per_frame+ov8820_offset) & 0x00FF));
		if (rc < 0) {
			return rc;
		}
	}
	rc = ov8820_i2c_write_b(ov8820_client->addr, OV8820_AEC_MSB, (uint8_t)aec_msb);
	if (rc < 0) {
		return rc;
	}
	rc = ov8820_i2c_write_b(ov8820_client->addr, OV8820_AEC_LSB, (uint8_t)aec_lsb);
	if(rc < 0)
		return rc;
	rc = ov8820_i2c_write_b(ov8820_client->addr, OV8820_GAIN, (uint8_t)gain);
	if (rc < 0)
		return rc;
	stored_line_length_ratio = line_length_ratio;
	return rc;
}/* endof ov8820_write_exp_gain*/
static int32_t ov8820_set_pict_exp_gain(uint16_t gain, uint32_t line)
{
	int32_t rc = 0;
	rc = ov8820_write_exp_gain(gain, line);
	return rc;
}/* endof ov8820_set_pict_exp_gain*/
static int32_t ov8820_test(enum ov8820_test_mode_t mo)
{
	int32_t rc = 0;
DBG_IN
	if (mo == TEST_OFF){
		return rc;
	}
	/* Activate  the Color bar test pattern */
	if(mo == TEST_1) {
		rc = ov8820_i2c_write_b(ov8820_client->addr,0x3303, 0x01);
		if (rc < 0) {
			return rc;
		}
		rc = ov8820_i2c_write_b(ov8820_client->addr, 0x3316, 0x02);
		if (rc < 0 ) {
			return rc;
		}
	}
	return rc;
}
static int32_t initialize_ov8820_registers(void)
{
	int32_t i, array_length;
	struct msm_camera_csi_params ov8820_csi_params;
	int32_t rc = 0;

DBG_IN

	ov8820_ctrl->sensormode = SENSOR_PREVIEW_MODE ;
	array_length = sizeof(ov8820_init_settings_array) /
	sizeof(ov8820_init_settings_array[0]);

	if (csi_config == 0) {
		ov8820_csi_params.lane_cnt = 2;
		ov8820_csi_params.data_format = CSI_10BIT;
		ov8820_csi_params.lane_assign = 0xe4;
		ov8820_csi_params.dpcm_scheme = 0;
		ov8820_csi_params.settle_cnt = 0x14;
		rc = msm_camio_csi_config(&ov8820_csi_params);
		msleep(5);
		csi_config++;
	}

	CDBG("initialize_ov8820_registers: array_length: %d\n",array_length);
	/* Configure sensor for Preview mode and Snapshot mode */
	for (i=0; i<array_length; i++) {
		rc = ov8820_i2c_write_b(ov8820_client->addr, ov8820_init_settings_array[i].reg_addr,
				ov8820_init_settings_array[i].reg_val);
		if ( rc < 0) {
			return rc;
		}
	}
	CDBG("initialize_ov8820_registers: Done\n");
	return rc;
} /* end of initialize_ov8820_ov8m0vc_registers. */
static int32_t ov8820_setting( int rt)
{
	int32_t rc = 0;
	int32_t i, array_length;
	struct msm_camera_csi_params ov8820_csi_params;

DBG_IN
	rc = ov8820_i2c_write_b(ov8820_client->addr,0x30fa, 0x00);
	if(rc < 0) {
		return rc;
	}

	if (csi_config == 0) {
		ov8820_csi_params.lane_cnt = 2;
		ov8820_csi_params.data_format = CSI_10BIT;
		ov8820_csi_params.lane_assign = 0xe4;
		ov8820_csi_params.dpcm_scheme = 0;
		ov8820_csi_params.settle_cnt = 0x14;
		rc = msm_camio_csi_config(&ov8820_csi_params);
		msleep(5);
		csi_config++;
	}

	switch(rt)
	{
		case QTR_SIZE:
			array_length = sizeof(ov8820_qtr_settings_array) /
					sizeof(ov8820_qtr_settings_array[0]);
			/* Configure sensor for preview mode */
			for (i=0; i<array_length; i++) {
				rc = ov8820_i2c_write_b(ov8820_client->addr, ov8820_qtr_settings_array[i].reg_addr,
					ov8820_qtr_settings_array[i].reg_val);
				if ( rc < 0) {
					return rc;
				}
			}
			ov8820_ctrl->curr_res = QTR_SIZE;
			break;
		case FULL_SIZE:
			if ( rc < 0)
				return rc;
			array_length = sizeof(ov8820_full_settings_array) /
				sizeof(ov8820_full_settings_array[0]);
			/* Configure sensor for capture mode */
			for (i=0; i<array_length; i++) {
				rc = ov8820_i2c_write_b(ov8820_client->addr, ov8820_full_settings_array[i].reg_addr,
						ov8820_full_settings_array[i].reg_val);
				if (rc < 0)
					return rc;
			}
			ov8820_ctrl->curr_res = FULL_SIZE;
			break;
		default:
			rc = -EFAULT;
			return rc;
	}
	rc = ov8820_i2c_write_b(ov8820_client->addr,0x30fa, 0x01);
	if (rc < 0)
		return rc;
	rc = ov8820_test(ov8820_ctrl->set_test);
	if ( rc < 0)
		return rc;
	return rc;
} /*endof  ov8820_setting*/
static int32_t ov8820_video_config(int mode )
{
	int32_t rc = 0;

DBG_IN

	if( ov8820_ctrl->curr_res != ov8820_ctrl->prev_res){
		rc = ov8820_setting(ov8820_ctrl->prev_res);
		if (rc < 0)
			return rc;
	}
	else {
		ov8820_ctrl->curr_res = ov8820_ctrl->prev_res;
	}
	ov8820_ctrl->sensormode = mode;
	return rc;
}/*end of ov354_video_config*/

static int32_t ov8820_snapshot_config(int mode)
{
	int32_t rc = 0;
DBG_IN
	if (ov8820_ctrl->curr_res != ov8820_ctrl->pict_res) {
		rc = ov8820_setting(ov8820_ctrl->pict_res);
		if (rc < 0)
			return rc;
	}
	else {
		ov8820_ctrl->curr_res = ov8820_ctrl->pict_res;
	}
	ov8820_ctrl->sensormode = mode;
	return rc;
}/*end of ov8820_snapshot_config*/

static int32_t ov8820_raw_snapshot_config(int mode)
{
	int32_t rc = 0;

DBG_IN
	ov8820_ctrl->sensormode = mode;
	if (ov8820_ctrl->curr_res != ov8820_ctrl->pict_res) {
		rc = ov8820_setting(ov8820_ctrl->pict_res);
		if (rc < 0)
			return rc;
	}
	else {
		ov8820_ctrl->curr_res = ov8820_ctrl->pict_res;
	}/* Update sensor resolution */
	ov8820_ctrl->sensormode = mode;
	return rc;
}/*end of ov8820_raw_snapshot_config*/
static int32_t ov8820_set_sensor_mode(int  mode,
			int  res)
{
	int32_t rc = 0;
DBG_IN

	switch (mode) {
	case SENSOR_PREVIEW_MODE:
		rc = ov8820_video_config(mode);
		break;
	case SENSOR_SNAPSHOT_MODE:
		rc = ov8820_snapshot_config(mode);
		break;
	case SENSOR_RAW_SNAPSHOT_MODE:
		rc = ov8820_raw_snapshot_config(mode);
		break;

	default:
		rc = -EINVAL;
		break;
	}
	return rc;
}
static int32_t ov8820_power_down(void)
{
	return 0;
}
static int ov8820_probe_init_sensor(const struct msm_camera_sensor_info *data)
{
	int32_t rc = 0;
	uint16_t  chipidl, chipidh;
	int i = 0;

DBG_IN
	rc = gpio_direction_output(data->sensor_pwd, 1);
	mdelay(5);
	rc = gpio_direction_output(data->sensor_pwd, 0);
	DBG("%s  enable power \n", __func__);
	mdelay(5);
	if (!rc){
		gpio_direction_output(data->sensor_reset, 0);
		mdelay(5);
		gpio_direction_output(data->sensor_reset, 1);
		DBG("%s  reset sensor\n", __func__);
	}
	else
		goto init_probe_done;
printk(KERN_ERR "%s %d\n", __func__, __LINE__);

//	mdelay(20);
printk(KERN_ERR "%s %d\n", __func__, __LINE__);

i2c_repeat:
	i++;
	/* 3. Read sensor Model ID: */
	if (ov8820_i2c_read(OV8820_PIDH_REG, &chipidh,1) < 0){
		printk(KERN_ERR "%s CHIPID H 0x%x\n", __func__, chipidh);
		if (i<6) goto i2c_repeat;
		goto init_probe_fail;
	}
	if (ov8820_i2c_read(OV8820_PIDL_REG, &chipidl,1) < 0){
		printk(KERN_ERR "%s CHIPID L 0x%x\n", __func__, chipidl);
		if (i<6) goto i2c_repeat;
		goto init_probe_fail;
	}
	CDBG("ov8820 model_id = 0x%x  0x%x\n", chipidh, chipidl);
	/* 4. Compare sensor ID to OV8820 ID: */
	if (chipidh != OV8820_PID || chipidl < OV8820_VER) {
		CDBG("%s enter into init_probe_fail\n",__func__);
		rc = -ENODEV;
		goto init_probe_fail;
	}
//	mdelay(OV8820_RESET_DELAY_MSECS);
	CDBG(" ov8820_probe_init_sensor ok\n");
	goto init_probe_done;
init_probe_fail:
	CDBG(" ov8820_probe_init_sensor fails\n");
	gpio_direction_output(data->sensor_reset, 0);
	gpio_direction_output(data->sensor_pwd, 1);

	//gpio_free(data->sensor_reset);
init_probe_done:
	CDBG(" ov8820_probe_init_sensor finishes\n");
	return rc;
}
int ov8820_sensor_open_init(const struct msm_camera_sensor_info *data)
{
	int i;
	int32_t  rc;
	uint16_t ov8820_nl_region_boundary = 3;
	uint16_t ov8820_nl_region_code_per_step = 101;
	uint16_t ov8820_l_region_code_per_step = 18;

DBG_IN
	CDBG("Calling ov8820_sensor_open_init\n");
	ov8820_ctrl = kzalloc(sizeof(struct ov8820_ctrl_t), GFP_KERNEL);
	if (!ov8820_ctrl) {
		CDBG("ov8820_init failed!\n");
		rc = -ENOMEM;
		goto init_done;
	}
	ov8820_ctrl->curr_lens_pos = -1;
	ov8820_ctrl->fps_divider = 1 * 0x00000400;
	ov8820_ctrl->pict_fps_divider = 1 * 0x00000400;
	ov8820_ctrl->set_test = TEST_OFF;
	ov8820_ctrl->prev_res = QTR_SIZE;
	ov8820_ctrl->pict_res = FULL_SIZE;
	ov8820_ctrl->curr_res = INVALID_SIZE;
	if (data)
		ov8820_ctrl->sensordata = data;
	/* enable mclk first */
	msm_camio_clk_rate_set(OV8820_DEFAULT_CLOCK_RATE);
	mdelay(20);
	CDBG("ov8820_sensor_open_init: Default clock is set.\n");

if (0)	ov8820_poweron_af();

	rc = ov8820_probe_init_sensor(data);
	if (rc < 0)
		goto init_fail;
	/* Initialize Sensor registers */
	rc = initialize_ov8820_registers();
	if (rc < 0) {
		return rc;
	}
	if( ov8820_ctrl->curr_res != ov8820_ctrl->prev_res) {
		rc = ov8820_setting(ov8820_ctrl->prev_res);
		if (rc < 0) {
			return rc;
		}
	}
	else {
		 ov8820_ctrl->curr_res = ov8820_ctrl->prev_res;
		}
	ov8820_ctrl->fps = 30*Q8;
	step_position_table[0] = 0;
	for(i = 1; i <= OV8820_TOTAL_STEPS_NEAR_TO_FAR; i++)
	{
		if ( i <= ov8820_nl_region_boundary) {
			ov8820_step_position_table[i] = ov8820_step_position_table[i-1] + ov8820_nl_region_code_per_step;
		}
		else {
			ov8820_step_position_table[i] = ov8820_step_position_table[i-1] + ov8820_l_region_code_per_step;
		}
	}
	 /* generate test pattern */
	if (rc < 0)
		goto init_fail;
	else
		goto init_done;
	/* reset the driver state */
	init_fail:
		CDBG(" init_fail \n");
		kfree(ov8820_ctrl);
	init_done:
		CDBG("init_done \n");
		return rc;
}/*endof ov8820_sensor_open_init*/
static int ov8820_init_client(struct i2c_client *client)
{
	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&ov8820_wait_queue);
	return 0;
}

static const struct i2c_device_id ov8820_i2c_id[] = {
	{ "ov8820", 0},
	{ }
};

static int ov8820_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;
DBG_IN
	CDBG("ov8820_probe called!\n");
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		CDBG("i2c_check_functionality failed\n");
		goto probe_failure;
	}
	ov8820_sensorw = kzalloc(sizeof(struct ov8820_work_t), GFP_KERNEL);
	if (!ov8820_sensorw) {
		CDBG("kzalloc failed.\n");
		rc = -ENOMEM;
		goto probe_failure;
	}
	i2c_set_clientdata(client, ov8820_sensorw);
	ov8820_init_client(client);
	ov8820_client = client;
	mdelay(50);
	CDBG("ov8820_probe successed! rc = %d\n", rc);
	return 0;
probe_failure:
	CDBG("ov8820_probe failed! rc = %d\n", rc);
	return rc;
}

static int __exit ov8820_remove(struct i2c_client *client)
{
	struct ov8820_work_t_t *sensorw = i2c_get_clientdata(client);
DBG_IN
	free_irq(client->irq, sensorw);
	ov8820_client = NULL;
	kfree(sensorw);
	return 0;
}

static struct i2c_driver ov8820_i2c_driver = {
	.id_table = ov8820_i2c_id,
	.probe	= ov8820_i2c_probe,
	.remove = __exit_p(ov8820_i2c_remove),
	.driver = {
		.name = "ov8820",
	},
};

int ov8820_sensor_config(void __user *argp)
{
	struct sensor_cfg_data cdata;
	long   rc = 0;
DBG_IN
	if (copy_from_user(&cdata,
				(void *)argp,
				sizeof(struct sensor_cfg_data)))
		return -EFAULT;
	down(&ov8820_sem);
	CDBG("ov8820_sensor_config: cfgtype = %d\n",
		cdata.cfgtype);
		switch (cdata.cfgtype) {
		case CFG_GET_PICT_FPS:
				ov8820_get_pict_fps(
				cdata.cfg.gfps.prevfps,
				&(cdata.cfg.gfps.pictfps));
			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;
		case CFG_GET_PREV_L_PF:
			cdata.cfg.prevl_pf =
			ov8820_get_prev_lines_pf();
			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;
		case CFG_GET_PREV_P_PL:
			cdata.cfg.prevp_pl =
				ov8820_get_prev_pixels_pl();
			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;
		case CFG_GET_PICT_L_PF:
			cdata.cfg.pictl_pf =
				ov8820_get_pict_lines_pf();
			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;
		case CFG_GET_PICT_P_PL:
			cdata.cfg.pictp_pl =
				ov8820_get_pict_pixels_pl();
			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;
		case CFG_GET_PICT_MAX_EXP_LC:
			cdata.cfg.pict_max_exp_lc =
				ov8820_get_pict_max_exp_lc();
			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;
		case CFG_SET_FPS:
		case CFG_SET_PICT_FPS:
			rc = ov8820_set_fps(&(cdata.cfg.fps));
			break;
		case CFG_SET_EXP_GAIN:
			rc =
				ov8820_write_exp_gain(
					cdata.cfg.exp_gain.gain,
					cdata.cfg.exp_gain.line);
			break;
		case CFG_SET_PICT_EXP_GAIN:
			rc =
				ov8820_set_pict_exp_gain(
					cdata.cfg.exp_gain.gain,
					cdata.cfg.exp_gain.line);
			break;
		case CFG_SET_MODE:
			rc = ov8820_set_sensor_mode(cdata.mode,
						cdata.rs);
			break;
		case CFG_PWR_DOWN:
			rc = ov8820_power_down();
			break;
		case CFG_MOVE_FOCUS:
			rc =
				ov8820_move_focus(
					cdata.cfg.focus.dir,
					cdata.cfg.focus.steps);
			break;
		case CFG_SET_DEFAULT_FOCUS:
			rc =
				ov8820_set_default_focus(
					cdata.cfg.focus.steps);
			break;
		case CFG_SET_EFFECT:
			rc = ov8820_set_default_focus(
						cdata.cfg.effect);
			break;
		default:
			rc = -EFAULT;
			break;
		}
	up(&ov8820_sem);
	return rc;
}


static int ov8820_probe_init_done(const struct msm_camera_sensor_info *data)
{
DBG_IN
	gpio_direction_output(data->sensor_reset, 0);
	//gpio_free(data->sensor_reset);
	return 0;
}

static int ov8820_sensor_release(void)
{
	int rc = -EBADF;
DBG_IN
	down(&ov8820_sem);
	ov8820_poweroff_af();
	ov8820_power_down();
	gpio_direction_output(ov8820_ctrl->sensordata->sensor_reset,
		0);
	gpio_direction_output(ov8820_ctrl->sensordata->sensor_pwd, 1);

	//gpio_free(ov8820_ctrl->sensordata->sensor_reset);
	kfree(ov8820_ctrl);
	ov8820_ctrl = NULL;
	CDBG("ov8820_release completed\n");
	up(&ov8820_sem);
	return rc;
}

static int ov8820_sensor_probe(const struct msm_camera_sensor_info *info,
		struct msm_sensor_ctrl *s)
{
	int rc = 0;
DBG_IN
	rc = i2c_add_driver(&ov8820_i2c_driver);
	if (rc < 0 || ov8820_client == NULL) {
		rc = -ENOTSUPP;
		goto probe_fail;
	}
	msm_camio_clk_rate_set(24000000);
	mdelay(20);
	rc = ov8820_probe_init_sensor(info);
	if (rc < 0)
		goto probe_fail;

	s->s_init = ov8820_sensor_open_init;
	s->s_release = ov8820_sensor_release;
	s->s_config  = ov8820_sensor_config;
	ov8820_probe_init_done(info);

	return rc;

probe_fail:
	CDBG("SENSOR PROBE FAILS!\n");
	return rc;
}

static int __ov8820_probe(struct platform_device *pdev)
{
	return msm_camera_drv_start(pdev, ov8820_sensor_probe);
}

static struct platform_driver msm_camera_driver = {
	.probe = __ov8820_probe,
	.driver = {
		.name = "msm_camera_ov8820",
		.owner = THIS_MODULE,
	},
};

static int __init ov8820_init(void)
{
DBG_IN
	return platform_driver_register(&msm_camera_driver);
}

module_init(ov8820_init);
void ov8820_exit(void)
{
	i2c_del_driver(&ov8820_i2c_driver);
}

