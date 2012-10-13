/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 and
* only version 2 as published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
* 02110-1301, USA.
*
*/



#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <media/msm_camera.h>
#include <mach/gpio.h>
#include <mach/camera.h>
#include "ov5647.h"
#include <mach/vreg.h>
#include <linux/proc_fs.h>

#define OV5647_PID_REG                       0x300A
#define OV5647_ID                    0x5647
#define DW9714_PROTECT_OFF_MSB   0xEC
#define DW9714_PROTECT_OFF_LSB    0xA3
#define DW9714_PROTECT_ON_MSB    0xDC
#define DW9714_PROTECT_ON_LSB     0x51
#define DW9714_T_SRC_MSB 0xF2
#define DW9714_T_SRC_LSB  0x00
#define DW9714_T_SRC_LSB_MASK 0xF8
#define DW9714_PD_FLAG_MASK 0x00
#define DW9714_S4_S0_MODE_MASK 0x00

#define M_TRULY		1
#define M_FOXCONN	0

struct ov5647_work {
	struct work_struct work;
};
static struct ov5647_work *ov5647_sensorw;
static struct i2c_client    *ov5647_client;

DEFINE_MUTEX(ov5647_mutex);

/*ov5647 proc file*/
struct ov5647_proc_t {
        unsigned int i2c_addr;
        unsigned int i2c_data;
};


#define OV5647_PROC_NAME "ov5647"
#define SINGLE_OP_NAME "sbdata"
#define DOUBLE_READ_NAME "double_byte_read"
#define MODULE_ID_NAME "camera_module_id"
#define I2C_ADDR_NAME "i2c_addr"
#define OV5647_PROC_LEN 16
static struct proc_dir_entry *ov5647_dir, *s_file,
	             *w_file, *dr_file, *module_id;
static char ov5647_proc_buffer[OV5647_PROC_LEN];
static struct ov5647_proc_t ov5647_proc_dt = {
	.i2c_addr = 0,
	.i2c_data = 0,
};
int ov5647_add_proc(void);
int ov5647_del_proc(void);
/*=============================================================*/

static u8 ov5647_i2c_buf[4];
static u8 ov5647_counter = 0;
static u16 m_id_reg;

struct __ov5647_ctrl 
{
	const struct msm_camera_sensor_info *sensordata;
	int sensormode;
	uint fps_divider; /* init to 1 * 0x00000400 */
	uint pict_fps_divider; /* init to 1 * 0x00000400 */
	u16 curr_step_pos;
	u16 curr_lens_pos;
	u16 init_curr_lens_pos;
	u16 my_reg_gain;
	u16 my_reg_line_count;
	enum msm_s_resolution prev_res;
	enum msm_s_resolution pict_res;
	enum msm_s_resolution curr_res;
	enum msm_s_test_mode  set_test;
};
static struct __ov5647_ctrl *ov5647_ctrl;
#ifdef FIXED_CAMIF_RECOVERY
struct platform_device *pldev;
#endif
static int ov5647_i2c_remove(struct i2c_client *client);
static int ov5647_i2c_probe(struct i2c_client *client,const struct i2c_device_id *id);

static int ov5647_i2c_txdata(u16 saddr,u8 *txdata,int length)
{
	struct i2c_msg msg[] = {
		{
			.addr  = saddr,
			.flags = 0,
			.len = length,
			.buf = txdata,
		},
	};
	if (i2c_transfer(ov5647_client->adapter, msg, 1) < 0)
		return -EIO;
	else 
		return 0;
}

static int ov5647_i2c_write(unsigned short saddr, unsigned int waddr,
	unsigned short bdata,u8 trytimes)
{
	int rc = -EIO;
	ov5647_counter = 0;
	ov5647_i2c_buf[0] = (waddr & 0xFF00)>>8;
	ov5647_i2c_buf[1] = (waddr & 0x00FF);
	ov5647_i2c_buf[2] = (bdata & 0x00FF);

	while ((ov5647_counter<trytimes) &&(rc != 0)) {
		rc = ov5647_i2c_txdata(saddr, ov5647_i2c_buf, 3);
		if (rc < 0) {
		  	ov5647_counter++;
//			printk(KERN_ERR "***Tom i2c_write_w failed,i2c addr=0x%x, command addr = 0x%x, val = 0x%x, s=%d, rc=%d, size of uint is %d!\n", 
//				saddr, waddr, bdata, ov5647_counter, rc, sizeof(unsigned int));
		  	msleep(4);
		} else {
	  		//printk(KERN_ERR "i2c_write_w succeed,i2c addr=0x%x, command addr = 0x%x, val = 0x%x, s=%d, rc=%d!\n",saddr,waddr, bdata,ov5647_counter,rc);
		}
	}
	return rc;
}

static int ov5647_af_i2c_write(unsigned short saddr, uint8_t hdata,
	uint8_t ldata,u8 trytimes)
{
	int rc = -EIO;
	ov5647_counter = 0;
	ov5647_i2c_buf[0] = hdata;
	ov5647_i2c_buf[1] = ldata;

	//printk("af w %02x %04x %02x\n", saddr, hdata, ldata);
	while ((ov5647_counter<trytimes) &&(rc != 0)) {
		rc = ov5647_i2c_txdata(saddr, ov5647_i2c_buf, 2);
		if (rc < 0) {
		  	ov5647_counter++;
		  	printk(KERN_ERR "***Tom i2c_write_w failed,i2c addr=0x%x, command addr = 0x%x, val = 0x%x, s=%d, rc=%d, size of uint is %d!\n", \
				saddr, hdata, ldata, ov5647_counter, rc, sizeof(unsigned int));
		  	msleep(4);
		} else {
	  		//printk(KERN_ERR "i2c_write_w af_driver succeed,i2c addr=0x%x, command addr = 0x%x, val = 0x%x, s=%d, rc=%d!\n",saddr, hdata, ldata,ov5647_counter,rc);
		}
	}
	return rc;
}


static int ov5647_i2c_rxdata(unsigned short saddr,
	unsigned char *rxdata, int length)
{
	struct i2c_msg msgs[] = {
	{
		.addr  = saddr,
		.flags = 0,
		.len   = 2,
		.buf   = rxdata,
	},
	{
		.addr  = saddr,
		.flags = I2C_M_RD,
		.len   = length,
		.buf   = rxdata,
	},
	};

	if (i2c_transfer(ov5647_client->adapter, msgs, 2) < 0) {
		CDBG("ov5647_i2c_rxdata failed!\n");
		return -EIO;
	}

	return 0;
}

static int32_t ov5647_i2c_read(unsigned short raddr,
				unsigned short *rdata, int rlen)
{
	int32_t rc = 0;
	unsigned char buf[2];//jerry,20101028

	//printk("r %02x %d\n", raddr, rlen);
	if (!rdata)
		return -EIO;

	memset(buf, 0, sizeof(buf));

	buf[0] = (raddr & 0xFF00) >> 8;
	buf[1] = (raddr & 0x00FF);

	//printk("client addr %02x\n", ov5647_client->addr);
	rc = ov5647_i2c_rxdata(ov5647_client->addr, buf, rlen);

	if (rc < 0) {
		CDBG("ov5647_i2c_read 0x%x failed!\n", raddr);
		return rc;
	}

	*rdata = (rlen == 2 ? buf[0] << 8 | buf[1] : buf[0]);

	return rc;

}

//new code from OV 2011.1.19 by litao
static int ov5647_write_exp_gain(uint16_t gain, uint32_t line){
        int rc = 0;
        //unsigned short debug_buf;
        static uint16_t max_line = 984;
        u8 intg_time_hsb,intg_time_msb, intg_time_lsb;
        uint8_t gain_lsb, gain_hsb;
        ov5647_ctrl->my_reg_gain = gain;
        ov5647_ctrl->my_reg_line_count = (uint16_t)line;

       /*printk("ov5647_write_exp_gain: gain = 0x%x,line = 0x%x\n", gain,line);*/

	/*Use i2c group write to avoid possible not sync problem*/
	//rc = ov5647_i2c_write(ov5647_client->addr, 0x3208, 0, 2);

        gain_lsb = (uint8_t) (ov5647_ctrl->my_reg_gain);
        gain_hsb = (uint8_t)((ov5647_ctrl->my_reg_gain& 0x300)>>8);
        //adjust frame rate
        if (line > 980) {
                if (line < 1020) {
                    rc = ov5647_i2c_write(ov5647_client->addr, \
                        0x380F, (uint8_t)((line+4) & 0x00FF) , 2);
                    rc = ov5647_i2c_write(ov5647_client->addr, \
                        0x380E, (uint8_t)((line+4) >> 8) , 2);
                    max_line = line + 4;
                }
                else{
                    rc = ov5647_i2c_write(ov5647_client->addr, \
                        0x380E, (uint8_t)((line+4) >> 8) , 2);
                    rc = ov5647_i2c_write(ov5647_client->addr, \
                        0x380F, (uint8_t)((line+4) & 0x00FF) , 2);
                    max_line = line + 4;
                }
        } else if (max_line > 984) {
                rc = ov5647_i2c_write(ov5647_client->addr, \
                        0x380F, (uint8_t)(984 & 0x00FF) , 2);
                rc = ov5647_i2c_write(ov5647_client->addr, \
                        0x380E, (uint8_t)(984 >> 8) , 2);
                max_line = 984;
        }

        line = line<<4;
        // ov5647 need this operation
        intg_time_hsb = (u8)(line>>16);
        intg_time_msb = (u8) ((line & 0xFF00) >> 8);
        intg_time_lsb = (u8) (line& 0x00FF);

        rc = ov5647_i2c_write(ov5647_client->addr, \
                REG_OV5647_LINE_HSB, intg_time_hsb, 2);
        udelay(500);
        rc = ov5647_i2c_write(ov5647_client->addr, \
                REG_OV5647_LINE_MSB, intg_time_msb, 2);
        udelay(500);
        rc = ov5647_i2c_write(ov5647_client->addr, \
                REG_OV5647_LINE_LSB, intg_time_lsb, 2);

        udelay(500);


        rc = ov5647_i2c_write(ov5647_client->addr, \
                REG_OV5647_GAIN_MSB, gain_hsb, 2);
        udelay(500);
        rc = ov5647_i2c_write(ov5647_client->addr, \
                REG_OV5647_GAIN_LSB, gain_lsb, 2);
        udelay(500);
/*	
	ov5647_i2c_read(0x380E, &debug_buf, 1);
	printk("ov5647_write_exp_gain: {0x380e,0x%2x}\n", debug_buf);
	ov5647_i2c_read(0x380F, &debug_buf, 1);
	printk("ov5647_write_exp_gain: {0x380f,0x%2x}\n", debug_buf);
	ov5647_i2c_read(0x3500, &debug_buf, 1);
	printk("ov5647_write_exp_gain: {0x3500,0x%2x}\n", debug_buf);
	ov5647_i2c_read(0x3501, &debug_buf, 1);
	printk("ov5647_write_exp_gain: {0x3501,0x%2x}\n", debug_buf);
	ov5647_i2c_read(0x3502, &debug_buf, 1);
	printk("ov5647_write_exp_gain: {0x3502,0x%2x}\n", debug_buf);
	ov5647_i2c_read(0x350A, &debug_buf, 1);
	printk("ov5647_write_exp_gain: {0x350a,0x%2x}\n", debug_buf);
	ov5647_i2c_read(0x350B, &debug_buf, 1);
	printk("ov5647_write_exp_gain: {0x350b,0x%2x}\n", debug_buf);
       printk("ov5647_write_exp_gain: <<<<<<<<<<<<<<<<\n");	
*/
	//group write end
	//rc = ov5647_i2c_write(ov5647_client->addr, 0x3208, 0x10, 2);
	//rc = ov5647_i2c_write(ov5647_client->addr, 0x3208, 0xa0, 2);

        return rc;
}

/*===========================================================================

FUNCTION      OV5647_SET_SNAPSHOT_EXPOSURE_GAIN

DESCRIPTION   UPdate exposure gain on sensor

DEPENDENCIES
  None

INPUT
  uint16 gain - value for new gain param
  uint32 line_count - num lines in whole gain table
  
RETURN VALUE
  camera_ret_code_type
  CAMERA_SUCCESS - if no error comes back from I2C
  CAMERA_FAILED  - if I2C reported an error

SIDE EFFECTS
  None

===========================================================================*/
static int32_t ov5647_set_snapshot_exposure_gain(uint16_t gain, uint32_t line)
{
	static uint16_t max_line;
       //unsigned short debug_buf;
	int rc = 0; 
	uint8_t gain_lsb, gain_hsb;
	u8 intg_time_hsb,intg_time_msb, intg_time_lsb;
	ov5647_ctrl->my_reg_gain = gain;
	ov5647_ctrl->my_reg_line_count = (uint16_t)line;

	gain_lsb = (uint8_t) (ov5647_ctrl->my_reg_gain);
	gain_hsb = (uint8_t)((ov5647_ctrl->my_reg_gain& 0x300)>>8);

       /*printk("ov5647_set_snapshot_exposure_gain: gain = 0x%x,line = 0x%x\n", gain,line);*/

	if (line > 1964) {
		rc = ov5647_i2c_write(ov5647_client->addr, \
			0x380E, (uint8_t)((line+4) >> 8) , 2);
		rc = ov5647_i2c_write(ov5647_client->addr, \
			0x380F, (uint8_t)((line+4) & 0x00FF) , 2);
		max_line = line + 4;
	} else if (max_line > 1968) {
		rc = ov5647_i2c_write(ov5647_client->addr, \
			0x380E, (uint8_t)(1968 >> 8) , 2);
		rc = ov5647_i2c_write(ov5647_client->addr, \
			0x380F, (uint8_t)(1968 & 0x00FF) , 2);
		max_line = 1968;
	}
	line = line<<4; 
	// ov5647 need this operation
	intg_time_hsb = (u8)(line>>16);
	intg_time_msb = (u8) ((line & 0xFF00) >> 8);
	intg_time_lsb = (u8) (line& 0x00FF);


	//FIXME for BLC trigger 
	/*
	rc = ov5647_i2c_write(ov5647_client->addr, \
		REG_OV5647_LINE_HSB, intg_time_hsb, 2);
	udelay(500);
	rc = ov5647_i2c_write(ov5647_client->addr, \
		REG_OV5647_LINE_MSB, intg_time_msb, 2);
	udelay(500);
	rc = ov5647_i2c_write(ov5647_client->addr, \
		REG_OV5647_LINE_LSB, intg_time_lsb, 2);
	udelay(500);


	rc = ov5647_i2c_write(ov5647_client->addr, \
		REG_OV5647_GAIN_MSB, gain_hsb, 2);
	udelay(500);
	rc = ov5647_i2c_write(ov5647_client->addr, \
		REG_OV5647_GAIN_LSB, gain_lsb - 1, 2);
	udelay(500);

	mdelay(200);
	*/
	rc = ov5647_i2c_write(ov5647_client->addr, \
		REG_OV5647_LINE_HSB, intg_time_hsb, 2);
	udelay(500);
	rc = ov5647_i2c_write(ov5647_client->addr, \
		REG_OV5647_LINE_MSB, intg_time_msb, 2);
	udelay(500);
	rc = ov5647_i2c_write(ov5647_client->addr, \
		REG_OV5647_LINE_LSB, intg_time_lsb, 2);
	udelay(500);


	rc = ov5647_i2c_write(ov5647_client->addr, \
		REG_OV5647_GAIN_MSB, gain_hsb, 2);
	udelay(500);
	rc = ov5647_i2c_write(ov5647_client->addr, \
		REG_OV5647_GAIN_LSB, gain_lsb, 2);
	udelay(500);
/*
	ov5647_i2c_read(0x380E, &debug_buf, 1);
	printk("ov5647_set_snapshot_exposure_gain: {0x380E,0x%2x}\n", debug_buf);
	ov5647_i2c_read(0x380F, &debug_buf, 1);
	printk("ov5647_set_snapshot_exposure_gain: {0x380F,0x%2x}\n", debug_buf);
	ov5647_i2c_read(0x3500, &debug_buf, 1);
	printk("ov5647_set_snapshot_exposure_gain: {0x3500,0x%2x}\n", debug_buf);
	ov5647_i2c_read(0x3501, &debug_buf, 1);
	printk("ov5647_set_snapshot_exposure_gain: {0x3501,0x%2x}\n", debug_buf);
	ov5647_i2c_read(0x3502, &debug_buf, 1);
	printk("ov5647_set_snapshot_exposure_gain: {0x3502,0x%2x}\n", debug_buf);
	ov5647_i2c_read(0x350A, &debug_buf, 1);
	printk("ov5647_set_snapshot_exposure_gain: {0x350A,0x%2x}\n", debug_buf);
	ov5647_i2c_read(0x350B, &debug_buf, 1);
	printk("ov5647_set_snapshot_exposure_gain: {0x350B,0x%2x}\n", debug_buf);
       printk("ov5647_set_snapshot_exposure_gain: <<<<<<<<<<<<<<<<\n");		
*/

	return rc;
}

/*===========================================================================

FUNCTION      OV5647_SET_DEFAULT_FOCUS

DESCRIPTION
  Move focus to location best suited to any subject: at the nearest 
  point where infinity is still in focus.
  specified.

DEPENDENCIES
  None

RETURN VALUE
  camera_ret_code_type
  CAMERA_SUCCESS - if no error comes back from I2C
  CAMERA_FAILED  - if I2C reported an error

SIDE EFFECTS
  None

===========================================================================*/

static int32_t ov5647_set_default_focus(void)
{
	uint8_t  code_val_msb = 0;
	uint8_t  code_val_lsb = 0;
	int 	rc = 0;
	
	ov5647_ctrl->curr_lens_pos = 0;
	ov5647_ctrl->init_curr_lens_pos = 0;
	ov5647_ctrl->curr_step_pos = 0;


	code_val_msb = (uint8_t)((ov5647_ctrl->curr_lens_pos & 0x03FF) >> 4);
	//code_val_msb |= DW9714_PD_FLAG_MASK;
	code_val_lsb = (uint8_t)((ov5647_ctrl->curr_lens_pos & 0x000F) << 4);
	//code_val_lsb |= DW9714_S4_S0_MODE_MASK;
	
/*
	rc = ov5647_af_i2c_write(OV5647_AF_I2C_ADDR, DW9714_PROTECT_OFF_MSB, DW9714_PROTECT_OFF_LSB, 2);
	if ( rc != 0) {
		printk(KERN_ERR "ov5647_set_default_focus:WRITE DW9714_PROTECT_OFF\n");
	}

	rc = ov5647_af_i2c_write(OV5647_AF_I2C_ADDR, DW9714_T_SRC_MSB, DW9714_T_SRC_LSB, 2);
	if ( rc != 0) {
		printk(KERN_ERR "ov5647_set_default_focus:WRITE DW9714_T_SRC ERROR \n");
	}

	rc = ov5647_af_i2c_write(OV5647_AF_I2C_ADDR, DW9714_PROTECT_ON_MSB, DW9714_PROTECT_ON_LSB, 2);
	if ( rc != 0) {
		printk(KERN_ERR "ov5647_set_default_focus:WRITE DW9714_PROTECT_ON ERROR \n");
	}
*/	
	rc = ov5647_af_i2c_write(OV5647_AF_I2C_ADDR, code_val_msb, code_val_lsb, 2);
	if ( rc != 0) {
		printk(KERN_ERR "ov5647_set_default_focus:WRITE ERROR lsb = 0x%x, msb = 0x%x", code_val_lsb, code_val_msb );
	}
	
	printk(KERN_INFO "ov5647_set_default_focus:lens pos = %d\n", ov5647_ctrl->curr_lens_pos);
	
	mdelay(10);
	return rc;
}



/*===========================================================================

===========================================================================*/
static int32_t ov5647_move_focus(int direction, int32_t num_steps)
{
	uint8_t   code_val_msb = 0;
	uint8_t   code_val_lsb = 0;
	int16_t   step_direction,actual_step,next_position;
	int rc;

	if (num_steps == 0) {
		return 0;
	}
  
	if ( direction == MOVE_NEAR ) {
		step_direction = 10;	/*20 -> 10, modify by jinlin.Hu for fix resolution problem*/
	} else if ( direction == MOVE_FAR) {
		step_direction = -10;	/*20 -> 10, modify by jinlin.Hu for fix resolution problem*/
	} else {
		return -EINVAL;
	}
  
	actual_step = (int16_t)(step_direction * num_steps);              
	next_position = (int16_t)ov5647_ctrl->curr_lens_pos + actual_step; 
	if(next_position < 0) {
		printk(KERN_WARNING "ov5647_move_focus:OV5647 position(=%d) out of range",next_position);
		next_position = 0;
	}
	if(next_position > 0x3FF) {
		printk(KERN_WARNING "ov5647_move_focus:OV5647 position(=%d) out of range",next_position);
		next_position = 0x3FF;
	}
	ov5647_ctrl->curr_lens_pos = next_position; 


	code_val_msb = (uint8_t)((ov5647_ctrl->curr_lens_pos & 0x03FF) >> 4);
	//code_val_msb |= DW9714_PD_FLAG_MASK;
	code_val_lsb = (uint8_t)((ov5647_ctrl->curr_lens_pos & 0x000F) << 4);
	code_val_lsb |= DW9714_S4_S0_MODE_MASK;

/*
	rc = ov5647_af_i2c_write(OV5647_AF_I2C_ADDR, DW9714_PROTECT_OFF_MSB, DW9714_PROTECT_OFF_LSB, 2);
	if ( rc != 0) {
		printk(KERN_ERR "ov5647_move_focus:WRITE DW9714_PROTECT_OFF\n");
	}

	rc = ov5647_af_i2c_write(OV5647_AF_I2C_ADDR, DW9714_T_SRC_MSB, DW9714_T_SRC_LSB, 2);
	if ( rc != 0) {
		printk(KERN_ERR "ov5647_move_focus:WRITE DW9714_T_SRC ERROR \n");
	}

	rc = ov5647_af_i2c_write(OV5647_AF_I2C_ADDR, DW9714_PROTECT_ON_MSB, DW9714_PROTECT_ON_LSB, 2);
	if ( rc != 0) {
		printk(KERN_ERR "ov5647_move_focus:WRITE DW9714_PROTECT_ON ERROR \n");
	}
*/
	rc = ov5647_af_i2c_write(OV5647_AF_I2C_ADDR, code_val_msb, code_val_lsb, 1);
	if ( rc != 0) {
		printk(KERN_ERR "ov5647_move_focus:WRITE ERROR lsb = 0x%x, msb = 0x%x",code_val_lsb, code_val_msb );
	} else {
		/*printk(KERN_INFO "ov5647_move_focus:Successful lsb = 0x%x, msb = 0x%x",code_val_lsb, code_val_msb );*/
		//change 100 to 30 by litao&lixiaochao,according to camera module provider
		//--change to 50ms, March 3rd, litao, according module producer 
		mdelay(10); //delay may set based on the steps moved when I2C write successful.
	}
	return 0;
}


static int ov5647_probe_init_sensor(const struct msm_camera_sensor_info *data)
{
	int32_t probe_retry = 1;//3;
	uint16_t  chipid = 0;
	int rc = 0;
	printk(KERN_ERR "--CAMERA-- %s (Start...)\n",__func__);

       gpio_tlmm_config(GPIO_CFG(data->sensor_reset, 0, GPIO_CFG_OUTPUT,
                                                      GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
       gpio_tlmm_config(GPIO_CFG(data->sensor_pwd, 0, GPIO_CFG_OUTPUT,
                                                      GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);

	msleep(20);
	//shut down sub camera
	gpio_set_value(33, 1);

	//open ov5647 pwd
	printk("ov5647 sensor pwd %d\n", data->sensor_pwd);
	gpio_set_value(data->sensor_pwd,  0);

	//(2) config pwd and rest pin
	gpio_set_value(data->sensor_reset, 0);
	mdelay(5);

	gpio_set_value(data->sensor_reset, 1);
	printk(KERN_ERR "--CAMERA-- %s : gpio_request=%d, result is %d\n",__func__,data->sensor_reset, rc);
	mdelay(20);

	/*added by litao for ID read*/
detect_retry:
	probe_retry--;
	printk("retry left: %d\nabout to read ID of 5647\n", probe_retry);
	/* 3. Read sensor Model ID: */
	ov5647_i2c_read(OV5647_PID_REG, &chipid, 2);
	printk("ov5647 model_id = 0x%x\n", chipid);
	/* 4. Compare sensor ID to OV5647 ID: */
	if (chipid != OV5647_ID) {
		if(probe_retry)
			goto detect_retry;
		rc = -ENODEV;
		goto init_probe_fail;
	}
	/*added by litao for ID read --end*/

	printk(KERN_ERR "--CAMERA-- %s ok , device id=0x%x\n",__func__,chipid);

	return 0;
init_probe_fail:
	return rc;
}
static void ov5647_stop_stream(void)
{  
    printk(KERN_ERR "%s  enter\n", __func__);
    if (ov5647_i2c_write(ov5647_client->addr, 0x0100, 0x00, 1)){
        printk(KERN_ERR "%s failed\n", __func__);
    }
}
static void ov5647_start_stream(void)
{
    printk(KERN_ERR "%s  enter\n", __func__);
    if (ov5647_i2c_write(ov5647_client->addr, 0x0100, 0x01, 1)){
        printk(KERN_ERR "%s failed\n", __func__);
    }
}

static int ov5647_setting(enum msm_s_reg_update rupdate,enum msm_s_setting rt)
{
	int rc = -EINVAL;
	int len;
	int i = 0;
       static int OV5647_CSI_CONFIG = 0;

	printk(KERN_ERR "--CAMERA-- %s (Start...), rupdate=%d \n",__func__,rupdate);
	switch (rupdate)	{
		case S_UPDATE_PERIODIC:
			{
				ov5647_stop_stream();
				msleep(100);
                                
				if (OV5647_CSI_CONFIG == 0){          
					struct msm_camera_csi_params ov5647_csi_params;                                       
#ifdef FIXED_CAMIF_RECOVERY
 					msm_camio_csi_reset(pldev);
                                        msleep(10);
#endif
					ov5647_csi_params.lane_cnt = 2;
					ov5647_csi_params.data_format = CSI_10BIT;
					ov5647_csi_params.lane_assign = 0xe4;
					ov5647_csi_params.dpcm_scheme = 0;
					ov5647_csi_params.settle_cnt = 0x18;//7;//0x14;
					printk(KERN_INFO "%s call msm_camio_csi_config\n", __func__);
					rc = msm_camio_csi_config(&ov5647_csi_params);
					OV5647_CSI_CONFIG =1;          
					msleep(20);   
				}
				
				printk(KERN_ERR "--CAMERA-- S_UPDATE_PERIODIC (Start)\n");
				if (rt == S_RES_CAPTURE) {
					printk(KERN_ERR "--CAMERA-- OV5647_RES_CAPTURE (Start)\n");
					len = sizeof (ov5647_capture_array) /sizeof (ov5647_capture_array[0]);
					for(i = 0; i < len; i++){
						rc = ov5647_i2c_write(ov5647_client->addr, \
							ov5647_capture_array[i].register_address, ov5647_capture_array[i].register_value,10);
					}

				} else if(rt == S_RES_PREVIEW ) {  
					len = sizeof (ov5647_preview_array) /sizeof (ov5647_preview_array[0]);
					for(i = 0; i < len; i++){
						rc = ov5647_i2c_write(ov5647_client->addr, \
							ov5647_preview_array[i].register_address, ov5647_preview_array[i].register_value,10);
					}
				}
				
				msleep(20);
				ov5647_start_stream();
			}
			break; /* UPDATE_PERIODIC */
			
		case S_REG_INIT:
			{
				OV5647_CSI_CONFIG = 0;

				printk(KERN_ERR "--CAMERA-- S_REG_INIT\n");
				len = sizeof (ov5647_init_array) /sizeof (ov5647_init_array[0]);

				if (m_id_reg == 0x12) {
					printk("this is truly module\n");
					for(i = 0;i<len;i++){
						rc = ov5647_i2c_write(ov5647_client->addr, \
						ov5647_init_array_truly[i].register_address, ov5647_init_array_truly[i].register_value, 2);
                                        }
				} else {
					printk("this is truly module\n");
					for(i = 0;i<len;i++)
					{
						rc = ov5647_i2c_write(ov5647_client->addr, \
						ov5647_init_array[i].register_address, ov5647_init_array[i].register_value, 2);
					}
				}
				ov5647_ctrl->fps_divider = 1 * 0x0400;
				msleep(20);
			}
			break; /* case REG_INIT: */
			
		default:
			break;
	}
	return rc;
}

static int32_t ov5647_poweron_af(void)
{
	int32_t rc = 0;

	gpio_tlmm_config(GPIO_CFG(ov5647_ctrl->sensordata->vcm_enable, 0, GPIO_CFG_OUTPUT,
	                           GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);	   
	gpio_set_value(ov5647_ctrl->sensordata->vcm_enable, 1);
	mdelay(12);
		
	rc = ov5647_set_default_focus();
	return rc;
}

static void ov5647_poweroff_af(void)
{
	gpio_set_value(ov5647_ctrl->sensordata->vcm_enable, 0);
}

static int ov5647_sensor_open_init(const struct msm_camera_sensor_info *data)
{
	int rc = -ENOMEM;
	printk(KERN_ERR "--CAMERA-- %s\n",__func__);

	ov5647_ctrl = kzalloc(sizeof(struct __ov5647_ctrl), GFP_KERNEL);
	if (!ov5647_ctrl)
	{
		printk(KERN_ERR "--CAMERA-- kzalloc ov5647_ctrl error !!\n");
		kfree(ov5647_ctrl);
		return rc;
	}
	ov5647_ctrl->fps_divider = 1 * 0x00000400;
	ov5647_ctrl->pict_fps_divider = 1 * 0x00000400;
	ov5647_ctrl->set_test = S_TEST_OFF;
	ov5647_ctrl->prev_res = S_QTR_SIZE;
	ov5647_ctrl->pict_res = S_FULL_SIZE;
	
	if (data) ov5647_ctrl->sensordata = data;

	/* enable mclk = 24 MHz first */
	msm_camio_clk_rate_set(24000000);
	mdelay(20);

	rc = ov5647_probe_init_sensor(data);
	if(rc < 0){
		printk(KERN_ERR "--camera-- init error\n");
		kfree(ov5647_ctrl);
		ov5647_ctrl = NULL;
		return rc;
	}
	
	if (ov5647_ctrl->prev_res == S_QTR_SIZE)
		rc = ov5647_setting(S_REG_INIT, S_RES_PREVIEW);
	else
		rc = ov5647_setting(S_REG_INIT, S_RES_CAPTURE);

	if (rc < 0)
	{
		printk(KERN_ERR "--CAMERA-- %s : ov5647_setting failed. rc = %d\n",__func__,rc);
		kfree(ov5647_ctrl);
		ov5647_ctrl = NULL;
		return rc;
	}

	ov5647_poweron_af();

	return rc;
}

static int ov5647_sensor_release(void)
{
	printk(KERN_ERR "--CAMERA--ov5647_sensor_release!!\n");

	mutex_lock(&ov5647_mutex);
	
	/*hardware reset (active low with internal pull-up resistor)*/
	gpio_set_value(ov5647_ctrl->sensordata->sensor_reset, 0);
	
	/*power down control(active high with internal pull-down resistor)*/
       gpio_set_value(ov5647_ctrl->sensordata->sensor_pwd, 1);
	   
       ov5647_poweroff_af();

	kfree(ov5647_ctrl);	
	ov5647_ctrl = NULL;
	mutex_unlock(&ov5647_mutex);
	//ov5647_del_proc();
	return 0;
}


			       
static const struct i2c_device_id ov5647_i2c_id[] = {
	{"ov5647", 0},{}
};

static int ov5647_i2c_remove(struct i2c_client *client)
{
	struct ov5647_work *data = i2c_get_clientdata(client);

	kfree(data);

   	return 0;
}

static int32_t ov5647_video_config(int mode)
{
	int32_t rc = 0;

	rc = ov5647_setting(S_UPDATE_PERIODIC, S_RES_PREVIEW);
	if (rc < 0)
		return rc;
	CDBG("ov5647 sensor configuration done!\n");

	ov5647_ctrl->curr_res = ov5647_ctrl->prev_res;
	ov5647_ctrl->sensormode = mode;

	rc = ov5647_write_exp_gain(ov5647_ctrl->my_reg_gain,
				    ov5647_ctrl->my_reg_line_count);

	mdelay(15);
	return rc;
}

static int32_t ov5647_snapshot_config(int mode)
{
	int32_t rc = 0;

	rc = ov5647_setting(S_UPDATE_PERIODIC, S_RES_CAPTURE);
	if (rc < 0)
		return rc;
	ov5647_ctrl->curr_res = ov5647_ctrl->pict_res;
	ov5647_ctrl->sensormode = mode;

	return rc;
}


static int ov5647_raw_snapshot_config(int mode)
{
	int32_t rc = 0;

	rc = ov5647_setting(S_UPDATE_PERIODIC, S_RES_CAPTURE);
	if (rc < 0)
		return rc;
	ov5647_ctrl->curr_res = ov5647_ctrl->pict_res;
	ov5647_ctrl->sensormode = mode;
	printk(KERN_ERR "--CAMERA-- %s (End...)\n",__func__);
	return rc;
}

static void ov5647_get_pict_fps(uint16_t fps, uint16_t *pfps){
	/* input fps is preview fps in Q8 format */
	u32 d1 = (u32)(
	((ov5647_reg_pat[0].frame_length_lines) *	0x400) / 
	(ov5647_reg_pat[1].frame_length_lines ));
	u32 d2 = (u32)(
		((ov5647_reg_pat[0].line_length_pck) *	0x400) / 
		(ov5647_reg_pat[1].line_length_pck));
	u32 divider = (uint32_t) (d1 * d2) / 0x00000400;
	/* Verify PCLK settings and frame sizes. */
	*pfps = (u16)(fps * divider / 0x00000400);
}

static uint16_t ov5647_get_prev_lines_pf(void)
{
	return  ov5647_reg_pat[0].frame_length_lines;
}

static uint16_t ov5647_get_prev_pixels_pl(void)
{
	return  ov5647_reg_pat[0].line_length_pck;
}

static uint16_t ov5647_get_pict_lines_pf(void)
{
	return  ov5647_reg_pat[1].frame_length_lines;
}

static uint16_t ov5647_get_pict_pixels_pl(void)
{
	return  ov5647_reg_pat[1].line_length_pck;
}

static uint32_t ov5647_get_pict_max_exp_lc(void)
{
	uint16_t snapshot_lines_per_frame;

	snapshot_lines_per_frame =
	    ov5647_reg_pat[1].frame_length_lines - 1;
	return snapshot_lines_per_frame * 24;
}


static int ov5647_set_sensor_mode(int mode, int res)
{
	int rc = -EINVAL;

	printk(KERN_ERR "--CAMERA-- ov5647_set_sensor_mode mode = %d, res = %d\n", mode, res);

	switch (mode)
	{
		case SENSOR_PREVIEW_MODE:
			printk(KERN_ERR "--CAMERA-- SENSOR_PREVIEW_MODE\n");
			rc = ov5647_video_config(mode);
			break;
		case SENSOR_SNAPSHOT_MODE:
			printk(KERN_ERR "--CAMERA-- SENSOR_SNAPSHOT_MODE\n");
			rc = ov5647_snapshot_config(mode);
			break;
		case SENSOR_RAW_SNAPSHOT_MODE:
			printk(KERN_ERR "--CAMERA-- SENSOR_RAW_SNAPSHOT_MODE\n");
			rc = ov5647_raw_snapshot_config(mode);
			break;
		default:
			break;
	}
	return rc;
}


static int32_t ov5647_lens_shading_enable(uint8_t is_enable)
{
	int32_t rc = 0;

	CDBG("%s: entered. enable = %d\n", __func__, is_enable);

	rc = ov5647_i2c_write(ov5647_client->addr, \
			0x5000,0xff,10);
	udelay(500);
	
	if (rc < 0)
		return rc;

	CDBG("%s: exiting. rc = %d\n", __func__, rc);
	return rc;
}

static int ov5647_sensor_config(void __user *argp)
{
	struct sensor_cfg_data cdata;
	long  rc = 0;
	//printk(KERN_ERR "--CAMERA-- %s (Start...)\n",__func__);
	if (copy_from_user(&cdata,(void *)argp,sizeof(struct sensor_cfg_data))) 
		return -EFAULT;
		
	mutex_lock(&ov5647_mutex);
	//down(&ov5647_mutex);
	switch (cdata.cfgtype)
	{
		case CFG_SET_MODE:   // 0
			rc =ov5647_set_sensor_mode(cdata.mode, cdata.rs);
			break;
		case CFG_SET_EFFECT: // 1
			CDBG("--CAMERA-- CFG_SET_EFFECT (Not Support) !!\n");
			// Not Support
			break;
		case CFG_START:      // 2
			CDBG("--CAMERA-- CFG_START (Not Support) !!\n");
			// Not Support
			break;
		case CFG_PWR_UP:     // 3
			CDBG("--CAMERA-- CFG_PWR_UP (Not Support) !!\n");
			// Not Support
			break;
		case CFG_PWR_DOWN:   // 4
			CDBG("--CAMERA-- CFG_PWR_DOWN (Not Support) \n");
			break;
		case CFG_SET_DEFAULT_FOCUS:  // 06
			CDBG("--CAMERA-- CFG_SET_DEFAULT_FOCUS !!\n");
			rc = ov5647_set_default_focus();
			break;		
		case CFG_MOVE_FOCUS:     //  07
			CDBG("--CAMERA-- CFG_MOVE_FOCUS: cdata.cfg.focus.dir=%d \
				cdata.cfg.focus.steps=%d\n",
				cdata.cfg.focus.dir, cdata.cfg.focus.steps);
			rc = ov5647_move_focus(cdata.cfg.focus.dir,
					cdata.cfg.focus.steps);
			break;
		case CFG_GET_AF_MAX_STEPS: //26
			CDBG("--CAMERA-- CFG_GET_AF_MAX_STEPS (!!\n");
			cdata.max_steps = 80; /*30 -> 80, modify by jinlin.Hu, for fix resolution problem*/
			if (copy_to_user((void *)argp,
				 &cdata, sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
			break;
		case CFG_SET_LENS_SHADING: //20
			CDBG("%s: CFG_SET_LENS_SHADING\n", __func__);
			rc = ov5647_lens_shading_enable(cdata.cfg.lens_shading);
			break;
		case CFG_GET_PICT_FPS: //21
			ov5647_get_pict_fps(cdata.cfg.gfps.prevfps,
				     &(cdata.cfg.gfps.pictfps));

			if (copy_to_user((void *)argp, &cdata,
				 sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;
		case CFG_SET_EXP_GAIN: //18
			//printk(KERN_ERR "--CAMERA-- CFG_SET_EXP_GAIN gain = %d, line = %d\n", cdata.cfg.exp_gain.gain, cdata.cfg.exp_gain.line);
			ov5647_write_exp_gain(cdata.cfg.exp_gain.gain,
					    cdata.cfg.exp_gain.line);
			break;
		case CFG_GET_PICT_MAX_EXP_LC: //27
			cdata.cfg.pict_max_exp_lc = ov5647_get_pict_max_exp_lc();

			if (copy_to_user((void *)argp, &cdata,
				 sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
			break;
		case CFG_GET_PREV_L_PF: //22
			cdata.cfg.prevl_pf = ov5647_get_prev_lines_pf();

			if (copy_to_user((void *)argp,
				 &cdata, sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

		case CFG_GET_PREV_P_PL: //23
			cdata.cfg.prevp_pl = ov5647_get_prev_pixels_pl();

			if (copy_to_user((void *)argp,
				 &cdata, sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;
		case CFG_GET_PICT_L_PF: //24
			cdata.cfg.pictl_pf = ov5647_get_pict_lines_pf();

			if (copy_to_user((void *)argp,
				 &cdata, sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

		case CFG_GET_PICT_P_PL: //25
			cdata.cfg.pictp_pl = ov5647_get_pict_pixels_pl();

			if (copy_to_user((void *)argp,
				 &cdata, sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;
		case CFG_SET_PICT_EXP_GAIN: //19			printk(KERN_ERR "--CAMERA-- CFG_SET_PICT_EXP_GAIN !!\n");
			rc = ov5647_set_snapshot_exposure_gain(cdata.cfg.exp_gain.gain,
				       cdata.cfg.exp_gain.line);
		break;

		case CFG_SET_FPS:
		case CFG_SET_PICT_FPS:
			printk(KERN_ERR "--CAMERA-- %s: Command=%d (Not Implement)!!\n",__func__,cdata.cfgtype);
			rc = 0;	//Implement in exp setting fuction			
			break;
		
		default:
			printk(KERN_ERR "--CAMERA-- %s: Command=%d (Not Implement)!!\n",__func__,cdata.cfgtype);
			if (cdata.cfgtype >= CFG_MAX)
				rc = -EINVAL;
			else 
				rc = 0;
		break;	
	}
	mutex_unlock(&ov5647_mutex);
	//up(&ov5647_mutex);
	//printk(KERN_ERR "--CAMERA-- %s (End...), result is %d\n",__func__, rc);
	return rc;	
}

static struct i2c_driver ov5647_i2c_driver = {
	.id_table = ov5647_i2c_id,
	.probe  = ov5647_i2c_probe,
	.remove = ov5647_i2c_remove,
	.driver = {
		.name = "ov5647",
	},
};


static int ov5647_sensor_probe(const struct msm_camera_sensor_info *info,struct msm_sensor_ctrl *s)
{
	int rc = -ENOTSUPP;

	printk(KERN_ERR "--CAMERA-- %s (Start...)\n",__func__);
	rc = i2c_add_driver(&ov5647_i2c_driver);
	if ((rc < 0 ) || (ov5647_client == NULL)) {
		printk(KERN_ERR "--CAMERA-- i2c_add_driver FAILS!!\n");
		return rc;
	}

	msm_camio_clk_rate_set(OV5647_DEFAULT_CLOCK_RATE);
	mdelay(20);//change 200 to 20ms by litao&lixiaochao

	rc = ov5647_probe_init_sensor(info);
	if (rc < 0) {
		printk(KERN_ERR "--CAMERA--ov5647_probe_init_sensor Fail !!~~~~!!\n");
		return rc;
	}
	s->s_init = ov5647_sensor_open_init;
	s->s_release = ov5647_sensor_release;
	s->s_config  = ov5647_sensor_config;
        s->s_camera_type = BACK_CAMERA_2D;
        s->s_mount_angle = 0;

	/*add by jinlin.Hu, for read the camera module id. Start.*/
	ov5647_i2c_write(ov5647_client->addr, 0x0100, 0x01, 2);
	ov5647_i2c_write(ov5647_client->addr, 0x3D21, 0x01, 2);
	udelay(500);

	rc = ov5647_i2c_read(0x3D05, &m_id_reg, 1);
	printk("module id is %x\n", m_id_reg);
	/*add by jinlin.Hu. End*/

	ov5647_add_proc();

	printk(KERN_ERR "--CAMERA-- %s (End...)\n",__func__);
	return rc;
}

static int ov5647_i2c_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
	printk(KERN_ERR "--CAMERA-- %s ... (Start...)\n",__func__);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "--CAMERA--i2c_check_functionality failed\n");
		return -ENOMEM;
	}

	ov5647_sensorw = kzalloc(sizeof(struct ov5647_work), GFP_KERNEL);
	if (!ov5647_sensorw) {
		printk(KERN_ERR "--CAMERA--kzalloc failed\n");
		return -ENOMEM;
	}
	i2c_set_clientdata(client, ov5647_sensorw);
	ov5647_client = client;
	printk("--CAMERA-- %s ... (End...)\n",__func__);
	return 0;
}

static int __ov5647_probe(struct platform_device *pdev)
{
#ifdef FIXED_CAMIF_RECOVERY
        pldev = pdev;
#endif
	return msm_camera_drv_start(pdev, ov5647_sensor_probe);
}

//=============================//

static int proc_ov5647_dread(char *page, char **start,
                             off_t off, int count,
                             int *eof, void *data)
{
	int len, rc;
	unsigned short tmp16;
	/************************/
	/************************/
	rc = ov5647_i2c_read(ov5647_proc_dt.i2c_addr, &tmp16, 2);
	if(rc < 0){
		len = sprintf(page, "double 0x%x@ov5647_i2c read error\n",
		                      ov5647_proc_dt.i2c_addr);
	}
	else{
		len = sprintf(page, "double 0x%x@ov5647_i2c = %4x\n",
		                      ov5647_proc_dt.i2c_addr, tmp16);
	}

	return len;
}


static int proc_ov5647_sread(char *page, char **start,
                            off_t off, int count,
                            int *eof, void *data)
{
        int len, rc;
	unsigned short tmp16;

	rc = ov5647_i2c_read(ov5647_proc_dt.i2c_addr, &tmp16, 2);
	if(rc < 0){
		len = sprintf(page, "single byte 0x%x@ov5647_i2c read error\n",
		                      ov5647_proc_dt.i2c_addr);
	}
	else{
		len = sprintf(page, "single byte 0x%x@ov5647_i2c = %4x\n",
		                      ov5647_proc_dt.i2c_addr, tmp16>>8);
	}

        return len;
}

static int proc_ov5647_swrite(struct file *file,
                            const char *buffer,
                            unsigned long count,
                            void *data)
{
        int len;

	memset(ov5647_proc_buffer, 0, OV5647_PROC_LEN);
        if(count > OV5647_PROC_LEN)
                len = OV5647_PROC_LEN;
        else
                len = count;

        if(copy_from_user(ov5647_proc_buffer, buffer, len))
                return -EFAULT;
	sscanf(ov5647_proc_buffer, "%x", &ov5647_proc_dt.i2c_data);
	ov5647_i2c_write(ov5647_client->addr, ov5647_proc_dt.i2c_addr, ov5647_proc_dt.i2c_data, 2);
        return len;

}

static int proc_ov5647_addr_read(char *page, char **start,
                            off_t off, int count,
                            int *eof, void *data)
{
        int len;

	/*read i2c will do a camif error test*/
	/*write a wrong value to 3809*/
	ov5647_i2c_write(ov5647_client->addr, 0x3809, 0x20, 2);
	/*delay for about 1 frame*/
	mdelay(40);
	len = sprintf(page, "addr is 0x%x\n", ov5647_proc_dt.i2c_addr);
	/*recover right value to 3809*/
	ov5647_i2c_write(ov5647_client->addr, 0x3809, 0x10, 2);

        return len;
}

static int proc_ov5647_addr_write(struct file *file,
                            const char *buffer,
                            unsigned long count,
                            void *data)
{
        int len;

	memset(ov5647_proc_buffer, 0, OV5647_PROC_LEN);
        if(count > OV5647_PROC_LEN)
                len = OV5647_PROC_LEN;
        else
                len = count;

        if(copy_from_user(ov5647_proc_buffer, buffer, len))
                return -EFAULT;
	sscanf(ov5647_proc_buffer, "%x", &ov5647_proc_dt.i2c_addr);
        return len;

}

/*add by jinlin.Hu. for user space get module info to distinguish truly and foxconn module*/
static int proc_ov5647_module_id_read(char *page, char **start,
					off_t off, int count,
					int *eof, void *data)
{
	int len;
	int module_t;

	if (m_id_reg == 0x12)
		module_t = M_TRULY;
	else
		module_t = M_FOXCONN;

	len = sprintf(page, "%d", module_t);
	
	return len;
}

int ov5647_add_proc(void)
{
	int rc;
	/* add for proc*/
	/* create directory */
        ov5647_dir = proc_mkdir(OV5647_PROC_NAME, NULL);
	if(ov5647_dir == NULL) {
	          rc = -ENOMEM;
	          goto init_fail;
	}

	//ov5647_dir->owner = THIS_MODULE;

        /* create readfile */
	s_file = create_proc_entry(SINGLE_OP_NAME,
	                                 0644, ov5647_dir);
	if(s_file == NULL) {
	          rc  = -ENOMEM;
	          goto no_s;
	}
        s_file->read_proc = proc_ov5647_sread;
        s_file->write_proc = proc_ov5647_swrite;
	//s_file->owner = THIS_MODULE;

	dr_file = create_proc_read_entry(DOUBLE_READ_NAME,
	                                 0444, ov5647_dir,
	                                 proc_ov5647_dread,
	                                 NULL);
	if(dr_file == NULL) {
	          rc  = -ENOMEM;
	          goto no_dr;
	}

	//dr_file->owner = THIS_MODULE;

        /* create write file */
        w_file = create_proc_entry(I2C_ADDR_NAME, 0644, ov5647_dir);
        if(w_file == NULL) {
                rc = -ENOMEM;
                goto no_wr;
        }

        w_file->read_proc = proc_ov5647_addr_read;
        w_file->write_proc = proc_ov5647_addr_write;
        //w_file->owner = THIS_MODULE;

	module_id = create_proc_entry(MODULE_ID_NAME, 0444, ov5647_dir);
	if (module_id == NULL) {
		rc = -ENOMEM;
		goto no_module_id;
	}

	module_id->read_proc = proc_ov5647_module_id_read;

        /* OK, out debug message */
        printk(KERN_INFO "%s %s %s initialised\n",
		              SINGLE_OP_NAME, DOUBLE_READ_NAME, I2C_ADDR_NAME);

	/*litao add end*/
	return 0;
	/*litao add for proc*/

no_module_id:
	remove_proc_entry(I2C_ADDR_NAME, ov5647_dir);
no_wr:
        remove_proc_entry(DOUBLE_READ_NAME, ov5647_dir);
no_dr:
        remove_proc_entry(SINGLE_OP_NAME, ov5647_dir);
no_s:
        remove_proc_entry(OV5647_PROC_NAME, NULL);
	/*litao add end*/
init_fail:
	return 1;
}

int ov5647_del_proc(void)
{
	remove_proc_entry(MODULE_ID_NAME, ov5647_dir);
        remove_proc_entry(I2C_ADDR_NAME, ov5647_dir);
        remove_proc_entry(DOUBLE_READ_NAME, ov5647_dir);
        remove_proc_entry(SINGLE_OP_NAME, ov5647_dir);
        remove_proc_entry(OV5647_PROC_NAME, NULL);
        printk(KERN_INFO "%s %s %s removed\n",
	              SINGLE_OP_NAME, DOUBLE_READ_NAME, I2C_ADDR_NAME);
	return 0;
}

//=============================================//

static int __ov5647_remove(struct platform_device *pdev)
{
	i2c_del_driver(&ov5647_i2c_driver);
	ov5647_del_proc();

	return 0;
}

static struct platform_driver msm_camera_driver = {
	.probe = __ov5647_probe,
	.remove = __ov5647_remove,
	.driver = {
		.name = "msm_camera_ov5647",
		.owner = THIS_MODULE,
	},
};

static int __init ov5647_init(void)
{
	ov5647_i2c_buf[0]=0x5A;
	return platform_driver_register(&msm_camera_driver);
}

static void __exit ov5647_exit(void)
{
	return platform_driver_unregister(&msm_camera_driver);
}

module_init(ov5647_init);
module_exit(ov5647_exit);
MODULE_DESCRIPTION("OMNI 5M Bayer sensor driver");
MODULE_LICENSE("GPL v2");

