/*****************************************************************************/
/* Copyright (c) 2009 NXP Semiconductors BV                                  */
/*                                                                           */
/* This program is free software; you can redistribute it and/or modify      */
/* it under the terms of the GNU General Public License as published by      */
/* the Free Software Foundation, using version 2 of the License.             */
/*                                                                           */
/* This program is distributed in the hope that it will be useful,           */
/* but WITHOUT ANY WARRANTY; without even the implied warranty of            */
/* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the              */
/* GNU General Public License for more details.                              */
/*                                                                           */
/* You should have received a copy of the GNU General Public License         */
/* along with this program; if not, write to the Free Software               */
/* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307       */
/* USA.                                                                      */
/*                                                                           */
/*****************************************************************************/

#define _tx_c_

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/cdev.h>  
#include <linux/fs.h>
#include <linux/ioctl.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <asm/uaccess.h>
#include <mach/gpio.h>
#include <linux/wakelock.h>
// Peter XIAO Fix HDMI freeze start
#include <linux/clk.h>
// Peter XIAO Fix HDMI freeze end
/* HDMI DevLib */
#include "tmNxCompId.h"
#include "tmdlHdmiTx_Types.h"
#include "tmdlHdmiTx_Functions.h"

/* local */
#include "tda998x_version.h"
#include "tda998x.h"
#include "tda998x_ioctl.h"

#ifdef I2C_DBG
#include "tmbslHdmiTx_types.h"
#include "tmbslTDA9989_local.h"
#endif

#ifdef ANDROID_DSS
/* DSS hack */
#endif

#ifdef ANDROID_DSS_MSM
#include "../msm/msm_fb.h"
#include "../msm/external_common.h"
#endif

/*
 *
 * DEFINITION
 * ----------
 * LEVEL 0
 *
 */

#define FRAME_PACKING 200
#define NO_FP(x) ((x) % FRAME_PACKING)
#define IS_FP(x) ((x) > FRAME_PACKING)
#define WITH_FP(x) ((x) + FRAME_PACKING * (this->tda.setio.video_in.structure3D == TMDL_HDMITX_3D_FRAME_PACKING))
/*
 *  Global
 */

tda_instance our_instance;
tda_instance *this;
static struct cdev our_cdev, *this_cdev=&our_cdev;
void show_video(tda_instance *this) ;
char *hdmi_tx_err_string(int err);
static irqreturn_t tda_irq(int irq, void *_udc);

/*
 *  Module params
 */

static int param_verbose=1,param_major=0,param_minor=0;
module_param_named(verbose,param_verbose,int,S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(verbose, "Make the driver verbose");

module_param_named(major, param_major, int, S_IRUGO);
MODULE_PARM_DESC(major, "The major number of the device mapper");


#ifdef ANDROID_DSS_MSM
static struct external_common_state_type hdmi_common;
int hdmi_enable(void);
int hdmi_disable(int event_tracking);
struct msm_hdmi_platform_data *pp=NULL;
void reset_hdmi(int hdcp_module);
struct wake_lock wlock;

static int core_power_on = 0;	
static int enable_5v_on = 0;
static int comm_power_on= 0;

#ifdef CONFIG_SUSPEND
static struct delayed_work resume_work;
static void tda998x_resume(struct work_struct *dummy);
#endif

typedef enum {
    TDA998X_POWER_COMMON,
    TDA998X_POWER_CORE,
    TDA998X_POWER_5V,
    TDA998X_POWER_ALL,      
}tda_power_enum;

// Peter XIAO Fix HDMI freeze start
static struct clk *tv_enc_clk;
// Peter XIAO Fix HDMI freeze end

static int hdmi_irq_init(void)
{	
	static int init=0;	
	int err=0;
	if(!init){
		init=1;
		err=request_irq(this->driver.i2c_client->irq, tda_irq, 
			IRQF_TRIGGER_FALLING|IRQF_DISABLED, "TDA IRQ", this);
			
		LOG(KERN_INFO, "irq number is : %d , status of request_irq: %d\n", 
			this->driver.i2c_client->irq, err);

		if (err <0) {
			printk(KERN_ERR "hdmitx:%s:Cannot request irq, err:%d\n",__func__,err);
			gpio_free(this->driver.gpio);
		}
	}
	return err;
}

static void tda998x_power_enable(tda_power_enum power,int on)
{ 
    int err=0;
    int show = param_verbose;
    if(pp == NULL) return;

    LOG(KERN_INFO,": power =%d,on=%d\n",power,on);

    if(power == TDA998X_POWER_ALL)   {
        if((pp->core_power == NULL) ||(pp->enable_5v== NULL) || ( pp->comm_power ==NULL)) return;
        
        if(core_power_on != on){
           err= pp->core_power(on,show);
           if(err==0) core_power_on = on;
        }
        
        if(enable_5v_on != on){
           err= pp->enable_5v(on);
           if(err==0) enable_5v_on = on;
        }
        
        if(comm_power_on != on){
           err= pp->comm_power(on,show);
           if(err==0) comm_power_on = on;
        }
        
    }    
    else if (power == TDA998X_POWER_CORE && pp->core_power != NULL) {
        if(core_power_on != on){
           err= pp->core_power(on,show);
           if(err==0) core_power_on = on;
        }
    }
    else if (power == TDA998X_POWER_COMMON && pp->comm_power != NULL) {
        if(comm_power_on != on){
           err= pp->comm_power(on,show);
           if(err==0) comm_power_on = on;
        }
    }
    else if (power == TDA998X_POWER_5V && pp->enable_5v != NULL) {
        if(enable_5v_on != on){
           err= pp->enable_5v(on);
           if(err==0) enable_5v_on = on;
        }
    }
}

int hdmi_enable(void)
{
   int err=0;
   
   LOG(KERN_INFO,"called\n");      

   if(this->driver.enable)
   	return 0;   
   	
   tda998x_power_enable(TDA998X_POWER_CORE,1); 
   	
   down(&this->driver.sem);   
   
   this->tda.power = tmPowerOn;
   TRY(tmdlHdmiTxSetPowerState(this->tda.instance,this->tda.power));
   if (err==TM_ERR_NO_RESOURCES) {
      LOG(KERN_INFO,"Busy...\n");
      TRY(tmdlHdmiTxHandleInterrupt(this->tda.instance));
      TRY(tmdlHdmiTxHandleInterrupt(this->tda.instance));
      TRY(tmdlHdmiTxHandleInterrupt(this->tda.instance));
   }
   tmdlHdmiTxGetHPDStatus(this->tda.instance,&this->tda.hot_plug_detect);
   //show_video(this);
 TRY_DONE:
   if(err == 0)
   	this->driver.enable = true;
   up(&this->driver.sem);   
   return err;
}
EXPORT_SYMBOL(hdmi_enable);

/*
 *  
 */
int hdmi_disable(int event_tracking)
{ 
   int err=0;
   
   LOG(KERN_INFO,"called\n");

   tda998x_power_enable(TDA998X_POWER_CORE,1); 

   down(&this->driver.sem);
   this->tda.power = (event_tracking?tmPowerSuspend: tmPowerStandby);
   TRY(tmdlHdmiTxSetPowerState(this->tda.instance,this->tda.power));

 TRY_DONE:
    if(err == 0)
   	this->driver.enable = false;
   up(&this->driver.sem);
   return err;
}
EXPORT_SYMBOL(hdmi_disable);

static int tda998x_on(struct platform_device *pdev)
{
	int err=0;
	LOG(KERN_INFO,"called\n");    	

	// Peter XIAO Fix HDMI freeze start
	clk_enable(tv_enc_clk);
	// Peter XIAO Fix HDMI freeze end

	if(external_common_state->hpd_feature_on) {    		
		tda998x_power_enable(TDA998X_POWER_ALL,1);         	
		err = hdmi_enable();
		if(external_common_state->hpd_state ) {
			 show_video(this);
			 switch_set_state(&external_common_state->sdev, 1);
		}
		wake_lock(&wlock);
	}	
	 
	return err;
}

static int tda998x_off(struct platform_device *pdev)
{ 
	int err=0;
	LOG(KERN_INFO,"called\n");
	wake_unlock(&wlock);

	// add this code to fix sumsung LE37M86BD TV issue
	if((this->tda.hot_plug_detect == TMDL_HDMITX_HOTPLUG_ACTIVE)  && (this->tda.rx_device_active == 0))
		return 0;

	err = hdmi_disable(0);   

	if(external_common_state->hpd_feature_on){    
		tda998x_power_enable(TDA998X_POWER_ALL,1); 
		err = hdmi_disable(1);
	}else {
		tda998x_power_enable(TDA998X_POWER_ALL,0);         
	}    
	// Peter XIAO Fix HDMI freeze start
	clk_disable(tv_enc_clk);  
	// Peter XIAO Fix HDMI freeze end
	switch_set_state(&external_common_state->sdev, 0);
	return err;
}

static struct msm_fb_panel_data tda998x_panel_data = {
	        .on  = tda998x_on,
	        .off = tda998x_off,
};

static struct platform_device tda998x_panel_device = {
	        .name = "tda998x",
	        .id   = 2, 
	        .dev  = {  
				.platform_data = &tda998x_panel_data,
			}    
};

static void change_hdmi_state(int online)
{
        if (!external_common_state)
                return;
        
        mutex_lock(&external_common_state_hpd_mutex);
        external_common_state->hpd_state = online;
        mutex_unlock(&external_common_state_hpd_mutex);

        if (!external_common_state->uevent_kobj)
                return;

        if (online){
                kobject_uevent(external_common_state->uevent_kobj,
                        KOBJ_ONLINE);
				switch_set_state(&external_common_state->sdev, 1);
        	}
        else{
                kobject_uevent(external_common_state->uevent_kobj,
                        KOBJ_OFFLINE);
				switch_set_state(&external_common_state->sdev, 0);
        	}
        DEV_INFO("tda998x_uevent: %d \n", online);
}

static int tda998x_hpd_feature(int on)
{
    int err=0;
        
    printk("%s enter on : %d\n", __func__, on);

    if (on) {        
        tda998x_power_enable(TDA998X_POWER_ALL,1);   
        hdmi_enable();    
	reset_hdmi(0); 	
	if(this->tda.hot_plug_detect != TMDL_HDMITX_HOTPLUG_ACTIVE) {
		err = hdmi_disable(1);
	}	
	hdmi_irq_init();
    } else {
        err =  hdmi_disable(0);	             
        tda998x_power_enable(TDA998X_POWER_ALL,0); 
        wake_unlock(&wlock);
    }	
    return err;
}

static int tda9988_read_edid_block(int block, uint8 *edid_buf)
{ 
    if(block < EDID_BLOCK_COUNT)
        memcpy(edid_buf, &(this->tda.raw_edid[block*EDID_BLOCK_SIZE]), EDID_BLOCK_SIZE);

    return 0;
}    
    
static uint32 get_mode_order(uint32 mode)
{
    switch (mode) {
            case HDMI_VFRMT_1280x720p60_16_9:
                return 11;
            case HDMI_VFRMT_1280x720p50_16_9:
                return 10;
            case HDMI_VFRMT_720x576p50_16_9:
                return 9;
            case HDMI_VFRMT_720x576p50_4_3:
                return 8;
            case HDMI_VFRMT_720x480p60_16_9:
                return 7;
            case HDMI_VFRMT_720x480p60_4_3:
                return 6;
            case HDMI_VFRMT_640x480p60_4_3:
            	return 5;
            case HDMI_VFRMT_1440x576i50_16_9:
                return 4;
            case HDMI_VFRMT_1440x576i50_4_3:
                return 3;		
            case HDMI_VFRMT_1440x480i60_16_9:
                return 2;
            case HDMI_VFRMT_1440x480i60_4_3:
                return 1;
            default:
                return 0;
        }
}


static uint32 tda9988_get_bestmode(void)
{
    uint32 bestmode=HDMI_VFRMT_640x480p60_4_3, bestorder=0, order=0,i=0 ;
    
    for(i=0;  i<external_common_state->disp_mode_list.num_of_elements; i++)
    {
        order = get_mode_order(external_common_state->disp_mode_list.disp_mode_list[i]);
        if(order > bestorder)
        {
            bestorder = order;
            bestmode = external_common_state->disp_mode_list.disp_mode_list[i];
        }
    }

    printk("%s  : bestmode = %d\n", __func__, bestmode);
    
    return bestmode+1;
}

static void tda9988_read_edid(void)
{
	external_common_state->read_edid_block = tda9988_read_edid_block;
        hdmi_common_read_edid();
        printk("%s  : disp_mode_list.num_of_elements = %d\n", __func__, 
            external_common_state->disp_mode_list.num_of_elements);
}

#endif

/*
 *
 * TOOLBOX
 * -------
 * LEVEL 1
 *
 * - i2c read/write
 * - chip Id check
 * - i2c client info
 * 
 */

/* 
 *  Get main and unique I2C Client driver handle
 */
struct i2c_client *GetThisI2cClient(void)
{
   return this->driver.i2c_client;
}

EXPORT_SYMBOL(GetThisI2cClient);

/*
 * error handling
 */
char *hdmi_tx_err_string(int err)
{
   switch (err & 0x0FFF)
      {
      case TM_ERR_COMPATIBILITY: {return "SW Interface compatibility";break;}
      case TM_ERR_MAJOR_VERSION: {return "SW Major Version error";break;}
      case TM_ERR_COMP_VERSION: {return "SW component version error";break;}
      case TM_ERR_BAD_UNIT_NUMBER: {return "Invalid device unit number";break;}
      case TM_ERR_BAD_INSTANCE: {return "Bad input instance value  ";break;}
      case TM_ERR_BAD_HANDLE: {return "Bad input handle";break;}
      case TM_ERR_BAD_PARAMETER: {return "Invalid input parameter";break;}
      case TM_ERR_NO_RESOURCES: {return "Resource is not available ";break;}
      case TM_ERR_RESOURCE_OWNED: {return "Resource is already in use";break;}
      case TM_ERR_RESOURCE_NOT_OWNED: {return "Caller does not own resource";break;}
      case TM_ERR_INCONSISTENT_PARAMS: {return "Inconsistent input params";break;}
      case TM_ERR_NOT_INITIALIZED: {return "Component is not initialised";break;}
      case TM_ERR_NOT_SUPPORTED: {return "Function is not supported";break;}
      case TM_ERR_INIT_FAILED: {return "Initialization failed";break;}
      case TM_ERR_BUSY: {return "Component is busy";break;}
      case TMDL_ERR_DLHDMITX_I2C_READ: {return "Read error";break;}
      case TMDL_ERR_DLHDMITX_I2C_WRITE: {return "Write error";break;}
      case TM_ERR_FULL: {return "Queue is full";break;}
      case TM_ERR_NOT_STARTED: {return "Function is not started";break;}
      case TM_ERR_ALREADY_STARTED: {return "Function is already starte";break;}
      case TM_ERR_ASSERTION: {return "Assertion failure";break;}
      case TM_ERR_INVALID_STATE: {return "Invalid state for function";break;}
      case TM_ERR_OPERATION_NOT_PERMITTED: {return "Corresponds to posix EPERM";break;}
      case TMDL_ERR_DLHDMITX_RESOLUTION_UNKNOWN: {return "Bad format";break;}
      case TM_OK: {return "OK";break;}
      default : {printk(KERN_INFO "(err:%x) ",err);return "unknown";break;}
      }
}

static char *tda_spy_event(int event)
{
   switch (event)
      {
      case TMDL_HDMITX_HDCP_ACTIVE: {return "HDCP active";break;}
      case TMDL_HDMITX_HDCP_INACTIVE: {return "HDCP inactive";break;}
      case TMDL_HDMITX_HPD_ACTIVE: {return "HPD active";break;}
      case TMDL_HDMITX_HPD_INACTIVE: {return "HPD inactive";break;}
      case TMDL_HDMITX_RX_KEYS_RECEIVED: {return "Rx keys received";break;}
      case TMDL_HDMITX_RX_DEVICE_ACTIVE: {return "Rx device active";break;}
      case TMDL_HDMITX_RX_DEVICE_INACTIVE: {return "Rx device inactive";break;}
      case TMDL_HDMITX_EDID_RECEIVED: {return "EDID received";break;}
      case TMDL_HDMITX_VS_RPT_RECEIVED: {return "VS interrupt has been received";break;}
         /*       case TMDL_HDMITX_B_STATUS: {return "TX received BStatus";break;} */ 
#if defined (TMFL_TDA19989) || defined (TMFL_TDA9984) 
      case TMDL_HDMITX_DEBUG_EVENT_1: {return "DEBUG_EVENT_1";break;}
#endif
      default : {return "Unkonwn event";break;}
      }
}
#if defined (TMFL_TDA19989) || defined (TMFL_TDA9984) 
static char *tda_spy_hsdc_fail_status(int fail)
{
   switch (fail)
      {
      case TMDL_HDMITX_HDCP_OK: {return "ok";break;}
      case  TMDL_HDMITX_HDCP_BKSV_RCV_FAIL: {return "Source does not receive Sink BKsv ";break;}
      case TMDL_HDMITX_HDCP_BKSV_CHECK_FAIL: {return "BKsv does not contain 20 zeros and 20 ones";break;}
      case TMDL_HDMITX_HDCP_BCAPS_RCV_FAIL: {return "Source does not receive Sink Bcaps";break;}
      case TMDL_HDMITX_HDCP_AKSV_SEND_FAIL: {return "Source does not send AKsv";break;}
      case TMDL_HDMITX_HDCP_R0_RCV_FAIL: {return "Source does not receive R'0";break;}
      case TMDL_HDMITX_HDCP_R0_CHECK_FAIL: {return "R0 = R'0 check fail";break;}
      case TMDL_HDMITX_HDCP_BKSV_NOT_SECURE: {return "bksv not secure";break;}
      case TMDL_HDMITX_HDCP_RI_RCV_FAIL: {return "Source does not receive R'i";break;}
      case TMDL_HDMITX_HDCP_RPT_RI_RCV_FAIL: {return "Source does not receive R'i repeater mode";break;}
      case TMDL_HDMITX_HDCP_RI_CHECK_FAIL: {return "RI = R'I check fail";break;}
      case TMDL_HDMITX_HDCP_RPT_RI_CHECK_FAIL: {return "RI = R'I check fail repeater mode";break;}
      case TMDL_HDMITX_HDCP_RPT_BCAPS_RCV_FAIL: {return "Source does not receive Sink Bcaps repeater mode";break;}
      case TMDL_HDMITX_HDCP_RPT_BCAPS_READY_TIMEOUT: {return "bcaps ready timeout";break;}
      case TMDL_HDMITX_HDCP_RPT_V_RCV_FAIL: {return "Source does not receive V";break;}
      case TMDL_HDMITX_HDCP_RPT_BSTATUS_RCV_FAIL: {return "Source does not receive BSTATUS repeater mode";break;}
      case TMDL_HDMITX_HDCP_RPT_KSVLIST_RCV_FAIL: {return "Source does not receive Ksv list in repeater mode";break;}
      case TMDL_HDMITX_HDCP_RPT_KSVLIST_NOT_SECURE: {return "ksvlist not secure";break;}
      default: {return "";break;}
      }
}

static char *tda_spy_hdcp_status(int status)
{
   switch (status)
      {
      case TMDL_HDMITX_HDCP_CHECK_NOT_STARTED: {return "Check not started";break;}
      case TMDL_HDMITX_HDCP_CHECK_IN_PROGRESS: {return "No failures, more to do";break;}
      case TMDL_HDMITX_HDCP_CHECK_PASS: {return "Final check has passed";break;}
      case TMDL_HDMITX_HDCP_CHECK_FAIL_FIRST: {return "First check failure code\nDriver not AUTHENTICATED";break;}
      case TMDL_HDMITX_HDCP_CHECK_FAIL_DEVICE_T0: {return "A T0 interrupt occurred";break;}
      case TMDL_HDMITX_HDCP_CHECK_FAIL_DEVICE_RI: {return "Device RI changed";break;}
      case TMDL_HDMITX_HDCP_CHECK_FAIL_DEVICE_FSM: {return "Device FSM not 10h";break;}
      default : {return "Unknown hdcp status";break;}
      }

}
#endif

static char *tda_spy_sink(int sink)
{
   switch (sink)
      {
      case TMDL_HDMITX_SINK_DVI: {return "DVI";break;}
      case TMDL_HDMITX_SINK_HDMI: {return "HDMI";break;}
      case TMDL_HDMITX_SINK_EDID: {return "As currently defined in EDID";break;}
      default : {return "Unkonwn sink";break;}
      }
}

#if defined (TMFL_TDA19989) || defined (TMFL_TDA9984) 
static char *tda_spy_aspect_ratio(int ar)
{
   switch (ar)
      {
      case TMDL_HDMITX_P_ASPECT_RATIO_UNDEFINED: {return "Undefined picture aspect rati";break;}
      case TMDL_HDMITX_P_ASPECT_RATIO_6_5: {return "6:5 picture aspect ratio (PAR";break;}
      case TMDL_HDMITX_P_ASPECT_RATIO_5_4: {return "5:4 PA";break;}
      case TMDL_HDMITX_P_ASPECT_RATIO_4_3: {return "4:3 PA";break;}
      case TMDL_HDMITX_P_ASPECT_RATIO_16_10: {return "16:10 PA";break;}
      case TMDL_HDMITX_P_ASPECT_RATIO_5_3: {return "5:3 PA";break;}
      case TMDL_HDMITX_P_ASPECT_RATIO_16_9: {return "16:9 PA";break;}
      case TMDL_HDMITX_P_ASPECT_RATIO_9_5: {return "9:5 PA";break;}
      default : {return "Unknown aspect ratio";break;}
      }
}

#if 0 /* no more used */
static char *tda_spy_edid_status(int status)
{
   switch (status)
      {
      case TMDL_HDMITX_EDID_READ: {return "All blocks read";break;}
      case TMDL_HDMITX_EDID_READ_INCOMPLETE: {return "All blocks read OK but buffer too small to return all of the";break;}
      case TMDL_HDMITX_EDID_ERROR_CHK_BLOCK_0: {return "Block 0 checksum erro";break;}
      case TMDL_HDMITX_EDID_ERROR_CHK: {return "Block 0 OK, checksum error in one or more other block";break;}
      case TMDL_HDMITX_EDID_NOT_READ: {return "EDID not read";break;}
      case TMDL_HDMITX_EDID_STATUS_INVALID: {return "Invalid ";break;}
      default : {return "Unknown edid status";break;}
      }
}
#endif

static char *tda_spy_vfmt(int fmt)
{
   switch (fmt)
      {
      case TMDL_HDMITX_VFMT_NULL: {return "NOT a valid format...";break;}
      case TMDL_HDMITX_VFMT_01_640x480p_60Hz: {return "vic 01: 640x480p 60Hz";break;}
      case TMDL_HDMITX_VFMT_02_720x480p_60Hz: {return "vic 02: 720x480p 60Hz";break;}
      case TMDL_HDMITX_VFMT_03_720x480p_60Hz: {return "vic 03: 720x480p 60Hz";break;}
      case TMDL_HDMITX_VFMT_04_1280x720p_60Hz: {return "vic 04: 1280x720p 60Hz";break;}
      case TMDL_HDMITX_VFMT_05_1920x1080i_60Hz: {return "vic 05: 1920x1080i 60Hz";break;}
      case TMDL_HDMITX_VFMT_06_720x480i_60Hz: {return "vic 06: 720x480i 60Hz";break;}
      case TMDL_HDMITX_VFMT_07_720x480i_60Hz: {return "vic 07: 720x480i 60Hz";break;}
      case TMDL_HDMITX_VFMT_08_720x240p_60Hz: {return "vic 08: 720x240p 60Hz";break;}
      case TMDL_HDMITX_VFMT_09_720x240p_60Hz: {return "vic 09: 720x240p 60Hz";break;}
      case TMDL_HDMITX_VFMT_10_720x480i_60Hz: {return "vic 10: 720x480i 60Hz";break;}
      case TMDL_HDMITX_VFMT_11_720x480i_60Hz: {return "vic 11: 720x480i 60Hz";break;}
      case TMDL_HDMITX_VFMT_12_720x240p_60Hz: {return "vic 12: 720x240p 60Hz";break;}
      case TMDL_HDMITX_VFMT_13_720x240p_60Hz: {return "vic 13: 720x240p 60Hz";break;}
      case TMDL_HDMITX_VFMT_14_1440x480p_60Hz: {return "vic 14: 1440x480p 60Hz";break;}
      case TMDL_HDMITX_VFMT_15_1440x480p_60Hz: {return "vic 15: 1440x480p 60Hz";break;}
      case TMDL_HDMITX_VFMT_16_1920x1080p_60Hz: {return "vic 16: 1920x1080p 60Hz";break;}
      case TMDL_HDMITX_VFMT_17_720x576p_50Hz: {return "vic 17: 720x576p 50Hz";break;}
      case TMDL_HDMITX_VFMT_18_720x576p_50Hz: {return "vic 18: 720x576p 50Hz";break;}
      case TMDL_HDMITX_VFMT_19_1280x720p_50Hz: {return "vic 19: 1280x720p 50Hz";break;}
      case TMDL_HDMITX_VFMT_20_1920x1080i_50Hz: {return "vic 20: 1920x1080i 50Hz";break;}
      case TMDL_HDMITX_VFMT_21_720x576i_50Hz: {return "vic 21: 720x576i 50Hz";break;}
      case TMDL_HDMITX_VFMT_22_720x576i_50Hz: {return "vic 22: 720x576i 50Hz";break;}
      case TMDL_HDMITX_VFMT_23_720x288p_50Hz: {return "vic 23: 720x288p 50Hz";break;}
      case TMDL_HDMITX_VFMT_24_720x288p_50Hz: {return "vic 24: 720x288p 50Hz";break;}
      case TMDL_HDMITX_VFMT_25_720x576i_50Hz: {return "vic 25: 720x576i 50Hz";break;}
      case TMDL_HDMITX_VFMT_26_720x576i_50Hz: {return "vic 26: 720x576i 50Hz";break;}
      case TMDL_HDMITX_VFMT_27_720x288p_50Hz: {return "vic 27: 720x288p 50Hz";break;}
      case TMDL_HDMITX_VFMT_28_720x288p_50Hz: {return "vic 28: 720x288p 50Hz";break;}
      case TMDL_HDMITX_VFMT_29_1440x576p_50Hz: {return "vic 29: 1440x576p 50Hz";break;}
      case TMDL_HDMITX_VFMT_30_1440x576p_50Hz: {return "vic 30: 1440x576p 50Hz";break;}
      case TMDL_HDMITX_VFMT_31_1920x1080p_50Hz: {return "vic 31: 1920x1080p 50Hz";break;}
      case TMDL_HDMITX_VFMT_32_1920x1080p_24Hz: {return "vic 32: 1920x1080p 24Hz";break;}
      case TMDL_HDMITX_VFMT_33_1920x1080p_25Hz: {return "vic 33: 1920x1080p 25Hz";break;}
      case TMDL_HDMITX_VFMT_34_1920x1080p_30Hz: {return "vic 34: 1920x1080p 30Hz";break;}
      case TMDL_HDMITX_VFMT_35_2880x480p_60Hz: {return "vic 3: 2880x480p 60Hz";break;}
      case TMDL_HDMITX_VFMT_36_2880x480p_60Hz: {return "vic 3: 2880x480p 60Hz";break;}
      case TMDL_HDMITX_VFMT_37_2880x576p_50Hz: {return "vic 3: 2880x576p 50Hz";break;}
      case TMDL_HDMITX_VFMT_38_2880x576p_50Hz: {return "vic 3: 2880x576p 50Hz";break;}
      case TMDL_HDMITX_VFMT_60_1280x720p_24Hz: {return "vic 60: 1280x720p 24Hz";break;}
      case TMDL_HDMITX_VFMT_61_1280x720p_25Hz: {return "vic 61: 1280x720p 25Hz";break;}
      case TMDL_HDMITX_VFMT_62_1280x720p_30Hz: {return "vic 62: 1280x720p 30Hz";break;}
      case TMDL_HDMITX_VFMT_PC_800x600p_60Hz: {return "PC 129";break;}
      case TMDL_HDMITX_VFMT_PC_1152x960p_60Hz: {return "PC 130";break;}
      case TMDL_HDMITX_VFMT_PC_1024x768p_60Hz: {return "PC 131";break;}
      case TMDL_HDMITX_VFMT_PC_1280x768p_60Hz: {return "PC 132";break;}
      case TMDL_HDMITX_VFMT_PC_1280x1024p_60Hz: {return "PC 133";break;}
      case TMDL_HDMITX_VFMT_PC_1360x768p_60Hz: {return "PC 134";break;}
      case TMDL_HDMITX_VFMT_PC_1400x1050p_60Hz: {return "PC 135";break;}
      case TMDL_HDMITX_VFMT_PC_1600x1200p_60Hz: {return "PC 136";break;}
      case TMDL_HDMITX_VFMT_PC_1024x768p_70Hz: {return "PC 137";break;}
      case TMDL_HDMITX_VFMT_PC_640x480p_72Hz: {return "PC 138";break;}
      case TMDL_HDMITX_VFMT_PC_800x600p_72Hz: {return "PC 139";break;}
      case TMDL_HDMITX_VFMT_PC_640x480p_75Hz: {return "PC 140";break;}
      case TMDL_HDMITX_VFMT_PC_1024x768p_75Hz: {return "PC 141";break;}
      case TMDL_HDMITX_VFMT_PC_800x600p_75Hz: {return "PC 142";break;}
      case TMDL_HDMITX_VFMT_PC_1024x864p_75Hz: {return "PC 143";break;}
      case TMDL_HDMITX_VFMT_PC_1280x1024p_75Hz: {return "PC 144";break;}
      case TMDL_HDMITX_VFMT_PC_640x350p_85Hz: {return "PC 145";break;}
      case TMDL_HDMITX_VFMT_PC_640x400p_85Hz: {return "PC 146";break;}
      case TMDL_HDMITX_VFMT_PC_720x400p_85Hz: {return "PC 147";break;}
      case TMDL_HDMITX_VFMT_PC_640x480p_85Hz: {return "PC 148";break;}
      case TMDL_HDMITX_VFMT_PC_800x600p_85Hz: {return "PC 149";break;}
      case TMDL_HDMITX_VFMT_PC_1024x768p_85Hz: {return "PC 150";break;}
      case TMDL_HDMITX_VFMT_PC_1152x864p_85Hz: {return "PC 151";break;}
      case TMDL_HDMITX_VFMT_PC_1280x960p_85Hz: {return "PC 152";break;}
      case TMDL_HDMITX_VFMT_PC_1280x1024p_85Hz: {return "PC 153";break;}
      case TMDL_HDMITX_VFMT_PC_1024x768i_87Hz: {return "PC 154";break;}
      case FRAME_PACKING + TMDL_HDMITX_VFMT_02_720x480p_60Hz: {return "vic 02: 720x480p 60Hz frame packing";break;}
      case FRAME_PACKING + TMDL_HDMITX_VFMT_17_720x576p_50Hz: {return "vic 17: 720x576p 50Hz frame packing";break;}
      case FRAME_PACKING + TMDL_HDMITX_VFMT_60_1280x720p_24Hz: {return "vic 60: 1280x720p 24Hz frame packing";break;}
      case FRAME_PACKING + TMDL_HDMITX_VFMT_61_1280x720p_25Hz: {return "vic 61: 1280x720p 25Hz frame packing";break;}
      case FRAME_PACKING + TMDL_HDMITX_VFMT_62_1280x720p_30Hz: {return "vic 62: 1280x720p 30Hz frame packing";break;}
      case FRAME_PACKING + TMDL_HDMITX_VFMT_19_1280x720p_50Hz: {return "vic 19: 1280x720p 50Hz frame packing";break;}
      case FRAME_PACKING + TMDL_HDMITX_VFMT_04_1280x720p_60Hz: {return "vic 04: 1280x720p 60Hz frame packing";break;}
      case FRAME_PACKING + TMDL_HDMITX_VFMT_32_1920x1080p_24Hz: {return "vic 32: 1920x1080p 24Hz frame packing";break;}
      case FRAME_PACKING + TMDL_HDMITX_VFMT_33_1920x1080p_25Hz: {return "vic 33: 1920x1080p 25Hz frame packing";break;}
      case FRAME_PACKING + TMDL_HDMITX_VFMT_34_1920x1080p_30Hz: {return "vic 34: 1920x1080p 30Hz frame packing";break;}     
      case FRAME_PACKING + TMDL_HDMITX_VFMT_31_1920x1080p_50Hz: {return "vic 31: 1920x1080p 50Hz frame packing";break;}
      case FRAME_PACKING + TMDL_HDMITX_VFMT_16_1920x1080p_60Hz: {return "vic 16: 1920x1080p 60Hz frame packing";break;}
      default : {return "unknown video format";break;}
      }
}
#endif

static char *tda_spy_audio_fmt(int fmt)
{
   switch (fmt)
      {
      case TMDL_HDMITX_AFMT_SPDIF: {return "SPDIF";break;}
      case TMDL_HDMITX_AFMT_I2S: {return "I2S";break;}
      case TMDL_HDMITX_AFMT_OBA: {return "OBA";break;}
      case TMDL_HDMITX_AFMT_DST: {return "DST";break;}
      case TMDL_HDMITX_AFMT_HBR: {return "HBR";break;}
      default : {return "Unknown audio format";break;}
      }
}

static char *tda_spy_audio_freq(int freq)
{
   switch (freq)
      {
      case TMDL_HDMITX_AFS_32K: {return "32k";break;}
      case TMDL_HDMITX_AFS_44K: {return "44k";break;}
      case TMDL_HDMITX_AFS_48K: {return "48k";break;}
      case TMDL_HDMITX_AFS_88K: {return "88k";break;}
      case TMDL_HDMITX_AFS_96K: {return "96k";break;}
      case TMDL_HDMITX_AFS_176K: {return "176k";break;}
      case TMDL_HDMITX_AFS_192K: {return "192k";break;}
      default : {return "Unknown audio freq";break;}
      }
}

static char *tda_spy_audio_i2c(int bits)
{
   switch (bits)
      {
      case TMDL_HDMITX_I2SQ_16BITS: {return "16 bits";break;}
      case TMDL_HDMITX_I2SQ_32BITS: {return "32 bits";break;}
      default : {return "Unknown audio i2c sampling";break;}
      }
}

static char *tda_spy_audio_i2c4(int align)
{
   switch (align)
      {
      case TMDL_HDMITX_I2SFOR_PHILIPS_L: {return "Philips Left";break;}
      case TMDL_HDMITX_I2SFOR_OTH_L: {return "other left";break;}
      case TMDL_HDMITX_I2SFOR_OTH_R: {return "other right";break;}
      default : {return "Unknown audio I2C alignement";break;}
      }
}

static void tda_spy_audio(tmdlHdmiTxAudioInConfig_t *audio)
{
   printk(KERN_INFO "hdmitx audio input\n format:%d(%s) rate:%d(%s) i2c_format:%d(%s) i2c_qualif:%d(%s) dst_rate:%d channel:%d\n", \
          audio->format,                                                \
          tda_spy_audio_fmt(audio->format),                             \
          audio->rate,                                                  \
          tda_spy_audio_freq(audio->rate),                              \
          audio->i2sFormat,                                             \
          tda_spy_audio_i2c4(audio->i2sFormat),                         \
          audio->i2sQualifier,                                          \
          tda_spy_audio_i2c(audio->i2sQualifier),                       \
          audio->dstRate,                                               \
          audio->channelAllocation);
 }


static char *tda_ioctl(int io)
{
   switch (io)
      {
      case TDA_VERBOSE_ON_CMD: {return "TDA_VERBOSE_ON_CMD";break;}
      case TDA_VERBOSE_OFF_CMD: {return "TDA_VERBOSE_OFF_CMD";break;}
      case TDA_BYEBYE_CMD: {return "TDA_BYEBYE_CMD";break;}
      case TDA_GET_SW_VERSION_CMD: {return "TDA_GET_SW_VERSION_CMD";break;}
      case TDA_SET_POWER_CMD: {return "TDA_SET_POWER_CMD";break;}
      case TDA_GET_POWER_CMD: {return "TDA_GET_POWER_CMD";break;}
      case TDA_SETUP_CMD: {return "TDA_SETUP_CMD";break;}
      case TDA_GET_SETUP_CMD: {return "TDA_GET_SETUP_CMD";break;}
      case TDA_WAIT_EVENT_CMD: {return "TDA_WAIT_EVENT_CMD";break;}
      case TDA_ENABLE_EVENT_CMD: {return "TDA_ENABLE_EVENT_CMD";break;}
      case TDA_DISABLE_EVENT_CMD: {return "TDA_DISABLE_EVENT_CMD";break;}
      case TDA_GET_VIDEO_SPEC_CMD: {return "TDA_GET_VIDEO_SPEC_CMD";break;}
      case TDA_SET_INPUT_OUTPUT_CMD: {return "TDA_SET_INPUT_OUTPUT_CMD";break;}
      case TDA_SET_AUDIO_INPUT_CMD: {return "TDA_SET_AUDIO_INPUT_CMD";break;}
      case TDA_SET_VIDEO_INFOFRAME_CMD: {return "TDA_SET_VIDEO_INFOFRAME_CMD";break;}
      case TDA_SET_AUDIO_INFOFRAME_CMD: {return "TDA_SET_AUDIO_INFOFRAME_CMD";break;}
      case TDA_SET_ACP_CMD: {return "TDA_SET_ACP_CMD";break;}
      case TDA_SET_GCP_CMD: {return "TDA_SET_GCP_CMD";break;}
      case TDA_SET_ISRC1_CMD: {return "TDA_SET_ISRC1_CMD";break;}
      case TDA_SET_ISRC2_CMD: {return "TDA_SET_ISRC2_CMD";break;}
      case TDA_SET_MPS_INFOFRAME_CMD: {return "TDA_SET_MPS_INFOFRAME_CMD";break;}
      case TDA_SET_SPD_INFOFRAME_CMD: {return "TDA_SET_SPD_INFOFRAME_CMD";break;}
      case TDA_SET_VS_INFOFRAME_CMD: {return "TDA_SET_VS_INFOFRAME_CMD";break;}
      case TDA_SET_AUDIO_MUTE_CMD: {return "TDA_SET_AUDIO_MUTE_CMD";break;}
      case TDA_RESET_AUDIO_CTS_CMD: {return "TDA_RESET_AUDIO_CTS_CMD";break;}
      case TDA_GET_EDID_STATUS_CMD: {return "TDA_GET_EDID_STATUS_CMD";break;}
      case TDA_GET_EDID_AUDIO_CAPS_CMD: {return "TDA_GET_EDID_AUDIO_CAPS_CMD";break;}
      case TDA_GET_EDID_VIDEO_CAPS_CMD: {return "TDA_GET_EDID_VIDEO_CAPS_CMD";break;}
      case TDA_GET_EDID_VIDEO_PREF_CMD: {return "TDA_GET_EDID_VIDEO_PREF_CMD";break;}
      case TDA_GET_EDID_SINK_TYPE_CMD: {return "TDA_GET_EDID_SINK_TYPE_CMD";break;}
      case TDA_GET_EDID_SOURCE_ADDRESS_CMD: {return "TDA_GET_EDID_SOURCE_ADDRESS_CMD";break;}
      case TDA_SET_GAMMUT_CMD: {return "TDA_SET_GAMMUT_CMD";break;}
      case TDA_GET_EDID_DTD_CMD: {return "TDA_GET_EDID_DTD_CMD";break;}
      case TDA_GET_EDID_MD_CMD: {return "TDA_GET_EDID_MD_CMD";break;}
      case TDA_GET_EDID_TV_ASPECT_RATIO_CMD: {return "TDA_GET_EDID_TV_ASPECT_RATIO_CMD";break;}
      case TDA_GET_EDID_LATENCY_CMD: {return "TDA_GET_EDID_LATENCY_CMD";break;}
      case TDA_GET_HPD_STATUS_CMD: {return "TDA_GET_HPD_STATUS_CMD";break;}
      default : {return "unknown";break;}
      }


}

#if defined (TMFL_TDA19989) || defined (TMFL_TDA9984) 
/*
 *  
 */
static int tda_spy(int verbose)
{
   int i,err=0;

   if (!verbose) {
      return err;
   }

   printk(KERN_INFO "\n<edid video caps>\n");
   this->tda.edid_video_caps.max=EXAMPLE_MAX_SVD;
   TRY(tmdlHdmiTxGetEdidVideoCaps(this->tda.instance,              \
                                  this->tda.edid_video_caps.desc, \
                                  this->tda.edid_video_caps.max,      \
                                  &this->tda.edid_video_caps.written, \
                                  &this->tda.edid_video_caps.flags));
   printk(KERN_INFO "written:%d\n",this->tda.edid_video_caps.written);
   printk(KERN_INFO "flags:0X%x\n",this->tda.edid_video_caps.flags);
   if (this->tda.edid_video_caps.written > this->tda.edid_video_caps.max) {
      printk(KERN_ERR "get %d video caps but was waiting for %d\n", \
             this->tda.edid_video_caps.written,                     \
             this->tda.edid_video_caps.max); 
      this->tda.edid_video_caps.written = this->tda.edid_video_caps.max;
   }
   for(i=0; i<this->tda.edid_video_caps.written;i++) {
      printk(KERN_INFO "videoFormat: %s\n",tda_spy_vfmt(this->tda.edid_video_caps.desc[i].videoFormat));
      printk(KERN_INFO "nativeVideoFormat:%s\n",(this->tda.edid_video_caps.desc[i].nativeVideoFormat?"yes":"no"));
   }

   printk(KERN_INFO "\n<edid video timings>\n");
   TRY(tmdlHdmiTxGetEdidVideoPreferred(this->tda.instance, \
                                       &this->tda.edid_video_timings));
   printk(KERN_INFO "Pixel Clock/10 000:%d\n",this->tda.edid_video_timings.pixelClock);
   printk(KERN_INFO "Horizontal Active Pixels:%d\n",this->tda.edid_video_timings.hActivePixels);
   printk(KERN_INFO "Horizontal Blanking Pixels:%d\n",this->tda.edid_video_timings.hBlankPixels);
   printk(KERN_INFO "Vertical Active Lines:%d\n",this->tda.edid_video_timings.vActiveLines);
   printk(KERN_INFO "Vertical Blanking Lines:%d\n",this->tda.edid_video_timings.vBlankLines);
   printk(KERN_INFO "Horizontal Sync Offset:%d\n",this->tda.edid_video_timings.hSyncOffset);
   printk(KERN_INFO "Horiz. Sync Pulse Width:%d\n",this->tda.edid_video_timings.hSyncWidth);
   printk(KERN_INFO "Vertical Sync Offset:%d\n",this->tda.edid_video_timings.vSyncOffset);
   printk(KERN_INFO "Vertical Sync Pulse Width:%d\n",this->tda.edid_video_timings.vSyncWidth);
   printk(KERN_INFO "Horizontal Image Size:%d\n",this->tda.edid_video_timings.hImageSize);
   printk(KERN_INFO "Vertical Image Size:%d\n",this->tda.edid_video_timings.vImageSize);
   printk(KERN_INFO "Horizontal Border:%d\n",this->tda.edid_video_timings.hBorderPixels);
   printk(KERN_INFO "Vertical Border:%d\n",this->tda.edid_video_timings.vBorderPixels);
   printk(KERN_INFO "Interlace/sync info:%x\n",this->tda.edid_video_timings.flags);

   printk(KERN_INFO "\n<sink type>\n");
   TRY(tmdlHdmiTxGetEdidSinkType(this->tda.instance,    \
                                 &this->tda.setio.sink));
   printk(KERN_INFO "%s\n",tda_spy_sink(this->tda.setio.sink));
   printk(KERN_INFO "\n<source address>\n");
   TRY(tmdlHdmiTxGetEdidSourceAddress(this->tda.instance,   \
                                      &this->tda.src_address));
   printk(KERN_INFO "%x\n",this->tda.src_address);
   printk(KERN_INFO "\n<detailled timing descriptors>\n");
   this->tda.edid_dtd.max=EXAMPLE_MAX_SVD;
   TRY(tmdlHdmiTxGetEdidDetailledTimingDescriptors(this->tda.instance,  \
                                                   this->tda.edid_dtd.desc, \
                                                   this->tda.edid_dtd.max, \
                                                   &this->tda.edid_dtd.written));
   printk(KERN_INFO "Interlace/sync info:%x\n",this->tda.edid_dtd.desc[i].flags);
   printk(KERN_INFO "written:%d\n",this->tda.edid_dtd.written);
   if (this->tda.edid_dtd.written > this->tda.edid_dtd.max) {
      printk(KERN_ERR "get %d video caps but was waiting for %d\n", \
             this->tda.edid_dtd.written,                     \
             this->tda.edid_dtd.max); 
      this->tda.edid_dtd.written = this->tda.edid_dtd.max;
   }
   for(i=0; i<this->tda.edid_dtd.written;i++) {
      printk(KERN_INFO "Pixel Clock/10 000:%d\n",this->tda.edid_dtd.desc[i].pixelClock);
      printk(KERN_INFO "Horizontal Active Pixels:%d\n",this->tda.edid_dtd.desc[i].hActivePixels);
      printk(KERN_INFO "Horizontal Blanking Pixels:%d\n",this->tda.edid_dtd.desc[i].hBlankPixels);
      printk(KERN_INFO "Vertical Active Lines:%d\n",this->tda.edid_dtd.desc[i].vActiveLines);
      printk(KERN_INFO "Vertical Blanking Lines:%d\n",this->tda.edid_dtd.desc[i].vBlankLines);
      printk(KERN_INFO "Horizontal Sync Offset:%d\n",this->tda.edid_dtd.desc[i].hSyncOffset);
      printk(KERN_INFO "Horiz. Sync Pulse Width:%d\n",this->tda.edid_dtd.desc[i].hSyncWidth);
      printk(KERN_INFO "Vertical Sync Offset:%d\n",this->tda.edid_dtd.desc[i].vSyncOffset);
      printk(KERN_INFO "Vertical Sync Pulse Width:%d\n",this->tda.edid_dtd.desc[i].vSyncWidth);
      printk(KERN_INFO "Horizontal Image Size:%d\n",this->tda.edid_dtd.desc[i].hImageSize);
      printk(KERN_INFO "Vertical Image Size:%d\n",this->tda.edid_dtd.desc[i].vImageSize);
      printk(KERN_INFO "Horizontal Border:%d\n",this->tda.edid_dtd.desc[i].hBorderPixels);
      printk(KERN_INFO "Vertical Border:%d\n",this->tda.edid_dtd.desc[i].vBorderPixels);
   }

   printk(KERN_INFO "\n<monitor descriptors>\n");
   this->tda.edid_md.max=EXAMPLE_MAX_SVD;
   TRY(tmdlHdmiTxGetEdidMonitorDescriptors(this->tda.instance,  \
                                           this->tda.edid_md.desc1,    \
                                           this->tda.edid_md.desc2,    \
                                           this->tda.edid_md.other,    \
                                           this->tda.edid_md.max,       \
                                           &this->tda.edid_md.written));
   printk(KERN_INFO "written:%d\n",this->tda.edid_md.written);
   if (this->tda.edid_md.written > this->tda.edid_md.max) {
      printk(KERN_ERR "get %d video caps but was waiting for %d\n", \
             this->tda.edid_md.written,                     \
             this->tda.edid_md.max); 
      this->tda.edid_md.written = this->tda.edid_md.max;
   }
   for(i=0; i<this->tda.edid_md.written;i++) {
      if (this->tda.edid_md.desc1[i].descRecord) {
         this->tda.edid_md.desc1[i].monitorName[EDID_MONITOR_DESCRIPTOR_SIZE-1]=0;
         printk(KERN_INFO "Monitor name:%s\n",this->tda.edid_md.desc1[i].monitorName);
      }
      if (this->tda.edid_md.desc1[i].descRecord) {
         printk(KERN_INFO "Min vertical rate in Hz:%d\n",this->tda.edid_md.desc2[i].minVerticalRate);
         printk(KERN_INFO "Max vertical rate in Hz:%d\n",this->tda.edid_md.desc2[i].maxVerticalRate);
         printk(KERN_INFO "Min horizontal rate in Hz:%d\n",this->tda.edid_md.desc2[i].minHorizontalRate);
         printk(KERN_INFO "Max horizontal rate in Hz:%d\n",this->tda.edid_md.desc2[i].maxHorizontalRate);
         printk(KERN_INFO "Max supported pixel clock rate in MHz:%d\n",this->tda.edid_md.desc2[i].maxSupportedPixelClk);
      }
   }

   printk(KERN_INFO "\n<TV picture ratio>\n");
   TRY(tmdlHdmiTxGetEdidTVPictureRatio(this->tda.instance,  \
                                       &this->tda.edid_tv_aspect_ratio));
   printk(KERN_INFO "%s\n",tda_spy_aspect_ratio(this->tda.edid_tv_aspect_ratio));

   printk(KERN_INFO "\n<latency info>\n");
   TRY(tmdlHdmiTxGetEdidLatencyInfo(this->tda.instance, \
                                    &this->tda.edid_latency));
   if (this->tda.edid_latency.latency_available) {
      printk(KERN_INFO "Edid video:%d\n",this->tda.edid_latency.Edidvideo_latency);
      printk(KERN_INFO "Edid audio:%d\n",this->tda.edid_latency.Edidaudio_latency);
   }
   if (this->tda.edid_latency.Ilatency_available) {
      printk(KERN_INFO "Edid Ivideo:%d\n",this->tda.edid_latency.EdidIvideo_latency);
      printk(KERN_INFO "Edid Iaudio:%d\n",this->tda.edid_latency.EdidIaudio_latency);
   }
 TRY_DONE:
   return err;
}
#endif

/*
 *
 * PROCESSING
 * ----------
 * LEVEL 2
 *
 * - 
 *
 */

/*
 * On HDCP
 */
void hdcp_on(tda_instance *this) {

   int err=0;

   if (this->tda.hdcp_status != HDCP_IS_NOT_INSTALLED) { /* check HDCP is installed ... */
         if (this->tda.hdcp_enable) { /* ... but requested ! */ 
            TRY(tmdlHdmiTxSetHdcp(this->tda.instance,True)); /* switch if on */
#if defined (TMFL_TDA19989) || defined (TMFL_TDA9984) 
            /* hide video content until HDCP authentification is finished */
            if (!this->tda.setup.simplayHd) {
               TRY(tmdlHdmiTxSetBScreen(this->tda.instance,TMDL_HDMITX_PATTERN_BLUE));
            }
#endif
         }
   }
 TRY_DONE:
   (void)0;
}

/*
 * Off HDCP
 */
void hdcp_off(tda_instance *this) {

   int err=0;

   if (this->tda.hdcp_status != HDCP_IS_NOT_INSTALLED) { /* check HDCP is installed ... */

         if (this->tda.hdcp_enable) { /* but no more requested */
            TRY(tmdlHdmiTxSetHdcp(this->tda.instance,False)); /* switch if off */
         }
   }
 TRY_DONE:
   (void)0;
}

/*
 * Run video
 */
void show_video(tda_instance *this) {

   int err=0;

   printk("%s rx_device_active = %d, hot_plug_detect = %d, power = %d, src_address = 0x%x\n", 
		   __func__,
		   this->tda.rx_device_active,
		   this->tda.hot_plug_detect,
                   this->tda.power,
                   this->tda.src_address
		   );

   if (this->tda.rx_device_active) { /* check RxSens */
      if (this->tda.hot_plug_detect == TMDL_HDMITX_HOTPLUG_ACTIVE) { /* should be useless, but legacy... */
         if (this->tda.power == tmPowerOn) { /* check CEC or DSS didn't switch it off */
            if (this->tda.src_address != 0xFFFF) { /* check EDID has been received */
				hdcp_off(this);        

            printk("%s tmdlHdmiTxSetInputOutput,format=%d\n", __func__,this->tda.setio.video_out.format);
            
               TRY(tmdlHdmiTxSetInputOutput(this->tda.instance,         \
                                            this->tda.setio.video_in,   \
                                            this->tda.setio.video_out,  \
                                            this->tda.setio.audio_in,   \
                                            this->tda.setio.sink));
               hdcp_on(this);
               
               /*
                 Mind that SetInputOutput disable the blue color matrix settings of tmdlHdmiTxSetBScreen ...
                 so put tmdlHdmiTxSetBScreen (or hdcp_on) always after
               */
            }
         }
      }
   }

 TRY_DONE:
   (void)0;
}

/*
 *  TDA interrupt polling
 */
static void interrupt_polling(struct work_struct *dummy)
{
   int err=0;

   LOG(KERN_INFO, "enter\n");

   /* Tx part */
   TRY(tmdlHdmiTxHandleInterrupt(this->tda.instance));

   /* CEC part */
   if (this->driver.cec_callback) this->driver.cec_callback(dummy);

   /* FIX : IT anti debounce */
   TRY(tmdlHdmiTxHandleInterrupt(this->tda.instance));

 TRY_DONE:

   /* setup next polling */
#ifndef IRQ
   mod_timer(&this->driver.no_irq_timer,jiffies + ( CHECK_EVERY_XX_MS * HZ / 1000 ));
#endif

   (void)0;
}

/*
 *  TDA interrupt polling
 */
static void hdcp_check(struct work_struct *dummy)
{
   int err=0;
   tmdlHdmiTxHdcpCheck_t hdcp_status;

   down(&this->driver.sem);

   if (this->tda.hdcp_status == HDCP_IS_NOT_INSTALLED) goto TRY_DONE;

   TRY(tmdlHdmiTxHdcpCheck(this->tda.instance,HDCP_CHECK_EVERY_MS));
   TRY(tmdlHdmiTxGetHdcpState(this->tda.instance, &hdcp_status));
#if defined (TMFL_TDA19989) || defined (TMFL_TDA9984) 
   if (this->tda.hdcp_status != hdcp_status) {
      LOG(KERN_INFO,"HDCP status:%s\n",tda_spy_hdcp_status(hdcp_status));
      this->tda.hdcp_status = hdcp_status;
   }
#endif
#ifdef HDCP_TEST
   /* TEST */
   if (test++>500) {
      test=0;
      this->tda.hdcp_enable=1-this->tda.hdcp_enable;
      printk("TEST hdcp:%d\n",this->tda.hdcp_enable);
      if (this->tda.rx_device_active) { /* check RxSens */
         if (this->tda.hot_plug_detect == TMDL_HDMITX_HOTPLUG_ACTIVE) { /* should be useless, but legacy... */
            if (this->tda.power == tmPowerOn) { /* check CEC didn't switch it off */
               if (this->tda.src_address != 0xFFFF) { /* check EDID has been received */
                  hdcp_off(this);
                  hdcp_on(this);				  
               }
            }
         }
      }
   }
#endif

 TRY_DONE:

   /* setup next polling */
   mod_timer(&this->driver.hdcp_check,jiffies + ( HDCP_CHECK_EVERY_MS * HZ / 1000 ));

   up(&this->driver.sem);
}

void register_cec_interrupt(cec_callback_t fct)
{
   this->driver.cec_callback = fct;
}
EXPORT_SYMBOL(register_cec_interrupt);

void unregister_cec_interrupt(void)
{
   this->driver.cec_callback = NULL;
}
EXPORT_SYMBOL(unregister_cec_interrupt);

static DECLARE_WORK(wq_irq, interrupt_polling);
void polling_timeout(unsigned long arg)
{
   /* derefered because ATOMIC context of timer does not support I2C_transfert */
   schedule_work(&wq_irq);
}

static DECLARE_WORK(wq_hdcp, hdcp_check);
void hdcp_check_timeout(unsigned long arg)
{
   /* derefered because ATOMIC context of timer does not support I2C_transfert */
   schedule_work(&wq_hdcp);
}

#ifdef IRQ
/*
 *  TDA irq
 */
static irqreturn_t tda_irq(int irq, void *_udc)
{

   /* do it now */
   schedule_work(&wq_irq);

   return IRQ_HANDLED;
}
#endif



/*
 *  TDA callback
 */
static void eventCallbackTx(tmdlHdmiTxEvent_t event)
{
   int err=0;
   uint32 mode=0;
   unsigned short new_addr;
#if defined (TMFL_TDA19989) || defined (TMFL_TDA9984) 
   tda_hdcp_fail hdcp_fail;
#endif

   this->tda.event=event;
   if (TMDL_HDMITX_HDCP_INACTIVE != event) {
      printk(KERN_INFO "hdmi %s\n",tda_spy_event(event));
   }

   switch (event) {
   case TMDL_HDMITX_EDID_RECEIVED:
      printk("%s TMDL_HDMITX_EDID_RECEIVED\n", __func__);
      TRY(tmdlHdmiTxGetEdidSourceAddress(this->tda.instance,        \
                                         &new_addr));
      LOG(KERN_INFO,"phy.@:%x\n",new_addr);
      /*       if (this->tda.src_address == new_addr) { */
      /*          break; */
      /*       } */
      this->tda.src_address = new_addr;
#if defined (TMFL_TDA19989) || defined (TMFL_TDA9984) 
      tda_spy(this->param.verbose>1);
#endif
      /* 
         Customer may add stuff to analyse EDID (see tda_spy())
         and select automatically some video/audio settings.
         By default, let go on with next case and activate
         default video/audio settings with tmdlHdmiTxSetInputOutput()
      */

      TRY(tmdlHdmiTxGetEdidSinkType(this->tda.instance,     \
                                    &this->tda.setio.sink));
      if (TMDL_HDMITX_SINK_HDMI != this->tda.setio.sink) {
         printk(KERN_INFO "/!\\ CAUTION /!\\ sink is not HDMI but %s\n",tda_spy_sink(this->tda.setio.sink));
      }
      
      
      tda9988_read_edid();
      mode =  tda9988_get_bestmode();

      if(mode != this->tda.setio.video_out.format)
        {
              this->tda.setio.video_out.format = mode;
              this->tda.setio.video_in.format = this->tda.setio.video_out.format;                    
        }
      
        //msleep(100);                
      /*
        /!\ WARNING /!                                              \
        the core driver does not send any HPD nor RXSENS when HDMI was plugged after at boot time
        and only EDID_RECEIVED is send, so rx_device_active shall be forced now.
        Do not skip the next case nor add any break here please
      */
   case TMDL_HDMITX_RX_DEVICE_ACTIVE: /* TV is ready to receive */      
      printk("%s  TMDL_HDMITX_RX_DEVICE_ACTIVE\n", __func__);            
      this->tda.rx_device_active = 1;           
      hdmi_enable();
      show_video(this);
      if(this->tda.src_address != NO_PHY_ADDR)
          change_hdmi_state(1);      
      break;
      
   case TMDL_HDMITX_RX_DEVICE_INACTIVE: /* TV is ignoring the source */
      this->tda.rx_device_active = 0;
      hdmi_disable(1);	          
      change_hdmi_state(0);
      break;
      
   case TMDL_HDMITX_HPD_ACTIVE: /* HDMI is so funny u can get RxSens without being plugged !!! */
      this->tda.hot_plug_detect = TMDL_HDMITX_HOTPLUG_ACTIVE;
#ifdef ANDROID_DSS_MSM
      if(this->tda.rx_device_active != 0)
      		hdmi_enable();
#endif
      printk("Event callback Tx : Hot-plug status switched to ACTIVE\n");
      break;
      
   case TMDL_HDMITX_HPD_INACTIVE: /* unplug */
      this->tda.hot_plug_detect = TMDL_HDMITX_HOTPLUG_INACTIVE;
#ifdef ANDROID_DSS_MSM
      if(this->tda.rx_device_active != 0)
  	hdmi_disable(1);	     
#endif
      this->tda.src_address = 0xFFFF;
      break;
      
#if defined (TMFL_TDA19989) || defined (TMFL_TDA9984) 
   case TMDL_HDMITX_HDCP_INACTIVE: /* HDCP drops off */
      tmdlHdmiTxGetHdcpFailStatus(this->tda.instance, \
                                  &hdcp_fail, \
                                  &this->tda.hdcp_raw_status);
      if (this->tda.hdcp_fail != hdcp_fail) {
         if (this->tda.hdcp_fail) {
            LOG(KERN_INFO,"%s (%d)\n",tda_spy_hsdc_fail_status(this->tda.hdcp_fail),this->tda.hdcp_raw_status);
         }
         this->tda.hdcp_fail = hdcp_fail;
         tmdlHdmiTxSetBScreen(this->tda.instance,TMDL_HDMITX_PATTERN_BLUE);
      }
#ifdef ANDROID_DSS_MSM
      external_common_state->hdcp_active = 0;
#endif
      break;

   case TMDL_HDMITX_HDCP_ACTIVE:
#ifdef ANDROID_DSS_MSM
      external_common_state->hdcp_active = 1;
#endif
      break;

   case TMDL_HDMITX_RX_KEYS_RECEIVED: /* end of HDCP authentification */
      if (!this->tda.setup.simplayHd) {
         tmdlHdmiTxRemoveBScreen(this->tda.instance);
      }
      break;
#endif
   default:
      break;
   }

   this->driver.poll_done=true;
   wake_up_interruptible(&this->driver.wait);
   
 TRY_DONE:
   (void)0;
}

/*
 *  hdmi Tx init
 */
static int hdmi_tx_init(tda_instance *this)
{
   int err=0;

   LOG(KERN_INFO,"called\n");


   /*Initialize HDMI Transmiter*/
   TRY(tmdlHdmiTxOpen(&this->tda.instance));
   /* Register the HDMI TX events callbacks */
   TRY(tmdlHdmiTxRegisterCallbacks(this->tda.instance,(ptmdlHdmiTxCallback_t)eventCallbackTx));
   /* EnableEvent, all by default */
   TRY(tmdlHdmiTxEnableEvent(this->tda.instance,TMDL_HDMITX_HDCP_ACTIVE));
   TRY(tmdlHdmiTxEnableEvent(this->tda.instance,TMDL_HDMITX_HDCP_INACTIVE));
   TRY(tmdlHdmiTxEnableEvent(this->tda.instance,TMDL_HDMITX_HPD_ACTIVE));
   TRY(tmdlHdmiTxEnableEvent(this->tda.instance,TMDL_HDMITX_HPD_INACTIVE));
   TRY(tmdlHdmiTxEnableEvent(this->tda.instance,TMDL_HDMITX_RX_KEYS_RECEIVED));
   TRY(tmdlHdmiTxEnableEvent(this->tda.instance,TMDL_HDMITX_RX_DEVICE_ACTIVE));
   TRY(tmdlHdmiTxEnableEvent(this->tda.instance,TMDL_HDMITX_RX_DEVICE_INACTIVE));
   TRY(tmdlHdmiTxEnableEvent(this->tda.instance,TMDL_HDMITX_EDID_RECEIVED));

   /* Size of the application EDID buffer */
   this->tda.setup.edidBufferSize=EDID_BLOCK_COUNT * EDID_BLOCK_SIZE;
   /* Buffer to store the application EDID data */
   this->tda.setup.pEdidBuffer=this->tda.raw_edid;
   /* To Enable/disable repeater feature, nor relevant here */
   this->tda.setup.repeaterEnable=false;
   /* To enable/disable simplayHD feature: blue screen when not authenticated */
#ifdef SIMPLAYHD
   this->tda.setup.simplayHd=(this->tda.hdcp_enable?true:false);
#else
   this->tda.setup.simplayHd=false;
#endif

   /* Provides HDMI TX instance configuration */
   TRY(tmdlHdmiTxInstanceSetup(this->tda.instance,&this->tda.setup));
   /* Get IC version */
   TRY(tmdlHdmiTxGetCapabilities(&this->tda.capabilities));

   /* Main settings */
   this->tda.setio.video_out.mode = TMDL_HDMITX_VOUTMODE_RGB444;
   this->tda.setio.video_out.colorDepth = TMDL_HDMITX_COLORDEPTH_24;
#ifdef TMFL_TDA19989
   this->tda.setio.video_out.dviVqr = TMDL_HDMITX_VQR_DEFAULT; /* Use HDMI rules for DVI output */
#endif
   /*    this->tda.setio.video_out.format = TMDL_HDMITX_VFMT_31_1920x1080p_50Hz; */
   /*    this->tda.setio.video_out.format = TMDL_HDMITX_VFMT_PC_640x480p_60Hz; */
   /*    this->tda.setio.video_out.format = TMDL_HDMITX_VFMT_PC_640x480p_72Hz; */
   this->tda.setio.video_out.format = TMDL_HDMITX_VFMT_04_1280x720p_60Hz; 
  //this->tda.setio.video_out.format = TMDL_HDMITX_VFMT_01_640x480p_60Hz; 
   /*    this->tda.setio.video_out.format = TMDL_HDMITX_VFMT_19_1280x720p_50Hz; */
   /*    this->tda.setio.video_out.format = TMDL_HDMITX_VFMT_02_720x480p_60Hz; */

   this->tda.setio.video_in.mode = TMDL_HDMITX_VINMODE_RGB444;
   /*    this->tda.setio.video_in.mode = TMDL_HDMITX_VINMODE_CCIR656; */
   /*    this->tda.setio.video_in.mode = TMDL_HDMITX_VINMODE_YUV422; */
   this->tda.setio.video_in.format = this->tda.setio.video_out.format;
   this->tda.setio.video_in.pixelRate = TMDL_HDMITX_PIXRATE_SINGLE;
   this->tda.setio.video_in.syncSource = TMDL_HDMITX_SYNCSRC_EXT_VS; /* we use HS,VS as synchronisation source */

   this->tda.setio.audio_in.format = TMDL_HDMITX_AFMT_I2S;
   this->tda.setio.audio_in.rate = TMDL_HDMITX_AFS_48K;
   this->tda.setio.audio_in.i2sFormat = TMDL_HDMITX_I2SFOR_PHILIPS_L;
   this->tda.setio.audio_in.i2sQualifier = TMDL_HDMITX_I2SQ_16BITS;
   this->tda.setio.audio_in.dstRate = TMDL_HDMITX_DSTRATE_SINGLE; /* not relevant here */
   this->tda.setio.audio_in.channelAllocation = 0; /* audio channel allocation (Ref to CEA-861D p85) */
   /* audio channel allocation (Ref to CEA-861D p85) */
   this->tda.setio.audio_in.channelStatus.PcmIdentification = TMDL_HDMITX_AUDIO_DATA_PCM;
   this->tda.setio.audio_in.channelStatus.CopyrightInfo = TMDL_HDMITX_CSCOPYRIGHT_UNPROTECTED;
   this->tda.setio.audio_in.channelStatus.FormatInfo = TMDL_HDMITX_CSFI_PCM_2CHAN_NO_PRE;
   this->tda.setio.audio_in.channelStatus.categoryCode = 0;
   this->tda.setio.audio_in.channelStatus.clockAccuracy = TMDL_HDMITX_CSCLK_LEVEL_II;
   this->tda.setio.audio_in.channelStatus.maxWordLength = TMDL_HDMITX_CSMAX_LENGTH_24;
   this->tda.setio.audio_in.channelStatus.wordLength = TMDL_HDMITX_CSWORD_DEFAULT;
   this->tda.setio.audio_in.channelStatus.origSampleFreq = TMDL_HDMITX_CSOFREQ_48k;


   this->tda.setio.sink = TMDL_HDMITX_SINK_HDMI; /* skip edid reading */
   /*    this->tda.src_address = 0x1000; /\* debug *\/ */
   this->tda.src_address = NO_PHY_ADDR; /* it's unref */

 TRY_DONE:
   return err;
}

void reset_hdmi(int hdcp_module)
{
   int err=0;

   down(&this->driver.sem);

   /* PATCH because of SetPowerState that calls SetHdcp that has just been removed by nwolc :( */
   if (hdcp_module==2) {
      tmdlHdmiTxSetHdcp(this->tda.instance,0);
      goto TRY_DONE;
   }

   TRY(tmdlHdmiTxSetPowerState(this->tda.instance,tmPowerStandby));
   tmdlHdmiTxClose(this->tda.instance);

   /* reset */
   this->tda.hdcp_enable = (hdcp_module?1:0);
   hdmi_tx_init(this);
   /* recover previous power state */
   TRY(tmdlHdmiTxSetPowerState(this->tda.instance,this->tda.power));
   tmdlHdmiTxGetHPDStatus(this->tda.instance,&this->tda.hot_plug_detect); /* check if activ for timer */
#ifndef USER_SET_INPUT_OUTPUT
   show_video(this);
#endif

   /* wake up or shut down hdcp checking */
   if (hdcp_module) {
      this->driver.hdcp_check.expires = jiffies +  ( HDCP_CHECK_EVERY_MS * HZ / 1000 );
      add_timer(&this->driver.hdcp_check);
      this->tda.hdcp_status = TMDL_HDMITX_HDCP_CHECK_NOT_STARTED;
#if defined (TMFL_TDA19989) || defined (TMFL_TDA9984) 
      tmdlHdmiTxSetBScreen(this->tda.instance,TMDL_HDMITX_PATTERN_BLUE);
#endif
   }
   else {
      del_timer(&this->driver.hdcp_check);
      this->tda.hdcp_status = HDCP_IS_NOT_INSTALLED; 
   }

 TRY_DONE:
   up(&this->driver.sem);
}
EXPORT_SYMBOL(reset_hdmi);

/*
 *  
 */
short edid_phy_addr(void)
{
   return this->tda.src_address;
}
EXPORT_SYMBOL(edid_phy_addr);

/*
 *  
 */
tda_power get_hdmi_status(void)
{ 
   return this->tda.power;
}
EXPORT_SYMBOL(get_hdmi_status);

/*
 *  
 */
tda_power get_hpd_status(void)
{ 
   return (this->tda.hot_plug_detect == TMDL_HDMITX_HOTPLUG_ACTIVE);
}
EXPORT_SYMBOL(get_hpd_status);

/*
 *  
 */
int edid_received(void)
{ 
   return (this->tda.event == TMDL_HDMITX_EDID_RECEIVED);
}
EXPORT_SYMBOL(edid_received);


/*
 *  ioctl driver :: opening
 */

static int this_cdev_open(struct inode *pInode, struct file *pFile)
{
   tda_instance *this;
   int minor=iminor(pInode);

   if(minor >= MAX_MINOR) {
      printk(KERN_ERR "hdmitx:%s:only one tda can be open\n",__func__);
      return -EINVAL;
   }

   if ((pFile->private_data != NULL) && (pFile->private_data != &our_instance)) {
      printk(KERN_ERR "hdmitx:%s:pFile missmatch\n",__func__);
   }
   this = pFile->private_data = &our_instance;
   down(&this->driver.sem);

   LOG(KERN_INFO,"major:%d minor:%d user:%d\n", imajor(pInode), iminor(pInode), this->driver.user_counter);

   if ((this->driver.user_counter++) && (this->driver.minor == minor)) {
      /* init already done */
      up(&this->driver.sem);
      return 0;
   }
   this->driver.minor = minor;


   up(&this->driver.sem);
   return 0;
}

/*
 *  ioctl driver :: ioctl
 */
static long this_cdev_ioctl(struct file *pFile, unsigned int cmd, unsigned long arg)
//static int this_cdev_ioctl(struct inode *pInode, struct file *pFile, unsigned int cmd, unsigned long arg)
{
   tda_instance* this = pFile->private_data;
   int err=0;

   LOG(KERN_INFO,":%s\n",tda_ioctl(_IOC_NR(cmd)));
//   BUG_ON(this->driver.minor!=iminor(pInode));
   if (_IOC_TYPE(cmd) != TDA_IOCTL_BASE) {
      printk(KERN_INFO "hdmitx:%s:unknown ioctl type: %x\n",__func__,_IOC_TYPE(cmd));
      return -ENOIOCTLCMD;
   }

   if (_IOC_DIR(cmd) & _IOC_READ) 
      err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd)) || !arg;
   else if (_IOC_DIR(cmd) & _IOC_WRITE)
      err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd)) || !arg;
   if (err) {
      printk(KERN_ERR "hdmitx:%s:argument access denied (check address vs value)\n",__func__);
      printk(KERN_ERR "_IOC_DIR:%d arg:%lx\n",_IOC_DIR(cmd),arg);
      return -EFAULT;
   }
   
   down(&this->driver.sem);

   /* Check DevLib consistancy here */

   switch ( _IOC_NR(cmd) )
      {
      case TDA_VERBOSE_ON_CMD:
         {
            this->param.verbose=1;
            printk(KERN_INFO "hdmitx:verbose on\n");
            break;
         }

      case TDA_VERBOSE_OFF_CMD:
         {
            printk(KERN_INFO "hdmitx:verbose off\n");
            this->param.verbose=0;
            break;
         }

      case TDA_BYEBYE_CMD:
         {
            LOG(KERN_INFO,"release event handeling request\n");
            this->tda.event=RELEASE;
            this->driver.poll_done = true;
            wake_up_interruptible(&this->driver.wait);
            break;
         }

      case TDA_GET_SW_VERSION_CMD:
         {
            TRY(tmdlHdmiTxGetSWVersion(&this->tda.version));
            BUG_ON(copy_to_user((tda_version*)arg,&this->tda.version,sizeof(tda_version)) != 0);
            break;
         }

      case TDA_SET_POWER_CMD:
         {
            if (this->driver.enable) {
               /* DSS uses HDMI panel => do not switch the power through the ioctl, this will be done be DSS */
            }
            else {
               BUG_ON(copy_from_user(&this->tda.power,(tda_power*)arg,sizeof(tda_power)) != 0);
               TRY(tmdlHdmiTxSetPowerState(this->tda.instance,this->tda.power));
            }
            break;
         }

      case TDA_GET_POWER_CMD:
         {
            TRY(tmdlHdmiTxGetPowerState(this->tda.instance, \
                                        &this->tda.power));
            BUG_ON(copy_to_user((tda_power*)arg,&this->tda.power,sizeof(tda_power)) != 0);
            break;
         }

      case TDA_SETUP_CMD:
         {
            BUG_ON(copy_from_user(&this->tda.setup,(tda_setup_info*)arg,sizeof(tda_setup_info)) != 0);
            TRY(tmdlHdmiTxInstanceSetup(this->tda.instance, \
                                        &this->tda.setup));
            break;
         }

      case TDA_GET_SETUP_CMD:
         {
            TRY(tmdlHdmiTxGetInstanceSetup(this->tda.instance, \
                                           &this->tda.setup));
            BUG_ON(copy_to_user((tda_setup*)arg,&this->tda.setup,sizeof(tda_setup)) != 0);
            break;
         }

      case TDA_WAIT_EVENT_CMD:
         {
            this->driver.poll_done = false;
            up(&this->driver.sem);
            if (wait_event_interruptible(this->driver.wait,this->driver.poll_done)) return -ERESTARTSYS;
            down(&this->driver.sem);
            BUG_ON(copy_to_user((tda_event*)arg,&this->tda.event,sizeof(tda_event)) != 0);
            break;
         }

      case TDA_ENABLE_EVENT_CMD:
         {
            tmdlHdmiTxEvent_t event;
            BUG_ON(copy_from_user(&event,(tmdlHdmiTxEvent_t*)arg,sizeof(tmdlHdmiTxEvent_t)) != 0);
            TRY(tmdlHdmiTxEnableEvent(this->tda.instance,event));
            break;
         }

      case TDA_DISABLE_EVENT_CMD:
         {
            tmdlHdmiTxEvent_t event;
            BUG_ON(copy_from_user(&event,(tmdlHdmiTxEvent_t*)arg,sizeof(tmdlHdmiTxEvent_t)) != 0);
            TRY(tmdlHdmiTxDisableEvent(this->tda.instance,event));
            break;
         }

      case TDA_GET_VIDEO_SPEC_CMD:
         {
            BUG_ON(copy_from_user(&this->tda.video_fmt,(tda_video_format*)arg,sizeof(tda_video_format)) != 0);
            TRY(tmdlHdmiTxGetVideoFormatSpecs(this->tda.instance, \
                                              this->tda.video_fmt.id, \
                                              &this->tda.video_fmt.spec));
            BUG_ON(copy_to_user((tda_video_format*)arg,&this->tda.video_fmt,sizeof(tda_video_format)) != 0);
            break;
         }

      case TDA_SET_INPUT_OUTPUT_CMD:
         {
            BUG_ON(copy_from_user(&this->tda.setio,(tda_set_in_out*)arg,sizeof(tda_set_in_out)) != 0);

            /*             TRY(tmdlHdmiTxSetInputOutput(this->tda.instance, \ */
            /*                                          this->tda.setio.video_in, \ */
            /*                                          this->tda.setio.video_out, \ */
            /*                                          this->tda.setio.audio_in, \ */
            /*                                          this->tda.setio.sink)); */
            break;
         }

      case TDA_SET_AUDIO_INPUT_CMD:
         {
            BUG_ON(copy_from_user(&this->tda.setio.audio_in,(tda_set_audio_in*)arg,sizeof(tda_set_audio_in)) != 0);
            TRY(tmdlHdmiTxSetAudioInput(this->tda.instance, \
                                        this->tda.setio.audio_in, \
                                        this->tda.setio.sink));
            break;
         }

      case TDA_SET_VIDEO_INFOFRAME_CMD:
         {
            BUG_ON(copy_from_user(&this->tda.video_infoframe,(tda_video_infoframe*)arg,sizeof(tda_video_infoframe)) != 0);
            TRY(tmdlHdmiTxSetVideoInfoframe(this->tda.instance, \
                                            this->tda.video_infoframe.enable, \
                                            &this->tda.video_infoframe.data));
            break;
         }

      case TDA_SET_AUDIO_INFOFRAME_CMD:
         {
            BUG_ON(copy_from_user(&this->tda.audio_infoframe,(tda_audio_infoframe*)arg,sizeof(tda_audio_infoframe)) != 0);
            TRY(tmdlHdmiTxSetAudioInfoframe(this->tda.instance, \
                                            this->tda.audio_infoframe.enable, \
                                            &this->tda.audio_infoframe.data));
            break;
         }

      case TDA_SET_ACP_CMD:
         {
            BUG_ON(copy_from_user(&this->tda.acp,(tda_acp*)arg,sizeof(tda_acp)) != 0);
            TRY(tmdlHdmiTxSetACPPacket(this->tda.instance, \
                                       this->tda.acp.enable, \
                                       &this->tda.acp.data));
            break;
         }

      case TDA_SET_GCP_CMD:
         {
            BUG_ON(copy_from_user(&this->tda.gcp,(tda_gcp*)arg,sizeof(tda_gcp)) != 0);
            TRY(tmdlHdmiTxSetGeneralControlPacket(this->tda.instance, \
                                                  this->tda.gcp.enable, \
                                                  &this->tda.gcp.data));
            break;
         }

      case TDA_SET_ISRC1_CMD:
         {
            BUG_ON(copy_from_user(&this->tda.isrc1,(tda_isrc1*)arg,sizeof(tda_isrc1)) != 0);
            TRY(tmdlHdmiTxSetISRC1Packet(this->tda.instance, \
                                         this->tda.isrc1.enable, \
                                         &this->tda.isrc1.data));
            break;
         }

      case TDA_SET_MPS_INFOFRAME_CMD:
         {
            BUG_ON(copy_from_user(&this->tda.mps_infoframe,(tda_mps_infoframe*)arg,sizeof(tda_mps_infoframe)) != 0);
            TRY(tmdlHdmiTxSetMPSInfoframe(this->tda.instance, \
                                          this->tda.mps_infoframe.enable, \
                                          &this->tda.mps_infoframe.data));
            break;
         }

      case TDA_SET_SPD_INFOFRAME_CMD:
         {
            BUG_ON(copy_from_user(&this->tda.spd_infoframe,(tda_spd_infoframe*)arg,sizeof(tda_spd_infoframe)) != 0);
            TRY(tmdlHdmiTxSetSpdInfoframe(this->tda.instance, \
                                          this->tda.spd_infoframe.enable, \
                                          &this->tda.spd_infoframe.data));
            break;
         }

      case TDA_SET_VS_INFOFRAME_CMD:
         {
            BUG_ON(copy_from_user(&this->tda.vs_infoframe,(tda_vs_infoframe*)arg,sizeof(tda_vs_infoframe)) != 0);
            TRY(tmdlHdmiTxSetVsInfoframe(this->tda.instance, \
                                         this->tda.vs_infoframe.enable, \
                                         &this->tda.vs_infoframe.data));
            break;
         }

      case TDA_SET_AUDIO_MUTE_CMD:
         {
            BUG_ON(copy_from_user(&this->tda.audio_mute,(bool*)arg,sizeof(bool)) != 0);
            TRY(tmdlHdmiTxSetAudioMute(this->tda.instance,			\
                                       this->tda.audio_mute));
            break;
         }

      case TDA_RESET_AUDIO_CTS_CMD:
         {
            TRY(tmdlHdmiTxResetAudioCts(this->tda.instance));
            break;
         }

      case TDA_GET_EDID_STATUS_CMD:
         {
            TRY(tmdlHdmiTxGetEdidStatus(this->tda.instance, \
                                        &this->tda.edid.status, \
                                        &this->tda.edid.block_count));
            BUG_ON(copy_to_user((tda_edid*)arg,&this->tda.edid,sizeof(tda_edid)) != 0);
            break;
         }

      case TDA_GET_EDID_AUDIO_CAPS_CMD:
         {
            BUG_ON(copy_from_user(&this->tda.edid_audio_caps,(tda_edid_audio_caps*)arg,sizeof(tda_edid_audio_caps)) != 0);
            TRY(tmdlHdmiTxGetEdidAudioCaps(this->tda.instance, \
                                           this->tda.edid_audio_caps.desc, \
                                           this->tda.edid_audio_caps.max, \
                                           &this->tda.edid_audio_caps.written, \
                                           &this->tda.edid_audio_caps.flags));
            BUG_ON(copy_to_user((tda_edid_audio_caps*)arg,&this->tda.edid_audio_caps,sizeof(tda_edid_audio_caps)) != 0);
            break;
         }

      case TDA_GET_EDID_VIDEO_CAPS_CMD:
         {
            BUG_ON(copy_from_user(&this->tda.edid_video_caps,(tda_edid_video_caps*)arg,sizeof(tda_edid_video_caps)) != 0);
            TRY(tmdlHdmiTxGetEdidVideoCaps(this->tda.instance, \
                                           this->tda.edid_video_caps.desc, \
                                           this->tda.edid_video_caps.max, \
                                           &this->tda.edid_video_caps.written, \
                                           &this->tda.edid_video_caps.flags));
            BUG_ON(copy_to_user((tda_edid_video_caps*)arg,&this->tda.edid_video_caps,sizeof(tda_edid_video_caps)) != 0);
            break;
         }

      case TDA_GET_EDID_VIDEO_PREF_CMD:
         {
            TRY(tmdlHdmiTxGetEdidVideoPreferred(this->tda.instance, \
                                                &this->tda.edid_video_timings));
            BUG_ON(copy_to_user((tda_edid_video_timings*)arg,&this->tda.edid_video_timings,sizeof(tda_edid_video_timings)) != 0);
            break;
         }

      case TDA_GET_EDID_SINK_TYPE_CMD:
         {
            TRY(tmdlHdmiTxGetEdidSinkType(this->tda.instance, \
                                          &this->tda.setio.sink));
            BUG_ON(copy_to_user((tda_sink*)arg,&this->tda.setio.sink,sizeof(tda_sink)) != 0);
            break;
         }

      case TDA_GET_EDID_SOURCE_ADDRESS_CMD:
         {
            TRY(tmdlHdmiTxGetEdidSourceAddress(this->tda.instance, \
                                               &this->tda.src_address));
            BUG_ON(copy_to_user((unsigned short*)arg,&this->tda.src_address,sizeof(unsigned short)) != 0);
            break;
         }

      case TDA_SET_GAMMUT_CMD:
         {
            BUG_ON(copy_from_user(&this->tda.gammut,(tda_gammut*)arg,sizeof(tda_gammut)) != 0);
            TRY(tmdlHdmiTxSetGamutPacket(this->tda.instance, \
                                         this->tda.gammut.enable, \
                                         &this->tda.gammut.data));
            break;
         }
	 
      case TDA_GET_EDID_DTD_CMD:
         {
            BUG_ON(copy_from_user(&this->tda.edid_dtd,(tda_edid_dtd*)arg,sizeof(tda_edid_dtd)) != 0);
            TRY(tmdlHdmiTxGetEdidDetailledTimingDescriptors(this->tda.instance, \
                                                            this->tda.edid_dtd.desc, \
                                                            this->tda.edid_dtd.max, \
                                                            &this->tda.edid_dtd.written));
            BUG_ON(copy_to_user((tda_edid_dtd*)arg,&this->tda.edid_dtd,sizeof(tda_edid_dtd)) != 0);
            break;
         }
     
#if defined (TMFL_TDA19989) || defined (TMFL_TDA9984) 
      case TDA_GET_EDID_MD_CMD:
         {
            BUG_ON(copy_from_user(&this->tda.edid_md,(tda_edid_md*)arg,sizeof(tda_edid_md)) != 0);
            TRY(tmdlHdmiTxGetEdidMonitorDescriptors(this->tda.instance, \
                                                    this->tda.edid_md.desc1, \
                                                    this->tda.edid_md.desc2, \
                                                    this->tda.edid_md.other, \
                                                    this->tda.edid_md.max, \
                                                    &this->tda.edid_md.written));
            BUG_ON(copy_to_user((tda_edid_md*)arg,&this->tda.edid_md,sizeof(tda_edid_md)) != 0);
            break;
         }

      case TDA_GET_EDID_TV_ASPECT_RATIO_CMD:
         {
            TRY(tmdlHdmiTxGetEdidTVPictureRatio(this->tda.instance, \
                                                &this->tda.edid_tv_aspect_ratio));
            BUG_ON(copy_to_user((tda_edid_tv_aspect_ratio*)arg,&this->tda.edid_tv_aspect_ratio,sizeof(tda_edid_tv_aspect_ratio)) != 0);
            break;
         }

      case TDA_GET_EDID_LATENCY_CMD:
         {
            TRY(tmdlHdmiTxGetEdidLatencyInfo(this->tda.instance, \
                                             &this->tda.edid_latency));
            BUG_ON(copy_to_user((tda_edid_latency*)arg,&this->tda.edid_latency,sizeof(tda_edid_latency)) != 0);
            break;
         }

      case TDA_SET_HDCP_CMD:
         {
            BUG_ON(copy_from_user(&this->tda.hdcp_enable,(bool*)arg,sizeof(bool)) != 0);
            break;
         }

      case TDA_GET_HPD_STATUS_CMD:
         {
            tmdlHdmiTxGetHPDStatus(this->tda.instance,&this->tda.hot_plug_detect); 
            BUG_ON(copy_to_user((tmdlHdmiTxHotPlug_t*)arg,&this->tda.hot_plug_detect,sizeof(tmdlHdmiTxHotPlug_t)) != 0);
            break;
         }
      case TDA_GET_HDCP_STATUS_CMD:
         {
            BUG_ON(copy_to_user((tda_edid_latency*)arg,&this->tda.hdcp_status,sizeof(tda_hdcp_status)) != 0);
            break;
         }
#endif

      default:
         {
            /* unrecognized ioctl */	
            printk(KERN_INFO "hdmitx:%s:unknown ioctl number: %x\n",__func__,cmd);
            up(&this->driver.sem);
            return -ENOIOCTLCMD;
         }
      }

 TRY_DONE:
   up(&this->driver.sem);
   return err;
}

/*
 *  ioctl driver :: releasing
 */
static int this_cdev_release(struct inode *pInode, struct file *pFile)
{
   tda_instance* this = pFile->private_data;
   int minor = iminor(pInode);

   LOG(KERN_INFO,"called\n");

   if(minor >= MAX_MINOR) {
      LOG(KERN_ERR,"minor too big!\n");
      return -EINVAL;
   }

   BUG_ON(this->driver.minor!=iminor(pInode));
   down(&this->driver.sem);

   this->driver.user_counter--;
   if(this->driver.user_counter == 0) {
      pFile->private_data = NULL;
   }
   else {
      LOG(KERN_INFO,"Still %d users pending\n",this->driver.user_counter);
   }

   up(&this->driver.sem);
   return 0;
}

/*
 *  I2C client :: creation
 */
static int this_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
   int err=0;

   LOG(KERN_INFO,"called\n");

   /*
     I2C setup
   */
   if (this->driver.i2c_client) {
      dev_err(&this->driver.i2c_client->dev, "<%s> HDMI Device already created \n",
              __func__);
      return -ENODEV;
   }

   this->driver.i2c_client = client;
   i2c_set_clientdata(client, this);


   pp = client->dev.platform_data;
   tda998x_power_enable(TDA998X_POWER_ALL,1);   
    
   /* I2C ok, then let's startup TDA */
   err = hdmi_tx_init(this);
   if (err) goto i2c_out;
   this->tda.hdcp_enable = 0;
   /* Standby the HDMI TX instance : this is mandatory for TDA boot up sequence, do not change it */
   this->tda.power = tmPowerStandby;  /* power start sequence phase 1, see phase 2 */
   tmdlHdmiTxSetPowerState(this->tda.instance,this->tda.power);
   /* update HPD */
   //tmdlHdmiTxGetHPDStatus(this->tda.instance,&this->tda.hot_plug_detect);


#ifdef ANDROID_DSS_MSM
{
	int err = 1;
        struct platform_device *fb_dev; 

	memset(&hdmi_common, 0, sizeof(hdmi_common));
	external_common_state = &hdmi_common;

	external_common_state->hpd_feature = tda998x_hpd_feature;
        external_common_state->dev = &client->dev;
         external_common_state->video_resolution = HDMI_VFRMT_1280x720p60_16_9;
       //external_common_state->video_resolution = HDMI_VFRMT_640x480p60_4_3;
        external_common_state->read_edid_block = tda9988_read_edid_block;
  
	HDMI_SETUP_LUT(640x480p60_4_3);
	HDMI_SETUP_LUT(720x480p60_4_3);
	HDMI_SETUP_LUT(720x480p60_16_9);
	HDMI_SETUP_LUT(1280x720p60_16_9);
	HDMI_SETUP_LUT(1440x480i60_4_3);
	HDMI_SETUP_LUT(1440x480i60_16_9);
	HDMI_SETUP_LUT(720x576p50_4_3);
	HDMI_SETUP_LUT(720x576p50_16_9);
	HDMI_SETUP_LUT(1280x720p50_16_9);
	HDMI_SETUP_LUT(1440x576i50_4_3);
	HDMI_SETUP_LUT(1440x576i50_16_9);


        hdmi_common_init_panel_info(&tda998x_panel_data.panel_info);

	fb_dev = msm_fb_add_device(&tda998x_panel_device);

        if (fb_dev) {
                err = external_common_state_create(fb_dev);
		pp->init_irq();
        } else 
                DEV_ERR("this_i2c_probe: failed to add fb device\n");

#ifdef CONFIG_FB_MSM_HDMI_AS_PRIMARY
	external_common_state->sdev.name = "hdmi_as_primary";
#else
	external_common_state->sdev.name = "hdmi";
#endif
	if (switch_dev_register(&external_common_state->sdev) < 0)
		DEV_ERR("Hdmi switch registration failed\n");

}
#endif

   if (err) goto i2c_tx_out;

   /* prepare event */
   this->driver.poll_done = true; /* currently idle */
   init_waitqueue_head(&this->driver.wait);

#ifdef IRQ

   this->driver.gpio = 18;
   #if 0
   err=request_irq(client->irq, tda_irq, 
                   IRQF_TRIGGER_FALLING|IRQF_DISABLED, "TDA IRQ", this);
   disable_irq(client->irq);
   LOG(KERN_INFO, "irq number is : %d , status of request_irq: %d\n", client->irq, err);

   if (err <0) {
      printk(KERN_ERR "hdmitx:%s:Cannot request irq, err:%d\n",__func__,err);
      gpio_free(this->driver.gpio);
      goto i2c_out;
   }
   #endif
#else
   init_timer(&this->driver.no_irq_timer);
   this->driver.no_irq_timer.function=polling_timeout;
   this->driver.no_irq_timer.data=0;
   this->driver.no_irq_timer.expires = jiffies + HZ; /* start polling in one sec */
   add_timer(&this->driver.no_irq_timer);
#endif

   /* setup hdcp check timer */
   init_timer(&this->driver.hdcp_check);
   this->driver.hdcp_check.function=hdcp_check_timeout;
   this->driver.hdcp_check.data=0;

   tmdlHdmiTxGetSWVersion(&this->tda.version);
   printk(KERN_INFO "HDMI TX SW Version:%lu.%lu compatibility:%lu\n",   \
          this->tda.version.majorVersionNr,\
          this->tda.version.minorVersionNr,\
          this->tda.version.compatibilityNr);

#ifdef CONFIG_SUSPEND
    INIT_DELAYED_WORK(&resume_work, tda998x_resume);
#endif

   LOG(KERN_INFO,"init end......\n");
   return 0;

 i2c_tx_out:
   LOG(KERN_INFO,"tmdlHdmiTx closed\n");
   /* close DevLib */
   err=tmdlHdmiTxClose(this->tda.instance);

 i2c_out:
   LOG(KERN_INFO,"this->driver.i2c_client removed\n");
   this->driver.i2c_client = NULL;

   return err;
}

/*
 *  I2C client :: destroy
 */
static int this_i2c_remove(struct i2c_client *client)
{
   int err=0;

   LOG(KERN_INFO,"called\n");

   /* close DevLib */
   err=tmdlHdmiTxClose(this->tda.instance);

   if (!client->adapter) {
      dev_err(&this->driver.i2c_client->dev, "<%s> No HDMI Device \n",
              __func__);
      return -ENODEV;
   }
   switch_dev_unregister(&external_common_state->sdev);
   this->driver.i2c_client = NULL;
   
   wake_lock_destroy(&wlock);

   return err;
}

#ifdef CONFIG_SUSPEND

static void tda998x_resume(struct work_struct *dummy)
{
	LOG(KERN_INFO, "enter\n");

	if(external_common_state->hpd_feature_on) {   	
		tda998x_power_enable(TDA998X_POWER_ALL,1);         

		hdmi_enable();		

		if(this->tda.hot_plug_detect != TMDL_HDMITX_HOTPLUG_ACTIVE){                   
			hdmi_disable(0);
			hdmi_disable(1);
		}else {
			reset_hdmi(0); 	
		}
	}
}

static int tda998x_i2c_suspend(struct device *dev)
{
    int err=0;
    LOG(KERN_INFO,"called\n");     
    
    if(external_common_state->hpd_feature_on) {
    	cancel_delayed_work_sync(&resume_work);
	err= hdmi_disable(0);	
	tda998x_power_enable(TDA998X_POWER_ALL,0);     
    }     
    
    return err;	
}

static int tda998x_i2c_resume(struct device *dev)
{
    int err=0;
    LOG(KERN_INFO,"called\n");
    
    if(external_common_state->hpd_feature_on) {
    	err = schedule_delayed_work(&resume_work,2*HZ);
    }
    
    return err;
}
#else
#define tda998x_i2c_suspend	NULL
#define tda998x_i2c_resume	NULL
#endif

static const struct dev_pm_ops tda998x_device_pm_ops = {
	.suspend = tda998x_i2c_suspend,
	.resume = tda998x_i2c_resume,
};

/*
 *  I2C client driver (backend)
 *  -----------------
 */
static const struct i2c_device_id this_i2c_id[] = {
   { TX_NAME, 0 },
   { },
};

MODULE_DEVICE_TABLE(i2c, this_i2c_id);

static struct i2c_driver this_i2c_driver = {
   .driver = {
      .owner = THIS_MODULE,
      .name = TX_NAME,
      .pm     = &tda998x_device_pm_ops,
   },
   .probe = this_i2c_probe,
   .remove = this_i2c_remove,
   .id_table = this_i2c_id,
};

/*
 *  ioctl driver (userland frontend)
 *  ------------
 */
static struct file_operations this_cdev_fops = {
 owner:    THIS_MODULE,
 open:     this_cdev_open,
 release:  this_cdev_release,
 unlocked_ioctl:    this_cdev_ioctl,
};

/*
 *  sysfs_attrs
 *  -----------
 */

static ssize_t reso_show(struct device *dev,struct device_attribute *attr, char *buf)
{
   int cnt = 0;

   cnt = sprintf(buf,"format in video %d ( %s )\n", \
                  this->tda.setio.video_in.format,                  \
                  tda_spy_vfmt(this->tda.setio.video_in.format));
   cnt += sprintf(buf + cnt,"format out video %d ( %s )\n", \
                  this->tda.setio.video_out.format,                  \
                  tda_spy_vfmt(this->tda.setio.video_out.format));
   return cnt;
}

static ssize_t reso_store(struct device *dev,
                          struct device_attribute *attr, const char *buf, size_t size)
{
   int resolution=0;

   sscanf(buf,"%d",&resolution);
   if (resolution != WITH_FP(this->tda.setio.video_in.format)) {
      LOG(KERN_INFO,"sys_attr new video format\n from %d:( %s )\n to %d:( %s )\n", \
          this->tda.setio.video_in.format,                              \
          tda_spy_vfmt(this->tda.setio.video_in.format),                \
          resolution,                                                   \
          tda_spy_vfmt(resolution));
      this->tda.setio.video_out.format = NO_FP(resolution);
      this->tda.setio.video_in.format = this->tda.setio.video_out.format;
      this->tda.setio.video_in.structure3D = (IS_FP(resolution)?TMDL_HDMITX_3D_FRAME_PACKING: TMDL_HDMITX_3D_NONE);
      
      if (resolution == 0) {
         this->tda.power = tmPowerStandby;
         tmdlHdmiTxSetPowerState(this->tda.instance,this->tda.power);
      } else {
         this->tda.power = tmPowerOn;
         tmdlHdmiTxSetPowerState(this->tda.instance,this->tda.power);
         show_video(this);
      }
   }
   return 0;
}


static ssize_t audio_show(struct device *dev,struct device_attribute *attr, char *buf)
{
   printk("Audio Show\n");

   tda_spy_audio(&this->tda.setio.audio_in);
   return sprintf(buf,"audio format %d - %d - %d - %d - %d - %d\n", \
                  this->tda.setio.audio_in.format,                  \
                  this->tda.setio.audio_in.rate,                    \
                  this->tda.setio.audio_in.i2sFormat,               \
                  this->tda.setio.audio_in.i2sQualifier,            \
                  this->tda.setio.audio_in.dstRate,                 \
                  this->tda.setio.audio_in.channelAllocation);
}

static ssize_t audio_store(struct device *dev,
                           struct device_attribute *attr, const char *buf, size_t size)
{
   char desc_format[]="%d - %d - %d - %d - %d - %d\n";
   tda_audio_in audio;
   
   /*
     Example:

     adb shell "echo '1 - 1 - 0 - 32 - 0 -' >/sys/hdmitx/audio"

     with :
     
     TMDL_HDMITX_AFMT_I2S, 
     TMDL_HDMITX_AFS_44K, 
     TMDL_HDMITX_I2SFOR_PHILIPS_L, 
     TMDL_HDMITX_I2SQ_32BITS,
     TMDL_HDMITX_DSTRATE_SINGLE, 
     channel:0
   */

   memcpy(&audio,&this->tda.setio.audio_in,sizeof(audio));
   sscanf(buf,desc_format,           \
          &audio.format,             \
          &audio.rate,               \
          &audio.i2sFormat,          \
          &audio.i2sQualifier,       \
          &audio.dstRate,            \
          &audio.channelAllocation);

   if (memcmp(&this->tda.setio.audio_in,&audio,sizeof(audio))) {
      tda_spy_audio(&this->tda.setio.audio_in);
      memcpy(&this->tda.setio.audio_in,&audio,sizeof(audio));
      tmdlHdmiTxSetAudioInput(this->tda.instance,           \
                              this->tda.setio.audio_in,     \
                              this->tda.setio.sink);
   }
   return 0;
}

#ifdef I2C_DBG
static ssize_t i2cR_store(struct device *dev,
                          struct device_attribute *attr, const char *buf, size_t size)
{
   /* 
      adb shell "echo '2 1' >/sys/hdmitx/i2cR"
      ... read page 0x02 address 0x01
   */
   tmHdmiTxobject_t *p;
   tmErrorCode_t err;
   unsigned int address=0;
   unsigned int value=0,page=0;
   char desc_format[]="%x %x\n";
   
   err = checkUnitSetDis(this->tda.instance, &p);
   sscanf(buf,desc_format,&page,&address);
   err = getHwRegister(p, SPA(E_SNONE,page,address), (unsigned char *)&value);
   printk("i2c read %x @ page:%x address:%x\n",value,page,address);
   return 0;
}

static ssize_t i2cW_store(struct device *dev,
                          struct device_attribute *attr, const char *buf, size_t size)
{
   /*
     adb shell "echo '2 1 0x03 2' >/sys/hdmitx/i2cW"
     ... write 0x02 page 0x02 address 0x01 using mask 0x03
   */

   tmHdmiTxobject_t *p;
   tmErrorCode_t err;
   unsigned int page=0,address=0,mask=0,value=0;
   char desc_format[]="%x %x %x %x\n";
   
   err = checkUnitSetDis(this->tda.instance, &p);
   sscanf(buf,desc_format,&page,&address,&mask,&value);
   err = setHwRegisterField(p,SPA(E_SNONE,page,address),mask,value); 
   printk("i2c write %x @ page:%x address:%x mask:%x\n",value,page,address,mask);
   return 0;
}
#endif

static DEVICE_ATTR(resolution, S_IRUGO|S_IWUSR, reso_show, reso_store);
static DEVICE_ATTR(audio, S_IRUGO|S_IWUSR, audio_show, audio_store);
#ifdef I2C_DBG
static DEVICE_ATTR(i2cW, S_IRUGO|S_IWUSR, NULL, i2cW_store);
static DEVICE_ATTR(i2cR, S_IRUGO|S_IWUSR, NULL, i2cR_store);
#endif
static struct device_attribute *display_sysfs_attrs[] = {
   &dev_attr_resolution,
   &dev_attr_audio,
#ifdef I2C_DBG
   &dev_attr_i2cW,
   &dev_attr_i2cR,
#endif
   NULL
};

static int comm_init(void)
{
   int retval=0;
   int i=0;	
   struct device_attribute *attr;

   while ((attr = display_sysfs_attrs[i++]) != NULL) {
      retval=device_create_file (this->driver.dev,attr);
      if (retval != 0) {
         goto out_create_file;
      }
   }
   /* create display sysfs links */
   retval = sysfs_create_link(NULL,&(this->driver.dev->kobj),HDMITX_NAME);
   if (retval != 0)
      goto out_create_link;
   return retval;

 out_create_link:
   sysfs_remove_link(NULL, HDMITX_NAME);
 out_create_file:
   while ((attr = display_sysfs_attrs[i++]) != NULL) {
      device_remove_file (this->driver.dev,attr);
   }
   return retval;
}

static void comm_exit(void)
{
   int i=0;	
   struct device_attribute *attr;
   while ((attr = display_sysfs_attrs[i++]) != NULL) {
      device_remove_file (this->driver.dev,attr);
   }
   sysfs_remove_link(NULL, HDMITX_NAME);
}

/*
 *  Module :: start up
 */
static int __init tx_init(void)
{   
   dev_t dev=0;
   int err=0;
   this=&our_instance;

   // Peter XIAO Fix HDMI freeze start
   tv_enc_clk = clk_get(NULL, "tv_enc_clk");
   if (IS_ERR(tv_enc_clk)) {
       printk(KERN_ERR "error: can't get tv_enc_clk!\n");
       return IS_ERR(tv_enc_clk);
   }
   // Peter XIAO Fix HDMI freeze end
   /* 
      general device context
   */
   memset(this,0,sizeof(tda_instance)); 
   this->param.verbose = param_verbose;
   this->param.major = param_major;
   this->param.minor = param_minor;

   /* Hello word */
   printk(KERN_INFO "%s(%s) %d.%d.%d compiled: %s %s %s\n", HDMITX_NAME, TDA_NAME, 
          TDA_VERSION_MAJOR,
          TDA_VERSION_MINOR,
          TDA_VERSION_PATCHLEVEL, 
          __DATE__, __TIME__, TDA_VERSION_EXTRA);
   if (this->param.verbose) LOG(KERN_INFO,".verbose mode\n");

   /*
     plug I2C (backend : Hw interfacing)
   */
   err = i2c_add_driver(&this_i2c_driver);
   if (err < 0) {
      printk(KERN_ERR "Driver registration failed\n");
      return -ENODEV;
   }

   if (this->driver.i2c_client == NULL) {
      printk(KERN_ERR "this->driver.i2c_client not allocated\n");
      /* unregister i2c */
      err = -ENODEV;
      goto init_out;
   }

   /*
     cdev init (userland frontend)
   */

   /* arbitray range of device numbers */
   if (this->param.major) {
      /* user force major number @ insmod */
      dev = MKDEV(this->param.major, this->param.minor);
      err = register_chrdev_region(dev,MAX_MINOR,HDMITX_NAME);
      if (err) {
         printk(KERN_ERR "unable to register %s, dev=%d %s\n",HDMITX_NAME,dev,ERR_TO_STR(err));
         goto init_out;
      }
   } else {
      /* fully dynamic major number */
      err = alloc_chrdev_region(&dev, this->param.minor, MAX_MINOR,HDMITX_NAME);
      if (err) {
         printk(KERN_ERR "unable to alloc chrdev region for %s, dev=%d %s\n",HDMITX_NAME,dev,ERR_TO_STR(err));
         goto init_out;
      }
      this->param.major = MAJOR(dev);
      this->param.minor = MINOR(dev);
      /*       create_dev("/dev/hdmitx",dev); */
      LOG(KERN_INFO,"/dev/hdmitx created major:%d minor:%d\n",this->param.major, this->param.minor);
   }

   cdev_init(this_cdev, &this_cdev_fops);
   this_cdev->owner = THIS_MODULE;

   this->driver.class = class_create(THIS_MODULE, HDMITX_NAME);
   if (IS_ERR(this->driver.class)) {
      printk(KERN_INFO "Error creating mmap device class.\n");
      err =-EIO;
      goto init_out;
   }
   this->driver.dev=device_create(this->driver.class, NULL /* parent */, dev, NULL, HDMITX_NAME);

   this->driver.devno = dev;
   err = cdev_add(this_cdev, this->driver.devno, MAX_MINOR);
   if (err){
      printk(KERN_INFO "unable to add device for %s, ipp_driver.devno=%d %s\n",HDMITX_NAME,this->driver.devno,ERR_TO_STR(err));
      device_destroy(this->driver.class,this->driver.devno);
      class_destroy(this->driver.class);
      unregister_chrdev_region(this->driver.devno, MAX_MINOR);
      goto init_out;
   }   

   /* 
      general device context
   */
//   init_MUTEX(&this->driver.sem);
	sema_init(&this->driver.sem, 1);
   
   /*
     /!\ WARNING /!                                                     \
     the startup power sequence SHALL BE standby AND THEN suspend (core driver legacy...)
     this is the only way to get the TDA idle but with active HDP and RxSens interrupt listening
   */
   //hdmi_disable(1); /* power start sequence phase 2 */
   hdmi_disable(0);	
   tda998x_power_enable(TDA998X_POWER_ALL,0); 

   /*
     /!\ WARNING /!                                                     \
     if HDMI is plugged, the core driver will send HDP nor RXSENS event when beeing powered on !
     So the Android HDMI service shall start by asking the HDP status using the IOCTL GET_HPD_STATUS 
   */
   //tmdlHdmiTxGetHPDStatus(this->tda.instance,
                         // &this->tda.hot_plug_detect); /* power start sequence phase 3 */
   
   /* sysfs_attrs */
   comm_init();

   wake_lock_init(&wlock, WAKE_LOCK_SUSPEND, "hdmi_active");

   LOG(KERN_INFO,"init end..\n");

   return 0;

 init_out:
   // Peter XIAO Fix HDMI freeze start
   if (tv_enc_clk)
       clk_put(tv_enc_clk);
   // Peter XIAO Fix HDMI freeze end

   i2c_del_driver(&this_i2c_driver);
   return err;
}

/*
 *  Module :: shut down
 */
static void __exit tx_exit(void)
{
   LOG(KERN_INFO,"called\n");

#ifdef IRQ
   free_irq(gpio_to_irq(this->driver.gpio), this);
   gpio_free(this->driver.gpio);
#else
   del_timer(&this->driver.no_irq_timer);
#endif

   del_timer(&this->driver.hdcp_check);

   /* sysfs_attrs */
   comm_exit();

   /* unregister cdevice */
   cdev_del(this_cdev); 
   unregister_chrdev_region(this->driver.devno, MAX_MINOR);
   
   /* unregister device */
   device_destroy(this->driver.class,this->driver.devno);
   class_destroy(this->driver.class);

   /* unregister i2c */
   i2c_del_driver(&this_i2c_driver);
}


/*
 *  Module
 *  ------
 */
/* late_initcall(tx_init); */
module_init(tx_init);
module_exit(tx_exit);

/*
 *  Disclamer
 *  ---------
 */
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andre Lepine <andre.lepine@nxp.com>");
MODULE_DESCRIPTION(HDMITX_NAME " driver");

