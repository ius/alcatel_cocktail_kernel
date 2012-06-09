/*this is a part of msm_sdcc ,ljm added for sdcc hang debug, in order to make
the source tree easy to merge,seprate the code from msm_sdcc.c
as msm_sdcc is not very stable. we leave this
*/
static void enable_sdcc_clock(struct mmc_host *mmc)
{
}

static void enable_host_irq(struct mmc_host *mmc)
{
	struct msmsdcc_host *host;
	host = mmc_priv(mmc);
	printk("mci_irqenable %#x\n",host->mci_irqenable);
	writel(host->mci_irqenable, host->base + MMCIMASK0);
}
static void enable_host_clock(struct mmc_host *mmc )
{
	unsigned long flags;
	struct msmsdcc_host *host;
	host = mmc_priv(mmc);
	spin_lock_irqsave(&host->lock, flags);
	clk_enable(host->pclk);
	clk_enable(host->clk);
	spin_unlock_irqrestore(&host->lock, flags);
}
static void power_on_host(struct mmc_host *mmc)
{
//	unsigned long flags;
	u32 status;
	struct msmsdcc_host *host = mmc_priv(mmc);
//	spin_lock_irqsave(&host->lock, flags);
	status=0;
	printk(" MMCIPOWER :");
	status = readl(host->base + MMCIPOWER);
	printk(" %#x\n",status);
	status |= MCI_PWR_ON;
	writel(status, host->base + MMCIPOWER);
//	spin_unlock_irqrestore(&host->lock, flags);

}

static void power_up_host(struct mmc_host *mmc)
{
//	unsigned long flags;
	u32 status;
	struct msmsdcc_host *host = mmc_priv(mmc);
//	spin_lock_irqsave(&host->lock, flags);
	status=0;
	printk(" MMCIPOWER :");
	status = readl(host->base + MMCIPOWER);
	printk(" %#x\n",status);
	status |= MCI_PWR_UP;
	writel(status, host->base + MMCIPOWER);
//	spin_unlock_irqrestore(&host->lock, flags);

}

static void reset_host_ios(struct mmc_host *mmc)
{
//	unsigned long flags;
//	struct msmsdcc_host *host = mmc_priv(mmc);
//	spin_lock_irqsave(&host->lock, flags);
	printk(" reset ios :\n");
	msmsdcc_set_ios(mmc, &mmc->ios);
//	spin_unlock_irqrestore(&host->lock, flags);
}

static void host_detect(struct mmc_host *mmc)
{
//	unsigned long flags;
//	struct msmsdcc_host *host = mmc_priv(mmc);
//	spin_lock_irqsave(&host->lock, flags);
	printk(" redetect :");
	mmc_detect_change(mmc, 0);
//	spin_unlock_irqrestore(&host->lock, flags);
}

static void resume_host_debug(struct mmc_host *mmc)
{
//	unsigned long flags;
//	struct msmsdcc_host *host = mmc_priv(mmc);
//	spin_lock_irqsave(&host->lock, flags);
	printk(" resume :");
	mmc_resume_host(mmc);
//	spin_unlock_irqrestore(&host->lock, flags);
}


static void disable_powersave(struct mmc_host *mmc)
{
//	unsigned long flags;
//	spin_lock_irqsave(&host->lock, flags);
	u32 status;
	struct msmsdcc_host *host ;
	host = mmc_priv(mmc);
	status=0;
	printk(" %s :",__FUNCTION__);
	msmsdcc_pwrsave=0;
	status = readl(host->base + MMCICLOCK);
	status = status & (~MCI_CLK_PWRSAVE);	
	writel(status, host->base + MMCICLOCK);
//	spin_unlock_irqrestore(&host->lock, flags);
}


static void vdd_on_bus(struct mmc_host *mmc)
{
	unsigned long flags;
	u32 status;
	struct msmsdcc_host *host ;
	host = mmc_priv(mmc);
	status=0;
	spin_lock_irqsave(&host->lock, flags);
	spin_unlock_irqrestore(&host->lock, flags);

}


static void host_enable_ios_vdd(struct mmc_host *mmc)
{
//	unsigned long flags;
	u32 status;
	struct msmsdcc_host *host ;
	host = mmc_priv(mmc);
	status=0;
/*18 means nothing, just means not zero.*/
	mmc->ios.vdd=18;
	if (host->plat->translate_vdd)
		host->plat->translate_vdd(mmc_dev(mmc), mmc->ios.vdd);
/*ios set too may arguments*/
//	msmsdcc_set_ios(mmc, &mmc->ios);
//	spin_lock_irqsave(&host->lock, flags);
//	spin_unlock_irqrestore(&host->lock, flags);

}
static void host_disable_ios_vdd(struct mmc_host *mmc)
{
//	unsigned long flags;
	u32 status;
	struct msmsdcc_host *host ;
	host = mmc_priv(mmc);
	status=0;
	mmc->ios.vdd=0;
	if (host->plat->translate_vdd)
		host->plat->translate_vdd(mmc_dev(mmc), mmc->ios.vdd);
/*ios set too may arguments*/
//	msmsdcc_set_ios(mmc, &mmc->ios);
//	spin_lock_irqsave(&host->lock, flags);
//	spin_unlock_irqrestore(&host->lock, flags);

}





static void template(struct mmc_host *mmc)
{
//	unsigned long flags;
//	u32 status;
	struct msmsdcc_host *host ;
	host = mmc_priv(mmc);
//	status=0;
//	spin_lock_irqsave(&host->lock, flags);
//	spin_unlock_irqrestore(&host->lock, flags);

}

static void get_clock(struct mmc_host *mmc)
{
//	unsigned long flags;
//	u32 status;
	struct msmsdcc_host *host ;
	host = mmc_priv(mmc);
//	status=0;
	printk(" host clock on %d \n",host->clks_on);
	printk(" host pclk %ld\n",clk_get_rate(host->pclk));
	printk(" host clk %ld\n",clk_get_rate(host->clk));
//	spin_lock_irqsave(&host->lock, flags);
//	spin_unlock_irqrestore(&host->lock, flags);

}


static void test_host_lock(struct mmc_host *mmc)
{
	unsigned long flags;
	struct msmsdcc_host *host = mmc_priv(mmc);
	printk("before lock\n");
	if(spin_trylock_irqsave(&host->lock, flags))
	{
		printk("locked\n");
		spin_unlock_irqrestore(&host->lock, flags);
		printk("unlocked\n");
	}
	else
	{
		printk("try lock faied\n");
	}

}

static void getstatus(struct mmc_host *mmc)
{
	/*show clk,power*/
	struct msmsdcc_host *host;
	u32 status;
	host = mmc_priv(mmc);
	status=0;
	printk(" MMCISTATUS:");
	status = readl(host->base + MMCISTATUS);
	printk(" %#x\n",status);
	printk(" MMCICLOCK :");
	status = readl(host->base + MMCICLOCK);
	printk(" %#x\n",status);
	printk(" MMCIMASK0 :");
	status = readl(host->base + MMCIMASK0);
	printk(" %#x\n",status);
	printk(" MMCIMASK1 :");
	status = readl(host->base + MMCIMASK1);
	printk(" %#x\n",status);
	printk(" MMCIRESPONSE0 :");
	status = readl(host->base + MMCIRESPONSE0);
	printk(" %#x\n",status);
	printk(" MMCIRESPONSE1 :");
	status = readl(host->base + MMCIRESPONSE1);
	printk(" %#x\n",status);
	printk(" MMCIRESPONSE2 :");
	status = readl(host->base + MMCIRESPONSE2);
	printk(" %#x\n",status);
	printk(" MMCIRESPONSE3 :");
	status = readl(host->base + MMCIRESPONSE3);
	printk(" %#x\n",status);

	printk(" MMCIPOWER :");
	status = readl(host->base + MMCIPOWER);
	printk(" %#x\n",status);
	
	printk(" MMCIDATACTRL :");
	status = readl(host->base + MMCIDATACTRL);
	printk(" %#x\n",status);

	printk(" MMCICOMMAND :");
	status = readl(host->base + MMCICOMMAND);
	printk(" %#x\n",status);

	printk(" MMCIARGUMENT :");
	status = readl(host->base + MMCIARGUMENT);
	printk(" %#x\n",status);

	printk(" MMCIDATATIMER :");
	status = readl(host->base + MMCIDATATIMER);
	printk(" %#x\n",status);

	printk(" MMCIDATALENGTH :");
	status = readl(host->base + MMCIDATALENGTH);
	printk(" %#x\n",status);

	printk(" MMCIRESPCMD :");
	status = readl(host->base + MMCIRESPCMD);
	printk(" %#x\n",status);


	printk(" MMCIDATACNT :");
	status = readl(host->base + MMCIDATACNT);
	printk(" %#x\n",status);

	printk(" MMCIFIFOCNT :");
	status = readl(host->base + MMCIFIFOCNT);
	printk(" %#x\n",status);

	printk(" MCICCSTIMER :");
	status = readl(host->base + MCICCSTIMER);
	printk(" %#x\n",status);

	printk(" MMCIPOWER :");
	status = readl(host->base + MMCIPOWER);
	printk(" %#x\n",status);

	printk(" MMCIPOWER :");
	status = readl(host->base + MMCIPOWER);
	printk(" %#x\n",status);

/*may block, so another function*/
#if 0
	printk(" host clock on %d \n",host->clks_on);
	prinkt("host pclk %d",clk_get_rate(host->pclk);
	prinkt("host clk %d",clk_get_rate(host->clk);
#endif

	printk(" host->mci_irqenable:");
	printk("%#x\n",host->mci_irqenable);

	printk(" host ios :\n");
	printk(" clk %d, vdd %d,busmode %#x chipmode %#x power_mode %#x bus_width %#x timing %#x\n",mmc->ios.clock,mmc->ios.vdd,mmc->ios.bus_mode,mmc->ios.chip_select,mmc->ios.power_mode,mmc->ios.bus_width,mmc->ios.timing);

	printk(" msmsdcc_pwrsave %d \n",msmsdcc_pwrsave);

	printk(" bus_dead ? %d \n",mmc->bus_dead);
	printk(" bus_ops ? %s \n",mmc->bus_ops==NULL?"NULL":"not null");
	printk(" host->bus_refs %d \n",mmc->bus_refs);
	printk(" host->eject %d \n",host->eject);

	


	

}

static void  testfn(struct mmc_host *mmc )
{
	printk("%s %d %s \n",__FUNCTION__,__LINE__,mmc_hostname(mmc));
}
struct msm_sdcc_sys_debug_struct testsuite[]=
{
	/*value,func,what it does,*/
	/*value: for sysfs set,this is not index, func:  the func that excute, descrip :what does it do*/
	{0,testfn,"testfn"},
	{1,getstatus,"getstatus"},
	{2,test_host_lock,"test_host_lock"},
	{3,power_on_host,"power_on_host"},
	{4,power_up_host,"power_up_host"},
	{5,enable_host_irq,"enable_host_irq"},
	{6,vdd_on_bus,"vdd_on_bus"},
	{7,disable_powersave,"disable_powersave"},
	{8,reset_host_ios,"reset_host_ios"},
	{9,enable_sdcc_clock,"enable_sdcc_clock"},
	{10,enable_host_clock,"enable_host_clock"},
	{11,template,"template"},
	{12,resume_host_debug,"resume_host_debug"},
	{13,host_detect,"host_detect"},
	{14,host_enable_ios_vdd,"host_enable_ios_vdd"},
	{15,host_disable_ios_vdd,"host_disable_ios_vdd"},
	{16,get_clock,"get_clock"},
	{999,NULL,"NULL"},
	{999,NULL,"NULL"},
};
/*printk is easy, TODO ,use sysfs to output information*/
static ssize_t
set_debug(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct mmc_host *mmc = dev_get_drvdata(dev);
//	struct msmsdcc_host *host = mmc_priv(mmc);
	int value;
//	unsigned long flags;
	int methods;
	/*don't lock, test function will lock it itself*/
//	spin_lock_irqsave(&host->lock, flags);

	sscanf(buf, "%d", &value);
	printk("set value %d\n",value);
	methods=sizeof(testsuite)/sizeof(struct msm_sdcc_sys_debug_struct);
#if 0
	/*this is not index*/
	if(value<methods && value >=0 && testsuite[value].issuefn!=NULL)	
		testsuite[value].issuefn(mmc);
#endif
	while(methods>0)
	{
		methods--;
		if(testsuite[methods].value==value && testsuite[methods].issuefn!=NULL)
		{
			printk("exclute %s\n",testsuite[value].description);
			/*anyway, you'd better not block. use thread to run it ?*/
			testsuite[methods].issuefn(mmc);
		}
	}
//	spin_unlock_irqrestore(&host->lock, flags);
	return count;

}

/*only show you how to use debug interface*/
static ssize_t
get_debug(struct device *dev, struct device_attribute *attr, char *buf)
{
	int poll=1;
	int i;
#if 0
	struct mmc_host *mmc = dev_get_drvdata(dev);
	struct msmsdcc_host *host = mmc_priv(mmc);
	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);
	spin_unlock_irqrestore(&host->lock, flags);
#endif
	int methods=sizeof(testsuite)/sizeof(struct msm_sdcc_sys_debug_struct);
	for(i=0;i<methods;i++)
		printk(" %d\t %s\n",testsuite[i].value,testsuite[i].description);
	return snprintf(buf, PAGE_SIZE, "%d\n", poll);
}

static DEVICE_ATTR(debug, 0666,
		get_debug, set_debug);
#if 0
static struct device_attribute debug = __ATTR(debug, 0666,
		get_debug, set_debug);
#endif

static struct attribute *dev_debug_attrs[] = {
	&dev_attr_debug.attr,
	NULL,
};
static struct attribute_group dev_debug_attr_grp = {
	.attrs = dev_debug_attrs,
};


