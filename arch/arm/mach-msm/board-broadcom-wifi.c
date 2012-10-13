/* linux/arch/arm/mach-msm/board-broadcom-wifi.c
*/
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <asm/mach-types.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <linux/skbuff.h>
#include <linux/wlan_plat.h>

#define BRCM_GPIO_WIFI_IRQ  42 //WL_OOB_IRQ , the GPIO number of qualcomm platform, modified

extern int brcm_wifi_power(int on);
extern int brcm_wifi_reset(int on);
extern int brcm_wifi_set_carddetect(int on);

#define PREALLOC_WLAN_NUMBER_OF_SECTIONS	4
#define PREALLOC_WLAN_NUMBER_OF_BUFFERS		160
#define PREALLOC_WLAN_SECTION_HEADER		24

#define WLAN_SECTION_SIZE_0	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 128)
#define WLAN_SECTION_SIZE_1	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 128)
#define WLAN_SECTION_SIZE_2	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 512)
#define WLAN_SECTION_SIZE_3	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 1024)

#define WLAN_SKB_BUF_NUM	17

static struct sk_buff *wlan_static_skb[WLAN_SKB_BUF_NUM];

typedef struct wifi_mem_prealloc_struct {
	void *mem_ptr;
	unsigned long size;
} wifi_mem_prealloc_t;

static wifi_mem_prealloc_t wifi_mem_array[PREALLOC_WLAN_NUMBER_OF_SECTIONS] = {
	{ NULL, (WLAN_SECTION_SIZE_0 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_1 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_2 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_3 + PREALLOC_WLAN_SECTION_HEADER) }
};

static void *brcm_wifi_mem_prealloc(int section, unsigned long size)
{
	if (section == PREALLOC_WLAN_NUMBER_OF_SECTIONS)
		return wlan_static_skb;
	if ((section < 0) || (section > PREALLOC_WLAN_NUMBER_OF_SECTIONS))
		return NULL;
	if (wifi_mem_array[section].size < size)
		return NULL;
	return wifi_mem_array[section].mem_ptr;
}

#define DHD_SKB_HDRSIZE			336
#define DHD_SKB_1PAGE_BUFSIZE	((PAGE_SIZE*1)-DHD_SKB_HDRSIZE)
#define DHD_SKB_2PAGE_BUFSIZE	((PAGE_SIZE*2)-DHD_SKB_HDRSIZE)
#define DHD_SKB_4PAGE_BUFSIZE	((PAGE_SIZE*4)-DHD_SKB_HDRSIZE)


int __init brcm_init_wifi_mem(void)
{
	int i;
	int j;

	for (i = 0; i < 8; i++) {
		wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_1PAGE_BUFSIZE);
		if (!wlan_static_skb[i])
			goto err_skb_alloc;
	}

	for (; i < 16; i++) {
		wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_2PAGE_BUFSIZE);
		if (!wlan_static_skb[i])
			goto err_skb_alloc;
	}

	wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_4PAGE_BUFSIZE);
	if (!wlan_static_skb[i])
		goto err_skb_alloc;
	
	for(i=0;( i < PREALLOC_WLAN_NUMBER_OF_SECTIONS );i++) {
		wifi_mem_array[i].mem_ptr = kmalloc(wifi_mem_array[i].size,
							GFP_KERNEL);
		if (wifi_mem_array[i].mem_ptr == NULL)
			goto err_mem_alloc;
	}

	printk("%s: WIFI MEM Allocated\n", __FUNCTION__);
	return 0;
	
err_mem_alloc:
	pr_err("Failed to mem_alloc for WLAN\n");
	for (j = 0 ; j < i ; j++)
		kfree(wifi_mem_array[j].mem_ptr);

	i = WLAN_SKB_BUF_NUM;

 err_skb_alloc:
	pr_err("Failed to skb_alloc for WLAN\n");
	for (j = 0 ; j < i ; j++)
		dev_kfree_skb(wlan_static_skb[j]);

	return -ENOMEM;
}

static struct resource brcm_wifi_resources[] = {
	[0] = {
		.name		= "bcm4329_wlan_irq",
		.start		= MSM_GPIO_TO_INT(BRCM_GPIO_WIFI_IRQ),
		.end		= MSM_GPIO_TO_INT(BRCM_GPIO_WIFI_IRQ),
		.flags          = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL,
	},
};

static struct wifi_platform_data brcm_wifi_control = {
	.set_power      = brcm_wifi_power,
	.set_reset      = brcm_wifi_reset,
	.set_carddetect = brcm_wifi_set_carddetect,
	.mem_prealloc   = brcm_wifi_mem_prealloc,
};

static struct platform_device brcm_wifi_device = {
        .name           = "bcm4329_wlan",
        .id             = 1,
        .num_resources  = ARRAY_SIZE(brcm_wifi_resources),
        .resource       = brcm_wifi_resources,
        .dev            = {
                .platform_data = &brcm_wifi_control,
        },
};

int __init broadcom_wifi_init(void)
{
	int ret;

	printk(KERN_INFO "%s: start\n", __func__);
	//brcm_init_wifi_mem();
	ret = platform_device_register(&brcm_wifi_device);
	return ret;
}
late_initcall(broadcom_wifi_init);
