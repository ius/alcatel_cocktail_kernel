/*
 * Copyright (C) 2011 NXP Semiconductors
 */


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>

#include <linux/pmic8058-nfc.h>
#include <mach/vreg.h>
#include <mach/gpio.h>
#include <mach/irqs.h>
#include <linux/err.h>
#include <linux/mfd/pmic8058.h>
#include <linux/pn544.h>

#define PN544_65N_I2C_BUS			 0x8
#define PN544_65N_I2C_ADDRESS		 0x28
#define PN544_65N_I2C_NAME			 "pn544"
#define MAX_BUFFER_SIZE				 512
#define PMIC_GPIO_16				 15

static struct i2c_client *client;
static struct i2c_adapter *adapter;

static struct pn544_i2c_platform_data pn544_pdata = {
	.irq_gpio = 46,
	.ven_gpio = 16,
	.firm_gpio = 101,
};

static struct pm8058_gpio pn544_ven = {
	.direction      = PM_GPIO_DIR_OUT,//0x01 out 0x02 in
	.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
	.output_value   = 0,
	.pull           = PM_GPIO_PULL_NO,
	.vin_sel        = PM_GPIO_VIN_S3,
	.out_strength   = PM_GPIO_STRENGTH_HIGH,
	.function       = PM_GPIO_FUNC_2,
};

struct pn544_dev {
	wait_queue_head_t	read_wq;
	struct mutex		read_mutex;
	struct i2c_client	*client;
	struct miscdevice	pn544_device;
	unsigned int 		ven_gpio;
	unsigned int 		firm_gpio;
	unsigned int		irq_gpio;
	bool			irq_enabled;
	spinlock_t		irq_enabled_lock;
};

static void pn544_disable_irq(struct pn544_dev *pn544_dev)
{
	unsigned long flags;

	spin_lock_irqsave(&pn544_dev->irq_enabled_lock, flags);
	if (pn544_dev->irq_enabled) {
		disable_irq_nosync(pn544_dev->client->irq);
		pn544_dev->irq_enabled = false;
	}

	spin_unlock_irqrestore(&pn544_dev->irq_enabled_lock, flags);
}

static irqreturn_t pn544_dev_irq_handler(int irq, void *dev_id)
{
	struct pn544_dev *pn544_dev = dev_id;

	pn544_disable_irq(pn544_dev);

	/* Wake up waiting readers */
	wake_up(&pn544_dev->read_wq);

	return IRQ_HANDLED;
}

static ssize_t pn544_dev_read(struct file *filp, char __user *buf,
		size_t count, loff_t *offset)
{
	struct pn544_dev *pn544_dev = filp->private_data;
	char tmp[MAX_BUFFER_SIZE];
	int ret;

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	pr_debug("%s : reading %zu bytes.\n", __func__, count);

	mutex_lock(&pn544_dev->read_mutex);

	if (!gpio_get_value(pn544_dev->irq_gpio)) {
		if (filp->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			goto fail;
		}

		pn544_dev->irq_enabled = true;
		enable_irq(pn544_dev->client->irq);
		ret = wait_event_interruptible(pn544_dev->read_wq,
				gpio_get_value(pn544_dev->irq_gpio));

		pn544_disable_irq(pn544_dev);

		if (ret)
			goto fail;

	}

	/* Read data */
	ret = i2c_master_recv(pn544_dev->client, tmp, count);
	mutex_unlock(&pn544_dev->read_mutex);

	if (ret < 0) {
		pr_err("%s: i2c_master_recv returned %d\n", __func__, ret);
		return ret;
	}
	if (ret > count) {
		pr_err("%s: received too many bytes from i2c (%d)\n",
				__func__, ret);
		return -EIO;
	}
	if (copy_to_user(buf, tmp, ret)) {
		pr_warning("%s : failed to copy to user space\n", __func__);
		return -EFAULT;
	}
	return ret;

fail:
	mutex_unlock(&pn544_dev->read_mutex);
	return ret;
}

static ssize_t pn544_dev_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *offset)
{
	struct pn544_dev  *pn544_dev;
	char tmp[MAX_BUFFER_SIZE];
	int ret;

	pn544_dev = filp->private_data;

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	if (copy_from_user(tmp, buf, count)) {
		pr_err("%s : failed to copy from user space\n", __func__);
		return -EFAULT;
	}

	pr_debug("%s : writing %zu bytes.\n", __func__, count);
	/* Write data */
	ret = i2c_master_send(pn544_dev->client, tmp, count);
	if (ret != count) {
		pr_err("%s : i2c_master_send returned %d\n", __func__, ret);
		ret = -EIO;
	}

	return ret;
}

static int pn544_dev_open(struct inode *inode, struct file *filp)
{

	struct pn544_dev *pn544_dev = container_of(filp->private_data,
			struct pn544_dev,
			pn544_device);

	filp->private_data = pn544_dev;
	pr_debug("%s : %d,%d\n", __func__, imajor(inode), iminor(inode));

	return 0;
}

static int pn544_dev_close(struct inode *inode, struct file *filp)
{
	return 0;
}

static int pn544_dev_ioctl(struct inode *inode, struct file *filp,
		unsigned int cmd, unsigned long arg)
{
	struct pn544_dev *pn544_dev = filp->private_data;

	switch (cmd) {
		case PN544_SET_PWR:
			if (arg == 2) {
				/* power on with firmware download (requires hw reset)
				*/
				pr_info("%s power on with firmware\n", __func__);
				gpio_set_value_cansleep(NR_GPIO_IRQS + PMIC_GPIO_16, 0); 
				gpio_set_value(pn544_dev->firm_gpio, 1);
				msleep(10);
				gpio_set_value_cansleep(NR_GPIO_IRQS + PMIC_GPIO_16, 1);
				msleep(10);
				gpio_set_value_cansleep(NR_GPIO_IRQS + PMIC_GPIO_16, 0);
				msleep(10);
			} else if (arg == 1) {
				/* power on */
				pr_info("%s power on\n", __func__);
				gpio_set_value(pn544_dev->firm_gpio, 0);
				gpio_set_value_cansleep(NR_GPIO_IRQS + PMIC_GPIO_16, 0);
				msleep(10);
			} else  if (arg == 0) {
				/* power off */
				pr_info("%s power off\n", __func__);
				gpio_set_value(pn544_dev->firm_gpio, 0);
				gpio_set_value_cansleep(NR_GPIO_IRQS + PMIC_GPIO_16, 1);
				msleep(10);
			} else {
				pr_err("%s bad arg %lu\n", __func__, arg);
				return -EINVAL;
			}
			break;
		default:
			pr_err("%s bad ioctl %u\n", __func__, cmd);
			return -EINVAL;
	}

	return 0;
}

static const struct file_operations pn544_dev_fops = {
	.owner	= THIS_MODULE,
	.llseek	= no_llseek,
	.read	= pn544_dev_read,
	.write	= pn544_dev_write,
	.open	= pn544_dev_open,
	.release = pn544_dev_close,
	.ioctl  = pn544_dev_ioctl,
};

static int pn544_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int ret;
	struct pn544_i2c_platform_data *platform_data;
	struct pn544_dev *pn544_dev;

	platform_data = client->dev.platform_data;

	if (platform_data == NULL) {
		pr_err("%s : nfc probe fail\n", __func__);
		return  -ENODEV;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s : need I2C_FUNC_I2C\n", __func__);
		return  -ENODEV;
	}
	//IRQ 
	ret = gpio_request(platform_data->irq_gpio, "nfc_int");
	if (ret){
		pr_err("gpio_nfc_int request error\n");
		return  -ENODEV;
	}
	ret = gpio_tlmm_config(GPIO_CFG(platform_data->irq_gpio, 0, 
				GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
			    GPIO_CFG_ENABLE);
	if (ret){
		pr_err("gpio_nfc_int config error\n");
		return  -ENODEV;
	}

	//VEN
	ret = gpio_request(NR_GPIO_IRQS + PMIC_GPIO_16, "nfc_ven");//platform_data->ven_gpio ,182+15
	if (ret){
		pr_err("gpio_nfc_ven request error\n");
		goto err_ven;
	}
	ret = pm8058_gpio_config(PMIC_GPIO_16, &pn544_ven);//platform_data->ven_gpio ,15 ###pn544_ven???
	if (ret) {
		pr_err("%s: FAIL pm8058_gpio_config(): ret=%d.\n",__func__, ret);
		goto err_ven;
	}

	//FIRM
	ret = gpio_request(platform_data->firm_gpio, "nfc_firm");
	if (ret){
		pr_err("gpio_nfc_firm request error\n");	
		goto err_firm;
	}

	ret = gpio_tlmm_config(GPIO_CFG(platform_data->firm_gpio, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	if (ret){
		pr_err("gpio_nfc_firm config error\n");
		goto err_firm;
	}

	pn544_dev = kzalloc(sizeof(*pn544_dev), GFP_KERNEL);
	if (pn544_dev == NULL) {
		dev_err(&client->dev,
				"failed to allocate memory for module data\n");
		ret = -ENOMEM;
		goto err_exit;
	}

	pn544_dev->irq_gpio = platform_data->irq_gpio;
	pn544_dev->ven_gpio  = platform_data->ven_gpio;
	pn544_dev->firm_gpio  = platform_data->firm_gpio;
	pn544_dev->client   = client;

	/* init mutex and queues */
	init_waitqueue_head(&pn544_dev->read_wq);
	mutex_init(&pn544_dev->read_mutex);
	spin_lock_init(&pn544_dev->irq_enabled_lock);

	pn544_dev->pn544_device.minor = MISC_DYNAMIC_MINOR;
	pn544_dev->pn544_device.name = PN544_65N_I2C_NAME;
	pn544_dev->pn544_device.fops = &pn544_dev_fops;

	ret = misc_register(&pn544_dev->pn544_device);
	if (ret) {
		pr_err("%s : misc_register failed\n", __FILE__);
		goto err_misc_register;
	}

	/* request irq.  the irq is set whenever the chip has data available
	 * for reading.  it is cleared when all data has been read.
	 */
	pr_info("%s : requesting IRQ %d\n", __func__, client->irq);
	pn544_dev->irq_enabled = true;
	ret = request_irq(client->irq, pn544_dev_irq_handler,
			IRQF_TRIGGER_HIGH, client->name, pn544_dev);
	if (ret) {
		dev_err(&client->dev, "request_irq failed\n");
		goto err_request_irq_failed;
	}
	pn544_disable_irq(pn544_dev);
	i2c_set_clientdata(client, pn544_dev);

	return 0;

err_request_irq_failed:
	misc_deregister(&pn544_dev->pn544_device);
err_misc_register:
	mutex_destroy(&pn544_dev->read_mutex);
	kfree(pn544_dev);
err_exit:
	gpio_free(platform_data->firm_gpio);
err_firm:
	gpio_free(platform_data->ven_gpio);
err_ven:
	gpio_free(platform_data->irq_gpio);
	return ret;
}

static int pn544_remove(struct i2c_client *client)
{
	struct pn544_dev *pn544_dev;

	pn544_dev = i2c_get_clientdata(client);
	free_irq(client->irq, pn544_dev);
	misc_deregister(&pn544_dev->pn544_device);
	mutex_destroy(&pn544_dev->read_mutex);
	gpio_free(pn544_dev->irq_gpio);
	gpio_free(pn544_dev->ven_gpio);
	gpio_free(pn544_dev->firm_gpio);
	kfree(pn544_dev);

	return 0;
}

static const struct i2c_device_id pn544_id[] = {
	{ PN544_65N_I2C_NAME, 0 },
	{ }
};

static struct i2c_driver pn544_driver = {
	.id_table	= pn544_id,
	.probe		= pn544_probe,
	.remove		= pn544_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= PN544_65N_I2C_NAME,
	},
};

/*
 * module load/unload record keeping
 */

static int __init pn544_dev_init(void)
{
	struct i2c_board_info info;
	int ret;

	ret = i2c_add_driver(&pn544_driver);
	if (ret != 0) {
		pr_info("pn544 can't add i2c driver\n");
		return ret;
	}

	memset(&info, 0, sizeof(struct i2c_board_info));
	info.addr = PN544_65N_I2C_ADDRESS;
	strlcpy(info.type, PN544_65N_I2C_NAME, I2C_NAME_SIZE);
	info.platform_data = &pn544_pdata;
	info.irq = MSM_GPIO_TO_INT(pn544_pdata.irq_gpio);

	adapter = i2c_get_adapter(PN544_65N_I2C_BUS);
	if (!adapter) {
		pr_info("pn544 can't get i2c adapter\n");
		goto err_driver;
	}

	client = i2c_new_device(adapter, &info);
	if (!client){
		pr_info("pn544 can't add i2c device\n");
		goto err_new_device;
	}

	return 0;

err_new_device:
	i2c_put_adapter(adapter);
err_driver:
	i2c_del_driver(&pn544_driver);
	return -ENODEV;
}
module_init(pn544_dev_init);

static void __exit pn544_dev_exit(void)
{
	pr_info("Unloading pn544 driver\n");
	i2c_del_driver(&pn544_driver);
	i2c_put_adapter(adapter);
	i2c_unregister_device(client);
}
module_exit(pn544_dev_exit);

MODULE_AUTHOR("Sylvain Fonteneau");
MODULE_DESCRIPTION("NFC PN544 driver");
MODULE_LICENSE("GPL");
