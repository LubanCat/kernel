#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/init.h>       
#include <asm/io.h>          
#include <asm/uaccess.h>     
#include <linux/device.h>    
#include <linux/cdev.h>
#include <linux/platform_device.h> 
#include <linux/of.h>        
#include <linux/kobject.h>   
#include <linux/sysfs.h>     
#include <linux/gpio/consumer.h> 
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/gpio.h>   
#include <linux/delay.h>

#define DS18B20_CMD_CONVERTT	(0x44)
#define DS18B20_CMD_SKIPROM		(0xCC)
#define DS18B20_CMD_READ		(0xBE)

static dev_t ds18b20_dev_num;
static struct cdev *ds18b20_dev;    
static struct class *ds18b20_class; 
static struct device *ds18b20;     
static struct gpio_desc *ds18b20_pin;

static void ds18b20_rst(void)
{
	gpiod_direction_output(ds18b20_pin, 1);
	udelay(100);
	gpiod_set_value(ds18b20_pin, 0);
	udelay(460);
	gpiod_set_value(ds18b20_pin, 1);
	udelay(100);
}

static int ds18b20_check(void)
{
	unsigned char retry = 0;
	
	gpiod_direction_input(ds18b20_pin);
	while (gpiod_get_value(ds18b20_pin) && retry < 200)
	{
		retry++;
		udelay(1);
	}
	if (retry >= 200)
	{
		return -1;
	}

	while (!gpiod_get_value(ds18b20_pin) && retry < 240)
	{
		retry++;
		udelay(1);
	}
	if (retry >= 240)
	{
		return -1;
	}

	return 0;
}

static unsigned char ds18b20_read_bit(void)
{
	unsigned char bit;

	gpiod_direction_output(ds18b20_pin, 0);
	gpiod_set_value(ds18b20_pin, 0);
	
	udelay(2);
	
	gpiod_set_value(ds18b20_pin, 1);
	gpiod_direction_input(ds18b20_pin);
	
	udelay(12);
	
	if (gpiod_get_value(ds18b20_pin))
	{
		bit = 1;
	}
	else
	{
		bit = 0;
	}
	
	udelay(50);
	
	return bit;
}

static unsigned char ds18b20_read_byte(void)
{
	unsigned char byte = 0;
	int i, j;
	
	for (i = 1; i <= 8; i++)
	{
		j = ds18b20_read_bit();
		byte = (j << 7) | (byte >> 1);
	}

	return byte;
}

static void ds18b20_write_byte(unsigned char byte)
{
	int i, bit;

	gpiod_direction_output(ds18b20_pin, 1);
	for (i = 1; i <= 8; i++)
	{
		bit = byte & 0x01;
		byte = byte >> 1;
		if (bit)
		{
			gpiod_set_value(ds18b20_pin, 0);
			udelay(2);
			gpiod_set_value(ds18b20_pin, 1);
			udelay(60);
		}
		else
		{
			
gpiod_set_value(ds18b20_pin, 0);
			udelay(60);
			gpiod_set_value(ds18b20_pin, 1);
			udelay(2);
		}
	}
}

static int ds18b20_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int ds18b20_close(struct inode *inode, struct file *filp)
{
	return 0;
}

static ssize_t ds18b20_read(struct file *filp, char __user *buff, size_t count, loff_t *offp)
{
	unsigned char sign;
	unsigned char tl, th;
	short int temp;
	char data_buf[16];
    int len;
	int ret;
	
	ds18b20_rst();
	ds18b20_check();
	msleep(420);
	ds18b20_write_byte(DS18B20_CMD_SKIPROM);
	ds18b20_write_byte(DS18B20_CMD_CONVERTT);
	msleep(750);
	
	ds18b20_rst();
	ds18b20_check();
	msleep(400);
	ds18b20_write_byte(DS18B20_CMD_SKIPROM);
	ds18b20_write_byte(DS18B20_CMD_READ);
	
	tl = ds18b20_read_byte();
	th = ds18b20_read_byte();

	if (th > 7)
	{
		th = ~th;
		tl = ~tl;
		sign = 0;
	}
	else
	{
		sign = 1;
	}

	temp = th;
	temp <<= 8;
	temp += tl;
	temp = temp * 625 / 1000;

	if (sign)
	{
        len = snprintf(data_buf, sizeof(data_buf), "%d.%d\n", temp / 10, temp % 10);
	}
	else

	{
        len = snprintf(data_buf, sizeof(data_buf), "-%d.%d\n", temp / 10, temp % 10);
	}
	
	if (len > count)
    {
		len = count;
	}
	
    ret = copy_to_user(buff, data_buf, len);
    if (ret)
    {
        return -EFAULT;
    }
	
    return len;
}

static struct file_operations ds18b20_ops = {
	.owner = THIS_MODULE,
	.open  = ds18b20_open,
	.read  = ds18b20_read,
	.release = ds18b20_close,
};

static int ds18b20_probe(struct platform_device *pdev)
{
    int ret;

    ds18b20_dev = cdev_alloc();
    if (!ds18b20_dev) {
        dev_err(&pdev->dev, "cdev_alloc failed!\n");
        return -ENOMEM;
    }

    ret = alloc_chrdev_region(&ds18b20_dev_num, 0, 1, "ds18b20");
    if (ret) {
        dev_err(&pdev->dev, "alloc_chrdev_region failed!\n");
        goto err_alloc_region;
    }

    ds18b20_dev->owner = THIS_MODULE;
    ds18b20_dev->ops   = &ds18b20_ops;
    ret = cdev_add(ds18b20_dev, ds18b20_dev_num, 1);
    if (ret) {
        dev_err(&pdev->dev, "cdev_add failed!\n");
        goto err_cdev_add;
    }

    ds18b20_class = class_create(THIS_MODULE, "ds18b20_class");
    if (IS_ERR(ds18b20_class)) {
        ret = PTR_ERR(ds18b20_class);
        dev_err(&pdev->dev, "class_create failed!\n");
        goto err_class_create;
    }

    ds18b20 = device_create(ds18b20_class, NULL, ds18b20_dev_num, NULL, "ds18b20");
    if (IS_ERR(ds18b20)) {
        ret = PTR_ERR(ds18b20);
        dev_err(&pdev->dev, "device_create failed!\n");
        goto err_device_create;
    }

    ds18b20_pin = devm_gpiod_get(&pdev->dev, "data", GPIOD_OUT_LOW);
    if (IS_ERR(ds18b20_pin)) {
        dev_err(&pdev->dev, "Get gpio data failed!\n");
        ret = PTR_ERR(ds18b20_pin);
        goto err_gpio;
    }

    ds18b20_rst();
    ret = ds18b20_check();
    if (ret) {
        dev_err(&pdev->dev, "Can not find DS18B20!\n");
        goto err_gpio;
    }

    dev_info(&pdev->dev, "ds18b20 probe success!\n");
    return 0;

err_gpio:
    device_destroy(ds18b20_class, ds18b20_dev_num);
err_device_create:
    class_destroy(ds18b20_class);
err_class_create:
    cdev_del(ds18b20_dev);
err_cdev_add:
    unregister_chrdev_region(ds18b20_dev_num, 1);
err_alloc_region:
    return ret;
}

static int ds18b20_remove(struct platform_device *pdev)
{
	device_destroy(ds18b20_class, ds18b20_dev_num);
    class_destroy(ds18b20_class);
    cdev_del(ds18b20_dev);
    unregister_chrdev_region(ds18b20_dev_num, 1);

    dev_info(&pdev->dev, "ds18b20 removed\n");
	return 0;
}

static struct of_device_id ds18b20_match_table[] = {  
    {.compatible = "fire,ds18b20",},  
    { },
}; 

static struct platform_device_id ds18b20_device_ids[] = {
    {.name = "ds18b20",},
    { },
};

static struct platform_driver ds18b20_driver = 
{
	.probe  = ds18b20_probe,  
    .remove = ds18b20_remove,  
    .driver={  
        .name = "ds18b20",  
        .of_match_table = ds18b20_match_table,  
    },  
    .id_table = ds18b20_device_ids,
};

static int ds18b20_driver_init(void)
{
	platform_driver_register(&ds18b20_driver);
	return 0;
};

static void ds18b20_driver_exit(void)
{
	platform_driver_unregister(&ds18b20_driver);
};

module_init(ds18b20_driver_init);
module_exit(ds18b20_driver_exit);

MODULE_LICENSE("GPL");          
MODULE_AUTHOR("embedfire");     
MODULE_VERSION("0.1");          
MODULE_DESCRIPTION("ds18b20_driver");
