#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/iio/consumer.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/of.h>

#define LIGHTSENSOR_IOCTL_MAGIC 'l'
#define LIGHTSENSOR_IOCTL_GET_ENABLED   _IOR(LIGHTSENSOR_IOCTL_MAGIC, 1, int *)
#define LIGHTSENSOR_IOCTL_ENABLE        _IOW(LIGHTSENSOR_IOCTL_MAGIC, 2, int *)
#define LIGHTSENSOR_IOCTL_SET_RATE      _IOW(LIGHTSENSOR_IOCTL_MAGIC, 3, int)
#define LIGHTSENSOR_IOCTL_GET_ADC       _IOR(LIGHTSENSOR_IOCTL_MAGIC, 4, int *)
#define DEFAULT_POLL_INTERVAL           200       // ms

struct adc_light_data {
    struct iio_channel *adc_chan;
    struct input_dev *input_dev;
    struct miscdevice miscdev;
    struct delayed_work work;

    atomic_t enabled;
    int poll_interval;
};

static int adc_light_read_adc(struct adc_light_data *data)
{
    int val, ret;
    if (!data->adc_chan)
        return 0;
    ret = iio_read_channel_raw(data->adc_chan, &val);
    if (ret < 0)
        return 0;
    return val;
}

static int adc_to_index(int val)
{
    if (val <= 10)
        return 0;
    else if (val <= 160)
        return 1;
    else if (val <= 225)
        return 2;
    else if (val <= 320)
        return 3;
    else if (val <= 640)
        return 4;
    else if (val <= 1280)
        return 5;
    else if (val <= 2600)
        return 6;
    else
        return 7;
}

static void adc_light_work_func(struct work_struct *work)
{
    struct adc_light_data *data = container_of(to_delayed_work(work), struct adc_light_data, work);
    int val, index;

    if (!atomic_read(&data->enabled))
        return;

    val = adc_light_read_adc(data);
    index = adc_to_index(val);

    input_report_abs(data->input_dev, ABS_MISC, index);
    input_sync(data->input_dev);

    schedule_delayed_work(&data->work, msecs_to_jiffies(data->poll_interval));
}


static int adc_light_sensor_enable(struct adc_light_data *data, int enable)
{
    if (enable) {
        if (!atomic_read(&data->enabled)) {
            atomic_set(&data->enabled, 1);
            schedule_delayed_work(&data->work, msecs_to_jiffies(data->poll_interval));
        }
    } else {
        if (atomic_read(&data->enabled)) {
            atomic_set(&data->enabled, 0);
            cancel_delayed_work_sync(&data->work);
        }
    }

    return 0;
}

static int adc_light_sensor_set_delay(struct adc_light_data *data, int delay_ms)
{
    if (delay_ms < 10)
        delay_ms = 10;
    if (delay_ms > 1000)
        delay_ms = 1000;

    data->poll_interval = delay_ms;
    if (atomic_read(&data->enabled)) {
        cancel_delayed_work_sync(&data->work);
        schedule_delayed_work(&data->work, msecs_to_jiffies(data->poll_interval));
    }
    return 0;
}

static long adc_light_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct miscdevice *miscdev = file->private_data;
    struct adc_light_data *data = dev_get_drvdata(miscdev->parent);
    void __user *argp = (void __user *)arg;
    int enable, delay_ms;
    int adc_val;

    switch (cmd) {
    case LIGHTSENSOR_IOCTL_GET_ENABLED:
        enable = atomic_read(&data->enabled);
        if (copy_to_user(argp, &enable, sizeof(enable)))
            return -EFAULT;
        break;
    case LIGHTSENSOR_IOCTL_ENABLE:
        if (copy_from_user(&enable, argp, sizeof(enable)))
            return -EFAULT;
        return adc_light_sensor_enable(data, enable);
    case LIGHTSENSOR_IOCTL_SET_RATE:
        if (copy_from_user(&delay_ms, argp, sizeof(delay_ms)))
            return -EFAULT;
        return adc_light_sensor_set_delay(data, delay_ms);
    case LIGHTSENSOR_IOCTL_GET_ADC:
        adc_val = adc_light_read_adc(data);
        if (copy_to_user(argp, &adc_val, sizeof(adc_val)))
            return -EFAULT;
        break;
    default:
        return -EINVAL;
    }

    return 0;
}

static int adc_light_open(struct inode *inode, struct file *file)
{
    return 0;
}

static struct file_operations adc_light_fops = {
    .owner = THIS_MODULE,
    .unlocked_ioctl = adc_light_ioctl,
    .open = adc_light_open,
};

static int adc_light_probe(struct platform_device *pdev)
{
    struct adc_light_data *data;
    struct device *dev = &pdev->dev;
    int ret;

    data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
    if (!data)
        return -ENOMEM;

    atomic_set(&data->enabled, 0);
    data->poll_interval = DEFAULT_POLL_INTERVAL;

    data->adc_chan = devm_iio_channel_get(dev, "light");
    if (IS_ERR(data->adc_chan)) {
        dev_err(dev, "failed to get light adc channel\n");
        return PTR_ERR(data->adc_chan);
    }

    data->input_dev = devm_input_allocate_device(&pdev->dev);
    if (!data->input_dev) {
        dev_err(dev, "failed to allocate input device\n");
        return -ENOMEM;
    }

    data->input_dev->name = "lightsensor-level";
    data->input_dev->id.bustype = BUS_HOST;
    set_bit(EV_ABS, data->input_dev->evbit);
    input_set_abs_params(data->input_dev, ABS_MISC, 0, 7, 0, 0);

    ret = input_register_device(data->input_dev);
    if (ret) {
        dev_err(dev, "failed to register input device\n");
        return ret;
    }

    data->miscdev.minor = MISC_DYNAMIC_MINOR;
    data->miscdev.name = "lightsensor";
    data->miscdev.parent = dev; 
    data->miscdev.fops = &adc_light_fops;
    ret = misc_register(&data->miscdev);
    if (ret) {
        dev_err(dev, "failed to register misc device\n");
        return ret;
    }

    INIT_DELAYED_WORK(&data->work, adc_light_work_func);

    platform_set_drvdata(pdev, data);

    // schedule_delayed_work(&data->work, msecs_to_jiffies(data->poll_interval));

    dev_info(dev, "ADC light sensor driver initialized\n");

    return 0;
}

static int adc_light_remove(struct platform_device *pdev)
{
    struct adc_light_data *data = platform_get_drvdata(pdev);
    cancel_delayed_work_sync(&data->work);
    misc_deregister(&data->miscdev);
    return 0;
}

static const struct of_device_id adc_light_of_match[] = {
    { .compatible = "embedfire,adc-light-sensor", },
    { }
};
MODULE_DEVICE_TABLE(of, adc_light_of_match);

static struct platform_driver adc_light_driver = {
    .probe = adc_light_probe,
    .remove = adc_light_remove,
    .driver = {
        .name = "adc_light",
        .of_match_table = adc_light_of_match,
    },
};
module_platform_driver(adc_light_driver);

MODULE_AUTHOR("LLM <llm@embedfire.com>");
MODULE_DESCRIPTION("ADC Light Sensor Driver");
MODULE_LICENSE("GPL");
