#include "jadard_platform.h"

extern struct jadard_ts_data *pjadard_ts_data;
extern struct jadard_ic_data *pjadard_ic_data;

#ifdef CONFIG_JD_HID
extern struct jadard_i2c_hid *ihid;
#endif

int jadard_dev_set(struct jadard_ts_data *ts)
{
    ts->input_dev = input_allocate_device();

    if (ts->input_dev == NULL) {
        JD_E("%s: Failed to allocate input device\n", __func__);
        return -ENOMEM;
    }
    ts->input_dev->name = "jadard-touchscreen";

    if (pjadard_ic_data->JD_STYLUS_EN) {
        ts->stylus_dev = input_allocate_device();

        if (ts->stylus_dev == NULL) {
            JD_E("%s: Failed to allocate input stylus_dev\n", __func__);
            input_free_device(ts->input_dev);
            return -ENOMEM;
        }
        ts->stylus_dev->name = "jadard-stylus";
    }

    return 0;
}

int jadard_input_register_device(struct input_dev *input_dev)
{
    return input_register_device(input_dev);
}

#ifdef JD_SPRD_EIC_MODE
static int jadard_pinctrl_init(struct jadard_platform_data *pdata)
{
	int ret = 0;

	pdata->pinctrl = devm_pinctrl_get(pjadard_ts_data->dev);
	if (IS_ERR_OR_NULL(pdata->pinctrl)) {
		JD_E("Failed to get pinctrl, please check dts\n");
		ret = PTR_ERR(pdata->pinctrl);
		return ret;
	}

	pdata->pin_eic = pinctrl_lookup_state(pdata->pinctrl, "eic_16");
	if (IS_ERR_OR_NULL(pdata->pin_eic)) {
		JD_E("Pin state [eic_16] not found\n");
		ret = PTR_ERR(pdata->pin_eic);
		return ret;
	} else {
		ret = pinctrl_select_state(pdata->pinctrl, pdata->pin_eic);
		if (ret) {
			JD_E("Set extint0 pin state error\n");
			return ret;
		}
	}

	pdata->pin_gpio = pinctrl_lookup_state(pdata->pinctrl, "gpio_144");
	if (IS_ERR_OR_NULL(pdata->pin_gpio)) {
		JD_E("Pin state [gpio_144] not found\n");
		ret = PTR_ERR(pdata->pin_gpio);
		return ret;
	}

	JD_I("%s: success\n", __func__);

	return 0;
}
#endif

int jadard_parse_dt(struct jadard_ts_data *ts,
                    struct jadard_platform_data *pdata)
{
    int coords_size = 0;
    uint32_t coords[4] = {0};
    uint32_t ret, data;
    struct property *prop = NULL;
    struct device_node *dt = pjadard_ts_data->client->dev.of_node;

    ret = of_property_read_u32(dt, "jadard,panel-max-points", &data);
    pjadard_ic_data->JD_MAX_PT = (!ret ? data : 10);
    ret = of_property_read_u32(dt, "jadard,int-is-edge", &data);
    pjadard_ic_data->JD_INT_EDGE = (!ret ? (data > 0 ? true : false) : true);
    ret = of_property_read_u32(dt, "jadard,stylus", &data);
    pjadard_ic_data->JD_STYLUS_EN = (!ret ? (data > 0 ? true : false) : false);

    JD_I("DT:MAX_PT = %d, INT_IS_EDGE = %d, STYLUS_EN = %d\n", pjadard_ic_data->JD_MAX_PT,
        pjadard_ic_data->JD_INT_EDGE, pjadard_ic_data->JD_STYLUS_EN);

    prop = of_find_property(dt, "jadard,panel-sense-nums", NULL);
    if (prop) {
        coords_size = prop->length / sizeof(uint32_t);

        if (coords_size != 2) {
            JD_E("%s:Invalid panel sense number size %d\n", __func__, coords_size);
            return -EINVAL;
        }
    }

    if (of_property_read_u32_array(dt, "jadard,panel-sense-nums", coords, coords_size) == 0) {
        pjadard_ic_data->JD_X_NUM = coords[0];
        pjadard_ic_data->JD_Y_NUM = coords[1];
        JD_I("DT:panel-sense-num = %d, %d\n",
            pjadard_ic_data->JD_X_NUM, pjadard_ic_data->JD_Y_NUM);
    }

    prop = of_find_property(dt, "jadard,panel-coords", NULL);
    if (prop) {
        coords_size = prop->length / sizeof(uint32_t);

        if (coords_size != 4) {
            JD_E("%s:Invalid panel coords size %d\n", __func__, coords_size);
            return -EINVAL;
        }
    }

    if (of_property_read_u32_array(dt, "jadard,panel-coords", coords, coords_size) == 0) {
        pdata->abs_x_min = coords[0];
        pdata->abs_x_max = coords[1];
        pdata->abs_y_min = coords[2];
        pdata->abs_y_max = coords[3];
        pjadard_ic_data->JD_X_RES = pdata->abs_x_max;
        pjadard_ic_data->JD_Y_RES = pdata->abs_y_max;

        JD_I("DT:panel-coords = %d, %d, %d, %d\n", pdata->abs_x_min,
            pdata->abs_x_max, pdata->abs_y_min, pdata->abs_y_max);
    }

#ifdef JD_SPRD_EIC_MODE
	if (jadard_pinctrl_init(pdata))
		JD_I("DT: pinctrl init failed\n");
#endif

    pdata->gpio_irq = of_get_named_gpio(dt, "jadard,irq-gpio", 0);
    if (!gpio_is_valid(pdata->gpio_irq)) {
        JD_I("DT:gpio_irq value is not valid\n");
    }

    pdata->gpio_reset = of_get_named_gpio(dt, "jadard,rst-gpio", 0);
    if (!gpio_is_valid(pdata->gpio_reset)) {
        JD_I("DT:gpio_rst value is not valid\n");
    }

    JD_I("DT:gpio_irq = %d, gpio_rst = %d\n", pdata->gpio_irq, pdata->gpio_reset);

    return 0;
}

#ifdef CONFIG_JD_HID
int jadard_bus_read_single(uint8_t *data, uint16_t data_len, uint8_t toRetry)
{
    int retry;
    struct i2c_client *client = pjadard_ts_data->client;
    struct i2c_msg msg[] = {
        {
            .addr = client->addr,
            .flags = client->flags & I2C_M_TEN,
            .flags = client->flags | I2C_M_RD,
            .len = data_len,
            .buf = data,
        }
    };

    mutex_lock(&pjadard_ts_data->rw_lock);

    for (retry = 0; retry < toRetry; retry++) {
        if (i2c_transfer(client->adapter, msg, 1) == 1)
            break;

        msleep(20);
    }

    if (retry == toRetry) {
        JD_E("%s: i2c_read_block retry over %d\n",
          __func__, toRetry);
        mutex_unlock(&pjadard_ts_data->rw_lock);
        return -EIO;
    }

    mutex_unlock(&pjadard_ts_data->rw_lock);
    return 0;
}
#endif

int jadard_bus_read(uint8_t *cmd, uint8_t cmd_len, uint8_t *data, uint32_t data_len, uint8_t toRetry)
{
    int retry;
    struct i2c_client *client = pjadard_ts_data->client;
    struct i2c_msg msg[] = {
        {
            .addr = client->addr,
            .flags = 0,
            .len = cmd_len,
            .buf = cmd,
        },
        {
            .addr = client->addr,
            .flags = I2C_M_RD,
            .len = data_len,
            .buf = data,
        }
    };

    mutex_lock(&pjadard_ts_data->rw_lock);

    for (retry = 0; retry < toRetry; retry++) {
        if (i2c_transfer(client->adapter, msg, 2) == 2)
            break;

        msleep(20);
    }

    if (retry == toRetry) {
        JD_E("%s: i2c_read_block retry over %d\n",
          __func__, toRetry);
        mutex_unlock(&pjadard_ts_data->rw_lock);
        return -EIO;
    }

    mutex_unlock(&pjadard_ts_data->rw_lock);
    return 0;
}

int jadard_bus_write(uint8_t *cmd, uint8_t cmd_len, uint8_t *data, uint32_t data_len, uint8_t toRetry)
{
    int retry;
    uint8_t *buf = NULL;
    struct i2c_client *client = pjadard_ts_data->client;
    struct i2c_msg msg;

    mutex_lock(&pjadard_ts_data->rw_lock);

    buf = kzalloc((cmd_len + data_len) * sizeof(uint8_t), GFP_KERNEL);
    if (buf == NULL) {
        JD_E("%s: Memory alloc fail\n", __func__);
        mutex_unlock(&pjadard_ts_data->rw_lock);
        return -ENOMEM;
    }

    memset(&msg, 0, sizeof(struct i2c_msg));
    /* Set I2C packet info. */
    msg.addr = client->addr;
    msg.flags = 0;
    msg.len = cmd_len + data_len;
    msg.buf = buf;

    memcpy(buf, cmd, cmd_len);
    memcpy(buf + cmd_len, data, data_len);

    for (retry = 0; retry < toRetry; retry++) {
        if (i2c_transfer(client->adapter, &msg, 1) == 1)
            break;

        msleep(20);
    }

    if (retry == toRetry) {
        JD_E("%s: i2c_write_block retry over %d\n",
          __func__, toRetry);
        kfree(buf);
        mutex_unlock(&pjadard_ts_data->rw_lock);
        return -EIO;
    }

    kfree(buf);
    mutex_unlock(&pjadard_ts_data->rw_lock);
    return 0;
}

void jadard_int_enable(bool enable)
{
    int irqnum = pjadard_ts_data->client->irq;
    unsigned long irqflags = 0;

    spin_lock_irqsave(&pjadard_ts_data->irq_active, irqflags);

    if (enable && (pjadard_ts_data->irq_enabled == 0)) {
        enable_irq(irqnum);
        pjadard_ts_data->irq_enabled = 1;
    } else if ((!enable) && (pjadard_ts_data->irq_enabled == 1)) {
        disable_irq_nosync(irqnum);
        pjadard_ts_data->irq_enabled = 0;
    }

    JD_I("irq_enable = %d\n", pjadard_ts_data->irq_enabled);
    spin_unlock_irqrestore(&pjadard_ts_data->irq_active, irqflags);
}

#ifdef JD_RST_PIN_FUNC
void jadard_gpio_set_value(int pin_num, uint8_t value)
{
    gpio_set_value(pin_num, value);
}
#endif

#if defined(CONFIG_JD_DB)
static int jadard_regulator_configure(struct jadard_platform_data *pdata)
{
    int retval;
    struct i2c_client *client = pjadard_ts_data->client;
    pdata->vcc_dig = regulator_get(&client->dev, "vdd");

    if (IS_ERR(pdata->vcc_dig)) {
        JD_E("%s: Failed to get regulator vdd\n",
          __func__);
        retval = PTR_ERR(pdata->vcc_dig);
        return retval;
    }

    pdata->vcc_ana = regulator_get(&client->dev, "avdd");

    if (IS_ERR(pdata->vcc_ana)) {
        JD_E("%s: Failed to get regulator avdd\n",
          __func__);
        retval = PTR_ERR(pdata->vcc_ana);
        regulator_put(pdata->vcc_ana);
        return retval;
    }

    return 0;
};

static int jadard_power_on(struct jadard_platform_data *pdata, bool on)
{
    int retval;

    if (on) {
        retval = regulator_enable(pdata->vcc_dig);

        if (retval) {
            JD_E("%s: Failed to enable regulator vdd\n",
              __func__);
            return retval;
        }

        msleep(100);
        retval = regulator_enable(pdata->vcc_ana);

        if (retval) {
            JD_E("%s: Failed to enable regulator avdd\n",
              __func__);
            regulator_disable(pdata->vcc_dig);
            return retval;
        }
    } else {
        regulator_disable(pdata->vcc_dig);
        regulator_disable(pdata->vcc_ana);
    }

    return 0;
}

int jadard_gpio_power_config(struct jadard_platform_data *pdata)
{
    int error;
    struct i2c_client *client = pjadard_ts_data->client;

    error = jadard_regulator_configure(pdata);
    if (error) {
        JD_E("Failed to intialize hardware\n");
        goto err_regulator_not_on;
    }

#ifdef JD_RST_PIN_FUNC
    if (gpio_is_valid(pdata->gpio_reset)) {
        error = gpio_request(pdata->gpio_reset, "jadard_reset_gpio");

        if (error) {
            JD_E("unable to request rst-gpio [%d]\n", pdata->gpio_reset);
            goto err_regulator_on;
        }

		error = gpio_direction_output(pdata->gpio_reset, 1);
		if (error) {
			JD_E("unable to set direction for rst-gpio [%d]\n", pdata->gpio_reset);
			goto err_gpio_reset_req;
		}
	} else {
		JD_E("rst-gpio [%d] is not valid\n", pdata->gpio_reset);
		goto err_regulator_on;
	}
#endif

    error = jadard_power_on(pdata, true);
    if (error) {
        JD_E("Failed to power on hardware\n");
        goto err_gpio_reset_req;
    }

    if (gpio_is_valid(pdata->gpio_irq)) {
        error = gpio_request(pdata->gpio_irq, "jadard_gpio_irq");

        if (error) {
            JD_E("unable to request irq-gpio [%d]\n", pdata->gpio_irq);
            goto err_power_on;
        }

        error = gpio_direction_input(pdata->gpio_irq);
        if (error) {
            JD_E("unable to set direction for irq-gpio [%d]\n", pdata->gpio_irq);
            goto err_gpio_irq_req;
        }

        client->irq = gpio_to_irq(pdata->gpio_irq);
        pjadard_ts_data->jd_irq = (int)client->irq;
    } else {
        JD_E("irq-gpio [%d] is not valid\n", pdata->gpio_irq);
        goto err_power_on;
    }

    msleep(20);
#ifdef JD_RST_PIN_FUNC
    if (gpio_is_valid(pdata->gpio_reset)) {
        error = gpio_direction_output(pdata->gpio_reset, 1);

        if (error) {
            JD_E("unable to set direction for rst-gpio [%d]\n", pdata->gpio_reset);
            goto err_gpio_irq_req;
        }
        gpio_free(pdata->gpio_reset);
    } else {
        JD_E("rst-gpio [%d] is not valid\n", pdata->gpio_reset);
        goto err_gpio_irq_req;
    }
#endif

    return 0;

err_gpio_irq_req:
    if (gpio_is_valid(pdata->gpio_irq))
        gpio_free(pdata->gpio_irq);

err_power_on:
    jadard_power_on(pdata, false);
err_gpio_reset_req:
#ifdef JD_RST_PIN_FUNC
    if (gpio_is_valid(pdata->gpio_reset))
        gpio_free(pdata->gpio_reset);

err_regulator_on:
#endif
err_regulator_not_on:

    return error;
}

void jadard_gpio_power_deconfig(struct jadard_platform_data *pdata)
{
    /* Only QCOM DB platform using */
}

#else
int jadard_gpio_power_config(struct jadard_platform_data *pdata)
{
    int error = 0;
    struct i2c_client *client = pjadard_ts_data->client;

#ifdef JD_RST_PIN_FUNC
    if (gpio_is_valid(pdata->gpio_reset)) {
        error = gpio_request(pdata->gpio_reset, "jadard_reset_gpio");

        if (error) {
            JD_E("unable to request rst-gpio [%d]\n", pdata->gpio_reset);
            return error;
        }

		error = gpio_direction_output(pdata->gpio_reset, 1);
		if (error) {
			JD_E("unable to set direction for rst-gpio [%d]\n", pdata->gpio_reset);
			gpio_free(pdata->gpio_reset);
			return error;
		}
	} else {
		JD_E("rst-gpio [%d] is not valid\n", pdata->gpio_reset);
		return error;
	}
#endif

    if (gpio_is_valid(pdata->gpio_irq)) {
        error = gpio_request(pdata->gpio_irq, "jadard_gpio_irq");

        if (error) {
            JD_E("unable to request irq-gpio [%d]\n", pdata->gpio_irq);
#ifdef JD_RST_PIN_FUNC
			if (gpio_is_valid(pdata->gpio_reset))
				gpio_free(pdata->gpio_reset);
#endif
            return error;
        }

        error = gpio_direction_input(pdata->gpio_irq);
        if (error) {
            JD_E("unable to set direction for irq-gpio [%d]\n", pdata->gpio_irq);
            gpio_free(pdata->gpio_irq);
#ifdef JD_RST_PIN_FUNC
            if (gpio_is_valid(pdata->gpio_reset)) {
                gpio_free(pdata->gpio_reset);
            }
#endif
            return error;
        }

        client->irq = gpio_to_irq(pdata->gpio_irq);
        pjadard_ts_data->jd_irq = client->irq;
    } else {
        JD_E("irq-gpio [%d] is not valid\n", pdata->gpio_irq);
#ifdef JD_RST_PIN_FUNC
		if (gpio_is_valid(pdata->gpio_reset))
			gpio_free(pdata->gpio_reset);
#endif
        return error;
    }

    msleep(20);
#ifdef JD_RST_PIN_FUNC
    if (gpio_is_valid(pdata->gpio_reset)) {
        error = gpio_direction_output(pdata->gpio_reset, 1);

        if (error) {
            JD_E("unable to set direction for rst-gpio [%d]\n", pdata->gpio_reset);
            gpio_free(pdata->gpio_reset);
            if (gpio_is_valid(pdata->gpio_irq)) {
                gpio_free(pdata->gpio_irq);
            }
            return error;
        }
        gpio_free(pdata->gpio_reset);
    } else {
        JD_E("rst-gpio [%d] is not valid\n", pdata->gpio_reset);
        if (gpio_is_valid(pdata->gpio_irq)) {
            gpio_free(pdata->gpio_irq);
        }
        return error;
    }
#endif

    return error;
}

void jadard_gpio_power_deconfig(struct jadard_platform_data *pdata)
{
    /* Only MTK plateform using */
}

#endif

irqreturn_t jadard_ts_isr_func(int irq, void *ptr)
{
#ifndef CONFIG_JD_HID
	if (pjadard_ts_data->fw_ready)
		jadard_ts_work((struct jadard_ts_data *)ptr);
#else
	if (ihid->ready)
		jadard_i2c_hid_work((struct jadard_i2c_hid *)ptr);
#endif
    return IRQ_HANDLED;
}

int jadard_int_register_trigger(void)
{
    int ret = JD_NO_ERR;
    struct i2c_client *client = pjadard_ts_data->client;

#ifndef CONFIG_JD_HID
    struct jadard_ts_data *ts = pjadard_ts_data;

    if (pjadard_ic_data->JD_INT_EDGE) {
        JD_I("%s edge triiger\n", __func__);
        ret = request_threaded_irq(client->irq, NULL, jadard_ts_isr_func,
                                    IRQF_TRIGGER_FALLING | IRQF_ONESHOT, client->name, ts);
    } else {
        JD_I("%s level trigger\n", __func__);
        ret = request_threaded_irq(client->irq, NULL, jadard_ts_isr_func,
                                    IRQF_TRIGGER_LOW | IRQF_ONESHOT, client->name, ts);
    }
#else
    if (pjadard_ic_data->JD_INT_EDGE) {
        JD_I("%s edge triiger\n", __func__);
        ret = request_threaded_irq(client->irq, NULL, jadard_ts_isr_func,
                                    IRQF_TRIGGER_FALLING | IRQF_ONESHOT, client->name, ihid);
    } else {
        JD_I("%s level trigger\n", __func__);
        ret = request_threaded_irq(client->irq, NULL, jadard_ts_isr_func,
                                    IRQF_TRIGGER_LOW | IRQF_ONESHOT, client->name, ihid);
    }
#endif
    return ret;
}

void jadard_int_en_set(bool enable)
{
    struct jadard_ts_data *ts = pjadard_ts_data;

    if (enable) {
        if (jadard_int_register_trigger() == 0) {
            ts->irq_enabled = 1;
        }
    } else {
        jadard_int_enable(false);
        free_irq(ts->client->irq, ts);
    }
}

int jadard_ts_register_interrupt(void)
{
    struct jadard_ts_data *ts = pjadard_ts_data;
    struct i2c_client *client = pjadard_ts_data->client;
    int ret = 0;
    ts->irq_enabled = 0;

    if (client->irq) {
        ret = jadard_int_register_trigger();

        if (ret == 0) {
            ts->irq_enabled = 1;
            JD_I("%s: irq enabled at IRQ: %d\n", __func__, client->irq);
        } else {
            JD_E("%s: request_irq failed\n", __func__);
        }
    } else {
        JD_I("%s: client->irq is empty.\n", __func__);
    }

    return ret;
}

void jadard_ts_free_interrupt(void)
{
    struct jadard_ts_data *ts = pjadard_ts_data;

    free_irq(ts->client->irq, ts);
}

/*
#if defined(JD_DEV_PM) || defined(JD_CONFIG_FB) || defined(JD_CONFIG_DRM) ||\
	defined(JD_CONFIG_DRM_V2) || defined(JD_CONFIG_DRM_MSM)
*/
static int jadard_common_suspend(struct device *dev)
{
    struct jadard_ts_data *ts = dev_get_drvdata(dev);

	JD_I("%s: enter\n", __func__);
	jadard_chip_common_suspend(ts);

    return 0;
}

static int jadard_common_resume(struct device *dev)
{
#ifndef JD_RESUME_NOT_WAIT_FW
	struct jadard_ts_data *ts = dev_get_drvdata(dev);

	JD_I("%s: enter\n", __func__);
	jadard_chip_common_resume(ts);
#endif
	return 0;
}
//#endif

#if defined(JD_CONFIG_DRM) || defined(JD_CONFIG_DRM_V2)
int jadard_drm_check_dt(struct jadard_ts_data *ts)
{
    struct device_node *dt = ts->dev->of_node;
    struct device_node *node = NULL;
    struct drm_panel *panel = NULL;
    int i = 0;
    int count = 0;

    count = of_count_phandle_with_args(dt, "panel", NULL);
    if (count <= 0) {
        JD_I("%s: find drm_panel count(%d) fail\n", __func__, count);
        return 0;
    }
    JD_I("%s: find drm_panel count(%d)\n", __func__, count);
    for (i = 0; i < count; i++) {
        node = of_parse_phandle(dt, "panel", i);
        JD_I("DRM:node = %p\n", node);
        panel = of_drm_find_panel(node);
        JD_I("DRM:panel = %p\n", panel);

        of_node_put(node);
        if (!IS_ERR(panel)) {
            JD_I("%s: find drm_panel successfully\n", __func__);
            ts->active_panel = panel;
            return 0;
        }
    }

    JD_E("%s: no find drm_panel\n", __func__);

    return 0;
}
#endif

#if defined(JD_CONFIG_FB)
#ifdef JD_CONFIG_DRM_V2
int jadard_drm_notifier_callback(enum panel_event_notifier_tag tag,
								struct panel_event_notification *notification,	void *data)
{
	JD_I("DRM: %s\n", __func__);

	if (!notification) {
		JD_I("Invalid notification");
		return 0;
	}

	switch (notification->notif_type) {
#if 0
	case DRM_PANEL_EVENT_UNBLANK:
		if (notification->notif_data.early_trigger) {
			JD_I("resume notification early commit");
		} else {
#if defined(JD_RESUME_NOT_WAIT_FW)
			JD_I("resume: TP_RESUME_THREAD undelay\n");
			queue_delayed_work(pjadard_ts_data->jadard_resume_wq,
				&pjadard_ts_data->jadard_resume_work, 0);
#else
			JD_I("resume: TP_RESUME\n");
			jadard_common_resume(pjadard_ts_data->dev);
#endif
		}
		break;
#else
	case DRM_PANEL_EVENT_UNBLANK:
		if (notification->notif_data.early_trigger) {
#if defined(JD_RESUME_NOT_WAIT_FW)
			JD_I("resume: TP_RESUME_THREAD\n");
			queue_delayed_work(pjadard_ts_data->jadard_resume_wq,
				&pjadard_ts_data->jadard_resume_work,
				msecs_to_jiffies(JD_RESUME_DELAY_TIME));
#else
			JD_I("resume notification early commit");
#endif
		} else {
#if defined(JD_RESUME_NOT_WAIT_FW)
			JD_I("resume: late event Skipped\n");
#else
			JD_I("resume: TP_RESUME\n");
			jadard_common_resume(pjadard_ts_data->dev);
#endif
		}
		break;
#endif
	case DRM_PANEL_EVENT_BLANK:
		if (notification->notif_data.early_trigger)
			jadard_common_suspend(pjadard_ts_data->dev);
		else
			JD_I("suspend notification late commit");
		break;
	default:
		JD_I("notification serviced :%d", notification->notif_type);
		break;
	}

	return 0;
}
#else

#ifdef JD_CONFIG_DRM
int jadard_fb_notifier_callback(struct notifier_block *self,
                            unsigned long event, void *data)
{
    struct drm_panel_notifier *evdata = data;
    int *blank;
    struct jadard_ts_data *ts =
        container_of(self, struct jadard_ts_data, fb_notif);
    JD_I("DRM: %s\n", __func__);

    if (evdata && evdata->data &&
        ((event == DRM_PANEL_EARLY_EVENT_BLANK) || (event == DRM_PANEL_EVENT_BLANK)) &&
        ts != NULL && ts->dev != NULL) {
        blank = evdata->data;
        JD_I("DRM event:%lu, blank:%d\n", event, *blank);

		switch (*blank) {
#if 0
		case DRM_PANEL_BLANK_UNBLANK:
			if (DRM_PANEL_EARLY_EVENT_BLANK == event) {
				JD_I("resume: event = %lu, Skipped\n", event);
			} else if (DRM_PANEL_EVENT_BLANK == event) {
#if defined(JD_RESUME_NOT_WAIT_FW)
				JD_I("resume: event = %lu, TP_RESUME_THREAD undelay\n", event);
				queue_delayed_work(ts->jadard_resume_wq, &ts->jadard_resume_work, 0);
#else
				JD_I("resume: event = %lu, TP_RESUME\n", event);
				jadard_common_resume(ts->dev);
#endif
			}
			break;
#else
		case DRM_PANEL_BLANK_UNBLANK:
			if (DRM_PANEL_EARLY_EVENT_BLANK == event) {
#if defined(JD_RESUME_NOT_WAIT_FW)
				JD_I("resume: event = %lu, TP_RESUME_THREAD\n", event);
				queue_delayed_work(ts->jadard_resume_wq, &ts->jadard_resume_work,
					msecs_to_jiffies(JD_RESUME_DELAY_TIME));
#else
				JD_I("resume: event = %lu, Skipped\n", event);
#endif
			} else if (DRM_PANEL_EVENT_BLANK == event) {
#if defined(JD_RESUME_NOT_WAIT_FW)
				JD_I("resume: event = %lu, Skipped\n", event);
#else
				JD_I("resume: event = %lu, TP_RESUME\n", event);
				jadard_common_resume(ts->dev);
#endif
			}
			break;
#endif
		case DRM_PANEL_BLANK_POWERDOWN:
			if (DRM_PANEL_EARLY_EVENT_BLANK == event) {
				JD_I("suspend: event = %lu, TP_SUSPEND\n", event);
				jadard_common_suspend(ts->dev);
			} else if (DRM_PANEL_EVENT_BLANK == event) {
				JD_I("suspend: event = %lu, Skipped\n", event);
			}
			break;
		}
	}

    return 0;
}
#else
#ifdef JD_CONFIG_DRM_MSM
int jadard_fb_notifier_callback(struct notifier_block *self,
                            unsigned long event, void *data)
{
    struct fb_event *evdata = data;
    int *blank;
    struct jadard_ts_data *ts =
        container_of(self, struct jadard_ts_data, fb_notif);
    JD_I("MSM_DRM: %s\n", __func__);

    if (evdata && evdata->data &&
        ((event == MSM_DRM_EARLY_EVENT_BLANK) || (event == MSM_DRM_EVENT_BLANK)) &&
        ts != NULL && ts->dev != NULL) {
        blank = evdata->data;
        JD_I("MSM_DRM event:%lu, blank:%d\n", event, *blank);

		switch (*blank) {
#if 0
		case MSM_DRM_BLANK_UNBLANK:
			if (MSM_DRM_EARLY_EVENT_BLANK == event) {
				JD_I("resume: event = %lu, Skipped\n", event);
			} else if (MSM_DRM_EVENT_BLANK == event) {
#if defined(JD_RESUME_NOT_WAIT_FW)
				JD_I("resume: event = %lu, TP_RESUME_THREAD undelay\n", event);
				queue_delayed_work(ts->jadard_resume_wq, &ts->jadard_resume_work, 0);
#else
				JD_I("resume: event = %lu, TP_RESUME\n", event);
				jadard_common_resume(ts->dev);
#endif
			}
			break;
#else
		case MSM_DRM_BLANK_UNBLANK:
			if (MSM_DRM_EARLY_EVENT_BLANK == event) {
#if defined(JD_RESUME_NOT_WAIT_FW)
				JD_I("resume: event = %lu, TP_RESUME_THREAD\n", event);
				queue_delayed_work(ts->jadard_resume_wq, &ts->jadard_resume_work,
					msecs_to_jiffies(JD_RESUME_DELAY_TIME));
#else
				JD_I("resume: event = %lu, Skipped\n", event);
#endif
			} else if (MSM_DRM_EVENT_BLANK == event) {
#if defined(JD_RESUME_NOT_WAIT_FW)
				JD_I("resume: event = %lu, Skipped\n", event);
#else
				JD_I("resume: event = %lu, TP_RESUME\n", event);
				jadard_common_resume(ts->dev);
#endif
			}
			break;
#endif
		case MSM_DRM_BLANK_POWERDOWN:
			if (MSM_DRM_EARLY_EVENT_BLANK == event) {
				JD_I("suspend: event = %lu, TP_SUSPEND\n", event);
				jadard_common_suspend(ts->dev);
			} else if (MSM_DRM_EVENT_BLANK == event) {
				JD_I("suspend: event = %lu, Skipped\n", event);
			}
			break;
		}
	}

    return 0;
}
#else
int jadard_fb_notifier_callback(struct notifier_block *self,
                            unsigned long event, void *data)
{
    struct fb_event *evdata = data;
    int *blank;
    struct jadard_ts_data *ts =
        container_of(self, struct jadard_ts_data, fb_notif);
    JD_I("FB: %s\n", __func__);

    if (evdata && evdata->data &&
        ((event == FB_EARLY_EVENT_BLANK) || (event == FB_EVENT_BLANK)) &&
        ts != NULL && ts->dev != NULL) {
        blank = evdata->data;
        JD_I("FB event:%lu, blank:%d\n", event, *blank);

		switch (*blank) {
#if 0
		case FB_BLANK_UNBLANK:
			if (FB_EARLY_EVENT_BLANK == event) {
				JD_I("resume: event = %lu, Skipped\n", event);
			} else if (FB_EVENT_BLANK == event) {
#if defined(JD_RESUME_NOT_WAIT_FW)
				JD_I("resume: event = %lu, TP_RESUME_THREAD undelay\n", event);
				queue_delayed_work(ts->jadard_resume_wq, &ts->jadard_resume_work, 0);
#else
				JD_I("resume: event = %lu, TP_RESUME\n", event);
				jadard_common_resume(ts->dev);
#endif
			}
			break;
#else
		case FB_BLANK_UNBLANK:
			if (FB_EARLY_EVENT_BLANK == event) {
#if defined(JD_RESUME_NOT_WAIT_FW)
				JD_I("resume: event = %lu, TP_RESUME_THREAD\n", event);
				queue_delayed_work(ts->jadard_resume_wq, &ts->jadard_resume_work,
					msecs_to_jiffies(JD_RESUME_DELAY_TIME));
#else
				JD_I("resume: event = %lu, Skipped\n", event);
#endif
			} else if (FB_EVENT_BLANK == event) {
#if defined(JD_RESUME_NOT_WAIT_FW)
				JD_I("resume: event = %lu, Skipped\n", event);
#else
				JD_I("resume: event = %lu, TP_RESUME\n", event);
				jadard_common_resume(ts->dev);
#endif
			}
			break;
#endif
		case FB_BLANK_POWERDOWN:
		case FB_BLANK_HSYNC_SUSPEND:
		case FB_BLANK_VSYNC_SUSPEND:
		case FB_BLANK_NORMAL:
			if (FB_EARLY_EVENT_BLANK == event) {
				JD_I("suspend: event = %lu, TP_SUSPEND\n", event);
				jadard_common_suspend(ts->dev);
			} else if (FB_EVENT_BLANK == event) {
				JD_I("suspend: event = %lu, Skipped\n", event);
			}
			break;
		}
	}

    return 0;
}
#endif /* JD_CONFIG_DRM_MSM */
#endif /* JD_CONFIG_DRM */
#endif /* JD_CONFIG_DRM_V2 */
#endif /* defined(JD_CONFIG_FB) */

int jadard_chip_common_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct jadard_ts_data *ts = NULL;

    JD_I("%s:Enter \n", __func__);

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        JD_E("%s: i2c check functionality error\n", __func__);
        return -ENODEV;
    }

    ts = kzalloc(sizeof(struct jadard_ts_data), GFP_KERNEL);
    if (ts == NULL) {
        JD_E("%s: allocate jadard_ts_data failed\n", __func__);
        return -ENOMEM;
    }

    ts->client = client;
    ts->dev = &client->dev;
    ts->spi = NULL;
    i2c_set_clientdata(client, ts);

    mutex_init(&ts->rw_lock);
    pjadard_ts_data = ts;
    spin_lock_init(&ts->irq_active);

    return jadard_chip_common_init();
}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)
static int jadard_chip_common_remove(struct i2c_client *client)
{
	jadard_chip_common_deinit();

    return 0;
}
#else
int jadard_chip_common_remove(struct i2c_client *client)
{
	jadard_chip_common_deinit();

    return 0;
}
#endif

static const struct i2c_device_id jadard_common_ts_id[] = {
    {JADARD_common_NAME, 0 },
    {}
};

#if 1 //defined(JD_DEV_PM)
static const struct dev_pm_ops jadard_common_pm_ops = {
#if (!defined(JD_CONFIG_FB)) && (!defined(JD_CONFIG_DRM)) &&\
	(!defined(JD_CONFIG_DRM_V2)) && (!defined(JD_CONFIG_DRM_MSM))
	.suspend = jadard_common_suspend,
	.resume	 = jadard_common_resume,
#endif
};
#endif

#ifdef CONFIG_OF
static struct of_device_id jadard_match_table[] = {
    {.compatible = "jadard,jdcommon" },
    {},
};
#else
#define jadard_match_table NULL
#endif

static struct i2c_driver jadard_common_driver = {
	.id_table	= jadard_common_ts_id,
	.probe		= jadard_chip_common_probe,
	.remove		= jadard_chip_common_remove,
	.driver		= {
		.name = JADARD_common_NAME,
		.owner = THIS_MODULE,
		.of_match_table = jadard_match_table,
#if 1 //CONFIG_PM
		.pm	= &jadard_common_pm_ops,
#endif
	},
};

#if defined(__JADARD_KMODULE__)
int jadard_common_init(void)
{
    JD_I("I2C Jadard kmodule common touch panel driver init\n");

    i2c_add_driver(&jadard_common_driver);

    return 0;
}

void jadard_common_exit(void)
{
    i2c_del_driver(&jadard_common_driver);
}

#else
static int __init jadard_common_init(void)
{
    JD_I("I2C Jadard common touch panel driver init\n");

#ifndef CONFIG_JD_DB
    i2c_add_driver(&jadard_common_driver);
#endif

    return 0;
}

#if defined(CONFIG_JD_DB)
void jadard_workarround_init(void)
{
    JD_I("I2C jadard_workarround_init by Driver\n");

    i2c_add_driver(&jadard_common_driver);
}
#endif

static void __exit jadard_common_exit(void)
{
    i2c_del_driver(&jadard_common_driver);
}

module_init(jadard_common_init);
module_exit(jadard_common_exit);

MODULE_DESCRIPTION("Jadard_common driver");
MODULE_LICENSE("GPL");

#endif
