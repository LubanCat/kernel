#ifndef JADARD_PLATFORM_H
#define JADARD_PLATFORM_H

#include "jadard_common.h"

#ifdef CONFIG_JD_HID
#include "jadard_hid.h"
#endif

#if defined(CONFIG_JD_DB)
#include <linux/regulator/consumer.h>
#endif

#define JADARD_common_NAME "jadard_tp"
#define JADARD_BUS_RETRY_TIMES 3

#if defined(CONFIG_TOUCHSCREEN_JADARD_DEBUG)
extern bool jd_g_dbg_enable;
extern bool jd_g_esd_check_enable;
#define JD_I(x...) \
do { if (!jd_g_esd_check_enable) printk(KERN_ERR "[JDTP] " x); } while (0)
#define JD_W(x...) printk(KERN_ERR "[JDTP][WARNING] " x)
#define JD_E(x...) printk(KERN_ERR "[JDTP][ERROR] " x)
#define JD_D(x...) \
do { if (jd_g_dbg_enable) printk(KERN_ERR "[JDTP][DEBUG] " x); } while (0)
#else
#define JD_I(x...)
#define JD_W(x...)
#define JD_E(x...)
#define JD_D(x...)
#endif

struct jadard_platform_data {
    int abs_x_min;
    int abs_x_max;
    int abs_x_fuzz;
    int abs_y_min;
    int abs_y_max;
    int abs_y_fuzz;
    int abs_pressure_min;
    int abs_pressure_max;
    int abs_pressure_fuzz;
    int abs_width_min;
    int abs_width_max;
    uint8_t usb_status[2];
    int gpio_irq;
    int gpio_reset;

#if defined(CONFIG_JD_DB)
    struct regulator *vcc_ana; /* For Dragon Board */
    struct regulator *vcc_dig; /* For Dragon Board */
#endif

#ifdef JD_SPRD_EIC_MODE
	struct pinctrl *pinctrl;
	struct pinctrl_state *pin_eic;
	struct pinctrl_state *pin_gpio;
#endif
};

#ifdef CONFIG_JD_HID
int jadard_bus_read_single(uint8_t *data, uint16_t data_len, uint8_t toRetry);
#endif
int jadard_bus_read(uint8_t *cmd, uint8_t cmd_len, uint8_t *data, uint32_t data_len, uint8_t toRetry);
int jadard_bus_write(uint8_t *cmd, uint8_t cmd_len, uint8_t *data, uint32_t data_len, uint8_t toRetry);
void jadard_int_en_set(bool enable);
void jadard_int_enable(bool enable);
int jadard_ts_register_interrupt(void);
void jadard_ts_free_interrupt(void);
int jadard_gpio_power_config(struct jadard_platform_data *pdata);
void jadard_gpio_power_deconfig(struct jadard_platform_data *pdata);

#if defined(JD_CONFIG_FB)
#if defined(JD_CONFIG_DRM_V2)
int jadard_drm_notifier_callback(enum panel_event_notifier_tag tag,
								struct panel_event_notification *notification,	void *data);
#endif
#if defined(JD_CONFIG_DRM)
int jadard_fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data);
#endif
#endif

void jadard_gpio_set_value(int pin_num, uint8_t value);
#endif
