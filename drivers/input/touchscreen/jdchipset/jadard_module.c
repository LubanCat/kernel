#include "jadard_common.h"
#include "jadard_platform.h"
#include "jadard_module.h"

extern struct jadard_module_fp g_module_fp;
extern struct jadard_ts_data *pjadard_ts_data;
extern struct jadard_ic_data *pjadard_ic_data;
struct jadard_common_variable g_common_variable;

#if defined(JD_AUTO_UPGRADE_FW) || defined(JD_ZERO_FLASH)
#ifdef JD_UPGRADE_FW_ARRAY
extern uint8_t jd_i_firmware[];
extern uint32_t jd_fw_size;
#endif
extern char *jd_i_CTPM_firmware_name;
#if defined(JD_ZERO_FLASH)
bool jd_g_f_0f_update;
#endif
#endif

const char *jadard_bit_map[16] = {
    [0] = "0000", [1] = "0001", [2] = "0010", [3] = "0011",
    [4] = "0100", [5] = "0101", [6] = "0110", [7] = "0111",
    [8] = "1000", [9] = "1001", [10] = "1010", [11] = "1011",
    [12] = "1100", [13] = "1101", [14] = "1110", [15] = "1111",
};

static int jadard_mcu_register_read(uint32_t ReadAddr, uint8_t *ReadData, uint32_t ReadLen)
{
    JD_I("%s: nothing to do, only for function pointer initial\n", __func__);
    return JD_NO_ERR;
}

static int jadard_mcu_register_write(uint32_t WriteAddr, uint8_t *WriteData, uint32_t WriteLen)
{
    JD_I("%s: nothing to do, only for function pointer initial\n", __func__);
    return JD_NO_ERR;
}

static uint8_t jadard_mcu_dd_register_write(uint8_t page, uint8_t cmd, uint8_t *par, uint8_t par_len, uint32_t offset)
{
	uint8_t page_en;

	if (g_common_variable.dbi_dd_reg_mode == JD_DDREG_MODE_1) {
		return jadard_DbiWriteDDReg(page, cmd, par, par_len, offset);
	} else {
		page_en = g_module_fp.fp_ReadDbicPageEn();
		if (page_en != 0xFF) {
			if ((cmd == 0xDE) && (page_en > 0)) {
				g_module_fp.fp_SetDbicPage(par[0]);
				return JD_DBIC_READ_WRITE_SUCCESS;
			} else {
				return jadard_DbicWriteDDReg(cmd, par, par_len);
			}
		} else {
			return jadard_DbicWriteDDReg(cmd, par, par_len);
		}
	}
}

static uint8_t jadard_mcu_dd_register_read(uint8_t page, uint8_t cmd, uint8_t *par, uint8_t par_len, uint32_t offset)
{
	if (g_common_variable.dbi_dd_reg_mode == JD_DDREG_MODE_1)
		return jadard_DbiReadDDReg(page, cmd, par, par_len, offset);
	else
		return jadard_DbicReadDDReg(cmd, par, par_len);
}

static void jadard_mcu_set_sleep_mode(uint8_t *value, uint8_t value_len)
{
    JD_I("%s: nothing to do, only for function pointer initial\n", __func__);
}

static void jadard_mcu_read_fw_ver(void)
{
    JD_I("%s: nothing to do, only for function pointer initial\n", __func__);
}

static void jadard_mcu_mutual_data_set(uint8_t data_type)
{
    JD_I("%s: nothing to do, only for function pointer initial\n", __func__);
}

static int jadard_mcu_get_mutual_data(uint8_t data_type, uint8_t *rdata, uint16_t rlen)
{
    JD_I("%s: nothing to do, only for function pointer initial\n", __func__);
    return JD_NO_ERR;
}

static bool jadard_mcu_get_touch_data(uint8_t *buf, uint8_t length)
{
    JD_I("%s: nothing to do, only for function pointer initial\n", __func__);
    return false;
}

static int jadard_mcu_read_mutual_data(uint8_t *rdata, uint16_t rlen)
{
    JD_I("%s: nothing to do, only for function pointer initial\n", __func__);
    return false;
}

static void jadard_mcu_report_points(struct jadard_ts_data *ts)
{
    jadard_report_points(ts);
}

static int jadard_mcu_parse_report_data(struct jadard_ts_data *ts, int irq_event, int ts_status)
{
    return jadard_parse_report_data(ts, irq_event, ts_status);
}

static int jadard_mcu_distribute_touch_data(struct jadard_ts_data *ts, uint8_t *buf, int irq_event, int ts_status)
{
    return jadard_distribute_touch_data(ts, buf, irq_event, ts_status);
}

#ifndef JD_ZERO_FLASH
static int jadard_mcu_flash_read(uint32_t start_addr, uint8_t *rdata, uint32_t rlen)
{
    JD_I("%s: nothing to do, only for function pointer initial\n", __func__);
    return JD_NO_ERR;
}

static int jadard_mcu_flash_write(uint32_t start_addr, uint8_t *wdata, uint32_t wlen)
{
    JD_I("%s: nothing to do, only for function pointer initial\n", __func__);
    return JD_NO_ERR;
}
#endif

static void jadard_mcu_EnterBackDoor(void)
{
    JD_I("%s: not support enter backdoor cmd\n", __func__);
}

static void jadard_mcu_ExitBackDoor(void)
{
    JD_I("%s: not support exit backdoor cmd\n", __func__);
}

static void jadard_mcu_PinReset(uint8_t reset_type)
{
#ifdef JD_RST_PIN_FUNC
	JD_I("%s: reset_type(%d)\n", __func__, reset_type);

	if (gpio_is_valid(pjadard_ts_data->rst_gpio)) {
		if (gpio_request(pjadard_ts_data->rst_gpio, "jadard_reset_gpio")) {
			JD_E("unable to request rst-gpio [%d]\n", pjadard_ts_data->rst_gpio);
			goto err_gpio;
		}

		if (gpio_direction_output(pjadard_ts_data->rst_gpio, 1)) {
			JD_E("unable to set direction for rst-gpio [%d]\n", pjadard_ts_data->rst_gpio);
			gpio_free(pjadard_ts_data->rst_gpio);
			goto err_gpio;
		}

		if (reset_type == 3) {
			jadard_gpio_set_value(pjadard_ts_data->rst_gpio, 1);
		} else if (reset_type == 2) {
			jadard_gpio_set_value(pjadard_ts_data->rst_gpio, 0);
		} else if (reset_type == 1) {
			jadard_gpio_set_value(pjadard_ts_data->rst_gpio, 0);
			mdelay(10);
			jadard_gpio_set_value(pjadard_ts_data->rst_gpio, 1);
		} else if (reset_type == 0) {
			jadard_gpio_set_value(pjadard_ts_data->rst_gpio, 1);
			mdelay(10);
			jadard_gpio_set_value(pjadard_ts_data->rst_gpio, 0);
			mdelay(10);
			jadard_gpio_set_value(pjadard_ts_data->rst_gpio, 1);
			mdelay(100);
		}

		gpio_free(pjadard_ts_data->rst_gpio);
	} else {
		JD_E("rst-gpio [%d] is not valid\n", pjadard_ts_data->rst_gpio);
	}

err_gpio:
#endif

	if (reset_type == 0)
		g_module_fp.fp_EnterBackDoor();
}

static void jadard_mcu_SoftReset(void)
{
    JD_I("%s: nothing to do, only for function pointer initial\n", __func__);
}

static void jadard_mcu_ResetMCU(void)
{
    JD_I("%s: nothing to do, only for function pointer initial\n", __func__);
}

static void jadard_mcu_PorInit(void)
{
    JD_I("%s: nothing to do, only for function pointer initial\n", __func__);
}

#ifdef JD_RST_PIN_FUNC
/*
 * Parameter signification:
 * reloadcfg int_off_on
 *   true      true : Reload config & HW reset & INT off->on
 *   true      false: Reload config & HW reset
 *   false     true : HW reset & INT off->on
 *   false     false: HW reset
*/
static void jadard_mcu_ic_reset(bool reload_cfg, bool int_off_on)
{
    JD_I("%s: reload_cfg = %d, int_off_on = %d\n", __func__, reload_cfg, int_off_on);
    pjadard_ts_data->rst_active = true;

	if (pjadard_ts_data->rst_gpio >= 0) {
		if (int_off_on)
			jadard_int_enable(false);

		g_module_fp.fp_pin_reset(0);

		if (reload_cfg)
			g_module_fp.fp_report_data_reinit();

		if (int_off_on)
			jadard_int_enable(true);
	}
}
#endif

static void jadard_mcu_ic_soft_reset(void)
{
    pjadard_ts_data->rst_active = true;
    jadard_int_enable(false);
    g_module_fp.fp_soft_reset();
    jadard_int_enable(true);
}

static void jadard_mcu_touch_info_set(void)
{
    pjadard_ic_data->JD_STYLUS_ID_EN = 0;
    pjadard_ic_data->JD_STYLUS_RATIO = 1;

    /* Data set by jadard_parse_dt() */
    JD_I("%s: JD_X_NUM = %d, JD_Y_NUM = %d, JD_MAX_PT = %d\n", __func__,
        pjadard_ic_data->JD_X_NUM, pjadard_ic_data->JD_Y_NUM, pjadard_ic_data->JD_MAX_PT);
    JD_I("%s: JD_X_RES = %d, JD_Y_RES = %d\n", __func__, pjadard_ic_data->JD_X_RES, pjadard_ic_data->JD_Y_RES);
    JD_I("%s: JD_INT_EDGE = %d\n", __func__, pjadard_ic_data->JD_INT_EDGE);
    JD_I("%s: JD_STYLUS_EN = %d, JD_STYLUS_ID_EN = %d, JD_STYLUS_RATIO = %d\n", __func__,
        pjadard_ic_data->JD_STYLUS_EN, pjadard_ic_data->JD_STYLUS_ID_EN, pjadard_ic_data->JD_STYLUS_RATIO);
}

static void jadard_mcu_report_data_reinit(void)
{
	if (jadard_report_data_init())
		JD_E("%s: allocate data fail\n", __func__);
}

static void jadard_mcu_usb_detect_set(uint8_t *usb_status)
{
    JD_I("%s: nothing to do, only for function pointer initial\n", __func__);
}

static uint8_t jadard_mcu_get_freq_band(void)
{
    JD_I("%s: not support get freq band cmd\n", __func__);
    return 0xFF;
}

static uint8_t jadard_mcu_ReadDbicPageEn(void)
{
    JD_I("%s: not support read dbic page\n", __func__);
    return 0xFF;
}

static int jadard_mcu_SetDbicPage(uint8_t page)
{
    JD_I("%s: not support set dbic page\n", __func__);
    return JD_NO_ERR;
}

static void jadard_mcu_log_touch_state(void)
{
    jadard_log_touch_state(jadard_bit_map);
}

#if defined(JD_SMART_WAKEUP) || defined(JD_USB_DETECT_GLOBAL) || defined(JD_USB_DETECT_CALLBACK) ||\
    defined(JD_HIGH_SENSITIVITY) || defined(JD_ROTATE_BORDER) || defined(JD_EARPHONE_DETECT)
static void jadard_mcu_resume_set_func(bool suspended)
{
#ifdef JD_SMART_WAKEUP
    g_module_fp.fp_set_SMWP_enable(pjadard_ts_data->SMWP_enable);
#endif
#ifdef JD_USB_DETECT_GLOBAL
    jadard_cable_detect(true);
#endif
#ifdef JD_USB_DETECT_CALLBACK
    jadard_usb_status(pjadard_ts_data->usb_connected, true);
#endif
#ifdef JD_HIGH_SENSITIVITY
    g_module_fp.fp_set_high_sensitivity(pjadard_ts_data->high_sensitivity_enable);
#endif
#ifdef JD_ROTATE_BORDER
    g_module_fp.fp_set_rotate_border(pjadard_ts_data->rotate_border);
#endif
#ifdef JD_EARPHONE_DETECT
    g_module_fp.fp_set_earphone_enable(pjadard_ts_data->earphone_enable);
#endif
}
#endif

#ifdef JD_HIGH_SENSITIVITY
static void jadard_mcu_set_high_sensitivity(bool enable)
{
	JD_I("%s: nothing to do, only for function pointer initial\n", __func__);
}
#endif

#ifdef JD_ROTATE_BORDER
static void jadard_mcu_set_rotate_border(uint16_t rotate)
{
	JD_I("%s: nothing to do, only for function pointer initial\n", __func__);
}
#endif

#ifdef JD_EARPHONE_DETECT
static void jadard_mcu_set_earphone_enable(uint8_t status)
{
    JD_I("%s: nothing to do, only for function pointer initial\n", __func__);
}
#endif

static void jadard_mcu_set_SMWP_enable(bool enable)
{
	JD_I("%s: nothing to do, only for function pointer initial\n", __func__);
}

#if (JD_PROXIMITY_MODE != JD_PROXIMITY_DISABLE)
static void jadard_mcu_set_virtual_proximity(bool enable)
{
	JD_I("%s: nothing to do, only for function pointer initial\n", __func__);
}
#endif

#if defined(JD_AUTO_UPGRADE_FW)
static int jadard_mcu_read_fw_ver_bin(void)
{
    JD_I("%s: nothing to do, only for function pointer initial\n", __func__);
    return JD_NO_ERR;
}
#endif

static int jadard_mcu_ram_read(uint32_t start_addr, uint8_t *rdata, uint32_t rlen)
{
    JD_I("%s: nothing to do, only for function pointer initial\n", __func__);
    return JD_NO_ERR;
}

#ifdef JD_ZERO_FLASH
static int jadard_mcu_ram_write(uint32_t start_addr, uint8_t *wdata, uint32_t wlen)
{
    JD_I("%s: nothing to do, only for function pointer initial\n", __func__);
    return JD_NO_ERR;
}

static int jadard_mcu_0f_upgrade_fw(char *file_name)
{
    int err = JD_NO_ERR;
#ifdef JD_UPGRADE_FW_ARRAY
    const uint8_t *fw_data = jd_i_firmware;
    JD_I("file name = %s\n", jd_i_CTPM_firmware_name);
#else
    int RetryCnt;
    const struct firmware *fw = NULL;

    JD_I("file name = %s\n", file_name);

    for (RetryCnt = 0; RetryCnt < JD_UPGRADE_FW_RETRY_TIME; RetryCnt++) {
        err = request_firmware(&fw, file_name, pjadard_ts_data->dev);
        if (err < 0) {
            JD_E("%s: Open file fail(ret:%d), RetryCnt = %d\n", __func__, err, RetryCnt);
            mdelay(1000);
        } else {
            break;
        }
    }

    if (RetryCnt == JD_UPGRADE_FW_RETRY_TIME) {
        JD_E("%s: Open file fail retry over %d\n", __func__, JD_UPGRADE_FW_RETRY_TIME);
        return JD_FILE_OPEN_FAIL;
    }
#endif
    if (jd_g_f_0f_update) {
        JD_W("%s: Other thread is upgrade now\n", __func__);
        err = JD_UPGRADE_CONFLICT;
    } else {
        JD_I("%s: Entering upgrade Flow\n", __func__);
        jadard_int_enable(false);
        jd_g_f_0f_update = true;

#ifdef JD_UPGRADE_FW_ARRAY
        JD_I("FW size = %d\n", (int)jd_fw_size);
        err = g_module_fp.fp_ram_write(0, (uint8_t *)fw_data, jd_fw_size);
#else
        JD_I("FW size = %d\n", (int)fw->size);
        err = g_module_fp.fp_ram_write(0, (uint8_t *)fw->data, fw->size);
        release_firmware(fw);
#endif
        jd_g_f_0f_update = false;

		if (err >= 0)
			pjadard_ts_data->fw_ready = true;
		else
			pjadard_ts_data->fw_ready = false;
	}

    return err;
}

#ifdef JD_ESD_CHECK
static int jadard_mcu_0f_esd_upgrade_fw(char *file_name)
{
    int err = JD_NO_ERR;
#ifdef JD_UPGRADE_FW_ARRAY
    const uint8_t *fw_data = jd_i_firmware;
    JD_I("file name = %s\n", jd_i_CTPM_firmware_name);
#else
	int RetryCnt;

	if ((pjadard_ts_data->esd_fw == NULL) || (pjadard_ts_data->esd_fw_reload)) {
		JD_I("file name = %s\n", file_name);

		pjadard_ts_data->esd_fw_reload = false;
		if (pjadard_ts_data->esd_fw) {
			release_firmware(pjadard_ts_data->esd_fw);
			pjadard_ts_data->esd_fw = NULL;
		}

		for (RetryCnt = 0; RetryCnt < JD_UPGRADE_FW_RETRY_TIME; RetryCnt++) {
			err = request_firmware(&pjadard_ts_data->esd_fw, file_name, pjadard_ts_data->dev);
			if (err < 0) {
				JD_E("%s: Open file fail(ret:%d), RetryCnt = %d\n", __func__, err, RetryCnt);
				mdelay(100);
			} else {
				break;
			}
		}

		if (RetryCnt == JD_UPGRADE_FW_RETRY_TIME) {
			JD_E("%s: Open file fail retry over %d\n", __func__, JD_UPGRADE_FW_RETRY_TIME);
			return JD_FILE_OPEN_FAIL;
		}
	}

#endif
    if (jd_g_f_0f_update) {
        JD_W("%s: Other thread is upgrade now\n", __func__);
        err = JD_UPGRADE_CONFLICT;
    } else {
        JD_I("%s: Entering upgrade Flow\n", __func__);
        jd_g_f_0f_update = true;

#ifdef JD_UPGRADE_FW_ARRAY
        JD_I("FW size = %d\n", (int)jd_fw_size);
        err = g_module_fp.fp_esd_ram_write(0, (uint8_t *)fw_data, jd_fw_size);
#else
		JD_I("FW size = %d\n", (int)pjadard_ts_data->esd_fw->size);
		err = g_module_fp.fp_esd_ram_write(0, (uint8_t *)pjadard_ts_data->esd_fw->data,
									pjadard_ts_data->esd_fw->size);
#endif
		jd_g_f_0f_update = false;

		if ((err >= 0) || (err == JD_PRAM_CRC_PASS))
			pjadard_ts_data->fw_ready = true;
	}

    return err;
}

static int jadard_mcu_esd_ram_write(uint32_t start_addr, uint8_t *wdata, uint32_t wlen)
{
    JD_I("%s: nothing to do, only for function pointer initial\n", __func__);
    return JD_NO_ERR;
}
#endif

static void jadard_mcu_0f_operation(struct work_struct *work)
{
    int err = g_module_fp.fp_0f_upgrade_fw(jd_i_CTPM_firmware_name);

	if (err >= 0) {
		g_module_fp.fp_read_fw_ver();
		jadard_int_enable(true);
#ifdef CONFIG_JD_HID
		queue_delayed_work(pjadard_ts_data->jadard_hid_mode_wq,
			&pjadard_ts_data->work_hid_mode, 0);
#endif
	}
}
#endif

#ifdef CONFIG_TOUCHSCREEN_JADARD_SORTING
static int jadard_mcu_sorting_test(char *json_buf, size_t len)
{
    JD_I("%s: not support ITO test\n", __func__);
    return JD_NO_ERR;
}

static void jadard_mcu_APP_SetSortingMode(uint8_t *value, uint8_t value_len)
{
    JD_I("%s: nothing to do, only for function pointer initial\n", __func__);
}

static void jadard_mcu_APP_ReadSortingMode(uint8_t *pValue, uint8_t pValue_len)
{
    JD_I("%s: nothing to do, only for function pointer initial\n", __func__);
}

static void jadard_mcu_APP_GetLcdSleep(uint8_t *pStatus)
{
    JD_I("%s: nothing to do, only for function pointer initial\n", __func__);
}

static void jadard_mcu_APP_SetSortingSkipFrame(uint8_t value)
{
    JD_I("%s: nothing to do, only for function pointer initial\n", __func__);
}

static void jadard_mcu_APP_SetSortingKeepFrame(uint8_t value)
{
    JD_I("%s: nothing to do, only for function pointer initial\n", __func__);
}

static bool jadard_mcu_APP_ReadSortingBusyStatus(uint8_t mpap_handshake_finish, uint8_t *pStatus)
{
    JD_I("%s: nothing to do, only for function pointer initial\n", __func__);
    return false;
}

static void jadard_mcu_GetSortingDiffData(uint8_t *pValue, uint16_t pValue_len)
{
    JD_I("%s: nothing to do, only for function pointer initial\n", __func__);
}

static void jadard_mcu_GetSortingDiffDataMax(uint8_t *pValue, uint16_t pValue_len)
{
    JD_I("%s: nothing to do, only for function pointer initial\n", __func__);
}

static void jadard_mcu_GetSortingDiffDataMin(uint8_t *pValue, uint16_t pValue_len)
{
    JD_I("%s: nothing to do, only for function pointer initial\n", __func__);
}

static void jadard_mcu_Fw_DBIC_Off(void)
{
    JD_I("%s: nothing to do, only for function pointer initial\n", __func__);
}

static void jadard_mcu_Fw_DBIC_On(void)
{
    JD_I("%s: nothing to do, only for function pointer initial\n", __func__);
}

static void jadard_mcu_StartMCU(void)
{
    JD_I("%s: nothing to do, only for function pointer initial\n", __func__);
}

static void jadard_mcu_SetMpBypassMain(void)
{
    JD_D("%s: nothing to do, only for function pointer initial\n", __func__);
}

static void jadard_mcu_ClearMpBypassMain(void)
{
    JD_D("%s: nothing to do, only for function pointer initial\n", __func__);
}

static void jadard_mcu_ReadMpapErrorMsg(uint8_t *pValue, uint8_t pValue_len)
{
    JD_I("%s: nothing to do, only for function pointer initial\n", __func__);
}
#endif

static bool jadard_mcu_DoneStatusIsHigh(uint8_t usValue, uint8_t DoneStatus)
{
	return ((usValue & DoneStatus) == DoneStatus);
}

static bool jadard_mcu_DoneStatusIsLow(uint8_t usValue, uint8_t BusyStatus, uint8_t DoneStatus)
{
	return ((usValue & BusyStatus) == DoneStatus);
}

#ifndef JD_ZERO_FLASH
#ifdef JD_FLASH_WP_EN
static int jadard_mcu_FlashUnlock(void)
{
	int ReCode;
	uint8_t wdata = (uint8_t)JD_EX_FLASH_ADDR_WP_UNLOCK;

	JD_I("Flash unlock\n");

	/* Write enable */
	ReCode = g_module_fp.fp_ICWriteExFlashFlow((uint8_t)JD_EX_FLASH_ADDR_WRITE_ENABLE, 0, 0, NULL, 0);
	if (ReCode < 0) {
		JD_E("%s: Set write enable fail\n", __func__);
		return ReCode;
	}

	/* Set WP unlock */
	ReCode = g_module_fp.fp_ICWriteExFlashFlow((uint8_t)JD_EX_FLASH_ADDR_WRITE_STATUS_REGISTER1, 0, 0, &wdata, 1);
	if (ReCode < 0)
		JD_E("%s: Set WP unlock fail\n", __func__);

	/* Read flash status1 status*/
	ReCode = g_module_fp.fp_ICReadExFlashFlow((uint8_t)JD_EX_FLASH_ADDR_STATUS_REGISTER1, 0, 0, 0, 0, &wdata, 1);
	JD_I("Flash Status1: 0x%02x\n", wdata);

	return ReCode;
}

static int jadard_mcu_FlashLock(void)
{
	int ReCode;
	uint8_t wdata = (uint8_t)JD_EX_FLASH_ADDR_WP_LOCK;

	JD_I("Flash lock\n");

	/* Write enable */
	ReCode = g_module_fp.fp_ICWriteExFlashFlow((uint8_t)JD_EX_FLASH_ADDR_WRITE_ENABLE, 0, 0, NULL, 0);
	if (ReCode < 0) {
		JD_E("%s: Set write enable fail\n", __func__);
		return ReCode;
	}

	/* Set WP lock */
	ReCode = g_module_fp.fp_ICWriteExFlashFlow((uint8_t)JD_EX_FLASH_ADDR_WRITE_STATUS_REGISTER1, 0, 0, &wdata, 1);
	if (ReCode < 0)
		JD_E("%s: Set WP lock fail\n", __func__);

	/* Read flash status1 status*/
	ReCode = g_module_fp.fp_ICReadExFlashFlow((uint8_t)JD_EX_FLASH_ADDR_STATUS_REGISTER1, 0, 0, 0, 0, &wdata, 1);
	JD_I("Flash Status1: 0x%02x\n", wdata);

	return ReCode;
}
#endif

static int jadard_mcu_SetFlashSPISpeed(uint8_t flash_spi_speed_level)
{
	int ReCode;
	uint8_t wBuf = flash_spi_speed_level & 0x07;

	ReCode = g_module_fp.fp_register_write((uint32_t)JD_FLASH_REG_ADDR_SPI_BAUD_RATE, &wBuf, 1);

	if (ReCode < 0)
		JD_E("%s: Set flash SPI speed fail\n", __func__);

	return ReCode;
}
#endif

static int jadard_mcu_GetFlashSPIFStatus(uint16_t usTimeOut, uint8_t *rdata)
{
	int ReCode;
	uint32_t i = 0;
	uint8_t usValue = (uint8_t)JD_MASTER_SPI_RELATED_SETTING_SPIF_BUSY;
	bool spif_busy_status_done = false;

	do {
		/* Read SPI status until done */
		mdelay(2);
		ReCode = g_module_fp.fp_register_read((uint32_t)JD_FLASH_REG_ADDR_SPI_STATUS, &usValue, 1);

		if (ReCode < 0) {
			JD_E("%s: Read flash SPI status fail\n", __func__);
			return ReCode;
		} else {
			i += 2;
		}

		spif_busy_status_done = g_module_fp.fp_DoneStatusIsHigh(usValue,
															(uint8_t)JD_MASTER_SPI_RELATED_SETTING_SPIF_DONE);

		/* Read SPI data when rdata was not NULL */
		if (rdata != NULL) {
			ReCode = g_module_fp.fp_register_read((uint32_t)JD_FLASH_REG_ADDR_SPI_RDATA, rdata, 1);
			if (ReCode < 0) {
				JD_E("%s: Read SPI RDATA fail\n", __func__);
				return ReCode;
			}
		}
	} while ((i < usTimeOut) && !spif_busy_status_done);

	if (i >= usTimeOut) {
		JD_E("%s: Get Flash SPI status timeout\n", __func__);
		return JD_TIME_OUT;
	}

	return ReCode;
}

static int jadard_mcu_ICSetExFlashCSNOutDisable(void)
{
	uint8_t wBuf = JD_MASTER_SPI_RELATED_SETTING_CSN_H;

	return g_module_fp.fp_register_write((uint32_t)JD_FLASH_REG_ADDR_SPI_CSN_OUT, &wBuf, 1);
}

static int jadard_mcu_ICSetExFlashCSNOutEnable(void)
{
	uint8_t wBuf = JD_MASTER_SPI_RELATED_SETTING_CSN_L;

	return g_module_fp.fp_register_write((uint32_t)JD_FLASH_REG_ADDR_SPI_CSN_OUT, &wBuf, 1);
}

static int jadard_mcu_ICWriteExFlashFlow(uint8_t cmd, uint32_t addr, uint16_t addr_len, uint8_t *wdata, uint16_t wdata_len)
{
	int ReCode, i;
	uint8_t wBuf[JD_THREE_SIZE];
	uint8_t dummy;

	/* Set SPI frequency to 12MHz */
	wBuf[0] = (uint8_t)JD_MASTER_SPI_RELATED_SETTING_SPEED_12MHz;

	ReCode = g_module_fp.fp_register_write((uint32_t)JD_FLASH_REG_ADDR_SPI_BAUD_RATE, wBuf, 1);
	if (ReCode < 0) {
		JD_E("%s: Set SPI frequency to 12MHz fail\n", __func__);
		return ReCode;
	}

	wBuf[0] = (uint8_t)JD_MASTER_SPI_RELATED_SETTING_CSN_L; /* Set CSN out enable [CSN = L] */
	wBuf[1] = 0x01; /* set SPI enable */
	wBuf[2] = cmd; /* set Flash command */

	ReCode = g_module_fp.fp_register_write((uint32_t)JD_FLASH_REG_ADDR_SPI_CSN_OUT, wBuf, sizeof(wBuf));
	if (ReCode < 0) {
		JD_E("%s: SPI setting fail\n", __func__);
		return ReCode;
	}

	/* Read SPI status until done */
	ReCode = g_module_fp.fp_GetFlashSPIFStatus(50, &dummy);
	if (ReCode < 0) {
		JD_E("%s: Get flash SPI status fail\n", __func__);
		return ReCode;
	}

	for (i = 0; i < addr_len; i++) {
		wBuf[0] = (uint8_t)(addr >> ((addr_len - i - 1) * 8));
		ReCode = g_module_fp.fp_register_write((uint32_t)JD_FLASH_REG_ADDR_SPI_WDATA, wBuf, 1);
		if (ReCode < 0) {
			JD_E("%s: Set flash SPI addrress fail\n", __func__);
			return ReCode;
		}

		ReCode = g_module_fp.fp_GetFlashSPIFStatus(50, &dummy);
		if (ReCode < 0) {
			JD_E("%s: Get flash SPI status fail\n", __func__);
			return ReCode;
		}
	}

	for (i = 0; i < wdata_len; i++) {
		ReCode = g_module_fp.fp_register_write((uint32_t)JD_FLASH_REG_ADDR_SPI_WDATA, wdata+i, 1);
		if (ReCode < 0) {
			JD_E("%s: Set write data fail\n", __func__);
			return ReCode;
		}

		ReCode = g_module_fp.fp_GetFlashSPIFStatus(50, &dummy);
		if (ReCode < 0) {
			JD_E("%s: Set write data SPI status fail\n", __func__);
			return ReCode;
		}
	}

	/* Set CSN out disable [CSN = H] */
	wBuf[0] = (uint8_t)JD_MASTER_SPI_RELATED_SETTING_CSN_H;
	ReCode = g_module_fp.fp_register_write((uint32_t)JD_FLASH_REG_ADDR_SPI_CSN_OUT, wBuf, 1);
	if (ReCode < 0)
		JD_E("%s: Set CSN high fail\n", __func__);

	return ReCode;
}

#ifndef JD_ZERO_FLASH
static int jadard_mcu_ICReadExFlashFlow(uint8_t cmd, uint32_t addr, uint16_t addr_len,
									  uint8_t dummy_par, uint16_t dummy_par_len, uint8_t *rdata, uint16_t rdata_len)
{
	int ReCode, i;
	uint8_t wBuf[JD_THREE_SIZE];
	uint8_t dummy;

	/* Set SPI frequency to 12MHz */
	wBuf[0] = (uint8_t)JD_MASTER_SPI_RELATED_SETTING_SPEED_12MHz;

	ReCode = g_module_fp.fp_register_write((uint32_t)JD_FLASH_REG_ADDR_SPI_BAUD_RATE, wBuf, 1);
	if (ReCode < 0) {
		JD_E("%s: Set SPI frequency to 12MHz fail\n", __func__);
		return ReCode;
	}

	/* Set CSN out enable [CSN = L] */
	wBuf[0] = (uint8_t)JD_MASTER_SPI_RELATED_SETTING_CSN_L;
	wBuf[1] = 0x01; /* set SPI enable */
	wBuf[2] = cmd; /* set Flash command */

	ReCode = g_module_fp.fp_register_write((uint32_t)JD_FLASH_REG_ADDR_SPI_CSN_OUT, wBuf, sizeof(wBuf));
	if (ReCode < 0) {
		JD_E("%s: SPI setting fail\n", __func__);
		return ReCode;
	}

	/* Read SPI status until done */
	ReCode = g_module_fp.fp_GetFlashSPIFStatus(50, &dummy);
	if (ReCode < 0) {
		JD_E("%s: Get flash SPI status fail\n", __func__);
		return ReCode;
	}

	for (i = 0; i < addr_len; i++) {
		wBuf[0] = (uint8_t)(addr >> ((addr_len - i - 1) * 8));
		ReCode = g_module_fp.fp_register_write((uint32_t)JD_FLASH_REG_ADDR_SPI_WDATA, wBuf, 1);
		if (ReCode < 0) {
			JD_E("%s(%d): Set flash SPI addrress fail\n", __func__, __LINE__);
			return ReCode;
		}

		ReCode = g_module_fp.fp_GetFlashSPIFStatus(50, &dummy);
		if (ReCode < 0) {
			JD_E("%s(%d): Get flash SPI status fail\n", __func__, __LINE__);
			return ReCode;
		}
	}

	for (i = 0; i < dummy_par_len; i++) {
		ReCode = g_module_fp.fp_register_write((uint32_t)JD_FLASH_REG_ADDR_SPI_WDATA, &dummy_par, 1);
		if (ReCode < 0) {
			JD_E("%s(%d): Set flash SPI addrress fail\n", __func__, __LINE__);
			return ReCode;
		}

		ReCode = g_module_fp.fp_GetFlashSPIFStatus(50, &dummy);
		if (ReCode < 0) {
			JD_E("%s(%d): Get flash SPI status fail\n", __func__, __LINE__);
			return ReCode;
		}
	}

	for (i = 0; i < rdata_len; i++) {
		/* set Dummy data */
		wBuf[0] = (uint8_t)(i + 1);
		ReCode = g_module_fp.fp_register_write((uint32_t)JD_FLASH_REG_ADDR_SPI_WDATA, wBuf, 1);
		if (ReCode < 0) {
			JD_E("%s(%d): Set flash SPI addrress fail\n", __func__, __LINE__);
			return ReCode;
		}

		/* Read SPI status until done & read SPI data */
		ReCode = g_module_fp.fp_GetFlashSPIFStatus(50, rdata + i);
		if (ReCode < 0) {
			JD_E("%s(%d): Get flash SPI status fail\n", __func__, __LINE__);
			return ReCode;
		}
	}

	/* Set CSN out disable [CSN = H] */
	wBuf[0] = (uint8_t)JD_MASTER_SPI_RELATED_SETTING_CSN_H;
	ReCode = g_module_fp.fp_register_write((uint32_t)JD_FLASH_REG_ADDR_SPI_CSN_OUT, wBuf, 1);
	if (ReCode < 0)
		JD_E("%s: Set CSN high fail\n", __func__);

	return ReCode;
}

static int jadard_mcu_ICGetExFlashStatus(uint16_t usTimeOut)
{
	int ReCode;
	uint32_t i = 0;
	uint8_t usValue = (uint8_t)JD_MASTER_SPI_RELATED_SETTING_FLASH_BUSY;
	uint8_t wBuf;
	bool flash_busy_status_done = false;

	do {
		/* Set CSN out enable [CSN = L] */
		wBuf = (uint8_t)JD_MASTER_SPI_RELATED_SETTING_CSN_L;
		ReCode = g_module_fp.fp_register_write((uint32_t)JD_FLASH_REG_ADDR_SPI_CSN_OUT, &wBuf, 1);
		if (ReCode < 0) {
			JD_E("%s: Set CSN low fail\n", __func__);
			return ReCode;
		}

		/* Set read flash status start */
		wBuf = (uint8_t)JD_DMA_RELATED_SETTING_READ_FLASH_STATUS;
		ReCode = g_module_fp.fp_register_write((uint32_t)JD_FLASH_REG_ADDR_DMA_2WFLASHEN_1WRITE_0START, &wBuf, 1);
		if (ReCode < 0) {
			JD_E("%s: Set read flash status start fail\n", __func__);
			return ReCode;
		}

		/* Wait Flash Status Register 1 ready */
		mdelay(5);

		/* Set CSN out disable [CSN = H] */
		wBuf = (uint8_t)JD_MASTER_SPI_RELATED_SETTING_CSN_H;
		ReCode = g_module_fp.fp_register_write((uint32_t)JD_FLASH_REG_ADDR_SPI_CSN_OUT, &wBuf, 1);
		if (ReCode < 0) {
			JD_E("%s: Set CSN high fail\n", __func__);
			return ReCode;
		}

		/* Read Ex-Flash status until done */
		ReCode = g_module_fp.fp_register_read((uint32_t)JD_FLASH_REG_ADDR_DMA_BUSY_OR_START, &usValue, 1);
		if (ReCode < 0) {
			JD_E("%s: Read Ex-Flash status fail\n", __func__);
			return ReCode;
		} else {
			i += 5;
		}

		flash_busy_status_done = g_module_fp.fp_DoneStatusIsLow(usValue,
															(uint8_t)JD_MASTER_SPI_RELATED_SETTING_FLASH_BUSY,
															(uint8_t)JD_MASTER_SPI_RELATED_SETTING_FLASH_DONE);
	} while ((i < usTimeOut) && !flash_busy_status_done);

	if (i >= usTimeOut) {
		JD_E("%s: Get Ex-Flash status timeout\n", __func__);
		return JD_TIME_OUT;
	}

	return ReCode;
}

static int jadard_mcu_EraseSector(uint32_t addr)
{
	int ReCode;

	/* Write enable */
	ReCode = g_module_fp.fp_ICWriteExFlashFlow((uint8_t)JD_EX_FLASH_ADDR_WRITE_ENABLE, 0, 0, NULL, 0);
	if (ReCode < 0) {
		JD_E("%s: Set write enable fail\n", __func__);
		return ReCode;
	}

	/* Erase cmd */
	ReCode = g_module_fp.fp_ICWriteExFlashFlow((uint8_t)JD_EX_FLASH_ADDR_ERASE_SECTOR, addr, 3, NULL, 0);
	if (ReCode < 0) {
		JD_E("%s: Set erase cmd fail\n", __func__);
		return ReCode;
	}

	msleep((uint16_t)JD_FLASH_RUN_TIME_ERASE_SECTOR_TIME);

	/* Read Ex-Flash status until done */
	ReCode = g_module_fp.fp_ICGetExFlashStatus(50);
	if (ReCode < 0)
		JD_E("%s: Read Ex-Flash status fail\n", __func__);

	return ReCode;
}

static int jadard_mcu_EraseBlock_32K(uint32_t addr)
{
	int ReCode;

	/* Write enable */
	ReCode = g_module_fp.fp_ICWriteExFlashFlow((uint8_t)JD_EX_FLASH_ADDR_WRITE_ENABLE, 0, 0, NULL, 0);
	if (ReCode < 0) {
		JD_E("%s: Set write enable fail\n", __func__);
		return ReCode;
	}

	/* Erase cmd */
	ReCode = g_module_fp.fp_ICWriteExFlashFlow((uint8_t)JD_EX_FLASH_ADDR_ERASE_BLOCK_32K, addr, 3, NULL, 0);
	if (ReCode < 0) {
		JD_E("%s: Set erase 32k cmd fail\n", __func__);
		return ReCode;
	}

	msleep((uint16_t)JD_FLASH_RUN_TIME_ERASE_BLOCK_32K_TIME);

	/* Read Ex-Flash status until done */
	ReCode = g_module_fp.fp_ICGetExFlashStatus(50);
	if (ReCode < 0)
		JD_E("%s: Read Ex-Flash status fail\n", __func__);

	return ReCode;
}

static int jadard_mcu_EraseBlock_64K(uint32_t addr)
{
	int ReCode;

	/* Write enable */
	ReCode = g_module_fp.fp_ICWriteExFlashFlow((uint8_t)JD_EX_FLASH_ADDR_WRITE_ENABLE, 0, 0, NULL, 0);
	if (ReCode < 0) {
		JD_E("%s: Set write enable fail\n", __func__);
		return ReCode;
	}

	/* Erase cmd */
	ReCode = g_module_fp.fp_ICWriteExFlashFlow((uint8_t)JD_EX_FLASH_ADDR_ERASE_BLOCK_64K, addr, 3, NULL, 0);
	if (ReCode < 0) {
		JD_E("%s: Set erase 64k cmd fail\n", __func__);
		return ReCode;
	}

	msleep((uint16_t)JD_FLASH_RUN_TIME_ERASE_BLOCK_64K_TIME);

	/* Read Ex-Flash status until done */
	ReCode = g_module_fp.fp_ICGetExFlashStatus(50);
	if (ReCode < 0)
		JD_E("%s: Read Ex-Flash status fail\n", __func__);

	return ReCode;
}

static int jadard_mcu_EraseChip(void)
{
	int ReCode;

	/* Write enable */
	ReCode = g_module_fp.fp_ICWriteExFlashFlow((uint8_t)JD_EX_FLASH_ADDR_WRITE_ENABLE, 0, 0, NULL, 0);
	if (ReCode < 0) {
		JD_E("%s: Set write enable fail\n", __func__);
		return ReCode;
	}

	/* Erase cmd */
	ReCode = g_module_fp.fp_ICWriteExFlashFlow((uint8_t)JD_EX_FLASH_ADDR_ERASE_CHIP, 0, 0, NULL, 0);
	if (ReCode < 0) {
		JD_E("%s: Set erase chip cmd fail\n", __func__);
		return ReCode;
	}

	msleep((uint16_t)JD_FLASH_RUN_TIME_ERASE_CHIP_TIME);

	/* Read Ex-Flash status until done */
	ReCode = g_module_fp.fp_ICGetExFlashStatus(500);
	if (ReCode < 0)
		JD_E("%s: Read Ex-Flash status fail\n", __func__);

	return ReCode;
}

static int jadard_mcu_EraseChipFlash(uint32_t addr, uint32_t len)
{
	int ReCode, i;
	int sector_start = 0;
	int sector_end = 0;

	if ((addr + len) > (JD_SIZE_DEF_BLOCK_64K * 2)) {
		ReCode = g_module_fp.fp_flash_erase();
		if (ReCode < 0) {
			JD_E("%s: Erase flash timeout error\n", __func__);
			return ReCode;
		}
	} else {
		sector_start = (int)(addr >> 12);

		if (((addr + len) & 0xFFF) == 0)
			sector_end = (int)((addr + len) >> 12);
		else
			sector_end = (int)((addr + len) >> 12) + 1;

		for (i = sector_start; i < sector_end;/* Acc in loop */) {
			if ((i == 0 || i == 16) && (sector_end - i) >= 16) {
				/* 64KB BLOCK ERASE */
				ReCode = g_module_fp.fp_EraseBlock_64K((uint32_t)(i << 12));
				if (ReCode < 0) {
					JD_E("%s: Erase 64KB block timeout error\n", __func__);
					return ReCode;
				} else {
					i += 16;
					JD_D("%s: EraseStart %08x\n", __func__, (uint32_t)(i << 12));
					JD_D("%s: Size 64KB\n", __func__);
					JD_D("%s: Erase 64KB block finish\n", __func__);
				}
			} else if ((i == 0 || i == 8 || i == 16 || i == 24) && (sector_end - i) >= 8) {
				/* 32KB BLOCK ERASE */
				ReCode = g_module_fp.fp_EraseBlock_32K((uint32_t)(i << 12));
				if (ReCode < 0) {
					JD_E("%s: Erase 32KB block timeout error\n", __func__);
					return ReCode;
				} else {
					i += 8;
					JD_D("%s: EraseStart %08x\n", __func__, (uint32_t)(i << 12));
					JD_D("%s: Size 32KB\n", __func__);
					JD_D("%s: Erase 32KB block finish\n", __func__);
				}
			} else {
				/* SECTOR ERASE */
				ReCode = g_module_fp.fp_EraseSector((uint32_t)(i << 12));
				if (ReCode < 0) {
					JD_E("%s: Erase sector timeout error\n", __func__);
					return ReCode;
				} else {
					i++;
					JD_D("%s: EraseStart %08x\n", __func__, (uint32_t)(i << 12));
					JD_D("%s: Size 4KB\n", __func__);
					JD_D("%s: Erase KB block finish\n", __func__);
				}
			}
		}
	}

	return ReCode;
}
#endif

static int jadard_mcu_SetCRCInitial(void)
{
	uint8_t wBuf[JD_ONE_SIZE];

	wBuf[0] = 0x01;
	return g_module_fp.fp_register_write((uint32_t)JD_FLASH_REG_ADDR_CRC_INIT, wBuf, sizeof(wBuf));
}

static int jadard_mcu_SetCRCInitialValue(void)
{
	uint8_t wBuf[JD_TWO_SIZE];

	wBuf[(uint32_t)JD_CRC_CODE_POSITION_HIGH_BYTE] =
		(uint8_t)((JD_CRC_INFO_CRC_INITIAL_VALUE & 0xFF00) >> 8);
	wBuf[(uint32_t)JD_CRC_CODE_POSITION_LOW_BYTE] =
		(uint8_t)((JD_CRC_INFO_CRC_INITIAL_VALUE & 0x00FF) >> 0);

	return g_module_fp.fp_register_write((uint32_t)JD_FLASH_REG_ADDR_CRC_INIT_CODE0, wBuf, sizeof(wBuf));
}

static int jadard_mcu_GetCRCResult(uint16_t *crc)
{
	int ReCode;
	uint8_t rBuf[JD_TWO_SIZE];

	ReCode = g_module_fp.fp_register_read((uint32_t)JD_FLASH_REG_ADDR_CRC_CHK0, rBuf, sizeof(rBuf));
	if (ReCode < 0) {
		*crc = 0;
	} else {
		*crc = (uint16_t)((rBuf[(uint32_t)JD_CRC_CODE_POSITION_HIGH_BYTE] << 8) +
						   rBuf[(uint32_t)JD_CRC_CODE_POSITION_LOW_BYTE]);
	}

	return ReCode;
}

#ifndef JD_ZERO_FLASH
static int jadard_mcu_SetCRCEnable(bool enable)
{
	uint8_t wBuf[JD_ONE_SIZE];

	wBuf[0] = enable;
	return g_module_fp.fp_register_write((uint32_t)JD_FLASH_REG_ADDR_CRC_ENABLE, wBuf, sizeof(wBuf));
}

static uint32_t jadard_mcu_Pow(uint8_t base, uint32_t pow)
{
	uint32_t i;
	uint32_t sum = 1;

	if (pow > 0) {
		for (i = 0; i < pow; i++)
			sum *= base;
	}

	return sum;
}

static int jadard_mcu_JEDEC_ID(uint8_t *id)
{
	int ReCode;
	uint8_t rdbuf[JD_THREE_SIZE];

	ReCode = g_module_fp.fp_ICReadExFlashFlow((uint8_t)JD_EX_FLASH_ADDR_READ_ID, 0, 0, 0, 0, rdbuf, sizeof(rdbuf));
	if (ReCode < 0) {
		JD_E("%s: Read external flash fail\n", __func__);
		return ReCode;
	}

	if (id != NULL)
		*id = rdbuf[2];

	JD_I("Flash RDID: 0x%02x%02x%02x\n", rdbuf[0], rdbuf[1], rdbuf[2]);

	if ((rdbuf[0] == 0x00) || (rdbuf[0] == 0xFF))
		JD_E("%s: Flash was not exist\n", __func__);
	else
		JD_I("Flash capacity: %d KB\n", g_module_fp.fp_Pow(2, rdbuf[2] - 10));

	/* Read flash status1 status*/
	ReCode = g_module_fp.fp_ICReadExFlashFlow((uint8_t)JD_EX_FLASH_ADDR_STATUS_REGISTER1, 0, 0, 0, 0, rdbuf, 2);
	if (ReCode < 0) {
		JD_E("%s: Read external flash status1 fail\n", __func__);
		return ReCode;
	}

	JD_I("Flash Status1: 0x%02x ,0x%02x\n", rdbuf[0], rdbuf[1]);

	/* Read flash status2 status*/
	ReCode = g_module_fp.fp_ICReadExFlashFlow((uint8_t)JD_EX_FLASH_ADDR_STATUS_REGISTER2, 0, 0, 0, 0, rdbuf, 2);
	if (ReCode < 0) {
		JD_E("%s: Read external flash status2 fail\n", __func__);
		return ReCode;
	}

	JD_I("Flash Status2: 0x%02x ,0x%02x\n", rdbuf[0], rdbuf[1]);

	return ReCode;
}

static int jadard_mcu_CheckSize(uint8_t data, uint32_t length)
{
	int ReCode = JD_NO_ERR;

	if (g_module_fp.fp_Pow(2, data) < length) {
		ReCode = JD_WRITE_OVERFLOW;
		JD_E("%s: Write size was overflow\n", __func__);
	}

	return ReCode;
}
#endif

static int jadard_mcu_SetPramCksumEn(bool enable)
{
	uint8_t value;

	if (enable)
		value = JD_MASTER_SPI_RELATED_SETTING_PRAM_CKSUM_EN_ENABLE;
	else
		value = JD_MASTER_SPI_RELATED_SETTING_PRAM_CKSUM_EN_DISABLE;

	return g_module_fp.fp_register_write((uint32_t)JD_FLASH_REG_ADDR_PRAM_CKSUM_EN, &value, 1);
}

static void jadard_mcu_fp_init(void)
{
    g_module_fp.fp_register_read               = jadard_mcu_register_read;
    g_module_fp.fp_register_write              = jadard_mcu_register_write;
    g_module_fp.fp_dd_register_read            = jadard_mcu_dd_register_read;
    g_module_fp.fp_dd_register_write           = jadard_mcu_dd_register_write;
    g_module_fp.fp_set_sleep_mode              = jadard_mcu_set_sleep_mode;
    g_module_fp.fp_read_fw_ver                 = jadard_mcu_read_fw_ver;
    g_module_fp.fp_mutual_data_set             = jadard_mcu_mutual_data_set;
    g_module_fp.fp_get_mutual_data             = jadard_mcu_get_mutual_data;
    g_module_fp.fp_get_touch_data              = jadard_mcu_get_touch_data;
    g_module_fp.fp_read_mutual_data            = jadard_mcu_read_mutual_data;
    g_module_fp.fp_report_points               = jadard_mcu_report_points;
    g_module_fp.fp_parse_report_data           = jadard_mcu_parse_report_data;
    g_module_fp.fp_distribute_touch_data       = jadard_mcu_distribute_touch_data;
#ifndef JD_ZERO_FLASH
	g_module_fp.fp_flash_read				   = jadard_mcu_flash_read;
	g_module_fp.fp_flash_write				   = jadard_mcu_flash_write;
	g_module_fp.fp_flash_erase				   = jadard_mcu_EraseChip;
#endif
    g_module_fp.fp_EnterBackDoor               = jadard_mcu_EnterBackDoor;
    g_module_fp.fp_ExitBackDoor                = jadard_mcu_ExitBackDoor;
    g_module_fp.fp_pin_reset                   = jadard_mcu_PinReset;
    g_module_fp.fp_soft_reset                  = jadard_mcu_SoftReset;
    g_module_fp.fp_ResetMCU                    = jadard_mcu_ResetMCU;
    g_module_fp.fp_PorInit                     = jadard_mcu_PorInit;
#ifdef JD_RST_PIN_FUNC
    g_module_fp.fp_ic_reset                    = jadard_mcu_ic_reset;
#endif

    g_module_fp.fp_ic_soft_reset               = jadard_mcu_ic_soft_reset;
    g_module_fp.fp_touch_info_set              = jadard_mcu_touch_info_set;
    g_module_fp.fp_report_data_reinit          = jadard_mcu_report_data_reinit;
    g_module_fp.fp_usb_detect_set              = jadard_mcu_usb_detect_set;
    g_module_fp.fp_get_freq_band               = jadard_mcu_get_freq_band;
    g_module_fp.fp_ReadDbicPageEn              = jadard_mcu_ReadDbicPageEn;
    g_module_fp.fp_SetDbicPage                 = jadard_mcu_SetDbicPage;
    g_module_fp.fp_log_touch_state             = jadard_mcu_log_touch_state;
#if defined(JD_SMART_WAKEUP) || defined(JD_USB_DETECT_GLOBAL) || defined(JD_USB_DETECT_CALLBACK) ||\
    defined(JD_HIGH_SENSITIVITY) || defined(JD_ROTATE_BORDER) || defined(JD_EARPHONE_DETECT)
    g_module_fp.fp_resume_set_func             = jadard_mcu_resume_set_func;
#endif
#ifdef JD_HIGH_SENSITIVITY
	g_module_fp.fp_set_high_sensitivity		   = jadard_mcu_set_high_sensitivity;
#endif
#ifdef JD_ROTATE_BORDER
	g_module_fp.fp_set_rotate_border		   = jadard_mcu_set_rotate_border;
#endif
#ifdef JD_EARPHONE_DETECT
    g_module_fp.fp_set_earphone_enable         = jadard_mcu_set_earphone_enable;
#endif
	g_module_fp.fp_set_SMWP_enable			   = jadard_mcu_set_SMWP_enable;
#if (JD_PROXIMITY_MODE != JD_PROXIMITY_DISABLE)
	g_module_fp.fp_set_virtual_proximity	   = jadard_mcu_set_virtual_proximity;
#endif
#ifdef JD_AUTO_UPGRADE_FW
    g_module_fp.fp_read_fw_ver_bin             = jadard_mcu_read_fw_ver_bin;
#endif

    g_module_fp.fp_ram_read                    = jadard_mcu_ram_read;
#ifdef JD_ZERO_FLASH
    g_module_fp.fp_ram_write                   = jadard_mcu_ram_write;
    g_module_fp.fp_0f_upgrade_fw               = jadard_mcu_0f_upgrade_fw;
#ifdef JD_ESD_CHECK
    g_module_fp.fp_0f_esd_upgrade_fw           = jadard_mcu_0f_esd_upgrade_fw;
    g_module_fp.fp_esd_ram_write               = jadard_mcu_esd_ram_write;
#endif
    g_module_fp.fp_0f_operation                = jadard_mcu_0f_operation;
#endif

#ifdef CONFIG_TOUCHSCREEN_JADARD_SORTING
	g_module_fp.fp_sorting_test				   = jadard_mcu_sorting_test;
	g_module_fp.fp_APP_SetSortingMode		   = jadard_mcu_APP_SetSortingMode;
	g_module_fp.fp_APP_ReadSortingMode		   = jadard_mcu_APP_ReadSortingMode;
	g_module_fp.fp_APP_GetLcdSleep			   = jadard_mcu_APP_GetLcdSleep;
	g_module_fp.fp_APP_SetSortingSkipFrame	   = jadard_mcu_APP_SetSortingSkipFrame;
	g_module_fp.fp_APP_SetSortingKeepFrame	   = jadard_mcu_APP_SetSortingKeepFrame;
	g_module_fp.fp_APP_ReadSortingBusyStatus   = jadard_mcu_APP_ReadSortingBusyStatus;
	g_module_fp.fp_GetSortingDiffData		   = jadard_mcu_GetSortingDiffData;
	g_module_fp.fp_GetSortingDiffDataMax	   = jadard_mcu_GetSortingDiffDataMax;
	g_module_fp.fp_GetSortingDiffDataMin	   = jadard_mcu_GetSortingDiffDataMin;
	g_module_fp.fp_Fw_DBIC_Off				   = jadard_mcu_Fw_DBIC_Off;
	g_module_fp.fp_Fw_DBIC_On				   = jadard_mcu_Fw_DBIC_On;
	g_module_fp.fp_StartMCU					   = jadard_mcu_StartMCU;
	g_module_fp.fp_SetMpBypassMain			   = jadard_mcu_SetMpBypassMain;
	g_module_fp.fp_ClearMpBypassMain		   = jadard_mcu_ClearMpBypassMain;
	g_module_fp.fp_ReadMpapErrorMsg			   = jadard_mcu_ReadMpapErrorMsg;
#endif
	g_module_fp.fp_DoneStatusIsHigh			   = jadard_mcu_DoneStatusIsHigh;
	g_module_fp.fp_DoneStatusIsLow			   = jadard_mcu_DoneStatusIsLow;
#ifndef JD_ZERO_FLASH
#ifdef JD_FLASH_WP_EN
	g_module_fp.fp_FlashUnlock				   = jadard_mcu_FlashUnlock;
	g_module_fp.fp_FlashLock				   = jadard_mcu_FlashLock;
#endif
	g_module_fp.fp_SetFlashSPISpeed			   = jadard_mcu_SetFlashSPISpeed;
	g_module_fp.fp_ICReadExFlashFlow		   = jadard_mcu_ICReadExFlashFlow;
	g_module_fp.fp_ICGetExFlashStatus		   = jadard_mcu_ICGetExFlashStatus;
	g_module_fp.fp_EraseSector				   = jadard_mcu_EraseSector;
	g_module_fp.fp_EraseBlock_32K			   = jadard_mcu_EraseBlock_32K;
	g_module_fp.fp_EraseBlock_64K			   = jadard_mcu_EraseBlock_64K;
	g_module_fp.fp_EraseChipFlash			   = jadard_mcu_EraseChipFlash;
	g_module_fp.fp_SetCRCEnable				   = jadard_mcu_SetCRCEnable;
	g_module_fp.fp_Pow						   = jadard_mcu_Pow;
	g_module_fp.fp_JEDEC_ID					   = jadard_mcu_JEDEC_ID;
	g_module_fp.fp_CheckSize				   = jadard_mcu_CheckSize;
#endif
	g_module_fp.fp_GetFlashSPIFStatus		   = jadard_mcu_GetFlashSPIFStatus;
	g_module_fp.fp_ICSetExFlashCSNOutDisable   = jadard_mcu_ICSetExFlashCSNOutDisable;
	g_module_fp.fp_ICSetExFlashCSNOutEnable	   = jadard_mcu_ICSetExFlashCSNOutEnable;
	g_module_fp.fp_ICWriteExFlashFlow		   = jadard_mcu_ICWriteExFlashFlow;
	g_module_fp.fp_SetCRCInitial			   = jadard_mcu_SetCRCInitial;
	g_module_fp.fp_SetCRCInitialValue		   = jadard_mcu_SetCRCInitialValue;
	g_module_fp.fp_GetCRCResult				   = jadard_mcu_GetCRCResult;
	g_module_fp.fp_SetPramCksumEn			   = jadard_mcu_SetPramCksumEn;
}

void jadard_mcu_cmd_struct_init(void)
{
    JD_D("%s: Entering!\n", __func__);

    jadard_mcu_fp_init();
}
