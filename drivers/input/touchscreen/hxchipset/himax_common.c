/* SPDX-License-Identifier: GPL-2.0 */
/*  Himax Android Driver Sample Code for common functions
 *
 *  Copyright (C) 2024 Himax Corporation.
 *
 *  This software is licensed under the terms of the GNU General Public
 *  License version 2,  as published by the Free Software Foundation,  and
 *  may be copied,  distributed,  and modified under those terms.
 *
 *  This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 */
#include "himax_platform.h"
#include "himax_common.h"
#include "himax_ic_core.h"
#include "himax_modular.h"

#if defined(HX_SMART_WAKEUP)
#define GEST_SUP_NUM 28
/* Setting cust key define (DF = double finger) */
/* {Double Tap, Up, Down, Left, Right, C, Z, M,
 *	O, S, V, W, e, m, @, (reserve),
 *	Finger gesture, ^, >, <, f(R), f(L), Up(DF), Down(DF),
 *	Left(DF), Right(DF)}
 *  Pen single click, Pen double click
 */
uint8_t gest_event[GEST_SUP_NUM] = {
	0x80, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
	0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
	0x81, 0x1D, 0x2D, 0x3D, 0x1F, 0x2F, 0x51, 0x52,
	0x53, 0x54, 0xA0, 0xA1};

/*gest_event mapping to gest_key_def*/
uint16_t gest_key_def[GEST_SUP_NUM] = {
	HX_KEY_DOUBLE_CLICK, HX_KEY_UP, HX_KEY_DOWN, HX_KEY_LEFT,
	HX_KEY_RIGHT,	HX_KEY_C, HX_KEY_Z, HX_KEY_M,
	HX_KEY_O, HX_KEY_S, HX_KEY_V, HX_KEY_W,
	HX_KEY_E, HX_KEY_LC_M, HX_KEY_AT, HX_KEY_RESERVE,
	HX_KEY_FINGER_GEST,	HX_KEY_V_DOWN, HX_KEY_V_LEFT, HX_KEY_V_RIGHT,
	HX_KEY_F_RIGHT,	HX_KEY_F_LEFT, HX_KEY_DF_UP, HX_KEY_DF_DOWN,
	HX_KEY_DF_LEFT,	HX_KEY_DF_RIGHT,
	HX_KEY_PEN_SINGLE_CLICK, HX_KEY_PEN_DOUBLE_CLICK};

uint8_t *wake_event_buffer;
#endif

struct frame_ring_buf g_rb_frame;
#define SUPPORT_FINGER_DATA_CHECKSUM 0x0F
#define TS_WAKE_LOCK_TIMEOUT		(5000)
#define FRAME_COUNT 5

#if defined(HX_TP_PROC_GUEST_INFO)
struct hx_guest_info *g_guest_info_data;
EXPORT_SYMBOL(g_guest_info_data);

char *g_guest_info_item[] = {
	"projectID",
	"CGColor",
	"BarCode",
	"Reserve1",
	"Reserve2",
	"Reserve3",
	"Reserve4",
	"Reserve5",
	"VCOM",
	"Vcom-3Gar",
	NULL
};
#endif

#if defined(HX_BOOT_UPGRADE) || defined(HX_ZERO_FLASH)
bool g_boot_upgrade_flag;
const struct firmware *hxfw;
int g_i_FW_VER;
int g_i_CFG_VER;
int g_i_CID_MAJ; /*GUEST ID*/
int g_i_CID_MIN; /*VER for GUEST*/
#if defined(HX_ZERO_FLASH)
int g_f_0f_updat;
uint8_t *g_update_cfg_buf;
EXPORT_SYMBOL(g_update_cfg_buf);
#endif

#endif

#if defined(HX_ZERO_FLASH)
struct zf_opt_crc g_zf_opt_crc;
EXPORT_SYMBOL(g_zf_opt_crc);
#endif

struct himax_ts_data *hx_s_ts;
EXPORT_SYMBOL(hx_s_ts);

struct himax_ic_data *hx_s_ic_data;
EXPORT_SYMBOL(hx_s_ic_data);

struct himax_report_data *hx_s_touch_data;
EXPORT_SYMBOL(hx_s_touch_data);

struct himax_core_fp hx_s_core_fp;
EXPORT_SYMBOL(hx_s_core_fp);

struct hx_fwbin_info *hx_s_fwbin;
EXPORT_SYMBOL(hx_s_fwbin);

struct himax_debug *hx_s_debug_data;
EXPORT_SYMBOL(hx_s_debug_data);

struct proc_dir_entry *hx_touch_proc_dir;
EXPORT_SYMBOL(hx_touch_proc_dir);

struct ic_setup_collect hx_s_ic_setup;
EXPORT_SYMBOL(hx_s_ic_setup);

int g_mmi_refcnt;
EXPORT_SYMBOL(g_mmi_refcnt);

#if defined(HX_FIRMWARE_HEADER)
int32_t g_hx_panel_id;
#endif

/*ts_work about start*/
struct himax_target_report_data *g_target_report_data;
EXPORT_SYMBOL(g_target_report_data);

/*ts_work about end*/


#if defined(HX_EXCP_RECOVERY)
u8 HX_EXCP_RESET_ACTIVATE;
EXPORT_SYMBOL(HX_EXCP_RESET_ACTIVATE);
#endif

static bool chip_test_r_flag;
u8 g_hw_rst_activate;

static uint8_t AA_press;
static uint8_t EN_NoiseFilter;
static uint8_t Last_EN_NoiseFilter;

static int p_point_num = 0xFFFF;
static uint8_t p_stylus_num = 0xFF;
static int probe_fail_flag;
#if defined(HX_USB_DETECT_GLOBAL)
bool USB_detect_flag;
#endif

#if defined(HX_GESTURE_TRACK)
static int gest_pt_cnt;
static int gest_pt_x[GEST_PT_MAX_NUM];
static int gest_pt_y[GEST_PT_MAX_NUM];
static int gest_start_x, gest_start_y, gest_end_x, gest_end_y;
static int gest_width, gest_height, gest_mid_x, gest_mid_y;
static int hx_gesture_coor[16];
#endif

int g_ts_dbg;
EXPORT_SYMBOL(g_ts_dbg);

/* File node for Selftest, SMWP and HSEN - Start*/
#define HX_PROC_TOUCH_FOLDER "android_touch"
#define HX_PROC_SELF_TEST_FILE	"self_test"
struct proc_dir_entry *proc_self_test_file;

uint8_t HX_PROC_SEND_FLAG;
EXPORT_SYMBOL(HX_PROC_SEND_FLAG);

#if defined(HX_SMART_WAKEUP)
#define HX_PROC_SMWP_FILE "SMWP"
struct proc_dir_entry *proc_SMWP_file;
#define HX_PROC_GESTURE_FILE "GESTURE"
struct proc_dir_entry *proc_gesture_file;
uint8_t HX_SMWP_EN;
#if defined(HX_ULTRA_LOW_POWER)
#define HX_PROC_PSENSOR_FILE "Psensor"
struct proc_dir_entry *proc_psensor_file;
#endif
#endif

#if defined(HX_HIGH_SENSE)
#define HX_PROC_HSEN_FILE "HSEN"
struct proc_dir_entry *proc_hsen_file;
#endif

#if defined(HX_HEADPHONE)
#define HX_PROC_HEADPHONE_FILE "headphone"
struct proc_dir_entry *proc_headphone_file;
#endif

#define HX_PROC_VENDOR_FILE "vendor"
struct proc_dir_entry *proc_vendor_file;

#if defined(HX_PALM_REPORT)
static int himax_palm_detect(const uint8_t *buf)
{
	struct himax_ts_data *ts = hx_s_ts;
	int32_t i;
	int base = 0;
	int x = 0, y = 0, w = 0;

	i = 0;
	base = i * 4;
	x = buf[base] << 8 | buf[base + 1];
	y = (buf[base + 2] << 8 | buf[base + 3]);
	w = buf[(ts->nFinger_support * 4) + i];
	I(" %s HX_PALM_REPORT_loopi=%d,base=%x,X=%x,Y=%x,W=%x\n",
		__func__, i, base, x, y, w);
	if ((!atomic_read(&ts->suspend_mode))
	&& (x == 0xFA5A)
	&& (y == 0xFA5A)
	&& (w == 0x00))
		return PALM_REPORT;
	else
		return NOT_REPORT;
}
#endif

#if defined(HX_FIRMWARE_HEADER)
uint32_t get_fw_index(int32_t fw_type)
{
	unsigned int i;
	uint32_t ret = 0xFF;

	for (i = 0; i < gTotal_N_firmware; i++) {
		if (gHimax_firmware_id[i] == g_hx_panel_id) {
			if (gHimax_firmware_type[i] == fw_type) {
				ret = i;
				break;
			}
		}
	}
	return ret;
}
void hx_get_embedded_fw(int type)
{
	uint32_t idx = 0;

	if (type == HX_FWTYPE_NORMAL)
		I("%s: Normal Type!\n", __func__);
	else if (type == HX_FWTYPE_MP)
		I("%s: MP Type!\n", __func__);

	idx = get_fw_index(type);
	if (idx < gTotal_N_firmware) {
		hx_s_ts->is_set_embedded_fw = 1;
		hx_s_ts->_embedded_fw.data = gHimax_firmware_list[idx];
		hx_s_ts->_embedded_fw.size = gHimax_firmware_size[idx];
	} else {
		hx_s_ts->is_set_embedded_fw = 0;
		E("%s: Embedded no this type fw!\n", __func__);
	}
}
#endif

static ssize_t himax_self_test(struct seq_file *s, void *v)
{

	size_t ret = 0;

	I("%s: enter, %d\n", __func__, __LINE__);

	if (hx_s_ts->suspended == 1) {
		E("%s: please do self test in normal active mode\n", __func__);
		return HX_INIT_FAIL;
	}

	if (hx_s_ts->in_self_test == 1) {
		W("%s: Self test is running now!\n", __func__);
		return ret;
	}
	hx_s_ts->in_self_test = 1;

	himax_int_enable(0);/* disable irq */
	if (hx_s_core_fp._chip_self_test != NULL)
		hx_s_core_fp._chip_self_test(s, v);
	else {
		E("%s: Doesn't turn on inspection function!\n", __func__);
		seq_puts(s, "Doesn't turn on inspection function!\n");
	}


#if defined(HX_EXCP_RECOVERY)
	HX_EXCP_RESET_ACTIVATE = 1;
#endif
	himax_int_enable(1);

	hx_s_ts->in_self_test = 0;

	return ret;
}

static void *hx_proc_selftest_seq_start(struct seq_file *s, loff_t *pos)
{
	if (*pos >= 1)
		return NULL;


	return (void *)((unsigned long) *pos + 1);
}

static void *hx_proc_selftest_seq_next(struct seq_file *s, void *v, loff_t *pos)
{
	return NULL;
}

static void hx_proc_selftest_seq_stop(struct seq_file *s, void *v)
{
}

static int hx_proc_selftest_seq_read(struct seq_file *s, void *v)
{
	size_t ret = 0;

	if (chip_test_r_flag) {
#if defined(CONFIG_TOUCHSCREEN_HIMAX_INSPECT)
		if (g_rslt_data) {
			if (s->size < g_rslt_data_len) {
				s->size = g_rslt_data_len;
				s->buf = kcalloc(s->size,
					sizeof(char), GFP_KERNEL);
				if (s->buf == NULL) {
					E("%s,%d: Memory allocation falied!\n",
						__func__, __LINE__);
					seq_puts(s, "Allocate Memory Fail!\n");
					return -ENOMEM;
				}
				memset(s->buf, 0, s->size * sizeof(char));
			}
			seq_printf(s, "%s", g_rslt_data);
		} else
#endif
			seq_puts(s, "No chip test data.\n");
	} else {
		himax_self_test(s, v);
	}

	return ret;
}

static const struct seq_operations hx_proc_selftest_seq_ops = {
	.start	= hx_proc_selftest_seq_start,
	.next	= hx_proc_selftest_seq_next,
	.stop	= hx_proc_selftest_seq_stop,
	.show	= hx_proc_selftest_seq_read,
};

static int hx_proc_selftest_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &hx_proc_selftest_seq_ops);
};

static ssize_t hx_proc_selftest_write(struct file *filp,
	const char __user *buff, size_t len, loff_t *data)
{
	char buf[80];

	if (len >= 80) {
		I("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}

	if (copy_from_user(buf, buff, len))
		return -EFAULT;

	if (buf[0] == 'r') {
		chip_test_r_flag = true;
		I("%s: Start to read chip test data.\n", __func__);
	}	else {
		chip_test_r_flag = false;
		I("%s: Back to do self test.\n", __func__);
	}

	return len;
}

static const struct proc_ops_name hx_proc_selftest_ops = {
	owner_line
	.proc_op(open) = hx_proc_selftest_open,
	.proc_op(read) = seq_read,
	.proc_op(write) = hx_proc_selftest_write,
	.proc_op(release) = seq_release,
};

#if defined(HX_HIGH_SENSE)
static ssize_t hx_proc_hsen_read(struct file *file, char *buf,
		size_t len, loff_t *pos)
{
	struct himax_ts_data *ts = hx_s_ts;
	size_t count = 0;
	char *temp_buf = NULL;

	if (!HX_PROC_SEND_FLAG) {
		temp_buf = kcalloc(len, sizeof(char), GFP_KERNEL);
		if (temp_buf != NULL) {
			count = snprintf(temp_buf, PAGE_SIZE, "%d\n",
					ts->HSEN_enable);

			if (copy_to_user(buf, temp_buf, len))
				I("%s, here:%d\n", __func__, __LINE__);

			kfree(temp_buf);
			HX_PROC_SEND_FLAG = 1;
		} else {
			E("%s, Failed to allocate memory\n", __func__);
		}
	} else {
		HX_PROC_SEND_FLAG = 0;
	}

	return count;
}

static ssize_t hx_proc_hsen_write(struct file *file, const char *buff,
		size_t len, loff_t *pos)
{
	struct himax_ts_data *ts = hx_s_ts;
	char buf[80] = {0};

	if (len >= 80) {
		I("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}

	if (copy_from_user(buf, buff, len))
		return -EFAULT;

	if (buf[0] == '0')
		ts->HSEN_enable = 0;
	else if (buf[0] == '1')
		ts->HSEN_enable = 1;
	else
		return -EINVAL;

	hx_s_core_fp._set_HSEN_enable(ts->HSEN_enable, ts->suspended);
	I("%s: HSEN_enable = %d.\n", __func__, ts->HSEN_enable);
	return len;
}

static const struct proc_ops_name hx_proc_hsen_ops = {
	owner_line
	.proc_op(read) = hx_proc_hsen_read,
	.proc_op(write) = hx_proc_hsen_write,
};
#endif

#if defined(HX_SMART_WAKEUP)
static ssize_t hx_proc_smwp_read(struct file *file, char *buf,
		size_t len, loff_t *pos)
{
	size_t count = 0;
	struct himax_ts_data *ts = hx_s_ts;
	char *temp_buf = NULL;

	if (!HX_PROC_SEND_FLAG) {
		temp_buf = kcalloc(len, sizeof(char), GFP_KERNEL);
		if (temp_buf != NULL) {
			count = snprintf(temp_buf, PAGE_SIZE, "%d\n",
					ts->SMWP_enable);

			if (copy_to_user(buf, temp_buf, len))
				I("%s, here:%d\n", __func__, __LINE__);

			kfree(temp_buf);
			HX_PROC_SEND_FLAG = 1;
		} else {
			E("%s, Failed to allocate memory\n", __func__);
		}
	} else {
		HX_PROC_SEND_FLAG = 0;
	}

	return count;
}

static ssize_t hx_proc_smwp_write(struct file *file, const char *buff,
		size_t len, loff_t *pos)
{
	struct himax_ts_data *ts = hx_s_ts;
	char buf[80] = {0};

	if (len >= 80) {
		I("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}

	if (copy_from_user(buf, buff, len))
		return -EFAULT;

	if (buf[0] == '0') {
		ts->SMWP_enable = 0;
		irq_set_irq_wake(hx_s_ts->hx_irq, 0);
	} else if (buf[0] == '1') {
		ts->SMWP_enable = 1;
		irq_set_irq_wake(hx_s_ts->hx_irq, 1);
	} else
		return -EINVAL;

	hx_s_core_fp._set_SMWP_enable(ts->SMWP_enable, ts->suspended);
	HX_SMWP_EN = ts->SMWP_enable;
	I("%s: SMART_WAKEUP_enable = %d.\n", __func__, HX_SMWP_EN);
	return len;
}

static const struct proc_ops_name hx_proc_smwp_ops = {
	owner_line
	.proc_op(read) = hx_proc_smwp_read,
	.proc_op(write) = hx_proc_smwp_write,
};

static ssize_t hx_proc_gesture_read(struct file *file, char *buf,
		size_t len, loff_t *pos)
{
	struct himax_ts_data *ts = hx_s_ts;
	int i;
	size_t ret = 0;
	char *temp_buf = NULL;

	if (!HX_PROC_SEND_FLAG) {
		temp_buf = kcalloc(len, sizeof(char), GFP_KERNEL);
		if (temp_buf != NULL) {
			for (i = 0; i < GEST_SUP_NUM; i++)
				ret += snprintf(temp_buf + ret, len - ret,
						"ges_en[%d]=%d\n",
						i, ts->gesture_cust_en[i]);

			if (copy_to_user(buf, temp_buf, len))
				I("%s, here:%d\n", __func__, __LINE__);

			kfree(temp_buf);
			HX_PROC_SEND_FLAG = 1;
		} else {
			E("%s, Failed to allocate memory\n", __func__);
		}
	} else {
		HX_PROC_SEND_FLAG = 0;
		ret = 0;
	}

	return ret;
}

static ssize_t hx_proc_gesture_write(struct file *file, const char *buff,
		size_t len, loff_t *pos)
{
	struct himax_ts_data *ts = hx_s_ts;
	int i = 0;
	int j = 0;
	char buf[80] = {0};

	if (len >= 80) {
		I("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}

	if (copy_from_user(buf, buff, len))
		return -EFAULT;

	I("himax_GESTURE_store= %s, len = %d\n", buf, (int)len);

	for (i = 0; i < len; i++) {
		if (buf[i] == '0' && j < GEST_SUP_NUM) {
			ts->gesture_cust_en[j] = 0;
			I("gesture en[%d]=%d\n", j, ts->gesture_cust_en[j]);
			j++;
		} else if (buf[i] == '1' && j < GEST_SUP_NUM) {
			ts->gesture_cust_en[j] = 1;
			I("gesture en[%d]=%d\n", j, ts->gesture_cust_en[j]);
			j++;
		} else
			I("Not 0/1 or >=GEST_SUP_NUM : buf[%d] = %c\n",
				i, buf[i]);
	}

	return len;
}

static const struct proc_ops_name himax_proc_Gesture_ops = {
	owner_line
	.proc_op(read) = hx_proc_gesture_read,
	.proc_op(write) = hx_proc_gesture_write,
};

#if defined(HX_ULTRA_LOW_POWER)
static ssize_t hx_proc_psensor_read(struct file *file, char *buf,
		size_t len, loff_t *pos)
{
	size_t count = 0;
	struct himax_ts_data *ts = hx_s_ts;
	char *temp_buf = NULL;

	if (!HX_PROC_SEND_FLAG) {
		temp_buf = kcalloc(len, sizeof(char), GFP_KERNEL);
		if (temp_buf != NULL) {
			count = snprintf(temp_buf, PAGE_SIZE,
					"p-sensor flag = %d\n",
					ts->psensor_flag);

			if (copy_to_user(buf, temp_buf, len))
				I("%s, here:%d\n", __func__, __LINE__);

			kfree(temp_buf);
			HX_PROC_SEND_FLAG = 1;
		} else {
			E("%s, Failed to allocate memory\n", __func__);
		}
	} else {
		HX_PROC_SEND_FLAG = 0;
	}

	return count;
}

static ssize_t hx_proc_psensor_write(struct file *file, const char *buff,
		size_t len, loff_t *pos)
{
	struct himax_ts_data *ts = hx_s_ts;
	char buf[80] = {0};

	if (len >= 80) {
		I("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}

	if (copy_from_user(buf, buff, len))
		return -EFAULT;

	if (buf[0] == '0' && ts->SMWP_enable == 1) {
		ts->psensor_flag = false;
		hx_s_core_fp._black_gest_ctrl(false);
	} else if (buf[0] == '1' && ts->SMWP_enable == 1) {
		ts->psensor_flag = true;
		hx_s_core_fp._black_gest_ctrl(true);
	} else if (ts->SMWP_enable == 0) {
		I("%s: SMWP is disable, not supprot to ctrl p-sensor.\n",
			__func__);
	} else
		return -EINVAL;

	I("%s: psensor_flag = %d.\n", __func__, ts->psensor_flag);
	return len;
}

static const struct proc_ops_name himax_proc_psensor_ops = {
	owner_line
	.proc_op(read) = hx_proc_psensor_read,
	.proc_op(write) = hx_proc_psensor_write,
};
#endif
#endif

#if defined(HX_HEADPHONE)
static ssize_t hx_proc_headphone_read(struct file *file, char *buf,
		size_t len, loff_t *pos)
{
	struct himax_ts_data *ts = hx_s_ts;
	size_t count = 0;
	char *temp_buf = NULL;

	if (!HX_PROC_SEND_FLAG) {
		temp_buf = kcalloc(len, sizeof(char), GFP_KERNEL);
		if (temp_buf != NULL) {
			count = snprintf(temp_buf, PAGE_SIZE, "%d\n",
					ts->hp_en);

			if (copy_to_user(buf, temp_buf, len))
				I("%s, here:%d\n", __func__, __LINE__);

			kfree(temp_buf);
			HX_PROC_SEND_FLAG = 1;
		} else {
			E("%s, Failed to allocate memory\n", __func__);
		}
	} else {
		HX_PROC_SEND_FLAG = 0;
	}

	return count;
}

static ssize_t hx_proc_headphone_write(struct file *file, const char *buff,
		size_t len, loff_t *pos)
{
	struct himax_ts_data *ts = hx_s_ts;
	char buf[80] = {0};

	if (len >= 80) {
		I("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}

	if (copy_from_user(buf, buff, len))
		return -EFAULT;

	if (buf[0] == '0')
		ts->hp_en = 0;
	else if (buf[0] == '1')
		ts->hp_en = 1;
	else
		return -EINVAL;

	hx_s_core_fp._set_headphone_en(ts->hp_en, ts->suspended);
	I("%s: Headphone enable = %d.\n", __func__, ts->hp_en);
	return len;
}

static const struct proc_ops_name hx_proc_headphone_ops = {
	owner_line
	.proc_op(read) = hx_proc_headphone_read,
	.proc_op(write) = hx_proc_headphone_write,
};
#endif

static ssize_t hx_proc_vendor_read(struct file *file, char *buf,
				size_t len, loff_t *pos)
{
	ssize_t ret = 0;
	char *temp_buf = NULL;

	if (!HX_PROC_SEND_FLAG) {
		temp_buf = kcalloc(len, sizeof(char), GFP_KERNEL);
		ret += snprintf(temp_buf + ret, len - ret,
			"IC = %s\n", hx_s_ts->chip_name);

		ret += snprintf(temp_buf + ret, len - ret,
			"FW_VER = 0x%2.2X\n", hx_s_ic_data->vendor_fw_ver);

		if (hx_s_ts->chip_cell_type == CHIP_IS_ON_CELL) {
			ret += snprintf(temp_buf + ret, len - ret,
				"CONFIG_VER = 0x%2.2X\n",
				hx_s_ic_data->vendor_config_ver);
		} else {
			ret += snprintf(temp_buf + ret, len - ret,
					"TOUCH_VER = 0x%2.2X\n",
					hx_s_ic_data->vendor_touch_cfg_ver);
			ret += snprintf(temp_buf + ret, len - ret,
					"DISPLAY_VER = 0x%2.2X\n",
					hx_s_ic_data->vendor_display_cfg_ver);
		}

		if (hx_s_ic_data->vendor_cid_maj_ver < 0
		&& hx_s_ic_data->vendor_cid_min_ver < 0) {
			ret += snprintf(temp_buf + ret, len - ret,
					"CID_VER = NULL\n");
		} else {
			ret += snprintf(temp_buf + ret, len - ret,
					"CID_VER = 0x%2.2X\n",
					(hx_s_ic_data->vendor_cid_maj_ver << 8 |
					hx_s_ic_data->vendor_cid_min_ver));
		}

		if (hx_s_ic_data->vendor_panel_ver < 0) {
			ret += snprintf(temp_buf + ret, len - ret,
					"PANEL_VER = NULL\n");
		} else {
			ret += snprintf(temp_buf + ret, len - ret,
					"PANEL_VER = 0x%2.2X\n",
					hx_s_ic_data->vendor_panel_ver);
		}
		if (hx_s_ts->chip_cell_type == CHIP_IS_IN_CELL) {
			ret += snprintf(temp_buf + ret, len - ret,
					"Cusomer = %s\n",
					hx_s_ic_data->vendor_cus_info);
			ret += snprintf(temp_buf + ret, len - ret,
					"Project = %s\n",
					hx_s_ic_data->vendor_proj_info);
		}
		ret += snprintf(temp_buf + ret, len - ret, "\n");
		ret += snprintf(temp_buf + ret, len - ret,
				"Himax Touch Driver Version:\n");
		ret += snprintf(temp_buf + ret, len - ret, "%s\n",
				HIMAX_DRIVER_VER);

		if (copy_to_user(buf, temp_buf, len))
			I("%s,here:%d\n", __func__, __LINE__);

		kfree(temp_buf);
		HX_PROC_SEND_FLAG = 1;
	} else {
		HX_PROC_SEND_FLAG = 0;
	}

	return ret;
}
static const struct proc_ops_name hx_proc_vendor_ops = {
	owner_line
	.proc_op(read) = hx_proc_vendor_read,
};

int himax_common_proc_init(void)
{
	hx_touch_proc_dir = proc_mkdir(HX_PROC_TOUCH_FOLDER, NULL);

	if (hx_touch_proc_dir == NULL) {
		E(" %s: hx_touch_proc_dir file create failed!\n", __func__);
		return -ENOMEM;
	}
#if defined(CONFIG_TOUCHSCREEN_HIMAX_INSPECT)
	if (_himax_self_test_init != NULL)
		_himax_self_test_init();
#endif

	proc_self_test_file = proc_create(HX_PROC_SELF_TEST_FILE, 0444,
		hx_touch_proc_dir, &hx_proc_selftest_ops);
	if (proc_self_test_file == NULL) {
		E(" %s: proc self_test file create failed!\n", __func__);
		goto fail_1;
	}

#if defined(HX_HIGH_SENSE)
	proc_hsen_file = proc_create(HX_PROC_HSEN_FILE, 0666,
			hx_touch_proc_dir, &hx_proc_hsen_ops);

	if (proc_hsen_file == NULL) {
		E(" %s: proc HSEN file create failed!\n", __func__);
		goto fail_2;
	}

#endif
#if defined(HX_SMART_WAKEUP)
	proc_SMWP_file = proc_create(HX_PROC_SMWP_FILE, 0666,
			hx_touch_proc_dir, &hx_proc_smwp_ops);

	if (proc_SMWP_file == NULL) {
		E(" %s: proc SMWP file create failed!\n", __func__);
		goto fail_3;
	}

	proc_gesture_file = proc_create(HX_PROC_GESTURE_FILE, 0666,
			hx_touch_proc_dir, &himax_proc_Gesture_ops);

	if (proc_gesture_file == NULL) {
		E(" %s: proc GESTURE file create failed!\n", __func__);
		goto fail_4;
	}
#if defined(HX_ULTRA_LOW_POWER)
	proc_psensor_file = proc_create(HX_PROC_PSENSOR_FILE, 0666,
			hx_touch_proc_dir, &himax_proc_psensor_ops);

	if (proc_psensor_file == NULL) {
		E(" %s: proc GESTURE file create failed!\n", __func__);
		goto fail_5;
	}
#endif
#endif
	proc_vendor_file = proc_create(HX_PROC_VENDOR_FILE, 0444,
		hx_touch_proc_dir, &hx_proc_vendor_ops);
	if (proc_vendor_file == NULL) {
		E(" %s: proc vendor file create failed!\n", __func__);
		goto fail_6;
	}
#if defined(HX_HEADPHONE)
	proc_headphone_file = proc_create(HX_PROC_HEADPHONE_FILE, 0444,
		hx_touch_proc_dir, &hx_proc_headphone_ops);
	if (proc_headphone_file == NULL) {
		E(" %s: proc headphone file create failed!\n", __func__);
		goto fail_7;
	}
#endif

	return 0;
#if defined(HX_HEADPHONE)
fail_7:
#endif
	remove_proc_entry(HX_PROC_VENDOR_FILE, hx_touch_proc_dir);
fail_6:
#if defined(HX_SMART_WAKEUP)
#if defined(HX_ULTRA_LOW_POWER)
	remove_proc_entry(HX_PROC_PSENSOR_FILE, hx_touch_proc_dir);
fail_5:
#endif
	remove_proc_entry(HX_PROC_GESTURE_FILE, hx_touch_proc_dir);
fail_4:
	remove_proc_entry(HX_PROC_SMWP_FILE, hx_touch_proc_dir);
fail_3:
#endif
#if defined(HX_HIGH_SENSE)
	remove_proc_entry(HX_PROC_HSEN_FILE, hx_touch_proc_dir);
fail_2:
#endif
	remove_proc_entry(HX_PROC_SELF_TEST_FILE, hx_touch_proc_dir);
fail_1:
	return -ENOMEM;
}

void himax_common_proc_deinit(void)
{
#if defined(HX_HEADPHONE)
	remove_proc_entry(HX_PROC_HEADPHONE_FILE, hx_touch_proc_dir);
#endif
	remove_proc_entry(HX_PROC_VENDOR_FILE, hx_touch_proc_dir);
#if defined(HX_SMART_WAKEUP)
#if defined(HX_ULTRA_LOW_POWER)
	remove_proc_entry(HX_PROC_PSENSOR_FILE, hx_touch_proc_dir);
#endif
	remove_proc_entry(HX_PROC_GESTURE_FILE, hx_touch_proc_dir);
	remove_proc_entry(HX_PROC_SMWP_FILE, hx_touch_proc_dir);
#endif
#if defined(HX_HIGH_SENSE)
	remove_proc_entry(HX_PROC_HSEN_FILE, hx_touch_proc_dir);
#endif
	remove_proc_entry(HX_PROC_SELF_TEST_FILE, hx_touch_proc_dir);

	remove_proc_entry(HX_PROC_TOUCH_FOLDER, NULL);
}

/* File node for SMWP and HSEN - End*/

void hx_parse_assign_cmd(uint32_t addr, uint8_t *cmd, int len)
{
	/*I("%s: Entering!\n", __func__);*/

	switch (len) {
	case 1:
		cmd[0] = addr;
		/*I("%s: cmd[0] = 0x%02X\n", __func__, cmd[0]);*/
		break;

	case 2:
		cmd[0] = addr & 0xFF;
		cmd[1] = (addr >> 8) & 0xFF;
		/*I("%s: cmd[0] = 0x%02X,cmd[1] = 0x%02X\n",*/
		/*	__func__, cmd[0], cmd[1]);*/
		break;

	case 4:
		cmd[0] = addr & 0xFF;
		cmd[1] = (addr >> 8) & 0xFF;
		cmd[2] = (addr >> 16) & 0xFF;
		cmd[3] = (addr >> 24) & 0xFF;
		/*  I("%s: cmd[0] = 0x%02X,cmd[1] = 0x%02X,*/
		/*cmd[2] = 0x%02X,cmd[3] = 0x%02X\n", */
		/* __func__, cmd[0], cmd[1], cmd[2], cmd[3]);*/
		break;

	default:
		E("%s: input length fault,len = %d!\n", __func__, len);
		break;
	}
}
EXPORT_SYMBOL(hx_parse_assign_cmd);

int himax_input_register(struct himax_ts_data *ts)
{
	int ret = 0;
#if defined(HX_SMART_WAKEUP)
	int i = 0;
#endif
	ret = himax_dev_set(ts);
	if (ret < 0) {
		I("%s, input device register fail!\n", __func__);
		return INPUT_REGISTER_FAIL;
	}

	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_ABS, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
	set_bit(KEY_BACK, ts->input_dev->keybit);
	set_bit(KEY_HOME, ts->input_dev->keybit);
	set_bit(KEY_MENU, ts->input_dev->keybit);
	set_bit(KEY_SEARCH, ts->input_dev->keybit);

#if defined(HX_SMART_WAKEUP)
	for (i = 0; i < GEST_SUP_NUM; i++)
		set_bit(gest_key_def[i], ts->input_dev->keybit);
#elif defined(CONFIG_TOUCHSCREEN_HIMAX_INSPECT) || defined(HX_PALM_REPORT)
	set_bit(KEY_POWER, ts->input_dev->keybit);
#endif
	set_bit(BTN_TOUCH, ts->input_dev->keybit);
	set_bit(KEY_APPSELECT, ts->input_dev->keybit);
	set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);
#if defined(HX_PROTOCOL_A)
	/*ts->input_dev->mtsize = ts->nFinger_support;*/
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 1, 10, 0, 0);
#else
	set_bit(MT_TOOL_FINGER, ts->input_dev->keybit);
#if defined(HX_PROTOCOL_B_3PA)
	input_mt_init_slots(ts->input_dev, ts->nFinger_support,
			INPUT_MT_DIRECT);
#else
	input_mt_init_slots(ts->input_dev, ts->nFinger_support);
#endif
#endif
	I("input_set_abs_params: mix_x %d, max_x %d, min_y %d, max_y %d\n",
			ts->pdata->abs_x_min,
			ts->pdata->abs_x_max,
			ts->pdata->abs_y_min,
			ts->pdata->abs_y_max);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X,
			ts->pdata->abs_x_min, ts->pdata->abs_x_max,
			ts->pdata->abs_x_fuzz, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y,
			ts->pdata->abs_y_min, ts->pdata->abs_y_max,
			ts->pdata->abs_y_fuzz, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR,
			ts->pdata->abs_pressure_min,
			ts->pdata->abs_pressure_max,
			ts->pdata->abs_pressure_fuzz, 0);
#if !defined(HX_PROTOCOL_A)
	input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE,
			ts->pdata->abs_pressure_min,
			ts->pdata->abs_pressure_max,
			ts->pdata->abs_pressure_fuzz, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR,
			ts->pdata->abs_width_min,
			ts->pdata->abs_width_max,
			ts->pdata->abs_pressure_fuzz, 0);
#endif
/*	input_set_abs_params(ts->input_dev, ABS_MT_AMPLITUDE, 0,*/
/*			((ts->pdata->abs_pressure_max << 16)*/
/*			| ts->pdata->abs_width_max),*/
/*			0, 0);*/
/*	input_set_abs_params(ts->input_dev, ABS_MT_POSITION,*/
/*			0, (BIT(31)*/
/*			| (ts->pdata->abs_x_max << 16)*/
/*			| ts->pdata->abs_y_max),*/
/*			0, 0);*/

	if (himax_input_register_device(ts->input_dev) == 0) {
		ret = NO_ERR;
	} else {
		E("%s: input register fail\n", __func__);
		input_free_device(ts->input_dev);
		return INPUT_REGISTER_FAIL;
	}

	if (!hx_s_ic_data->stylus_func)
		goto skip_stylus_operation;

	set_bit(EV_SYN, ts->stylus_dev->evbit);
	set_bit(EV_ABS, ts->stylus_dev->evbit);
	set_bit(EV_KEY, ts->stylus_dev->evbit);
	set_bit(BTN_TOUCH, ts->stylus_dev->keybit);
	set_bit(INPUT_PROP_DIRECT, ts->stylus_dev->propbit);

	set_bit(BTN_TOOL_PEN, ts->stylus_dev->keybit);
	set_bit(BTN_TOOL_RUBBER, ts->stylus_dev->keybit);

	input_set_abs_params(ts->stylus_dev, ABS_PRESSURE, 0, 4095, 0, 0);
	input_set_abs_params(ts->stylus_dev, ABS_DISTANCE, 0, 1, 0, 0);
	input_set_abs_params(ts->stylus_dev, ABS_TILT_X, -60, 60, 0, 0);
	input_set_abs_params(ts->stylus_dev, ABS_TILT_Y, -60, 60, 0, 0);
	/*input_set_capability(ts->hx_pen_dev, EV_SW, SW_PEN_INSERT);*/
	input_set_capability(ts->stylus_dev, EV_KEY, BTN_TOUCH);
	input_set_capability(ts->stylus_dev, EV_KEY, BTN_STYLUS);
	input_set_capability(ts->stylus_dev, EV_KEY, BTN_STYLUS2);

	input_set_abs_params(ts->stylus_dev, ABS_X, ts->pdata->abs_x_min,
			((ts->pdata->abs_x_max+1)*hx_s_ic_data->stylus_ratio-1),
			ts->pdata->abs_x_fuzz, 0);
	input_set_abs_params(ts->stylus_dev, ABS_Y, ts->pdata->abs_y_min,
			((ts->pdata->abs_y_max+1)*hx_s_ic_data->stylus_ratio-1),
			ts->pdata->abs_y_fuzz, 0);

	if (himax_input_register_device(ts->stylus_dev) == 0) {
		ret = NO_ERR;
	} else {
		E("%s: input register stylus fail\n", __func__);
		input_unregister_device(ts->input_dev);
		input_free_device(ts->stylus_dev);
		return INPUT_REGISTER_FAIL;
	}

skip_stylus_operation:

	I("%s, input device registered.\n", __func__);

	return ret;
}
EXPORT_SYMBOL(himax_input_register);


#if defined(HX_BOOT_UPGRADE)
int himax_get_fw_ver_bin(const struct firmware *hxfw)
{
	I("%s: use default address.\n", __func__);
	if (hxfw != NULL) {
		I("Catch fw version in bin file!\n");
		if (hx_s_fwbin->addr_fw_ver_maj <= (hxfw->size-2))
			g_i_FW_VER =
				(hxfw->data[hx_s_fwbin->addr_fw_ver_maj] << 8)
				| hxfw->data[hx_s_fwbin->addr_fw_ver_min];
		if (hx_s_fwbin->addr_cfg_ver_maj <= (hxfw->size-2))
			g_i_CFG_VER = hxfw->data[hx_s_fwbin->addr_cfg_ver_maj];
		if (hx_s_fwbin->addr_cid_ver_maj <= (hxfw->size-2))
			g_i_CID_MAJ = hxfw->data[hx_s_fwbin->addr_cid_ver_maj];
		if (hx_s_fwbin->addr_cid_ver_min <= (hxfw->size-1))
			g_i_CID_MIN = hxfw->data[hx_s_fwbin->addr_cid_ver_min];
	} else {
		I("FW data is null!\n");
		return 1;
	}
	return NO_ERR;
}
static int himax_auto_update_check(void)
{
	int32_t ret;

	I("%s: Entering!\n", __func__);
	if (himax_get_fw_ver_bin(hxfw) == 0) {
		if (((hx_s_ic_data->vendor_fw_ver < g_i_FW_VER)
		|| (hx_s_ic_data->vendor_touch_cfg_ver < g_i_CFG_VER))) {
			I("%s: Need update\n", __func__);
			ret = 0;
		} else {
			I("%s: Need not update!\n", __func__);
			ret = 1;
		}
	} else {
		E("%s: FW bin fail!\n", __func__);
		ret = 1;
	}

	return ret;
}
#endif

#if defined(HX_BOOT_UPGRADE) || defined(HX_ZERO_FLASH)
static int i_get_FW(void)
{
	int ret = -1;
	int result = NO_ERR;

	ret = request_firmware(&hxfw, hx_s_ts->boot_fw_name, hx_s_ts->dev);
	I("%s: request file %s finished\n", __func__, hx_s_ts->boot_fw_name);
	if (ret < 0) {
#if defined(HX_FIRMWARE_HEADER)
		hx_get_embedded_fw(HX_FWTYPE_NORMAL);
		if (hx_s_ts->is_set_embedded_fw == 1) {
			hxfw = &hx_s_ts->_embedded_fw;
			I("%s: Not find FW, use Header FW(size:%zu)",
				__func__, hxfw->size);
			result = HX_EMBEDDED_FW;
		} else {
			E("%s,%d: error code = %d\n", __func__, __LINE__, ret);
			result = OPEN_FILE_FAIL;
			goto END;
		}
#else
		E("%s,%d: error code = %d\n", __func__, __LINE__, ret);
		result = OPEN_FILE_FAIL;
		goto END;
#endif
	}

	I("Gotten FW last 4bytes=0x%02X,%02X,%02X,%02X\n",
		hxfw->data[hxfw->size - 4],
		hxfw->data[hxfw->size - 3],
		hxfw->data[hxfw->size - 2],
		hxfw->data[hxfw->size - 1]);
END:
	return result;
}

static int i_update_FW(void)
{
	int upgrade_times = 0;
	int8_t ret = 0;
	int8_t result = 0;

update_retry:
#if defined(HX_ZERO_FLASH)

	ret = hx_s_core_fp._firmware_update_0f(hxfw, 0);
	if (ret != 0) {
		upgrade_times++;
		E("%s: TP upgrade error, upgrade_times = %d\n",
				__func__, upgrade_times);

		if (upgrade_times < 3)
			goto update_retry;
		else
			result = -1;

	} else {
		result = 1;/*upgrade success*/
		I("%s: TP upgrade OK\n", __func__);
	}

#else
	I("%s:Now fw size = %d!\n", __func__, (int)hxfw->size);
	ret = hx_s_core_fp._fts_ctpm_fw_upgrade_with_sys_fs(
		(unsigned char *)hxfw->data, hxfw->size, false);

	if (ret == 0) {
		upgrade_times++;
		E("%s: TP upgrade error, upgrade_times = %d\n",
				__func__, upgrade_times);

		if (upgrade_times < 3)
			goto update_retry;
		else
			result = -1;

	} else {
		result = 1;/*upgrade success*/
		I("%s: TP upgrade OK\n", __func__);
	}
#endif

	return result;
}
#endif
/*
 *static int himax_loadSensorConfig(struct himax_platform_data *pdata)
 *{
 *	I("%s: initialization complete\n", __func__);
 *	return NO_ERR;
 *}
 */
#if defined(HX_SMART_WAKEUP)
#if defined(HX_GESTURE_TRACK)
static void gest_pt_log_coordinate(int rx, int tx)
{
	/*driver report x y with range 0 - 255 , we scale it up to x/y pixel*/
	gest_pt_x[gest_pt_cnt] = rx * (hx_s_ic_data->x_res) / 255;
	gest_pt_y[gest_pt_cnt] = tx * (hx_s_ic_data->y_res) / 255;
}
#endif
static int himax_wake_event_parse(struct himax_ts_data *ts, int ts_status)
{
	uint8_t *buf = wake_event_buffer;
#if defined(HX_GESTURE_TRACK)
	int tmp_max_x = 0x00;
	int tmp_min_x = 0xFFFF;
	int tmp_max_y = 0x00;
	int tmp_min_y = 0xFFFF;
	int gest_len;
#endif
	int i = 0, check_FC = 0, ret;
	int j = 0, gesture_pos = 0, gesture_flag = 0;

	if (g_ts_dbg != 0)
		I("%s: Entering!, ts_status=%d\n", __func__, ts_status);

	if (buf == NULL) {
		ret = -ENOMEM;
		goto END;
	}

	memcpy(buf, hx_s_touch_data->event_buf, hx_s_touch_data->event_size);

	for (i = 0; i < GEST_PTLG_ID_LEN; i++) {
		for (j = 0; j < GEST_SUP_NUM; j++) {
			if (buf[i] == gest_event[j]) {
				gesture_flag = buf[i];
				gesture_pos = j;
				break;
			}
		}
		I("0x%2.2X ", buf[i]);
		if (buf[i] == gesture_flag) {
			check_FC++;
		} else {
			I("ID START at %x , value = 0x%2X skip the event\n",
					i, buf[i]);
			break;
		}
	}

	I("Himax gesture_flag= %x\n", gesture_flag);
	I("Himax check_FC is %d\n", check_FC);

	if (check_FC != GEST_PTLG_ID_LEN) {
		ret = 0;
		goto END;
	}

	if (buf[GEST_PTLG_ID_LEN] != GEST_PTLG_HDR_ID1
	|| buf[GEST_PTLG_ID_LEN + 1] != GEST_PTLG_HDR_ID2) {
		ret = 0;
		goto END;
	}

#if defined(HX_GESTURE_TRACK)

	if (buf[GEST_PTLG_ID_LEN] == GEST_PTLG_HDR_ID1
	&& buf[GEST_PTLG_ID_LEN + 1] == GEST_PTLG_HDR_ID2) {
		gest_len = buf[GEST_PTLG_ID_LEN + 2];
		I("gest_len = %d\n", gest_len);
		i = 0;
		gest_pt_cnt = 0;
		I("gest doornidate start\n %s", __func__);

		while (i < (gest_len + 1) / 2) {
			gest_pt_log_coordinate(
					buf[GEST_PTLG_ID_LEN + 4 + i * 2],
					buf[GEST_PTLG_ID_LEN + 4 + i * 2 + 1]);
			i++;
			I("gest_pt_x[%d]=%d,gest_pt_y[%d]=%d\n",
				gest_pt_cnt,
				gest_pt_x[gest_pt_cnt],
				gest_pt_cnt,
				gest_pt_y[gest_pt_cnt]);
			gest_pt_cnt += 1;
		}

		if (gest_pt_cnt) {
			for (i = 0; i < gest_pt_cnt; i++) {
				if (tmp_max_x < gest_pt_x[i])
					tmp_max_x = gest_pt_x[i];
				if (tmp_min_x > gest_pt_x[i])
					tmp_min_x = gest_pt_x[i];
				if (tmp_max_y < gest_pt_y[i])
					tmp_max_y = gest_pt_y[i];
				if (tmp_min_y > gest_pt_y[i])
					tmp_min_y = gest_pt_y[i];
			}

			I("gest_point x_min=%d,x_max=%d,y_min=%d,y_max=%d\n",
				tmp_min_x, tmp_max_x, tmp_min_y, tmp_max_y);

			gest_start_x = gest_pt_x[0];
			hx_gesture_coor[0] = gest_start_x;
			gest_start_y = gest_pt_y[0];
			hx_gesture_coor[1] = gest_start_y;
			gest_end_x = gest_pt_x[gest_pt_cnt - 1];
			hx_gesture_coor[2] = gest_end_x;
			gest_end_y = gest_pt_y[gest_pt_cnt - 1];
			hx_gesture_coor[3] = gest_end_y;
			gest_width = tmp_max_x - tmp_min_x;
			hx_gesture_coor[4] = gest_width;
			gest_height = tmp_max_y - tmp_min_y;
			hx_gesture_coor[5] = gest_height;
			gest_mid_x = (tmp_max_x + tmp_min_x) / 2;
			hx_gesture_coor[6] = gest_mid_x;
			gest_mid_y = (tmp_max_y + tmp_min_y) / 2;
			hx_gesture_coor[7] = gest_mid_y;
			/*gest_up_x*/
			hx_gesture_coor[8] = gest_mid_x;
			/*gest_up_y*/
			hx_gesture_coor[9] = gest_mid_y - gest_height / 2;
			/*gest_down_x*/
			hx_gesture_coor[10] = gest_mid_x;
			/*gest_down_y*/
			hx_gesture_coor[11] = gest_mid_y + gest_height / 2;
			/*gest_left_x*/
			hx_gesture_coor[12] = gest_mid_x - gest_width / 2;
			/*gest_left_y*/
			hx_gesture_coor[13] = gest_mid_y;
			/*gest_right_x*/
			hx_gesture_coor[14] = gest_mid_x + gest_width / 2;
			/*gest_right_y*/
			hx_gesture_coor[15] = gest_mid_y;
		}
	}

#endif

	if (!ts->gesture_cust_en[gesture_pos]) {
		I("%s NOT report key [%d] = %d\n", __func__,
				gesture_pos, gest_key_def[gesture_pos]);
		g_target_report_data->SMWP_event_chk = 0;
		ret = 0;
	} else {
		g_target_report_data->SMWP_event_chk =
				gest_key_def[gesture_pos];
		ret = gesture_pos;
	}
END:
	return ret;
}

static void himax_wake_event_report(void)
{
	int KEY_EVENT = g_target_report_data->SMWP_event_chk;

	if (g_ts_dbg != 0)
		I("%s: Entering!\n", __func__);

	if (KEY_EVENT) {
		I("%s SMART WAKEUP KEY event %d press\n", __func__, KEY_EVENT);
		input_report_key(hx_s_ts->input_dev, KEY_EVENT, 1);
		input_sync(hx_s_ts->input_dev);
		I("%s SMART WAKEUP KEY event %d release\n",
				__func__, KEY_EVENT);
		input_report_key(hx_s_ts->input_dev, KEY_EVENT, 0);
		input_sync(hx_s_ts->input_dev);
#if defined(HX_GESTURE_TRACK)
		I("gest_start_x=%d,start_y=%d,end_x=%d,end_y=%d\n",
			gest_start_x,
			gest_start_y,
			gest_end_x,
			gest_end_y);
		I("gest_width=%d,height=%d,mid_x=%d,mid_y=%d\n",
			gest_width,
			gest_height,
			gest_mid_x,
			gest_mid_y);
		I("gest_up_x=%d,up_y=%d,down_x=%d,down_y=%d\n",
			hx_gesture_coor[8],
			hx_gesture_coor[9],
			hx_gesture_coor[10],
			hx_gesture_coor[11]);
		I("gest_left_x=%d,left_y=%d,right_x=%d,right_y=%d\n",
			hx_gesture_coor[12],
			hx_gesture_coor[13],
			hx_gesture_coor[14],
			hx_gesture_coor[15]);
#endif
		g_target_report_data->SMWP_event_chk = 0;
	}
}

#endif

int himax_report_data_init(void)
{
	if (hx_s_touch_data->coord_buf != NULL) {
		kfree(hx_s_touch_data->coord_buf);
		hx_s_touch_data->coord_buf = NULL;
	}

	if (hx_s_touch_data->rawdata_buf != NULL) {
		kfree(hx_s_touch_data->rawdata_buf);
		hx_s_touch_data->rawdata_buf = NULL;
	}

#if defined(HX_SMART_WAKEUP)
	hx_s_touch_data->event_size = hx_s_core_fp._get_touch_data_size();

	if (hx_s_touch_data->event_buf != NULL) {
		kfree(hx_s_touch_data->event_buf);
		hx_s_touch_data->event_buf = NULL;
	}

	if (wake_event_buffer != NULL) {
		kfree(wake_event_buffer);
		wake_event_buffer = NULL;
	}

#endif

	if (g_target_report_data != NULL) {
		if (hx_s_ic_data->stylus_func) {
			kfree(g_target_report_data->s);
			g_target_report_data->s = NULL;
		}

		kfree(g_target_report_data->p);
		g_target_report_data->p = NULL;
		kfree(g_target_report_data);
		g_target_report_data = NULL;
	}

	hx_s_touch_data->touch_all_size =
		HX_FULL_STACK_RAWDATA_SIZE;
	hx_s_touch_data->touch_all_size_full_stack =
		HX_FULL_STACK_RAWDATA_SIZE;
	hx_s_touch_data->touch_all_size_normal =
		hx_s_core_fp._get_touch_data_size();

	hx_s_touch_data->raw_cnt_max = hx_s_ic_data->max_pt / 4;
	hx_s_touch_data->raw_cnt_rmd = hx_s_ic_data->max_pt % 4;
	/* more than 4 fingers */
	if (hx_s_touch_data->raw_cnt_rmd != 0x00) {
		hx_s_touch_data->rawdata_size =
			hx_s_core_fp._cal_data_len(
				hx_s_touch_data->raw_cnt_rmd,
				hx_s_ic_data->max_pt,
				hx_s_touch_data->raw_cnt_max);

		hx_s_touch_data->touch_info_size = (hx_s_ic_data->max_pt
				+ hx_s_touch_data->raw_cnt_max + 2) * 4;
	} else { /* less than 4 fingers */
		hx_s_touch_data->rawdata_size =
			hx_s_core_fp._cal_data_len(
				hx_s_touch_data->raw_cnt_rmd,
				hx_s_ic_data->max_pt,
				hx_s_touch_data->raw_cnt_max);

		hx_s_touch_data->touch_info_size = (hx_s_ic_data->max_pt
				+ hx_s_touch_data->raw_cnt_max + 1) * 4;
	}

	if (hx_s_ic_data->stylus_func) {
		hx_s_touch_data->touch_info_size += STYLUS_INFO_SZ;
		hx_s_touch_data->rawdata_size -= STYLUS_INFO_SZ;
	}

	// hx_s_touch_data->touch_info_size = HX_STACK_ORG_LEN;
	hx_s_touch_data->rawdata_size_normal =
		hx_s_touch_data->rawdata_size;
	// hx_s_touch_data->rawdata_size =
	// hx_s_touch_data->touch_all_size - HX_STACK_ORG_LEN;
	hx_s_touch_data->rawdata_size_full_stack =
		hx_s_touch_data->touch_all_size - HX_STACK_ORG_LEN;

	if ((hx_s_ic_data->tx_num
	* hx_s_ic_data->rx_num
	+ hx_s_ic_data->tx_num
	+ hx_s_ic_data->rx_num)
	% hx_s_touch_data->rawdata_size == 0)
		hx_s_touch_data->rawdata_frame_size =
			(hx_s_ic_data->tx_num
			* hx_s_ic_data->rx_num
			+ hx_s_ic_data->tx_num
			+ hx_s_ic_data->rx_num)
			/ hx_s_touch_data->rawdata_size;
	else
		hx_s_touch_data->rawdata_frame_size =
			(hx_s_ic_data->tx_num
			* hx_s_ic_data->rx_num
			+ hx_s_ic_data->tx_num
			+ hx_s_ic_data->rx_num)
			/ hx_s_touch_data->rawdata_size
			+ 1;

	I("%s:rawdata_fsz = %d,max_pt:%d,hx_raw_cnt_max:%d\n",
		__func__,
		hx_s_touch_data->rawdata_frame_size,
		hx_s_ic_data->max_pt,
		hx_s_touch_data->raw_cnt_max);
	I("%s:hx_raw_cnt_rmd:%d,g_hx_rawdata_size:%d,touch_info_size:%d\n",
		__func__,
		hx_s_touch_data->raw_cnt_rmd,
		hx_s_touch_data->rawdata_size,
		hx_s_touch_data->touch_info_size);

	hx_s_touch_data->coord_buf = kzalloc(sizeof(uint8_t)
			* (hx_s_touch_data->touch_info_size),
			GFP_KERNEL);

	if (hx_s_touch_data->coord_buf == NULL)
		goto mem_alloc_fail_coord_buf;

#if defined(HX_SMART_WAKEUP)
	wake_event_buffer = kcalloc(hx_s_touch_data->event_size,
			sizeof(uint8_t), GFP_KERNEL);
	if (wake_event_buffer == NULL)
		goto mem_alloc_fail_smwp;

	hx_s_touch_data->event_buf = kzalloc(sizeof(uint8_t)
			* (hx_s_touch_data->event_size), GFP_KERNEL);
	if (hx_s_touch_data->event_buf == NULL)
		goto mem_alloc_fail_event_buf;
#endif

	hx_s_touch_data->rawdata_buf = kzalloc(sizeof(uint8_t)
		* (hx_s_touch_data->touch_all_size
		- HX_STACK_ORG_LEN),
		GFP_KERNEL);

	if (hx_s_touch_data->rawdata_buf == NULL)
		goto mem_alloc_fail_rawdata_buf;

	if (g_target_report_data == NULL) {
		g_target_report_data =
			kzalloc(sizeof(struct himax_target_report_data),
			GFP_KERNEL);
		if (g_target_report_data == NULL)
			goto mem_alloc_fail_report_data;
/*
 *#if defined(HX_SMART_WAKEUP)
 *		g_target_report_data->SMWP_event_chk = 0;
 *#endif
 *		I("%s: SMWP_event_chk = %d\n", __func__,
 *			g_target_report_data->SMWP_event_chk);
 */

		g_target_report_data->p = kzalloc(
			sizeof(struct himax_target_point_data)
			*(hx_s_ic_data->max_pt), GFP_KERNEL);
		if (g_target_report_data->p == NULL)
			goto mem_alloc_fail_report_data_p;

		if (!hx_s_ic_data->stylus_func)
			goto skip_stylus_operation;

		g_target_report_data->s = kzalloc(
			sizeof(struct himax_target_stylus_data)*2, GFP_KERNEL);
		if (g_target_report_data->s == NULL)
			goto mem_alloc_fail_report_data_s;
	}

skip_stylus_operation:
	hx_s_touch_data->touch_all_size =
		hx_s_touch_data->touch_all_size_normal;
	hx_s_touch_data->rawdata_size =
		hx_s_touch_data->rawdata_size_normal;
	return NO_ERR;

mem_alloc_fail_report_data_s:
	kfree(g_target_report_data->p);
	g_target_report_data->p = NULL;
mem_alloc_fail_report_data_p:
	kfree(g_target_report_data);
	g_target_report_data = NULL;
mem_alloc_fail_report_data:
	kfree(hx_s_touch_data->rawdata_buf);
	hx_s_touch_data->rawdata_buf = NULL;
mem_alloc_fail_rawdata_buf:
#if defined(HX_SMART_WAKEUP)
	kfree(hx_s_touch_data->event_buf);
	hx_s_touch_data->event_buf = NULL;
mem_alloc_fail_event_buf:
	kfree(wake_event_buffer);
	wake_event_buffer = NULL;
mem_alloc_fail_smwp:
#endif
	kfree(hx_s_touch_data->coord_buf);
	hx_s_touch_data->coord_buf = NULL;
mem_alloc_fail_coord_buf:

	E("%s: Failed to allocate memory\n", __func__);
	return MEM_ALLOC_FAIL;
}
EXPORT_SYMBOL(himax_report_data_init);

void himax_report_data_deinit(void)
{
	if (hx_s_ic_data->stylus_func) {
		kfree(g_target_report_data->s);
		g_target_report_data->s = NULL;
	}

	kfree(g_target_report_data->p);
	g_target_report_data->p = NULL;
	kfree(g_target_report_data);
	g_target_report_data = NULL;

#if defined(HX_SMART_WAKEUP)
	kfree(wake_event_buffer);
	wake_event_buffer = NULL;
	kfree(hx_s_touch_data->event_buf);
	hx_s_touch_data->event_buf = NULL;
#endif
	kfree(hx_s_touch_data->rawdata_buf);
	hx_s_touch_data->rawdata_buf = NULL;
	kfree(hx_s_touch_data->coord_buf);
	hx_s_touch_data->coord_buf = NULL;
}

/*start ts_work*/
#if defined(HX_USB_DETECT_GLOBAL)
void himax_cable_detect_func(bool force_renew)
{
	struct himax_ts_data *ts;

	/*u32 connect_status = 0;*/
	uint8_t connect_status = 0;

	connect_status = USB_detect_flag;/* upmu_is_chr_det(); */
	ts = hx_s_ts;

	/* I("Touch: cable status=%d, cable_config=%p, usb_connected=%d\n",*/
	/*		connect_status, ts->cable_config, ts->usb_connected); */
	if (ts->cable_config) {
		if ((connect_status != ts->usb_connected) || force_renew) {
			if (connect_status) {
				ts->cable_config[1] = 0x01;
				ts->usb_connected = 0x01;
			} else {
				ts->cable_config[1] = 0x00;
				ts->usb_connected = 0x00;
			}

			hx_s_core_fp._usb_detect_set(ts->cable_config);
			I("%s: Cable status change: 0x%2.2X\n", __func__,
					ts->usb_connected);
		}

		/*else*/
		/*	I("%s: Cable status is the same as*/
		/*		previous one, ignore.\n", __func__);*/
	}
}
#endif

static int himax_ts_work_status(struct himax_ts_data *ts)
{
	/* 1: normal, 2:SMWP */
	int result = HX_REPORT_COORD;

	hx_s_touch_data->diag_cmd = ts->diag_cmd;
	if (hx_s_touch_data->diag_cmd)
		result = HX_REPORT_COORD_RAWDATA;

#if defined(HX_SMART_WAKEUP)
	if (atomic_read(&ts->suspend_mode)
	&& (ts->SMWP_enable)
	&& (!hx_s_touch_data->diag_cmd))
		result = HX_REPORT_SMWP_EVENT;
#endif
	/* I("Now Status is %d\n", result); */
	return result;
}

static int himax_touch_get(struct himax_ts_data *ts, uint8_t *buf,
		int ts_path, int ts_status)
{
	uint32_t read_size = 0;

	if (g_ts_dbg != 0) {
		I("%s: Entering, ts_status=%d!\n",
			__func__, ts_status);
		I("%s: Max Size=%d\n",
			__func__, hx_s_touch_data->touch_all_size);
	}

	switch (ts_path) {
	/*normal*/
	case HX_REPORT_COORD:
		if ((g_hw_rst_activate)
#if defined(HX_EXCP_RECOVERY)
			|| (HX_EXCP_RESET_ACTIVATE)
#endif
		) {
			read_size = hx_s_touch_data->touch_all_size;
		} else {
			read_size = hx_s_touch_data->touch_info_size;
		}
		break;
#if defined(HX_SMART_WAKEUP)

	/*SMWP*/
	case HX_REPORT_SMWP_EVENT:
		__pm_wakeup_event(ts->ts_SMWP_wake_lock, TS_WAKE_LOCK_TIMEOUT);
		msleep(20);
		read_size = hx_s_touch_data->event_size;

		break;
#endif
	case HX_REPORT_COORD_RAWDATA:
		read_size = hx_s_touch_data->touch_all_size;
		break;
	default:
		E("%s: size fault!\n", __func__);
		break;
	}

	if (read_size == 0) {
		E("%s: Read size fault!\n", __func__);
		ts_status = HX_TS_GET_DATA_FAIL;
	} else {
		if (!hx_s_core_fp._read_event_stack(buf, read_size)) {
			E("%s: can't read data from chip!\n", __func__);
			ts_status = HX_TS_GET_DATA_FAIL;
		}
	}

	return ts_status;
}

/* start error_control*/
static int himax_checksum_cal(struct himax_ts_data *ts, uint8_t *buf,
		int ts_path, int ts_status)
{
	uint16_t check_sum_cal = 0;
	int32_t	i = 0;
	int length = 0;
	int zero_cnt = 0;
	int raw_data_sel = 0;
	int ret_val = ts_status;

	if (g_ts_dbg != 0)
		I("%s: Entering, ts_status=%d!\n", __func__, ts_status);

	/* Normal */
	switch (ts_path) {
	case HX_REPORT_COORD:
		length = hx_s_touch_data->touch_info_size;
		break;
#if defined(HX_SMART_WAKEUP)
/* SMWP */
	case HX_REPORT_SMWP_EVENT:
		length = (GEST_PTLG_ID_LEN + GEST_PTLG_HDR_LEN);
		break;
#endif
	case HX_REPORT_COORD_RAWDATA:
		length = hx_s_touch_data->touch_info_size;
		break;
	default:
		I("%s, Neither Normal Nor SMWP error!\n", __func__);
		ret_val = HX_PATH_FAIL;
		goto END_FUNCTION;
	}

	for (i = 0; i < length; i++) {
		check_sum_cal += buf[i];
		if (buf[i] == 0x00)
			zero_cnt++;
	}

	if (check_sum_cal % 0x100 != 0) {
		I("point data_checksum not match check_sum_cal: 0x%02X",
			check_sum_cal);
		ret_val = HX_CHKSUM_FAIL;
	} else if (zero_cnt == length) {
		if (ts->use_irq)
			I("[HIMAX TP MSG] All Zero event\n");

		ret_val = HX_CHKSUM_FAIL;
	} else {
		raw_data_sel = buf[HX_TOUCH_INFO_POINT_CNT]>>4 & 0x0F;
		/*I("%s:raw_out_sel=%x , hx_s_touch_data->diag_cmd=%x.\n",*/
		/*		__func__, raw_data_sel,*/
		/*		hx_s_touch_data->diag_cmd);*/
		/*raw data out not match skip it*/
		if (g_ts_dbg != 0)
			I("%s: skip check rawout select, now val=%d!\n",
				__func__, raw_data_sel);
		if (!hx_s_debug_data->
			is_stack_full_raw) {
			if ((raw_data_sel != 0x0F)
			&& (raw_data_sel != hx_s_touch_data->diag_cmd)) {
				/*I("%s:raw data out not match.\n", __func__);*/
				if (!hx_s_touch_data->diag_cmd) {
					/*Need to clear event stack here*/
					hx_s_core_fp._read_event_stack(buf,
						(HX_STACK_ORG_LEN -
						hx_s_touch_data->
							touch_info_size));
				}
				ret_val = HX_READY_SERVE;
			}
		}
	}

END_FUNCTION:
	if (g_ts_dbg != 0)
		I("%s: END, ret_val=%d!\n", __func__, ret_val);
	return ret_val;
}

#if defined(HX_EXCP_RECOVERY)

static int himax_ts_event_check(struct himax_ts_data *ts,
		const uint8_t *buf, int ts_path, int ts_status)
{
	int length = 0;
	int ret_val = ts_status;

	if (g_ts_dbg != 0)
		I("%s: Entering, ts_status=%d!\n", __func__, ts_status);

	/* Normal */
	switch (ts_path) {
	case HX_REPORT_COORD:
		length = hx_s_touch_data->touch_info_size;
		break;
#if defined(HX_SMART_WAKEUP)
/* SMWP */
	case HX_REPORT_SMWP_EVENT:
		length = (GEST_PTLG_ID_LEN + GEST_PTLG_HDR_LEN);
		break;
#endif
	case HX_REPORT_COORD_RAWDATA:
		length = hx_s_touch_data->touch_info_size;
		break;
	default:
		I("%s, Neither Normal Nor SMWP error!\n", __func__);
		ret_val = HX_PATH_FAIL;
		goto END_FUNCTION;
	}

	if (g_ts_dbg != 0)
		I("Now Path=%d, Now status=%d, length=%d\n",
				ts_path, ts_status, length);

	ret_val = hx_s_core_fp._excp_event_chk(ts,
		buf, ts_path, ts_status, length);

END_FUNCTION:
	if (g_ts_dbg != 0)
		I("%s: END, ret_val=%d!\n", __func__, ret_val);

	return ret_val;
}
#endif

static int himax_err_ctrl(struct himax_ts_data *ts,
		uint8_t *buf, int ts_path, int ts_status)
{
#if defined(HX_RST_PIN_FUNC)
	if (g_hw_rst_activate) {
		/* drop 1st interrupts after chip reset */
		g_hw_rst_activate = 0;
		I("[g_hw_rst_activate]%s:Back from reset,ready to serve.\n",
				__func__);
		ts_status = HX_RST_OK;
		goto END_FUNCTION;
	}
#endif

	ts_status = himax_checksum_cal(ts, buf, ts_path, ts_status);
	if (ts_status == HX_CHKSUM_FAIL) {
		goto CHK_FAIL;
	} else {
#if defined(HX_EXCP_RECOVERY)
		/* continuous N times record, not total N times. */
		hx_s_ic_data->zero_event = 0;
#endif
		goto END_FUNCTION;
	}

CHK_FAIL:
#if defined(HX_EXCP_RECOVERY)
	ts_status = himax_ts_event_check(ts, buf, ts_path, ts_status);
#endif

END_FUNCTION:
	if (g_ts_dbg != 0)
		I("%s: END, ts_status=%d!\n", __func__, ts_status);
	return ts_status;
}
/* end error_control*/

/* start distribute_data*/
static int himax_distribute_touch_data(uint8_t *buf,
		int ts_path, int ts_status)
{
	uint8_t hx_state_info_pos = hx_s_touch_data->touch_info_size - 3;

	if (hx_s_ic_data->stylus_func)
		hx_state_info_pos -= STYLUS_INFO_SZ;

	if (g_ts_dbg != 0)
		I("%s: Entering, ts_status=%d!\n", __func__, ts_status);

	if (ts_path == HX_REPORT_COORD) {
		memcpy(hx_s_touch_data->coord_buf, &buf[0],
				hx_s_touch_data->touch_info_size);

		if (buf[hx_state_info_pos] != 0xFF
		&& buf[hx_state_info_pos + 1] != 0xFF)
			memcpy(hx_s_touch_data->state_info,
					&buf[hx_state_info_pos], 2);
		else
			memset(hx_s_touch_data->state_info, 0x00,
					sizeof(hx_s_touch_data->state_info));

		if ((g_hw_rst_activate)
#if defined(HX_EXCP_RECOVERY)
		|| (HX_EXCP_RESET_ACTIVATE)
#endif
		) {
			if (!hx_s_debug_data->
				is_stack_full_raw) {
				memcpy(hx_s_touch_data->rawdata_buf,
						&buf[hx_s_touch_data->
							touch_info_size],
						hx_s_touch_data->touch_all_size
						- hx_s_touch_data->
							touch_info_size);
			} else {
				memcpy(hx_s_touch_data->rawdata_buf,
						&buf[HX_STACK_ORG_LEN],
						hx_s_touch_data->touch_all_size
						- HX_STACK_ORG_LEN);
			}
		}
	} else if (ts_path == HX_REPORT_COORD_RAWDATA) {
		memcpy(hx_s_touch_data->coord_buf, &buf[0],
				hx_s_touch_data->touch_info_size);

		if (buf[hx_state_info_pos] != 0xFF
		&& buf[hx_state_info_pos + 1] != 0xFF)
			memcpy(hx_s_touch_data->state_info,
					&buf[hx_state_info_pos], 2);
		else
			memset(hx_s_touch_data->state_info, 0x00,
					sizeof(hx_s_touch_data->state_info));
		if (!hx_s_debug_data->
			is_stack_full_raw) {
			memcpy(hx_s_touch_data->rawdata_buf,
					&buf[hx_s_touch_data->touch_info_size],
					hx_s_touch_data->touch_all_size
					- hx_s_touch_data->touch_info_size);
		} else {
			memcpy(hx_s_touch_data->rawdata_buf,
					&buf[HX_STACK_ORG_LEN],
					hx_s_touch_data->touch_all_size
					- HX_STACK_ORG_LEN);
		}
		if (g_ts_dbg != 0)
			I("%s:Assign rawdata done!\n", __func__);
#if defined(HX_SMART_WAKEUP)
	} else if (ts_path == HX_REPORT_SMWP_EVENT) {
		memcpy(hx_s_touch_data->event_buf, buf,
				hx_s_touch_data->event_size);
#endif
	} else {
		E("%s, Fail Path!\n", __func__);
		ts_status = HX_PATH_FAIL;
	}

	if (g_ts_dbg != 0)
		I("%s: End, ts_status=%d!\n", __func__, ts_status);
	return ts_status;
}
/* end assign_data*/

#define READ_VAR_BIT(var, nb)			(((var) >> (nb)) & 0x1)

static bool wgp_pen_id_crc(uint8_t *p_id)
{
	uint64_t pen_id, input;
	uint8_t hash_id, devidend;
	uint8_t pol = 0x43;
	int i = 0;

	if (g_ts_dbg != 0) {
		for (i = 0; i < 8; i++)
			I("%s:pen id[%d]= %x\n", __func__,
				i, p_id[i]);
	}
	pen_id = (uint64_t)p_id[0] | ((uint64_t)p_id[1] << 8) |
		((uint64_t)p_id[2] << 16) | ((uint64_t)p_id[3] << 24) |
		((uint64_t)p_id[4] << 32) | ((uint64_t)p_id[5] << 40) |
		((uint64_t)p_id[6] << 48);
	hash_id = p_id[7];
	if (g_ts_dbg != 0)
		I("%s:pen id=%llx, hash id=%x\n", __func__,
			pen_id, hash_id);

	input = pen_id << 6;
	devidend = input >> (44 + 6);

	for (i = (44 + 6 - 1); i >= 0; i--)	{
		if (READ_VAR_BIT(devidend, 6))
			devidend = devidend ^ pol;
		devidend = devidend << 1 | READ_VAR_BIT(input, i);
	}
	if (READ_VAR_BIT(devidend, 6))
		devidend = devidend ^ pol;
	if (g_ts_dbg != 0)
		I("%s:devidend=%x\n", __func__, devidend);

	if (devidend == hash_id) {
		g_target_report_data->s[0].id = pen_id;
		return 1;
	}

	g_target_report_data->s[0].id = 0;
	return 0;
}

/* start parse_report_data*/
static	uint8_t p_id[8];

int himax_parse_report_points(struct himax_ts_data *ts,
		int ts_path, int ts_status)
{
	int x = 0, y = 0, w = 0;
	uint8_t p_hover = 0, p_btn = 0, p_btn2 = 0;
	uint8_t ratio = hx_s_ic_data->stylus_ratio;
	uint8_t p_id_en = 0, p_id_sel = 0;
	int8_t p_tilt_x = 0, p_tilt_y = 0;
	int p_x = 0, p_y = 0, p_w = 0;
	bool ret;
	static uint8_t p_p_on;

	int base = 0;
	int32_t	i = 0;

	if (g_ts_dbg != 0)
		I("%s: start!\n", __func__);

	base = hx_s_touch_data->touch_info_size;

	if (!hx_s_ic_data->stylus_func)
		goto skip_stylus_operation;

	p_p_on = 0;
	base -= STYLUS_INFO_SZ;
	if (hx_s_ic_data->stylus_id_v2) {/*Pen format ver2*/
		p_x = hx_s_touch_data->coord_buf[base] << 8
			| hx_s_touch_data->coord_buf[base + 1];
		p_y = (hx_s_touch_data->coord_buf[base + 2] << 8
			| hx_s_touch_data->coord_buf[base + 3]);
		p_w = (hx_s_touch_data->coord_buf[base + 4] << 8
			| hx_s_touch_data->coord_buf[base + 5]);
		p_tilt_x = (int8_t)hx_s_touch_data->coord_buf[base + 6];
		p_tilt_y = (int8_t)hx_s_touch_data->coord_buf[base + 7];
		p_hover = hx_s_touch_data->coord_buf[base + 8] & 0x01;
		p_btn = hx_s_touch_data->coord_buf[base + 8] & 0x02;
		p_btn2 = hx_s_touch_data->coord_buf[base + 8] & 0x04;
		p_id_en = hx_s_touch_data->coord_buf[base + 8] & 0x08;
		if (!p_id_en) {
			g_target_report_data->s[0].battery_info =
				hx_s_touch_data->coord_buf[base + 9];
			if (g_ts_dbg != 0) {
				I("%s:update battery info = %x\n", __func__,
				g_target_report_data->s[0].battery_info);
			}
		} else {
			p_id_sel =
			(hx_s_touch_data->coord_buf[base + 8] & 0xF0) >> 4;
			if (p_id_sel*2 < 8) {
				p_id[p_id_sel*2] =
				hx_s_touch_data->coord_buf[base + 9];
				p_id[p_id_sel*2 + 1] =
				hx_s_touch_data->coord_buf[base + 10];
				if (g_ts_dbg != 0) {
					I("%s:update pen id, p_id_sel = %d\n",
						__func__,
						p_id_sel);
				}
				if (p_id_sel == 3) {
					ret = wgp_pen_id_crc(p_id);
					if (!ret)
						I("Pen_ID CRC not match\n");
				}
			} else {
				E("%s:p_id offset is wrong=%d\n",
					__func__, p_id_sel);
			}
		}
	} else {/*Pen format ver1*/
		p_x = hx_s_touch_data->coord_buf[base] << 8
			| hx_s_touch_data->coord_buf[base + 1];
		p_y = (hx_s_touch_data->coord_buf[base + 2] << 8
			| hx_s_touch_data->coord_buf[base + 3]);
		p_w = (hx_s_touch_data->coord_buf[base + 4] << 8
			| hx_s_touch_data->coord_buf[base + 5]);
		p_tilt_x = (int8_t)hx_s_touch_data->coord_buf[base + 6];
		p_hover = hx_s_touch_data->coord_buf[base + 7];
		p_btn = hx_s_touch_data->coord_buf[base + 8];
		p_btn2 = hx_s_touch_data->coord_buf[base + 9];
		p_tilt_y = (int8_t)hx_s_touch_data->coord_buf[base + 10];
	}
	if (g_ts_dbg != 0) {
		D("%s: p_x=%d, p_y=%d, p_w=%d,p_tilt_x=%d, p_hover=%d\n",
			__func__, p_x, p_y, p_w, p_tilt_x, p_hover);
		D("%s: p_btn=%d, p_btn2=%d, p_tilt_y=%d\n",
			__func__, p_btn, p_btn2, p_tilt_y);
	}

	if (p_x >= 0
	&& p_x <= ((ts->pdata->abs_x_max+1)*ratio-1)
	&& p_y >= 0
	&& p_y <= ((ts->pdata->abs_y_max+1)*ratio-1)) {
		g_target_report_data->s[0].x = p_x;
		g_target_report_data->s[0].y = p_y;
		g_target_report_data->s[0].w = p_w;
		g_target_report_data->s[0].hover = p_hover;
		g_target_report_data->s[0].btn = p_btn;
		g_target_report_data->s[0].btn2 = p_btn2;
		g_target_report_data->s[0].tilt_x = p_tilt_x;
		g_target_report_data->s[0].tilt_y = p_tilt_y;
		g_target_report_data->s[0].on = 1;
		ts->hx_stylus_num++;
	} else {/* report coordinates */
		g_target_report_data->s[0].x = 0;
		g_target_report_data->s[0].y = 0;
		g_target_report_data->s[0].w = 0;
		g_target_report_data->s[0].hover = 0;
		g_target_report_data->s[0].btn = 0;
		g_target_report_data->s[0].btn2 = 0;
		g_target_report_data->s[0].tilt_x = 0;
		g_target_report_data->s[0].tilt_y = 0;
		g_target_report_data->s[0].on = 0;
	}

	if (g_ts_dbg != 0) {
		if (p_p_on != g_target_report_data->s[0].on) {
			I("s[0].on = %d, hx_stylus_num=%d\n",
				g_target_report_data->s[0].on,
				ts->hx_stylus_num);
			p_p_on = g_target_report_data->s[0].on;
		}
	}
skip_stylus_operation:

	ts->old_finger = ts->pre_finger_mask;
	if (ts->hx_point_num == 0) {
		if (g_ts_dbg != 0)
			I("%s: hx_point_num = 0!\n", __func__);
		return ts_status;
	}
	ts->pre_finger_mask = 0;
	hx_s_touch_data->finger_num =
		hx_s_touch_data->coord_buf[base - 4] & 0x0F;
	hx_s_touch_data->finger_on = 1;
	AA_press = 1;

	g_target_report_data->finger_num = hx_s_touch_data->finger_num;
	g_target_report_data->finger_on = hx_s_touch_data->finger_on;
	g_target_report_data->ig_count =
		hx_s_touch_data->coord_buf[base - 5];

	if (g_ts_dbg != 0)
		I("%s:finger_num = 0x%2X, finger_on = %d\n", __func__,
				g_target_report_data->finger_num,
				g_target_report_data->finger_on);

	for (i = 0; i < ts->nFinger_support; i++) {
		base = i * 4;
		x = hx_s_touch_data->coord_buf[base] << 8
			| hx_s_touch_data->coord_buf[base + 1];
		y = (hx_s_touch_data->coord_buf[base + 2] << 8
			| hx_s_touch_data->coord_buf[base + 3]);
		w = hx_s_touch_data->coord_buf[(ts->nFinger_support * 4) + i];

		if (g_ts_dbg != 0)
			D("%s: now parsing[%d]:x=%d, y=%d, w=%d\n", __func__,
					i, x, y, w);

		if (x >= 0
		&& x <= ts->pdata->abs_x_max
		&& y >= 0
		&& y <= ts->pdata->abs_y_max) {
			hx_s_touch_data->finger_num--;

			g_target_report_data->p[i].x = x;
			g_target_report_data->p[i].y = y;
			g_target_report_data->p[i].w = w;
			g_target_report_data->p[i].id = 1;

			/*I("%s: g_target_report_data->x[loop_i]=%d,*/
			/*g_target_report_data->y[loop_i]=%d,*/
			/*g_target_report_data->w[loop_i]=%d",*/
			/*__func__, g_target_report_data->x[loop_i],*/
			/*g_target_report_data->y[loop_i],*/
			/*g_target_report_data->w[loop_i]); */

			if (!ts->first_pressed) {
				ts->first_pressed = 1;
				I("S1@%d, %d\n", x, y);
			}

			ts->pre_finger_data[i][0] = x;
			ts->pre_finger_data[i][1] = y;

			ts->pre_finger_mask = ts->pre_finger_mask + (1<<i);
		} else {/* report coordinates */
			g_target_report_data->p[i].x = x;
			g_target_report_data->p[i].y = y;
			g_target_report_data->p[i].w = w;
			g_target_report_data->p[i].id = 0;

			if (i == 0 && ts->first_pressed == 1) {
				ts->first_pressed = 2;
				I("E1@%d, %d\n", ts->pre_finger_data[0][0],
						ts->pre_finger_data[0][1]);
			}
		}
	}

	if (g_ts_dbg != 0) {
		for (i = 0; i < ts->nFinger_support; i++)
			D("DBG X=%d  Y=%d ID=%d\n",
				g_target_report_data->p[i].x,
				g_target_report_data->p[i].y,
				g_target_report_data->p[i].id);

		D("DBG finger number %d\n", g_target_report_data->finger_num);
	}

	if (g_ts_dbg != 0)
		I("%s: end!\n", __func__);
	return ts_status;
}

static int himax_parse_report_data(struct himax_ts_data *ts,
		int ts_path, int ts_status)
{

	if (g_ts_dbg != 0)
		I("%s: start now_status=%d!\n", __func__, ts_status);


	EN_NoiseFilter =
		(hx_s_touch_data->coord_buf[HX_TOUCH_INFO_POINT_CNT + 2] >> 3);
	/* I("EN_NoiseFilter=%d\n", EN_NoiseFilter); */
	EN_NoiseFilter = EN_NoiseFilter & 0x01;
	/* I("EN_NoiseFilter2=%d\n", EN_NoiseFilter); */
	p_point_num = ts->hx_point_num;

	if (hx_s_touch_data->coord_buf[HX_TOUCH_INFO_POINT_CNT] == 0xff)
		ts->hx_point_num = 0;
	else
		ts->hx_point_num =
			hx_s_touch_data->coord_buf[HX_TOUCH_INFO_POINT_CNT]
			& 0x0f;

	if (hx_s_ic_data->stylus_func) {
		p_stylus_num = ts->hx_stylus_num;
		ts->hx_stylus_num = 0;
	}

	switch (ts_path) {
	case HX_REPORT_COORD:
		ts_status = himax_parse_report_points(ts, ts_path, ts_status);
		break;
	case HX_REPORT_COORD_RAWDATA:
		/* touch monitor rawdata */
		if (hx_s_debug_data != NULL) {
			if (hx_s_debug_data->is_stack_full_raw) {
				if (g_ts_dbg != 0)
					I("%s: full stack raw!\n", __func__);
				if (hx_s_debug_data->
					_raw_full_stack(hx_s_ic_data,
					hx_s_touch_data))
					I("%s:raw data_checksum not match\n",
						__func__);
			} else{
				if (g_ts_dbg != 0)
					I("%s: event stack raw!\n", __func__);
				if (hx_s_debug_data->_set_diag_cmd(hx_s_ic_data,
					hx_s_touch_data))
					I("%s:raw data_checksum not match\n",
						__func__);
			}

		} else {
			E("%s,There is no init set_diag_cmd\n", __func__);
		}
		ts_status = himax_parse_report_points(ts, ts_path, ts_status);
		break;
#if defined(HX_SMART_WAKEUP)
	case HX_REPORT_SMWP_EVENT:
		himax_wake_event_parse(ts, ts_status);
		break;
#endif
	default:
		E("%s:Fail Path!\n", __func__);
		ts_status = HX_PATH_FAIL;
		break;
	}
	if (g_ts_dbg != 0)
		I("%s: end now_status=%d!\n", __func__, ts_status);
	return ts_status;
}

/* end parse_report_data*/

void himax_report_all_leave_event(struct himax_ts_data *ts)
{
	int i = 0;

	if (g_ts_dbg != 0)
		I("%s: entering!\n", __func__);

	for (i = 0; i < ts->nFinger_support; i++) {
#if !defined(HX_PROTOCOL_A)
		input_mt_slot(ts->input_dev, i);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
		input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);
		input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);
#endif
	}
	input_report_key(ts->input_dev, BTN_TOUCH, 0);
	input_sync(ts->input_dev);
}

/* start report_point*/
static void himax_point_report(struct himax_ts_data *ts)
{
	int i;
	bool valid;


	if (g_ts_dbg != 0) {
		I("%s:start hx_s_touch_data->finger_num=%d\n",
			__func__, hx_s_touch_data->finger_num);
	}
	for (i = 0; i < ts->nFinger_support; i++) {
		if (g_target_report_data->p[i].x >= 0
		&& g_target_report_data->p[i].x <= ts->pdata->abs_x_max
		&& g_target_report_data->p[i].y >= 0
		&& g_target_report_data->p[i].y <= ts->pdata->abs_y_max)
			valid = true;
		else
			valid = false;
		if (g_ts_dbg != 0)
			I("valid=%d\n", valid);
		if (valid) {
			if (g_ts_dbg != 0) {
				I("report_data->x[i]=%d,y[i]=%d,w[i]=%d",
					g_target_report_data->p[i].x,
					g_target_report_data->p[i].y,
					g_target_report_data->p[i].w);
			}
#if !defined(HX_PROTOCOL_A)
			input_mt_slot(ts->input_dev, i);
			input_mt_report_slot_state(ts->input_dev,
					MT_TOOL_FINGER, 1);
#else
			input_report_key(ts->input_dev, BTN_TOUCH, 1);
#endif
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR,
					g_target_report_data->p[i].w);
#if !defined(HX_PROTOCOL_A)
			input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR,
					g_target_report_data->p[i].w);
			input_report_abs(ts->input_dev, ABS_MT_PRESSURE,
					g_target_report_data->p[i].w);
#else
			input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID,
					i + 1);
#endif
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X,
					g_target_report_data->p[i].x);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,
					g_target_report_data->p[i].y);
#if !defined(HX_PROTOCOL_A)
			ts->last_slot = i;
#else
			input_mt_sync(ts->input_dev);
#endif
		} else {
#if !defined(HX_PROTOCOL_A)
			input_mt_slot(ts->input_dev, i);
			input_mt_report_slot_state(ts->input_dev,
					MT_TOOL_FINGER, 0);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
			input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);
			input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);
#endif
		}
	}
#if !defined(HX_PROTOCOL_A)
	input_report_key(ts->input_dev, BTN_TOUCH, 1);
#endif
	input_sync(ts->input_dev);

	if (g_ts_dbg != 0)
		I("%s:end\n", __func__);
}

static void himax_point_leave(struct himax_ts_data *ts)
{
#if !defined(HX_PROTOCOL_A)
	int32_t i = 0;
#endif

	if (g_ts_dbg != 0)
		I("%s: start!\n", __func__);
#if defined(HX_PALM_REPORT)
	if (himax_palm_detect(hx_s_touch_data->coord_buf) == PALM_REPORT) {
		I(" %s HX_PALM_REPORT KEY power event press\n", __func__);
		input_report_key(ts->input_dev, KEY_POWER, 1);
		input_sync(ts->input_dev);
		msleep(100);

		I(" %s HX_PALM_REPORT KEY power event release\n", __func__);
		input_report_key(ts->input_dev, KEY_POWER, 0);
		input_sync(ts->input_dev);
		return;
	}
#endif

	hx_s_touch_data->finger_on = 0;
	g_target_report_data->finger_on  = 0;
	g_target_report_data->finger_num = 0;
	AA_press = 0;

#if defined(HX_PROTOCOL_A)
	input_mt_sync(ts->input_dev);
#endif
#if !defined(HX_PROTOCOL_A)
	for (i = 0; i < ts->nFinger_support; i++) {
		input_mt_slot(ts->input_dev, i);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
		input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);
		input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);
	}
#endif
	if (ts->pre_finger_mask > 0)
		ts->pre_finger_mask = 0;

	if (ts->first_pressed == 1) {
		ts->first_pressed = 2;
		I("E1@%d, %d\n", ts->pre_finger_data[0][0],
				ts->pre_finger_data[0][1]);
	}

	/*if (ts->debug_log_level & BIT(1))*/
	/*	himax_log_touch_event(x, y, w, loop_i, EN_NoiseFilter,*/
	/*			HX_FINGER_LEAVE); */

	input_report_key(ts->input_dev, BTN_TOUCH, 0);
	input_sync(ts->input_dev);

	if (g_ts_dbg != 0)
		I("%s: end!\n", __func__);
}

static void himax_stylus_report(struct himax_ts_data *ts)
{
	bool valid = false;
	uint8_t ratio = hx_s_ic_data->stylus_ratio;

	if (g_ts_dbg != 0) {
		I("%s:start hx_s_touch_data->stylus_num=%d\n",
			__func__, ts->hx_stylus_num);
	}

	if (g_target_report_data->s[0].x >= 0
	&& g_target_report_data->s[0].x <= ((ts->pdata->abs_x_max+1)*ratio-1)
	&& g_target_report_data->s[0].y >= 0
	&& g_target_report_data->s[0].y <= ((ts->pdata->abs_y_max+1)*ratio-1)
	&& (g_target_report_data->s[0].on == 1))
		valid = true;
	else
		valid = false;

	if (g_ts_dbg != 0)
		I("stylus valid=%d\n", valid);

	if (valid) {/*stylus down*/
		if (g_ts_dbg != 0)
			I("s[i].x=%d, s[i].y=%d, s[i].w=%d\n",
					g_target_report_data->s[0].x,
					g_target_report_data->s[0].y,
					g_target_report_data->s[0].w);

		input_report_abs(ts->stylus_dev, ABS_X,
				g_target_report_data->s[0].x);
		input_report_abs(ts->stylus_dev, ABS_Y,
				g_target_report_data->s[0].y);

		if (g_target_report_data->s[0].btn !=
		g_target_report_data->s[0].pre_btn) {
			if (g_ts_dbg != 0)
				I("BTN_STYLUS:%d\n",
					g_target_report_data->s[0].btn);

			input_report_key(ts->stylus_dev, BTN_STYLUS,
					g_target_report_data->s[0].btn);

			g_target_report_data->s[0].pre_btn =
					g_target_report_data->s[0].btn;
		} else {
			if (g_ts_dbg != 0)
				I("BTN_STYLUS status is %d!\n",
						g_target_report_data->s[0].btn);
		}

		if (g_target_report_data->s[0].btn2
		!= g_target_report_data->s[0].pre_btn2) {
			if (g_ts_dbg != 0)
				I("BTN_STYLUS2:%d\n",
					g_target_report_data->s[0].btn2);

			input_report_key(ts->stylus_dev, BTN_STYLUS2,
					g_target_report_data->s[0].btn2);

			g_target_report_data->s[0].pre_btn2 =
					g_target_report_data->s[0].btn2;
		} else {
			if (g_ts_dbg != 0)
				I("BTN_STYLUS2 status is %d!\n",
					g_target_report_data->s[0].btn2);
		}
		input_report_abs(ts->stylus_dev, ABS_TILT_X,
				g_target_report_data->s[0].tilt_x);

		input_report_abs(ts->stylus_dev, ABS_TILT_Y,
				g_target_report_data->s[0].tilt_y);

		input_report_key(ts->stylus_dev, BTN_TOOL_PEN, 1);

		if (g_target_report_data->s[0].hover == 0) {
			input_report_key(ts->stylus_dev, BTN_TOUCH, 1);
			input_report_abs(ts->stylus_dev, ABS_DISTANCE, 0);
			input_report_abs(ts->stylus_dev, ABS_PRESSURE,
					g_target_report_data->s[0].w);
		} else {
			input_report_key(ts->stylus_dev, BTN_TOUCH, 0);
			input_report_abs(ts->stylus_dev, ABS_DISTANCE, 1);
			input_report_abs(ts->stylus_dev, ABS_PRESSURE, 0);
		}
	} else {/*Pen up*/
		g_target_report_data->s[0].pre_btn = 0;
		g_target_report_data->s[0].pre_btn2 = 0;
		input_report_key(ts->stylus_dev, BTN_STYLUS, 0);
		input_report_key(ts->stylus_dev, BTN_STYLUS2, 0);
		input_report_key(ts->stylus_dev, BTN_TOUCH, 0);
		input_report_abs(ts->stylus_dev, ABS_PRESSURE, 0);
		input_sync(ts->stylus_dev);

		input_report_abs(ts->stylus_dev, ABS_DISTANCE, 0);
		input_report_key(ts->stylus_dev, BTN_TOOL_RUBBER, 0);
		input_report_key(ts->stylus_dev, BTN_TOOL_PEN, 0);
		input_report_abs(ts->stylus_dev, ABS_PRESSURE, 0);
	}
	input_sync(ts->stylus_dev);

	if (g_ts_dbg != 0)
		I("%s:end\n", __func__);
}

static void himax_stylus_leave(struct himax_ts_data *ts)
{
	if (g_ts_dbg != 0)
		I("%s: start!\n", __func__);

	g_target_report_data->s[0].pre_btn = 0;
	g_target_report_data->s[0].pre_btn2 = 0;
	input_report_key(ts->stylus_dev, BTN_STYLUS, 0);
	input_report_key(ts->stylus_dev, BTN_STYLUS2, 0);
	input_report_key(ts->stylus_dev, BTN_TOUCH, 0);
	input_report_abs(ts->stylus_dev, ABS_PRESSURE, 0);
	input_sync(ts->stylus_dev);

	input_report_abs(ts->stylus_dev, ABS_DISTANCE, 0);
	input_report_abs(ts->stylus_dev, ABS_TILT_X, 0);
	input_report_abs(ts->stylus_dev, ABS_TILT_Y, 0);
	input_report_key(ts->stylus_dev, BTN_TOOL_RUBBER, 0);
	input_report_key(ts->stylus_dev, BTN_TOOL_PEN, 0);
	input_sync(ts->stylus_dev);

	if (g_ts_dbg != 0)
		I("%s: end!\n", __func__);
}

int himax_report_data(struct himax_ts_data *ts, int ts_path, int ts_status)
{
	if (g_ts_dbg != 0)
		I("%s: Entering, ts_status=%d!\n", __func__, ts_status);

	if (ts_path == HX_REPORT_COORD || ts_path == HX_REPORT_COORD_RAWDATA) {
		/* Touch Point information */

		if (ts->hx_point_num != 0)
			himax_point_report(ts);
		else if ((ts->hx_point_num == 0) && (p_point_num != 0))
			himax_point_leave(ts);

		if (hx_s_ic_data->stylus_func) {
			if (ts->hx_stylus_num != 0)
				himax_stylus_report(ts);
			else if ((ts->hx_stylus_num == 0)
				&& (p_stylus_num != 0))
				himax_stylus_leave(ts);
		}

		Last_EN_NoiseFilter = EN_NoiseFilter;

#if defined(HX_SMART_WAKEUP)
	} else if (ts_path == HX_REPORT_SMWP_EVENT) {
		himax_wake_event_report();
#endif
	} else {
		E("%s:Fail Path!\n", __func__);
		ts_status = HX_PATH_FAIL;
	}

	if (g_ts_dbg != 0)
		I("%s: END, ts_status=%d!\n", __func__, ts_status);
	return ts_status;
}
/* end report_data */

void himax_ts_work(struct himax_ts_data *ts)
{

	int ts_status = HX_TS_NORMAL_END;
	int ts_path = 0;

	if (!ts->initialized) {
		I("%s: Entering, but init doesn't done, skip\n",
			__func__);
		return;
	}

	if (hx_s_debug_data != NULL) {
		if (hx_s_debug_data->is_checking_irq) {
			if (g_ts_dbg != 0)
				I("Now checking IRQ, skip it!\n");
			return;
		}
		hx_s_debug_data->_ts_dbg_func(ts, HX_FINGER_ON);
	}

#if defined(HX_USB_DETECT_GLOBAL)
	himax_cable_detect_func(false);
#endif

	ts_path = himax_ts_work_status(ts);

	memset(ts->xfer_buff,
		0x00,
		hx_s_touch_data->touch_all_size * sizeof(uint8_t));
	ts_status = himax_touch_get(ts, ts->xfer_buff, ts_path, ts_status);
	if (ts_status == HX_TS_GET_DATA_FAIL)
		goto END_FUNCTION;

	ts_status = himax_distribute_touch_data(ts->xfer_buff,
			ts_path, ts_status);
	ts_status = himax_err_ctrl(ts, ts->xfer_buff, ts_path, ts_status);
	if (ts_status == HX_REPORT_DATA || ts_status == HX_TS_NORMAL_END)
		ts_status = himax_parse_report_data(ts, ts_path, ts_status);
	else
		goto END_FUNCTION;

	ts_status = himax_report_data(ts, ts_path, ts_status);

	if (ts_status == HX_TS_GET_DATA_FAIL)
		goto GET_TOUCH_FAIL;
	else
		goto END_FUNCTION;

GET_TOUCH_FAIL:
	I("%s: Now reset the Touch chip.\n", __func__);
	hx_s_core_fp._ic_reset(0);

#if defined(HX_ZERO_FLASH)
	if (hx_s_core_fp._0f_reload_to_active)
		hx_s_core_fp._0f_reload_to_active();
#endif
END_FUNCTION:
	if (hx_s_debug_data != NULL)
		hx_s_debug_data->_ts_dbg_func(ts, HX_FINGER_LEAVE);

}
/*end ts_work*/
enum hrtimer_restart himax_ts_timer_func(struct hrtimer *timer)
{
	struct himax_ts_data *ts;


	ts = container_of(timer, struct himax_ts_data, timer);
	queue_work(ts->himax_wq, &ts->work);
	hrtimer_start(&ts->timer, ktime_set(0, 12500000), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

#if !defined(HX_ZERO_FLASH)
static int hx_chk_flash_sts(uint32_t size)
{
	int rslt = 0;

	I("%s: Entering, %d\n", __func__, size);

	rslt = (!hx_s_core_fp._calculateChecksum(false, size));
	/*avoid the FW is full of zero*/
	rslt |= hx_s_core_fp._flash_lastdata_check(size);

	return rslt;
}
#endif

static void himax_boot_upgrade(struct work_struct *work)
{
#if defined(HX_BOOT_UPGRADE) || defined(HX_ZERO_FLASH)
	int fw_sts = -1;
#endif

#if defined(HX_ZERO_FLASH)
	g_boot_upgrade_flag = true;
#else
	if (hx_chk_flash_sts(hx_s_ic_data->flash_size) == 1) {
		E("%s: check flash fail, please upgrade FW\n", __func__);
	#if defined(HX_BOOT_UPGRADE)
		g_boot_upgrade_flag = true;
	#else
		goto END;
	#endif
	} else {
		hx_s_core_fp._reload_disable(0);
		hx_s_core_fp._power_on_init();
		hx_s_core_fp._read_FW_ver();
		hx_s_core_fp._tp_info_check();
	}
#endif

#if defined(HX_BOOT_UPGRADE) || defined(HX_ZERO_FLASH)
	fw_sts = i_get_FW();
	if (fw_sts < NO_ERR)
		goto END;

	hx_s_core_fp._bin_desc_get((unsigned char *)hxfw->data, HX1K);

#if defined(HX_BOOT_UPGRADE)
	if (g_boot_upgrade_flag == false) {
		if (himax_auto_update_check() == 0)
			g_boot_upgrade_flag = true;
	}
#endif

	if (g_boot_upgrade_flag == true) {
		if (i_update_FW() <= 0) {
			E("%s: Update FW fail\n", __func__);
		} else {
			I("%s: Update FW success\n", __func__);
			if (!g_has_alg_overlay) {
				hx_s_core_fp._reload_disable(0);
				hx_s_core_fp._power_on_init();
			}
				hx_s_core_fp._read_FW_ver();
				hx_s_core_fp._tp_info_check();
		}
	}

	if (fw_sts == NO_ERR)
		release_firmware(hxfw);
	hxfw = NULL;
#endif

END:
	ic_boot_done = 1;
	himax_int_enable(1);
}

#if defined(HX_CONTAINER_SPEED_UP)
static void himax_resume_work_func(struct work_struct *work)
{
	himax_chip_common_resume(hx_s_ts);
}

#endif
int hx_ic_register(void)
{
	int ret = !NO_ERR;

	I("%s:Entering!\n", __func__);

#if defined(CONFIG_TOUCHSCREEN_HIMAX_IC_HX83102E)
	if (_hx83102e_init()) {
		ret = NO_ERR;
		goto END;
	}
#endif

#if defined(CONFIG_TOUCHSCREEN_HIMAX_IC_HX83102G)
	if (_hx83102g_init()) {
		ret = NO_ERR;
		goto END;
	}
#endif

#if defined(CONFIG_TOUCHSCREEN_HIMAX_IC_HX83108)
	if (_hx83108_init()) {
		ret = NO_ERR;
		goto END;
	}
#endif

#if defined(CONFIG_TOUCHSCREEN_HIMAX_IC_HX83122A)
	if (_hx83122a_init()) {
		ret = NO_ERR;
		goto END;
	}
#endif

#if defined(CONFIG_TOUCHSCREEN_HIMAX_IC_HX83123A)
	if (_hx83123a_init()) {
		ret = NO_ERR;
		goto END;
	}
#endif

#if defined(CONFIG_TOUCHSCREEN_HIMAX_IC_HX83102J)
	if (_hx83102j_init()) {
		ret = NO_ERR;
		goto END;
	}
#endif

#if defined(CONFIG_TOUCHSCREEN_HIMAX_IC_HX83112F)
	if (_hx83112f_init()) {
		ret = NO_ERR;
		goto END;
	}
#endif


#if defined(CONFIG_TOUCHSCREEN_HIMAX_IC_HX83121A)
	if (_hx83121a_init()) {
		ret = NO_ERR;
		goto END;
	}
#endif

#if defined(CONFIG_TOUCHSCREEN_HIMAX_IC_HX85200A)
	if (_hx85200a_init()) {
		ret = NO_ERR;
		goto END;
	}
#endif

#if defined(CONFIG_TOUCHSCREEN_HIMAX_IC_HX83132A)
	if (_hx83132a_init()) {
		ret = NO_ERR;
		goto END;
	}
#endif

END:
	if (ret == NO_ERR)
		I("%s: detect IC!\n", __func__);
	else
		E("%s: There is no IC!\n", __func__);
	I("%s:END!\n", __func__);

	return ret;
}

#if defined(HX_FIRMWARE_HEADER)
void mapping_panel_id_from_dt(struct device_node *dt)
{
	const char *panel_id_dt = "panel_id";
	uint32_t data = 0;

	if (of_property_read_u32(dt, panel_id_dt, &data) == 0) {
		I(" Found %s = %d\n", panel_id_dt, data);
		g_hx_panel_id = data;
	} else {
		I(" %s not found! set to default 0\n", panel_id_dt);
		g_hx_panel_id = 0;
	}
	I(" DT:panel_id = %d\n", g_hx_panel_id);
}
#endif

int himax_chip_common_init(void)
{
	int ret = 0;
	int err = PROBE_FAIL;
	struct himax_ts_data *ts = hx_s_ts;
	struct himax_platform_data *pdata;


	ts->xfer_buff = devm_kzalloc(ts->dev,
		HX_FULL_STACK_RAWDATA_SIZE * sizeof(uint8_t),
		GFP_KERNEL);

	if (ts->xfer_buff == NULL) {
		err = -ENOMEM;
		goto err_xfer_buff_fail;
	}

	I("PDATA START\n");
	pdata = kzalloc(sizeof(struct himax_platform_data), GFP_KERNEL);

	if (pdata == NULL) { /*Allocate Platform data space*/
		err = -ENOMEM;
		goto err_dt_platform_data_fail;
	}

	I("hx_s_ic_data START\n");
	hx_s_ic_data = kzalloc(sizeof(struct himax_ic_data), GFP_KERNEL);
	if (hx_s_ic_data == NULL) { /*Allocate IC data space*/
		err = -ENOMEM;
		goto err_dt_ic_data_fail;
	}
	memset(hx_s_ic_data, 0xFF, sizeof(struct himax_ic_data));
	/* default 128k, different size please follow HX83121A style */
	hx_s_ic_data->flash_size = 131072;

	/* allocate report data */
	hx_s_touch_data = kzalloc(sizeof(struct himax_report_data), GFP_KERNEL);
	if (hx_s_touch_data == NULL) {
		err = -ENOMEM;
		goto err_alloc_touch_data_failed;
	}

	hx_s_fwbin = kzalloc(sizeof(struct hx_fwbin_info), GFP_KERNEL);
	if (hx_s_fwbin == NULL) {
		err = -ENOMEM;
		goto err_alloc_touch_data_failed;
	}

	ts->pdata = pdata;

	if (himax_parse_dt(ts, pdata) < 0) {
		I(" pdata is NULL for DT\n");
		goto err_alloc_dt_pdata_failed;
	}

#if defined(HX_RST_PIN_FUNC)
	ts->rst_gpio = pdata->gpio_reset;
#endif

	if (himax_gpio_power_config(pdata) < 0) {
		E("%s: gpio config fail, exit!\n", __func__);
		goto err_alloc_dt_pdata_failed;
	}

#if !defined(CONFIG_OF)
	if (pdata->power) {
		ret = pdata->power(1);
		if (ret < 0) {
			E("%s: power on failed\n", __func__);
			goto err_power_failed;
		}
	}
#endif

#if defined(CONFIG_OF)
	ts->power = pdata->power;
#endif


	if (hx_ic_register() != NO_ERR) {
		E("%s: can't detect IC!\n", __func__);
		goto error_ic_detect_failed;
	}


	if (hx_s_core_fp._chip_init != NULL) {
		hx_s_core_fp._chip_init();
	} else {
		E("%s: function point of chip_init is NULL!\n", __func__);
		goto error_ic_detect_failed;
	}

	ts->proj_name = NULL;
	/* Assign proj name*/

#if defined(HX_BOOT_UPGRADE) || defined(HX_ZERO_FLASH)
	ts->boot_fw_name = BOOT_UPGRADE_FWNAME;
#endif
#if defined(HX_ZERO_FLASH)
	ts->mp_fw_name = MPAP_FWNAME;
#endif

	himax_parse_dt_ic_info(ts, pdata);

	hx_s_core_fp._touch_information();

	spin_lock_init(&ts->irq_lock);

	if (himax_ts_register_interrupt()) {
		E("%s: register interrupt failed\n", __func__);
		goto err_register_interrupt_failed;
	}

	himax_int_enable(0);

#if defined(HX_ZERO_FLASH)
	g_update_cfg_buf = kzalloc(hx_s_ic_data->dsram_sz[0] *
		sizeof(uint8_t),
		GFP_KERNEL);
		if (g_update_cfg_buf == NULL) {
			err = -ENOMEM;
			goto err_update_cfg_buf_adlled;
		}
#endif

	ts->himax_boot_upgrade_wq =
		create_singlethread_workqueue("HX_boot_upgrade");
	if (!ts->himax_boot_upgrade_wq) {
		E("allocate himax_boot_upgrade_wq failed\n");
		err = -ENOMEM;
		goto err_boot_upgrade_wq_failed;
	}
	INIT_DELAYED_WORK(&ts->work_boot_upgrade, himax_boot_upgrade);
	queue_delayed_work(ts->himax_boot_upgrade_wq, &ts->work_boot_upgrade,
			msecs_to_jiffies(HX_DELAY_BOOT_UPDATE));

	hx_s_core_fp._calc_touch_data_size();

	/*Himax Power On and Load Config*/
/*	if (himax_loadSensorConfig(pdata)) {
 *		E("%s: Load Sesnsor configuration failed, unload driver.\n",
 *				__func__);
 *		goto err_detect_failed;
 *	}
 */

#if defined(CONFIG_OF)
	ts->pdata->abs_pressure_min        = 0;
	ts->pdata->abs_pressure_max        = 200;
	ts->pdata->abs_width_min           = 0;
	ts->pdata->abs_width_max           = 200;
	pdata->cable_config[0]             = 0xF0;
	pdata->cable_config[1]             = 0x00;
#endif

	ts->suspended                      = false;

#if defined(HX_USB_DETECT_GLOBAL)
	ts->usb_connected = 0x00;
	ts->cable_config = pdata->cable_config;
#endif

#if defined(HX_PROTOCOL_A)
	ts->protocol_type = PROTOCOL_TYPE_A;
#else
	ts->protocol_type = PROTOCOL_TYPE_B;
#endif
	I("%s: Use Protocol Type %c\n", __func__,
		ts->protocol_type == PROTOCOL_TYPE_A ? 'A' : 'B');

#if defined(HX_SMART_WAKEUP)
	ts->SMWP_enable = 0;
#if defined(KERNEL_VER_ABOVE_4_19)
	ts->ts_SMWP_wake_lock =
		wakeup_source_register(ts->dev, HIMAX_common_NAME);
#else
	if (!ts->ts_SMWP_wake_lock)
		ts->ts_SMWP_wake_lock = kzalloc(sizeof(struct wakeup_source),
			GFP_KERNEL);

	if (!ts->ts_SMWP_wake_lock) {
		E("%s: allocate ts_SMWP_wake_lock failed\n", __func__);
		goto err_smwp_wake_lock_failed;
	}

	wakeup_source_init(ts->ts_SMWP_wake_lock, HIMAX_common_NAME);
#endif
#endif

#if defined(HX_HIGH_SENSE)
	ts->HSEN_enable = 0;
#endif

	if (himax_common_proc_init()) {
		E(" %s: himax_common proc_init failed!\n", __func__);
		goto err_creat_proc_file_failed;
	}

#if defined(CONFIG_TOUCHSCREEN_HIMAX_DEBUG)
	if (himax_debug_init()) {
		E(" %s: debug initial failed!\n", __func__);
		goto err_debug_init_failed;
	}
#endif

	ret = himax_input_register(ts);
	if (ret) {
		E("%s: Unable to register %s input device\n",
			__func__, ts->input_dev->name);
		goto err_input_register_device_failed;
	}

#if defined(HX_CONTAINER_SPEED_UP)
	ts->ts_int_workqueue =
			create_singlethread_workqueue("himax_ts_resume_wq");
	if (!ts->ts_int_workqueue) {
		E("create ts_resume workqueue failed\n");
		goto err_create_ts_resume_wq_failed;
	}
	INIT_DELAYED_WORK(&ts->ts_int_work, himax_resume_work_func);
#endif

	ts->initialized = true;

	I("%s: Now load:%s\n", __func__, HIMAX_DRIVER_VER);

	return 0;

#if defined(HX_CONTAINER_SPEED_UP)
err_create_ts_resume_wq_failed:
#endif
	input_unregister_device(ts->input_dev);
	if (hx_s_ic_data->stylus_func)
		input_unregister_device(ts->stylus_dev);
err_input_register_device_failed:
#if defined(CONFIG_TOUCHSCREEN_HIMAX_DEBUG)
	himax_debug_remove();
err_debug_init_failed:
#endif
	himax_common_proc_deinit();
err_creat_proc_file_failed:
#if defined(HX_SMART_WAKEUP)
#if defined(KERNEL_VER_ABOVE_4_19)
	wakeup_source_unregister(ts->ts_SMWP_wake_lock);
#else
	wakeup_source_trash(ts->ts_SMWP_wake_lock);
	kfree(ts->ts_SMWP_wake_lock);
	ts->ts_SMWP_wake_lock = NULL;
err_smwp_wake_lock_failed:
#endif
#endif
	cancel_delayed_work_sync(&ts->work_boot_upgrade);
	destroy_workqueue(ts->himax_boot_upgrade_wq);
err_boot_upgrade_wq_failed:
	himax_ts_unregister_interrupt();
#if defined(HX_ZERO_FLASH)
	kfree(g_update_cfg_buf);
err_update_cfg_buf_adlled:
#endif
err_register_interrupt_failed:
/*err_detect_failed:*/
error_ic_detect_failed:
#if !defined(CONFIG_OF)
err_power_failed:
#endif
	himax_gpio_power_deconfig(pdata);
err_alloc_dt_pdata_failed:
	kfree(hx_s_touch_data);
	hx_s_touch_data = NULL;
err_alloc_touch_data_failed:
	kfree(hx_s_ic_data);
	hx_s_ic_data = NULL;
err_dt_ic_data_fail:
	kfree(pdata);
	pdata = NULL;
err_dt_platform_data_fail:
	devm_kfree(ts->dev, ts->xfer_buff);
	ts->xfer_buff = NULL;
err_xfer_buff_fail:
	probe_fail_flag = 1;
	return err;
}

void himax_chip_common_deinit(void)
{
	struct himax_ts_data *ts = hx_s_ts;

#if defined(HX_ZERO_FLASH)
	if (g_update_cfg_buf != NULL)
		kfree(g_update_cfg_buf);
#endif

	himax_ts_unregister_interrupt();

#if defined(CONFIG_TOUCHSCREEN_HIMAX_INSPECT)
	himax_inspect_data_clear();
#endif

#if defined(CONFIG_TOUCHSCREEN_HIMAX_DEBUG)
	himax_debug_remove();
#endif

	himax_common_proc_deinit();
	himax_report_data_deinit();

#if defined(HX_SMART_WAKEUP)
#if defined(KERNEL_VER_ABOVE_4_19)
	wakeup_source_unregister(ts->ts_SMWP_wake_lock);
#else
	wakeup_source_trash(ts->ts_SMWP_wake_lock);
	kfree(ts->ts_SMWP_wake_lock);
	ts->ts_SMWP_wake_lock = NULL;
#endif
#endif

	input_unregister_device(ts->input_dev);
	if (hx_s_ic_data->stylus_func)
		input_unregister_device(ts->stylus_dev);
#if defined(HX_CONTAINER_SPEED_UP)
	cancel_delayed_work_sync(&ts->ts_int_work);
	destroy_workqueue(ts->ts_int_workqueue);
#endif

#if defined(HX_BOOT_UPGRADE) || defined(HX_ZERO_FLASH)
	cancel_delayed_work_sync(&ts->work_boot_upgrade);
	destroy_workqueue(ts->himax_boot_upgrade_wq);
#endif
	himax_gpio_power_deconfig(ts->pdata);

	kfree(hx_s_touch_data);
	hx_s_touch_data = NULL;
	kfree(hx_s_ic_data);
	hx_s_ic_data = NULL;
	devm_kfree(ts->dev, ts->xfer_buff);
	ts->xfer_buff = NULL;
	kfree(ts->pdata);
	ts->pdata = NULL;
	kfree(ts);
	ts = NULL;
	probe_fail_flag = 0;
#if defined(HX_USE_KSYM)
	hx_release_chip_entry();
#endif

	I("%s: Common section deinited!\n", __func__);
}

int himax_chip_common_suspend(struct himax_ts_data *ts)
{
	if (ts->suspended) {
		I("%s: Already suspended, skip...\n", __func__);
		goto END;
	} else {
		ts->suspended = true;
	}

	if (hx_s_debug_data != NULL
		&& hx_s_debug_data->flash_dump_going == true) {
		I("%s: It is dumping flash, reject suspend\n", __func__);
		goto END;
	}

	I("%s: enter\n", __func__);

	if (ts->in_self_test == 1) {
		atomic_set(&ts->suspend_mode, 1);
		ts->pre_finger_mask = 0;
		if (hx_s_core_fp._ap_notify_fw_sus != NULL)
			hx_s_core_fp._ap_notify_fw_sus(1);
		ts->suspend_resume_done = 1;
		goto END;
	}

	hx_s_core_fp._suspend_proc(ts->suspended);

	if (ts->SMWP_enable) {
		if (hx_s_core_fp._0f_overlay != NULL)
			hx_s_core_fp._0f_overlay(2, 0);

		if (hx_s_core_fp._ap_notify_fw_sus != NULL)
			hx_s_core_fp._ap_notify_fw_sus(1);
		atomic_set(&ts->suspend_mode, 1);
		ts->pre_finger_mask = 0;
		I("%s: SMART WAKE UP enable, reject suspend\n", __func__);
		goto END;
	}

	himax_int_enable(0);
	if (hx_s_core_fp._suspend_ic_action != NULL)
		hx_s_core_fp._suspend_ic_action();

	if (!ts->use_irq) {
		int32_t cancel_state;

		cancel_state = cancel_work_sync(&ts->work);
		if (cancel_state)
			himax_int_enable(1);
	}

#if defined(HX_RST_PIN_FUNC)
	if (himax_int_gpio_read(hx_s_ts->rst_gpio)) {
		I("%s:Now RST pin is High, trigger Low!\n", __func__);
		himax_mcu_pin_reset(1);
	}
#endif

	/*ts->first_pressed = 0;*/
	atomic_set(&ts->suspend_mode, 1);
	ts->pre_finger_mask = 0;

	if (ts->pdata)
		if (ts->pdata->powerOff3V3 && ts->pdata->power)
			ts->pdata->power(0);

END:
	himax_report_all_leave_event(ts);
	I("%s: END\n", __func__);

	return 0;
}

int himax_chip_common_resume(struct himax_ts_data *ts)
{
	if (ts->suspended == false) {
		I("%s: Already resumed, skip...\n", __func__);
		goto END;
	} else {
		ts->suspended = false;
	}

	I("%s: enter\n", __func__);

	if (ts->in_self_test == 1) {
		atomic_set(&ts->suspend_mode, 0);
		ts->diag_cmd = 0;
		if (hx_s_core_fp._ap_notify_fw_sus != NULL)
			hx_s_core_fp._ap_notify_fw_sus(0);
		ts->suspend_resume_done = 1;
		goto END;
	}

#if defined(HX_EXCP_RECOVERY)
	/* continuous N times record, not total N times. */
	hx_s_ic_data->zero_event = 0;
#endif

	atomic_set(&ts->suspend_mode, 0);
	ts->diag_cmd = 0;
	if (ts->SMWP_enable)
		himax_int_enable(0);
	if (ts->pdata)
		if (ts->pdata->powerOff3V3 && ts->pdata->power)
			ts->pdata->power(1);
#if defined(HX_RST_PIN_FUNC) || defined(HX_RESUME_HW_RESET)
	if (hx_s_core_fp._ic_reset != NULL)
		hx_s_core_fp._ic_reset(0);
#endif

	hx_s_core_fp._resume_proc(ts->suspended);
	himax_report_all_leave_event(ts);
	himax_int_enable(1);
/*
 *#if defined(HX_ZERO_FLASH) && defined(HX_RESUME_SET_FW)
 *ESCAPE_0F_UPDATE:
 *#*endif
 */
END:

	I("%s: END\n", __func__);
	return 0;
}
