// SPDX-License-Identifier: GPL-2.0+
/*
 * Jadard Touch IC driver
 *
 * Copyright (c) 2018-2025 Jadard Technology Corporation.
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
#include "jadard_common.h"
#include "jadard_module.h"
#include "jadard_debug.h"

extern struct jadard_module_fp g_module_fp;
extern struct jadard_ts_data *pjadard_ts_data;
extern struct jadard_ic_data *pjadard_ic_data;
extern struct jadard_report_data *pjadard_report_data;
extern struct jadard_host_data *pjadard_host_data;
extern struct jadard_debug *pjadard_debug;
extern struct proc_dir_entry *pjadard_touch_proc_dir;
extern struct jadard_common_variable g_common_variable;

#if !defined(JD_ZERO_FLASH) && defined(JD_UPGRADE_FW_ARRAY)
extern char *jd_i_CTPM_firmware_name;
extern uint8_t jd_i_firmware[];
extern uint32_t jd_fw_size;
#endif

static struct timespec64 timeStart, timeEnd, timeDelta;

void jadard_log_touch_data(uint8_t finger_event)
{
	int i = 0;
	int coord_size = pjadard_report_data->touch_coord_size;
	uint8_t *buf = pjadard_report_data->touch_coord_info;

	if (finger_event == JD_ATTN_OUT) {
		JD_I("finger_num = %d\n", pjadard_host_data->finger_num);

		if (buf != NULL) {
			for (i = 0; i < coord_size; i += 5) {
				if ((i + 4) < coord_size) {
					JD_I("[%2d] 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X\n",
						i, buf[i + 0], buf[i + 1], buf[i + 2], buf[i + 3], buf[i + 4]);
				}
			}
		}

		g_module_fp.fp_log_touch_state();
	}
}

void jadard_log_touch_int_devation(int finger_event)
{
	if (finger_event == JD_ATTN_IN) {
		ktime_get_ts64(&timeStart);
	} else if (finger_event == JD_ATTN_OUT) {
		ktime_get_ts64(&timeEnd);
		timeDelta.tv_nsec = (timeEnd.tv_sec * 1000000000 + timeEnd.tv_nsec) -
							(timeStart.tv_sec * 1000000000 + timeStart.tv_nsec);
		JD_I("Touch latency = %ld us\n", timeDelta.tv_nsec / 1000);
	}
}

void jadard_log_touch_event(struct jadard_ts_data *ts, uint8_t finger_event)
{
	int i = 0;

	if (finger_event == JD_ATTN_OUT) {
		for (i = 0; i < pjadard_ic_data->JD_MAX_PT; i++) {
			if (pjadard_host_data->event[i] == JD_DOWN_EVENT) {
				JD_I("status: ID:%d Down, X:%d, Y:%d, W:%d\n", pjadard_host_data->id[i],
					pjadard_host_data->x[i], pjadard_host_data->y[i], pjadard_host_data->w[i]);
			} else if (pjadard_host_data->event[i] == JD_UP_EVENT) {
				JD_I("status: ID:%d	  Up, X:%d, Y:%d\n", pjadard_host_data->id[i],
					pjadard_host_data->x[i], pjadard_host_data->y[i]);
			}
		}
	}
}

void jadard_touch_dbg_func(struct jadard_ts_data *ts, uint8_t start)
{
	switch (ts->debug_log_touch_level) {
	case 1:
		jadard_log_touch_data(start);
		break;
	case 2:
		jadard_log_touch_int_devation(start);
		break;
	case 3:
		jadard_log_touch_event(ts, start);
		break;
	}
}

static ssize_t jadard_debug_level_read(struct file *file, char *buf,
										size_t len, loff_t *pos)
{
	size_t ret = 0;
	char *buf_tmp = NULL;

	if (len < 64) {
		JD_E("%s: len size less than 64\n", __func__);
		return -ENOMEM;
	}

	if (!pjadard_debug->proc_send_flag) {
		buf_tmp = kzalloc(len * sizeof(char), GFP_KERNEL);

		if (buf_tmp != NULL) {
			ret += scnprintf(buf_tmp + ret, len - ret, "debug_log_touch_level = %d\n",
						pjadard_ts_data->debug_log_touch_level);

			if (copy_to_user(buf, buf_tmp, len))
				JD_E("%s, copy_to_user fail: %d\n", __func__, __LINE__);

			kfree(buf_tmp);
		} else {
			JD_E("%s, Memory allocate fail: %d\n", __func__, __LINE__);
		}

		pjadard_debug->proc_send_flag = true;
	} else {
		pjadard_debug->proc_send_flag = false;
	}

	return ret;
}

static ssize_t jadard_debug_level_write(struct file *file, const char *buf,
										size_t len, loff_t *pos)
{
	struct jadard_ts_data *ts = pjadard_ts_data;
	char buf_tmp[10] = {0};

	if (len >= sizeof(buf_tmp)) {
		JD_I("%s: no command exceeds %ld chars.\n", __func__, sizeof(buf_tmp));
		return -EFAULT;
	}

	if (copy_from_user(buf_tmp, buf, (len < sizeof(buf_tmp) - 1) ? len : sizeof(buf_tmp) - 1))
		return -EFAULT;

	ts->debug_log_touch_level = 0;

	if (buf_tmp[0] >= '0' && buf_tmp[0] <= '3') {
		ts->debug_log_touch_level = (buf_tmp[0] - '0');
	} else {
		JD_E("%s: Not support command!\n", __func__);
		return -EINVAL;
	}

	return len;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0)
static struct proc_ops jadard_proc_debug_level_ops = {
	.proc_read = jadard_debug_level_read,
	.proc_write = jadard_debug_level_write,
};
#else
static struct file_operations jadard_proc_debug_level_ops = {
	.owner = THIS_MODULE,
	.read = jadard_debug_level_read,
	.write = jadard_debug_level_write,
};
#endif

static ssize_t jadard_fw_package_read(struct file *file, char *buf,
										size_t len, loff_t *pos)
{
	size_t ret = 0;
	char *buf_tmp = NULL;
	int i = 0, tmp, packet_len;

	if (len < 1024) {
		JD_E("%s: len size less than 1024\n", __func__);
		return -ENOMEM;
	}

	if (!pjadard_debug->proc_send_flag) {
		buf_tmp = kzalloc(len * sizeof(char), GFP_KERNEL);

		if (buf_tmp != NULL) {
		#if defined(JD_SANSUMG_PALM_EN)
			packet_len = JD_HID_TOUCH_DATA_SIZE;
		#else
			if (pjadard_ic_data->JD_STYLUS_EN)
				packet_len = JD_HID_TOUCH_DATA_SIZE;
			else
				packet_len = JD_HID_TOUCH_DATA_SIZE - JD_TOUCH_STYLUS_SIZE;
		#endif

			for (i = 0; i < packet_len; i++) {
				tmp = i % 5;

				if (tmp == 0) {
					ret += scnprintf(buf_tmp + ret, len - ret, "[%2d] 0x%02X",
									i, pjadard_report_data->touch_all_info[i]);
				} else if (tmp == 4) {
					ret += scnprintf(buf_tmp + ret, len - ret, " 0x%02X\n",
									pjadard_report_data->touch_all_info[i]);
				} else {
					ret += scnprintf(buf_tmp + ret, len - ret, " 0x%02X",
									pjadard_report_data->touch_all_info[i]);
				}
			}
			ret += scnprintf(buf_tmp + ret, len - ret, "\n");

			if (copy_to_user(buf, buf_tmp, len))
				JD_E("%s, copy_to_user fail: %d\n", __func__, __LINE__);

			kfree(buf_tmp);
		} else {
			JD_E("%s, Memory allocate fail: %d\n", __func__, __LINE__);
		}

		pjadard_debug->proc_send_flag = true;
	} else {
		pjadard_debug->proc_send_flag = false;
	}

	return ret;
}

static ssize_t jadard_fw_package_write(struct file *file, const char *buf,
										size_t len, loff_t *pos)
{
	struct jadard_ts_data *ts = pjadard_ts_data;
	char buf_tmp[10] = {0};

	if (len >= sizeof(buf_tmp)) {
		JD_I("%s: no command exceeds %ld chars.\n", __func__, sizeof(buf_tmp));
		return -EFAULT;
	}

	if (copy_from_user(buf_tmp, buf, (len < sizeof(buf_tmp) - 1) ? len : sizeof(buf_tmp) - 1))
		return -EFAULT;

	if (buf_tmp[0] >= '1') {
		pjadard_report_data->report_rate_count = 0;
		pjadard_report_data->touch_all_info[1] = 0;
		pjadard_report_data->touch_all_info[2] = 0;
		ts->debug_fw_package_enable = true;
	} else {
		ts->debug_fw_package_enable = false;
	}

	return len;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0)
static struct proc_ops jadard_proc_fw_package_ops = {
	.proc_read = jadard_fw_package_read,
	.proc_write = jadard_fw_package_write,
};
#else
static struct file_operations jadard_proc_fw_package_ops = {
	.owner = THIS_MODULE,
	.read = jadard_fw_package_read,
	.write = jadard_fw_package_write,
};
#endif

static ssize_t jadard_reset_write(struct file *file, const char *buf,
									size_t len, loff_t *pos)
{
	char buf_tmp[10] = {0};

	if (len >= sizeof(buf_tmp)) {
		JD_I("%s: no command exceeds %ld chars.\n", __func__, sizeof(buf_tmp));
		return -EFAULT;
	}

	if (copy_from_user(buf_tmp, buf, (len < sizeof(buf_tmp) - 1) ? len : sizeof(buf_tmp) - 1))
		return -EFAULT;

#ifdef JD_RST_PIN_FUNC
	if (buf_tmp[0] == '1') {
		g_module_fp.fp_ic_reset(false, false);
	} else if (buf_tmp[0] == '2') {
		g_module_fp.fp_ic_reset(false, true);
	} else if (buf_tmp[0] == '3') {
		g_module_fp.fp_ic_reset(true, false);
	} else if (buf_tmp[0] == '4') {
		g_module_fp.fp_ic_reset(true, true);
	} else if (buf_tmp[0] == 's') {
#else
	if (buf_tmp[0] == 's') {
#endif
		g_module_fp.fp_ic_soft_reset();
	} else {
		JD_E("%s: Not support command!\n", __func__);
		return -EINVAL;
	}

	return len;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0)
static struct proc_ops jadard_proc_reset_ops = {
	.proc_write = jadard_reset_write,
};
#else
static struct file_operations jadard_proc_reset_ops = {
	.owner = THIS_MODULE,
	.write = jadard_reset_write,
};
#endif

static ssize_t jadard_attn_read(struct file *file, char *buf,
								size_t len, loff_t *pos)
{
	ssize_t ret = 0;
	char *buf_tmp = NULL;

	if (len < 16) {
		JD_E("%s: len size less than 16\n", __func__);
		return -ENOMEM;
	}

	if (!pjadard_debug->proc_send_flag) {
		buf_tmp = kzalloc(len * sizeof(char), GFP_KERNEL);
		if (buf_tmp != NULL) {
			ret += scnprintf(buf_tmp + ret, len - ret, "attn = %d\n",
							gpio_get_value(pjadard_ts_data->pdata->gpio_irq));

			if (copy_to_user(buf, buf_tmp, len))
				JD_E("%s, copy_to_user fail: %d\n", __func__, __LINE__);

			kfree(buf_tmp);
		} else {
			JD_E("%s, Memory allocate fail: %d\n", __func__, __LINE__);
		}

		pjadard_debug->proc_send_flag = true;
	} else {
		pjadard_debug->proc_send_flag = false;
	}

	return ret;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0)
static struct proc_ops jadard_proc_attn_ops = {
	.proc_read = jadard_attn_read,
};
#else
static struct file_operations jadard_proc_attn_ops = {
	.owner = THIS_MODULE,
	.read = jadard_attn_read,
};
#endif

static ssize_t jadard_int_en_read(struct file *file, char *buf,
									size_t len, loff_t *pos)
{
	struct jadard_ts_data *ts = pjadard_ts_data;
	size_t ret = 0;
	char *buf_tmp = NULL;

	if (len < 16) {
		JD_E("%s: len size less than 16\n", __func__);
		return -ENOMEM;
	}

	if (!pjadard_debug->proc_send_flag) {
		buf_tmp = kzalloc(len * sizeof(char), GFP_KERNEL);
		if (buf_tmp != NULL) {
			ret += scnprintf(buf_tmp + ret, len - ret, "irq = %d\n", ts->irq_enabled);

			if (copy_to_user(buf, buf_tmp, len))
				JD_E("%s, copy_to_user fail: %d\n", __func__, __LINE__);

			kfree(buf_tmp);
		} else {
			JD_E("%s, Memory allocate fail: %d\n", __func__, __LINE__);
		}

		pjadard_debug->proc_send_flag = true;
	} else {
		pjadard_debug->proc_send_flag = false;
	}

	return ret;
}

static ssize_t jadard_int_en_write(struct file *file, const char *buf,
									size_t len, loff_t *pos)
{
	char buf_tmp[10] = {0};

	if (len >= sizeof(buf_tmp)) {
		JD_I("%s: no command exceeds %ld chars.\n", __func__, sizeof(buf_tmp));
		return -EFAULT;
	}

	if (copy_from_user(buf_tmp, buf, (len < sizeof(buf_tmp) - 1) ? len : sizeof(buf_tmp) - 1))
		return -EFAULT;

	if (buf_tmp[0] == '0') {
		jadard_int_en_set(false);
	} else if (buf_tmp[0] == '1') {
		jadard_int_en_set(true);
	} else {
		JD_E("%s: Not support command!\n", __func__);
		return -EINVAL;
	}

	return len;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0)
static struct proc_ops jadard_proc_int_en_ops = {
	.proc_read = jadard_int_en_read,
	.proc_write = jadard_int_en_write,
};
#else
static struct file_operations jadard_proc_int_en_ops = {
	.owner = THIS_MODULE,
	.read = jadard_int_en_read,
	.write = jadard_int_en_write,
};
#endif

static void jadard_diag_arrange_print(struct seq_file *s, bool transpose, int i, int j,
									struct jadard_diag_mutual_data *dump)
{
	if (transpose) {
		if (dump == NULL) {
			seq_printf(s, "%7d", jd_diag_mutual[i * pjadard_ic_data->JD_Y_NUM + j]);
		} else {
			dump->write_len += scnprintf(dump->buf + dump->write_len,
				dump->buf_len - dump->write_len, "%7d", jd_diag_mutual[i * pjadard_ic_data->JD_Y_NUM + j]);
		}
	} else {
		if (dump == NULL) {
			seq_printf(s, "%7d", jd_diag_mutual[j * pjadard_ic_data->JD_Y_NUM + i]);
		} else {
			dump->write_len += scnprintf(dump->buf + dump->write_len,
				dump->buf_len - dump->write_len, "%7d", jd_diag_mutual[j * pjadard_ic_data->JD_Y_NUM + i]);
		}
	}
}

static void jadard_diag_arrange_row(struct seq_file *s, bool transpose, int i, int row_num,
									struct jadard_diag_mutual_data *dump)
{
	int j, in_loop;

	if (transpose)
		in_loop = pjadard_ic_data->JD_Y_NUM;
	else
		in_loop = pjadard_ic_data->JD_X_NUM;

	if (row_num > 0) {
		for (j = row_num - 1; j >= 0; j--)
			jadard_diag_arrange_print(s, transpose, i, j, dump);

		if (dump == NULL) {
			seq_printf(s, "\n");
		} else {
			dump->write_len += scnprintf(dump->buf + dump->write_len,
								dump->buf_len - dump->write_len, "\n");
		}
	} else {
		for (j = 0; j < in_loop; j++)
			jadard_diag_arrange_print(s, transpose, i, j, dump);

		if (dump == NULL) {
			seq_printf(s, "\n");
		} else {
			dump->write_len += scnprintf(dump->buf + dump->write_len,
								dump->buf_len - dump->write_len, "\n");
		}
	}
}

static void jadard_diag_arrange_column(struct seq_file *s, bool transpose, int col_num, int row_num,
										struct jadard_diag_mutual_data *dump)
{
	int i, out_loop;

	if (transpose)
		out_loop = pjadard_ic_data->JD_X_NUM;
	else
		out_loop = pjadard_ic_data->JD_Y_NUM;

	if (col_num > 0) {
		for (i = col_num - 1; i >= 0; i--) {
			if (dump == NULL) {
				seq_printf(s, "%4c%02d%c", '[', col_num - i, ']');
			} else {
				dump->write_len += scnprintf(dump->buf + dump->write_len,
									dump->buf_len - dump->write_len, "%4c%02d%c", '[', col_num - i, ']');
			}

			jadard_diag_arrange_row(s, transpose, i, row_num, dump);
		}
	} else {
		for (i = 0; i < out_loop; i++) {
			if (dump == NULL) {
				seq_printf(s, "%4c%02d%c", '[', i + 1, ']');
			} else {
				dump->write_len += scnprintf(dump->buf + dump->write_len,
									dump->buf_len - dump->write_len, "%4c%02d%c", '[', i + 1, ']');
			}
			jadard_diag_arrange_row(s, transpose, i, row_num, dump);
		}
	}
}

static void jadard_diag_arrange(struct seq_file *s, int x_num, int y_num,
								struct jadard_diag_mutual_data *dump)
{
	int i;
	bool transpose = (bool)((diag_arr_num >> 2) & 0x01);
	uint8_t y_reverse = (diag_arr_num >> 1) & 0x01;
	uint8_t x_reverse = (diag_arr_num >> 0) & 0x01;

	if (((s == NULL) && (dump == NULL)) || ((s != NULL) && (dump != NULL))) {
		JD_E("%s: Not support this condition\n", __func__);
		return;
	}

	if (dump == NULL) {
		seq_printf(s, "%7c", ' ');
	} else {
		dump->write_len += scnprintf(dump->buf + dump->write_len,
							dump->buf_len - dump->write_len, "%7c", ' ');
	}

	if (transpose) {
		for (i = 1 ; i <= y_num; i++) {
			if (dump == NULL) {
				seq_printf(s, "%4c%02d%c", '[', i, ']');
			} else {
				dump->write_len += scnprintf(dump->buf + dump->write_len,
									dump->buf_len - dump->write_len, "%4c%02d%c", '[', i, ']');
			}
		}

		if (dump == NULL) {
			seq_printf(s, "\n");
		} else {
			dump->write_len += scnprintf(dump->buf + dump->write_len,
								dump->buf_len - dump->write_len, "\n");
		}
		jadard_diag_arrange_column(s, transpose, y_reverse * x_num, x_reverse * y_num, dump);
	} else {
		for (i = 1; i <= x_num; i++) {
			if (dump == NULL) {
				seq_printf(s, "%4c%02d%c", '[', i, ']');
			} else {
				dump->write_len += scnprintf(dump->buf + dump->write_len,
									dump->buf_len - dump->write_len, "%4c%02d%c", '[', i, ']');
			}
		}

		if (dump == NULL) {
			seq_printf(s, "\n");
		} else {
			dump->write_len += scnprintf(dump->buf + dump->write_len,
								dump->buf_len - dump->write_len, "\n");
		}
		jadard_diag_arrange_column(s, transpose, y_reverse * y_num, x_reverse * x_num, dump);
	}

	if (dump == NULL) {
		seq_printf(s, "\nChannelEnd\n");
	} else {
		dump->write_len += scnprintf(dump->buf + dump->write_len,
								dump->buf_len - dump->write_len, "\n");
	}
}

static void *jadard_diag_seq_start(struct seq_file *s, loff_t *pos)
{
	if (*pos >= 1)
		return NULL;

	return (void *)((unsigned long) *pos + 1);
}

static void jadard_diag_seq_stop(struct seq_file *s, void *v)
{
	/* Nothing to do */
}

static void *jadard_diag_seq_next(struct seq_file *s, void *v, loff_t *pos)
{
	(*pos)++;
	return NULL; /* if retun NULL, next step will stop */
}

static int jadard_diag_seq_show(struct seq_file *s, void *v)
{
	int x_num = pjadard_ic_data->JD_X_NUM;
	int y_num = pjadard_ic_data->JD_Y_NUM;

	seq_printf(s, "ChannelStart: %4d, %4d\n", x_num, y_num);
	jd_diag_mutual_cnt &= 0x7FFF;
	seq_printf(s, "Frame: %d\n", jd_diag_mutual_cnt);
	jadard_diag_arrange(s, x_num, y_num, NULL);

	return 0;
}

/* start->show->next->stop */
static struct seq_operations jadard_diag_seq_ops = {
	.start	= jadard_diag_seq_start,
	.stop	= jadard_diag_seq_stop,
	.next	= jadard_diag_seq_next,
	.show	= jadard_diag_seq_show,
};

static int jadard_diag_proc_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &jadard_diag_seq_ops);
};

/* Mutual data thread */
static bool jadard_ts_diag_func(void)
{
	uint32_t index = 0;
	int i, j, new_data;
	uint8_t *rdata = NULL;
	int x_num = pjadard_ic_data->JD_X_NUM;
	int y_num = pjadard_ic_data->JD_Y_NUM;
	uint16_t rdata_size = x_num * y_num * sizeof(uint16_t);
	int *PeakMax = NULL;
	int *PeakMin = NULL;

	rdata = kzalloc(rdata_size * sizeof(uint8_t), GFP_KERNEL);
	if (rdata == NULL) {
		JD_E("%s, rdata memory allocate fail: %d\n", __func__, __LINE__);
		return false;
	}

	if (KeepType == JD_KEEP_TYPE_PeakValue) {
		PeakMax = kzalloc(y_num * x_num * sizeof(int), GFP_KERNEL);
		if (PeakMax == NULL) {
			JD_E("%s, PeakMax memory allocate fail: %d\n", __func__, __LINE__);
			kfree(rdata);
			return false;
		}

		PeakMin = kzalloc(y_num * x_num * sizeof(int), GFP_KERNEL);
		if (PeakMin == NULL) {
			JD_E("%s, PeakMin memory allocate fail: %d\n", __func__, __LINE__);
			kfree(rdata);
			kfree(PeakMax);
			return false;
		}
	}

	if (g_module_fp.fp_get_mutual_data(DataType, rdata, rdata_size) < 0) {
		/* Time out, set -23131(Blue screen) */
		for (i = 0; i < x_num * y_num; i++)
			jd_diag_mutual[i] = -23131;

		kfree(rdata);
		if (KeepType == JD_KEEP_TYPE_PeakValue) {
			kfree(PeakMax);
			kfree(PeakMin);
		}
		jd_diag_mutual_cnt++;

		return false;
	}

	for (i = 0; i < y_num; i++) {
		for (j = 0; j < x_num; j++) {
			if (DataType == JD_DATA_TYPE_Difference || DataType == JD_DATA_TYPE_LAPLACE) {
				if (pjadard_ts_data->rawdata_little_endian)
					new_data = (((int8_t)rdata[index + 1] << 8) | rdata[index]);
				else
					new_data = (((int8_t)rdata[index] << 8) | rdata[index + 1]);
			} else {
				if (pjadard_ts_data->rawdata_little_endian)
					new_data = (((uint8_t)rdata[index + 1] << 8) | rdata[index]);
				else
					new_data = (((uint8_t)rdata[index] << 8) | rdata[index + 1]);
			}

			/* KeepType => 1:NormalValue 2:MaxValue 3:MinValue 4:PeakValue */
			if (KeepType == JD_KEEP_TYPE_NormalValue) {
				jd_diag_mutual[i * x_num + j] = new_data;
			} else if (KeepType == JD_KEEP_TYPE_MaxValue) {
				if (jd_diag_mutual[i * x_num + j] < new_data)
					jd_diag_mutual[i * x_num + j] = new_data;
			} else if (KeepType == JD_KEEP_TYPE_MinValue) {
				if (jd_diag_mutual[i * x_num + j] > new_data)
					jd_diag_mutual[i * x_num + j] = new_data;
			} else if (KeepType == JD_KEEP_TYPE_PeakValue) {
				if (new_data > PeakMax[i * x_num + j])
					PeakMax[i * x_num + j] = new_data;
				else if (new_data < PeakMin[i * x_num + j])
					PeakMin[i * x_num + j] = new_data;

				new_data = PeakMax[i * x_num + j] - PeakMin[i * x_num + j];

				if (jd_diag_mutual[i * x_num + j] < new_data)
					jd_diag_mutual[i * x_num + j] = new_data;
			}

			index += 2;
		}
	}
	kfree(rdata);
	/* Free memory */
	if (PeakMax)
		kfree(PeakMax);
	/* Free memory */
	if (PeakMin)
		kfree(PeakMin);

	if ((KeepFrame > 0) && (KeepFrame > jd_diag_mutual_cnt)) {
#ifdef JD_KEEP_RAWDATA_DISABLE
		struct jadard_diag_mutual_data dump;

		dump.buf_len = x_num * y_num * 10;
		dump.buf = kzalloc(dump.buf_len * sizeof(char), GFP_KERNEL);
		if (dump.buf == NULL) {
			JD_E("%s, Memory allocate fail: %d\n", __func__, __LINE__);
			return false;
		}

		dump.write_len = 0;
		jd_diag_mutual_cnt++;

		if (jd_diag_mutual_cnt == 1) {
			dump.write_len += scnprintf(dump.buf + dump.write_len,
				dump.buf_len - dump.write_len, "ChannelStart: %4d, %4d\n", x_num, y_num);
		}

		dump.write_len += scnprintf(dump.buf + dump.write_len,
							dump.buf_len - dump.write_len, "Frame: %d\n", jd_diag_mutual_cnt);

		jadard_diag_arrange(NULL, x_num, y_num, &dump);

		if (jd_diag_mutual_cnt == KeepFrame) {
			dump.write_len += scnprintf(dump.buf + dump.write_len,
								dump.buf_len - dump.write_len, "ChannelEnd\n");
		}

		/* save mutual data in file */
		if (!IS_ERR(jd_diag_mutual_fn)) {
#if defined(__JADARD_GKI__)
			JD_I("Cancel write keep rawdata file for GKI\n");
#else
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
			kernel_write(jd_diag_mutual_fn, dump.buf, dump.write_len, &jd_diag_mutual_fn->f_pos);
#else
			kernel_write(jd_diag_mutual_fn, dump.buf, dump.write_len, jd_diag_mutual_fn->f_pos);
			jd_diag_mutual_fn->f_pos += dump.write_len;
#endif
#endif
		}
		kfree(dump.buf);
		dump.buf = NULL;
#endif
	} else {
		jd_diag_mutual_cnt++;
	}

	return true;
}

/* Mutual data thread */
static bool jadard_ts_buf_func(void)
{
	uint16_t index = 0;
	int x_num = pjadard_ic_data->JD_X_NUM;
	int y_num = pjadard_ic_data->JD_Y_NUM;
	uint16_t rdata_size = x_num * y_num * buf_rd_byte_num;

	if (g_module_fp.fp_get_mutual_data(DataType, jd_buf, rdata_size) < 0) {
		/* Time out, set -23131(Blue screen) */
		for (index = 0; index < rdata_size; index++)
			jd_buf[index] = 0xA5;

		jd_diag_mutual_cnt++;

		return false;
	}
	jd_diag_mutual_cnt++;

	return true;
}

static void jadard_ts_diag_work_func(struct work_struct *work)
{
	while (pjadard_ts_data->diag_thread_active) {
		if (jd_g_buf_rd_enable) {
			if (!jadard_ts_buf_func())
				JD_E("%s : Read mutual data fail\n", __func__);
		} else {
			if (!jadard_ts_diag_func())
				JD_E("%s : Get mutual data fail\n", __func__);
		}
	}
}

/*
 * DataType	 => 1:RawData 2:Baseline 3:Difference 4:Listen 5:Label 6:Laplace Reserve:7~F(15)
 * KeepType	 => 1:NormalValue 2:MaxValue 3:MinValue 4:PeakValue
 * KeepFrame => Number(Disable:0, Enable:1 ~ 2000)
 *
 * Example: echo 0_0_0 > /proc/jadard_touch/diag
 *			Stop diag thread and set MCU into report coordinate mode
 * Example: echo 1_2_0 > /proc/jadard_touch/diag
 *			Set DataType = 1, KeepType = 2, KeepFrame = 0(Disable)
 * Example: echo 1_2_100 > /proc/jadard_touch/diag
 *			Set DataType = 1, KeepType = 2, KeepFrame = 100
 *			KeepFrame data will save to "/sdcard/JD_XXX_Dump.txt"
 */
static ssize_t jadard_diag_write(struct file *filp, const char __user *buf,
									size_t len, loff_t *data)
{
	struct jadard_ts_data *ts = pjadard_ts_data;
	char buf_tmp[20] = {0};

	if (len >= sizeof(buf_tmp)) {
		JD_I("%s: no command exceeds %ld chars.\n", __func__, sizeof(buf_tmp));
		return -EFAULT;
	}

	if (copy_from_user(buf_tmp, buf, (len - 1 < sizeof(buf_tmp) - 1) ? len - 1 : sizeof(buf_tmp) - 1))
		return -EFAULT;

	/* Cancel mutual data thread, if exist */
	if (pjadard_ts_data->diag_thread_active) {
		pjadard_ts_data->diag_thread_active = false;
		cancel_delayed_work_sync(&pjadard_ts_data->jadard_diag_delay_wrok);
	}
	ts->debug_diag_apk_enable = false;
	jd_g_buf_rd_enable = false;

	DataType = JD_DATA_TYPE_RawData;
	KeepType = JD_KEEP_TYPE_NormalValue;
	KeepFrame = 0;

	if ((len >= 6) && (len <= 9)) {
		if ((buf_tmp[1] == '_') && (buf_tmp[3] == '_')) {
			if ((buf_tmp[0] >= 48) && (buf_tmp[0] <= 57)) { /* 0~9 */
				DataType = buf_tmp[0] - '0';
			} else if ((buf_tmp[0] >= 65) && (buf_tmp[0] <= 70)) { /* A~F */
				DataType = buf_tmp[0] - 'A' + 10;
			} else if ((buf_tmp[0] >= 97) && (buf_tmp[0] <= 102)) { /* a~f */
				DataType = buf_tmp[0] - 'a' + 10;
			}
			KeepType = buf_tmp[2] - '0';

			if (len == 6) {
				KeepFrame = buf_tmp[4] - '0';
			} else if (len == 7) {
				KeepFrame = (buf_tmp[4] - '0') * 10 + (buf_tmp[5] - '0');
			} else if (len == 8) {
				KeepFrame = (buf_tmp[4] - '0') * 100 + (buf_tmp[5] - '0') * 10 +
							(buf_tmp[6] - '0');
			} else {
				KeepFrame = (buf_tmp[4] - '0') * 1000 + (buf_tmp[5] - '0') * 100 +
							(buf_tmp[6] - '0') * 10 + (buf_tmp[7] - '0');
			}

			if (KeepFrame > 2000)
				KeepFrame = 2000;
		} else {
			JD_E("%s: Not support command:<%s>\n", __func__, buf_tmp);
			return -EINVAL;
		}
	} else {
		JD_E("%s: Not support command:<%s>\n", __func__, buf_tmp);
		return -EINVAL;
	}

	JD_I("%s: DataType:%d, KeepType:%d, KeepFrame:%d\n", __func__, DataType, KeepType, KeepFrame);

#ifndef JD_KEEP_RAWDATA_DISABLE
	if (KeepFrame > 0) {
		JD_I("%s: Keep rawdata was disable, please using JDMT APK\n", __func__);
		return len;
	}
#endif
	/* Check DataType and KeepType were available */
	if ((DataType < 1) || (DataType > 15) || (KeepType < 1) || (KeepType > 4)) {
		if ((DataType != 0) || (KeepType != 0)) {
			JD_E("%s: DataType must (0~F) and KeepType must (0~4)\n", __func__);
			return -EINVAL;
		}
	}

	if (jd_diag_mutual) {
		memset(jd_diag_mutual, 0x00, pjadard_ic_data->JD_X_NUM * pjadard_ic_data->JD_Y_NUM * sizeof(int));
	} else {
		JD_E("%s, jd_diag_mutual memory allocate fail: %d\n", __func__, __LINE__);
		return len;
	}

	if ((DataType > 0) && (KeepType > 0)) {
		/* 1. Set mutual data type */
		g_module_fp.fp_mutual_data_set((uint8_t)DataType);

#ifdef JD_KEEP_RAWDATA_DISABLE
		/* 1.1 Open file for save mutual data log, if KeepFrame > 0 */
		if (KeepFrame > 0) {
			switch (DataType) {
			case JD_DATA_TYPE_RawData:
				jd_diag_mutual_fn = filp_open(JD_RAWDATA_DUMP_FILE, O_CREAT | O_TRUNC | O_WRONLY, 0);
				jd_diag_mutual_fn->f_pos = 0;
				break;
			case JD_DATA_TYPE_Baseline:
				jd_diag_mutual_fn = filp_open(JD_BASE_DUMP_FILE, O_CREAT | O_TRUNC | O_WRONLY, 0);
				jd_diag_mutual_fn->f_pos = 0;
				break;
			case JD_DATA_TYPE_Difference:
				jd_diag_mutual_fn = filp_open(JD_DIFF_DUMP_FILE, O_CREAT | O_TRUNC | O_WRONLY, 0);
				jd_diag_mutual_fn->f_pos = 0;
				break;
			case JD_DATA_TYPE_LISTEN:
				jd_diag_mutual_fn = filp_open(JD_LISTEN_DUMP_FILE, O_CREAT | O_TRUNC | O_WRONLY, 0);
				jd_diag_mutual_fn->f_pos = 0;
				break;
			case JD_DATA_TYPE_LABEL:
				jd_diag_mutual_fn = filp_open(JD_LABEL_DUMP_FILE, O_CREAT | O_TRUNC | O_WRONLY, 0);
				jd_diag_mutual_fn->f_pos = 0;
			case JD_DATA_TYPE_LAPLACE:
				jd_diag_mutual_fn = filp_open(JD_LAPLACE_DUMP_FILE, O_CREAT | O_TRUNC | O_WRONLY, 0);
				jd_diag_mutual_fn->f_pos = 0;
				break;
			default:
				JD_I("%s: Data type is not support. DataType is %d\n", __func__, DataType);
			}
		}
#endif
		/* 2. Start mutual data thread */
		jadard_int_enable(false);
		msleep(1000);
		pjadard_ts_data->diag_thread_active = true;

		g_module_fp.fp_EnterBackDoor();
		JD_I("Enter backdoor\n");

		jd_diag_mutual_cnt = 0;
		queue_delayed_work(pjadard_ts_data->jadard_diag_wq,
			&pjadard_ts_data->jadard_diag_delay_wrok, msecs_to_jiffies(1000));
		JD_I("%s: Start get mutual data\n", __func__);
		jadard_int_enable(true);

#ifdef JD_KEEP_RAWDATA_DISABLE
		if (KeepFrame > 0) {
			/* 2.1 Wait data ready */
			while (KeepFrame > jd_diag_mutual_cnt)
				msleep(100);

			switch (DataType) {
			case JD_DATA_TYPE_RawData:
				JD_I("%s: Save file to %s\n", __func__, JD_RAWDATA_DUMP_FILE);
				break;
			case JD_DATA_TYPE_Baseline:
				JD_I("%s: Save file to %s\n", __func__, JD_BASE_DUMP_FILE);
				break;
			case JD_DATA_TYPE_Difference:
				JD_I("%s: Save file to %s\n", __func__, JD_DIFF_DUMP_FILE);
				break;
			case JD_DATA_TYPE_LISTEN:
				JD_I("%s: Save file to %s\n", __func__, JD_LISTEN_DUMP_FILE);
				break;
			case JD_DATA_TYPE_LABEL:
				JD_I("%s: Save file to %s\n", __func__, JD_LABEL_DUMP_FILE);
				break;
			case JD_DATA_TYPE_LAPLACE:
				JD_I("%s: Save file to %s\n", __func__, JD_LAPLACE_DUMP_FILE);
				break;
			default:
				JD_I("%s: Data type is not support. DataType is %d\n", __func__, DataType);
			}

			if (jd_diag_mutual_fn != NULL) {
				filp_close(jd_diag_mutual_fn, NULL);
				jd_diag_mutual_fn = NULL;
			}

			/* 3. Stop thread */
			if (pjadard_ts_data->diag_thread_active) {
				pjadard_ts_data->diag_thread_active = false;
				cancel_delayed_work_sync(&pjadard_ts_data->jadard_diag_delay_wrok);
			}
		}
#endif
	} else if ((DataType == 0) && (KeepType == 0) && (KeepFrame == 0)) {
		g_module_fp.fp_mutual_data_set(JD_DATA_TYPE_RawData);
	}

	return len;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0)
static struct proc_ops jadard_proc_diag_ops = {
	.proc_open = jadard_diag_proc_open,
	.proc_read = seq_read,
	.proc_release = seq_release,
	.proc_write = jadard_diag_write,
};
#else
static struct file_operations jadard_proc_diag_ops = {
	.owner = THIS_MODULE,
	.open = jadard_diag_proc_open,
	.read = seq_read,
	.release = seq_release,
	.write = jadard_diag_write,
};
#endif

static ssize_t jadard_diag_arrange_read(struct file *file, char *buf,
									size_t len, loff_t *pos)
{
	size_t ret = 0;
	char *buf_tmp = NULL;

	if (len < 32) {
		JD_E("%s: len size less than 32\n", __func__);
		return -ENOMEM;
	}

	if (!pjadard_debug->proc_send_flag) {
		buf_tmp = kzalloc(len * sizeof(char), GFP_KERNEL);
		if (buf_tmp != NULL) {
			ret += scnprintf(buf_tmp + ret, len - ret, "diag_arr_num = %d\n", diag_arr_num);

			if (copy_to_user(buf, buf_tmp, len))
				JD_E("%s, copy_to_user fail: %d\n", __func__, __LINE__);

			kfree(buf_tmp);
		} else {
			JD_E("%s, Memory allocate fail: %d\n", __func__, __LINE__);
		}
		pjadard_debug->proc_send_flag = true;
	} else {
		pjadard_debug->proc_send_flag = false;
	}

	return ret;
}

/*
 * Bit0: X reverse
 * Bit1: Y reverse
 * Bit2: Transpose
 * Example: echo 5 > /proc/jadard_touch/diag_arr
 *			Transpose and X reverse
 */
static ssize_t jadard_diag_arrange_write(struct file *file, const char *buf,
										size_t len, loff_t *pos)
{
	char buf_tmp[10] = {0};

	if (len >= sizeof(buf_tmp)) {
		JD_I("%s: no command exceeds %ld chars.\n", __func__, sizeof(buf_tmp));
		return -EFAULT;
	}

	if (copy_from_user(buf_tmp, buf, (len < sizeof(buf_tmp) - 1) ? len : sizeof(buf_tmp) - 1))
		return -EFAULT;

	diag_arr_num = buf_tmp[0] - '0';

	if ((diag_arr_num > 7) || (diag_arr_num < 0)) {
		diag_arr_num = 0;
		JD_E("%s: Not support command, set to 0\n", __func__);
	}

	JD_I("%s: diag_arr_num = %d\n", __func__, diag_arr_num);

	return len;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0)
static struct proc_ops jadard_proc_diag_arrange_ops = {
	.proc_read = jadard_diag_arrange_read,
	.proc_write = jadard_diag_arrange_write,
};
#else
static struct file_operations jadard_proc_diag_arrange_ops = {
	.owner = THIS_MODULE,
	.read = jadard_diag_arrange_read,
	.write = jadard_diag_arrange_write,
};
#endif

#ifdef JD_RAWDATA_V2
static void *jadard_diag_apk_seq_start(struct seq_file *s, loff_t *pos)
{
	if (*pos >= 1)
		return NULL;

	return (void *)((unsigned long) *pos + 1);
}

static void jadard_diag_apk_seq_stop(struct seq_file *s, void *v)
{
	/* Nothing to do */
}

static void *jadard_diag_apk_seq_next(struct seq_file *s, void *v, loff_t *pos)
{
	(*pos)++;
	return NULL; /* if retun NULL, next step will stop */
}

static int jadard_diag_apk_seq_show(struct seq_file *s, void *v)
{
	int x_num = pjadard_ic_data->JD_X_NUM;
	int y_num = pjadard_ic_data->JD_Y_NUM;
	int i, j, max = 0, min = 65535;

	if (ApkOrManual == 'a') {
		seq_printf(s, "%d %d ", x_num, y_num);
		jd_diag_mutual_cnt &= 0x7FFF;
		seq_printf(s, "%d ", jd_diag_mutual_cnt);

		for (i = 0; i < y_num; i++) {
			for (j = 0; j < x_num; j++)
				seq_printf(s, "%d ", jd_diag_mutual[j * y_num + i]);
		}

		for (i = 0; i < JD_HID_TOUCH_DATA_SIZE; i++)
			seq_printf(s, "0x%02X ", pjadard_report_data->touch_all_info[i]);
	} else {
		seq_printf(s, "Total_1,Tx_%d,Rx_%d,Width_%d,Height_%d\n", x_num, y_num,
					pjadard_ic_data->JD_X_RES, pjadard_ic_data->JD_Y_RES);

		for (i = 0; i < y_num; i++) {
			for (j = 0; j < x_num; j++) {
				if (jd_diag_mutual[i * x_num + j] > max)
					max = jd_diag_mutual[i * x_num + j];

				if (jd_diag_mutual[i * x_num + j] < min)
					min = jd_diag_mutual[i * x_num + j];
			}
		}

		jd_diag_mutual_cnt &= 0x7FFF;
		seq_printf(s, "No_%d,Max_%d,Min_%d,Peak_%d\nFrame_%d", jd_diag_mutual_cnt, max, min, max - min, jd_diag_mutual_cnt);

		for (i = 0; i < y_num; i++)
			seq_printf(s, ",Cy_%d", i + 1);
		seq_printf(s, ",\n");

		for (i = 0; i < x_num; i++) {
			seq_printf(s, "Rx_%d", i + 1);
			for (j = 0; j < y_num; j++)
				seq_printf(s, ",%d", jd_diag_mutual[i * x_num + j]);
			seq_printf(s, "\n");
		}

		seq_printf(s, "Frm_1,");
		for (i = 0; i < 16; i++)
			seq_printf(s, "[%d],", i);
		seq_printf(s, "\n");

		for (i = 0; i < JD_HID_TOUCH_DATA_SIZE; i++) {
			seq_printf(s, ",0x%02X", pjadard_report_data->touch_all_info[i]);
			if ((i == 15) || ((i % 16) == 15))
				seq_printf(s, "\n");
		}
	}

	seq_printf(s, "\n");

	return 0;
}

/* start->show->next->stop */
static struct seq_operations jadard_diag_apk_seq_ops = {
	.start	= jadard_diag_apk_seq_start,
	.stop	= jadard_diag_apk_seq_stop,
	.next	= jadard_diag_apk_seq_next,
	.show	= jadard_diag_apk_seq_show,
};

static int jadard_diag_apk_proc_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &jadard_diag_apk_seq_ops);
};

/*
 * ApkOrManual => a:apk m:manual other:manual
 * DataType	 => 1:RawData 2:Baseline 3:Difference 4:Listen 5:Label 6:Laplace Reserve:7~F(15)
 */
static ssize_t jadard_diag_apk_write(struct file *filp, const char __user *buf,
									size_t len, loff_t *data)
{
	struct jadard_ts_data *ts = pjadard_ts_data;
	char buf_tmp[10] = {0};

	if (len >= sizeof(buf_tmp)) {
		JD_I("%s: no command exceeds %ld chars.\n", __func__, sizeof(buf_tmp));
		return -EFAULT;
	}

	if (copy_from_user(buf_tmp, buf, (len - 1 < sizeof(buf_tmp) - 1) ? len - 1 : sizeof(buf_tmp) - 1))
		return -EFAULT;

	/* Cancel mutual data thread, if exist */
	if (ts->diag_thread_active) {
		ts->diag_thread_active = false;
		cancel_delayed_work_sync(&ts->jadard_diag_delay_wrok);
	}
	ts->debug_fw_package_enable = false;
	ts->debug_diag_apk_enable = false;

	DataType = JD_DATA_TYPE_RawData;
	KeepType = JD_KEEP_TYPE_NormalValue;
	KeepFrame = 0;

	if ((len >= 6) && (len <= 9)) {
		if ((buf_tmp[0] >= 48) && (buf_tmp[0] <= 57)) { /* 0~9 */
			DataType = buf_tmp[0] - '0';
		} else if ((buf_tmp[0] >= 65) && (buf_tmp[0] <= 70)) { /* A~F */
			DataType = buf_tmp[0] - 'A' + 10;
		} else if ((buf_tmp[0] >= 97) && (buf_tmp[0] <= 102)) { /* a~f */
			DataType = buf_tmp[0] - 'a' + 10;
		}

		if (buf_tmp[2] == 'a')
			ApkOrManual = 'a';
		else
			ApkOrManual = 'm';

		if (len == 6) {
			KeepFrame = buf_tmp[4] - '0';
		} else if (len == 7) {
			KeepFrame = (buf_tmp[4] - '0') * 10 + (buf_tmp[5] - '0');
		} else if (len == 8) {
			KeepFrame = (buf_tmp[4] - '0') * 100 + (buf_tmp[5] - '0') * 10 +
						(buf_tmp[6] - '0');
		} else {
			KeepFrame = (buf_tmp[4] - '0') * 1000 + (buf_tmp[5] - '0') * 100 +
						(buf_tmp[6] - '0') * 10 + (buf_tmp[7] - '0');
		}

		if (KeepFrame > 2000)
			KeepFrame = 2000;
	} else {
		JD_E("%s: Not support command:<%s>, len:%zu\n", __func__, buf_tmp, len);
		return -EINVAL;
	}

	JD_I("%s: DataType:%d, ApkOrManual:%c, KeepFrame:%d\n", __func__, DataType, ApkOrManual, KeepFrame);
#ifndef JD_KEEP_RAWDATA_DISABLE
	if (KeepFrame > 0) {
		JD_I("%s: apk keep rawdata was disable, please using JDMT APK\n", __func__);
		return len;
	}
#endif

	/* Check DataType was available */
	if ((DataType < 1) || (DataType > 15)) {
		if (DataType != 0) {
			JD_E("%s: DataType must (0~F)\n", __func__);
			return -EINVAL;
		}
	}

	if (jd_diag_mutual) {
		memset(jd_diag_mutual, 0x00, pjadard_ic_data->JD_X_NUM * pjadard_ic_data->JD_Y_NUM * sizeof(int));
	} else {
		JD_E("%s, jd_diag_mutual memory allocate fail: %d\n", __func__, __LINE__);
		return len;
	}

	if (DataType > 0) {
		ts->debug_diag_apk_enable = true;
		/* 1. Set mutual data type */
		g_module_fp.fp_mutual_data_set((uint8_t)DataType);

		/* 2. Start mutual data thread */
		jd_diag_mutual_cnt = 0;
		JD_I("%s: Start get mutual data\n", __func__);

		pjadard_report_data->report_rate_count = 0;
		pjadard_report_data->touch_all_info[1] = 0;
		pjadard_report_data->touch_all_info[2] = 0;
		ts->debug_fw_package_enable = true;

#ifdef JD_KEEP_RAWDATA_DISABLE
		if ((ApkOrManual == 'm') && (KeepFrame > 0)) {
			jd_diag_mutual_fn = filp_open(JD_RAWDATA_DUMP_FILE, O_CREAT | O_TRUNC | O_WRONLY, 0);
			jd_diag_mutual_fn->f_pos = 0;

			/* Wait data ready */
			while (KeepFrame > jd_diag_mutual_cnt)
				msleep(1000);

			JD_I("%s: Save file to %s\n", __func__, JD_RAWDATA_DUMP_FILE);
			if (jd_diag_mutual_fn != NULL) {
				filp_close(jd_diag_mutual_fn, NULL);
				jd_diag_mutual_fn = NULL;
			}

			ts->debug_fw_package_enable = false;
			ts->debug_diag_apk_enable = false;
		} else {
			KeepFrame = 0;
		}
#endif
	} else { /* DataType == 0 */
		g_module_fp.fp_mutual_data_set(JD_DATA_TYPE_RawData);
	}

	return len;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0)
static struct proc_ops jadard_proc_diag_apk_ops = {
	.proc_open = jadard_diag_apk_proc_open,
	.proc_read = seq_read,
	.proc_release = seq_release,
	.proc_write = jadard_diag_apk_write,
};
#else
static struct file_operations jadard_proc_diag_apk_ops = {
	.owner = THIS_MODULE,
	.open = jadard_diag_apk_proc_open,
	.read = seq_read,
	.release = seq_release,
	.write = jadard_diag_apk_write,
};
#endif
#endif

static void *jadard_buf_rd_seq_start(struct seq_file *s, loff_t *pos)
{
	if (*pos >= 1)
		return NULL;

	return (void *)((unsigned long) *pos + 1);
}

static void jadard_buf_rd_seq_stop(struct seq_file *s, void *v)
{
	/* Nothing to do */
}

static void *jadard_buf_rd_seq_next(struct seq_file *s, void *v, loff_t *pos)
{
	(*pos)++;
	return NULL; /* if retun NULL, next step will stop */
}

static int jadard_buf_rd_seq_show(struct seq_file *s, void *v)
{
	int index, i;
	int x_num = pjadard_ic_data->JD_X_NUM;
	int y_num = pjadard_ic_data->JD_Y_NUM;

	jd_diag_mutual_cnt &= 0x7FFF;
	seq_printf(s, "%d ", jd_diag_mutual_cnt);

	for (index = 0; index < x_num * y_num * buf_rd_byte_num; index += buf_rd_byte_num) {
		for (i = buf_rd_byte_num - 1; i >= 0; i--)
			seq_printf(s, "%02X", jd_buf[index + i]);
		seq_printf(s, " ");
	}

	seq_printf(s, "\n");

	return 0;
}

/* start->show->next->stop */
static struct seq_operations jadard_buf_rd_seq_ops = {
	.start	= jadard_buf_rd_seq_start,
	.stop	= jadard_buf_rd_seq_stop,
	.next	= jadard_buf_rd_seq_next,
	.show	= jadard_buf_rd_seq_show,
};

static int jadard_buf_rd_proc_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &jadard_buf_rd_seq_ops);
};

/*
 * DataType	 => 1:RawData 2:Baseline 3:Difference 4:Listen 5:Label 6:Laplace Reserve:7~F(15)
 */
static ssize_t jadard_buf_rd_write(struct file *filp, const char __user *buf,
									size_t len, loff_t *data)
{
	char buf_tmp[10] = {0};
	uint32_t type = 0;
	int i;

	if (len >= sizeof(buf_tmp)) {
		JD_I("%s: no command exceeds %ld chars.\n", __func__, sizeof(buf_tmp));
		return -EFAULT;
	}

	if (copy_from_user(buf_tmp, buf, (len - 1 < sizeof(buf_tmp) - 1) ? len - 1 : sizeof(buf_tmp) - 1))
		return -EFAULT;

	if ((buf_tmp[0] == 't') && (buf_tmp[1] == ' ')) {
		/* Cancel mutual data thread, if exist */
		if (pjadard_ts_data->diag_thread_active) {
			pjadard_ts_data->diag_thread_active = false;
			cancel_delayed_work_sync(&pjadard_ts_data->jadard_diag_delay_wrok);
		}
		jd_g_buf_rd_enable = false;

		for (i = 2; i < len - 1; i++) {
			uint8_t byte = buf_tmp[i];

			if (byte >= '0' && byte <= '9') { /* 0~9 */
				byte = byte - '0';
			} else if (byte >= 'A' && byte <= 'F') { /* A~F */
				byte = byte - 'A' + 10;
			} else if (byte >= 'a' && byte <= 'f') { /* a~f */
				byte = byte - 'a' + 10;
			}
			type = (type << 4) | (byte & 0xF);
		}
		DataType = (int)type;

		JD_I("%s: DataType:%d\n", __func__, DataType);

		if (DataType > 0) {
			/* 1. Set mutual data type */
			g_module_fp.fp_mutual_data_set((uint8_t)DataType);
			/* 2. Start mutual data thread */
			jadard_int_enable(false);
			msleep(1000);
			pjadard_ts_data->diag_thread_active = true;
			jd_g_buf_rd_enable = true;

			g_module_fp.fp_EnterBackDoor();
			JD_I("Enter backdoor\n");

			jd_diag_mutual_cnt = 0;
			queue_delayed_work(pjadard_ts_data->jadard_diag_wq,
				&pjadard_ts_data->jadard_diag_delay_wrok, msecs_to_jiffies(1000));
			JD_I("%s: Start get mutual data\n", __func__);
			jadard_int_enable(true);
		} else if (DataType == 0) {
			g_module_fp.fp_mutual_data_set(JD_DATA_TYPE_RawData);
		}
	} else if ((buf_tmp[0] == 'b') && (buf_tmp[1] == ' ')) {
		buf_rd_byte_num = buf_tmp[2] - '0';

		if ((buf_rd_byte_num != 4) && (buf_rd_byte_num != 2)) {
			JD_E("%s: Not support num = %d, set to 2\n", __func__, buf_rd_byte_num);
			buf_rd_byte_num = 2;
		}

		JD_I("%s: buf_rd_byte_num = %d\n", __func__, buf_rd_byte_num);

		/* Reallocate jd_buf memory */
		if (jd_buf) {
			kfree(jd_buf);
			jd_buf = NULL;
		}

		jd_buf = kzalloc(pjadard_ic_data->JD_X_NUM * pjadard_ic_data->JD_Y_NUM *
				buf_rd_byte_num * sizeof(uint8_t), GFP_KERNEL);
		if (jd_buf == NULL) {
			JD_E("%s: jd_buf buffer reallocate failed\n", __func__);
			return -ENOMEM;
		}
	}

	return len;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0)
static struct proc_ops jadard_proc_buf_rd_ops = {
	.proc_open = jadard_buf_rd_proc_open,
	.proc_read = seq_read,
	.proc_release = seq_release,
	.proc_write = jadard_buf_rd_write,
};
#else
static struct file_operations jadard_proc_buf_rd_ops = {
	.owner = THIS_MODULE,
	.open = jadard_buf_rd_proc_open,
	.read = seq_read,
	.release = seq_release,
	.write = jadard_buf_rd_write,
};
#endif

static ssize_t jadard_proc_fw_dump_read(struct file *file, char *buf,
										size_t len, loff_t *pos)
{
	ssize_t ret = 0;
	char *buf_tmp = NULL;

	if (len < 64) {
		JD_E("%s: len size less than 64\n", __func__);
		return -ENOMEM;
	}

	if (!pjadard_debug->proc_send_flag) {
		buf_tmp = kzalloc(len * sizeof(char), GFP_KERNEL);
		if (buf_tmp != NULL) {
			if (fw_dump_going && (!fw_dump_complete)) {
				ret += scnprintf(buf_tmp + ret, len - ret, "Please wait a moment\n");
				ret += scnprintf(buf_tmp + ret, len - ret, "FW dump is running\n");
			} else if ((!fw_dump_going) && fw_dump_complete) {
				ret += scnprintf(buf_tmp + ret, len - ret, "FW dump finish\n");
			}

			if (copy_to_user(buf, buf_tmp, len))
				JD_E("%s, copy_to_user fail: %d\n", __func__, __LINE__);

			kfree(buf_tmp);
		} else {
			JD_E("%s, Memory allocate fail: %d\n", __func__, __LINE__);
		}
		pjadard_debug->proc_send_flag = true;
	} else {
		pjadard_debug->proc_send_flag = false;
	}

	return ret;
}

static void jadard_ts_fw_dump_work_func(struct work_struct *work)
{
#if defined(__JADARD_GKI__)
	JD_I("Cancel dump fw bin for GKI\n");
#else
	struct file *fn;
	uint32_t read_len;
	uint8_t i, chip_num = 1;

	JD_I("%s: enter\n", __func__);

	jadard_int_enable(false);
	fw_dump_going = true;

#ifdef JD_ZERO_FLASH
	if (pjadard_ic_data->JD_MODULE_CASCADE_MODE == JD_CASCADE_MODE_ENABLE)
		chip_num = 2;
#endif

	for (i = 0; i < chip_num; i++) {
#ifdef JD_ZERO_FLASH
		read_len = g_common_variable.RAM_LEN;
		if (i == 1)
			g_module_fp.fp_ram_read(pjadard_ic_data->JD_SECOND_ADDR_OFFSET, fw_buffer, read_len);
		else
			g_module_fp.fp_ram_read(pjadard_ic_data->JD_MASTER_ADDR_OFFSET, fw_buffer, read_len);
#else
		read_len = g_common_variable.FW_SIZE;
		g_module_fp.fp_flash_read(0, fw_buffer, read_len);
#endif
		JD_I("%s: Read FW from chip complete\n", __func__);

		if (i == 1)
			fn = filp_open(JD_SECOND_FW_DUMP_FILE, O_CREAT | O_TRUNC | O_WRONLY, 0);
		else
			fn = filp_open(JD_MASTER_FW_DUMP_FILE, O_CREAT | O_TRUNC | O_WRONLY, 0);

		if (!IS_ERR(fn)) {
			JD_I("%s: create file and ready to write\n", __func__);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
			kernel_write(fn, fw_buffer, read_len * sizeof(uint8_t), &fn->f_pos);
#else
			kernel_write(fn, fw_buffer, read_len * sizeof(uint8_t), fn->f_pos);
#endif
			filp_close(fn, NULL);

			if (i == 1)
				JD_I("%s: Save data to %s\n", __func__, JD_SECOND_FW_DUMP_FILE);
			else
				JD_I("%s: Save data to %s\n", __func__, JD_MASTER_FW_DUMP_FILE);
		}
	}

	jadard_int_enable(true);
	fw_dump_going = false;
#endif
	fw_dump_complete = true;
	fw_dump_busy = false;
}

/*
 * Dump FW(by flash/ram) then Save data to "JD_FW_DUMP_FILE"
 */
static ssize_t jadard_proc_fw_dump_write(struct file *file, const char *buf,
									size_t len, loff_t *pos)
{
	char buf_tmp[10] = {0};

	if (len >= sizeof(buf_tmp)) {
		JD_I("%s: no command exceeds %ld chars.\n", __func__, sizeof(buf_tmp));
		return -EFAULT;
	}

	if (copy_from_user(buf_tmp, buf, (len < sizeof(buf_tmp) - 1) ? len : sizeof(buf_tmp) - 1))
		return -EFAULT;

	JD_I("%s: buf_tmp = %s\n", __func__, buf_tmp);

	if (fw_dump_busy) {
		JD_E("%s: FW dump is busy, reject request!\n", __func__);
		return len;
	}

	if (buf_tmp[0] == '1') {
		fw_dump_busy = true;
		fw_dump_complete = false;

		queue_work(pjadard_ts_data->fw_dump_wq, &pjadard_ts_data->fw_dump_work);
	} else {
		JD_E("%s: Not support command!\n", __func__);
		return -EINVAL;
	}

	return len;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0)
static struct proc_ops jadard_proc_fw_dump_ops = {
	.proc_read = jadard_proc_fw_dump_read,
	.proc_write = jadard_proc_fw_dump_write,
};
#else
static struct file_operations jadard_proc_fw_dump_ops = {
	.owner = THIS_MODULE,
	.read = jadard_proc_fw_dump_read,
	.write = jadard_proc_fw_dump_write,
};
#endif

static void *jadard_proc_register_seq_start(struct seq_file *s, loff_t *pos)
{
	if (*pos >= 1)
		return NULL;

	return (void *)((unsigned long) *pos + 1);
}

static void jadard_proc_register_seq_stop(struct seq_file *s, void *v)
{
	/* Nothing to do */
}

static void *jadard_proc_register_seq_next(struct seq_file *s, void *v, loff_t *pos)
{
	(*pos)++;
	return NULL; /* if retun NULL, next step will stop */
}

static int jadard_proc_register_seq_show(struct seq_file *s, void *v)
{
	uint8_t *rdata = NULL;
	uint32_t ReadAddr = 0;
	int i = 0;

	/* Read data from chip */
	if (reg_read_len == 0) {
		/* if reg_read_len = 0, default 1 */
		reg_read_len = 1;
	}

	rdata = kzalloc(reg_read_len * sizeof(char), GFP_KERNEL);
	if (rdata == NULL) {
		JD_E("%s, Memory allocate fail: %d\n", __func__, __LINE__);
		return 0;
	}

	ReadAddr = (reg_cmd[3] << 24) + (reg_cmd[2] << 16) + (reg_cmd[1] << 8) + reg_cmd[0];
	if (g_module_fp.fp_register_read(ReadAddr, rdata, reg_read_len) == JD_READ_LEN_OVERFLOW) {
		seq_printf(s, "Read length was overflow\n");
	} else {
		/* Set output data */
		seq_printf(s, "Register: %02X,%02X,%02X,%02X\n",
						reg_cmd[3], reg_cmd[2], reg_cmd[1], reg_cmd[0]);

		for (i = 0; i < reg_read_len; i++) {
			seq_printf(s, "0x%2.2X ", rdata[i]);

			if ((i % 16) == 15)
				seq_printf(s, "\n");
		}
		seq_printf(s, "\n");
	}
	kfree(rdata);

	return 0;
}

/* start->show->next->stop */
static struct seq_operations jadard_proc_register_seq_ops = {
	.start	= jadard_proc_register_seq_start,
	.stop	= jadard_proc_register_seq_stop,
	.next	= jadard_proc_register_seq_next,
	.show	= jadard_proc_register_seq_show,
};

static int jadard_proc_register_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &jadard_proc_register_seq_ops);
};

/*
 * Example: Only support lower case
 * ADDRESS: 1~4 byte size, according to the chip of HW design
 * Write register: echo w:xADDRESS:x11:x22:x33 > /proc/jadard_touch/register
 *                 write ADDRESS 3bytes data(0x11,0x22,0x33), MAX:128bytes
 * Read register: echo r:xADDRESS:L_length:H_length > /proc/jadard_touch/register
 *            EX: echo r:xADDRESS:x1F > /proc/jadard_touch/register
 *                read ADDRESS 0x1F(31)bytes, MAX:0xFF
 *            EX: echo r:xADDRESS:x1D:x10 > /proc/jadard_touch/register
 *                read ADDRESS 0x101D(4125)bytes, MAX:0xFFFF
 */
static ssize_t jadard_proc_register_write(struct file *file, const char *buf,
											size_t len, loff_t *pos)
{
	char buf_tmp[130 * 4 + 12] = {0};
	char tok_data[130 * 4 + 12] = {0};
	char *p_tok_data = tok_data;
	char *token = NULL;
	char *const delim = ":";
	uint8_t w_data[130];
	int w_data_len = 0;
	unsigned long result = 0;
	bool reg_addr = true;
	bool format_err = false;
	char temp[4];
	int i;
	int byte_count = 0;

	if (len >= sizeof(buf_tmp)) {
		JD_I("%s: no command exceeds %ld chars.\n", __func__, sizeof(buf_tmp));
		return -EFAULT;
	}

	if (copy_from_user(buf_tmp, buf, (len - 1 < sizeof(buf_tmp) - 1) ? len - 1 : sizeof(buf_tmp) - 1))
		return -EFAULT;

	JD_I("%s: buf_tmp = %s\n", __func__, buf_tmp);

	if ((buf_tmp[0] == 'r' || buf_tmp[0] == 'w') && buf_tmp[1] == ':' && buf_tmp[2] == 'x') {
		memset(w_data, 0x00, sizeof(w_data));
		memcpy(p_tok_data, buf_tmp + 3, strlen(buf_tmp) - 3);

		while ((token = strsep(&p_tok_data, delim)) != NULL) {
			/* I("token=%s, strlen(token)=%d\n", token, (int)strlen(token)); */
			if (reg_addr) {
				/* Save register addrress */
				if (strlen(token) % 2 == 0) {
					memset(reg_cmd, 0x00, sizeof(reg_cmd));
					reg_cmd_len = (uint8_t)(strlen(token) / 2);
					memset(temp, 0x00, sizeof(temp));

					for (i = 0; i < reg_cmd_len; i++) {
						memcpy(temp, token + i * 2, 2);

						if (!kstrtoul(temp, 16, &result)) {
							reg_cmd[reg_cmd_len - 1 - i] = result;
						} else {
							format_err = true;
							break;
						}
					}
				} else {
					format_err = true;
				}

				reg_addr = false;
			} else {
				/* Save register data */
				if ((token[0] == 'x') && (strlen(token) == 3)) {
					memset(temp, 0x00, sizeof(temp));
					memcpy(temp, token + 1, 2);

					if (!kstrtoul(temp, 16, &result)) {
						if (buf_tmp[0] == 'w') {
							w_data[w_data_len++] = (uint8_t)result;
						} else {
							if (byte_count == 0) {
								reg_read_len = (uint8_t)result;
								byte_count++;
							} else {
								reg_read_len += ((uint8_t)result)*256;
								break;
							}
						}
					} else {
						format_err = true;
					}
				} else {
					format_err = true;
				}
			}

			if (format_err) {
				JD_E("%s: (%s)Command format error!\n", __func__, token);
				return -EINVAL;
			}
		}

		if (buf_tmp[0] == 'w') {
			uint32_t WriteAddr = (reg_cmd[3] << 24) + (reg_cmd[2] << 16) +
									(reg_cmd[1] << 8) + reg_cmd[0];

			if (g_module_fp.fp_register_write(WriteAddr, w_data, w_data_len) < 0)
				JD_E("%s: Write register(%x) fail!\n", __func__, WriteAddr);
		}
	}

	return len;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0)
static struct proc_ops jadard_proc_register_ops = {
	.proc_open = jadard_proc_register_open,
	.proc_read = seq_read,
	.proc_release = seq_release,
	.proc_write = jadard_proc_register_write,
};
#else
static struct file_operations jadard_proc_register_ops = {
	.owner = THIS_MODULE,
	.open = jadard_proc_register_open,
	.read = seq_read,
	.release = seq_release,
	.write = jadard_proc_register_write,
};
#endif

static ssize_t jadard_proc_display_read(struct file *file, char *buf,
											size_t len, loff_t *pos)
{
	int ret = 0;
	uint8_t *rdata = NULL;
	char *buf_tmp = NULL;
	int i;

	if (len < 2048) {
		JD_E("%s: len size less than 2048\n", __func__);
		return -ENOMEM;
	}

	if (!pjadard_debug->proc_send_flag) {
		/* Read data from chip */
		if (dd_reg_read_len == 0) {
			/* if reg_read_len = 0, default 1 */
			dd_reg_read_len = 1;
		}

		rdata = kzalloc(dd_reg_read_len * sizeof(char), GFP_KERNEL);
		if (rdata == NULL) {
			JD_E("%s, Memory allocate fail: %d\n", __func__, __LINE__);
			return ret;
		}

		buf_tmp = kzalloc(len * sizeof(char), GFP_KERNEL);
		if (buf_tmp != NULL) {
			if (pjadard_ic_data->JD_MODULE_CASCADE_MODE == JD_CASCADE_MODE_ENABLE) {
				/* Read	 master dd register */
				if (g_module_fp.fp_dd_register_read(dd_reg_page, dd_reg_cmd, rdata, dd_reg_read_len,
					pjadard_ic_data->JD_MASTER_ADDR_OFFSET) == JD_DBIC_READ_WRITE_FAIL) {
					ret += scnprintf(buf_tmp + ret, len - ret, "Read master display register fail!\n");
				} else {
					/* Set output data */
					if (g_common_variable.dbi_dd_reg_mode == JD_DDREG_MODE_1) {
						ret += scnprintf(buf_tmp + ret, len - ret,
								"Master display register page: 0x%02X\n", dd_reg_page);
					}
					ret += scnprintf(buf_tmp + ret, len - ret, "Master display register: 0x%02X\n", dd_reg_cmd);

					for (i = 0; i < dd_reg_read_len; i++) {
						ret += scnprintf(buf_tmp + ret, len - ret, "0x%2.2X ", rdata[i]);

						if ((i % 16) == 15)
							ret += scnprintf(buf_tmp + ret, len - ret, "\n");
					}
					ret += scnprintf(buf_tmp + ret, len - ret, "\n");
				}

				/* Read secoond dd register */
				if (g_module_fp.fp_dd_register_read(dd_reg_page, dd_reg_cmd, rdata, dd_reg_read_len,
					pjadard_ic_data->JD_SECOND_ADDR_OFFSET) == JD_DBIC_READ_WRITE_FAIL) {
					ret += scnprintf(buf_tmp + ret, len - ret, "Read secoond display register fail!\n");
				} else {
					/* Set output data */
					if (g_common_variable.dbi_dd_reg_mode == JD_DDREG_MODE_1) {
						ret += scnprintf(buf_tmp + ret, len - ret,
								"Secoond display register page: 0x%02X\n", dd_reg_page);
					}
					ret += scnprintf(buf_tmp + ret, len - ret, "Secoond display register: 0x%02X\n", dd_reg_cmd);

					for (i = 0; i < dd_reg_read_len; i++) {
						ret += scnprintf(buf_tmp + ret, len - ret, "0x%2.2X ", rdata[i]);

						if ((i % 16) == 15)
							ret += scnprintf(buf_tmp + ret, len - ret, "\n");
					}
					ret += scnprintf(buf_tmp + ret, len - ret, "\n");
				}
			} else {
				if (g_module_fp.fp_dd_register_read(dd_reg_page, dd_reg_cmd, rdata, dd_reg_read_len, 0) == JD_DBIC_READ_WRITE_FAIL) {
					ret += scnprintf(buf_tmp + ret, len - ret, "Read display register fail!\n");
				} else {
					/* Set output data */
					if (g_common_variable.dbi_dd_reg_mode == JD_DDREG_MODE_1)
						ret += scnprintf(buf_tmp + ret, len - ret, "Display register page: 0x%02X\n", dd_reg_page);
					ret += scnprintf(buf_tmp + ret, len - ret, "Display register: 0x%02X\n", dd_reg_cmd);

					for (i = 0; i < dd_reg_read_len; i++) {
						ret += scnprintf(buf_tmp + ret, len - ret, "0x%2.2X ", rdata[i]);

						if ((i % 16) == 15)
							ret += scnprintf(buf_tmp + ret, len - ret, "\n");
					}
					ret += scnprintf(buf_tmp + ret, len - ret, "\n");
				}
			}

			if (copy_to_user(buf, buf_tmp, len))
				JD_E("%s, copy_to_user fail: %d\n", __func__, __LINE__);
			kfree(buf_tmp);
		} else {
			JD_E("%s, Memory allocate fail: %d\n", __func__, __LINE__);
		}

		kfree(rdata);
		pjadard_debug->proc_send_flag = true;
	} else {
		pjadard_debug->proc_send_flag = false;
	}

	return ret;
}

/*
 * Example: Only support lower case
 * ADDRESS: 1byte size, according to the chip of HW design
 * < MODE_0 >
 * Write display register: echo w:xADDRESS:x11:x22:x33 > /proc/jadard_touch/display
 *						   write ADDRESS 3bytes data(0x11,0x22,0x33), MAX:64bytes
 * Read display register: echo r:xADDRESS:x11 > /proc/jadard_touch/display
 *						  read ADDRESS 0x11(17)bytes, MAX:0xFF
 * < MODE_1 >
 * Write display register: echo w:xPAGE:xADDRESS:x11:x22:x33 > /proc/jadard_touch/display
 *						   (PAGE >> 4) & 0x0F : Set write Master(2)/Second(1)/Both(0)
 *						   (PAGE >> 0) & 0x0F : Set page(0-15) of the Master/Second/Both
 *						   write ADDRESS of the Master/Second/Both page 3bytes data(0x11,0x22,0x33), MAX:128bytes
 * Read display register: echo r:xPAGE:xADDRESS:x11 > /proc/jadard_touch/display
 *						  read ADDRESS of the PAGE 0x11(17)bytes, MAX:0xFF
 * Don't care PAGE when nokia standard comand
 * Write sleep in: echo w:x00:x10 > /proc/jadard_touch/display
 */
static ssize_t jadard_proc_display_write(struct file *file, const char *buf,
											size_t len, loff_t *pos)
{
	char buf_tmp[130 * 4 + 10] = {0};
	char tok_data[130 * 4 + 10] = {0};
	char *p_tok_data = tok_data;
	char *token = NULL;
	char *const delim = ":";
	uint8_t w_data[130];
	uint8_t w_data_len = 0;
	unsigned long result = 0;
	bool reg_page = false;
	bool reg_addr = true;
	bool format_err = false;
	char temp[4];

	if (g_common_variable.dbi_dd_reg_mode == JD_DDREG_MODE_1) {
		reg_page = true;
		reg_addr = false;
	} else { /* MODE_0 */
		reg_page = false;
		reg_addr = true;
	}

	if (len >= sizeof(buf_tmp)) {
		JD_I("%s: no command exceeds %ld chars.\n", __func__, sizeof(buf_tmp));
		return -EFAULT;
	}

	if (copy_from_user(buf_tmp, buf, (len - 1 < sizeof(buf_tmp) - 1) ? len - 1 : sizeof(buf_tmp) - 1))
		return -EFAULT;

	JD_I("%s: buf_tmp = %s\n", __func__, buf_tmp);

	if ((buf_tmp[0] == 'r' || buf_tmp[0] == 'w') && buf_tmp[1] == ':' && buf_tmp[2] == 'x') {
		memset(w_data, 0x00, sizeof(w_data));
		memcpy(p_tok_data, buf_tmp + 2, strlen(buf_tmp) - 2);

		while ((token = strsep(&p_tok_data, delim)) != NULL) {
			if ((token[0] == 'x') && (strlen(token) == 3)) {
				memset(temp, 0x00, sizeof(temp));
				memcpy(temp, token + 1, 2);

				if (reg_page) {
					/* Save register page */
					dd_reg_m_s_b = 0x00;
					dd_reg_page = 0x00;

					if (!kstrtoul(temp, 16, &result)) {
						dd_reg_m_s_b = (uint8_t)(result >> 4) & 0x0F;
						dd_reg_page = (uint8_t)result & 0x0F;
					} else {
						format_err = true;
					}

					reg_page = false;
					reg_addr = true;
				} else {
					/* Save register addr/data */
					if (!kstrtoul(temp, 16, &result)) {
						if (reg_addr) {
							dd_reg_cmd = (uint8_t)result;
							reg_addr = false;
						} else {
							if (buf_tmp[0] == 'w') {
								w_data[w_data_len++] = (uint8_t)result;
							} else {
								dd_reg_read_len = (uint8_t)result;
								break;
							}
						}
					} else {
						format_err = true;
					}
				}
			} else {
				format_err = true;
			}

			if (format_err) {
				JD_E("%s: (%s)Command format error!\n", __func__, token);
				return -EINVAL;
			}
		}

		if (buf_tmp[0] == 'w') {
			if (g_module_fp.fp_dd_register_write(dd_reg_page, dd_reg_cmd, w_data, w_data_len, dd_reg_m_s_b) == JD_DBIC_READ_WRITE_FAIL)
				JD_E("%s: Write display register(%x) fail!\n", __func__, dd_reg_cmd);

			dd_reg_read_len = w_data_len;
		}
	}

	return len;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0)
static struct proc_ops jadard_proc_display_ops	= {
	.proc_read = jadard_proc_display_read,
	.proc_write = jadard_proc_display_write,
};
#else
static struct file_operations jadard_proc_display_ops  = {
	.owner = THIS_MODULE,
	.read = jadard_proc_display_read,
	.write = jadard_proc_display_write,
};
#endif

/*
 * dbi_std_mode_enable:
 * 0: Only can write nokia standard command(0x10 0x11 0x28 0x29 0x34 0x35)
 * 1: Can write all nokia standard command
 */
static ssize_t jadard_proc_display_std_write(struct file *file, const char *buf,
											size_t len, loff_t *pos)
{
	char buf_tmp[128] = {0};

	if (len >= sizeof(buf_tmp)) {
		JD_I("%s: no command exceeds %ld chars.\n", __func__, sizeof(buf_tmp));
		return -EFAULT;
	}

	if (copy_from_user(buf_tmp, buf, (len < sizeof(buf_tmp) - 1) ? len : sizeof(buf_tmp) - 1))
		return -EFAULT;

	JD_I("%s: buf_tmp = %s", __func__, buf_tmp);

	if ((buf_tmp[0] == 'S') && (buf_tmp[1] == 'T') && (buf_tmp[2] == 'D')) {
		 pjadard_ts_data->dbi_std_mode_enable = !(pjadard_ts_data->dbi_std_mode_enable);

		if (pjadard_ts_data->dbi_std_mode_enable)
			JD_I("Dbi std mode enable\n");
		else
			JD_I("Dbi std mode disable\n");
	} else {
		JD_E("%s: Command format error!\n", __func__);
	}

	return len;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0)
static struct proc_ops jadard_proc_display_std_ops	= {
	.proc_write = jadard_proc_display_std_write,
};
#else
static struct file_operations jadard_proc_display_std_ops  = {
	.owner = THIS_MODULE,
	.write = jadard_proc_display_std_write,
};
#endif

static ssize_t jadard_debug_read(struct file *file, char *buf,
								size_t len, loff_t *pos)
{
	size_t ret = 0;
	char *buf_tmp = NULL;

	if (len < 512) {
		JD_E("%s: len size less than 512\n", __func__);
		return -ENOMEM;
	}

	if (!pjadard_debug->proc_send_flag) {
		buf_tmp = kzalloc(len * sizeof(char), GFP_KERNEL);
		if (buf_tmp != NULL) {
			if (debug_cmd == 'v') {
				ret += scnprintf(buf_tmp + ret, len - ret, "IC ID: %s\n", pjadard_ic_data->chip_id);
				ret += scnprintf(buf_tmp + ret, len - ret, "FW_VER: %08x\n", pjadard_ic_data->fw_ver);
				ret += scnprintf(buf_tmp + ret, len - ret, "FW_CID_VER: %08x\n", pjadard_ic_data->fw_cid_ver);
				ret += scnprintf(buf_tmp + ret, len - ret, "PANEL_MAKER: %s\n", pjadard_ic_data->panel_maker);
				ret += scnprintf(buf_tmp + ret, len - ret, "PANEL_VER: %02x\n", pjadard_ic_data->panel_ver);
				ret += scnprintf(buf_tmp + ret, len - ret, "Driver_VER: %s\n", JADARD_DRIVER_VER);
				ret += scnprintf(buf_tmp + ret, len - ret, "Driver_CID_VER: %s\n", JADARD_DRIVER_CID_VER);
			} else if (debug_cmd == 'i') {
				ret += scnprintf(buf_tmp + ret, len - ret, "Jadard Touch IC Information: ");
				ret += scnprintf(buf_tmp + ret, len - ret, "%s\n", pjadard_ic_data->chip_id);

				if (pjadard_ic_data->JD_INT_EDGE)
					ret += scnprintf(buf_tmp + ret, len - ret, "Driver register Interrupt: EDGE TIRGGER\n");
				else
					ret += scnprintf(buf_tmp + ret, len - ret, "Driver register Interrupt: LEVEL TRIGGER\n");

				if (pjadard_ts_data->protocol_type == PROTOCOL_TYPE_A)
					ret += scnprintf(buf_tmp + ret, len - ret, "Protocol: TYPE_A\n");
				else
					ret += scnprintf(buf_tmp + ret, len - ret, "Protocol: TYPE_B\n");

				ret += scnprintf(buf_tmp + ret, len - ret, "Freq_Band: %d\n", g_module_fp.fp_get_freq_band());
				ret += scnprintf(buf_tmp + ret, len - ret, "X_Num: %d\n", pjadard_ic_data->JD_X_NUM);
				ret += scnprintf(buf_tmp + ret, len - ret, "Y_Num: %d\n", pjadard_ic_data->JD_Y_NUM);
				ret += scnprintf(buf_tmp + ret, len - ret, "X_Resolution: %d\n", pjadard_ic_data->JD_X_RES);
				ret += scnprintf(buf_tmp + ret, len - ret, "Y_Resolution: %d\n", pjadard_ic_data->JD_Y_RES);
				ret += scnprintf(buf_tmp + ret, len - ret, "Max Points: %d\n", pjadard_ic_data->JD_MAX_PT);
				ret += scnprintf(buf_tmp + ret, len - ret, "Product_Type: %d\n", JD_PRODUCT_TYPE);
				ret += scnprintf(buf_tmp + ret, len - ret, "Cascade_Mode: %d\n", pjadard_ic_data->JD_MODULE_CASCADE_MODE);
			} else if (debug_cmd == 't') {
				if (fw_upgrade_complete)
					ret += scnprintf(buf_tmp + ret, len - ret, "FW Upgrade Complete\n");
				else
					ret += scnprintf(buf_tmp + ret, len - ret, "FW Upgrade Fail\n");
			} else if (debug_cmd == 'm') {
				if (pjadard_ic_data->JD_DISABLE_MASTER_TO_SECOND)
					ret += scnprintf(buf_tmp + ret, len - ret, "Direct read secoond register\n");
				else
					ret += scnprintf(buf_tmp + ret, len - ret, "Read secoond by master\n");
			} else if (debug_cmd == 'd') {
				if (jd_g_dbg_enable)
					ret += scnprintf(buf_tmp + ret, len - ret, "Debug Enable\n");
				else
					ret += scnprintf(buf_tmp + ret, len - ret, "Debug Disable\n");
			}

			if (copy_to_user(buf, buf_tmp, len))
				JD_E("%s, copy_to_user fail: %d\n", __func__, __LINE__);

			kfree(buf_tmp);
		} else {
			JD_E("%s, Memory allocate fail: %d\n", __func__, __LINE__);
		}

		pjadard_debug->proc_send_flag = true;
	} else {
		pjadard_debug->proc_send_flag = false;
	}

	return ret;
}

static ssize_t jadard_debug_write(struct file *file, const char *buf,
								 size_t len, loff_t *pos)
{
	char buf_tmp[128] = {0};
	char fileName[128];
	int ret = 0;
#if !defined(JD_ZERO_FLASH) && !defined(JD_UPGRADE_FW_ARRAY)
	const struct firmware *fw = NULL;
#endif

	if (len >= sizeof(buf_tmp)) {
		JD_I("%s: no command exceeds %ld chars.\n", __func__, sizeof(buf_tmp));
		return -EFAULT;
	}

	if (copy_from_user(buf_tmp, buf, (len < sizeof(buf_tmp) - 1) ? len : sizeof(buf_tmp) - 1))
		return -EFAULT;

	JD_I("%s: buf_tmp = %s", __func__, buf_tmp);

	if (buf_tmp[0] == 'v') {
		/* FW and Driver version */
		g_module_fp.fp_read_fw_ver();
		debug_cmd = buf_tmp[0];
	} else if ((buf_tmp[0] == 'i') && (buf_tmp[1] != 'b')) {
		/* IC information */
		debug_cmd = buf_tmp[0];
	} else if (buf_tmp[0] == 't') {
		if (pjadard_ts_data->diag_thread_active) {
			JD_I("Diag thread is active! Skip Update with zero flash\n");
			return -EFAULT;
		}
		debug_cmd = buf_tmp[0];
		/* Start upgrade flow*/
		jadard_int_enable(false);
		fw_upgrade_complete = false;
		memset(fileName, 0, sizeof(fileName));
		scnprintf(fileName, len - 2, "%s", buf_tmp + 2);
#ifndef JD_UPGRADE_FW_ARRAY
		JD_I("%s: Bin file name(%s)\n", __func__, fileName);
#endif

#ifdef JD_ESD_CHECK
		if (pjadard_ts_data->esd_check_running == true) {
			JD_I("Stop esd check\n");
			pjadard_ts_data->esd_check_running = false;
			cancel_delayed_work_sync(&pjadard_ts_data->work_esd_check);
		}
#endif

#ifdef JD_ZERO_FLASH
		JD_I("Running Zero flash upgrade\n");

		pjadard_ts_data->power_on_upgrade = true;
		ret = g_module_fp.fp_0f_upgrade_fw(fileName);
		pjadard_ts_data->power_on_upgrade = false;

		if (ret < 0) {
			fw_upgrade_complete = false;
			JD_I("Zero flash upgrade fail\n");
		} else {
			fw_upgrade_complete = true;
			JD_I("Zero flash upgrade success\n");
		}
#else
		JD_I("Running flash upgrade\n");
#ifdef JD_UPGRADE_FW_ARRAY
		JD_I("file name = %s\n", jd_i_CTPM_firmware_name);
		JD_I("FW size is %dBytes\n", (int)jd_fw_size);

		ret = g_module_fp.fp_flash_write(0, (uint8_t *)jd_i_firmware, jd_fw_size);
		if (ret < 0) {
			JD_E("%s: TP upgrade fail\n", __func__);
			fw_upgrade_complete = false;
		} else {
			JD_I("%s: TP upgrade success\n", __func__);
			fw_upgrade_complete = true;
		}
#else
		ret = request_firmware(&fw, fileName, pjadard_ts_data->dev);
		if (ret < 0) {
			JD_I("Fail to open file: %s (ret:%d)\n", fileName, ret);
			return ret;
		}

		JD_I("FW size is %dBytes\n", (int)fw->size);

		if (g_module_fp.fp_flash_write(0, (uint8_t *)fw->data, fw->size) < 0) {
			JD_E("%s: TP upgrade fail\n", __func__);
			fw_upgrade_complete = false;
		} else {
			JD_I("%s: TP upgrade success\n", __func__);
			fw_upgrade_complete = true;
		}

		release_firmware(fw);
#endif
#ifdef JD_FLASH_WP_EN
		g_module_fp.fp_FlashUnlock();
#endif
#endif
		g_module_fp.fp_read_fw_ver();
		jadard_int_enable(true);

#ifdef JD_ESD_CHECK
		if (pjadard_ts_data->esd_check_running == false) {
			JD_I("Start esd check\n");
			pjadard_ts_data->esd_check_running = true;
			queue_delayed_work(pjadard_ts_data->jadard_esd_check_wq, &pjadard_ts_data->work_esd_check, 0);
		}
#endif
#ifndef JD_ZERO_FLASH
	} else if ((buf_tmp[0] == 'e') && (buf_tmp[1] == 'r') && (buf_tmp[2] == 'a') &&
				(buf_tmp[3] == 's') && (buf_tmp[4] == 'e')) {
#ifdef JD_ESD_CHECK
		if (pjadard_ts_data->esd_check_running == true) {
			JD_I("Stop esd check\n");
			pjadard_ts_data->esd_check_running = false;
			cancel_delayed_work_sync(&pjadard_ts_data->work_esd_check);
		}
#endif

#ifdef JD_FLASH_WP_EN
		g_module_fp.fp_FlashUnlock();
#endif
		if (g_module_fp.fp_flash_erase() < 0)
			JD_E("%s: Flash erase fail\n", __func__);
		else
			JD_I("%s: Flash erase finish\n", __func__);
#endif
	} else if (buf_tmp[0] == 'T') {
		if (pjadard_ts_data->diag_thread_active) {
			JD_I("Diag thread is active! Skip Update with zero flash\n");
			return -EFAULT;
		}
		debug_cmd = buf_tmp[0];
		/* Start upgrade flow*/
		jadard_int_enable(false);
		fw_upgrade_complete = false;
		memset(fileName, 0, sizeof(fileName));
		scnprintf(fileName, len - 2, "%s", buf_tmp + 2);
#ifndef JD_UPGRADE_FW_ARRAY
		JD_I("%s: Bin file name(%s)\n", __func__, fileName);
#endif
#ifdef JD_ZERO_FLASH
		JD_I("Running Zero flash upgrade\n");

		ret = g_module_fp.fp_0f_upgrade_fw(fileName);
		if (ret < 0) {
			fw_upgrade_complete = false;
			JD_I("Zero flash upgrade fail\n");
		} else {
			fw_upgrade_complete = true;
			JD_I("Zero flash upgrade success\n");
			ret = len;
		}
#else
		JD_I("Running flash upgrade\n");
#ifdef JD_UPGRADE_FW_ARRAY
		JD_I("file name = %s\n", jd_i_CTPM_firmware_name);
		JD_I("FW size is %dBytes\n", (int)jd_fw_size);

		ret = g_module_fp.fp_flash_write(0, (uint8_t *)jd_i_firmware, jd_fw_size);
		if (ret < 0) {
			JD_E("%s: TP upgrade fail\n", __func__);
			fw_upgrade_complete = false;
		} else {
			JD_I("%s: TP upgrade success\n", __func__);
			fw_upgrade_complete = true;
			ret = len;
		}
#else
		ret = request_firmware(&fw, fileName, pjadard_ts_data->dev);
		if (ret < 0) {
			JD_I("Fail to open file: %s (ret:%d)\n", fileName, ret);
			return ret;
		}

		JD_I("FW size is %dBytes\n", (int)fw->size);

		ret = g_module_fp.fp_flash_write(0, (uint8_t *)fw->data, fw->size);
		if (ret < 0) {
			JD_E("%s: TP upgrade fail\n", __func__);
			fw_upgrade_complete = false;
		} else {
			JD_I("%s: TP upgrade success\n", __func__);
			fw_upgrade_complete = true;
			ret = len;
		}

		release_firmware(fw);
#endif
#endif
		g_module_fp.fp_read_fw_ver();
		jadard_int_enable(true);

		return ret;
#ifdef JD_ESD_CHECK
	} else if (buf_tmp[0] == 'E') {
		debug_cmd = buf_tmp[0];
		if (buf_tmp[1] == '1') {
			if (pjadard_ts_data->esd_check_running == false) {
				JD_I("Start esd check\n");
				pjadard_ts_data->esd_check_running = true;
				queue_delayed_work(pjadard_ts_data->jadard_esd_check_wq, &pjadard_ts_data->work_esd_check, 0);
			}
		} else {
			if (pjadard_ts_data->esd_check_running == true) {
				JD_I("Stop esd check\n");
				pjadard_ts_data->esd_check_running = false;
				cancel_delayed_work_sync(&pjadard_ts_data->work_esd_check);
			}
		}
#endif
	} else if (buf_tmp[0] == 'm') {
		debug_cmd = buf_tmp[0];
		if (pjadard_ic_data->JD_MODULE_CASCADE_MODE == JD_CASCADE_MODE_ENABLE) {
			pjadard_ic_data->JD_DISABLE_MASTER_TO_SECOND = !(pjadard_ic_data->JD_DISABLE_MASTER_TO_SECOND);

			if (pjadard_ic_data->JD_DISABLE_MASTER_TO_SECOND)
				JD_I("Direct read secoond register\n");
			else
				JD_I("Read secoond by master\n");
		} else {
			pjadard_ic_data->JD_DISABLE_MASTER_TO_SECOND = true;
			JD_I("Direct read secoond register\n");
		}
	} else if (buf_tmp[0] == 'd') {
		debug_cmd = buf_tmp[0];
		jd_g_dbg_enable = !jd_g_dbg_enable;

		if (jd_g_dbg_enable)
			JD_I("Debug Enable\n");
		else
			JD_I("Debug Disable\n");
	} else if ((buf_tmp[0] == 'i') && (buf_tmp[1] == 'b')) {
		g_module_fp.fp_EnterBackDoor();
		JD_I("Enter backdoor\n");
	} else if ((buf_tmp[0] == 'o') && (buf_tmp[1] == 'b')) {
		g_module_fp.fp_ExitBackDoor();
		JD_I("Exit backdoor\n");
	} else {
		debug_cmd = 0;
	}

	return len;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0)
static struct proc_ops jadard_proc_debug_ops = {
	.proc_read = jadard_debug_read,
	.proc_write = jadard_debug_write,
};
#else
static struct file_operations jadard_proc_debug_ops = {
	.owner = THIS_MODULE,
	.read = jadard_debug_read,
	.write = jadard_debug_write,
};
#endif

int jadard_touch_proc_init(void)
{
	jadard_proc_report_debug_file = proc_create(JADARD_PROC_REPORT_DEBUG_FILE, (S_IWUGO | S_IRUGO),
								  pjadard_touch_proc_dir, &jadard_proc_debug_level_ops);
	if (jadard_proc_report_debug_file == NULL) {
		JD_E(" %s: proc report_debug file create failed!\n", __func__);
		goto fail;
	}

	jadard_proc_fw_package_file = proc_create(JADARD_PROC_FW_PACKAGE_FILE, (S_IWUGO | S_IRUGO),
								  pjadard_touch_proc_dir, &jadard_proc_fw_package_ops);
	if (jadard_proc_fw_package_file == NULL) {
		JD_E(" %s: proc report_debug file create failed!\n", __func__);
		goto fail;
	}

	jadard_proc_reset_file = proc_create(JADARD_PROC_RESET_FILE, (S_IWUGO),
										pjadard_touch_proc_dir, &jadard_proc_reset_ops);
	if (jadard_proc_reset_file == NULL) {
		JD_E(" %s: proc reset file create failed!\n", __func__);
		goto fail;
	}

	jadard_proc_attn_file = proc_create(JADARD_PROC_ATTN_FILE, (S_IRUGO),
									   pjadard_touch_proc_dir, &jadard_proc_attn_ops);
	if (jadard_proc_attn_file == NULL) {
		JD_E(" %s: proc attn file create failed!\n", __func__);
		goto fail;
	}

	jadard_proc_int_en_file = proc_create(JADARD_PROC_INT_EN_FILE, (S_IWUGO | S_IRUGO),
										 pjadard_touch_proc_dir, &jadard_proc_int_en_ops);
	if (jadard_proc_int_en_file == NULL) {
		JD_E(" %s: proc int en file create failed!\n", __func__);
		goto fail;
	}

	jadard_proc_fw_dump_file = proc_create(JADARD_PROC_FW_DUMP_FILE, (S_IWUGO | S_IRUGO),
								 pjadard_touch_proc_dir, &jadard_proc_fw_dump_ops);
	if (jadard_proc_fw_dump_file == NULL) {
		JD_E(" %s: proc flash dump file create failed!\n", __func__);
		goto fail;
	}

	jadard_proc_diag_file = proc_create(JADARD_PROC_DIAG_FILE, (S_IWUGO | S_IRUGO),
									   pjadard_touch_proc_dir, &jadard_proc_diag_ops);
	if (jadard_proc_diag_file == NULL) {
		JD_E(" %s: proc diag file create failed!\n", __func__);
		goto fail;
	}

	jadard_proc_diag_arr_file = proc_create(JADARD_PROC_DIAG_ARR_FILE, (S_IWUGO | S_IRUGO),
								   pjadard_touch_proc_dir, &jadard_proc_diag_arrange_ops);
	if (jadard_proc_diag_arr_file == NULL) {
		JD_E(" %s: proc diag file create failed!\n", __func__);
		goto fail;
	}

	jadard_proc_register_file = proc_create(JADARD_PROC_REGISTER_FILE, (S_IWUGO | S_IRUGO),
										   pjadard_touch_proc_dir, &jadard_proc_register_ops);
	if (jadard_proc_register_file == NULL) {
		JD_E(" %s: proc register file create failed!\n", __func__);
		goto fail;
	}

	jadard_proc_display_file = proc_create(JADARD_PROC_DISPLAY_FILE, (S_IWUGO | S_IRUGO),
										   pjadard_touch_proc_dir, &jadard_proc_display_ops);
	if (jadard_proc_display_file == NULL) {
		JD_E(" %s: proc register file create failed!\n", __func__);
		goto fail;
	}

	if (g_common_variable.dbi_dd_reg_mode == JD_DDREG_MODE_1) {
		jadard_proc_display_std_file = proc_create(JADARD_PROC_DISPLAY_STD_FILE, (S_IWUGO),
											   pjadard_touch_proc_dir, &jadard_proc_display_std_ops);
		if (jadard_proc_display_std_file == NULL) {
			JD_E(" %s: proc register file create failed!\n", __func__);
			goto fail;
		}
	}

	jadard_proc_debug_file = proc_create(JADARD_PROC_DEBUG_FILE, (S_IWUGO | S_IRUGO),
										pjadard_touch_proc_dir, &jadard_proc_debug_ops);
	if (jadard_proc_debug_file == NULL) {
		JD_E(" %s: proc debug file create failed!\n", __func__);
		goto fail;
	}

#ifdef JD_RAWDATA_V2
	jadard_proc_diag_apk_file = proc_create(JADARD_PROC_DIAG_APK_FILE, (S_IWUGO | S_IRUGO),
									   pjadard_touch_proc_dir, &jadard_proc_diag_apk_ops);
	if (jadard_proc_diag_apk_file == NULL) {
		JD_E(" %s: proc diag apk file create failed!\n", __func__);
		goto fail;
	}
#endif

	jadard_proc_buf_rd_file = proc_create(JADARD_PROC_BUF_RD_FILE, (S_IWUGO | S_IRUGO),
									   pjadard_touch_proc_dir, &jadard_proc_buf_rd_ops);
	if (jadard_proc_buf_rd_file == NULL) {
		JD_E(" %s: proc diag apk file create failed!\n", __func__);
		goto fail;
	}

	return 0;

fail:
	jadard_touch_proc_deinit();

	return -ENOMEM;
}

static void jadard_debug_data_init(void)
{
	/* Global variable initial */
	jd_diag_mutual_fn = NULL;
	jd_diag_mutual = NULL;
	jd_diag_mutual_cnt = 0;
	jd_buf = NULL;
	diag_arr_num = 0;
	reg_read_len = 1;
	dd_reg_read_len = 1;
	fw_buffer = NULL;
	debug_cmd = 'v';
	fw_upgrade_complete = false;
	jd_g_dbg_enable = false;
	jd_g_buf_rd_enable = false;
	buf_rd_byte_num = 2;

	pjadard_debug->fp_touch_dbg_func = jadard_touch_dbg_func;
	pjadard_debug->fw_dump_going = &fw_dump_going;
	pjadard_debug->proc_send_flag = false;
}

int jadard_debug_init(void)
{
	struct jadard_ts_data *ts = pjadard_ts_data;
	int err = 0;

	JD_I("%s:Enter\n", __func__);

	if (ts == NULL) {
		JD_E("%s: pjadard_ts_data struct is NULL\n", __func__);
		return -EPROBE_DEFER;
	}

	pjadard_debug = kzalloc(sizeof(*pjadard_debug), GFP_KERNEL);
	if (pjadard_debug == NULL) {
		JD_E("%s, pjadard_debug buffer allocate fail\n", __func__);
		err = -ENOMEM;
		goto err_alloc_debug_data_failed;
	}

	jadard_debug_data_init();

	ts->fw_dump_wq = create_singlethread_workqueue("jadard_fw_dump_wq");

	if (!ts->fw_dump_wq) {
		JD_E("%s: create flash workqueue failed\n", __func__);
		err = -ENOMEM;
		goto err_create_fw_dump_wq_failed;
	}

	INIT_WORK(&ts->fw_dump_work, jadard_ts_fw_dump_work_func);
	fw_dump_busy = false;
	fw_buffer = kzalloc(g_common_variable.FW_SIZE * sizeof(uint8_t), GFP_KERNEL);

	if (fw_buffer == NULL) {
		JD_E("%s: FW buffer allocate failed\n", __func__);
		err = -ENOMEM;
		goto err_alloc_fw_buffer_failed;
	}

	ts->jadard_diag_wq = create_singlethread_workqueue("jadard_diag");

	if (!ts->jadard_diag_wq) {
		JD_E("%s: create diag workqueue failed\n", __func__);
		err = -ENOMEM;
		goto err_create_diag_wq_failed;
	}

	INIT_DELAYED_WORK(&ts->jadard_diag_delay_wrok, jadard_ts_diag_work_func);

	/* Allocate mutual data memory */
	jd_diag_mutual = kzalloc(pjadard_ic_data->JD_X_NUM *
							pjadard_ic_data->JD_Y_NUM * sizeof(int), GFP_KERNEL);

	if (jd_diag_mutual == NULL) {
		JD_E("%s: mutual buffer allocate failed\n", __func__);
		err = -ENOMEM;
		goto err_alloc_memory_failed;
	}

	jd_buf = kzalloc(pjadard_ic_data->JD_X_NUM * pjadard_ic_data->JD_Y_NUM *
					buf_rd_byte_num * sizeof(uint8_t), GFP_KERNEL);

	if (jd_buf == NULL) {
		JD_E("%s: Rawdata buffer allocate failed\n", __func__);
		err = -ENOMEM;
		goto err_alloc_jd_buf_failed;
	}

	jadard_touch_proc_init();

	return 0;

err_alloc_jd_buf_failed:
	kfree(jd_diag_mutual);
err_alloc_memory_failed:
	cancel_delayed_work_sync(&ts->jadard_diag_delay_wrok);
	destroy_workqueue(ts->jadard_diag_wq);
err_create_diag_wq_failed:
	destroy_workqueue(ts->fw_dump_wq);
err_alloc_fw_buffer_failed:
	kfree(fw_buffer);
err_create_fw_dump_wq_failed:
	kfree(pjadard_debug);
err_alloc_debug_data_failed:

	return err;
}

void jadard_touch_proc_deinit(void)
{
	if (jadard_proc_buf_rd_file) {
		remove_proc_entry(JADARD_PROC_BUF_RD_FILE, pjadard_touch_proc_dir);
		jadard_proc_buf_rd_file = NULL;
	}

	if (jadard_proc_diag_apk_file) {
		remove_proc_entry(JADARD_PROC_DIAG_APK_FILE, pjadard_touch_proc_dir);
		jadard_proc_diag_apk_file = NULL;
	}

	if (jadard_proc_debug_file) {
		remove_proc_entry(JADARD_PROC_DEBUG_FILE, pjadard_touch_proc_dir);
		jadard_proc_debug_file = NULL;
	}

	if (g_common_variable.dbi_dd_reg_mode == JD_DDREG_MODE_1) {
		if (jadard_proc_display_std_file) {
			remove_proc_entry(JADARD_PROC_DISPLAY_STD_FILE, pjadard_touch_proc_dir);
			jadard_proc_display_std_file = NULL;
		}
	}

	if (jadard_proc_display_file) {
		remove_proc_entry(JADARD_PROC_DISPLAY_FILE, pjadard_touch_proc_dir);
		jadard_proc_display_file = NULL;
	}

	if (jadard_proc_register_file) {
		remove_proc_entry(JADARD_PROC_REGISTER_FILE, pjadard_touch_proc_dir);
		jadard_proc_register_file = NULL;
	}

	if (jadard_proc_diag_arr_file) {
		remove_proc_entry(JADARD_PROC_DIAG_ARR_FILE, pjadard_touch_proc_dir);
		jadard_proc_diag_arr_file = NULL;
	}

	if (jadard_proc_diag_file) {
		remove_proc_entry(JADARD_PROC_DIAG_FILE, pjadard_touch_proc_dir);
		jadard_proc_diag_file = NULL;
	}

	if (jadard_proc_fw_dump_file) {
		remove_proc_entry(JADARD_PROC_FW_DUMP_FILE, pjadard_touch_proc_dir);
		jadard_proc_fw_dump_file = NULL;
	}

	if (jadard_proc_int_en_file) {
		remove_proc_entry(JADARD_PROC_INT_EN_FILE, pjadard_touch_proc_dir);
		jadard_proc_int_en_file = NULL;
	}

	if (jadard_proc_attn_file) {
		remove_proc_entry(JADARD_PROC_ATTN_FILE, pjadard_touch_proc_dir);
		jadard_proc_attn_file = NULL;
	}

	if (jadard_proc_reset_file) {
		remove_proc_entry(JADARD_PROC_RESET_FILE, pjadard_touch_proc_dir);
		jadard_proc_reset_file = NULL;
	}

	if (jadard_proc_fw_package_file) {
		remove_proc_entry(JADARD_PROC_FW_PACKAGE_FILE, pjadard_touch_proc_dir);
		jadard_proc_fw_package_file = NULL;
	}

	if (jadard_proc_report_debug_file) {
		remove_proc_entry(JADARD_PROC_REPORT_DEBUG_FILE, pjadard_touch_proc_dir);
		jadard_proc_report_debug_file = NULL;
	}
}

int jadard_debug_remove(void)
{
	struct jadard_ts_data *ts = pjadard_ts_data;

	jadard_touch_proc_deinit();
	kfree(jd_buf);
	kfree(jd_diag_mutual);
	cancel_delayed_work_sync(&ts->jadard_diag_delay_wrok);
	destroy_workqueue(ts->jadard_diag_wq);
	destroy_workqueue(ts->fw_dump_wq);
	kfree(fw_buffer);
	kfree(pjadard_debug);

	return 0;
}

#if (!defined(CONFIG_JD_DB)) && (!defined(__JADARD_GKI__))
MODULE_IMPORT_NS(VFS_internal_I_am_really_a_filesystem_and_am_NOT_a_driver);
MODULE_IMPORT_NS(ANDROID_GKI_VFS_EXPORT_ONLY);
#endif
