/* SPDX-License-Identifier: GPL-2.0 */
/*  Himax Android Driver Sample Code for debug nodes
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

#include "himax_debug.h"
#include "himax_ic_core.h"

#if defined(HX_TP_PROC_2T2R)
	bool Is_2T2R;
EXPORT_SYMBOL(Is_2T2R);
	int HX_RX_NUM_2;
	int HX_TX_NUM_2;
#endif

static void hx_dbg_rst_rb(void);

uint8_t g_diag_arr_num;
int g_re_ass_rb_idx = -9487;

int g_max_mutual;
int g_min_mutual = 0xFFFF;
int g_max_self;
int g_min_self = 0xFFFF;

/* moved from debug.h */

static char g_file_path[256];

uint8_t proc_reg_addr[4];
uint8_t proc_reg_addr_type;
uint8_t *proc_reg_buf;

struct proc_dir_entry *himax_proc_stack_file;
struct proc_dir_entry *himax_proc_delta_file;
struct proc_dir_entry *himax_proc_dc_file;
struct proc_dir_entry *himax_proc_baseline_file;
bool dsram_flag;

#if defined(HX_TP_PROC_2T2R)
uint32_t *diag_mutual_2;
#endif
int32_t *diag_mutual;
uint8_t *diag_mutual_byte;
int32_t *diag_mutual_new;
int32_t *diag_mutual_old;
uint8_t state_info[2];
uint8_t diag_coor[128];
int32_t *diag_self;
uint8_t *diag_self_byte;
int32_t *diag_self_new;
int32_t *diag_self_old;

struct proc_dir_entry *himax_proc_debug_file;
bool	fw_update_complete;
bool	fw_update_going;
int handshaking_result;
unsigned char debug_level_cmd;
uint8_t cmd_set[8];
uint8_t mutual_set_flag;

struct proc_dir_entry *himax_proc_flash_dump_file;

uint32_t **raw_data_array;
uint8_t X_NUM;
uint8_t Y_NUM;
uint8_t sel_type = 0x0D;

/* Moved from debug.h End */

struct proc_dir_entry *himax_proc_pen_pos_file;
static struct dump_func g_dump;
struct time_var timeStart, timeEnd, timeDelta;
int g_switch_mode;
/*
 *	Segment : Himax PROC Debug Function
 */

/****** useful functions ******/
static void _str_to_arr_in_char(char **arr,
	int arr_size, const char *str, char c)
{
	int i = 0;
	int _arr_str_idx = 0;
	int _arr_idx = 0;
	int _arr_max_idx = arr_size;

	for (i = 0; str[i] != '\0'; i++) {
		if (_arr_max_idx <= _arr_idx) {
			I("%s: Oversize!\n", __func__);
			goto END;
		}
		if (str[i] == c || str[i] == '\0' || str[i] == '\n') {
			I("%s: String parse compelete!\n", __func__);
			_arr_idx++;
			_arr_str_idx = 0;
		} else {
			arr[_arr_idx][_arr_str_idx] = str[i];
			_arr_str_idx++;
			continue;
		}
	}
END:
	return;
}

static int chk_slave_reg(char *str, int str_len)
{
	char *buf_write_slave_cmd = NULL;
	char *buf_read_slave_cmd = NULL;
	int write_cmd_len = -1;
	int read_cmd_len = -1;
	int chk;

	I("Entering: %s\n", __func__);

	I("cmd=%s\n", str);
	I("len=%d\n", str_len);
	write_cmd_len = (int)strcspn(str, ",");
	if (write_cmd_len == -1 || write_cmd_len >= str_len) {
		hx_s_ts->slave_write_reg = 0;
		I("%s: no other cmd\n", __func__);
		goto END;
	}

	buf_write_slave_cmd = kcalloc(write_cmd_len + 1,
		sizeof(char), GFP_KERNEL);
	memset(buf_write_slave_cmd, 0x00, write_cmd_len + 1);
	memcpy(&buf_write_slave_cmd[0], &str[0], write_cmd_len);
	str_len = str_len - write_cmd_len - 1;
	memcpy(&str[0], &str[write_cmd_len + 1], str_len);
	str[str_len] = '\0';
	chk = kstrtou8(buf_write_slave_cmd, 16, &hx_s_ts->slave_write_reg);
	if (chk) {
		E("%s, kstrtoint return fail1!\n", __func__);
		str_len = 0;
		goto END;
	}


	I("%s: str=%s, str_len=%d\n", __func__, str, str_len);
	I("%s: buf_write_slave_cmd=%s, write_cmd_len=%d, slave_write_reg=%d\n",
		__func__, buf_write_slave_cmd,
		write_cmd_len, hx_s_ts->slave_write_reg);

	I("cmd=%s, len=%d\n", str, str_len);
	read_cmd_len = (int)strcspn(str, ",");
	if (read_cmd_len == -1 || read_cmd_len >= str_len) {
		hx_s_ts->slave_read_reg = 0;
		I("%s: no other cmd\n", __func__);
		goto END;
	}
	buf_read_slave_cmd = kcalloc(read_cmd_len + 1,
		sizeof(char), GFP_KERNEL);
	memset(buf_read_slave_cmd, 0x00, read_cmd_len + 1);
	memcpy(&buf_read_slave_cmd[0], &str[0], read_cmd_len);
	str_len = str_len - read_cmd_len - 1;
	memcpy(&str[0], &str[read_cmd_len + 1], str_len);
	str[str_len] = '\0';
	chk = kstrtou8(buf_read_slave_cmd, 16, &hx_s_ts->slave_read_reg);
	if (chk) {
		E("%s, kstrtoint return fail2!\n", __func__);
		str_len = 0;
		goto END;
	}

	I("%s: str=%s, str_len=%d\n", __func__, str, str_len);
	I("%s: buf_read_slave_cmd=%s, read_cmd_len=%d, slave_read_reg=%d\n",
		__func__, buf_read_slave_cmd,
		read_cmd_len, hx_s_ts->slave_read_reg);

	hx_s_ts->select_slave_reg = true;
	I("%s : Now try to R/W slave address!\n", __func__);
END:
	if (buf_write_slave_cmd != NULL) {
		kfree(buf_write_slave_cmd);
		buf_write_slave_cmd = NULL;
	}
	if (buf_read_slave_cmd != NULL) {
		kfree(buf_read_slave_cmd);
		buf_read_slave_cmd = NULL;
	}
	return str_len;

}
static int himax_crc_test_read(struct seq_file *m)
{
	int ret = 0;
#if !defined(HX_ZERO_FLASH)
	uint8_t result = 0;
	uint32_t size = 0;

	I("%s: now flash = %d\n", __func__, hx_s_ic_data->flash_size);
	size = hx_s_ic_data->flash_size;

	hx_s_core_fp._sense_off(true);
	msleep(20);
	result = hx_s_core_fp._calculateChecksum(false, size);
	hx_s_core_fp._sense_on(0x01);

	if (result)
		seq_printf(m,
				"CRC test is Pass!\n");
	else
		seq_printf(m,
				"CRC test is Fail!\n");
#else
	seq_printf(m,
				"No flash, skip!\n");
#endif
	return ret;
}

static int himax_proc_FW_debug_read(struct seq_file *m)
{
	int ret = 0;
	uint8_t i = 0;
	uint8_t addr[4] = {0};
	uint8_t data[4] = {0};
	int len = 0;

	len = (size_t)(sizeof(hx_s_ic_data->dbg_reg_ary)/sizeof(uint32_t));

	for (i = 0; i < len; i++) {
		hx_parse_assign_cmd(hx_s_ic_data->dbg_reg_ary[i], addr, 4);
		hx_s_core_fp._register_read(hx_s_ic_data->dbg_reg_ary[i],
			data, DATA_LEN_4);

		seq_printf(m,
		"reg[0-3] : 0x%08X = 0x%02X, 0x%02X, 0x%02X, 0x%02X\n",
		hx_s_ic_data->dbg_reg_ary[i],
		data[0], data[1], data[2], data[3]);
		I("reg[0-3] : 0x%08X = 0x%02X, 0x%02X, 0x%02X, 0x%02X\n",
		hx_s_ic_data->dbg_reg_ary[i],
		data[0], data[1], data[2], data[3]);
	}

	return ret;
}

static int himax_attn_read(struct seq_file *m)
{
	int ret = 0;
	struct himax_ts_data *ts_data;

	ts_data = hx_s_ts;

	seq_printf(m, "attn = %x\n",
			himax_int_gpio_read(ts_data->pdata->gpio_irq));

	return ret;
}

static int himax_layout_read(struct seq_file *m)
{
	struct himax_ts_data *ts = hx_s_ts;
	size_t ret = 0;

	if (hx_s_debug_data->is_call_help) {
		seq_printf(m, HELP_LAYOUT);
		hx_s_debug_data->is_call_help = false;
	} else {
		seq_printf(m, "%d ",
				ts->pdata->abs_x_min);
		seq_printf(m, "%d ",
				ts->pdata->abs_x_max);
		seq_printf(m, "%d ",
				ts->pdata->abs_y_min);
		seq_printf(m, "%d ",
				ts->pdata->abs_y_max);
		 seq_puts(m, "\n");
	}

	return ret;
}

static ssize_t himax_layout_write(char *buf, size_t len)
{
	struct himax_ts_data *ts = hx_s_ts;
	char buf_tmp[5] = {0};
	int i = 0, j = 0, k = 0;
	unsigned long value;
	int layout[4] = {0};
	int chk;

	if (len >= 80) {
		I("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}

	if (strnstr(buf, "help", len) != NULL) {
		hx_s_debug_data->is_call_help = true;
		goto END;
	} else {

		for (i = 0; i < 20; i++) {
			if (buf[i] == ',' || buf[i] == '\n') {
				memset(buf_tmp, 0x0, sizeof(buf_tmp));

				if (i - j <= 5) {
					memcpy(buf_tmp, buf + j, i - j);
				} else {
					I("buffer size is over 5 char\n");
					return len;
				}

				j = i + 1;

				if (k < 4) {
					chk = kstrtoul(buf_tmp, 10, &value);
					if (chk < 0) {
						E("%s: string parse fail!\n",
							__func__);
						len = -EFAULT;
						goto END;
					}
					layout[k++] = value;
				}
			}
		}

		if (k == 4) {
			ts->pdata->abs_x_min = layout[0];
			ts->pdata->abs_x_max = (layout[1] - 1);
			ts->pdata->abs_y_min = layout[2];
			ts->pdata->abs_y_max = (layout[3] - 1);
			I("%d, %d, %d, %d\n",
			ts->pdata->abs_x_min, ts->pdata->abs_x_max,
			ts->pdata->abs_y_min, ts->pdata->abs_y_max);
			input_unregister_device(ts->input_dev);
			himax_input_register(ts);
		} else {
			I("ERR@%d, %d, %d, %d\n",
			ts->pdata->abs_x_min, ts->pdata->abs_x_max,
			ts->pdata->abs_y_min, ts->pdata->abs_y_max);
		}
	}
END:
	return len;
}

#if defined(HX_EXCP_RECOVERY)
static int himax_excp_cnt_read(struct seq_file *m)
{
	int ret = 0;

	I("%s: enter, %d\n", __func__, __LINE__);
	if (hx_s_debug_data->is_call_help) {
		seq_printf(m, HELP_EXCPT);
		hx_s_debug_data->is_call_help = false;
	} else {
		seq_printf(m,
			"Counts of EB=%d, EC=%d, ED=%d, EE=%d, all zero=%d\n",
			hx_s_ic_data->eb_event,
			hx_s_ic_data->ec_event,
			hx_s_ic_data->ed_event,
			hx_s_ic_data->ee_event,
			hx_s_ic_data->zero_event);
	}
	return ret;
}

static ssize_t himax_excp_cnt_write(char *buf, size_t len)
{
	// int i = 0;

	if (len >= 12) {
		I("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}
	if (strnstr(buf, "help", len) != NULL) {
		hx_s_debug_data->is_call_help = true;
	} else {
		I("Clear EXCEPTION Flag\n");

		if (buf[0] == '0') {
			I("reset count of events\n");
			hx_s_ic_data->eb_event = 0;
			hx_s_ic_data->ec_event = 0;
			hx_s_ic_data->ed_event = 0;
			hx_s_ic_data->ee_event = 0;
		}
	}
	return len;
}
#endif

static ssize_t himax_sense_on_off_write(char *buf, size_t len)
{
	if (len >= 80) {
		I("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}

	if (buf[0] == '0') {
		hx_s_core_fp._sense_off(true);
		I("Sense off\n");
	} else if (buf[0] == '1') {
		if (buf[1] == 's') {
			hx_s_core_fp._sense_on(0x00);
			I("Sense on re-map on, run sram\n");
		} else {
			hx_s_core_fp._sense_on(0x01);
			I("Sense on re-map off, run flash\n");
		}
	} else {
		I("Do nothing\n");
	}

	return len;
}

static int test_irq_pin(void)
{
	struct himax_ts_data *ts = hx_s_ts;
	int result = NO_ERR;
	int irq_sts = -1;
	uint8_t tmp_data[DATA_LEN_4] = {0};
	uint8_t tmp_read[DATA_LEN_4] = {0};

	hx_s_core_fp._sense_off(true);

	I("check IRQ LOW\n");
	usleep_range(20000, 20001);
	hx_parse_assign_cmd(0x00000002, tmp_data, DATA_LEN_4);
	hx_s_core_fp._register_write(0x90028060, tmp_data, DATA_LEN_4);
	usleep_range(20000, 20001);
	hx_s_core_fp._register_read(0x90028060, tmp_read, DATA_LEN_4);
	I("R%08XH = 0x%02X%02X%02X%02X\n",
		0x90028060,
		tmp_read[3], tmp_read[2], tmp_read[1], tmp_read[0]);

	usleep_range(20000, 20001);
	hx_parse_assign_cmd(0x00000001, tmp_data, DATA_LEN_4);
	hx_s_core_fp._register_write(0x90028064, tmp_data, DATA_LEN_4);
	usleep_range(20000, 20001);
	hx_s_core_fp._register_read(0x90028064, tmp_read, DATA_LEN_4);
	I("R%08XH = 0x%02X%02X%02X%02X\n",
		0x90028064,
		tmp_read[3], tmp_read[2], tmp_read[1], tmp_read[0]);

	usleep_range(20000, 20001);
	hx_parse_assign_cmd(0x00000000, tmp_data, DATA_LEN_4);
	hx_s_core_fp._register_write(0x90028068, tmp_data, DATA_LEN_4);
	usleep_range(20000, 20001);
	hx_s_core_fp._register_read(0x90028068, tmp_read, DATA_LEN_4);
	I("R%08XH = 0x%02X%02X%02X%02X\n",
		0x90028068,
		tmp_read[3], tmp_read[2], tmp_read[1], tmp_read[0]);

	usleep_range(20000, 20001);
	irq_sts = himax_int_gpio_read(ts->pdata->gpio_irq);
	if (irq_sts == 0) {
		I("[LOW]Now IRQ is LOW!\n");
		result += NO_ERR;
	} else {
		I("[LOW]Now IRQ is High!\n");
		result += 1;
	}

	I("check IRQ High\n");
	usleep_range(20000, 20001);
	hx_parse_assign_cmd(0x00000002, tmp_data, DATA_LEN_4);
	hx_s_core_fp._register_write(0x90028060, tmp_data, DATA_LEN_4);
	usleep_range(20000, 20001);
	hx_s_core_fp._register_read(0x90028060, tmp_read, DATA_LEN_4);
	I("R%08XH = 0x%02X%02X%02X%02X\n",
		0x90028060,
		tmp_read[3], tmp_read[2], tmp_read[1], tmp_read[0]);

	usleep_range(20000, 20001);
	hx_parse_assign_cmd(0x00000001, tmp_data, DATA_LEN_4);
	hx_s_core_fp._register_write(0x90028064, tmp_data, DATA_LEN_4);
	usleep_range(20000, 20001);
	hx_s_core_fp._register_read(0x90028064, tmp_read, DATA_LEN_4);
	I("R%08XH = 0x%02X%02X%02X%02X\n",
		0x90028064,
		tmp_read[3], tmp_read[2], tmp_read[1], tmp_read[0]);

	usleep_range(20000, 20001);
	hx_parse_assign_cmd(0x00000001, tmp_data, DATA_LEN_4);
	hx_s_core_fp._register_write(0x90028068, tmp_data, DATA_LEN_4);
	usleep_range(20000, 20001);
	hx_s_core_fp._register_read(0x90028068, tmp_read, DATA_LEN_4);
	I("R%08XH = 0x%02X%02X%02X%02X\n",
		0x90028068,
		tmp_read[3], tmp_read[2], tmp_read[1], tmp_read[0]);

	usleep_range(20000, 20001);
	irq_sts = himax_int_gpio_read(ts->pdata->gpio_irq);
	if (irq_sts == 0) {
		I("[High]Now IRQ is LOW!\n");
		result += 1;
	} else {
		I("[High]Now IRQ is High!\n");
		result += NO_ERR;
	}
	hx_s_debug_data->is_checking_irq = false;

	hx_s_core_fp._sense_on(0x00);

	return result;
}
static int himax_int_en_read(struct seq_file *m)
{
	struct himax_ts_data *ts = hx_s_ts;
	int ret = 0;


	if (hx_s_debug_data->is_checking_irq) {
		if (test_irq_pin() == NO_ERR) {
			seq_printf(m,
			"IRQ check OK!\n");
		} else {
			seq_printf(m,
			"IRQ check Fail!\n");
		}
	} else {
		seq_printf(m, "irq_status:%d\n",
		ts->irq_enabled);
	}
	return ret;
}

static ssize_t himax_int_en_write(char *buf, size_t len)
{
	struct himax_ts_data *ts = hx_s_ts;
	int ret;

	if (len >= 12) {
		I("%s: no command exceeds 12 chars.\n", __func__);
		return -EFAULT;
	}

	if (buf[0] == '0') {
		himax_int_enable(0);
	} else if (buf[0] == '1') {
		himax_int_enable(1);
	} else if (buf[0] == '2') {
		himax_int_enable(0);
		free_irq(ts->hx_irq, ts);
		ts->irq_enabled = 0;
	} else if (buf[0] == '3') {
		ret = himax_int_en_set();

		if (ret == 0) {
			ts->irq_enabled = 1;
			atomic_set(&ts->irq_state, 1);
		}
	} else if (strnstr(buf, "test", len) != NULL) {
		hx_s_debug_data->is_checking_irq = true;
		I("Checking IRQ start!\n");
	} else
		return -EINVAL;

	return len;
}

static int himax_irq_info_read(struct seq_file *m)
{
	// struct himax_ts_data *ts = hx_s_ts;
	int ret = 0;

	if (hx_s_core_fp._read_ic_trigger_type() == 1)
		seq_printf(m,
			"IC Interrupt type is edge trigger.\n");
	else if (hx_s_core_fp._read_ic_trigger_type() == 0)
		seq_printf(m,
			"IC Interrupt type is level trigger.\n");
	else
		seq_printf(m,
			"Unkown IC trigger type.\n");

	if (hx_s_ic_data->int_is_edge)
		seq_printf(m,
			"Driver register Interrupt : EDGE TIRGGER\n");
	else
		seq_printf(m,
			"Driver register Interrupt : LEVEL TRIGGER\n");

	return ret;
}

static ssize_t himax_irq_info_write(char *buf, size_t len)
{
	// struct himax_ts_data *ts = hx_s_ts;
	// int ret = 0;

	if (len >= 12) {
		I("%s: no command exceeds 12 chars.\n", __func__);
		return -EFAULT;
	}


	return len;
}

static int himax_debug_level_read(struct seq_file *m)
{
	struct himax_ts_data *ts_data;
	int ret = 0;

	ts_data = hx_s_ts;
	seq_printf(m, "tsdbg: %d\n",
			g_ts_dbg);
	seq_printf(m, "level: %X\n",
			ts_data->debug_log_level);


	return ret;
}

static ssize_t himax_debug_level_write(char *buf, size_t len)
{
	struct himax_ts_data *ts;
	int i;

	ts = hx_s_ts;

	if (len >= 12) {
		I("%s: no command exceeds 12 chars.\n", __func__);
		return -EFAULT;
	}

	if (strnstr(buf, "tsdbg", len) != NULL) {
		if (buf[5] == '1') {
			I("Open Ts Debug!\n");
			g_ts_dbg = 1;
		} else if (buf[5] == '0') {
			I("Close Ts Debug!\n");
			g_ts_dbg = 0;
		} else {
			E("Parameter fault for ts debug\n");
		}
	} else {

		ts->debug_log_level = 0;

		for (i = 0; i < len; i++) {
			if (buf[i] >= '0' && buf[i] <= '9')
				ts->debug_log_level |= (buf[i] - '0');
			else if (buf[i] >= 'A' && buf[i] <= 'F')
				ts->debug_log_level |= (buf[i] - 'A' + 10);
			else if (buf[i] >= 'a' && buf[i] <= 'f')
				ts->debug_log_level |= (buf[i] - 'a' + 10);

			if (i != len - 1)
				ts->debug_log_level <<= 4;
		}
		I("Now debug level value=%d\n", ts->debug_log_level);

		if (ts->debug_log_level & BIT(4)) {
			I("Turn on/Enable Debug Mode for Inspection!\n");
			goto END_FUNC;
		}

		if (ts->debug_log_level & BIT(5)) {
			I("Turn on/Enable Debug Mode for Zero flash update!\n");
			goto END_FUNC;
		}

		if (ts->debug_log_level & BIT(3)) {
			if (ts->pdata->screenWidth > 0
			&& ts->pdata->screenHeight > 0
			&& (ts->pdata->abs_x_max - ts->pdata->abs_x_min) > 0
			&& (ts->pdata->abs_y_max - ts->pdata->abs_y_min) > 0) {
				ts->widthFactor =
					(ts->pdata->screenWidth << SHIFTBITS)
					/ (ts->pdata->abs_x_max
					- ts->pdata->abs_x_min);
				ts->heightFactor =
					(ts->pdata->screenHeight << SHIFTBITS)
					/ (ts->pdata->abs_y_max
					- ts->pdata->abs_y_min);

				if (ts->widthFactor > 0 &&
					ts->heightFactor > 0) {
					ts->useScreenRes = 1;
				} else {
					ts->heightFactor = 0;
					ts->widthFactor = 0;
					ts->useScreenRes = 0;
				}
			} else
				I("En-finger-dbg with raw position mode!\n");
		} else {
			ts->useScreenRes = 0;
			ts->widthFactor = 0;
			ts->heightFactor = 0;
		}
	}
END_FUNC:
	return len;
}

static int himax_proc_register_read(struct seq_file *m)
{
	int ret = 0;
	uint16_t i;
	uint32_t addr32 = 0x00;
	uint8_t test_data[DATA_LEN_4] = {0};

	if (hx_s_debug_data->is_call_help) {
		seq_printf(m, HELP_REGISTER);
		hx_s_debug_data->is_call_help = false;
	} else {
		if (!hx_s_ts->acc_slave_reg
			&& hx_s_ts->select_slave_reg) {
			I("First read Slave, test it !\n");
			ret = himax_bus_read(0x13, test_data, 1);
			if (ret < 0) {
				E("%s: bus access fail!\n", __func__);
				seq_puts(m, "Reading Slave fail, maybe no support!\n");
				ret = BUS_FAIL;
				goto bus_fail_end;
			}
			if (test_data[0] == 0x00) {
				seq_puts(m, "Reading HW Slave fail, maybe no support!\n");
				ret = BUS_FAIL;
				goto bus_fail_end;
			}
			hx_s_ts->acc_slave_reg = true;

		}
		if (hx_s_ts->select_slave_reg)
			seq_printf(m, "[Slave W:0x%02X, R:0x%02X]\n",
				hx_s_ts->slave_write_reg,
				hx_s_ts->slave_read_reg);

		memset(proc_reg_buf,
			0x00,
			128 * sizeof(uint8_t));

		I("himax_register_show: %02X,%02X,%02X,%02X\n",
			proc_reg_addr[3], proc_reg_addr[2], proc_reg_addr[1],
			proc_reg_addr[0]);

		if (proc_reg_addr_type == 1) {
			ret = himax_bus_read(proc_reg_addr[0], proc_reg_buf,
				128);
			if (ret < 0) {
				E("%s: bus access fail!\n", __func__);
				return BUS_FAIL;
			}
		} else {
			addr32 = proc_reg_addr[3] << 24 |
					proc_reg_addr[2] << 16 |
					proc_reg_addr[1] << 8 |
					proc_reg_addr[0];
			hx_s_core_fp._register_read(addr32, proc_reg_buf,
				128);
		}

		seq_printf(m, "command:  %02X,%02X,%02X,%02X\n",
			proc_reg_addr[3], proc_reg_addr[2], proc_reg_addr[1],
			proc_reg_addr[0]);

		for (i = 0; i < 128; i++) {
			seq_printf(m, "0x%2.2X ", proc_reg_buf[i]);
			if ((i % 16) == 15)
				seq_puts(m, "\n");

		}

	}
bus_fail_end:
	seq_puts(m, "\n");
	if (hx_s_ts->select_slave_reg)
		hx_s_ts->select_slave_reg = false;
	return ret;
}

static ssize_t himax_proc_register_write(char *buf, size_t len)
{
	char *buff_tmp;
	int length = 0;
	int byte_length = 0;
	unsigned long result = 0;
	int i = 0;
	char *data_str = NULL;
	uint8_t *w_data;
	int x_pos[32] = {0};
	int count = 0;
	uint32_t addr32;
	int ret = 0;
	int len_reg;
	int idx = 0;

	if (len >= 4096) {
		I("%s: no command exceeds 4096 chars.\n", __func__);
		return -EFAULT;
	}


	if (strnstr(buf, "help", len) != NULL) {
		hx_s_debug_data->is_call_help = true;
		goto END;
	}

	buff_tmp = kzalloc(sizeof(char) * 4096, GFP_KERNEL);
	w_data = kzalloc(sizeof(uint8_t) * 4096, GFP_KERNEL);

	len = chk_slave_reg(buf, len);
	if (len == 0) {
		E("%s: Parse cmd fail:%s", __func__, buf);
		goto PARSE_FAIL_END;
	}

	memset(proc_reg_addr, 0x0, sizeof(proc_reg_addr));

	if ((buf[0] == 'r' || buf[0] == 'w')
	&& buf[1] == ':' && buf[2] == 'x') {
		length = len;

		I("%s: length = %d.\n", __func__, length);
		for (i = 0; i < length; i++) {
			/* find postion of 'x' */
			if (buf[i] == 'x') {
				x_pos[count] = i;
				count++;
			}
		}

		data_str = strrchr(buf, 'x');
		I("%s: %s.\n", __func__, data_str);
		if (x_pos[1] > 0
			&&  len > (x_pos[1] + 1))
			length = len - x_pos[1] - 1;//strlen(data_str + 1);
		else
			length = strlen(data_str + 1);
		I("para length = %d\n", length);

		switch (buf[0]) {
		case 'r':
		if (buf[3] == 'F' && buf[4] == 'E' && length == 4) {
			len_reg = 2; // Now len of reg is 1 byte(char size = 2)
			length = length - 2;
			proc_reg_addr_type = 1;
			memcpy(buff_tmp, data_str + 3, length);
		} else {
			len_reg = 8; // Now len of reg is 4 byte(char size = 8)
			proc_reg_addr_type = 0;
			memcpy(buff_tmp, data_str + 1, length);
		}
		byte_length = length / 2;
		if (!kstrtoul(buff_tmp, 16, &result)) {
			for (i = 0; i < byte_length; i++)
				proc_reg_addr[i] = (uint8_t)(result >> (i * 8));
		}

		break;

		case 'w':
		if (buf[3] == 'F' && buf[4] == 'E') {
			len_reg = 2;
			proc_reg_addr_type = 1;
			memcpy(buff_tmp, buf + 5, len_reg);
		} else {
			len_reg = 8;
			proc_reg_addr_type = 0;
			memcpy(buff_tmp, buf + 3, len_reg);
		}

		if (count < 3) {
			byte_length = length;
			I("%s: 1byte_length=%d\n", __func__, byte_length);
			if (!kstrtoul(buff_tmp, 16, &result)) {
				/* command */
				for (i = 0; i < len_reg / 2; i++) {
					proc_reg_addr[i] =
						(uint8_t)(result >> (i * 8));
					// I("%s: proc_reg_addr[%d]=0x%02X\n",
						// __func__,
						// i,
						// proc_reg_addr[i]);
				}
			}

			for (i = 0; i < byte_length / 2; i++) {
				idx = (byte_length - (i+1) * 2) / 2;
				memcpy(&buff_tmp[0], data_str + 1 + i * 2, 2);
				buff_tmp[2] = '\0';
				if (!kstrtoul(buff_tmp, 16, &result))
					w_data[idx] = (uint8_t)(result);
				// I("%s: w_data[%d]=0x%02X\n",
					// __func__,
					// idx, w_data[idx]);
			}
		} else {
			for (i = 0; i < count; i++) {
				/* parsing addr after 'x' */
				memset(buff_tmp, 0x0, 4096);
				if (proc_reg_addr_type != 0 && i != 0)
					byte_length = 2;
				else
					byte_length = length;
					/* original */
				I("%s: 2byte_length=%d\n",
					__func__,
					byte_length);
				memcpy(buff_tmp, buf+x_pos[i]+1, byte_length);

				if (!kstrtoul(buff_tmp, 16, &result)) {
					if (i == 0)
						proc_reg_addr[i] =
							(uint8_t)(result);
					else
						w_data[i - 1] =
							(uint8_t)(result);
				}
			}

			byte_length = count - 1;
		}

		if (proc_reg_addr_type == 1) {
			ret = himax_bus_write(proc_reg_addr[0],
				NUM_NULL,
				w_data,
				byte_length);
			if (ret < 0) {
				E("%s: bus access fail!\n", __func__);
				return BUS_FAIL;
			}
		} else {
			addr32 = proc_reg_addr[3] << 24 |
					proc_reg_addr[2] << 16 |
					proc_reg_addr[1] << 8 |
					proc_reg_addr[0];
			hx_s_core_fp._register_write(addr32, w_data,
				byte_length / 2);
		}

		break;

		default:
			E("%s:cmd fail\n", __func__);
			break;
		};
	}
PARSE_FAIL_END:
	kfree(buff_tmp);
	kfree(w_data);
END:
	return len;
}

uint8_t *getMutualBufferByte(void)
{
	return diag_mutual_byte;
}

int32_t *getMutualBuffer(void)
{
	return diag_mutual;
}
int32_t *getMutualNewBuffer(void)
{
	return diag_mutual_new;
}
int32_t *getMutualOldBuffer(void)
{
	return diag_mutual_old;
}

uint8_t *getSelfBufferByte(void)
{
	return diag_self_byte;
}
int32_t *getSelfBuffer(void)
{
	return diag_self;
}
int32_t *getSelfNewBuffer(void)
{
	return diag_self_new;
}
int32_t *getSelfOldBuffer(void)
{
	return diag_self_old;
}

void setMutualBufferByte(uint8_t x_num, uint8_t y_num)
{
	diag_mutual_byte = kzalloc(x_num * y_num * 2*
		sizeof(uint8_t), GFP_KERNEL);
}

void setMutualBuffer(uint8_t x_num, uint8_t y_num)
{
	diag_mutual = kzalloc(x_num * y_num * sizeof(int32_t), GFP_KERNEL);
}
void setMutualNewBuffer(uint8_t x_num, uint8_t y_num)
{
	diag_mutual_new = kzalloc(x_num * y_num * sizeof(int32_t), GFP_KERNEL);
}
void setMutualOldBuffer(uint8_t x_num, uint8_t y_num)
{
	diag_mutual_old = kzalloc(x_num * y_num * sizeof(int32_t), GFP_KERNEL);
}
void setSelfBufferByte(uint8_t x_num, uint8_t y_num)
{
	diag_self_byte = kzalloc((x_num + y_num) * 2*
		sizeof(uint8_t), GFP_KERNEL);
}
void setSelfBuffer(uint8_t x_num, uint8_t y_num)
{
	diag_self = kzalloc((x_num + y_num) * sizeof(int32_t), GFP_KERNEL);
}
void setSelfNewBuffer(uint8_t x_num, uint8_t y_num)
{
	diag_self_new = kzalloc((x_num + y_num) * sizeof(int32_t), GFP_KERNEL);
}
void setSelfOldBuffer(uint8_t x_num, uint8_t y_num)
{
	diag_self_old = kzalloc((x_num + y_num) * sizeof(int32_t), GFP_KERNEL);
}

#if defined(HX_TP_PROC_2T2R)
int32_t *getMutualBuffer_2(void)
{
	return diag_mutual_2;
}
void setMutualBuffer_2(uint8_t x_num_2, uint8_t y_num_2)
{
	diag_mutual_2 =
	  kzalloc(x_num_2 * y_num_2 * sizeof(int32_t), GFP_KERNEL);
}
#endif

int himax_set_diag_cmd(struct himax_ic_data *hx_s_ic_data,
		struct himax_report_data *hx_s_touch_data)
{
	struct himax_ts_data *ts = hx_s_ts;
	int32_t *mutual_data;
	int32_t *self_data;
	uint8_t *mutual_data_byte;
	uint8_t *self_data_byte;
	int mul_num;
	int self_num;
	/* int RawDataLen = 0; */
	hx_s_touch_data->diag_cmd = ts->diag_cmd;

	if (hx_s_touch_data->diag_cmd >= 1 && hx_s_touch_data->diag_cmd <= 7) {
		/* Check event stack CRC */
		if (!hx_s_core_fp._diag_check_sum(hx_s_touch_data))
			goto bypass_checksum_failed_packet;

#if defined(HX_TP_PROC_2T2R)
		if (Is_2T2R && (hx_s_touch_data->diag_cmd >= 4 &&
			hx_s_touch_data->diag_cmd <= 6)) {
			mutual_data = getMutualBuffer_2();
			self_data = getSelfBuffer();
			mutual_data_byte = getMutualBufferByte();
			self_data_byte = getSelfBufferByte();
			/* initiallize the block number of mutual and self */
			mul_num = hx_s_ic_data->rx_num_2 *
				hx_s_ic_data->tx_num_2;
			self_num = hx_s_ic_data->rx_num_2 +
				hx_s_ic_data->tx_num_2;
		} else
#endif
		{
			mutual_data = getMutualBuffer();
			self_data = getSelfBuffer();
			mutual_data_byte = getMutualBufferByte();
			self_data_byte = getSelfBufferByte();
			/* initiallize the block number of mutual and self */
			mul_num = hx_s_ic_data->rx_num * hx_s_ic_data->tx_num;
			self_num = hx_s_ic_data->rx_num + hx_s_ic_data->tx_num;
		}
		hx_s_core_fp._diag_parse_raw_data(hx_s_touch_data, mul_num,
			self_num, hx_s_touch_data->diag_cmd, mutual_data,
			self_data, mutual_data_byte, self_data_byte);
	} else if (hx_s_touch_data->diag_cmd == 8) {
		memset(diag_coor, 0x00, sizeof(diag_coor));
		memcpy(&(diag_coor[0]), &hx_s_touch_data->coord_buf[0],
		  hx_s_touch_data->touch_info_size);
	}

	/* assign state info data */
	memcpy(&(state_info[0]), &hx_s_touch_data->state_info[0], 2);
	return NO_ERR;
bypass_checksum_failed_packet:
	return 1;
}

int hx_dbg_raw_full_stack(struct himax_ic_data *hx_s_ic_data,
		struct himax_report_data *hx_s_touch_data)
{
	struct himax_ts_data *ts = hx_s_ts;
	int32_t *mutual_data;
	int32_t *self_data;
	int x_channel = hx_s_ic_data->rx_num;
	int y_channel = hx_s_ic_data->tx_num;
	int mul_num;
	int self_num;
	int update_idx;
	int data_status = 0;
	uint8_t *mutual_data_byte;
	uint8_t *self_data_byte;

	hx_s_touch_data->diag_cmd = ts->diag_cmd;
	mutual_data = getMutualBuffer();
	self_data = getSelfBuffer();
	mutual_data_byte = getMutualBufferByte();
	self_data_byte = getSelfBufferByte();
	/* initiallize the block number of mutual and self */
	mul_num = hx_s_ic_data->rx_num * hx_s_ic_data->tx_num;
	self_num = hx_s_ic_data->rx_num + hx_s_ic_data->tx_num;

	data_status = hx_s_core_fp._diag_parse_raw_data(hx_s_touch_data,
		mul_num,
		self_num, hx_s_touch_data->diag_cmd, mutual_data,
		self_data, mutual_data_byte, self_data_byte);
	if (data_status != NO_ERR)
		return data_status;
	update_idx = atomic_read(&(g_rb_frame.p_update));
	// memcpy(&g_rb_frame.rawdata[update_idx].self[0],
	//	&self_data[0], self_num * sizeof(int32_t));
	// memcpy(&g_rb_frame.rawdata[update_idx].mutual[0],
	//	&mutual_data[0], mul_num * sizeof(int32_t));
	memcpy(&g_rb_frame.rawdata[update_idx].self[0],
		&hx_s_touch_data->rawdata_buf[4 + 2 * (x_channel * y_channel)],
		2 * (x_channel + y_channel) * sizeof(uint8_t));
	memcpy(&g_rb_frame.rawdata[update_idx].mutual[0],
		&hx_s_touch_data->rawdata_buf[4],
		2 * (x_channel * y_channel) * sizeof(uint8_t));
	g_rb_frame.frame_idx[update_idx] = g_rb_frame.rawdata->index++;

	if (atomic_read(&(g_rb_frame.p_update)) == (HX_RB_FRAME_SIZE - 1))
		atomic_set(&(g_rb_frame.p_update), 0);
	else {
		atomic_set(&(g_rb_frame.p_update),
			atomic_read(&(g_rb_frame.p_update)) + 1);
	}
	g_rb_frame.rawdata->cnt_update++;
	if (ts->debug_log_level & BIT(5))
		I("Updated: rb_idx=%d, value_idx=%d\n",
			update_idx, g_rb_frame.frame_idx[update_idx]);

	return NO_ERR;

}


/* #if defined(HX_DEBUG_LEVEL) */
#define HX_DL_COORD_FORMAT \
	"P%3d = 0x%2.2X,P%3d = 0x%2.2X,P%3d = 0x%2.2X,P%3d = 0x%2.2X\n"
void himax_log_touch_data(struct himax_ts_data *ts, int start)
{
	int i = 0;
	int print_size = 128;

	if (start == 1)
		return; /* report data when end of ts_work*/

	if (print_size > hx_s_touch_data->touch_all_size) {
		E("%s: print size is too large!\n", __func__);
		return;
	}

	for (i = 0; i < print_size; i += 8) {
		if ((i + 7) >= print_size) {
			I(HX_DL_COORD_FORMAT,
				i,
				ts->xfer_buff[i],
				i + 1,
				ts->xfer_buff[i + 1],
				i + 2,
				ts->xfer_buff[i + 2],
				i + 3,
				ts->xfer_buff[i + 3]);
			break;
		}
		I(HX_DL_COORD_FORMAT,
		  i, ts->xfer_buff[i], i + 1, ts->xfer_buff[i + 1],
		  i + 2, ts->xfer_buff[i + 2], i + 3, ts->xfer_buff[i + 3]);
		I(HX_DL_COORD_FORMAT,
		  i + 4, ts->xfer_buff[i + 4], i + 5, ts->xfer_buff[i + 5],
		  i + 6, ts->xfer_buff[i + 6], i + 7, ts->xfer_buff[i + 7]);
	}
}

#define HX_PEN_LV2_LOG(x) "Stylus[" \
	x \
	"] X:%d,Y:%d,W:%d,btn1:%d,btn2:%d,tilt_x:%d,tilt_y:%d\n"
void himax_log_touch_event(struct himax_ts_data *ts, int start)
{
	int i;
	bool stylus_valid;

	if (start == 1)
		return;   /*report data when end of ts_work*/

	/* finger log info */
	if (g_target_report_data->finger_on > 0 &&
		g_target_report_data->finger_num > 0) {
		for (i = 0; i < ts->nFinger_support; i++) {
			if (g_target_report_data->p[i].x >= 0
			&& g_target_report_data->p[i].x
			<= ts->pdata->abs_x_max
			&& g_target_report_data->p[i].y >= 0
			&& g_target_report_data->p[i].y
			<= ts->pdata->abs_y_max) {
				I(PRT_LOG,
					i + 1,
					g_target_report_data->p[i].x,
					g_target_report_data->p[i].y,
					g_target_report_data->p[i].w,
					g_target_report_data->p[i].w,
					i + 1,
					g_target_report_data->ig_count);
				}
		}
	} else if (g_target_report_data->finger_on == 0
	&& g_target_report_data->finger_num == 0) {
		I("All Finger leave\n");
	} else {
		I("%s : Finger wrong input!\n", __func__);
	}

	/* stylus log info */
	if (hx_s_ic_data->stylus_func) {
		if (g_target_report_data->s[0].x >= 0
		&& g_target_report_data->s[0].x <=
			((ts->pdata->abs_x_max+1)*hx_s_ic_data->stylus_ratio-1)
		&& g_target_report_data->s[0].y >= 0
		&& g_target_report_data->s[0].y <=
			((ts->pdata->abs_y_max+1)*hx_s_ic_data->stylus_ratio-1)
		&& (g_target_report_data->s[0].on == 1))
			stylus_valid = true;
		else
			stylus_valid = false;

		if (stylus_valid) {/*stylus down*/
			if (g_target_report_data->s[0].hover)
				I(HX_PEN_LV2_LOG("Hover"),
					g_target_report_data->s[0].x,
					g_target_report_data->s[0].y,
					g_target_report_data->s[0].w,
					g_target_report_data->s[0].btn,
					g_target_report_data->s[0].btn2,
					g_target_report_data->s[0].tilt_x,
					g_target_report_data->s[0].tilt_y);
			else
				I(HX_PEN_LV2_LOG("Contact"),
					g_target_report_data->s[0].x,
					g_target_report_data->s[0].y,
					g_target_report_data->s[0].w,
					g_target_report_data->s[0].btn,
					g_target_report_data->s[0].btn2,
					g_target_report_data->s[0].tilt_x,
					g_target_report_data->s[0].tilt_y);
		}
	}
}
void himax_log_touch_int_devation(int touched)
{
	if (touched == HX_FINGER_ON) {
		time_func(&timeStart);
		/* I(" Irq start time = %ld.%06ld s\n",
		 * timeStart.tv_sec, timeStart.tv_nsec/1000);
		 */
	} else if (touched == HX_FINGER_LEAVE) {
		time_func(&timeEnd);
		timeDelta.tv_nsec =
		  (timeEnd.tv_sec * 1000000000 + timeEnd.tv_nsec) -
		  (timeStart.tv_sec * 1000000000 + timeStart.tv_nsec);
		/*  I("Irq finish time = %ld.%06ld s\n",
		 *	timeEnd.tv_sec, timeEnd.tv_nsec/1000);
		 */
		I("Touch latency = %ld us\n", timeDelta.tv_nsec / 1000);
		I("bus_speed = %d kHz\n", hx_s_ts->bus_speed);
		if (g_target_report_data->finger_on == 0
		&& g_target_report_data->finger_num == 0)
			I("All Finger leave\n");
	} else {
		I("%s : wrong input!\n", __func__);
	}
}

void himax_log_touch_event_detail(struct himax_ts_data *ts, int start)
{
	int i;

	if (start == HX_FINGER_LEAVE) {
		for (i = 0; i < ts->nFinger_support; i++) {
			if (((ts->old_finger >> i & 1) == 0)
			&& ((ts->pre_finger_mask >> i & 1) == 1)) {
				if (g_target_report_data->p[i].x >= 0
				&& g_target_report_data->p[i].x
				<= ts->pdata->abs_x_max
				&& g_target_report_data->p[i].y >= 0
				&& g_target_report_data->p[i].y
				<= ts->pdata->abs_y_max) {
					I(RAW_DOWN_STATUS, i + 1,
					g_target_report_data->p[i].x,
					g_target_report_data->p[i].y,
					g_target_report_data->p[i].w);
				}
			} else if ((((ts->old_finger >> i & 1) == 1)
			&& ((ts->pre_finger_mask >> i & 1) == 0))) {
				I(RAW_UP_STATUS, i + 1,
					ts->pre_finger_data[i][0],
					ts->pre_finger_data[i][1]);
			} else {
				/* I("dbg hx_point_num=%d, old_finger=0x%02X,"
				 * " pre_finger_mask=0x%02X\n",
				 * ts->hx_point_num, ts->old_finger,
				 * ts->pre_finger_mask);
				 */
			}
		}
	}
}

void himax_ts_dbg_func(struct himax_ts_data *ts, int start)
{
	if (ts->debug_log_level & BIT(0)) {
		/* I("debug level 1\n"); */
		himax_log_touch_data(ts, start);
	}
	if (ts->debug_log_level & BIT(1)) {
		/* I("debug level 2\n"); */
		himax_log_touch_event(ts, start);
	}
	if (ts->debug_log_level & BIT(2)) {
		/* I("debug level 4\n"); */
		himax_log_touch_int_devation(start);
	}
	if (ts->debug_log_level & BIT(3)) {
		/* I("debug level 8\n"); */
		himax_log_touch_event_detail(ts, start);
	}
}

static int himax_change_mode(uint8_t str_pw, uint8_t end_pw)
{
	uint8_t data[4] = {0};
	int count = 0;

	/*sense off*/
	hx_s_core_fp._sense_off(true);
	/*mode change*/
	data[1] = str_pw; data[0] = str_pw;
	if (hx_s_core_fp._assign_sorting_mode != NULL)
		hx_s_core_fp._assign_sorting_mode(data);

	/*sense on*/
	hx_s_core_fp._sense_on(1);
	/*wait mode change*/
	do {
		if (hx_s_core_fp._check_sorting_mode != NULL)
			hx_s_core_fp._check_sorting_mode(data);
		if ((data[0] == end_pw) && (data[1] == end_pw))
			return 0;

		I("Now retry %d times!\n", count);
		count++;
		msleep(50);
	} while (count < 50);

	return ERR_WORK_OUT;
}

static ssize_t himax_irq_dbg_cmd_write(char *buf, size_t len)
{
	// struct himax_ts_data *ts = hx_s_ts;
	int cmd = 0;

	if (!kstrtoint(buf, 16, &cmd)) {
		I("%s, now irq_dbg status=%d!\n", __func__, cmd);
		g_ts_dbg = cmd;
	} else {
		E("%s: command not int!\n", __func__);
	}
	return len;
}

static ssize_t himax_diag_cmd_write(char *buf, size_t len)
{
	struct himax_ts_data *ts = hx_s_ts;
	char *dbg_map_str = "mode:";
	char *str_ptr = NULL;
	int str_len = 0;
	int rst = 0;
	uint8_t str_pw = 0;
	uint8_t end_pw = 0;
	int frame_indx;

	if (strnstr(buf, "help", len) != NULL) {
		hx_s_debug_data->is_call_help = true;
		goto END;
	}

	if (buf[0] == 'r' && buf[1] == 'b'
		&& hx_s_debug_data->is_stack_full_raw == true){
		if (len > 2) {
			strlcpy(&buf[0], &buf[2], len);
			len -= 2;
			if (!kstrtoint(buf, 10, &rst)) {
				frame_indx = rst;
				if (frame_indx >= HX_RB_FRAME_SIZE)
					E("Bigger than rb max size, %d\n",
						frame_indx);
				else {
					I("Now set to idx %d\n", frame_indx);
					g_re_ass_rb_idx = frame_indx;
				}
			}
			goto END;
		} else {
			E("%s:Fail command!\n", __func__);
			goto END;
		}
	}
	switch (len) {
	case 1:/*raw out select - diag,X*/
		if (!kstrtoint(buf, 16, &rst)) {
			process_type = (rst >> 4) & 0xF;
			ts->diag_cmd = rst & 0xF;
			I("%s: dsram_flag = %d\n", __func__, dsram_flag);
			if (dsram_flag) {
				/*Cancal work queue and return to stack*/
				process_type = 0;
				while (dsram_flag == true)
					usleep_range(10000, 11000);
				cancel_delayed_work(&ts->himax_diag_delay_work);
				himax_int_enable(1);
				hx_s_core_fp._return_event_stack();
			}

			hx_s_core_fp._diag_register_set(ts->diag_cmd,
					process_type, false);

			if (ts->diag_cmd == 0) {
				hx_s_touch_data->touch_all_size =
					hx_s_touch_data->touch_all_size_normal;
				hx_s_touch_data->rawdata_size =
					hx_s_touch_data->rawdata_size_normal;
				hx_s_debug_data->is_stack_full_raw = false;
				hx_s_debug_data->is_stack_output_bin = false;
			}

			I("%s: Set raw out select 0x%X.\n",
					__func__, ts->diag_cmd);
		}
		if (!ts->diag_cmd) {
			if (mode_flag) /*back to normal mode*/
				himax_change_mode(0x00, 0x99);
		}
		break;
	case 2:/*data processing + rawout select - diag,XY*/
		if (!kstrtoint(buf, 16, &rst)) {
			process_type = (rst >> 4) & 0xF;
			ts->diag_cmd = rst & 0xF;
		}
		if (ts->diag_cmd == 0)
			break;
		else if (process_type > 0 && process_type <= 3) {
			if (!dsram_flag) {
				/*Start wrok queue*/
				himax_int_enable(0);
				hx_s_core_fp._diag_register_set(ts->diag_cmd,
					process_type, false);

				queue_delayed_work(ts->himax_diag_wq,
				&ts->himax_diag_delay_work, 2 * HZ / 100);
				dsram_flag = true;

				I("%s: Start get raw data in DSRAM\n",
					__func__);
			} else {
				hx_s_core_fp._diag_register_set(ts->diag_cmd,
					process_type, false);
			}
		}
		break;
	case 4:/*data processing + rawout select - diag,XXYY*/
		/*ex:XXYY=010A=dsram rawdata*/
		I("%s, now case 4\n", __func__);
		if (!kstrtoint(buf, 16, &rst)) {
			process_type = (rst >> 8) & 0xFF;
			ts->diag_cmd = rst & 0xFF;
			I("%s:process_type=0x%02X, diag_cmd=0x%02X\n",
			__func__, process_type, ts->diag_cmd);
		}
		if (process_type == 0 || ts->diag_cmd == 0)
			break;
		else if ((process_type > 0) && (process_type <= 3)) {
			if (!dsram_flag) {
				/*Start wrok queue*/
				himax_int_enable(0);
				hx_s_core_fp._diag_register_set(ts->diag_cmd,
					process_type, true);

				queue_delayed_work(ts->himax_diag_wq,
				&ts->himax_diag_delay_work, 2 * HZ / 100);
				dsram_flag = true;

				I("%s: Start get raw data in DSRAM\n",
					__func__);
			} else {
				hx_s_core_fp._diag_register_set(ts->diag_cmd,
					process_type, true);
			}
		} else if (process_type == 0x0f) {
			hx_s_touch_data->touch_all_size =
				hx_s_touch_data->touch_all_size_full_stack;
			hx_s_touch_data->rawdata_size =
				hx_s_touch_data->rawdata_size_full_stack;
			hx_dbg_rst_rb();
			hx_s_debug_data->is_stack_full_raw = true;
			hx_s_core_fp._diag_register_set(ts->diag_cmd,
					0, true);
		} else if (process_type == 0x0e) {
			hx_s_debug_data->is_stack_output_bin = true;

			if (ts->diag_cmd <= 3 &&
				ts->diag_cmd >= 1) {
				hx_s_core_fp._diag_register_set(ts->diag_cmd,
					0, true);
			} else {
				process_type = 1;
				if (!dsram_flag) {
					/*Start wrok queue*/
					himax_int_enable(0);
					hx_s_core_fp._diag_register_set(
						ts->diag_cmd,
						0, true);

					queue_delayed_work(ts->himax_diag_wq,
					&ts->himax_diag_delay_work,
					2 * HZ / 100);
					dsram_flag = true;

					I("%s: Start get raw data in DSRAM\n",
						__func__);
				} else {
					hx_s_core_fp._diag_register_set(
						ts->diag_cmd,
						0, true);
				}
			}
		}
		break;
	case 9:/*change mode - mode:XXYY(start PW,end PW)*/
		str_ptr = strnstr(buf, dbg_map_str, len);
		if (str_ptr) {
			str_len = strlen(dbg_map_str);
			if (!kstrtoint(buf + str_len, 16, &rst)) {
				str_pw = (rst >> 8) & 0xFF;
				end_pw = rst & 0xFF;
				if (!himax_change_mode(str_pw, end_pw)) {
					mode_flag = 1;
					I(PRT_OK_LOG, __func__,
					rst, str_pw, end_pw);
				} else
					I(PRT_FAIL_LOG,	__func__,
					str_pw, end_pw);
			}
		} else {
			I("%s: Can't find string [%s].\n",
					__func__, dbg_map_str);
		}
		break;
	default:
		hx_s_debug_data->is_stack_full_raw = false;
		I("%s: Length is not correct.\n", __func__);
		break;
	}
END:
	return len;
}

static int himax_diag_arrange_read(struct seq_file *m)
{
	int ret = NO_ERR;

	if (hx_s_debug_data->is_call_help) {
		seq_printf(m, HELP_DIAG_ARR);
		hx_s_debug_data->is_call_help = false;
	} else {
		seq_printf(m, "diag value=%d\n", g_diag_arr_num);
	}
	return ret;
}

static ssize_t himax_diag_arrange_write(char *buf, size_t len)
{
	if (len >= 80) {
		I("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}

	if (strnstr(buf, "help", len) != NULL) {
		hx_s_debug_data->is_call_help = true;
	} else {
		g_diag_arr_num = buf[0] - '0';
		I("%s: g_diag_arr_num = %d\n", __func__, g_diag_arr_num);
	}
	return len;
}

void himax_get_mutual_edge(void)
{
	int i = 0;

	for (i = 0; i < (hx_s_ic_data->rx_num * hx_s_ic_data->tx_num); i++) {
		if (diag_mutual[i] > g_max_mutual)
			g_max_mutual = diag_mutual[i];

		if (diag_mutual[i] < g_min_mutual)
			g_min_mutual = diag_mutual[i];
	}
}

void himax_get_self_edge(void)
{
	int i = 0;

	for (i = 0; i < (hx_s_ic_data->rx_num + hx_s_ic_data->tx_num); i++) {
		if (diag_self[i] > g_max_self)
			g_max_self = diag_self[i];

		if (diag_self[i] < g_min_self)
			g_min_self = diag_self[i];
	}
}

static void print_state_info(struct seq_file *s)
{
	/* seq_printf(s, "State_info_2bytes:%3d, %3d\n",
	 * _state_info[0],state_info[1]);
	 */

#if defined(HX_NEW_EVENT_STACK_FORMAT)
	seq_printf(s, "ReCal = %d\t", state_info[0] & 0x03);
	seq_printf(s, "Base Line = %d\t", state_info[0] >> 2 & 0x01);
	seq_printf(s, "Palm = %d\t", state_info[0] >> 3 & 0x01);
	seq_printf(s, "Idle mode = %d\t", state_info[0] >> 4 & 0x01);
	seq_printf(s, "Water = %d\n", state_info[0] >> 5 & 0x01);
	seq_printf(s, "TX Hop = %d\t", state_info[0] >> 6 & 0x01);
	seq_printf(s, "AC mode = %d\t", state_info[0] >> 7 & 0x01);
	seq_printf(s, "Glove = %d\t", state_info[1] & 0x01);
	seq_printf(s, "Stylus = %d\t", state_info[1] >> 1 & 0x01);
	seq_printf(s, "Hovering = %d\t", state_info[1] >> 2 & 0x01);
	seq_printf(s, "Proximity = %d\t", state_info[1] >> 3 & 0x01);
	seq_printf(s, "KEY = %d\n", state_info[1] >> 4 & 0x0F);
#else
	seq_printf(s, "ReCal = %d\t", state_info[0] & 0x01);
	seq_printf(s, "Palm = %d\t", state_info[0] >> 1 & 0x01);
	seq_printf(s, "AC mode = %d\t", state_info[0] >> 2 & 0x01);
	seq_printf(s, "Water = %d\n", state_info[0] >> 3 & 0x01);
	seq_printf(s, "Glove = %d\t", state_info[0] >> 4 & 0x01);
	seq_printf(s, "TX Hop = %d\t", state_info[0] >> 5 & 0x01);
	seq_printf(s, "Base Line = %d\t", state_info[0] >> 6 & 0x01);
	seq_printf(s, "OSR Hop = %d\t", state_info[1] >> 3 & 0x01);
	seq_printf(s, "KEY = %d\n", state_info[1] >> 4 & 0x0F);
#endif
}

static void himax_diag_arrange_print(struct seq_file *s, int i, int j,
				int transpose)
{
	if (transpose)
		seq_printf(s, "%6d", diag_mutual[j + i * hx_s_ic_data->rx_num]);
	else
		seq_printf(s, "%6d", diag_mutual[i + j * hx_s_ic_data->rx_num]);
}

/* ready to print second step which is column*/
static void himax_diag_arrange_inloop(struct seq_file *s, int in_init,
				int out_init, bool transpose, int j)
{
	int x_channel = hx_s_ic_data->rx_num;
	int y_channel = hx_s_ic_data->tx_num;
	int i;
	int in_max = 0;

	if (transpose)
		in_max = y_channel;
	else
		in_max = x_channel;

	if (in_init > 0) { /* bit0 = 1 */
		for (i = in_init - 1; i >= 0; i--)
			himax_diag_arrange_print(s, i, j, transpose);

		if (transpose) {
			if (out_init > 0)
				seq_printf(s, " %5d\n", diag_self[j]);
			else
				seq_printf(s, " %5d\n",
					diag_self[x_channel - j - 1]);
		}
	} else {	/* bit0 = 0 */
		for (i = 0; i < in_max; i++)
			himax_diag_arrange_print(s, i, j, transpose);

		if (transpose) {
			if (out_init > 0)
				seq_printf(s, " %5d\n",
					diag_self[x_channel - j - 1]);
			else
				seq_printf(s, " %5d\n", diag_self[j]);
		}
	}
}

/* print first step which is row */
static void himax_diag_arrange_outloop(struct seq_file *s, int transpose,
				int out_init, int in_init)
{
	int j;
	int x_channel = hx_s_ic_data->rx_num;
	int y_channel = hx_s_ic_data->tx_num;
	int out_max;
	int self_cnt;

	if (transpose)
		out_max = x_channel;
	else
		out_max = y_channel;

	if (out_init > 0) { /* bit1 = 1 */
		self_cnt = 1;

		for (j = out_init - 1; j >= 0; j--) {
			seq_printf(s, "%3c%02d%c", '[', j + 1, ']');
			himax_diag_arrange_inloop(s, in_init, out_init,
					transpose, j);

			if (!transpose) {
				seq_printf(s, " %5d\n",
				  diag_self[y_channel + x_channel - self_cnt]);
				self_cnt++;
			}
		}
	} else {	/* bit1 = 0 */
		/* self_cnt = x_channel; */
		for (j = 0; j < out_max; j++) {
			seq_printf(s, "%3c%02d%c", '[', j + 1, ']');
			himax_diag_arrange_inloop(s, in_init, out_init,
					transpose, j);

			if (!transpose) {
				seq_printf(s, " %5d\n",
					diag_self[j + x_channel]);
			}
		}
	}
}

/* determin the output format of diag */
static void himax_diag_arrange(struct seq_file *s)
{
	int x_channel = hx_s_ic_data->rx_num;
	int y_channel = hx_s_ic_data->tx_num;
	int bit2, bit1, bit0;
	int i;
	/* rotate bit */
	bit2 = g_diag_arr_num >> 2;
	/* reverse Y */
	bit1 = g_diag_arr_num >> 1 & 0x1;
	/* reverse X */
	bit0 = g_diag_arr_num & 0x1;

	if (g_diag_arr_num < 4) {
		for (i = 0 ; i <= x_channel; i++)
			seq_printf(s, "%3c%02d%c", '[', i, ']');

		seq_puts(s, "\n");
		himax_diag_arrange_outloop(s, bit2, bit1 * y_channel,
					bit0 * x_channel);
		seq_printf(s, "%6c", ' ');

		if (bit0 == 1) {
			for (i = x_channel - 1; i >= 0; i--)
				seq_printf(s, "%6d", diag_self[i]);
		} else {
			for (i = 0; i < x_channel; i++)
				seq_printf(s, "%6d", diag_self[i]);
		}
	} else {
		for (i = 0 ; i <= y_channel; i++)
			seq_printf(s, "%3c%02d%c", '[', i, ']');

		seq_puts(s, "\n");
		himax_diag_arrange_outloop(s, bit2, bit1 * x_channel,
					bit0 * y_channel);
		seq_printf(s, "%6c", ' ');

		if (bit1 == 1) {
			for (i = x_channel + y_channel - 1; i >= x_channel;
			  i--)
				seq_printf(s, "%6d", diag_self[i]);
		} else {
			for (i = x_channel; i < x_channel + y_channel; i++)
				seq_printf(s, "%6d", diag_self[i]);
		}
	}
}

/* DSRAM thread */
bool himax_ts_diag_func(int dsram_type)
{
	int i, j;
	unsigned int index = 0;
	int x_channel = hx_s_ic_data->rx_num;
	int y_channel = hx_s_ic_data->tx_num;
	int total_size = (y_channel * x_channel + y_channel + x_channel) * 2;
	uint8_t *info_data = NULL;
	int32_t *mutual_data = NULL;
	int32_t *mutual_data_new = NULL;
	int32_t *mutual_data_old = NULL;
	int32_t *self_data = NULL;
	int32_t *self_data_new = NULL;
	int32_t *self_data_old = NULL;
	uint8_t *mutual_data_byte = NULL;
	uint8_t *self_data_byte = NULL;
	int32_t new_data;
	int update_idx = 0;
	/* 1:common dsram,2:100 frame Max,3:N-(N-1)frame */
	if (dsram_type < 1 || dsram_type > 3) {
		E("%s: type %d is out of range\n", __func__, dsram_type);
		return false;
	}

	info_data = kcalloc(total_size, sizeof(uint8_t), GFP_KERNEL);
	if (info_data == NULL) {
		E("%s: Failed to allocate memory\n", __func__);
		return false;
	}

	memset(info_data, 0, total_size * sizeof(uint8_t));

	if (dsram_type == 1 || dsram_type == 2) {
		mutual_data = getMutualBuffer();
		self_data = getSelfBuffer();
	} else if (dsram_type == 3) {
		mutual_data = getMutualBuffer();
		mutual_data_new = getMutualNewBuffer();
		mutual_data_old = getMutualOldBuffer();
		self_data = getSelfBuffer();
		self_data_new = getSelfNewBuffer();
		self_data_old = getSelfOldBuffer();
	}

	if (!hx_s_core_fp._get_DSRAM_data(info_data, dsram_flag)) {
		E("%s: Get DSRAM data failed\n", __func__);
		kfree(info_data);
		return false;
	}
	index = 0;
	if (hx_s_debug_data->is_stack_output_bin) {
		mutual_data_byte = getMutualBufferByte();
		self_data_byte = getSelfBufferByte();
		memcpy(mutual_data_byte, info_data, y_channel * x_channel*2);
		memcpy(self_data_byte, info_data+(y_channel * x_channel*2),
			(y_channel + x_channel)*2);
	} else {
		for (i = 0; i < y_channel; i++) { /*mutual data*/
			for (j = 0; j < x_channel; j++) {
				new_data =
					(((int8_t)info_data[index + 1] << 8)|
					info_data[index]);

				if (dsram_type <= 1) {
					mutual_data[i * x_channel + j] =
						new_data;
				} else if (dsram_type == 2) {
					/* Keep max data */
					if (mutual_data[i * x_channel + j] <
						new_data)
						mutual_data[i * x_channel + j] =
							new_data;
				} else if (dsram_type == 3) {
					/* Cal data for [N]-[N-1] frame */
					mutual_data_new[i * x_channel + j] =
						new_data;
					mutual_data[i * x_channel + j] =
					  mutual_data_new[i * x_channel + j]
					- mutual_data_old[i * x_channel + j];
				}
				index += 2;
			}
		}

		for (i = 0; i < x_channel + y_channel; i++) { /*self data*/
			new_data = (((int8_t)info_data[index + 1] << 8) |
					info_data[index]);
			if (dsram_type <= 1) {
				self_data[i] = new_data;
			} else if (dsram_type == 2) { /* Keep max data */
				if (self_data[i] < new_data)
					self_data[i] = new_data;
			} else if (dsram_type == 3) {
				/* Cal data for [N]-[N-1] frame */
				self_data_new[i] = new_data;
				self_data[i] = self_data_new[i] -
					self_data_old[i];
			}
			index += 2;
		}
	}

	if (dsram_type == 3) {
		memcpy(mutual_data_old, mutual_data_new,
			x_channel * y_channel * sizeof(int32_t));
			/* copy N data to N-1 array */
		memcpy(self_data_old, self_data_new,
			(x_channel + y_channel) * sizeof(int32_t));
			/* copy N data to N-1 array */
	}
	if (hx_s_debug_data->is_stack_full_raw) {
		update_idx = atomic_read(&(g_rb_frame.p_update));
		// memcpy(&g_rb_frame.rawdata[update_idx].self[0],
		//	&self_data[0],
		// (x_channel + y_channel) * sizeof(int32_t));
		// memcpy(&g_rb_frame.rawdata[update_idx].mutual[0],
		//	&mutual_data[0],
		// (x_channel * y_channel) * sizeof(int32_t));
		memcpy(&g_rb_frame.rawdata[update_idx].self[0],
			&info_data[2 * (x_channel * y_channel)],
			2 * (x_channel + y_channel) * sizeof(uint8_t));
		memcpy(&g_rb_frame.rawdata[update_idx].mutual[0],
			&info_data[0],
			2 * (x_channel * y_channel) * sizeof(uint8_t));
		g_rb_frame.frame_idx[update_idx] = g_rb_frame.rawdata->index++;

		if (atomic_read(&(g_rb_frame.p_update)) ==
			(HX_RB_FRAME_SIZE - 1))
			atomic_set(&(g_rb_frame.p_update), 0);
		else {
			atomic_set(&(g_rb_frame.p_update),
				atomic_read(&(g_rb_frame.p_update)) + 1);
		}
		g_rb_frame.rawdata->cnt_update++;
		I("Updated: rb_idx=%d, value_idx=%d\n",
			update_idx,
			g_rb_frame.frame_idx[update_idx]);
	}
	kfree(info_data);

	return true;
}

static int find_idx_min_array(const int *array, int size)
{
	int result = 0;
	int i = 0;

	for (i = 0; i < size; i++) {
		if (array[result] > array[i])
			result = i;
	}

	return result;
}

static void node_output_bin(struct seq_file *s, int mutual_len,
		int self_len, uint8_t *self,
		uint8_t *mutual, int rb_idx, int frm_idx){
	uint8_t header[RAW_DATA_HEADER_LENGTH];

	header[0] = 0x03;//Mutual+self
	header[1] = 0xff&rb_idx;
	header[2] = 0xff&frm_idx;
	header[3] = 0xff&(frm_idx>>8);
	header[4] = 0xff&(frm_idx>>16);
	header[5] = 0xff&(frm_idx>>24);

	seq_write(s, header, RAW_DATA_HEADER_LENGTH);
	seq_write(s, self,
		sizeof(uint8_t) * self_len);
	// Mutual
	seq_write(s, mutual,
		sizeof(uint8_t) * mutual_len);
}
static int himax_diag_print(struct seq_file *s, void *v)
{
	int x_num = hx_s_ic_data->rx_num;
	int y_num = hx_s_ic_data->tx_num;
	size_t ret = 0;
	uint16_t mutual_num, self_num;
	int output_p = atomic_read(&(g_rb_frame.p_output));
	int next_output_p;

	mutual_num	= (x_num * y_num)*2;
	self_num	= (x_num + y_num)*2;
	/* don't add KEY_COUNT */
	if (!hx_s_debug_data->is_stack_full_raw) {
		if (hx_s_debug_data->is_stack_output_bin) {

			node_output_bin(s, mutual_num,
				self_num, diag_self_byte,
					diag_mutual_byte, 0, 0);

		} else {
			seq_printf(s, "ChannelStart: %4d,%4d\n\n",
				x_num, y_num);
			/*	start to show out the raw data in adb shell */
			himax_diag_arrange(s);
			seq_puts(s, "\n");
			seq_puts(s, "ChannelEnd");
			seq_puts(s, "\n");

			/* print Mutual/Slef Maximum and Minimum */
			himax_get_mutual_edge();
			himax_get_self_edge();
			seq_printf(s, "Mutual Max:%3d, Min:%3d\n", g_max_mutual,
				g_min_mutual);
			seq_printf(s, "Self Max:%3d, Min:%3d\n", g_max_self,
				g_min_self);

			/*pring state info*/
			print_state_info(s);
		}
	} else {
		if (output_p + 1 == HX_RB_FRAME_SIZE)
			next_output_p = 0;
		else
			next_output_p = output_p + 1;
		if (g_rb_frame.frame_idx[output_p] <
			g_rb_frame.frame_idx[next_output_p])
			output_p = next_output_p;

		if (g_rb_frame.rawdata->cnt_update >= (HX_RB_FRAME_SIZE - 1))
			output_p = find_idx_min_array(
				g_rb_frame.frame_idx,
				HX_RB_FRAME_SIZE);

		if (g_re_ass_rb_idx >= 0) {
			I("reassign to get idx %d!\n", g_re_ass_rb_idx);
			output_p = g_re_ass_rb_idx;
			g_re_ass_rb_idx = -9487;
		}
		node_output_bin(s, mutual_num,
			self_num, g_rb_frame.rawdata[output_p].self,
			g_rb_frame.rawdata[output_p].mutual, output_p,
			g_rb_frame.frame_idx[output_p]);


		atomic_set(&(g_rb_frame.p_output), output_p);
		g_rb_frame.rawdata->cnt_update = 0;
	}

	/* recovery status after print*/
	g_max_mutual = 0;
	g_min_mutual = 0xFFFF;
	g_max_self = 0;
	g_min_self = 0xFFFF;

	if (s->count >= s->size)
		overflow++;

	return ret;
}

static int himax_stack_show(struct seq_file *s, void *v)
{
	struct himax_ts_data *ts = hx_s_ts;

	if (hx_s_debug_data->is_call_help) {
		seq_puts(s, HELP_DIAG);
		hx_s_debug_data->is_call_help = false;
	} else {
		if (ts->diag_cmd)
			himax_diag_print(s, v);
		else
			seq_puts(s, "Please set raw out select 'echo diag,X > debug'\n\n");
	}
	return 0;
}
__CREATE_OREAD_NODE_HX(stack);

static int himax_sram_read(struct seq_file *s, void *v, uint8_t rs)
{
	struct himax_ts_data *ts = hx_s_ts;
	int d_type;
	int current_size =
		((hx_s_ic_data->tx_num + 1) * (hx_s_ic_data->rx_num + 1)
		+ hx_s_ic_data->tx_num + hx_s_ic_data->rx_num) * 10 + 256;
	if (hx_s_debug_data->is_call_help) {
		seq_puts(s, HELP_DIAG);
		hx_s_debug_data->is_call_help = false;
	} else {
		d_type = (!ts->diag_cmd)?rs:ts->diag_cmd;

		s->size = current_size;
		I("%s, s->size = %d\n", __func__, (int)s->size);

		s->buf = kcalloc(s->size, sizeof(char), GFP_KERNEL);
		if (s->buf == NULL) {
			E("%s,%d: Memory allocation falied!\n",
				__func__, __LINE__);
			return -ENOMEM;
		}

		memset(s->buf, 0, s->size * sizeof(char));

		if (!overflow) {
			if (!process_type) {
				himax_int_enable(0);
				hx_s_core_fp._diag_register_set(d_type,
					0, false);

				/* use process type 1 for default */
				if (!himax_ts_diag_func(1))
					seq_puts(s, "Get sram data failed.");
				else
					himax_diag_print(s, v);

				ts->diag_cmd = 0;
				hx_s_core_fp._diag_register_set(0, 0, false);
				himax_int_enable(1);
			}
		}

		if ((process_type <= 3
		&& ts->diag_cmd
		&& dsram_flag)
		|| overflow) {
			himax_diag_print(s, v);
			overflow = 0;
		}
	}
	return 0;
}

static int himax_delta_show(struct seq_file *s, void *v)
{
	return himax_sram_read(s, v, 0x09);
}
__CREATE_OREAD_NODE_HX(delta);

static int himax_dc_show(struct seq_file *s, void *v)
{
	return himax_sram_read(s, v, 0x0A);
}
__CREATE_OREAD_NODE_HX(dc);

static int himax_baseline_show(struct seq_file *s, void *v)
{
	return himax_sram_read(s, v, 0x0B);
}
__CREATE_OREAD_NODE_HX(baseline);

#if defined(HX_RST_PIN_FUNC)
static int test_rst_pin(void)
{
	int rst_sts1 = -1;
	int rst_sts2 = -1;
	int cnt = 0;
	uint8_t tmp_addr[DATA_LEN_4] = {0};
	uint8_t tmp_data[DATA_LEN_4] = {0};
	uint8_t tmp_read[DATA_LEN_4] = {0};
	int ret = NO_ERR;

	himax_int_enable(0);
	hx_s_core_fp._sense_off(true);


	usleep_range(20000, 20001);
	hx_parse_assign_cmd(0x00000001, tmp_data, DATA_LEN_4);
	hx_s_core_fp._register_write(0x900000F0, tmp_data, DATA_LEN_4);
	usleep_range(20000, 20001);
	hx_s_core_fp._register_read(0x900000F0, tmp_read, DATA_LEN_4);
	I("R%02X%02X%02X%02XH = 0x%02X%02X%02X%02X\n",
		tmp_addr[3], tmp_addr[2], tmp_addr[1], tmp_addr[0],
		tmp_read[3], tmp_read[2], tmp_read[1], tmp_read[0]);
	I("trigger Reset Pin\n");
#if defined(HX_ZERO_FLASH)
	hx_s_core_fp._ic_reset(3);
#else
	hx_s_core_fp._ic_reset(0);
#endif
	usleep_range(20000, 20001);
	do {
		hx_parse_assign_cmd(0x900000A8, tmp_addr, DATA_LEN_4);
		hx_s_core_fp._register_read(0x900000A8, tmp_read, DATA_LEN_4);
		I("R%02X%02X%02X%02XH = 0x%02X%02X%02X%02X\n",
			tmp_addr[3], tmp_addr[2], tmp_addr[1], tmp_addr[0],
			tmp_read[3], tmp_read[2], tmp_read[1], tmp_read[0]);
		rst_sts1 = tmp_read[0];
		cnt++;
		if (rst_sts1 == 0x05)
			break;
		if (rst_sts1 == 0x00)
			cnt += 5;
		if (cnt > 20)
			goto END_FUNC;
	} while (rst_sts1 == 0x04);

	hx_parse_assign_cmd(0x900000F0, tmp_addr, DATA_LEN_4);
	hx_s_core_fp._register_read(0x900000F0, tmp_read, DATA_LEN_4);
	I("R%02X%02X%02X%02XH = 0x%02X%02X%02X%02X\n",
		tmp_addr[3], tmp_addr[2], tmp_addr[1], tmp_addr[0],
		tmp_read[3], tmp_read[2], tmp_read[1], tmp_read[0]);
	rst_sts2 = tmp_read[0];

END_FUNC:
	if (rst_sts1 == 0x05 && rst_sts2 == 0x00)
		I("%s: TP Reset test OK!\n", __func__);
	else if (rst_sts1 == 0xFF || rst_sts2 == 0x01) {
		I("%s: TP Reset test Fail!\n", __func__);
		ret++;
	} else {
		I("%s, Unknown Fail state1=0x%02X, state2=0x%02X!\n",
			__func__, rst_sts1, rst_sts2);
		ret++;
	}

	hx_s_core_fp._sense_on(0x00);
	himax_int_enable(1);
	return ret;
}
#endif

static int himax_reset_read(struct seq_file *m)
{
	int ret = NO_ERR;

	if (hx_s_debug_data->is_checking_reset) {
#if defined(HX_RST_PIN_FUNC)
		if (test_rst_pin() == NO_ERR) {
			seq_printf(m,
			"Reset PIN check OK!\n");
		} else {
			seq_printf(m,
			"Reset PIN check Fail!\n");
		}
#endif
		hx_s_debug_data->is_checking_reset = false;
	} else {
		if (hx_s_debug_data->is_call_help) {
			seq_printf(m, HELP_RST);
			hx_s_debug_data->is_call_help = false;
		} else {
			seq_printf(m, "reset,%d finish\n",
				(hx_s_debug_data->reset_cmd_value+1));
		}
	}
	return ret;
}
static ssize_t himax_reset_write(char *buf, size_t len)
{
	if (len >= 12) {
		I("%s: no command exceeds 12 chars.\n", __func__);
		return -EFAULT;
	}

	if (strnstr(buf, "help", len) != NULL) {
		hx_s_debug_data->is_call_help = true;
	} else {
#if defined(HX_RST_PIN_FUNC)
		if (strnstr(buf, "test", len) != NULL) {
			hx_s_debug_data->is_checking_reset = true;
			goto END;
		}
#endif
		if (buf[0] == '1')
			hx_s_debug_data->reset_cmd_value = 0;
		else if (buf[0] == '2')
			hx_s_debug_data->reset_cmd_value = 1;
		else if (buf[0] == '3')
			hx_s_debug_data->reset_cmd_value = 2;
		else if (buf[0] == '4')
			hx_s_debug_data->reset_cmd_value = 3;
		else if (buf[0] == 'l' && buf[1] == 'o')
			hx_s_debug_data->reset_cmd_value = 4;
		else if (buf[0] == 'h' && buf[1] == 'i')
			hx_s_debug_data->reset_cmd_value = 5;
		hx_s_core_fp._ic_reset(hx_s_debug_data->reset_cmd_value);
		/* else if (buf[0] == '5') */
		/*	ESD_HW_REST(); */

	}
#if defined(HX_ZERO_FLASH)
	if (buf[0] != '4')
		hx_s_core_fp._0f_reload_to_active();
#endif
#if defined(HX_RST_PIN_FUNC)
END:
#endif
	return len;
}



void hx_dump_prog_set(uint8_t prog)
{
	g_dump._progress = prog;
	if (prog == HX_DUMP_GOING)
		hx_s_debug_data->flash_dump_going = HX_DUMP_GOING;
	else
		hx_s_debug_data->flash_dump_going = HX_DUMP_START;
}

static int himax_flash_dump_show(struct seq_file *s, void *v)
{
	ssize_t ret = 0;
	uint32_t sram_idx = 0;
	uint32_t i, j;
	uint8_t flash_progress = g_dump._progress;
	// uint8_t flash_cmd = g_dump._show;
	bool flash_rst = g_dump._rst;
	uint32_t dump_size = 0;
	uint32_t print_size = 0;
#if defined(HX_RW_FILE)
	loff_t pos = 0;
#endif

	I("dump_progress = %d\n", flash_progress);

	if (g_dump._cmd == 0) {
		seq_printf(s, HELP_FLASH_DUMP);
		goto END;
	}

	if (!flash_rst) {
		seq_puts(s, "Dump:FAIL,please check\n");
		goto END;
	}

	if (flash_progress == HX_DUMP_START) {
		seq_puts(s, "Dump:Start\n");
		goto END;
	} else if (flash_progress == HX_DUMP_GOING) {
		seq_puts(s, "Dump: on-going\n");
		goto END;
	}

	if (flash_progress == HX_DUMP_END)
		seq_puts(s, "Dump:Finish\n");

	dump_size = g_dump._size;

	if (dump_size > 0)
		print_size = dump_size * 6 + 256;

	I("Now buffer size=%d, data size = %d ,print size = %d!\n",
		(int)s->size, dump_size, print_size);
	/*print flash dump data*/
	//g_dump._show=2 or 3
	if (g_dump._show == HX_DUMP_PRINT ||
		g_dump._show == HX_DUMP_BOTH) {
		if (s->size < print_size) {
			s->size = print_size;
			I("%s, s->size = %d\n", __func__, (int)s->size);
			s->buf = kcalloc(s->size, sizeof(char), GFP_KERNEL);
			if (s->buf == NULL) {
				E("%s,%d: Memory allocation falied!\n",
					__func__, __LINE__);
				ret = -ENOMEM;
				goto END;
			}
			memset(s->buf, 0, s->size * sizeof(char));
		}

		seq_puts(s, "Start to print dump data\n");
		if (g_dump._cmd == HX_DUMP_ISRAM) {
			j = 0;
			for (i = 0; i < dump_size; i++) {
				if (i == 0) {
					seq_printf(s, "isram[%d]=0x%08X:\n",
						0,
						hx_s_ic_data->isram_addr[0]);
					sram_idx = hx_s_ic_data->isram_sz[0];
					j++;
				}
				if (i == sram_idx
					&& j < hx_s_ic_data->isram_num) {
					seq_printf(s, "\nisram[%d]=0x%08X:\n",
						j,
						hx_s_ic_data->isram_addr[j]);
					sram_idx += hx_s_ic_data->isram_sz[j];
					j++;
				}
				seq_printf(s, "0x%02X,", g_dump._buf[i]);
				if ((i % 16) == 15)
					seq_puts(s, "\n");
			}
		} else if ((g_dump._cmd == HX_DUMP_DSRAM &&
			g_dump._addr != HX_DUMP_NSET) ||
			g_dump._cmd == HX_DUMP_FLASH) {
			for (i = 0; i < dump_size; i++) {
				seq_printf(s, "0x%02X,", g_dump._buf[i]);
				if ((i % 16) == 15)
					seq_puts(s, "\n");
			}

		} else {
			j = 0;
			for (i = 0; i < dump_size; i++) {
				if (i == 0) {
					seq_printf(s, "dsram[%d]=0x%08X:\n",
						0,
						hx_s_ic_data->dsram_addr[0]);
					sram_idx = hx_s_ic_data->dsram_sz[0];
					j++;
				}
				if (i == sram_idx
					&& j < hx_s_ic_data->dsram_num) {
					seq_printf(s, "\ndsram[%d]=0x%08X:\n",
						j,
						hx_s_ic_data->dsram_addr[j]);
					sram_idx += hx_s_ic_data->dsram_sz[j];
					j++;
				}
				seq_printf(s, "0x%02X,", g_dump._buf[i]);
				if ((i % 16) == 15)
					seq_puts(s, "\n");
			}
		}
	}

	if (g_dump._show == HX_DUMP_FILE ||
		g_dump._show == HX_DUMP_BOTH) {
#if defined(HX_RW_FILE)
		I("Now vts_name=%s\n", g_file_path);
		if (hx_open_file(g_file_path) == NO_ERR) {
			hx_write_file(g_dump._buf, dump_size, pos);
			hx_close_file();
		}
#else
		I("%s:No support RW file\n", __func__);
#endif
	}

END:
	seq_puts(s, "Dump:END\n");
	return ret;
}
#define FDS_DBG1 "%s, input cmd:is_flash=%d, is_file=%d, dump_size=%d, addr=0x%08X\n"
#define FDS_DBG2 "%s, assigned:is_flash=%d, is_file=%d, dump_size=%d, addr=0x%08X\n"
static ssize_t himax_flash_dump_store(struct file *filp,
			const char __user *buff, size_t len, loff_t *data)
{
	char buf[80] = {0};
	char *input_str[4];
	int i = 0;
	int is_flash = -1;
	int is_file = -1;
	int dump_size = -1;
	uint32_t addr = 0x000000;
	uint32_t str_len = 0;
	int chk;

	g_dump._cmd = is_flash = HX_DUMP_NSET;
	g_dump._size = dump_size = HX_DUMP_NSET;
	g_dump._show = is_file = HX_DUMP_NSET;
	g_dump._addr = addr = HX_DUMP_NSET;

	if (len >= 80) {
		I("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}

	if (copy_from_user(buf, buff, len))
		return -EFAULT;

	I("%s: buf = %s\n", __func__, buf);

	if (g_dump._progress == HX_DUMP_GOING) {
		E("%s: Now is busy, skip!\n", __func__);
		return len;
	}

	str_len = len;
	buf[str_len - 1] = '\0';

	for (i = 0; i < 4; i++)
		input_str[i] = kzalloc(sizeof(char) * 128, GFP_KERNEL);

	_str_to_arr_in_char(input_str, 4, buf, ',');

	for (i = 0; i < 4; i++) {
		if (input_str[i] == NULL) {
			continue;
		} else {
			I("%d:%s\n", i, input_str[i]);
			if (i == 0) {
				if (strnstr(input_str[i], "help",
					len) != NULL) {
					is_flash = 0;
					goto CON_WORK;
				} else if (strnstr(input_str[i], "flash",
					len) != NULL) {
					is_flash = HX_DUMP_FLASH;
				} else if (strnstr(input_str[i], "dsram",
					len) != NULL) {
					is_flash = HX_DUMP_DSRAM;
				} else if (strnstr(input_str[i], "isram",
					len) != NULL) {
					is_flash = HX_DUMP_ISRAM;
				} else {
					is_flash = HX_DUMP_NSET;
					goto CON_WORK;
				}
			} else if (i == 1) {
				if (strnstr(input_str[i], "print",
					len) != NULL) {
					is_file = HX_DUMP_PRINT;
				} else if (strnstr(input_str[i], "file",
					len) != NULL) {
					is_file = HX_DUMP_FILE;
				} else {
					is_file = HX_DUMP_NSET;
					goto CON_WORK;
				}
			} else if (i == 2) {
				chk = kstrtoint(input_str[i], 10, &dump_size);
				if (chk) {
					I("%s:value parse fail!\n",
						__func__);
					dump_size = HX_DUMP_NSET;
				}
				I("size=%d\n", dump_size);
			} else if (i == 3) {
				chk = kstrtoint(input_str[i], 16, &addr);
				if (chk) {
					E("%s:address parse fail!\n",
						__func__);
					addr = HX_DUMP_NSET;
				}
				I("addr=0x%08X\n", addr);
			}
		}
	}
CON_WORK:
	I(FDS_DBG1,
		__func__, is_flash, is_file, dump_size, addr);

	g_dump._cmd = is_flash;
	g_dump._size = dump_size;
	g_dump._show = is_file;
	g_dump._addr = addr;

	if ((g_dump._cmd != 0 &&
		g_dump._cmd != HX_DUMP_NSET) &&
		g_dump._size == HX_DUMP_NSET &&
		g_dump._show == HX_DUMP_NSET &&
		g_dump._addr == HX_DUMP_NSET) {
		g_dump._show = HX_DUMP_BOTH;
		I("%s: Single CMD will setup next!\n",
			__func__);
	}

	I(FDS_DBG2,
		__func__, g_dump._cmd,
		g_dump._show, g_dump._size, g_dump._addr);
	hx_dump_prog_set(HX_DUMP_START);
	g_dump._rst = true;
	queue_work(hx_s_ts->dump_wq, &hx_s_ts->dump_work);

	for (i = 0; i < 4; i++)
		kfree(input_str[i]);

	return len;
}
__CREATE_RW_NODE_HX(flash_dump);

void himax_ts_dump_func(void)
{
	int ret = NO_ERR;
	int bus_sz = 0;
	uint32_t tmp_addr = 0;
	int read_times = 0;
	int i = 0;
	int j = 0;

	g_dump._size = g_dump._size % 4 ?
			g_dump._size + g_dump._size % 4 :
			g_dump._size;

#if defined(BUS_R_DLEN)
	if (g_dump._size < BUS_R_DLEN) {
		bus_sz = g_dump._size;
		read_times = 1;
	} else {
		bus_sz = BUS_R_DLEN;
		read_times = (g_dump._size / bus_sz) +
			(g_dump._size % bus_sz > 0 ? 1 : 0);
	}
#endif

	hx_dump_prog_set(HX_DUMP_GOING);

	/*msleep(100);*/
	// I("%s: flash_command = %d enter.\n", __func__, flash_command);
	I("%s:is_flash=%d, is_file=%d, size=%d, add=0x%08X\n",
		__func__, g_dump._cmd, g_dump._show,
		g_dump._size, g_dump._addr);

	I("%s: read_size=%d, read_times=%d\n",
		__func__, bus_sz, read_times);

	if (g_dump._cmd == HX_DUMP_FLASH) {
		g_dump._size = hx_s_ic_data->flash_size;
		himax_int_enable(0);
		hx_s_core_fp._flash_dump_func(g_dump._cmd, g_dump._size,
		  g_dump._buf);
		g_dump._rst = true;
		himax_int_enable(1);
	} else if (g_dump._cmd == HX_DUMP_DSRAM
		&& g_dump._addr != HX_DUMP_NSET) {
		I("dump dsram start\n");
		if (g_dump._size > g_dump._buf_len) {
			E("Size is over than buffer, plz check it\n");
			g_dump._rst = false;
			goto END;
		}
		for (i = 0; i < read_times; i++) {
			tmp_addr = g_dump._addr + i * bus_sz;
			if (i == (read_times - 1))
				ret = hx_s_core_fp._register_read(
					tmp_addr,
					&g_dump._buf[i * bus_sz],
					g_dump._size - i * bus_sz);
			else
				ret = hx_s_core_fp._register_read(
					tmp_addr,
					&g_dump._buf[i * bus_sz],
					bus_sz);
			if (ret) {
				E("read data fail,plz check\n");
				g_dump._rst = false;
				goto END;
			}
		}
		g_dump._rst = true;
	} else if (g_dump._cmd == HX_DUMP_DSRAM) {
		I("dump DSRAM all start\n");
		/*Use to record total size*/
		g_dump._size = 0;
		himax_int_enable(0);
		hx_s_core_fp._sense_off(true);
		for (i = 0; i < hx_s_ic_data->dsram_num; i++) {
			I("[%d] start 0x%08X\n",
				i,
				hx_s_ic_data->dsram_addr[i]);
			if ((g_dump._size + hx_s_ic_data->dsram_sz[i]) >
				g_dump._buf_len) {
				E("Size is over than buffer, plz check it\n");
				ret = -9487;
				break;
			}
#if defined(BUS_R_DLEN)
			if (hx_s_ic_data->dsram_sz[i] < BUS_R_DLEN) {
				bus_sz = hx_s_ic_data->dsram_sz[i];
				read_times = 1;
			} else {
				bus_sz = BUS_R_DLEN;
				read_times = (hx_s_ic_data->dsram_sz[i] /
					bus_sz) +
					(hx_s_ic_data->dsram_sz[i] % bus_sz
					> 0 ? 1 : 0);
			}
#endif
			I("%s: read_size=%d, read_times=%d\n",
				__func__, bus_sz, read_times);
			tmp_addr = hx_s_ic_data->dsram_addr[i];
			for (j = 0; j < read_times; j++) {
				tmp_addr += j * bus_sz;
				if (j == (read_times - 1))
					ret = hx_s_core_fp._register_read(
						tmp_addr,
						&g_dump._buf[g_dump._size + j *
							bus_sz],
						hx_s_ic_data->dsram_sz[i] - j *
							bus_sz);
				else
					ret = hx_s_core_fp._register_read(
						tmp_addr,
						&g_dump._buf[g_dump._size + j *
							bus_sz],
						bus_sz);
				if (ret) {
					E("read data fail,plz check\n");
					g_dump._rst = false;
					goto END;
				}
			}
			g_dump._size += hx_s_ic_data->dsram_sz[i];
		}
		hx_s_core_fp._sense_on(0x01);
		himax_int_enable(1);
		if (ret) {
			E("read data fail,plz check\n");
			g_dump._rst = false;
			goto END;
		}
		g_dump._rst = true;
	} else if (g_dump._cmd == HX_DUMP_ISRAM) {
		I("dump ISRAM all start\n");
		/*Use to record total size*/
		g_dump._size = 0;
		himax_int_enable(0);
		hx_s_core_fp._sense_off(true);
		for (i = 0; i < hx_s_ic_data->isram_num; i++) {
			I("[%d] start 0x%08X\n",
				i,
				hx_s_ic_data->isram_addr[i]);
			if ((g_dump._size + hx_s_ic_data->isram_sz[i]) >
				g_dump._buf_len) {
				E("Size is over than buffer, plz check it\n");
				ret = -9487;
				break;
			}
#if defined(BUS_R_DLEN)
			if (hx_s_ic_data->isram_sz[i] < BUS_R_DLEN) {
				bus_sz = hx_s_ic_data->isram_sz[i];
				read_times = 1;
			} else {
				bus_sz = BUS_R_DLEN;
				read_times = (hx_s_ic_data->isram_sz[i] /
					bus_sz) +
					(hx_s_ic_data->isram_sz[i] % bus_sz
					> 0 ? 1 : 0);
			}
#endif
			I("%s: read_size=%d, read_times=%d\n",
				__func__, bus_sz, read_times);
			tmp_addr = hx_s_ic_data->isram_addr[i];
			for (j = 0; j < read_times; j++) {
				tmp_addr += j * bus_sz;
				if (j == (read_times - 1))
					ret = hx_s_core_fp._register_read(
						tmp_addr,
						&g_dump._buf[g_dump._size + j *
							bus_sz],
						hx_s_ic_data->isram_sz[i] - j *
							bus_sz);
				else
					ret = hx_s_core_fp._register_read(
						tmp_addr,
						&g_dump._buf[g_dump._size + j *
							bus_sz],
						bus_sz);
				if (ret) {
					E("read data fail,plz check\n");
					g_dump._rst = false;
					goto END;
				}
			}
			g_dump._size += hx_s_ic_data->isram_sz[i];
		}
		hx_s_core_fp._sense_on(0x01);
		himax_int_enable(1);
		if (ret) {
			E("read data fail,plz check\n");
			g_dump._rst = false;
			goto END;
		}
		g_dump._rst = true;
	} else {
		E("Check cmd or address!\n");
		g_dump._rst = false;
		goto END;
	}

	I("Dump OK & Complete\n");


END:
	hx_dump_prog_set(HX_DUMP_END);
}

#if defined(HX_TP_PROC_GUEST_INFO)
static int printMat(struct seq_file *m, int max_size,
		 uint8_t *guest_str, int loc)
{
	int ret = loc;
	uint32_t i;

	for (i = 0; i < max_size; i++) {
		if ((i % 16) == 0 && i > 0)
			seq_puts(m, "\n");

		seq_printf(m, "0x%02X\t",
			guest_str[i]);
	}
	return ret;
}

static int printUnit(struct seq_file *m, int max_size,
		char *info_item, uint8_t *guest_str, int loc)
{
	int ret = loc;

	seq_printf(m, "%s:\n", info_item);
	ret = printMat(m, max_size, guest_str, ret);
	seq_puts(m, "\n");
	return ret;
}

static int himax_proc_guest_info_read(struct seq_file *m)
{
	int ret = 0;
	int j = 0;
	int max_size = 128;
	struct hx_guest_info *info = g_guest_info_data;


	if (hx_s_debug_data->is_call_help) {
		seq_printf(m, HELP_GUEST_INFO);
		hx_s_debug_data->is_call_help = false;
	} else {
		I("guest info progress\n");

		if (hx_s_core_fp.guest_info_get_status()) {
			seq_printf(m,
				"Not Ready\n");
			goto END_FUNCTION;
		} else {
			if (info->g_guest_info_type == 1) {
				for (j = 0; j < 3; j++) {
					ret = printUnit(m, max_size,
						g_guest_info_item[j],
						info->g_guest_str[j], ret);
					I("str[%d] %s\n", j,
						info->g_guest_str[j]);
				}
				ret = printUnit(m, max_size,
						g_guest_info_item[8],
						info->g_guest_str[8],
						ret);

				I("str[8] %s\n",
				info->g_guest_str[8]);

				ret = printUnit(m, max_size,
						g_guest_info_item[9],
						info->g_guest_str[9],
						ret);

				I("str[9] %s\n", info->g_guest_str[9]);
			} else if (info->g_guest_info_type == 0) {
				for (j = 0; j < 10; j++) {
					seq_printf(m, "%s:\n",
						g_guest_info_item[j]);

					if (info->g_guest_data_type[j] == 0) {
						seq_printf(m, "%s",
						info->g_guest_str_in_format[j]);
					} else {
						ret = printMat(m,
						info->g_guest_data_len[j],
						info->g_guest_str_in_format[j],
						ret);
					}
					seq_puts(m, "\n");
				}
			}
		}
	}
END_FUNCTION:
	return ret;
}

static ssize_t himax_proc_guest_info_write(char *buf, size_t len)
{
	if (len >= 80) {
		I("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}

	I("%s: buf = %s\n", __func__, buf);

	if (strnstr(buf, "help", len) != NULL) {
		hx_s_debug_data->is_call_help = true;
	} else {
		if (buf[0] == 'r') {
			I("%s,Test to get", __func__);
			queue_work(hx_s_ts->guest_info_wq,
					&hx_s_ts->guest_info_work);
		}
	}
	return len;
}

#endif

static int himax_test_bus_read(struct seq_file *m)
{
	int ret = NO_ERR;

	if (hx_s_core_fp._read_i2c_status())
		seq_printf(m,
			"Bus communication is bad.\n");
	else
		seq_printf(m,
			"Bus communication is good.\n");
	i2c_error_count = 0;

	return ret;
}

static int himax_info_read(struct seq_file *m)
{
	int ret = NO_ERR;

	seq_printf(m,
		"Himax Touch IC Information :\n");
	seq_printf(m,
		"%s\n", hx_s_ts->chip_name);

	switch (hx_s_ic_data->checksum_type) {
	case HX_TP_BIN_CHECKSUM_SW:
		seq_printf(m,
			"IC Checksum : SW\n");
		break;

	case HX_TP_BIN_CHECKSUM_HW:
		seq_printf(m,
			"IC Checksum : HW\n");
		break;

	case HX_TP_BIN_CHECKSUM_CRC:
		seq_printf(m,
			"IC Checksum : CRC\n");
		break;

	default:
		seq_printf(m,
			"IC Checksum error.\n");
		break;
	}

	if (hx_s_ic_data->int_is_edge)
		seq_printf(m,
			"Driver register Interrupt : EDGE TIRGGER\n");
	else
		seq_printf(m,
			"Driver register Interrupt : LEVEL TRIGGER\n");
	if (hx_s_ic_data->fw_int_is_edge != hx_s_ic_data->int_is_edge)
		seq_printf(m,
			"\tTirgger different in fw\n");
	if (hx_s_ts->protocol_type == PROTOCOL_TYPE_A)
		seq_printf(m,
			"Protocol : TYPE_A\n");
	else
		seq_printf(m,
			"Protocol : TYPE_B\n");

	seq_printf(m,
		"RX Num : %d\n", hx_s_ic_data->rx_num);
	if (hx_s_ic_data->fw_rx_num != hx_s_ic_data->rx_num)
		seq_printf(m,
			"\t[Warning]FW RX Num : %d\n", hx_s_ic_data->fw_rx_num);
	seq_printf(m,
		"TX Num : %d\n", hx_s_ic_data->tx_num);
	if (hx_s_ic_data->fw_tx_num != hx_s_ic_data->tx_num)
		seq_printf(m,
			"\t[Warning]FW TX Num : %d\n", hx_s_ic_data->fw_tx_num);
	seq_printf(m,
		"BT Num : %d\n", hx_s_ic_data->bt_num);
	if (hx_s_ic_data->fw_bt_num != hx_s_ic_data->bt_num)
		seq_printf(m,
			"\t[Warning]FW BT Num : %d\n", hx_s_ic_data->fw_bt_num);
	seq_printf(m,
		"X Resolution : %d\n", hx_s_ic_data->x_res);
	seq_printf(m,
		"Y Resolution : %d\n", hx_s_ic_data->y_res);
	seq_printf(m,
		"Max Point : %d\n", hx_s_ic_data->max_pt);
	if (hx_s_ic_data->fw_max_pt != hx_s_ic_data->max_pt)
		seq_printf(m,
			"\t[Warning]FW Max point : %d\n",
			hx_s_ic_data->fw_max_pt);
	seq_printf(m,
		"Stylus Support : %d\n", hx_s_ic_data->stylus_func);
	if (hx_s_ic_data->fw_stylus_func != hx_s_ic_data->stylus_func)
		seq_printf(m,
			"\t[Warning]FW Stylus Support : %d\n",
			hx_s_ic_data->fw_stylus_func);
	if (hx_s_ic_data->stylus_func)
		seq_printf(m,
			"Stylus Ratio :  %d\n", hx_s_ic_data->stylus_ratio);
	if ((hx_s_ic_data->fw_stylus_ratio !=
		hx_s_ic_data->stylus_ratio) &&
		hx_s_ic_data->stylus_func)
		seq_printf(m,
			"\t[Warning]FW Stylus Ratio : %d\n",
			hx_s_ic_data->fw_stylus_ratio);
	if (hx_s_ic_data->stylus_func)
		seq_printf(m,
			"Stylus V2 EN :  %d\n", hx_s_ic_data->stylus_id_v2);
	if ((hx_s_ic_data->stylus_id_v2 !=
		hx_s_ic_data->fw_stylus_id_v2) &&
		hx_s_ic_data->stylus_func)
		seq_printf(m,
			"\t[Warning]FW Stylus V2 EN : %d\n",
			hx_s_ic_data->fw_stylus_id_v2);
#if defined(HX_TP_PROC_2T2R)
	if (Is_2T2R) {
		seq_printf(m,
			"2T2R panel\n");
		seq_printf(m,
			"RX Num_2 : %d\n", HX_RX_NUM_2);
		seq_printf(m,
			"TX Num_2 : %d\n", HX_TX_NUM_2);
	}
#endif
#if defined(HX_ZERO_FLASH)
	seq_printf(m,
			"Flash Type: Zero Flash\n");
#else
	seq_printf(m,
			"Flash Type: With Flash\n");
#endif

	if (hx_s_ts->spi == NULL) {
		seq_printf(m,
			"Bus: I2C\n");
	} else {
		seq_printf(m,
			"Bus: SPI\n");
	}
	return ret;
}

static int himax_pen_info_read(struct seq_file *m)
{
	int ret = NO_ERR;

	if (FIX_HX_STYLUS_FUNC != 0 ||
		FIX_HX_STYLUS_ID_V2 != 0) {
		seq_printf(m,
			"Pen Battery info:\n");
		seq_printf(m,
			"%d\n", g_target_report_data->s[0].battery_info);

		seq_printf(m,
			"Pen ID:\n");
		seq_printf(m,
			"%llx\n", g_target_report_data->s[0].id);

	} else {
		seq_printf(m,
			"No support pen\n");
	}
	return ret;
}

static int hx_node_update(char *buf, size_t is_len)
{
	int result;
	char fileName[128];
	int fw_type = 0;

	const struct firmware *fw = NULL;

	fw_update_complete = false;
	fw_update_going = true;
	memset(fileName, 0, 128);
	/* parse the file name */
	if (!is_len)
		memcpy(fileName, buf, strlen(buf) * sizeof(char));
	else
		snprintf(fileName, is_len - 2, "%s", &buf[2]);

	I("%s: upgrade from file(%s) start!\n", __func__, fileName);
	/* manual upgrade will not use embedded firmware */
	result = request_firmware(&fw, fileName, hx_s_ts->dev);
	if (result < 0) {
		E("request FW %s failed(%d)\n", fileName, result);
		return result;
	}

	I("Gotten FW last 4bytes=0x%02X,%02X,%02X,%02X\n",
		fw->data[fw->size - 4],
		fw->data[fw->size - 3],
		fw->data[fw->size - 2],
		fw->data[fw->size - 1]);

	I("%s: FW image: %02X, %02X, %02X, %02X\n", __func__,
			fw->data[0], fw->data[1],
			fw->data[2], fw->data[3]);

	himax_int_enable(0);

#if defined(HX_ZERO_FLASH)
	I("NOW Running Zero flash update!\n");

	/* FW type: 0, normal; 1, MPFW */
	if (strcmp(fileName, MPAP_FWNAME) == 0)
		fw_type = 1;

	// CFG_TABLE_FLASH_ADDR = CFG_TABLE_FLASH_ADDR_T;
	hx_s_core_fp._bin_desc_get((unsigned char *)fw->data, HX1K);

	result = hx_s_core_fp._firmware_update_0f(fw, fw_type);
	if (result) {
		fw_update_complete = false;
		I("Zero flash update fail!\n");
	} else {
		fw_update_complete = true;
		I("Zero flash update complete!\n");
	}
#else
	I("NOW Running common flow update!\n");

	fw_type = (fw->size) / 1024;
	I("Now FW size is : %dk\n", fw_type);

	if (hx_s_core_fp._fts_ctpm_fw_upgrade_with_sys_fs(
	(unsigned char *)fw->data, fw->size, false) == 0) {
		E("%s: TP upgrade error, line: %d\n",
				__func__, __LINE__);
		fw_update_complete = false;
	} else {
		I("%s: TP upgrade OK, line: %d\n",
				__func__, __LINE__);
		fw_update_complete = true;
	}
#endif
	release_firmware(fw);
	goto firmware_upgrade_done;
firmware_upgrade_done:
	fw_update_going = false;
	hx_s_core_fp._reload_disable(0);
	hx_s_core_fp._power_on_init();
	hx_s_core_fp._read_FW_ver();

	hx_s_core_fp._tp_info_check();

	himax_int_enable(1);
	return result;
}

static int himax_update_read(struct seq_file *m)
{
	int ret = NO_ERR;

	if (hx_s_debug_data->is_call_help) {
		seq_printf(m, HELP_UPDATE);
		hx_s_debug_data->is_call_help = false;
	} else {
		if (!fw_update_going) {
			if (fw_update_complete)
				seq_printf(m,
					"FW Update Complete\n");
			else
				seq_printf(m,
					"FW Update Fail\n");
		} else {
			seq_printf(m,
					"FW Update Ongoing\n");
		}
	}

	return ret;
}
static ssize_t himax_update_write(char *buf, size_t len)
{

	I("Now cmd=%s", buf);
	if (strnstr(buf, "help", len) != NULL)
		hx_s_debug_data->is_call_help = true;
	else
		hx_node_update(buf, 0);
	return len;
}

static int himax_version_read(struct seq_file *m)
{
	int ret = NO_ERR;

	if (hx_s_debug_data->is_call_help) {
		seq_printf(m, HELP_VER);
		hx_s_debug_data->is_call_help = false;
	} else {
		seq_printf(m,
					"FW_VER = 0x%2.2X\n",
					hx_s_ic_data->vendor_fw_ver);
		if (hx_s_ts->chip_cell_type == CHIP_IS_ON_CELL)
			seq_printf(m,
				"CONFIG_VER = 0x%2.2X\n",
				hx_s_ic_data->vendor_config_ver);
		else {
			seq_printf(m,
				"TOUCH_VER = 0x%2.2X\n",
				hx_s_ic_data->vendor_touch_cfg_ver);
			seq_printf(m,
				"DISPLAY_VER = 0x%2.2X\n",
				hx_s_ic_data->vendor_display_cfg_ver);
		}
		if (hx_s_ic_data->vendor_cid_maj_ver < 0
		&& hx_s_ic_data->vendor_cid_min_ver < 0)
			seq_printf(m,
				"CID_VER = NULL\n");
		else
			seq_printf(m,
				"CID_VER = 0x%2.2X\n",
				(hx_s_ic_data->vendor_cid_maj_ver << 8 |
				hx_s_ic_data->vendor_cid_min_ver));

		if (hx_s_ic_data->vendor_panel_ver < 0)
			seq_printf(m,
				"PANEL_VER = NULL\n");
		else
			seq_printf(m,
				"PANEL_VER = 0x%2.2X\n",
				hx_s_ic_data->vendor_panel_ver);
		if (hx_s_ts->chip_cell_type == CHIP_IS_IN_CELL) {
			seq_printf(m,
				"Cusomer = %s\n",
				hx_s_ic_data->vendor_cus_info);

			seq_printf(m,
				"Project = %s\n",
				hx_s_ic_data->vendor_proj_info);
		}
		seq_puts(m, "\n");
		seq_printf(m,
			"Himax Touch Driver Version:\n");
		seq_printf(m,
			"%s\n", HIMAX_DRIVER_VER);
	}
	return ret;
}
static ssize_t himax_version_write(char *buf, size_t len)
{
	if (len >= 80) {
		I("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}
	I("Now cmd=%s", buf);
	if (strnstr(buf, "help", len) != NULL)
		hx_s_debug_data->is_call_help = true;
	else
		hx_s_core_fp._read_FW_ver();

	return len;
}

static int himax_list_cmd_read(struct seq_file *m)
{
	int ret = NO_ERR;
	int i, j;

	for (i = CMD_START_IDX; dbg_cmd_str[i] != NULL; i++) {
		for (j = 0; dbg_cmd_str[i][j] != NULL ; j++) {
			if (j != 0)
				seq_puts(m, ", ");
			seq_printf(m, dbg_cmd_str[i][j]);
		}
		seq_puts(m, "\n");
	}
	return ret;
}

static int himax_help_read(struct seq_file *m)
{
	int ret = NO_ERR;

	seq_printf(m, HELP_ALL_DEBUG);
	return ret;
}
static ssize_t himax_help_write(char *buf, size_t len)
{
	if (len >= 80) {
		I("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}
	I("Now cmd=%s", buf);

	return len;
}

static int himax_debug_show(struct seq_file *m, void *v)
{
	int ret = 0;

	if (dbg_cmd_flag) {
		if (dbg_func_ptr_r[dbg_cmd_flag]) {
			dbg_func_ptr_r[dbg_cmd_flag](m);
			goto END_FUNC_R;
		}
	}

	if (debug_level_cmd == 't') {
		if (!fw_update_going) {
			if (fw_update_complete)
				seq_printf(m,
					"FW Update Complete ");
			else
				seq_printf(m,
					"FW Update Fail ");
		} else {
			seq_printf(m,
					"FW Update Ongoing ");
		}
	} else if (debug_level_cmd == 'h') {
		if (handshaking_result == 0)
			seq_printf(m,
				"Handshaking Result = %d (MCU Running)\n",
				handshaking_result);
		else if (handshaking_result == 1)
			seq_printf(m,
				"Handshaking Result = %d (MCU Stop)\n",
				handshaking_result);
		else if (handshaking_result == 2)
			seq_printf(m,
				"Handshaking Result = %d (I2C Error)\n",
				handshaking_result);
		else
			seq_printf(m,
				"Handshaking Result = error\n");

	} else {
		if (hx_s_debug_data->is_info_check) {
			if (hx_s_core_fp._tp_info_check() == 0) {
				seq_printf(m,
					"FIX_TOUCH_INFO is OK\n");
			} else {
				seq_printf(m,
					"FIX_TOUCH_INFO does NOT match to FW information\n");
			}
			hx_s_debug_data->is_info_check = false;
		} else
			himax_list_cmd_read(m);
	}

END_FUNC_R:

	return ret;
}

static ssize_t himax_debug_store(struct file *file, const char __user *ubuf,
		size_t len, loff_t *data)
{

	char *buf;
	char *str_ptr = NULL;
	int str_len = 0;
	int input_cmd_len = 0;
	int i = 0, j = 0;

	if (len >= 4096) {
		I("%s: no command exceeds 4096 chars.\n", __func__);
		return -EFAULT;
	}

	buf = kzalloc(sizeof(char) * 4096, GFP_KERNEL);

	if (copy_from_user(buf, ubuf, len)) {
		len = -EFAULT;
		goto ENDFUCTION;
	}

	str_len = len;
	buf[str_len - 1] = '\0';/*remove \n*/
	input_cmd_len = (int)strcspn(buf, ",");
	if (input_cmd_len == -1)
		input_cmd_len = str_len;

	i = CMD_START_IDX; /* start from 1*/
	while (dbg_cmd_str[i] != NULL) {
		while (dbg_cmd_str[i][j] != NULL) {
			str_ptr = strnstr(buf, dbg_cmd_str[i][j], len);
			str_len = strlen(dbg_cmd_str[i][j]);
			if (str_len != input_cmd_len) {
				str_ptr = NULL;
				j++;
				continue;
			}
			if (str_ptr) {
				dbg_cmd_flag = i;
				debug_level_cmd = 0;
				I("Cmd is correct :%s, dbg_cmd = %d\n",
						str_ptr, dbg_cmd_flag);
				goto CHECK;
			}
			j++;
		}
		j = 0;
		i++;
	}
CHECK:
	if (!str_ptr) {
		I("Cmd is in old fromat or incorrect\n");
		dbg_cmd_flag = 0;
		goto CONTI;
	}

	if (buf[str_len] == ',') {
		dbg_cmd_par = buf + str_len + 1;
		if (dbg_func_ptr_w[dbg_cmd_flag])
			/* 2 => '/n' + ','*/
			dbg_func_ptr_w[dbg_cmd_flag](dbg_cmd_par,
					len - str_len - 2);

		I("string of paremeter is %s; dbg_cmd_par = %s, size = %d\n",
				buf + str_len + 1,
				dbg_cmd_par, (int)(len - str_len - 2));
	} else if (dbg_cmd_flag) {
		dbg_cmd_par = "";
		if (dbg_func_ptr_w[dbg_cmd_flag])
			dbg_func_ptr_w[dbg_cmd_flag](dbg_cmd_par, 0);
		I("No parameter case!\n");
	} else {
		I("Write cmd=%s, without parameter\n",
			dbg_cmd_str[dbg_cmd_flag][0]);
	}
CONTI:
	if (dbg_cmd_flag)
		goto ENDFUCTION;

	if (buf[0] == 't') {
		debug_level_cmd = buf[0];
		hx_node_update(buf, len);
	}  else if (buf[0] == 'c' && buf[1] == 't' && buf[2] == 'i') {
		/* Compare Touch Information */
		hx_s_debug_data->is_info_check = true;
		goto ENDFUCTION;
	} else {
		/* others,do nothing */
		debug_level_cmd = 0;
	}

ENDFUCTION:
	kfree(buf);
	return len;
}
__CREATE_RW_NODE_HX(debug);

static void himax_himax_data_init(void)
{
	hx_s_debug_data->_ts_dbg_func = himax_ts_dbg_func;
	hx_s_debug_data->_set_diag_cmd = himax_set_diag_cmd;
	hx_s_debug_data->_raw_full_stack = hx_dbg_raw_full_stack;
	hx_s_debug_data->flash_dump_going = false;
	hx_s_debug_data->is_checking_irq = false;
	hx_s_debug_data->is_info_check = false;
	hx_s_debug_data->is_checking_reset = false;
	hx_s_debug_data->is_call_help = false;
	hx_s_debug_data->is_stack_full_raw = false;
	hx_s_ts->select_slave_reg = false;
	hx_s_ts->acc_slave_reg = false;
}

static void himax_ts_dump_work_func(struct work_struct *work)
{
	himax_ts_dump_func();
}
#if defined(HX_TP_PROC_GUEST_INFO)
static void himax_ts_guest_info_work_func(struct work_struct *work)
{
	hx_s_core_fp.read_guest_info();
}
#endif

static void himax_ts_diag_work_func(struct work_struct *work)
{
	himax_ts_diag_func(process_type);

	if (process_type != 0) {
		queue_delayed_work(hx_s_ts->himax_diag_wq,
			&hx_s_ts->himax_diag_delay_work, 1 / 10 * HZ);
	} else {
		dsram_flag = false;
	}
}

void dbg_func_ptr_init(void)
{
	/*debug function ptr init*/
	int idx = 1;

	dbg_func_ptr_r[idx] = himax_crc_test_read;
	idx++;

	dbg_func_ptr_r[idx] = himax_proc_FW_debug_read;
	idx++;

	dbg_func_ptr_r[idx] = himax_attn_read;
	idx++;

	dbg_func_ptr_w[idx] = himax_layout_write;
	dbg_func_ptr_r[idx] = himax_layout_read;
	idx++;

#if defined(HX_EXCP_RECOVERY)
	dbg_func_ptr_w[idx] = himax_excp_cnt_write;
	dbg_func_ptr_r[idx] = himax_excp_cnt_read;
#endif
	idx++;

	dbg_func_ptr_w[idx] = himax_sense_on_off_write;
	idx++;

	dbg_func_ptr_w[idx] = himax_debug_level_write;
	dbg_func_ptr_r[idx] = himax_debug_level_read;
	idx++;

#if defined(HX_TP_PROC_GUEST_INFO)
	dbg_func_ptr_w[idx] = himax_proc_guest_info_write;
	dbg_func_ptr_r[idx] = himax_proc_guest_info_read;
#endif
	idx++;

	dbg_func_ptr_w[idx] = himax_int_en_write;
	dbg_func_ptr_r[idx] = himax_int_en_read;
	idx++;

	dbg_func_ptr_w[idx] = himax_irq_info_write;
	dbg_func_ptr_r[idx] = himax_irq_info_read;
	idx++;

	dbg_func_ptr_r[idx] = himax_proc_register_read;
	dbg_func_ptr_w[idx] = himax_proc_register_write;
	idx++;

	dbg_func_ptr_r[idx] = himax_reset_read;
	dbg_func_ptr_w[idx] = himax_reset_write;
	idx++;

	dbg_func_ptr_r[idx] = himax_diag_arrange_read;
	dbg_func_ptr_w[idx] = himax_diag_arrange_write;
	idx++;

	dbg_func_ptr_w[idx] = himax_diag_cmd_write;
	idx++;

	dbg_func_ptr_w[idx] = himax_irq_dbg_cmd_write;
	idx++;

	dbg_func_ptr_r[idx] = himax_test_bus_read;
	idx++;

	dbg_func_ptr_w[idx] = himax_update_write;
	dbg_func_ptr_r[idx] = himax_update_read;
	idx++;

	dbg_func_ptr_w[idx] = himax_version_write;
	dbg_func_ptr_r[idx] = himax_version_read;
	idx++;

	dbg_func_ptr_r[idx] = himax_info_read;
	idx++;

	dbg_func_ptr_r[idx] = himax_pen_info_read;
	idx++;

	dbg_func_ptr_r[idx] = himax_list_cmd_read;
	idx++;

	dbg_func_ptr_w[idx] = himax_help_write;
	dbg_func_ptr_r[idx] = himax_help_read;

}

int himax_touch_proc_init(void)
{
	himax_proc_diag_dir = proc_mkdir(HIMAX_PROC_DIAG_FOLDER,
			hx_touch_proc_dir);

	if (himax_proc_diag_dir == NULL) {
		E(" %s: himax_proc_diag_dir file create failed!\n", __func__);
		return -ENOMEM;
	}

	himax_proc_stack_file = proc_create(HIMAX_PROC_STACK_FILE, 0444,
	  himax_proc_diag_dir, &himax_stack_ops);
	if (himax_proc_stack_file == NULL) {
		E(" %s: proc stack file create failed!\n", __func__);
		goto fail_2_1;
	}

	himax_proc_delta_file = proc_create(HIMAX_PROC_DELTA_FILE, 0444,
	  himax_proc_diag_dir, &himax_delta_ops);
	if (himax_proc_delta_file == NULL) {
		E(" %s: proc delta file create failed!\n", __func__);
		goto fail_2_2;
	}

	himax_proc_dc_file = proc_create(HIMAX_PROC_DC_FILE, 0444,
	  himax_proc_diag_dir, &himax_dc_ops);
	if (himax_proc_dc_file == NULL) {
		E(" %s: proc dc file create failed!\n", __func__);
		goto fail_2_3;
	}

	himax_proc_baseline_file = proc_create(HIMAX_PROC_BASELINE_FILE, 0444,
	  himax_proc_diag_dir, &himax_baseline_ops);
	if (himax_proc_baseline_file == NULL) {
		E(" %s: proc baseline file create failed!\n", __func__);
		goto fail_2_4;
	}

	himax_proc_debug_file = proc_create(HIMAX_PROC_DEBUG_FILE,
				0644, hx_touch_proc_dir,
				&himax_debug_ops);
	if (himax_proc_debug_file == NULL) {
		E(" %s: proc debug file create failed!\n", __func__);
		goto fail_3;
	}
	dbg_func_ptr_init();

	himax_proc_flash_dump_file = proc_create(HIMAX_PROC_FLASH_DUMP_FILE,
				0644, hx_touch_proc_dir,
				&himax_flash_dump_ops);
	if (himax_proc_flash_dump_file == NULL) {
		E(" %s: proc flash dump file create failed!\n", __func__);
		goto fail_4;
	}

	return 0;

fail_4: remove_proc_entry(HIMAX_PROC_DEBUG_FILE, hx_touch_proc_dir);
fail_3:	remove_proc_entry(HIMAX_PROC_BASELINE_FILE, himax_proc_diag_dir);
fail_2_4: remove_proc_entry(HIMAX_PROC_DC_FILE, himax_proc_diag_dir);
fail_2_3: remove_proc_entry(HIMAX_PROC_DELTA_FILE, himax_proc_diag_dir);
fail_2_2: remove_proc_entry(HIMAX_PROC_STACK_FILE, himax_proc_diag_dir);
fail_2_1: remove_proc_entry(HIMAX_PROC_DIAG_FOLDER, hx_touch_proc_dir);
	return -ENOMEM;
}

void himax_touch_proc_deinit(void)
{

	remove_proc_entry(HIMAX_PROC_FLASH_DUMP_FILE, hx_touch_proc_dir);
	remove_proc_entry(HIMAX_PROC_DEBUG_FILE, hx_touch_proc_dir);
	remove_proc_entry(HIMAX_PROC_BASELINE_FILE, himax_proc_diag_dir);
	remove_proc_entry(HIMAX_PROC_DC_FILE, himax_proc_diag_dir);
	remove_proc_entry(HIMAX_PROC_DELTA_FILE, himax_proc_diag_dir);
	remove_proc_entry(HIMAX_PROC_STACK_FILE, himax_proc_diag_dir);
	remove_proc_entry(HIMAX_PROC_DIAG_FOLDER, hx_touch_proc_dir);
}

static void hx_dbg_rst_rb(void)
{
	int i = 0;

	I("%s: Now reset ring buffer!\n", __func__);
	g_rb_frame.rawdata->index = 0;
	g_rb_frame.rawdata->cnt_update = 0;
	atomic_set(&(g_rb_frame.p_update), 0);
	atomic_set(&(g_rb_frame.p_output), 0);
	atomic_set(&(g_rb_frame.length), 0);
	for (i = 0; i < HX_RB_FRAME_SIZE; i++) {
		memset(g_rb_frame.rawdata[i].mutual,
			0x00,
			(sizeof(uint8_t) *
			(hx_s_ic_data->rx_num * hx_s_ic_data->tx_num)
			* 2));
		memset(g_rb_frame.rawdata[i].self,
			0x00,
			(sizeof(uint8_t) *
			(hx_s_ic_data->rx_num + hx_s_ic_data->tx_num)
			* 2));
		g_rb_frame.frame_idx[i] = 0;
	}
}

static uint32_t hx_debug_gen_dump_size(void)
{
	uint32_t isram_size = 0;
	uint32_t dsram_size = 0;
	uint32_t max = 0;
	int i;

	for (i = 0; i < hx_s_ic_data->isram_num; i++)
		isram_size += hx_s_ic_data->isram_sz[i];

	for (i = 0; i < hx_s_ic_data->dsram_num; i++)
		dsram_size += hx_s_ic_data->dsram_sz[i];

	max = hx_s_ic_data->flash_size;

	if (isram_size > max)
		max = isram_size;

	if (dsram_size > max)
		max = dsram_size;

	I("%s: dump max size = %d\n", __func__, max);

	return max;
}

int himax_debug_init(void)
{
	struct himax_ts_data *ts = hx_s_ts;
	int i = 0;

	I("%s:Enter\n", __func__);

	if (ts == NULL) {
		E("%s: ts struct is NULL\n", __func__);
		return -EPROBE_DEFER;
	}

	proc_reg_buf = kzalloc(128 * sizeof(uint8_t),
		GFP_KERNEL);
	if (proc_reg_buf == NULL) {
		E("%s: reg_read_data allocate failed\n", __func__);
		goto err_alloc_reg_read_data_fail;
	}

	hx_s_debug_data = kzalloc(sizeof(struct himax_debug), GFP_KERNEL);
	if (hx_s_debug_data == NULL) { /*Allocate debug data space*/
		E("%s: hx_s_debug_data allocate failed\n", __func__);
		goto err_alloc_debug_data_fail;
	}

	himax_himax_data_init();

	g_dump._buf_len = hx_debug_gen_dump_size();

	g_dump._buf = kcalloc(g_dump._buf_len,
		sizeof(uint8_t),
		GFP_KERNEL);
	if (g_dump._buf == NULL) {
		E("%s: dump buffer allocate fail failed\n", __func__);
		goto err_dump_buf_alloc_failed;
	}

	ts->dump_wq = create_singlethread_workqueue("himax_dump_wq");
	if (!ts->dump_wq) {
		E("%s: create flash workqueue failed\n", __func__);
		goto err_create_flash_dump_wq_failed;
	}
	INIT_WORK(&ts->dump_work, himax_ts_dump_work_func);
	g_dump._progress = HX_DUMP_START;

#if defined(HX_TP_PROC_GUEST_INFO)
	if (g_guest_info_data == NULL) {
		g_guest_info_data = kzalloc(sizeof(struct hx_guest_info),
				GFP_KERNEL);
		if (g_guest_info_data == NULL) {
			E("%s: flash buffer allocate fail failed\n", __func__);
			goto err_guest_info_alloc_failed;
		}
		g_guest_info_data->g_guest_info_ongoing = 0;
		g_guest_info_data->g_guest_info_type = 0;
	}

	ts->guest_info_wq =
		create_singlethread_workqueue("himax_guest_info_wq");
	if (!ts->guest_info_wq) {
		E("%s: create guest info workqueue failed\n", __func__);
		goto err_create_guest_info_wq_failed;
	}
	INIT_WORK(&ts->guest_info_work, himax_ts_guest_info_work_func);
#endif

	setSelfBuffer(hx_s_ic_data->rx_num, hx_s_ic_data->tx_num);
	if (getSelfBuffer() == NULL) {
		E("%s: self buffer allocate failed\n", __func__);
		goto err_self_buf_alloc_failed;
	}

	setSelfBufferByte(hx_s_ic_data->rx_num, hx_s_ic_data->tx_num);
	if (getSelfBufferByte() == NULL) {
		E("%s: self buffer allocate failed\n", __func__);
		goto err_self_byte_buf_alloc_failed;
	}

	setSelfNewBuffer(hx_s_ic_data->rx_num, hx_s_ic_data->tx_num);
	if (getSelfNewBuffer() == NULL) {
		E("%s: self new buffer allocate failed\n", __func__);
		goto err_self_new_alloc_failed;
	}

	setSelfOldBuffer(hx_s_ic_data->rx_num, hx_s_ic_data->tx_num);
	if (getSelfOldBuffer() == NULL) {
		E("%s: self old buffer allocate failed\n", __func__);
		goto err_self_old_alloc_failed;
	}

	setMutualBuffer(hx_s_ic_data->rx_num, hx_s_ic_data->tx_num);
	if (getMutualBuffer() == NULL) {
		E("%s: mutual buffer allocate failed\n", __func__);
		goto err_mut_buf_alloc_failed;
	}

	setMutualBufferByte(hx_s_ic_data->rx_num, hx_s_ic_data->tx_num);
	if (getMutualBufferByte() == NULL) {
		E("%s: mutual buffer allocate failed\n", __func__);
		goto err_mut_byte_buf_alloc_failed;
	}

	setMutualNewBuffer(hx_s_ic_data->rx_num, hx_s_ic_data->tx_num);
	if (getMutualNewBuffer() == NULL) {
		E("%s: mutual new buffer allocate failed\n", __func__);
		goto err_mut_new_alloc_failed;
	}

	setMutualOldBuffer(hx_s_ic_data->rx_num, hx_s_ic_data->tx_num);
	if (getMutualOldBuffer() == NULL) {
		E("%s: mutual old buffer allocate failed\n", __func__);
		goto err_mut_old_alloc_failed;
	}

#if defined(HX_TP_PROC_2T2R)
	if (Is_2T2R) {
		setMutualBuffer_2(hx_s_ic_data->rx_num_2,
			hx_s_ic_data->tx_num_2);
		if (getMutualBuffer_2() == NULL) {
			E("%s: mutual buffer 2 allocate failed\n", __func__);
			goto err_mut_buf2_alloc_failed;
		}
	}
#endif

	ts->himax_diag_wq = create_singlethread_workqueue("himax_diag");
	if (!ts->himax_diag_wq) {
		E("%s: create diag workqueue failed\n", __func__);
		goto err_create_diag_wq_failed;
	}
	INIT_DELAYED_WORK(&ts->himax_diag_delay_work, himax_ts_diag_work_func);

	if (himax_touch_proc_init())
		goto err_proc_init_failed;


	snprintf(g_file_path, (int)(strlen(HX_RSLT_OUT_PATH)
		+ strlen(HX_RSLT_OUT_FILE)+1),
		"%s%s", HX_RSLT_OUT_PATH, HX_RSLT_OUT_FILE);

	g_rb_frame.rawdata = kzalloc(
		sizeof(struct frame_data)
		* HX_RB_FRAME_SIZE,
		GFP_KERNEL);
	g_rb_frame.rawdata->index = 0;
	g_rb_frame.rawdata->cnt_update = 0;
	atomic_set(&(g_rb_frame.p_update), 0);
	atomic_set(&(g_rb_frame.p_output), 0);
	atomic_set(&(g_rb_frame.length), 0);
	for (i = 0; i < HX_RB_FRAME_SIZE; i++) {
		g_rb_frame.rawdata[i].mutual = kzalloc(
			sizeof(uint8_t) *
			(hx_s_ic_data->rx_num * hx_s_ic_data->tx_num)
			* 2,
			GFP_KERNEL);
		g_rb_frame.rawdata[i].self = kzalloc(
			sizeof(uint8_t) *
			(hx_s_ic_data->rx_num + hx_s_ic_data->tx_num)
			* 2,
			GFP_KERNEL);
	}

	return 0;

err_proc_init_failed:
	cancel_delayed_work_sync(&ts->himax_diag_delay_work);
	destroy_workqueue(ts->himax_diag_wq);
err_create_diag_wq_failed:
#if defined(HX_TP_PROC_2T2R)
	kfree(diag_mutual_2);
	diag_mutual_2 = NULL;
err_mut_buf2_alloc_failed:
#endif
	kfree(diag_mutual_old);
	diag_mutual_old = NULL;
err_mut_old_alloc_failed:
	kfree(diag_mutual_new);
	diag_mutual_new = NULL;
err_mut_new_alloc_failed:
	kfree(diag_mutual_byte);
	diag_mutual_byte = NULL;
err_mut_byte_buf_alloc_failed:
	kfree(diag_mutual);
	diag_mutual = NULL;
err_mut_buf_alloc_failed:
	kfree(diag_self_old);
	diag_self_old = NULL;
err_self_old_alloc_failed:
	kfree(diag_self_new);
	diag_self_new = NULL;
err_self_new_alloc_failed:
	kfree(diag_self_byte);
	diag_self_byte = NULL;
err_self_byte_buf_alloc_failed:
	kfree(diag_self);
	diag_self = NULL;
err_self_buf_alloc_failed:
#if defined(HX_TP_PROC_GUEST_INFO)
	cancel_work_sync(&ts->guest_info_work);
	destroy_workqueue(ts->guest_info_wq);
err_create_guest_info_wq_failed:
	if (g_guest_info_data != NULL) {
		kfree(g_guest_info_data);
		g_guest_info_data = NULL;
	}
err_guest_info_alloc_failed:
#endif
	cancel_work_sync(&ts->dump_work);
	destroy_workqueue(ts->dump_wq);
err_create_flash_dump_wq_failed:
	kfree(g_dump._buf);
	g_dump._buf = NULL;
err_dump_buf_alloc_failed:
	kfree(hx_s_debug_data);
	hx_s_debug_data = NULL;
err_alloc_debug_data_fail:
	kfree(proc_reg_buf);
	proc_reg_buf = NULL;
err_alloc_reg_read_data_fail:

	return -ENOMEM;
}
EXPORT_SYMBOL(himax_debug_init);

int himax_debug_remove(void)
{
	struct himax_ts_data *ts = hx_s_ts;

	himax_touch_proc_deinit();

	cancel_delayed_work_sync(&ts->himax_diag_delay_work);
	destroy_workqueue(ts->himax_diag_wq);

#if defined(HX_TP_PROC_2T2R)
	kfree(diag_mutual_2);
	diag_mutual_2 = NULL;
#endif

	kfree(diag_mutual_byte);
	diag_mutual_byte = NULL;

	kfree(diag_mutual_old);
	diag_mutual_old = NULL;

	kfree(diag_mutual_new);
	diag_mutual_new = NULL;

	kfree(diag_mutual);
	diag_mutual = NULL;

	kfree(diag_self_old);
	diag_self_old = NULL;

	kfree(diag_self_new);
	diag_self_new = NULL;

	kfree(diag_self_byte);
	diag_self_byte = NULL;

	kfree(diag_self);
	diag_self = NULL;

#if defined(HX_TP_PROC_GUEST_INFO)
	cancel_work_sync(&ts->guest_info_work);
	destroy_workqueue(ts->guest_info_wq);
	if (g_guest_info_data != NULL) {
		kfree(g_guest_info_data);
		g_guest_info_data = NULL;
	}
#endif

	cancel_work_sync(&ts->dump_work);
	destroy_workqueue(ts->dump_wq);

	kfree(g_dump._buf);
	g_dump._buf = NULL;

	kfree(hx_s_debug_data);
	hx_s_debug_data = NULL;

	kfree(proc_reg_buf);
	proc_reg_buf = NULL;

	return 0;
}
EXPORT_SYMBOL(himax_debug_remove);

