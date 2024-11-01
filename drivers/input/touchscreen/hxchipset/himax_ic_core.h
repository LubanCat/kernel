/* SPDX-License-Identifier: GPL-2.0 */
/*  Himax Android Driver Sample Code for ic core functions
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

#ifndef __HIMAX_IC_CORE_H__
#define __HIMAX_IC_CORE_H__

#include "himax_platform.h"
#include "himax_common.h"
#include <linux/slab.h>


#define Arr4_to_Arr4(A, B) {\
	A[3] = B[3];\
	A[2] = B[2];\
	A[1] = B[1];\
	A[0] = B[0];\
	}



#define DATA_LEN_8				8
#define DATA_LEN_4				4
#define DATA_LEN_2				2
#define DATA_LEN_1				1
#define FLASH_RW_MAX_LEN		256
#define FLASH_WRITE_BURST_SZ	8
#define MAX_I2C_TRANS_SZ		128
#define HIMAX_REG_RETRY_TIMES	5
#define FW_BIN_16K_SZ			0x4000
#define HIMAX_TOUCH_DATA_SIZE	128
#define MASK_BIT_0				0x01
#define MASK_BIT_1				0x02
#define MASK_BIT_2				0x04

#define FW_SECTOR_PER_BLOCK		8
#define FW_PAGE_PER_SECTOR		64
#define FW_PAGE_SZ			128
#define HX256B					0x100
#define HX1K					0x400
#define HX4K					0x1000
#define HX_32K_SZ				0x8000
#define HX_40K_SZ				0xA000
#define HX_48K_SZ				0xC000
#define HX64K					0x10000
#define HX124K					0x1f000
#define HX4000K					0x1000000

#define HX_NORMAL_MODE			1
#define HX_SORTING_MODE			2
#define HX_CHANGE_MODE_FAIL		(-1)
#define HX_RW_REG_FAIL			(-1)
#define HX_DRIVER_MAX_IC_NUM	12

#if defined(__HIMAX_HX852xJ_MOD__)
#define HX_MOD_KSYM_HX852xJ HX_MOD_KSYM_HX852xJ
#endif

#if defined(__HIMAX_HX83102J_MOD__)
#define HX_MOD_KSYM_HX83102J HX_MOD_KSYM_HX83102J
#endif

#if defined(__HIMAX_HX83108_MOD__)
#define HX_MOD_KSYM_HX83108 HX_MOD_KSYM_HX83108
#endif

#if defined(__HIMAX_HX83112F_MOD__)
#define HX_MOD_KSYM_HX83112F HX_MOD_KSYM_HX83112F
#endif


#if defined(__HIMAX_HX83112_MOD__)
#define HX_MOD_KSYM_HX83112 HX_MOD_KSYM_HX83112
#endif

#if defined(__HIMAX_HX83121A_MOD__)
#define HX_MOD_KSYM_HX83121A HX_MOD_KSYM_HX83121A
#endif

/* CORE_INIT */
/* CORE_IC */
/* CORE_FW */
/* CORE_FLASH */
/* CORE_SRAM */
/* CORE_DRIVER */

#define HX_0F_DEBUG

#if defined(HX_TP_PROC_GUEST_INFO)
extern struct hx_guest_info *g_guest_info_data;
#endif

void himax_mcu_in_cmd_struct_free(void);

#if defined(HX_BOOT_UPGRADE) || defined(HX_ZERO_FLASH)
extern int g_i_FW_VER;
extern int g_i_CFG_VER;
extern int g_i_CID_MAJ;
extern int g_i_CID_MIN;
extern const struct firmware *hxfw;
#if defined(HX_ZERO_FLASH)
extern int g_f_0f_updat;
#endif
#endif

extern struct himax_debug *hx_s_debug_data;

extern u8 g_hw_rst_activate;
#if defined(HX_RST_PIN_FUNC)
void himax_rst_gpio_set(int pinnum, uint8_t value);
#endif

#if defined(HX_USB_DETECT_GLOBAL)
void himax_cable_detect_func(bool force_renew);
#endif

int himax_report_data_init(void);
extern int i2c_error_count;

#if defined(HX_EXCP_RECOVERY)
extern u8 HX_EXCP_RESET_ACTIVATE;
#endif

/* CORE_INIT */
int himax_mcu_in_cmd_struct_init(void);
void himax_mcu_in_cmd_init(void);

int himax_mcu_on_cmd_struct_init(void);
void himax_mcu_on_cmd_init(void);

/* CORE_INIT */

enum HX_FLASH_SPEED_ENUM {
	HX_FLASH_SPEED_25M = 0,
	HX_FLASH_SPEED_12p5M = 0x01,
	HX_FLASH_SPEED_8p3M = 0x02,
	HX_FLASH_SPEED_6p25M = 0x03,
};

#if defined(HX_TP_PROC_GUEST_INFO)
#define HX_GUEST_INFO_FLASH_SADDR 0x20000
#define HX_GUEST_INFO_SIZE	10
#define HX_GUEST_INFO_LEN_SIZE	4
#define HX_GUEST_INFO_ID_SIZE	4

struct hx_guest_info {
	int g_guest_info_ongoing; /* 0 stop, 1 ongoing */
	uint8_t g_guest_str[10][128];
	uint8_t g_guest_str_in_format[10][128];
	uint8_t g_guest_data_type[10];
	int g_guest_data_len[10];
	int g_guest_info_type;
};
#endif



struct ic_setup_collect {

	uint32_t _addr_psl;
	uint32_t _addr_cs_central_state;

	uint32_t _func_hsen;
	uint32_t _func_smwp;
	uint32_t _func_headphone;
	uint32_t _func_usb_detect;
	uint32_t _func_ap_notify_fw_sus;
	uint32_t _func_en;
	uint32_t _func_dis;
	uint32_t _data_handshaking_end;

	uint32_t _addr_system_reset;
	uint32_t _addr_leave_safe_mode;
	uint32_t _addr_reload_to_active;
	uint32_t _addr_ctrl_fw;
	uint32_t _addr_flag_reset_event;
	uint32_t _addr_program_reload_from;
	uint32_t _addr_program_reload_to;
	uint32_t _addr_program_reload_page_write;
	uint32_t _addr_rawout_sel;
	uint32_t _addr_reload_status;
	uint32_t _addr_reload_crc32_result;
	uint32_t _addr_reload_addr_from;
	uint32_t _addr_reload_addr_cmd_beat;
	uint32_t _data_system_reset;
	uint32_t _data_leave_safe_mode;
	uint32_t _data_reload_to_active;
	uint32_t _data_fw_stop;
	uint32_t _data_program_reload_start;
	uint32_t _data_program_reload_compare;
	uint32_t _data_program_reload_break;
	uint32_t _addr_set_frame;
	uint32_t _data_set_frame;
	uint8_t _para_idle_dis;
	uint8_t _para_idle_en;
	uint32_t _addr_sorting_mode_en;
	uint32_t _addr_fw_mode_status;
	uint32_t _addr_fw_ver;
	uint32_t _addr_fw_cfg;
	uint32_t _addr_fw_vendor;
	uint32_t _addr_cus_info;
	uint32_t _addr_proj_info;
	uint32_t _addr_fw_state;
	uint32_t _addr_fw_dbg_msg;
	uint32_t _addr_chk_fw_status;
	uint32_t _addr_chk_dd_status;


	uint8_t _data_rawdata_ready_hb;
	uint8_t _data_rawdata_ready_lb;
	uint8_t _addr_ahb;
	uint8_t _data_ahb_dis;
	uint8_t _data_ahb_en;
	uint8_t _addr_event_stack;

	uint8_t	_addr_ulpm_33;
	uint8_t	_addr_ulpm_34;
	uint8_t	_data_ulpm_11;
	uint8_t	_data_ulpm_22;
	uint8_t	_data_ulpm_33;
	uint8_t	_data_ulpm_aa;
	uint32_t _addr_ctrl_mpap_ovl;
	uint32_t _data_ctrl_mpap_ovl_on;

	uint32_t _addr_flash_spi200_trans_fmt;
	uint32_t _addr_flash_spi200_trans_ctrl;
	uint32_t _addr_flash_spi200_cmd;
	uint32_t _addr_flash_spi200_addr;
	uint32_t _addr_flash_spi200_data;
	uint32_t _addr_flash_spi200_flash_speed;
	uint32_t _data_flash_spi200_txfifo_rst;
	uint32_t _data_flash_spi200_rxfifo_rst;
	uint32_t _data_flash_spi200_trans_fmt;
	uint32_t _data_flash_spi200_trans_ctrl_1;
	uint32_t _data_flash_spi200_trans_ctrl_2;
	uint32_t _data_flash_spi200_trans_ctrl_3;
	uint32_t _data_flash_spi200_trans_ctrl_4;
	uint32_t _data_flash_spi200_trans_ctrl_5;
	uint32_t _data_flash_spi200_trans_ctrl_6;
	uint32_t _data_flash_spi200_trans_ctrl_7;
	uint32_t _data_flash_spi200_cmd_1;
	uint32_t _data_flash_spi200_cmd_2;
	uint32_t _data_flash_spi200_cmd_3;
	uint32_t _data_flash_spi200_cmd_4;
	uint32_t _data_flash_spi200_cmd_5;
	uint32_t _data_flash_spi200_cmd_6;
	uint32_t _data_flash_spi200_cmd_7;
	uint32_t _data_flash_spi200_cmd_8;


	uint32_t _addr_mkey;
	uint32_t _addr_rawdata_buf;
	uint32_t _data_rawdata_end;
	uint32_t _pwd_get_rawdata_start;
	uint32_t _pwd_get_rawdata_end;

	uint32_t _addr_chk_fw_reload;
	uint32_t _addr_chk_fw_reload2;
	uint32_t _data_fw_reload_dis;
	uint32_t _data_fw_reload_en;
	uint32_t _addr_chk_irq_edge;
	uint32_t _addr_info_channel_num;
	uint32_t _addr_info_max_pt;
	uint32_t _addr_info_def_stylus;
	uint32_t _addr_info_stylus_ratio;

	int _ovl_section_num;
	uint32_t _ovl_addr_handshake;
	uint8_t _ovl_gesture_request;
	uint8_t _ovl_gesture_reply;
	uint8_t _ovl_border_request;
	uint8_t _ovl_border_reply;
	uint8_t _ovl_sorting_request;
	uint8_t _ovl_sorting_reply;
	uint8_t _ovl_fault;
	uint8_t _ovl_alg_request;
	uint8_t _ovl_alg_reply;
	uint8_t _ovl_alg_fault;


	uint32_t _addr_normal_noise_thx;
	uint32_t _addr_lpwug_noise_thx;
	uint32_t _addr_noise_scale;
	uint32_t _addr_recal_thx;
	uint32_t _addr_palm_num;
	uint32_t _addr_weight_sup;
	uint32_t _addr_normal_weight_a;
	uint32_t _addr_lpwug_weight_a;
	uint32_t _addr_weight_b;
	uint32_t _addr_max_dc;
	uint32_t _addr_skip_frame;
	uint32_t _addr_neg_noise_sup;
	uint32_t _data_neg_noise;

};

extern struct ic_setup_collect hx_s_ic_setup;

enum AHB_Interface_Command_Table {
	ADDR_AHB_ADDRESS_BYTE_0		=	0x00,
	ADDR_AHB_RDATA_BYTE_0		=	0x08,
	ADDR_AHB_ACCESS_DIRECTION	=	0x0C,
	ADDR_AHB_CONTINOUS			=	0x13,
	ADDR_AHB_INC4				=	0x0D,
	ADDR_SENSE_ON_OFF_0			=	0x31,
	ADDR_SENSE_ON_OFF_1			=	0x32,
	ADDR_READ_EVENT_STACK		=	0x30,
	PARA_AHB_ACCESS_DIRECTION_READ	=	0x00,
	PARA_AHB_CONTINOUS			=	0x31,
	PARA_AHB_INC4				=	0x10,
	PARA_SENSE_OFF_0			=	0x27,
	PARA_SENSE_OFF_1			=	0x95,
	ADDR_TCON_ON_RST            = 0x80020020,
	ADDR_ADC_ON_RST             = 0x80020094,
};
/* CORE_IC */
	#define ADDR_PSL                       0x900000A0
	#define ADDR_CS_CENTRAL_STATE          0x900000A8
/* CORE_IC */

	#define FUNC_HSEN                 0x10007F14
	#define FUNC_SMWP                 0x10007F10
	#define FUNC_HEADPHONE            0x10007FE8
	#define FUNC_USB_DETECT           0x10007F38
	#define FUNC_AP_NOTIFY_FW_SUS     0x10007FD0
	#define FUNC_EN         0xA55AA55A
	#define FUNC_DIS        0x00000000
	#define DATA_HANDSHAKING_END             0x77887788

/* CORE_FW */
	#define ADDR_SYSTEM_RESET                0x90000018
	#define ADDR_LEAVE_SAFE_MODE             0x90000098
	#define ADDR_RELOAD_TO_ACTIVE            0x90000048
	#define ADDR_CTRL_FW                     0x9000005c
	#define ADDR_FLAG_RESET_EVENT            0x900000e4

	#define ADDR_PROGRAM_RELOAD_FROM         0x00000000
	#define ADDR_PROGRAM_RELOAD_TO           0x08000000
	#define ADDR_PROGRAM_RELOAD_PAGE_WRITE   0x0000fb00
	#define ADDR_RAWOUT_SEL                  0x800204b4
	#define ADDR_RELOAD_STATUS               0x80050000
	#define ADDR_RELOAD_CRC32_RESULT         0x80050018
	#define ADDR_RELOAD_ADDR_FROM            0x80050020
	#define ADDR_RELOAD_ADDR_CMD_BEAT        0x80050028
	#define DATA_SYSTEM_RESET                0x00000055
	#define DATA_LEAVE_SAFE_MODE             0x00000053
	#define DATA_RELOAD_TO_ACTIVE            0x000000EC
	#define DATA_CLEAR                       0x00000000
	#define DATA_FW_STOP                     0x000000A5
	#define DATA_PROGRAM_RELOAD_START        0x0A3C3000
	#define DATA_PROGRAM_RELOAD_COMPARE      0x04663000
	#define DATA_PROGRAM_RELOAD_BREAK        0x15E75678

	#define ADDR_SET_FRAME                    0x10007294
	#define DATA_SET_FRAME                    0x0000000A

	#define PARA_IDLE_DIS                0x17
	#define PARA_IDLE_EN                 0x1f
	#define ADDR_SORTING_MODE_EN             0x10007f04
	#define ADDR_FW_MODE_STATUS              0x10007088
	#define ADDR_FW_VER                 0x10007004
	#define ADDR_FW_CFG                 0x10007084
	#define ADDR_FW_VENDOR              0x10007000
	#define ADDR_CUS_INFO                    0x10007008
	#define ADDR_PROJ_INFO                   0x10007014
	#define ADDR_FW_STATE               0x900000f8
	#define ADDR_FW_DBG_MSG             0x10007f40
	#define ADDR_CHK_FW_STATUS               0x900000a8
	#define ADDR_CHK_DD_STATUS               0x900000E8

	#define DATA_RAWDATA_READY_HB            0xa3
	#define DATA_RAWDATA_READY_LB            0x3a
	#define ADDR_AHB                    0x11
	#define DATA_AHB_DIS                     0x00
	#define DATA_AHB_EN                      0x01
	#define ADDR_EVENT_STACK                  0x30
	#define ADDR_ULPM_33                     0x33
	#define ADDR_ULPM_34                     0x34
	#define DATA_ULPM_11                     0x11
	#define DATA_ULPM_22                     0x22
	#define DATA_ULPM_33                     0x33
	#define DATA_ULPM_AA                     0xAA
	#define ADDR_CTRL_MPAP_OVL               0x100073EC
	#define DATA_CTRL_MPAP_OVL_ON            0x107380

/* CORE_FW */

/* CORE_FLASH */
	#define ADDR_FLASH_CTRL_BASE           0x80000000
	#define ADDR_FLASH_SPI200_TRANS_FMT    (ADDR_FLASH_CTRL_BASE + 0x10)
	#define ADDR_FLASH_SPI200_TRANS_CTRL   (ADDR_FLASH_CTRL_BASE + 0x20)
	#define ADDR_FLASH_SPI200_CMD          (ADDR_FLASH_CTRL_BASE + 0x24)
	#define ADDR_FLASH_SPI200_ADDR         (ADDR_FLASH_CTRL_BASE + 0x28)
	#define ADDR_FLASH_SPI200_DATA         (ADDR_FLASH_CTRL_BASE + 0x2c)
	#define ADDR_FLASH_SPI200_FLASH_SPEED  (ADDR_FLASH_CTRL_BASE + 0x40)
	#define ADDR_FLASH_SPI200_BT_NUM       (ADDR_FLASH_CTRL_BASE + 0xe8)
	#define DATA_FLASH_SPI200_TXFIFO_RST   0x00000004
	#define DATA_FLASH_SPI200_RXFIFO_RST   0x00000002
	#define DATA_FLASH_SPI200_TRANS_FMT    0x00020780
	#define DATA_FLASH_SPI200_TRANS_CTRL_1 0x42000003
	#define DATA_FLASH_SPI200_TRANS_CTRL_2 0x47000000
	#define DATA_FLASH_SPI200_TRANS_CTRL_3 0x67000000
	#define DATA_FLASH_SPI200_TRANS_CTRL_4 0x610ff000
	#define DATA_FLASH_SPI200_TRANS_CTRL_5 0x694002ff
	#define DATA_FLASH_SPI200_TRANS_CTRL_6 0x42000000
	#define DATA_FLASH_SPI200_TRANS_CTRL_7 0x6940020f
	#define DATA_FLASH_SPI200_CMD_1        0x00000005
	#define DATA_FLASH_SPI200_CMD_2        0x00000006
	#define DATA_FLASH_SPI200_CMD_3        0x000000C7
	#define DATA_FLASH_SPI200_CMD_4        0x000000D8
	#define DATA_FLASH_SPI200_CMD_5        0x00000020
	#define DATA_FLASH_SPI200_CMD_6        0x00000002
	#define DATA_FLASH_SPI200_CMD_7        0x0000003b
	#define DATA_FLASH_SPI200_CMD_8        0x00000003

/* CORE_FLASH */

/* CORE_SRAM */
	#define ADDR_MKEY         0x100070E8
	#define ADDR_RAWDATA_BUF 0x10000000
	#define DATA_RAWDATA_END  0x00000000
	#define	PWD_GET_RAWDATA_START    0x5AA5
	#define	PWD_GET_RAWDATA_END      0xA55A
/* CORE_SRAM */

/* CORE_DRIVER */
	#define ADDR_CHK_FW_RELOAD              0x10007f00
	#define ADDR_CHK_FW_RELOAD2          0x100072c0
	#define DATA_CHK_FW_RELOAD2			 0x000072c0
	#define DATA_FW_RELOAD_DIS          0x0000a55a
	#define DATA_FW_RELOAD_EN           0x00000000
	#define ADDR_CHK_IRQ_EDGE               0x10007088
	#define ADDR_INFO_CHANNEL_NUM               0x100070f4
	#define ADDR_INFO_MAX_PT               0x100070f8
	#define ADDR_INFO_DEF_STYLUS		   0x1000719c
	#define ADDR_INFO_STYLUS_RATIO		   0x100071fc

/* CORE_DRIVER */

#if defined(HX_ZERO_FLASH)
	#define DATA_ZF_FW_RELOAD_DIS 0x00009AA9

	#define OVL_ADDR_HANDSHAKE   0x10007ffc
	#define OVL_SECTION_NUM      3
	#define OVL_GESTURE_REQUEST  0x11
	#define OVL_GESTURE_REPLY    0x22
	#define OVL_BORDER_REQUEST   0x55
	#define OVL_BORDER_REPLY     0x66
	#define OVL_SORTING_REQUEST  0x99
	#define OVL_SORTING_REPLY    0xAA
	#define OVL_FAULT            0xFF


	#define OVL_ALG_REQUEST  0x11
	#define OVL_ALG_REPLY    0x22
	#define OVL_ALG_FAULT    0xFF


struct zf_info {
	uint8_t sram_addr[4];
	int write_size;
	uint32_t fw_addr;
	uint32_t cfg_addr;
};
enum opt_crc_setting {
	NO_OPT_CRC = 0,
	OPT_CRC_SWITCH = 1,
	OPT_CRC_RANGE = 2,
	OPT_CRC_BOTH = 3,
};
enum opt_crc_status {
	OPT_CRC_START_ADDR = 0x01,
	OPT_CRC_END_ADDR = 0x02,
	OPT_CRC_SWITCH_ADDR = 0x03,
};
struct zf_opt_crc {
	/* 0:off, 1:only switch, 2: only range, 3: both range and switch*/
	int en_opt_hw_crc;
	bool en_crc_clear;
	bool en_start_addr;
	bool en_end_addr;
	bool en_switch_addr;
	uint32_t fw_addr;
	uint32_t start_addr;
	uint8_t start_data[4];
	uint32_t end_addr;
	uint8_t end_data[4];
	uint32_t switch_addr;
	uint8_t switch_data[4];
};
#endif


/*Inspection register*/
#define ADDR_NORMAL_NOISE_THX   0X1000708C
#define ADDR_LPWUG_NOISE_THX    0X10007090
#define ADDR_NOISE_SCALE        0X10007094
#define ADDR_RECAL_THX          0X10007090
#define ADDR_PALM_NUM           0X100070A8
#define ADDR_WEIGHT_SUP         0X100072C8
#define ADDR_NORMAL_WEIGHT_A    0X1000709C
#define ADDR_LPWUG_WEIGHT_A     0X100070A0
#define ADDR_WEIGHT_B           0X10007094
#define ADDR_MAX_DC             0X10007FC8
#define ADDR_SKIP_FRAME         0X100070F4
#define ADDR_NEG_NOISE_SUP      0X10007FD8
#define DATA_NEG_NOISE          0X7F0C0000


/* New Version 1K*/
enum bin_desc_map_table {
	TP_CONFIG_HW_OPT_CRC = 0x00000A01,
	TP_CONFIG_TABLE = 0x00000A00,
	FW_CID = 0x10000000,
	FW_VER = 0x10000100,
	CFG_VER = 0x10000600,
};

/*Old Version 1K
 *enum bin_desc_map_table {
 *TP_CONFIG_TABLE = 0x0000000A,
 *FW_CID = 0x10000000,
 *FW_VER = 0x10000001,
 *CFG_VER = 0x10000005,
 *};
 **/

struct himax_core_fp {
/* CORE_IC */
	void (*_burst_enable)(uint8_t auto_add_4_byte);
	int (*_register_read)(uint32_t addr, uint8_t *buf, uint32_t len);
	/*int (*_flash_write_burst)(uint8_t *reg_byte, uint8_t *write_data);*/
	/*void (*_flash_write_burst_lenth)(uint8_t *reg_byte,
	 *		uint8_t *write_data, uint32_t length);
	 */
	int (*_register_write)(uint32_t addr, uint8_t *val, uint32_t len);
	void (*_interface_on)(void);
	void (*_sense_on)(uint8_t FlashMode);
	void (*_leave_safe_mode)(void);
	void (*_tcon_on)(void);
	bool (*_watch_dog_off)(void);
	bool (*_sense_off)(bool check_en);
	void (*_sleep_in)(void);
	bool (*_wait_wip)(int Timing);
	void (*_init_psl)(void);
	void (*_resume_ic_action)(void);
	void (*_suspend_ic_action)(void);
	int (*_power_on_init)(void);
	bool (*_slave_tcon_reset)(void);
	bool (*_slave_adc_reset_slave)(void);
	bool (*_slave_wdt_off_slave)(void);
/* CORE_IC */

/* CORE_FW */
	void (*_parse_raw_data)(struct himax_report_data *hx_s_touch_data,
			int mul_num,
			int self_num,
			uint8_t diag_cmd,
			int16_t *mutual_data,
			int16_t *self_data);
	void (*_system_reset)(void);
	int (*_calc_crc_by_ap)(const uint8_t *fw_content,
		int crc_from_fw,
			int len);
	uint32_t (*_calc_crc_by_hw)(uint8_t *start_addr, int reload_length);
	void (*_set_reload_cmd)(uint8_t *write_data,
			int idx,
			uint32_t cmd_from,
			uint32_t cmd_to,
			uint32_t cmd_beat);
	bool (*_program_reload)(void);
	void (*_set_SMWP_enable)(uint8_t SMWP_enable, bool suspended);
	void (*_set_HSEN_enable)(uint8_t HSEN_enable, bool suspended);
	void (*_set_headphone_en)(uint8_t en, bool suspended);
	void (*_diag_register_set)(uint8_t diag_command,
			uint8_t storage_type, bool is_dirly);
	void (*_ap_notify_fw_sus)(int suspend);
#if defined(HX_TP_SELF_TEST_DRIVER)
	void (*_control_reK)(bool enable);
#endif
	int (*_chip_self_test)(struct seq_file *s, void *v);
	void (*_idle_mode)(int disable);
	void (*_reload_disable)(int disable);
	int (*_read_ic_trigger_type)(void);
	int (*_read_i2c_status)(void);
	void (*_read_FW_ver)(void);
	void (*_show_FW_ver)(void);
	bool (*_read_event_stack)(uint8_t *buf, uint32_t length);
	void (*_return_event_stack)(void);
	bool (*_calculateChecksum)(bool change_iref, uint32_t size);
	void (*_read_FW_status)(void);
	void (*_irq_switch)(int switch_on);
	int (*_assign_sorting_mode)(uint8_t *tmp_data);
	int (*_check_sorting_mode)(uint8_t *tmp_data);
	int (*_get_max_dc)(void);
	int (*_ulpm_in)(void);
	int (*_black_gest_ctrl)(bool enable);
	int	(*_diff_overlay_bin)(void);
/* CORE_FW */

/* CORE_FLASH */
	void (*_chip_erase)(void);
	bool (*_block_erase)(int start_addr, int length);
	bool (*_sector_erase)(int start_addr);
	bool (*_flash_programming)(uint8_t *FW_content, int FW_Size);
	void (*_flash_page_write)(uint8_t *write_addr, int length,
			uint8_t *write_data);
	int (*_fts_ctpm_fw_upgrade_with_sys_fs)(unsigned char *fw,
			int len, bool change_iref);
	void (*_flash_dump_func)(uint8_t local_flash_command,
			int Flash_Size, uint8_t *flash_buffer);
	bool (*_flash_lastdata_check)(uint32_t size);
	bool (*_bin_desc_data_get)(uint32_t addr, const uint8_t *flash_buf);
	bool (*_bin_desc_get)(unsigned char *fw, uint32_t max_sz);
	bool (*_ahb_squit)(void);
	void (*_flash_read)(uint8_t *r_data, int start_addr, int length);
	bool (*_sfr_rw)(uint8_t *addr, int length,
			uint8_t *data, uint8_t rw_ctrl);
	bool (*_lock_flash)(void);
	bool (*_unlock_flash)(void);
	void (*_init_auto_func)(void);
	int (*_diff_overlay_flash)(void);
/* CORE_FLASH */

/* CORE_SRAM */
	void (*_sram_write)(uint8_t *FW_content);
	bool (*_sram_verify)(uint8_t *FW_File, int FW_Size);
	int (*_write_read_reg)(uint32_t tmp_addr, uint8_t *tmp_data,
		uint8_t hb, uint8_t lb);
	bool (*_get_DSRAM_data)(uint8_t *info_data, bool DSRAM_Flag);
/* CORE_SRAM */

/* CORE_DRIVER */
	void (*_chip_init)(void);
	void (*_pin_reset)(int opt);
	uint8_t (*_tp_info_check)(void);
	void (*_touch_information)(void);
	void (*_calc_touch_data_size)(void);
	void (*_reload_config)(void);
	int (*_get_touch_data_size)(void);
	void (*_usb_detect_set)(const uint8_t *cable_config);
	int (*_hand_shaking)(void);
	int (*_cal_data_len)(int raw_cnt_rmd, int max_pt, int raw_cnt_max);
	bool (*_diag_check_sum)(struct himax_report_data *hx_s_touch_data);
	int (*_diag_parse_raw_data)(struct himax_report_data *hx_s_touch_data,
			int mul_num,
			int self_num,
			uint8_t diag_cmd,
			int32_t *mutual_data,
			int32_t *self_data,
			uint8_t *mutual_data_byte,
			uint8_t *self_data_byte);
	void (*_ic_reset)(int level);
	void (*_excp_hw_reset)(void);
	int (*_excp_event_chk)(struct himax_ts_data *ts,
		const uint8_t *buf, int ts_path, int ts_status, int length);
	int (*_ic_excp_recovery)(uint32_t hx_excp_event,
			uint32_t hx_zero_event, uint32_t length);
	void (*_excp_ic_reset)(void);
	void (*_resend_cmd_func)(bool suspended);
#if defined(HX_TP_PROC_GUEST_INFO)
	int (*guest_info_get_status)(void);
	int (*read_guest_info)(void);
#endif
/* CORE_DRIVER */
	int (*_turn_on_mp_func)(int on);
#if defined(HX_ZERO_FLASH)
	void (*_clean_sram_0f)(uint32_t addr, int write_len, int type);
	void (*_write_sram_0f)(const uint8_t *addr, const uint8_t *data,
		uint32_t len);
	int (*_write_sram_0f_crc)(uint8_t *addr, const uint8_t *data,
		uint32_t len);
	int (*_firmware_update_0f)(const struct firmware *fw_entry, int type);
	int (*_0f_op_file_dirly)(char *file_name);
	int (*_0f_excp_check)(void);
	void (*_0f_reload_to_active)(void);
	void (*_en_hw_crc)(int en);
#if defined(HX_0F_DEBUG)
	void (*_read_sram_0f)(const struct firmware *fw_entry,
			const uint8_t *addr,
			int start_index,
			int read_len);
	void (*_read_all_sram)(const uint8_t *addr, int read_len);
	void (*_firmware_read_0f)(const struct firmware *fw_entry, int type);
	void (*_setup_opt_hw_crc)(const struct firmware *fw);
	void (*_opt_crc_clear)(void);
#endif




#endif
	int (*_0f_overlay)(int ovl_type, int mode);
	void (*_suspend_proc)(bool suspended);
	void (*_resume_proc)(bool suspended);
/*Inspection*/
#if defined(CONFIG_TOUCHSCREEN_HIMAX_INSPECT)
	void (*_get_noise_base)(bool is_lpwup, int *rslt);
	void (*_neg_noise_sup)(uint8_t *data);
	int (*_get_noise_weight_test)(uint8_t checktype);
	uint16_t (*_get_palm_num)(void);
	int (*_switch_mode)(int mode);
	void (*_switch_data_type)(uint8_t checktype, char **str_mode);
	uint32_t (*_check_mode)(uint8_t checktype, char **str_mode);
	uint32_t (*_wait_sorting_mode)(uint8_t checktype, char **str_mode);
#endif
	void (*_cascade_cmd)(void);
};

extern int hx_mcu_reg_read(uint32_t addr_val, uint8_t *buf, int len);

extern void himax_mcu_interface_on(void);
extern bool himax_mcu_wait_wip(int Timing);
extern void himax_mcu_sense_on(uint8_t FlashMode);
extern void himax_mcu_init_psl(void);
extern void himax_mcu_resume_ic_action(void);
extern void himax_mcu_suspend_ic_action(void);
extern int himax_mcu_power_on_init(void);
extern void himax_mcu_system_reset(void);
extern void himax_mcu_leave_safe_mode(void);
extern uint32_t himax_mcu_calc_crc_by_hw(uint8_t *start_addr,
	int reload_length);
extern bool himax_mcu_program_reload(void);
extern int himax_mcu_ulpm_in(void);
extern int himax_mcu_black_gest_ctrl(bool enable);
extern void himax_mcu_set_SMWP_enable(uint8_t SMWP_enable, bool suspended);
extern void himax_mcu_set_HSEN_enable(uint8_t HSEN_enable, bool suspended);
extern void himax_mcu_set_headphone_en(uint8_t en, bool suspended);
extern void himax_mcu_usb_detect_set(const uint8_t *cable_config);
extern int himax_mcu_chip_self_test(struct seq_file *s, void *v);
extern void himax_mcu_idle_mode(int disable);
extern void himax_mcu_reload_disable(int disable);
extern int himax_mcu_read_ic_trigger_type(void);
extern int himax_mcu_read_i2c_status(void);
extern void himax_mcu_read_FW_ver(void);
extern void himax_mcu_show_FW_ver(void);
extern bool himax_mcu_read_event_stack(uint8_t *buf, uint32_t length);
extern void himax_mcu_return_event_stack(void);
extern bool himax_mcu_calculateChecksum(bool change_iref, uint32_t size);
extern void himax_mcu_read_FW_status(void);
extern void himax_mcu_irq_switch(int switch_on);
extern int himax_mcu_assign_sorting_mode(uint8_t *tmp_data);
extern int himax_mcu_check_sorting_mode(uint8_t *tmp_data);
extern void himax_get_noise_base(bool is_lpwup, int *rslt);
extern uint16_t himax_get_palm_num(void);
extern void hx_ap_notify_fw_sus(int suspend);
extern void himax_mcu_chip_erase(void);
extern bool himax_mcu_block_erase(int start_addr, int length);
extern bool himax_mcu_sector_erase(int start_addr);
extern bool himax_mcu_flash_programming(uint8_t *FW_content, int FW_Size);
extern bool himax_mcu_flash_lastdata_check(uint32_t size);
extern bool hx_bin_desc_data_get(uint32_t addr, const uint8_t *flash_buf);
extern bool hx_mcu_bin_desc_get(unsigned char *fw, uint32_t max_sz);
extern int hx_mcu_diff_overlay_flash(void);
extern void himax_mcu_sram_write(uint8_t *FW_content);
extern bool himax_mcu_sram_verify(uint8_t *FW_File, int FW_Size);
extern int himax_mcu_write_read_reg(uint32_t tmp_addr, uint8_t *tmp_data,
	uint8_t hb, uint8_t lb);
extern bool himax_mcu_get_DSRAM_data(uint8_t *info_data, bool DSRAM_Flag);
extern void himax_mcu_init_ic(void);
extern void himax_suspend_proc(bool suspended);
extern void himax_resume_proc(bool suspended);
extern void himax_mcu_pin_reset(int opt);
extern void himax_mcu_ic_reset(int level);
extern uint8_t himax_mcu_tp_info_check(void);
extern void himax_mcu_touch_information(void);
extern void himax_mcu_calcTouchDataSize(void);
extern int himax_mcu_get_touch_data_size(void);
extern int himax_mcu_hand_shaking(void);
extern bool himax_mcu_diag_check_sum(struct himax_report_data *hx_s_touch_data);
extern void himax_mcu_excp_ic_reset(void);
extern int himax_guest_info_get_status(void);
extern void himax_guest_info_set_status(int setting);
extern int hx_read_guest_info(void);
extern void himax_mcu_resend_cmd_func(bool suspended);
/* inspection functions*/
extern int hx_turn_on_mp_func(int on);
extern void himax_mcu_neg_noise_sup(uint8_t *data);
extern int himax_mcu_get_noise_weight_test(uint8_t checktype);
extern int himax_mcu_switch_mode_inspection(int mode);
extern void himax_mcu_switch_data_type(uint8_t checktype, char **str_mode);
extern uint32_t himax_mcu_check_mode(uint8_t checktype, char **str_mode);
extern uint32_t himax_mcu_wait_sorting_mode(uint8_t checktype, char **str_mode);
extern void hx_dis_rload_0f(int disable);
extern void himax_mcu_clean_sram_0f(uint32_t addr, int write_len, int type);
extern void himax_mcu_write_sram_0f(const uint8_t *addr,
	const uint8_t *data, uint32_t len);
extern int himax_sram_write_crc_check(uint8_t *addr,
	const uint8_t *data, uint32_t len);
extern int himax_mcu_firmware_update_0f(const struct firmware *fw, int type);
extern int hx_0f_op_file_dirly(char *file_name);
extern int himax_mcu_0f_excp_check(void);
extern void himax_mcu_read_all_sram(const uint8_t *addr, int read_len);
extern int himax_mcu_calc_crc_by_ap(const uint8_t *fw_content,
		int crc_from_fw, int len);
extern void himax_mcu_set_reload_cmd(uint8_t *write_data, int idx,
		uint32_t cmd_from, uint32_t cmd_to, uint32_t cmd_beat);
extern void himax_mcu_diag_register_set(uint8_t diag_command,
		uint8_t storage_type, bool is_dirly);
extern void himax_mcu_flash_page_write(uint8_t *write_addr, int length,
		uint8_t *write_data);
extern int himax_mcu_fts_ctpm_fw_upgrade_with_sys_fs(unsigned char *fw,
		int len, bool change_iref);
extern void himax_mcu_flash_dump_func(uint8_t local_flash_command,
		int Flash_Size, uint8_t *flash_buffer);
extern int himax_mcu_cal_data_len(int raw_cnt_rmd, int max_pt,
		int raw_cnt_max);
extern int himax_mcu_diag_parse_raw_data(
		struct himax_report_data *hx_s_touch_data,
		int mul_num, int self_num, uint8_t diag_cmd,
		int32_t *mutual_data, int32_t *self_data,
		uint8_t *mutual_data_byte,
		uint8_t *self_data_byte);
extern void himax_mcu_excp_hw_reset(void);
extern int himax_mcu_excp_event_chk(struct himax_ts_data *ts,
		const uint8_t *buf, int ts_path, int ts_status, int length);
extern int himax_mcu_ic_excp_recovery(uint32_t hx_excp_event,
		uint32_t hx_zero_event, uint32_t length);
extern void himax_mcu_read_sram_0f(const struct firmware *fw_entry,
		const uint8_t *addr, int start_index, int read_len);
extern void hx_mcu_setup_opt_hw_crc(const struct firmware *fw);
extern void hx_mcu_opt_hw_crc(void);


/* common functions */
#if defined(HX_FIRMWARE_HEADER)
extern void hx_get_embedded_fw(int type);
#endif
extern void himax_report_all_leave_event(struct himax_ts_data *ts);

#endif
