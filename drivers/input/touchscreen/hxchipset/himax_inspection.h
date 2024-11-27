/* SPDX-License-Identifier: GPL-2.0 */
/*  Himax Android Driver Sample Code for inspection functions
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

#ifndef H_HIMAX_INSPECTION
#define H_HIMAX_INSPECTION

#include "himax_platform.h"
#include "himax_common.h"
#include "himax_ic_core.h"
#if defined(HX_RW_FILE)
#include <linux/rtc.h>
#include <linux/time64.h>
#include <linux/timekeeping.h>
#endif
/*#define HX_GAP_TEST*/
/*#define HX_INSP_LP_TEST*/
/*#define HX_ACT_IDLE_TEST*/

/*#define HX_INSPT_DBG*/

#define HX_RSLT_OUT_PATH "/sdcard/"
#define HX_RSLT_OUT_FILE "hx_test_result.txt"
#define HX_SELF_RSLT_OUT_FILE "hx_selftest_"
#define PI(x...) pr_cont(x)
#define HX_SZ_ICID 60

#if defined(HX_EXCP_RECOVERY)
extern u8 HX_EXCP_RESET_ACTIVATE;
#endif

#define BS_RAWDATA     8
#define BS_NOISE       8
#define BS_OPENSHORT   0
#define	BS_LPWUG       1
#define	BS_LP_dile  1
#define	BS_ACT_IDLE    1

/* skip notch & dummy */
#define SKIP_NOTCH_START    5
#define SKIP_NOTCH_END      10
/* TX+SKIP_NOTCH_START */
#define SKIP_DUMMY_START    23
/* TX+SKIP_NOTCH_END*/
#define SKIP_DUMMY_END      28


#define	NOISEFRAME                      60
#define NORMAL_IDLE_RAWDATA_NOISEFRAME  10
#define LP_RAWDATAFRAME              1
#define LP_NOISEFRAME                1
#define LP_IDLE_RAWDATAFRAME         1
#define LP_IDLE_NOISEFRAME           1
#define PEN_MODE_RAWDATAFRAME        1

#define OTHERSFRAME		2

#define	UNIFMAX			500


/*Himax MP Password*/
#define	PWD_OPEN_START          0x77
#define	PWD_OPEN_END            0x88
#define	PWD_SHORT_START         0x11
#define	PWD_SHORT_END           0x33
#define	PWD_RAWDATA_START       0x00
#define	PWD_RAWDATA_END         0x99
#define	PWD_NOISE_START         0x00
#define	PWD_NOISE_END           0x99
#define	PWD_SORTING_START       0xAA
#define	PWD_SORTING_END         0xCC


#define PWD_ACT_IDLE_START      0x22
#define PWD_ACT_IDLE_END        0x44



#define PWD_LP_START         0x55
#define PWD_LP_END           0x66

#define PWD_LP_IDLE_START    0x50
#define PWD_LP_IDLE_END      0x60

#define PEN_MODE_START    0x64
#define PEN_MODE_END      0x74

/*Himax DataType*/
#define DATA_SORTING            0x0A
#define DATA_OPEN               0x0B
#define DATA_MICRO_OPEN         0x0C
#define DATA_SHORT              0x0A
#define DATA_RAWDATA            0x0A
#define DATA_NOISE              0x0F
#define DATA_BACK_NORMAL        0x00
#define DATA_LP_RAWDATA      0x0C
#define DATA_LP_NOISE        0x0F
#define DATA_ACT_IDLE_RAWDATA   0x0A
#define DATA_ACT_IDLE_NOISE     0x0F
#define DATA_LP_IDLE_RAWDATA 0x0A
#define DATA_LP_IDLE_NOISE   0x0F

#define DATA_PEN_MODE    0x11

/*Himax Data Ready Password*/
#define	Data_PWD0       0xA5
#define	Data_PWD1       0x5A

/* ASCII format */
#define ASCII_LF        (0x0A)
#define ASCII_CR        (0x0D)
#define ASCII_COMMA     (0x2C)
#define ASCII_ZERO      (0x30)
#define CHAR_EL         '\0'
#define CHAR_NL         '\n'
#define ACSII_SPACE     (0x20)
/* INSOECTION Setting */

void himax_inspection_init(void);
extern int HX_CRITERIA_ITEM;
extern int *g_test_item_flag;

// extern void himax_inspect_data_clear(void);

/*Need to map *g_hx_inspt_mode[]*/
enum THP_INSPECTION_ENUM {
	HX_OPEN,
	HX_SELF_OPEN,
	HX_MICRO_OPEN,
	HX_SHORT,
	HX_SELF_SHORT,
	HX_SC,
	HX_WT_NOISE,
	HX_SELF_WT_NOISE,
	HX_ABS_NOISE,
	HX_SELF_ABS_NOISE,
	HX_RAWDATA,
	HX_BPN_RAWDATA,
	HX_SELF_BPN_RAWDATA,
	HX_PEN_MODE_DATA,
	HX_SORTING,

	HX_GAPTEST_RAW,
	/*HX_GAPTEST_RAW_X,*/
	/*HX_GAPTEST_RAW_Y,*/

	HX_ACT_IDLE_NOISE,
	HX_ACT_IDLE_RAWDATA,
	HX_ACT_IDLE_BPN_RAWDATA,
/*LPWUG test must put after Normal test*/
	HX_LP_WT_NOISE,
	HX_LP_ABS_NOISE,
	HX_LP_RAWDATA,
	HX_LP_BPN_RAWDATA,

	HX_LP_IDLE_NOISE,
	HX_LP_IDLE_RAWDATA,
	HX_LP_IDLE_BPN_RAWDATA,

	HX_BACK_NORMAL,/*Must put in the end*/
};


enum HX_CRITERIA_ENUM {
	IDX_RAWMIN = 0,
	IDX_RAWMAX,
	IDX_BPN_RAWMIN,
	IDX_BPN_RAWMAX,
	IDX_SELF_BPN_RAWMIN,
	IDX_SELF_BPN_RAWMAX,
	IDX_SCMIN,
	IDX_SCMAX,
	IDX_SC_GOLDEN,
	IDX_SHORTMIN,
	IDX_SHORTMAX,
	IDX_SELF_SHORTMIN,
	IDX_SELF_SHORTMAX,
	IDX_OPENMIN,
	IDX_OPENMAX,
	IDX_SELF_OPENMIN,
	IDX_SELF_OPENMAX,
	IDX_M_OPENMIN,
	IDX_M_OPENMAX,
	IDX_WT_NOISEMIN,
	IDX_WT_NOISEMAX,
	IDX_SELF_WT_NOISEMIN,
	IDX_SELF_WT_NOISEMAX,
	IDX_ABS_NOISEMIN,
	IDX_ABS_NOISEMAX,
	IDX_SELF_ABS_NOISEMIN,
	IDX_SELF_ABS_NOISEMAX,
	IDX_PEN_MODE_DATAMIN,
	IDX_PEN_MODE_DATAMAX,
	IDX_SORTMIN,
	IDX_SORTMAX,

	IDX_GAP_HOR_RAWMAX,
	IDX_GAP_HOR_RAWMIN,
	IDX_GAP_VER_RAWMAX,
	IDX_GAP_VER_RAWMIN,

	IDX_ACT_IDLE_NOISE_MIN,
	IDX_ACT_IDLE_NOISE_MAX,
	IDX_ACT_IDLE_RAWDATA_MIN,
	IDX_ACT_IDLE_RAWDATA_MAX,
	IDX_ACT_IDLE_RAW_BPN_MIN,
	IDX_ACT_IDLE_RAW_BPN_MAX,

	IDX_LP_WT_NOISEMIN,
	IDX_LP_WT_NOISEMAX,
	IDX_LP_NOISE_ABS_MIN,
	IDX_LP_NOISE_ABS_MAX,
	IDX_LP_RAWDATA_MIN,
	IDX_LP_RAWDATA_MAX,
	IDX_LP_RAW_BPN_MIN,
	IDX_LP_RAW_BPN_MAX,

	IDX_LP_IDLE_NOISE_MIN,
	IDX_LP_IDLE_NOISE_MAX,
	IDX_LP_IDLE_RAWDATA_MIN,
	IDX_LP_IDLE_RAWDATA_MAX,
	IDX_LP_IDLE_RAW_BPN_MIN,
	IDX_LP_IDLE_RAW_BPN_MAX,
};

enum HX_INSPT_SETTING_IDX {
	RAW_BS_FRAME = 0,
	NOISE_BS_FRAME,
	ACT_IDLE_BS_FRAME,
	LP_BS_FRAME,
	LP_IDLE_BS_FRAME,

	NFRAME,
	IDLE_NFRAME,
	LP_RAW_NFRAME,
	LP_NOISE_NFRAME,
	LP_IDLE_RAW_NFRAME,
	LP_IDLE_NOISE_NFRAME,
};

enum HX_INSP_TEST_ERR_ENUM {
	/* OK */
	HX_INSP_OK = 0,

	/* FAIL */
	HX_INSP_FAIL = 1,

	/* Memory allocate errors */
	HX_INSP_MEMALLCTFAIL = 1 << 1,

	/* Abnormal screen state */
	HX_INSP_ESCREEN = 1 << 2,

	/* Out of specification */
	HX_INSP_ESPEC = 1 << 3,

	/* Criteria file error*/
	HX_INSP_EFILE = 1 << 4,

	/* Switch mode error*/
	HX_INSP_ESWITCHMODE = 1 << 5,

	/* Get raw data errors */
	HX_INSP_EGETRAW = 1 << 6,

	/* Update fw errors */
	HX_INSP_EUPDATE = 1 << 7,

};

/* Error code of Inspection */
enum HX_INSP_ITEM_ERR_ENUM {
	/* Sensor open error */
	HX_EOPEN = 1 << HX_OPEN,

	/* Sensor micro open error */
	HX_EMOPEN = 1 << HX_MICRO_OPEN,

	/* Sensor short error */
	HX_ESHORT = 1 << HX_SHORT,

	/* Raw data error */
	HX_ERAW = 1 << HX_RAWDATA,

	/* Raw data BPN error */
	HX_EBPNRAW = 1 << HX_BPN_RAWDATA,

	/* Get SC errors */
	HX_ESC = 1 << HX_SC,

	/* Noise error */
	HX_WT_ENOISE = 1 << HX_WT_NOISE,

	/* Noise error */
	HX_ABS_ENOISE = 1 << HX_ABS_NOISE,

	/* Sorting error*/
	HX_ESORT = 1 << HX_SORTING,

	/* Raw Data GAP  */
	HX_EGAP_RAW = 1 << HX_GAPTEST_RAW,

	/* ACT_IDLE RAW ERROR */
	HX_EACT_IDLE_RAW = 1 << HX_ACT_IDLE_RAWDATA,

	/* ACT_IDLE NOISE ERROR */
	HX_EACT_IDLE_NOISE = 1 << HX_ACT_IDLE_NOISE,

	HX_EACT_IDLE_BPNRAW = 1 << HX_ACT_IDLE_BPN_RAWDATA,

	/* LPWUG RAW ERROR */
	HX_ELP_RAW = 1 << HX_LP_RAWDATA,

	/* LPWUG NOISE ERROR */
	HX_ELP_WT_NOISE = 1 << HX_LP_WT_NOISE,

	/* LPWUG NOISE ERROR */
	HX_ELP_ABS_NOISE = 1 << HX_LP_ABS_NOISE,

	/* LPWUG IDLE RAW ERROR */
	HX_ELP_IDLE_RAW = 1 << HX_LP_IDLE_RAWDATA,

	/* LPWUG IDLE NOISE ERROR */
	HX_ELP_IDLE_NOISE = 1 << HX_LP_IDLE_NOISE,
	HX_ELP_BPNRAW = 1 << HX_LP_BPN_RAWDATA,
	HX_ELP_IDLE_BPNRAW = 1 << HX_LP_IDLE_BPN_RAWDATA,
};


#define HX_PRINT_STS_LOG \
"%s:0x%08X,tmp_data[0]=%x,tmp_data[1]=%x,tmp_data[2]=%x,tmp_data[3]=%x\n"

#endif
