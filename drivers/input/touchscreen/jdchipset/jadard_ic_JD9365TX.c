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
#include "jadard_platform.h"
#include "jadard_common.h"
#include "jadard_module.h"
#include "jadard_ic_JD9365TX.h"

extern struct jadard_module_fp g_module_fp;
extern struct jadard_ts_data *pjadard_ts_data;
extern struct jadard_ic_data *pjadard_ic_data;
extern struct jadard_report_data *pjadard_report_data;
extern struct jadard_common_variable g_common_variable;

extern char *jd_panel_maker_list[JD_PANEL_MAKER_LIST_SIZE];

#ifdef JD_AUTO_UPGRADE_FW
extern uint8_t *g_jadard_fw;
extern uint32_t g_jadard_fw_ver;
extern uint32_t g_jadard_fw_cid_ver;
#endif

#ifdef CONFIG_TOUCHSCREEN_JADARD_DEBUG
extern int *jd_diag_mutual;
#endif

static void jd9365tx_ChipInfoInit(void)
{
	/* Initial section info */
	g_jd9365tx_section_info.section_info_ready			  = false;
	g_jd9365tx_section_info.section_info_ready_addr		  = JD9365TX_MEMORY_ADDR_ERAM;
	g_jd9365tx_section_info.dsram_num_start_addr		  = JD9365TX_MEMORY_ADDR_ERAM + 2;
	g_jd9365tx_section_info.dsram_section_info_start_addr = JD9365TX_MEMORY_ADDR_ERAM + 4;
	g_jd9365tx_section_info.esram_num_start_addr		  = (g_jd9365tx_section_info.dsram_section_info_start_addr +
															JD9365TX_MAX_DSRAM_NUM * sizeof(struct JD9365TX_INFO_CONTENT));
	g_jd9365tx_section_info.esram_section_info_start_addr = g_jd9365tx_section_info.esram_num_start_addr + 4;
}

static int jd9365tx_Read_BackDoor_RegSingle(uint32_t addr, uint8_t *rdata)
{
	int ReCode;
	uint8_t addrBuf[JD_SIX_SIZE];
	uint8_t readBuf[JD_ONE_SIZE];

	addrBuf[0] = 0xF3;
	addrBuf[1] = (uint8_t)((addr & 0xFF000000) >> 24);
	addrBuf[2] = (uint8_t)((addr & 0x00FF0000) >> 16);
	addrBuf[3] = (uint8_t)((addr & 0x0000FF00) >> 8);
	addrBuf[4] = (uint8_t)((addr & 0x000000FF) >> 0);
	addrBuf[5] = 0x03;

	ReCode = jadard_bus_read(addrBuf, sizeof(addrBuf), readBuf, sizeof(readBuf), JADARD_BUS_RETRY_TIMES);
	*rdata = readBuf[0];

	return ReCode;
}

static int jd9365tx_Write_BackDoor_RegSingle(uint32_t addr, uint8_t wdata)
{
	uint8_t addrBuf[JD_SIX_SIZE];
	uint8_t writeBuf[JD_ONE_SIZE];

	addrBuf[0] = 0xF2;
	addrBuf[1] = (uint8_t)((addr & 0xFF000000) >> 24);
	addrBuf[2] = (uint8_t)((addr & 0x00FF0000) >> 16);
	addrBuf[3] = (uint8_t)((addr & 0x0000FF00) >> 8);
	addrBuf[4] = (uint8_t)((addr & 0x000000FF) >> 0);
	addrBuf[5] = 0x03;
	writeBuf[0] = wdata;

	return jadard_bus_write(addrBuf, sizeof(addrBuf), writeBuf, sizeof(writeBuf), JADARD_BUS_RETRY_TIMES);
}

static int jd9365tx_Read_BackDoor_RegMulti(uint32_t addr, uint8_t *rdata, uint16_t rlen)
{
	uint8_t addrBuf[JD_SIX_SIZE];

	addrBuf[0] = 0xF3;
	addrBuf[1] = (uint8_t)((addr & 0xFF000000) >> 24);
	addrBuf[2] = (uint8_t)((addr & 0x00FF0000) >> 16);
	addrBuf[3] = (uint8_t)((addr & 0x0000FF00) >> 8);
	addrBuf[4] = (uint8_t)((addr & 0x000000FF) >> 0);
	addrBuf[5] = 0x03;

	return jadard_bus_read(addrBuf, sizeof(addrBuf), rdata, rlen, JADARD_BUS_RETRY_TIMES);
}

static int jd9365tx_Write_BackDoor_RegMulti(uint32_t addr, uint8_t *wdata, uint16_t wlen)
{
	uint8_t addrBuf[JD_SIX_SIZE];

	addrBuf[0] = 0xF2;
	addrBuf[1] = (uint8_t)((addr & 0xFF000000) >> 24);
	addrBuf[2] = (uint8_t)((addr & 0x00FF0000) >> 16);
	addrBuf[3] = (uint8_t)((addr & 0x0000FF00) >> 8);
	addrBuf[4] = (uint8_t)((addr & 0x000000FF) >> 0);
	addrBuf[5] = 0x03;

	return jadard_bus_write(addrBuf, sizeof(addrBuf), wdata, wlen, JADARD_BUS_RETRY_TIMES);
}

static int jd9365tx_Read_FW_RegSingleI2c(uint32_t addr, uint8_t *rdata)
{
	int ReCode;
	uint8_t addrBuf[JD_SIX_SIZE];
	uint8_t readBuf[JD_ONE_SIZE];

	addrBuf[0] = (uint8_t)((addr & 0xFF000000) >> 24);
	addrBuf[1] = (uint8_t)((addr & 0x00FF0000) >> 16);
	addrBuf[2] = (uint8_t)((addr & 0x0000FF00) >> 8);
	addrBuf[3] = (uint8_t)((addr & 0x000000FF) >> 0);
	addrBuf[4] = 0x00;
	addrBuf[5] = 0x01;

	ReCode = jadard_bus_read(addrBuf, sizeof(addrBuf), readBuf, sizeof(readBuf), JADARD_BUS_RETRY_TIMES);
	*rdata = readBuf[0];

	return ReCode;
}

static int jd9365tx_Write_FW_RegSingleI2c(uint32_t addr, uint8_t wdata)
{
	uint8_t addrBuf[JD_SIX_SIZE];
	uint8_t writeBuf[JD_ONE_SIZE];

	addrBuf[0] = (uint8_t)((addr & 0xFF000000) >> 24);
	addrBuf[1] = (uint8_t)((addr & 0x00FF0000) >> 16);
	addrBuf[2] = (uint8_t)((addr & 0x0000FF00) >> 8);
	addrBuf[3] = (uint8_t)((addr & 0x000000FF) >> 0);
	addrBuf[4] = 0x00;
	addrBuf[5] = 0x01;
	writeBuf[0] = wdata;

	return jadard_bus_write(addrBuf, sizeof(addrBuf), writeBuf, sizeof(writeBuf), JADARD_BUS_RETRY_TIMES);
}

static int jd9365tx_Read_FW_RegMultiI2c(uint32_t addr, uint8_t *rdata, uint16_t rlen)
{
	uint8_t addrBuf[JD_SIX_SIZE];

	addrBuf[0] = (uint8_t)((addr & 0xFF000000) >> 24);
	addrBuf[1] = (uint8_t)((addr & 0x00FF0000) >> 16);
	addrBuf[2] = (uint8_t)((addr & 0x0000FF00) >> 8);
	addrBuf[3] = (uint8_t)((addr & 0x000000FF) >> 0);
	addrBuf[4] = (uint8_t)((rlen & 0xFF00) >> 8);
	addrBuf[5] = (uint8_t)((rlen & 0x00FF) >> 0);

	return jadard_bus_read(addrBuf, sizeof(addrBuf), rdata, rlen, JADARD_BUS_RETRY_TIMES);
}

static int jd9365tx_Write_FW_RegMultiI2c(uint32_t addr, uint8_t *wdata, uint16_t wlen)
{
	uint8_t addrBuf[JD_SIX_SIZE];

	addrBuf[0] = (uint8_t)((addr & 0xFF000000) >> 24);
	addrBuf[1] = (uint8_t)((addr & 0x00FF0000) >> 16);
	addrBuf[2] = (uint8_t)((addr & 0x0000FF00) >> 8);
	addrBuf[3] = (uint8_t)((addr & 0x000000FF) >> 0);
	addrBuf[4] = (uint8_t)((wlen & 0xFF00) >> 8);
	addrBuf[5] = (uint8_t)((wlen & 0x00FF) >> 0);

	return jadard_bus_write(addrBuf, sizeof(addrBuf), wdata, wlen, JADARD_BUS_RETRY_TIMES);
}

static int jd9365tx_Read_FW_RegSingleSpi(uint32_t addr, uint8_t *rdata)
{
	int ReCode;
	uint8_t addrBuf[JD_SEVEN_SIZE];
	uint8_t readBuf[JD_ONE_SIZE];

	addrBuf[0] = 0xF3;
	addrBuf[1] = (uint8_t)((addr & 0xFF000000) >> 24);
	addrBuf[2] = (uint8_t)((addr & 0x00FF0000) >> 16);
	addrBuf[3] = (uint8_t)((addr & 0x0000FF00) >> 8);
	addrBuf[4] = (uint8_t)((addr & 0x000000FF) >> 0);
	addrBuf[5] = 0x00;
	addrBuf[6] = 0x01;

	ReCode = jadard_bus_read(addrBuf, sizeof(addrBuf), readBuf, sizeof(readBuf), JADARD_BUS_RETRY_TIMES);
	*rdata = readBuf[0];

	return ReCode;
}

static int jd9365tx_Write_FW_RegSingleSpi(uint32_t addr, uint8_t wdata)
{
	uint8_t addrBuf[JD_SEVEN_SIZE];
	uint8_t writeBuf[JD_ONE_SIZE];

	addrBuf[0] = 0xF2;
	addrBuf[1] = (uint8_t)((addr & 0xFF000000) >> 24);
	addrBuf[2] = (uint8_t)((addr & 0x00FF0000) >> 16);
	addrBuf[3] = (uint8_t)((addr & 0x0000FF00) >> 8);
	addrBuf[4] = (uint8_t)((addr & 0x000000FF) >> 0);
	addrBuf[5] = 0x00;
	addrBuf[6] = 0x01;
	writeBuf[0] = wdata;

	return jadard_bus_write(addrBuf, sizeof(addrBuf), writeBuf, sizeof(writeBuf), JADARD_BUS_RETRY_TIMES);
}

static int jd9365tx_Read_FW_RegMultiSpi(uint32_t addr, uint8_t *rdata, uint16_t rlen)
{
	uint8_t addrBuf[JD_SEVEN_SIZE];

	addrBuf[0] = 0xF3;
	addrBuf[1] = (uint8_t)((addr & 0xFF000000) >> 24);
	addrBuf[2] = (uint8_t)((addr & 0x00FF0000) >> 16);
	addrBuf[3] = (uint8_t)((addr & 0x0000FF00) >> 8);
	addrBuf[4] = (uint8_t)((addr & 0x000000FF) >> 0);
	addrBuf[5] = (uint8_t)((rlen & 0xFF00) >> 8);
	addrBuf[6] = (uint8_t)((rlen & 0x00FF) >> 0);

	return jadard_bus_read(addrBuf, sizeof(addrBuf), rdata, rlen, JADARD_BUS_RETRY_TIMES);
}

static int jd9365tx_Write_FW_RegMultiSpi(uint32_t addr, uint8_t *wdata, uint16_t wlen)
{
	uint8_t addrBuf[JD_SEVEN_SIZE];

	addrBuf[0] = 0xF2;
	addrBuf[1] = (uint8_t)((addr & 0xFF000000) >> 24);
	addrBuf[2] = (uint8_t)((addr & 0x00FF0000) >> 16);
	addrBuf[3] = (uint8_t)((addr & 0x0000FF00) >> 8);
	addrBuf[4] = (uint8_t)((addr & 0x000000FF) >> 0);
	addrBuf[5] = (uint8_t)((wlen & 0xFF00) >> 8);
	addrBuf[6] = (uint8_t)((wlen & 0x00FF) >> 0);

	return jadard_bus_write(addrBuf, sizeof(addrBuf), wdata, wlen, JADARD_BUS_RETRY_TIMES);
}

static int jd9365tx_ReadRegSingle(uint32_t addr, uint8_t *rdata)
{
	int ReCode;

	if (!pjadard_ts_data->diag_thread_active)
		jd9365tx_EnterBackDoor(NULL);

	if (g_jd9365tx_chip_info.back_door_mode) {
		ReCode = jd9365tx_Read_BackDoor_RegSingle(addr, rdata);
	} else {
		if ((pjadard_ts_data != NULL) && (pjadard_ts_data->spi != NULL))
			ReCode = jd9365tx_Read_FW_RegSingleSpi(addr, rdata);
		else
			ReCode = jd9365tx_Read_FW_RegSingleI2c(addr, rdata);
	}

	if (!pjadard_ts_data->diag_thread_active)
		jd9365tx_ExitBackDoor();

	return ReCode;
}

static int jd9365tx_WriteRegSingle(uint32_t addr, uint8_t wdata)
{
	int ReCode;

	if (!pjadard_ts_data->diag_thread_active)
		jd9365tx_EnterBackDoor(NULL);

	if (g_jd9365tx_chip_info.back_door_mode) {
		ReCode = jd9365tx_Write_BackDoor_RegSingle(addr, wdata);
	} else {
		if ((pjadard_ts_data != NULL) && (pjadard_ts_data->spi != NULL))
			ReCode = jd9365tx_Write_FW_RegSingleSpi(addr, wdata);
		else
			ReCode = jd9365tx_Write_FW_RegSingleI2c(addr, wdata);
	}

	if (!pjadard_ts_data->diag_thread_active)
		jd9365tx_ExitBackDoor();

	return ReCode;
}

static int jd9365tx_ReadRegMulti(uint32_t addr, uint8_t *rdata, uint16_t rlen)
{
	int ReCode;

#ifndef JD_I2C_SINGLE_MODE
	if ((addr >= JD9365TX_MEMORY_ADDR_ERAM) &&
		(addr < (JD9365TX_MEMORY_ADDR_ERAM + JD9365TX_MEMORY_ERAM_SIZE))) {
		ReCode = jd9365tx_Read_BackDoor_RegMulti(addr, rdata, rlen);
	} else {
#endif
		if (!pjadard_ts_data->diag_thread_active)
			jd9365tx_EnterBackDoor(NULL);

		if (g_jd9365tx_chip_info.back_door_mode) {
			ReCode = jd9365tx_Read_BackDoor_RegMulti(addr, rdata, rlen);
		} else {
			if ((pjadard_ts_data != NULL) && (pjadard_ts_data->spi != NULL))
				ReCode = jd9365tx_Read_FW_RegMultiSpi(addr, rdata, rlen);
			else
				ReCode = jd9365tx_Read_FW_RegMultiI2c(addr, rdata, rlen);
		}

		if (!pjadard_ts_data->diag_thread_active)
			jd9365tx_ExitBackDoor();
#ifndef JD_I2C_SINGLE_MODE
	}
#endif

	return ReCode;
}

static int jd9365tx_WriteRegMulti(uint32_t addr, uint8_t *wdata, uint16_t wlen)
{
	int ReCode;

	if (!pjadard_ts_data->diag_thread_active)
		jd9365tx_EnterBackDoor(NULL);

	if (g_jd9365tx_chip_info.back_door_mode) {
		ReCode = jd9365tx_Write_BackDoor_RegMulti(addr, wdata, wlen);
	} else {
		if ((pjadard_ts_data != NULL) && (pjadard_ts_data->spi != NULL))
			ReCode = jd9365tx_Write_FW_RegMultiSpi(addr, wdata, wlen);
		else
			ReCode = jd9365tx_Write_FW_RegMultiI2c(addr, wdata, wlen);
	}

	if (!pjadard_ts_data->diag_thread_active)
		jd9365tx_ExitBackDoor();

	return ReCode;
}

static int jd9365tx_GetID(uint16_t *pRomID)
{
	int ReCode;
	uint8_t rbuf[JD_TWO_SIZE];

	ReCode = jd9365tx_ReadRegMulti((uint32_t)JD9365TX_SOC_REG_ADDR_CHIP_ID2, rbuf, sizeof(rbuf));

	if (ReCode < 0)
		return ReCode;

	*pRomID = (uint16_t)((rbuf[1] << 8) + rbuf[0]);

	if (*pRomID == JD9365TX_ID) {
		JD_I("%s: IC ID: %04x\n", __func__, *pRomID);
	} else {
		JD_E("%s: Error IC ID: %04x\n", __func__, *pRomID);
		ReCode = JD_CHIP_ID_ERROR;
	}

	return ReCode;
}

static int jd9365tx_EnterBackDoor(uint16_t *pRomID)
{
	int ReCode;
#ifndef JD_I2C_SINGLE_MODE
	uint8_t addrBuf[JD_FIVE_SIZE];
	uint8_t writeBuf[JD_ONE_SIZE];

	#define POR_BACKDOOR

	#if defined(POR_BACKDOOR)
		addrBuf[0] = 0xF2;
		addrBuf[1] = 0xAA;
		addrBuf[2] = 0xF0;
		addrBuf[3] = 0x0F;
		addrBuf[4] = 0x55;
		writeBuf[0] = 0x68;
	#else
		addrBuf[0] = 0xF2;
		addrBuf[1] = 0xAA;
		addrBuf[2] = 0x55;
		addrBuf[3] = 0x0F;
		addrBuf[4] = 0xF0;
		writeBuf[0] = 0x68;
	#endif

	ReCode = jadard_bus_write(addrBuf, sizeof(addrBuf), writeBuf, sizeof(writeBuf), JADARD_BUS_RETRY_TIMES);

	if (ReCode < 0) {
		JD_E("%s: EnterBackDoor fail\n", __func__);
	} else {
		g_jd9365tx_chip_info.back_door_mode = true;

		/* If pRomID != NULL, Read IC ID */
		if (pRomID)
			ReCode = jd9365tx_GetID(pRomID);
	}

	return ReCode;
#else
	g_jd9365tx_chip_info.back_door_mode = false;
	/* If pRomID != NULL, Read IC ID */
	if (pRomID)
		ReCode = jd9365tx_GetID(pRomID);
	else
		ReCode = JD_NO_ERR;

	return ReCode;
#endif
}

static int jd9365tx_ExitBackDoor(void)
{
#ifndef JD_I2C_SINGLE_MODE
	int ReCode;
	uint8_t addrBuf[JD_FIVE_SIZE];
	uint8_t writeBuf[JD_ONE_SIZE];

	addrBuf[0] = 0xF2;
	addrBuf[1] = 0xAA;
	addrBuf[2] = 0x88;
	addrBuf[3] = 0x00;
	addrBuf[4] = 0x00;
	writeBuf[0] = 0x00;

	ReCode = jadard_bus_write(addrBuf, sizeof(addrBuf), writeBuf, sizeof(writeBuf), JADARD_BUS_RETRY_TIMES);

	if (ReCode < 0)
		JD_E("%s: ExitBackDoor fail\n", __func__);
	else
		g_jd9365tx_chip_info.back_door_mode = false;

	return ReCode;
#else
	return JD_NO_ERR;
#endif
}

/* static int jd9365tx_DisableSleepOutInt(void)
{
	return jd9365tx_WriteRegSingle((uint32_t)jd9365tx_SOC_REG_ADDR_SLPOUT_INT_EN,
									(uint8_t)SOC_RELATED_SETTING_DISABLE_SLEEP_OUT_INT);
}

static int jd9365tx_EnableSleepOutInt(void)
{
	return jd9365tx_WriteRegSingle((uint32_t)jd9365tx_SOC_REG_ADDR_SLPOUT_INT_EN,
									(uint8_t)SOC_RELATED_SETTING_ENABLE_SLEEP_OUT_INT);
} */

static int jd9365tx_StartMCUClock(void)
{
	int ReCode;

	ReCode = jd9365tx_WriteRegSingle((uint32_t)JD9365TX_SOC_REG_ADDR_CPU_CLK_STOP,
										(uint8_t)JD9365TX_SOC_PASSWORD_START_MCU_CLOCK);

	if (ReCode < 0)
		JD_E("%s: Start MCU clock fail\n", __func__);

	return ReCode;
}

static int jd9365tx_StopMCUClock(void)
{
#ifndef JD_I2C_SINGLE_MODE
	int ReCode;

	ReCode = jd9365tx_WriteRegSingle((uint32_t)JD9365TX_SOC_REG_ADDR_CPU_CLK_STOP,
										(uint8_t)JD9365TX_SOC_PASSWORD_STOP_MCU_CLOCK);

	if (ReCode < 0)
		JD_E("%s: Stop MCU clock fail\n", __func__);

	return ReCode;
#else
	return JD_NO_ERR;
#endif
}

static int jd9365tx_StartMCU(void)
{
	int ReCode;

	ReCode = jd9365tx_WriteRegSingle((uint32_t)JD9365TX_SOC_REG_ADDR_PRAM_PROG,
										(uint8_t)JD9365TX_SOC_PASSWORD_START_MCU);

	if (ReCode < 0)
		JD_E("%s: Start MCU fail\n", __func__);

	return ReCode;
}

static int jd9365tx_StopMCU(void)
{
	int ReCode;

	ReCode = jd9365tx_WriteRegSingle((uint32_t)JD9365TX_SOC_REG_ADDR_PRAM_PROG,
										(uint8_t)JD9365TX_SOC_PASSWORD_STOP_MCU);

	if (ReCode < 0)
		JD_E("%s: Stop MCU fail\n", __func__);

	return ReCode;
}

static int jd9365tx_ResetSOC(void)
{
	int ReCode;
	int retry = 0;

	do {
		ReCode = jd9365tx_WriteRegSingle((uint32_t)JD9365TX_SOC_REG_ADDR_RGU_0,
										(uint8_t)JD9365TX_SOC_PASSWORD_SOC_RESET);
	} while ((++retry < 5) && (ReCode < 0));

	if (ReCode < 0)
		JD_E("%s: Reset SOC fail\n", __func__);
	else
		g_jd9365tx_chip_info.back_door_mode = false;

	return ReCode;
}

static int jd9365tx_ResetMCU(void)
{
	int ReCode;

	ReCode = jd9365tx_WriteRegSingle((uint32_t)JD9365TX_SOC_REG_ADDR_CPU_SOFT_RESET,
										(uint8_t)JD9365TX_SOC_PASSWORD_MCU_RESET);

	if (ReCode < 0)
		JD_E("%s: Reset MCU fail\n", __func__);

	return ReCode;
}

static int jd9365tx_PorInit(void)
{
	int ReCode;

	ReCode = jd9365tx_WriteRegSingle((uint32_t)JD9365TX_SOC_REG_ADDR_POR_INIT,
										(uint8_t)JD9365TX_SOC_PASSWORD_POR_CLEAR);

	if (ReCode < 0)
		JD_E("%s: Por init fail\n", __func__);

	return ReCode;
}

static int jd9365tx_DisableTouchScan(void)
{
	return jd9365tx_WriteRegSingle((uint32_t)JD9365TX_STC1_REG_ADDR_AFE_SCAN_CTRL,
									(uint8_t)JD9365TX_STC1_RELATED_SETTING_STC_SCAN_DISABLE);
}

#ifndef JD_ZERO_FLASH
static int jd9365tx_EnableTouchScan(void)
{
	return jd9365tx_WriteRegSingle((uint32_t)JD9365TX_STC1_REG_ADDR_AFE_SCAN_CTRL,
									(uint8_t)JD9365TX_STC1_RELATED_SETTING_STC_SCAN_ENABLE);
}

static int jd9365tx_EnableRTCRun(void)
{
	return jd9365tx_WriteRegSingle((uint32_t)JD9365TX_TIMER_REG_ADDR_RTC_CONFIG,
									(uint8_t)JD9365TX_TIMER_RELATED_SETTING_ENABLE_RTC_RUN);
}
#endif

static int jd9365tx_DisableRTCRun(void)
{
	return jd9365tx_WriteRegSingle((uint32_t)JD9365TX_TIMER_REG_ADDR_RTC_CONFIG,
									(uint8_t)JD9365TX_TIMER_RELATED_SETTING_DISABLE_RTC_RUN);
}

static int jd9365tx_DisableWDTRun(void)
{
	int ReCode;

	ReCode = jd9365tx_WriteRegSingle((uint32_t)JD9365TX_TIMER_REG_ADDR_WDT_CONFIG,
									(uint8_t)JD9365TX_TIMER_RELATED_SETTING_WDT_STOP);

	if (ReCode < 0)
		JD_E("%s: Disable WDT run fail\n", __func__);

	return ReCode;
}

static bool jd9365tx_GetSectionFirstValueStatus(uint16_t usTimeOut)
{
	uint32_t i = 0;
	uint8_t rbuf[JD_TWO_SIZE];
	bool section_first_value_status_done = false;

	do {
		/* Pulling section ready == JD9365TX_SECTION_INFO_READY_VALUE */
		mdelay(2);
		if (jd9365tx_ReadRegMulti(g_jd9365tx_section_info.section_info_ready_addr, rbuf, sizeof(rbuf)) < 0) {
			JD_E("%s: Read section info fail\n", __func__);
			return false;
		} else {
			i += 2;
		}

		if (((rbuf[1] << 8) + rbuf[0]) == JD9365TX_SECTION_INFO_READY_VALUE)
			section_first_value_status_done = true;
		else
			section_first_value_status_done = false;
	} while ((i < usTimeOut) && !section_first_value_status_done);

	if (i >= usTimeOut) {
		JD_E("%s: Get section status timeout\n", __func__);
		return false;
	}

	return section_first_value_status_done;
}

static int jd9365tx_GetSectionInfo(uint32_t start_addr, uint8_t section_num, struct JD9365TX_INFO_CONTENT *info_content)
{
	int ReCode, i;
	uint8_t info_content_size = (uint8_t)sizeof(struct JD9365TX_INFO_CONTENT);
	uint8_t *rbuf = NULL;
	int rbuf_len = section_num * info_content_size;

	rbuf = kzalloc(rbuf_len * sizeof(uint8_t), GFP_KERNEL);
	if (rbuf == NULL) {
		JD_E("%s: Memory alloc fail\n", __func__);
		return JD_MEM_ALLOC_FAIL;
	}

	ReCode = jd9365tx_ReadRegMulti(start_addr, rbuf, rbuf_len);
	if (ReCode < 0) {
		JD_E("%s: Read section info fail\n", __func__);
		kfree(rbuf);
		return ReCode;
	}

	if (info_content_size == 8) {
		for (i = 0; i < section_num; i++) {
			info_content[i].info_content_addr = (uint32_t)(rbuf[i * info_content_size + 0] +
														(rbuf[i * info_content_size + 1] << 8) +
														(rbuf[i * info_content_size + 2] << 16) +
														(rbuf[i * info_content_size + 3] << 24));

			info_content[i].info_content_len = (uint32_t)(rbuf[i * info_content_size + 4] +
														(rbuf[i * info_content_size + 5] << 8) +
														(rbuf[i * info_content_size + 6] << 16) +
														(rbuf[i * info_content_size + 7] << 24));
		}
	} else {
		JD_E("%s: info_content_size was not 8, must to check flow\n", __func__);
	}

	kfree(rbuf);
	return ReCode;
}

static int jd9365tx_ReadSectionInfo(bool reinit_config)
{
	int ReCode, i;
	struct JD9365TX_INFO_CONTENT dsram_info_content[JD9365TX_MAX_DSRAM_NUM];
	struct JD9365TX_INFO_CONTENT esram_info_content[JD9365TX_MAX_ESRAM_NUM];
	uint8_t rbuf[JD_SIX_SIZE];
	bool reinit = false;

	/* 1. Get section ready */
	if (jd9365tx_GetSectionFirstValueStatus(50) == true) {
		/* 2. Get section address */
		jd9365tx_GetSectionInfo(g_jd9365tx_section_info.dsram_section_info_start_addr,
								JD9365TX_REPAIR_MODE_SWITCH + 1, &dsram_info_content[0]);

		jd9365tx_GetSectionInfo(g_jd9365tx_section_info.esram_section_info_start_addr,
								JD9365TX_ESRAM_DDREG_DATA_BUF + 1, &esram_info_content[0]);

		/* 2.1 Dump section info */
		for (i = 0; i <= JD9365TX_REPAIR_MODE_SWITCH; i++) {
			/* Mapping to enum JD9365TX_DSRAM_SECTION_INFO_ORDER */
			JD_D("%s: Dsram section(%d) Address: 0x%04x, Length: %d\n", __func__, i,
				dsram_info_content[i].info_content_addr,
				dsram_info_content[i].info_content_len);
		}

		for (i = 0; i <= JD9365TX_ESRAM_DDREG_DATA_BUF; i++) {
			/* Mapping to enum JD9365TX_ESRAM_SECTION_INFO_ORDER */
			JD_D("%s: Esram section(%d) Address: 0x%04x, Length: %d\n", __func__, i,
				esram_info_content[i].info_content_addr,
				esram_info_content[i].info_content_len);
		}

		/* 3.1 Set Coordinate addr. and length */
		g_jd9365tx_chip_info.esram_info_content_addr.coordinate_report =
			esram_info_content[JD9365TX_ESRAM_COORDINATE_REPORT].info_content_addr;
		pjadard_report_data->touch_data_size = esram_info_content[JD9365TX_ESRAM_COORDINATE_REPORT].info_content_len;

		if (pjadard_report_data->touch_data_size > JD_TOUCH_DATA_SIZE) {
			JD_E("%s: FW report packet size %d is greater than Driver %d\n", __func__,
				pjadard_report_data->touch_data_size, JD_TOUCH_DATA_SIZE);
		}

		/* 3.2 Set ADC number and resolution */
		ReCode = jd9365tx_ReadRegMulti(dsram_info_content[JD9365TX_DSRAM_TOUCH_CFG].info_content_addr,
										rbuf, sizeof(rbuf));
		if (ReCode < 0) {
			JD_E("%s: Update global config fail\n", __func__);
		} else {
			/* Update global config */
			if ((pjadard_ic_data) && (pjadard_ts_data) && (pjadard_ts_data->pdata)) {
				if ((pjadard_ic_data->JD_X_NUM != rbuf[4]) || (pjadard_ic_data->JD_Y_NUM != rbuf[5])) {
					/* Set adc number by FW */
					pjadard_ic_data->JD_X_NUM = rbuf[4];
					pjadard_ic_data->JD_Y_NUM = rbuf[5];
					JD_I("FW:panel-sense-num = %d, %d\n", pjadard_ic_data->JD_X_NUM, pjadard_ic_data->JD_Y_NUM);
#ifdef CONFIG_TOUCHSCREEN_JADARD_DEBUG
					if (jd_diag_mutual) {
						kfree(jd_diag_mutual);
						jd_diag_mutual = NULL;
					}

					/* Reallocate mutual data memory */
					jd_diag_mutual = kzalloc(pjadard_ic_data->JD_X_NUM *
											pjadard_ic_data->JD_Y_NUM * sizeof(int), GFP_KERNEL);
					if (jd_diag_mutual == NULL)
						JD_E("%s: mutual buffer allocate failed\n", __func__);
#endif
				}

				if (reinit_config) {
					pjadard_ts_data->pdata->abs_x_min = 0;
					pjadard_ts_data->pdata->abs_x_max = (int)((rbuf[1] << 8) + rbuf[0]);
					pjadard_ts_data->pdata->abs_y_min = 0;
					pjadard_ts_data->pdata->abs_y_max = (int)((rbuf[3] << 8) + rbuf[2]);

					if (pjadard_ic_data->JD_X_RES != pjadard_ts_data->pdata->abs_x_max) {
						pjadard_ic_data->JD_X_RES = pjadard_ts_data->pdata->abs_x_max;
						reinit = true;
					}

					if (pjadard_ic_data->JD_Y_RES != pjadard_ts_data->pdata->abs_y_max) {
						pjadard_ic_data->JD_Y_RES = pjadard_ts_data->pdata->abs_y_max;
						reinit = true;
					}

					if (reinit) {
						/* Touch input config reinit */
						input_unregister_device(pjadard_ts_data->input_dev);

						if (jadard_input_register(pjadard_ts_data)) {
							JD_E("%s: Unable to register %s input device\n",
							  __func__, pjadard_ts_data->input_dev->name);
						}
					}
				}
			} else {
				JD_E("%s: Can`t update global config, pjadard_ic_data/pjadard_ts_data/pdata was null\n", __func__);
			}

			/* Record JD9365TX_DSRAM_HOST_ADDR */
			g_jd9365tx_chip_info.dsram_host_addr.fw_cid_version =
				dsram_info_content[JD9365TX_DSRAM_HOST].info_content_addr + 12;
			g_jd9365tx_chip_info.dsram_host_addr.fw_version =
				dsram_info_content[JD9365TX_DSRAM_HOST].info_content_addr + 16;
			g_jd9365tx_chip_info.dsram_host_addr.game_mode_en =
				dsram_info_content[JD9365TX_DSRAM_HOST].info_content_addr + 30;
			g_jd9365tx_chip_info.dsram_host_addr.usb_en =
				dsram_info_content[JD9365TX_DSRAM_HOST].info_content_addr + 32;
			g_jd9365tx_chip_info.dsram_host_addr.gesture_en =
				dsram_info_content[JD9365TX_DSRAM_HOST].info_content_addr + 34;
			g_jd9365tx_chip_info.dsram_host_addr.high_sensitivity_en =
				dsram_info_content[JD9365TX_DSRAM_HOST].info_content_addr + 36;
			g_jd9365tx_chip_info.dsram_host_addr.border_en =
				dsram_info_content[JD9365TX_DSRAM_HOST].info_content_addr + 38;
			g_jd9365tx_chip_info.dsram_host_addr.proximity_en =
				dsram_info_content[JD9365TX_DSRAM_HOST].info_content_addr + 44;
			g_jd9365tx_chip_info.dsram_host_addr.panel_maker =
				dsram_info_content[JD9365TX_DSRAM_HOST].info_content_addr + 56;
			g_jd9365tx_chip_info.dsram_host_addr.panel_version =
				dsram_info_content[JD9365TX_DSRAM_HOST].info_content_addr + 57;
			g_jd9365tx_chip_info.dsram_host_addr.earphone_en =
				dsram_info_content[JD9365TX_DSRAM_HOST].info_content_addr + 58;
#ifdef CONFIG_TOUCHSCREEN_JADARD_SORTING
			g_jd9365tx_chip_info.dsram_host_addr.mpap_pw =
				dsram_info_content[JD9365TX_DSRAM_HOST].info_content_addr + 40;
			g_jd9365tx_chip_info.dsram_host_addr.mpap_handshake =
				dsram_info_content[JD9365TX_DSRAM_HOST].info_content_addr + 42;
			g_jd9365tx_chip_info.dsram_host_addr.mpap_keep_frame =
				dsram_info_content[JD9365TX_DSRAM_HOST].info_content_addr + 43;
			g_jd9365tx_chip_info.dsram_host_addr.mpap_skip_frame =
				dsram_info_content[JD9365TX_DSRAM_HOST].info_content_addr + 52;
			g_jd9365tx_chip_info.dsram_host_addr.mpap_mux_switch =
				dsram_info_content[JD9365TX_DSRAM_HOST].info_content_addr + 53;
			g_jd9365tx_chip_info.dsram_host_addr.fw_dbic_en =
				dsram_info_content[JD9365TX_DSRAM_HOST].info_content_addr + 54;
			g_jd9365tx_chip_info.dsram_host_addr.mpap_pw_sync =
				dsram_info_content[JD9365TX_DSRAM_HOST].info_content_addr + 76;
			g_jd9365tx_chip_info.dsram_host_addr.mpap_error_msg =
				dsram_info_content[JD9365TX_DSRAM_HOST].info_content_addr + 78;
			/* Set mapa diff max/min addr */
			g_jd9365tx_chip_info.dsram_host_addr.mpap_diff_max =
				dsram_info_content[JD9365TX_DSRAM_MP_DIFF_MAX].info_content_addr;
			g_jd9365tx_chip_info.dsram_host_addr.mpap_diff_min =
				dsram_info_content[JD9365TX_DSRAM_MP_DIFF_MIN].info_content_addr;
#endif
#ifdef JD_RAWDATA_V2
			g_jd9365tx_chip_info.dsram_host_addr.data_output_v2 =
				dsram_info_content[JD9365TX_DSRAM_HOST].info_content_addr + 86;
#endif
			/* Record JD9365TX_DSRAM_DEBUG_ADDR */
			g_jd9365tx_chip_info.dsram_debug_addr.output_data_addr =
				dsram_info_content[JD9365TX_DSRAM_DEBUG].info_content_addr + 28;
			g_jd9365tx_chip_info.dsram_debug_addr.output_data_sel =
				dsram_info_content[JD9365TX_DSRAM_DEBUG].info_content_addr + 34;
			g_jd9365tx_chip_info.dsram_debug_addr.output_data_handshake =
				dsram_info_content[JD9365TX_DSRAM_DEBUG].info_content_addr + 36;
			g_jd9365tx_chip_info.dsram_debug_addr.freq_band =
				dsram_info_content[JD9365TX_DSRAM_DEBUG].info_content_addr + 40;

			/* Get esram_output_buf.output_buf.data address */
			ReCode = jd9365tx_ReadRegMulti(g_jd9365tx_chip_info.dsram_debug_addr.output_data_addr, rbuf, 4);
			if (ReCode < 0) {
				JD_E("%s: Get output buf data fail\n", __func__);
			} else {
				g_jd9365tx_chip_info.dsram_debug_addr.output_data_addr = ((rbuf[3] << 24) |
																		  (rbuf[2] << 16) |
																		  (rbuf[1] << 8) |
																		  (rbuf[0]));
				pjadard_ts_data->fw_ready = true;
				JD_I("%s: Section info was ready\n", __func__);
			}
		}
	} else {
		pjadard_ts_data->fw_ready = false;
		JD_E("%s: Section info was not ready\n", __func__);
		ReCode = JD_APP_START_FAIL;
	}

	return ReCode;
}

static int jd9365tx_ClearSectionFirstValue(void)
{
	return jd9365tx_WriteRegSingle(g_jd9365tx_section_info.section_info_ready_addr, 0x00);
}

static void jd9365tx_GetDataCRC16(uint8_t *buf, uint32_t buf_len, uint32_t start_index,
									uint32_t len, uint16_t *crc, uint16_t PolynomialCRC16)
{
	uint32_t i, j;
	uint16_t sum = *crc;

	for (i = 0; i < len; i++) {
		if ((start_index + i) >= buf_len)
			sum ^= (uint16_t)(0xFF << 8);
		else
			sum ^= (uint16_t)(buf[start_index + i] << 8);

		for (j = 0; j < 8; j++) {
			if ((sum & 0x8000) == 0x8000)
				sum = (uint16_t)((sum << 1) ^ PolynomialCRC16);
			else
				sum <<= 1;
		}
	}

	*crc = sum;
}

static int jd9365tx_GetDMAStatus(uint16_t usTimeOut)
{
	int ReCode;
	uint32_t i = 0;
	uint8_t usValue = (uint8_t)JD9365TX_DMA_RELATED_SETTING_DMA_BUSY;
	bool dma_busy_status_done = false;

	do {
		/* Pulling DMA busy = JD9365TX_DMA_RELATED_SETTING_DMA_DONE */
		mdelay(2);
		ReCode = jd9365tx_ReadRegSingle((uint32_t)JD_FLASH_REG_ADDR_DMA_BUSY_OR_START, &usValue);

		if (ReCode < 0) {
			JD_E("%s: Read DMA_BUSY_OR_START register fail\n", __func__);
			return ReCode;
		} else {
			i += 2;
		}

		dma_busy_status_done = g_module_fp.fp_DoneStatusIsLow(usValue, (uint8_t)JD9365TX_DMA_RELATED_SETTING_DMA_BUSY,
																(uint8_t)JD9365TX_DMA_RELATED_SETTING_DMA_DONE);
	} while ((i < usTimeOut) && !dma_busy_status_done);

	if (i >= usTimeOut) {
		JD_E("%s: Get DMA status timeout\n", __func__);
		return JD_TIME_OUT;
	}

	return ReCode;
}

static int jd9365tx_GetPageProgramStatus(uint16_t usTimeOut)
{
	int ReCode;
	uint32_t i = 0;
	uint8_t usValue = (uint8_t)JD9365TX_DMA_RELATED_SETTING_PAGE_PROGRAM_FLASH;
	bool dma_page_program_status_done = false;

	do {
		/* Pulling DMA Page Program = JD9365TX_DMA_RELATED_SETTING_PAGE_PROGRAM_DONE */
		mdelay(2);
		ReCode = jd9365tx_ReadRegSingle((uint32_t)JD_FLASH_REG_ADDR_DMA_2WFLASHEN_1WRITE_0START, &usValue);

		if (ReCode < 0) {
			JD_E("%s: Read DMA_2WFLASHEN_1WRITE_0START register fail\n", __func__);
			return ReCode;
		} else {
			i += 2;
		}

		dma_page_program_status_done = g_module_fp.fp_DoneStatusIsLow(usValue, (uint8_t)JD9365TX_DMA_RELATED_SETTING_PAGE_PROGRAM_FLASH,
																		(uint8_t)JD9365TX_DMA_RELATED_SETTING_PAGE_PROGRAM_DONE);
	} while ((i < usTimeOut) && !dma_page_program_status_done);

	if (i >= usTimeOut) {
		JD_E("%s: Get DMA status timeout\n", __func__);
		return JD_TIME_OUT;
	}

	return ReCode;
}

static int jd9365tx_SetDMAStart(uint32_t flash_addr, uint32_t dma_size, uint32_t target_addr, uint8_t dma_cmd)
{
	int ReCode;
	uint8_t wBuf[JD_SIX_SIZE + JD_SEVEN_SIZE];

	if (dma_cmd == (uint8_t)JD9365TX_DMA_RELATED_SETTING_WRITE_TO_FLASH) {
		ReCode = g_module_fp.fp_ICWriteExFlashFlow((uint8_t)JD_EX_FLASH_ADDR_WRITE_ENABLE, 0, 0, NULL, 0);
		if (ReCode < 0) {
			JD_E("%s: Set write enable fail\n", __func__);
			return ReCode;
		}
	}

	if ((dma_cmd == (uint8_t)JD9365TX_DMA_RELATED_SETTING_READ_FROM_FLASH) ||
		(dma_cmd == (uint8_t)JD9365TX_DMA_RELATED_SETTING_WRITE_TO_FLASH)) {
		ReCode = g_module_fp.fp_ICSetExFlashCSNOutEnable();
		if (ReCode < 0) {
			JD_E("%s: Set CSN enable fail\n", __func__);
			return ReCode;
		}
	}

	wBuf[0] =  (uint8_t)((flash_addr & 0x000000FF) >> 0);  /* Set flash_addr 0byte */
	wBuf[1] =  (uint8_t)((flash_addr & 0x0000FF00) >> 8);  /* Set flash_addr 1byte */
	wBuf[2] =  (uint8_t)((flash_addr & 0x00FF0000) >> 16); /* Set flash_addr 2byte */
	wBuf[3] =  (uint8_t)((flash_addr & 0xFF000000) >> 24); /* Set flash_addr 3byte */

	wBuf[4] =  (uint8_t)((dma_size & 0x000000FF) >> 0);	 /* Set dma_size 0byte */
	wBuf[5] =  (uint8_t)((dma_size & 0x0000FF00) >> 8);	 /* Set dma_size 1byte */
	wBuf[6] =  (uint8_t)((dma_size & 0x00FF0000) >> 16); /* Set dma_size 2byte */
	wBuf[7] =  (uint8_t)((dma_size & 0xFF000000) >> 24); /* Set dma_size 3byte */

	wBuf[8] =  (uint8_t)((target_addr & 0x000000FF) >> 0);	/* Set target_addr 0byte */
	wBuf[9] =  (uint8_t)((target_addr & 0x0000FF00) >> 8);	/* Set target_addr 1byte */
	wBuf[10] = (uint8_t)((target_addr & 0x00FF0000) >> 16); /* Set target_addr 2byte */
	wBuf[11] = (uint8_t)((target_addr & 0xFF000000) >> 24); /* Set target_addr 3byte */
	wBuf[12] = dma_cmd; /* dma_1write_0start */

	ReCode = jd9365tx_WriteRegMulti((uint32_t)JD_FLASH_REG_ADDR_DMA_FLASH_ADDR0, wBuf, sizeof(wBuf));
	if (ReCode < 0) {
		JD_E("%s: Set DMA parameters fail\n", __func__);
		return ReCode;
	}

	if (dma_cmd == (uint8_t)JD9365TX_DMA_RELATED_SETTING_PAGE_PROGRAM_FLASH) {
		/* Pulling DMA Page Program = JD9365TX_DMA_RELATED_SETTING_PAGE_PROGRAM_DONE */
		mdelay(1);
		ReCode = jd9365tx_GetPageProgramStatus(100);
		if (ReCode < 0) {
			JD_E("%s: Get DMA page program status fail\n", __func__);
			return ReCode;
		}
	} else {
		/* Pulling DMA busy = JD9365TX_DMA_RELATED_SETTING_DMA_DONE */
		ReCode = jd9365tx_GetDMAStatus(50);
		if (ReCode < 0) {
			JD_E("%s: Get DMA status fail\n", __func__);
			return ReCode;
		}
	}

	if ((dma_cmd == (uint8_t)JD9365TX_DMA_RELATED_SETTING_READ_FROM_FLASH) ||
		(dma_cmd == (uint8_t)JD9365TX_DMA_RELATED_SETTING_WRITE_TO_FLASH)) {
		ReCode = g_module_fp.fp_ICSetExFlashCSNOutDisable();
		if (ReCode < 0)
			JD_E("%s: Set CSN disable fail\n", __func__);
	}

	return ReCode;
}

static int jd9365tx_SetDMAByteMode(void)
{
	uint8_t wBuf[JD_ONE_SIZE];

	wBuf[0] = (uint8_t)JD9365TX_DMA_RELATED_SETTING_TRANSFER_DATA_1_BYTE_MODE;

	return jd9365tx_WriteRegMulti((uint32_t)JD_FLASH_REG_ADDR_DMA_BYTE_MODE, wBuf, sizeof(wBuf));
}

static int jd9365tx_AbortHostDMA(void)
{
	uint8_t wBuf[JD_ONE_SIZE];

	wBuf[0] = (uint8_t)JD9365TX_DMA_RELATED_SETTING_DMA_ABORT;

	return jd9365tx_WriteRegMulti((uint32_t)JD_FLASH_REG_ADDR_DMA_0ABORT, wBuf, sizeof(wBuf));
}

static uint8_t jd9365tx_GetMoveInfoNumber(uint8_t *pFileData)
{
	if (pFileData[JD9365TX_HW_HEADER_LEN] == 0xFF) {
		return 0;
	} else {
		return (uint8_t)((pFileData[JD9365TX_HW_HEADER_LEN] - JD9365TX_HW_HEADER_INITIAL_NUMBER) /
							JD9365TX_HW_HEADER_MOVE_INFO_NUMBER);
	}
}

static void jd9365tx_GetHeaderInfo(uint8_t *pFileData, struct JD9365TX_MOVE_INFO *move_info, int move_info_sz)
{
	int i;
	int header_shift = JD9365TX_HW_HEADER_CRC_NUMBER + JD9365TX_HW_HEADER_INITIAL_NUMBER;

	for (i = 0; i < move_info_sz; i++) {
		move_info[i].mov_mem_cmd =
			pFileData[header_shift + i * JD9365TX_HW_HEADER_MOVE_INFO_NUMBER + 0];

		move_info[i].fl_st_addr =
			(uint32_t)((pFileData[header_shift + i * JD9365TX_HW_HEADER_MOVE_INFO_NUMBER + 1] << 16) +
						(pFileData[header_shift + i * JD9365TX_HW_HEADER_MOVE_INFO_NUMBER + 2] << 8) +
						(pFileData[header_shift + i * JD9365TX_HW_HEADER_MOVE_INFO_NUMBER + 3] << 0));

		move_info[i].fl_len =
			(uint32_t)((pFileData[header_shift + i * JD9365TX_HW_HEADER_MOVE_INFO_NUMBER + 4] << 8) +
						(pFileData[header_shift + i * JD9365TX_HW_HEADER_MOVE_INFO_NUMBER + 5] << 0) + 1);

		move_info[i].to_mem_st_addr =
			(uint32_t)((pFileData[header_shift + i * JD9365TX_HW_HEADER_MOVE_INFO_NUMBER + 6] << 24) +
						(pFileData[header_shift + i * JD9365TX_HW_HEADER_MOVE_INFO_NUMBER + 7] << 16) +
						(pFileData[header_shift + i * JD9365TX_HW_HEADER_MOVE_INFO_NUMBER + 8] << 8) +
						(pFileData[header_shift + i * JD9365TX_HW_HEADER_MOVE_INFO_NUMBER + 9] << 0));

		move_info[i].to_mem_crc =
			(uint16_t)((pFileData[header_shift + i * JD9365TX_HW_HEADER_MOVE_INFO_NUMBER + 10] << 8) +
						(pFileData[header_shift + i * JD9365TX_HW_HEADER_MOVE_INFO_NUMBER + 11] << 0));
	}
}

static uint8_t jd9365tx_GetHeaderPsramIndex(struct JD9365TX_MOVE_INFO *move_info, int move_info_sz)
{
	int i;
	uint8_t index = 0;

	for (i = 0; i < move_info_sz; i++) {
		if (move_info[i].to_mem_st_addr == JD9365TX_HW_HEADER_PSRAM_START_ADDR) {
			index = (uint8_t)i;
			break;
		}
	}

	JD_I("%s: Header Psram Index = %d\n", __func__, index);
	return index;
}

#ifndef JD_ZERO_FLASH
static int jd9365tx_ReadFlash(uint32_t addr, uint8_t *rdata, uint32_t rlen)
{
	int ReCode;
	uint16_t packsize = (uint16_t)JD9365TX_FLASH_PACK_READ_SIZE;
	uint32_t pos = 0;
	uint16_t packlen = 0;
	/* MCU aram config */
	uint16_t ramsize = JD9365TX_MEMORY_ARAM_L_SIZE;
	uint32_t ramaddr = 0;
	uint16_t ramlen = 0;
	/* DMA config */
	uint32_t flashaddr = addr;
	uint32_t targetaddr = (uint32_t)JD9365TX_MEMORY_ADDR_ARAM;
	/* Read buf */
	uint8_t *rBuf = NULL;

	rBuf = kzalloc(packsize * sizeof(uint8_t), GFP_KERNEL);
	if (rBuf == NULL) {
		JD_E("%s: Memory alloc fail\n", __func__);
		return JD_MEM_ALLOC_FAIL;
	}

	/* Set CRC initial value */
	ReCode = g_module_fp.fp_SetCRCInitialValue();
	if (ReCode < 0) {
		JD_E("%s: Set CRC initial value fail\n", __func__);
		kfree(rBuf);
		return ReCode;
	}

	/* Set CRC Initial */
	ReCode = g_module_fp.fp_SetCRCInitial();
	if (ReCode < 0) {
		JD_E("%s: Set CRC initial fail\n", __func__);
		kfree(rBuf);
		return ReCode;
	}

	/* Read start */
	while (rlen > 0) {
		if (rlen > ramsize)
			ramlen = ramsize;
		else
			ramlen = (uint16_t)rlen;

		/* Set next ram addr */
		rlen -= ramlen;
		ramaddr = (uint32_t)JD9365TX_MEMORY_ADDR_ARAM;

		/* Set DMA Start & Pulling DMA busy = JD9365TX_DMA_RELATED_SETTING_DMA_DONE */
		JD_D("%s: Flash DMA start\n", __func__);

		ReCode = jd9365tx_SetDMAStart(flashaddr, ramlen, targetaddr,
										(uint8_t)JD9365TX_DMA_RELATED_SETTING_READ_FROM_FLASH);
		if (ReCode < 0) {
			JD_E("%s: DMA start error\n", __func__);
			kfree(rBuf);
			return ReCode;
		} else {
			JD_D("%s: DMA start finish\n", __func__);
		}

		/* Read from ram */
		while (ramlen > 0) {
			if (ramlen > packsize)
				packlen = packsize;
			else
				packlen = (uint16_t)ramlen;

			/* Read Data from ARAM */
			ReCode = jd9365tx_ReadRegMulti(ramaddr, rBuf, packlen);
			if (ReCode < 0) {
				JD_E("%s: Read ARAM data fail\n", __func__);
				kfree(rBuf);
				return ReCode;
			} else {
				memcpy(rdata + pos, rBuf, packlen);
			}

			/* Set next packet */
			ramlen -= packlen;
			pos += packlen;
			ramaddr += packlen;
			flashaddr += packlen;
		}

		JD_D("%s: Read flash data %d KB...\n", __func__, (uint16_t)((flashaddr - addr) / 1024));
	}

	kfree(rBuf);
	return ReCode;
}

static int jd9365tx_WriteFlash(uint32_t addr, uint8_t *wdata, uint32_t wlen)
{
	int ReCode;
	uint16_t packsize = (uint16_t)JD9365TX_FLASH_PACK_WRITE_SIZE;
	uint32_t pos = addr;
	uint16_t packlen = 0;
	/* MCU aram config */
	uint16_t ramsize = JD9365TX_MEMORY_ARAM_L_SIZE;
	uint32_t ramaddr = 0;
	uint16_t ramlen = 0;
	/* DMA config */
#if defined(CONFIG_TOUCHSCREEN_JADARD_DEBUG)
	uint32_t start_flashaddr = addr;
#endif
	uint32_t flashaddr = 0;
	uint16_t dmasize = 0;
	uint32_t targetaddr = (uint32_t)JD9365TX_MEMORY_ADDR_ARAM;

	/* Set CRC initial value */
	ReCode = g_module_fp.fp_SetCRCInitialValue();
	if (ReCode < 0) {
		JD_E("%s: Set CRC initial value fail\n", __func__);
		return ReCode;
	}

	/* Set CRC Initial */
	ReCode = g_module_fp.fp_SetCRCInitial();
	if (ReCode < 0) {
		JD_E("%s: Set CRC initial fail\n", __func__);
		return ReCode;
	}

	/* Write start */
	while (wlen > 0) {
		if (wlen > ramsize)
			ramlen = ramsize;
		else
			ramlen = (uint16_t)wlen;

		flashaddr = addr;
		dmasize = ramlen;
		/* Set next ram addr */
		wlen -= ramlen;
		ramaddr = (uint32_t)JD9365TX_MEMORY_ADDR_ARAM;

		/* Write to ram */
		while (ramlen > 0) {
			if (ramlen > packsize)
				packlen = packsize;
			else
				packlen = (uint16_t)ramlen;

			/* Write Data to ARAM */
			ReCode = jd9365tx_WriteRegMulti(ramaddr, wdata + pos, packlen);
			if (ReCode < 0) {
				JD_E("%s: Write data to ARAM fail\n", __func__);
				return ReCode;
			}

			/* Set next packet */
			ramlen -= packlen;
			pos += packlen;
			ramaddr += packlen;
			addr += packlen;
		}

		/* Set DMA Start & Pulling DMA busy = JD9365TX_DMA_RELATED_SETTING_DMA_DONE */
		ReCode = jd9365tx_SetDMAStart(flashaddr, dmasize, targetaddr,
										(uint8_t)JD9365TX_DMA_RELATED_SETTING_WRITE_TO_FLASH);
		if (ReCode < 0) {
			JD_E("%s: DMA start error\n", __func__);
			return ReCode;
		} else {
			JD_D("%s: Flash DMA start\n", __func__);
		}

		/* Read Ex-Flash status until done */
		ReCode = g_module_fp.fp_ICGetExFlashStatus(50);
		if (ReCode < 0) {
			JD_E("%s: Read Ex-Flash status fail\n", __func__);
			return ReCode;
		} else {
			JD_D("%s: DMA status ok\n", __func__);
		}

		JD_D("%s: Write flash data %d KB...\n", __func__, (uint16_t)((flashaddr - start_flashaddr) / 1024));
	}

	ReCode = g_module_fp.fp_ICWriteExFlashFlow((uint8_t)JD_EX_FLASH_ADDR_WRITE_DISABLE, 0, 0, NULL, 0);
	if (ReCode < 0)
		JD_E("%s: Write flash disable fail\n", __func__);

	return ReCode;
}

static int jd9365tx_CheckFlashContent(uint8_t *pFileData, uint32_t addr, uint32_t len)
{
	int ReCode, i;
	uint32_t error_count = 0;
	uint8_t *pData = kzalloc(len * sizeof(uint8_t), GFP_KERNEL);

	if (pData == NULL) {
		JD_E("%s: Memory alloc fail\n", __func__);
		return JD_MEM_ALLOC_FAIL;
	}

	ReCode = jd9365tx_ReadFlash(addr, pData, len);
	if (ReCode < 0) {
		kfree(pData);
		return ReCode;
	}

	for (i = 0; i < len; i++) {
		if (pFileData[addr + i] != pData[i]) {
			JD_E("%s: Error data[%d]=0x%02x, Current data[%d]=0x%02x\n", __func__, i, pData[i], i, pFileData[addr + i]);
			error_count++;
			if (error_count > 5) {
				kfree(pData);
				return JD_CHECK_DATA_ERROR;
			}
		}
	}

	kfree(pData);
	return ReCode;
}

static int jd9365tx_HostReadFlash(uint32_t ReadAddr, uint8_t *pDataBuffer, uint32_t ReadLen)
{
	int ReCode;
	uint16_t romid = 0;
	uint16_t crc, softcrc;

	/* 0. Disable pram check sum */
	ReCode = g_module_fp.fp_SetPramCksumEn(false);
	if (ReCode < 0) {
		JD_E("%s: Set pram checksum disable fail\n", __func__);
		return ReCode;
	}

	ReCode = jd9365tx_AbortHostDMA();
	if (ReCode < 0) {
		JD_E("%s: Initial DMA fail\n", __func__);
		return ReCode;
	}

	/* 1. Enter backdoor & 2. Read IC ID */
	ReCode = jd9365tx_EnterBackDoor(&romid);
	if (ReCode < 0)
		return ReCode;

	/* 3.1 IC RTC run off */
	ReCode = jd9365tx_DisableRTCRun();
	if (ReCode < 0)
		return ReCode;

	/* 3.2 IC touch scan off */
	ReCode = jd9365tx_DisableTouchScan();
	if (ReCode < 0)
		return ReCode;

	/* 3.3 Set DMA 1-byte mode */
	ReCode = jd9365tx_SetDMAByteMode();
	if (ReCode < 0)
		return ReCode;

	/* 4.1 Stop WDT */
	ReCode = jd9365tx_DisableWDTRun();
	if (ReCode < 0)
		return ReCode;

	/* 4.2 Stop MCU clock */
	ReCode = jd9365tx_StopMCUClock();
	if (ReCode < 0)
		return ReCode;

	/* 5. Set Flash SPI speed & 6. Read Flash id */
	ReCode = g_module_fp.fp_JEDEC_ID(NULL);
	if (ReCode < 0)
		return ReCode;

	/* 7. Read data from flash */
	JD_I("%s: Read flash start\n", __func__);
	ReCode = jd9365tx_ReadFlash(ReadAddr, pDataBuffer, ReadLen);
	if (ReCode < 0) {
		JD_E("%s: Read flash error\n", __func__);
		return ReCode;
	} else {
		JD_I("%s: Read flash finish\n", __func__);
	}

	/* 8.1 Get IC crc */
	ReCode = g_module_fp.fp_GetCRCResult(&crc);
	if (ReCode < 0) {
		JD_E("%s: Read flash ecc fail\n", __func__);
		return ReCode;
	}

	/* 8.2 Calculate software crc */
	softcrc = JD_CRC_INFO_CRC_INITIAL_VALUE;
	jd9365tx_GetDataCRC16(pDataBuffer, ReadLen, 0, ReadLen, &softcrc, JD_CRC_INFO_PolynomialCRC16);

	/* 8.3 Compare IC and software crc */
	if (softcrc != crc)
		JD_E("%s: softEcc is %04x, but read IC crc is %04x\n", __func__, softcrc, crc);

	/* 9.1 IC RTC run on */
	ReCode = jd9365tx_EnableRTCRun();
	if (ReCode < 0)
		return ReCode;

	/* 9.2 IC touch scan on */
	ReCode = jd9365tx_EnableTouchScan();
	if (ReCode < 0)
		return ReCode;

	/* 9.3 start MCU clock */
	ReCode = jd9365tx_StartMCUClock();
	if (ReCode < 0)
		return ReCode;

	return ReCode;
}

static int jd9365tx_HostWriteFlash(uint32_t WriteAddr, uint8_t *pFileData, uint32_t FileSize)
{
	int retry = 0;
	int ReCode;
	uint16_t romid = 0;
	uint16_t crc, softcrc;
	uint8_t id;
	uint8_t MoveInfoNumber;
	uint8_t MoveInfoPsramIndex = 0;
	struct JD9365TX_MOVE_INFO *MoveInfo = NULL;

	ReCode = g_module_fp.fp_SetPramCksumEn(false);
	if (ReCode < 0) {
		JD_E("%s: Set pram checksum disable fail\n", __func__);
		return ReCode;
	}

	ReCode = jd9365tx_AbortHostDMA();
	if (ReCode < 0) {
		JD_E("%s: Initial DMA fail\n", __func__);
		return ReCode;
	}

	do {
		/* 1. Enter backdoor & 2. Read IC ID */
		ReCode = jd9365tx_EnterBackDoor(&romid);
		if (ReCode < 0)
			return ReCode;

		/* 3.1 IC RTC run off */
		ReCode = jd9365tx_DisableRTCRun();
		if (ReCode < 0)
			return ReCode;

		/* 3.2 IC touch scan off */
		ReCode = jd9365tx_DisableTouchScan();
		if (ReCode < 0)
			return ReCode;

		/* 3.3 Set DMA 1-byte mode */
		ReCode = jd9365tx_SetDMAByteMode();
		if (ReCode < 0)
			return ReCode;

		/* 3.4 Clear section first value */
		ReCode = jd9365tx_ClearSectionFirstValue();
		if (ReCode < 0)
			return ReCode;

		/* 4.1 Stop WDT */
		ReCode = jd9365tx_DisableWDTRun();
		if (ReCode < 0)
			return ReCode;

		/* 4.2 Stop MCU clock */
		ReCode = jd9365tx_StopMCUClock();
		if (ReCode < 0)
			return ReCode;

		/* 5. Set Flash SPI speed & Read Flash id */
		ReCode = g_module_fp.fp_JEDEC_ID(&id);
		if (ReCode < 0)
			return ReCode;

		/* 6. Check flash size is enough */
		ReCode = g_module_fp.fp_CheckSize(id, FileSize);
		if (ReCode < 0)
			return ReCode;

#ifdef JD_FLASH_WP_EN
		/* 6.2 flash WP unlock */
		ReCode = g_module_fp.fp_FlashUnlock();
		if (ReCode < 0) {
			JD_E("%s: Flash unlock fail\n", __func__);
			return ReCode;
		}
#endif

		/* 7. Erase flash */
		JD_I("%s: Erase flash start, Start Address: 0x%08x, Size: %d\n", __func__, WriteAddr, FileSize);
		ReCode = g_module_fp.fp_EraseChipFlash(WriteAddr, FileSize);
		if (ReCode < 0) {
			JD_E("%s: Flash erase fail\n", __func__);
			return ReCode;
		} else {
			JD_I("%s: Erase Flash completed\n", __func__);
		}

		/* 8. Set CRC disable */
		ReCode = g_module_fp.fp_SetCRCEnable(false);
		if (ReCode < 0) {
			JD_E("%s: CRC disable fail\n", __func__);
			return ReCode;
		}

		/* 9. Set Flash SPI speed */
		ReCode = g_module_fp.fp_SetFlashSPISpeed(pFileData[JD9365TX_LOAD_CODE_SPI_FREQ]);
		if (ReCode < 0)
			return ReCode;

		/* 10. Get header info */
		MoveInfoNumber = jd9365tx_GetMoveInfoNumber(pFileData);
		if (MoveInfoNumber == 0) {
			JD_E("%s: Header information error, please check FW bin!\n", __func__);
			return JD_CHECK_DATA_ERROR;
		}

		MoveInfo = kzalloc(MoveInfoNumber * sizeof(struct JD9365TX_MOVE_INFO), GFP_KERNEL);
		if (MoveInfo == NULL) {
			JD_E("%s: Memory allocate fail\n", __func__);
			return JD_MEM_ALLOC_FAIL;
		}

		jd9365tx_GetHeaderInfo(pFileData, MoveInfo, MoveInfoNumber);
		MoveInfoPsramIndex = jd9365tx_GetHeaderPsramIndex(MoveInfo, MoveInfoNumber);

		/* 10.1 Write data to flash */
		JD_I("%s: Write flash start\n", __func__);
		if (FileSize >= JD9365TX_HW_HEADER_MAX_SIZE + MoveInfo[MoveInfoPsramIndex].fl_len) {
			ReCode = jd9365tx_WriteFlash(WriteAddr, pFileData, (uint32_t)(JD9365TX_HW_HEADER_MAX_SIZE + MoveInfo[MoveInfoPsramIndex].fl_len));
		} else {
			JD_E("%s: Write data was not enough!\n", __func__);
			kfree(MoveInfo);
			return JD_WRITE_OVERFLOW;
		}

		if (ReCode < 0) {
			JD_E("%s: Write flash fail\n", __func__);
			kfree(MoveInfo);
			return ReCode;
		} else {
			JD_I("%s: Write flash finish\n", __func__);
		}

		/* 11.1 Get IC crc */
		ReCode = g_module_fp.fp_GetCRCResult(&crc);
		if (ReCode < 0) {
			JD_E("%s: Read flash ecc fail\n", __func__);
			kfree(MoveInfo);
			return ReCode;
		}

		/* 11.2 calculate software crc */
		softcrc = JD_CRC_INFO_CRC_INITIAL_VALUE;
		jd9365tx_GetDataCRC16(pFileData, (uint32_t)(JD9365TX_HW_HEADER_MAX_SIZE + MoveInfo[MoveInfoPsramIndex].fl_len),
								WriteAddr, (uint32_t)(JD9365TX_HW_HEADER_MAX_SIZE + MoveInfo[MoveInfoPsramIndex].fl_len),
								&softcrc, JD_CRC_INFO_PolynomialCRC16);

		kfree(MoveInfo);
		/* 11.3 Compare IC and software crc */
		if (softcrc != crc) {
			JD_E("%s: softEcc is %04x, but read IC crc is %04x\n", __func__, softcrc, crc);
			ReCode = JD_CHECK_DATA_ERROR;
			goto JD_UPGRADE_FW_RETRY;
		} else {
			JD_I("%s: CRC check pass\n", __func__);
		}

		/* 12.1 Write data to flash */
		ReCode = jd9365tx_WriteFlash(JD9365TX_HW_HEADER_MAX_SIZE + JD9365TX_PRAM_MAX_SIZE, pFileData,
										FileSize - (JD9365TX_HW_HEADER_MAX_SIZE + JD9365TX_PRAM_MAX_SIZE));

		/* 12.2 Get IC crc */
		ReCode = g_module_fp.fp_GetCRCResult(&crc);
		if (ReCode < 0) {
			JD_E("%s: Read flash ecc fail\n", __func__);
			return ReCode;
		}

		/* 12.3 calculate software crc */
		softcrc = JD_CRC_INFO_CRC_INITIAL_VALUE;
		jd9365tx_GetDataCRC16(pFileData, FileSize - (JD9365TX_HW_HEADER_MAX_SIZE + JD9365TX_PRAM_MAX_SIZE),
								JD9365TX_HW_HEADER_MAX_SIZE + JD9365TX_PRAM_MAX_SIZE,
								FileSize - (JD9365TX_HW_HEADER_MAX_SIZE + JD9365TX_PRAM_MAX_SIZE),
								&softcrc, JD_CRC_INFO_PolynomialCRC16);

		/* 12.4 Compare IC and software crc */
		//if (softcrc != crc) {
			//JD_E("%s: softEcc is %04x, but read IC crc is %04x\n", __func__, softcrc, crc);
			//ReCode = JD_CHECK_DATA_ERROR;
			//goto JD_UPGRADE_FW_RETRY;
		//} else {
			//JD_I("%s: CRC check pass\n", __func__);
		//}

		/* 13. Read back Flash content [option] */
		ReCode = jd9365tx_CheckFlashContent(pFileData, 0, 256);

#ifdef JD_FLASH_WP_EN
		 /* 13.1 flash WP lock */
		if (g_module_fp.fp_FlashLock() < 0)
			JD_E("%s: Flash lock fail\n", __func__);
#endif

#ifdef JD_I2C_SINGLE_MODE
	JD_UPGRADE_FW_RETRY:
		retry++;
	} while ((retry < 3) && (ReCode < 0));

	/* 14. Reset IC touch soc */
	jd9365tx_PorInit();
#ifdef JD_RST_PIN_FUNC
	g_module_fp.fp_pin_reset(1);
#else
	ReCode = jd9365tx_ResetSOC();
	if (ReCode < 0)
		JD_E("%s: Reset touch soc fail\n", __func__);
	else
		JD_I("%s: Reset touch soc\n", __func__);
#endif
#else
	JD_UPGRADE_FW_RETRY:
		retry++;

		/* 14. Reset IC touch soc */
		if (ReCode >= 0) {
			jd9365tx_PorInit();
#ifdef JD_RST_PIN_FUNC
			g_module_fp.fp_pin_reset(1);
#else
			ReCode = jd9365tx_ResetSOC();
			if (ReCode < 0)
				JD_E("%s: Reset touch soc fail\n", __func__);
			else
				JD_I("%s: Reset touch soc\n", __func__);
#endif
		}
	} while ((retry < 3) && (ReCode < 0));
#endif

	/* Wait flash reload to Pram */
	msleep(300);

	/* 15. Enter back door & read IC id */
	ReCode = jd9365tx_EnterBackDoor(&romid);
	if (ReCode < 0)
		return ReCode;

	/* 16. Set Flash SPI speed */
	ReCode = g_module_fp.fp_SetFlashSPISpeed(pFileData[JD9365TX_LOAD_CODE_SPI_FREQ]);
	if (ReCode < 0)
		return ReCode;

	/* 17. Reload section info. */
	ReCode = jd9365tx_ReadSectionInfo(true);

	return ReCode;
}
#endif

#ifdef JD_ZERO_FLASH
static int jd9365tx_WritePram(uint32_t addr, uint8_t *pData, uint32_t len)
{
	int ReCode;
	uint16_t packsize = (uint16_t)JD9365TX_FLASH_PACK_WRITE_SIZE;
	uint32_t pos = 0;
	uint16_t packlen = 0;
	/* MCU aram config */
	uint32_t ramaddr = addr;
	uint32_t ramlen = len;

	/* Write start */
	if (len > 0) {
		JD_I("%s: Start write ram addr: 0x%08x\n", __func__, ramaddr);
		JD_I("%s: Write pram data %d bytes\n", __func__, ramlen);

		/* Write to ram */
		while (ramlen > 0) {
			if (ramlen > packsize)
				packlen = packsize;
			else
				packlen = (uint16_t)ramlen;

			/* Write Data to PRAM/ARAM */
			ReCode = jd9365tx_WriteRegMulti(ramaddr, pData + pos, packlen);
			if (ReCode < 0) {
				JD_E("%s: Write data to PRAM/ARAM fail\n", __func__);
				return ReCode;
			}

			/* Set next packet */
			ramlen -= packlen;
			pos += packlen;
			ramaddr += packlen;
		}
	}

	/* Set CRC initial value */
	ReCode = g_module_fp.fp_SetCRCInitialValue();
	if (ReCode < 0) {
		JD_E("%s: Set CRC initial value fail\n", __func__);
		return ReCode;
	}

	/* Set CRC Initial */
	ReCode = g_module_fp.fp_SetCRCInitial();
	if (ReCode < 0) {
		JD_E("%s: Set CRC initial fail\n", __func__);
		return ReCode;
	}

	/* Set DMA Start & Pulling DMA busy = JD9365TX_DMA_RELATED_SETTING_DMA_DONE */
	ReCode = jd9365tx_SetDMAStart(addr, len, addr, (uint8_t)JD9365TX_DMA_RELATED_SETTING_WRITE_TO_PRAM);
	if (ReCode < 0) {
		JD_E("%s: DMA start error\n", __func__);
		return ReCode;
	} else {
		JD_I("%s: DMA start finish\n", __func__);
	}

	return ReCode;
}

static int jd9365tx_HostWritePram(uint32_t WriteAddr, uint8_t *pFileData, uint32_t FileSize)
{
	int ReCode, i;
	uint16_t romid = 0;
	uint16_t crc, softcrc;
	uint8_t MoveInfoNumber;
	uint8_t MoveInfoPsramIndex = 0;
	struct JD9365TX_MOVE_INFO *MoveInfo = NULL;
	bool bypass_cpu_reset = false;

	ReCode = g_module_fp.fp_SetPramCksumEn(false);
	if (ReCode < 0) {
		JD_E("%s: Set pram checksum disable fail\n", __func__);
		return ReCode;
	}

#ifndef JD_RESUME_TIME_REDUCE
	ReCode = jd9365tx_AbortHostDMA();
	if (ReCode < 0) {
		JD_E("%s: Initial DMA fail\n", __func__);
		return ReCode;
	}

	/* 1. Enter backdoor & 2. Read IC ID */
	ReCode = jd9365tx_EnterBackDoor(&romid);
	if (ReCode < 0)
		return ReCode;

	/* 3.1 IC RTC run off */
	ReCode = jd9365tx_DisableRTCRun();
	if (ReCode < 0)
		return ReCode;

	/* 3.2 IC touch scan off */
	ReCode = jd9365tx_DisableTouchScan();
	if (ReCode < 0)
		return ReCode;

	/* 3.3 Set DMA 1-byte mode */
	ReCode = jd9365tx_SetDMAByteMode();
	if (ReCode < 0)
		return ReCode;

	/* 3.4 Clear section first value */
	ReCode = jd9365tx_ClearSectionFirstValue();
	if (ReCode < 0)
		return ReCode;

	/* 4.1 Stop WDT */
	ReCode = jd9365tx_DisableWDTRun();
	if (ReCode < 0)
		return ReCode;

	/* 4.2 Stop MCU */
	ReCode = jd9365tx_StopMCU();
	if (ReCode < 0)
		return ReCode;

	/* 5. Stop MCU clock */
	ReCode = jd9365tx_StopMCUClock();
	if (ReCode < 0)
		return ReCode;
#endif

	/* 6. Get header info.*/
	MoveInfoNumber = jd9365tx_GetMoveInfoNumber(pFileData);
	MoveInfo = kzalloc(MoveInfoNumber * sizeof(struct JD9365TX_MOVE_INFO), GFP_KERNEL);
	if (MoveInfo == NULL) {
		JD_E("%s: Memory allocate fail\n", __func__);
		return JD_MEM_ALLOC_FAIL;
	}

	jd9365tx_GetHeaderInfo(pFileData, MoveInfo, MoveInfoNumber);
	MoveInfoPsramIndex = jd9365tx_GetHeaderPsramIndex(MoveInfo, MoveInfoNumber);

	/* START: Check need upgrade fw */
	if (pjadard_ts_data->power_on_upgrade == false) {
		/* Set CRC initial value */
		ReCode = g_module_fp.fp_SetCRCInitialValue();
		if (ReCode < 0) {
			JD_E("%s: [Check ram]Set CRC initial value fail\n", __func__);
			kfree(MoveInfo);
			return ReCode;
		}

		/* Set CRC Initial */
		ReCode = g_module_fp.fp_SetCRCInitial();
		if (ReCode < 0) {
			JD_E("%s: [Check ram]Set CRC initial fail\n", __func__);
			kfree(MoveInfo);
			return ReCode;
		}

		/* Set DMA Start & Pulling DMA busy = JD9365T_DMA_RELATED_SETTING_DMA_DONE */
		ReCode = jd9365tx_SetDMAStart(MoveInfo[MoveInfoPsramIndex].to_mem_st_addr, MoveInfo[MoveInfoPsramIndex].fl_len,
									  MoveInfo[MoveInfoPsramIndex].to_mem_st_addr, (uint8_t)JD9365TX_DMA_RELATED_SETTING_WRITE_TO_PRAM);
		if (ReCode < 0) {
			JD_E("%s: [Check ram]DMA start error\n", __func__);
			kfree(MoveInfo);
			return ReCode;
		} else {
			JD_D("%s: [Check ram]DMA start finish\n", __func__);
		}


		/* Get IC crc */
		ReCode = g_module_fp.fp_GetCRCResult(&crc);
		if (ReCode < 0) {
			JD_E("%s: [Check ram]Read crc fail\n", __func__);
			kfree(MoveInfo);
			return ReCode;
		}

		/* Check IC crc */
		if (crc != MoveInfo[MoveInfoPsramIndex].to_mem_crc) {
			JD_D("%s: [Check ram]Binary crc is %04x, but read IC crc is %04x\n", __func__, MoveInfo[MoveInfoPsramIndex].to_mem_crc, crc);
		} else {
			JD_I("%s: [Check ram]HW CRC check pass, bypass upgrade flow\n", __func__);
			goto SKIP_UPGRADE_FW;
		}
	}
	/* END: Check need upgrade fw */

#ifdef JD_RESUME_TIME_REDUCE
	ReCode = jd9365tx_AbortHostDMA();
	if (ReCode < 0) {
		JD_E("%s: Initial DMA fail\n", __func__);
		return ReCode;
	}

	/* 1. Enter backdoor & 2. Read IC ID */
	ReCode = jd9365tx_EnterBackDoor(&romid);
	if (ReCode < 0)
		return ReCode;

	/* 3.1 IC RTC run off */
	ReCode = jd9365tx_DisableRTCRun();
	if (ReCode < 0)
		return ReCode;

	/* 3.2 IC touch scan off */
	ReCode = jd9365tx_DisableTouchScan();
	if (ReCode < 0)
		return ReCode;

	/* 3.3 Set DMA 1-byte mode */
	ReCode = jd9365tx_SetDMAByteMode();
	if (ReCode < 0)
		return ReCode;

	/* 3.4 Clear section first value */
	ReCode = jd9365tx_ClearSectionFirstValue();
	if (ReCode < 0)
		return ReCode;

	/* 4.1 Stop WDT */
	ReCode = jd9365tx_DisableWDTRun();
	if (ReCode < 0)
		return ReCode;

	/* 4.2 Stop MCU */
	ReCode = jd9365tx_StopMCU();
	if (ReCode < 0)
		return ReCode;

	/* 5. Stop MCU clock */
	ReCode = jd9365tx_StopMCUClock();
	if (ReCode < 0)
		return ReCode;
#endif

	/* 7. Write data to PRAM/DRAM */
	JD_I("%s: Write ram start\n", __func__);

#ifdef JD_ESD_CHECK
	pjadard_ts_data->esd_fw_reload = true;
#endif
	pjadard_ts_data->power_on_upgrade = false;
	for (i = 0; i < MoveInfoNumber; i++) {
		if ((MoveInfo[i].fl_st_addr + MoveInfo[i].fl_len) > FileSize) {
			JD_E("%s: Write ram overflow\n", __func__);
			kfree(MoveInfo);
			return ReCode;
		}

		ReCode = jd9365tx_WritePram(MoveInfo[i].to_mem_st_addr, pFileData + MoveInfo[i].fl_st_addr, MoveInfo[i].fl_len);
		if (ReCode < 0) {
			JD_E("%s: Write pram fail\n", __func__);
			kfree(MoveInfo);
			return ReCode;
		}

		/* 8.1 Get IC crc */
		ReCode = g_module_fp.fp_GetCRCResult(&crc);
		if (ReCode < 0) {
			JD_E("%s: Read crc fail\n", __func__);
			kfree(MoveInfo);
			return ReCode;
		}

		/* 8.2 Check IC crc */
		if (crc != MoveInfo[i].to_mem_crc) {
			JD_E("%s: Binary crc is %04x, but read IC crc is %04x\n", __func__, MoveInfo[i].to_mem_crc, crc);
			kfree(MoveInfo);
			return ReCode;
		} else {
			JD_I("%s: HW CRC check pass\n", __func__);
		}

		/* 8.3 Calculate software crc */
		softcrc = JD_CRC_INFO_CRC_INITIAL_VALUE;
		jd9365tx_GetDataCRC16(pFileData + MoveInfo[i].fl_st_addr, MoveInfo[i].fl_len, 0, MoveInfo[i].fl_len, &softcrc, JD_CRC_INFO_PolynomialCRC16);

		/* 8.4 check software crc */
		if (softcrc != MoveInfo[i].to_mem_crc) {
			JD_E("%s: Binary crc is %04x, but soft crc is %04x\n", __func__, MoveInfo[i].to_mem_crc, softcrc);
			kfree(MoveInfo);
			return ReCode;
		} else {
			JD_I("%s: Software CRC check pass\n", __func__);
		}
	}

	/* 8-5. Clear PorInit */
	jd9365tx_PorInit();

	/* 8-6. Software reset SOC */
	ReCode = jd9365tx_ResetSOC();
	if (ReCode >= 0) {
		JD_I("%s: Reset touch SOC\n", __func__);
		bypass_cpu_reset = true;
	}

SKIP_UPGRADE_FW:
	kfree(MoveInfo);
	JD_I("%s: Write ram finish\n", __func__);

	/* 9. Software reset MCU */
	if (bypass_cpu_reset == false) {
		ReCode = jd9365tx_ResetMCU();
		if (ReCode < 0)
			return ReCode;
		else
			JD_I("%s: Reset touch mcu\n", __func__);
	}

	/* 10. Start MCU clock */
	ReCode = jd9365tx_StartMCUClock();
	if (ReCode < 0)
		return ReCode;
	else
		JD_I("%s: Start MCU clock\n", __func__);

	/* 11. start MCU */
	ReCode = jd9365tx_StartMCU();
	if (ReCode < 0)
		return ReCode;
	else
		JD_I("%s: Start MCU\n", __func__);

	/* 12. Reload section info. */
	ReCode = jd9365tx_ReadSectionInfo(false);

	return ReCode;
}
#endif

static int jd9365tx_ReadPram(uint32_t addr, uint8_t *pData, uint32_t len)
{
	int ReCode;
	uint16_t packsize = (uint16_t)JD9365TX_FLASH_PACK_READ_SIZE;
	uint32_t pos = 0;
	uint16_t packlen = 0;
	/* MCU ram config */
	uint32_t ramaddr = addr;
	uint32_t ramlen = len;
	/* Read buf */
	uint8_t *rBuf = NULL;

	rBuf = kzalloc(packsize * sizeof(uint8_t), GFP_KERNEL);
	if (rBuf == NULL) {
		JD_E("%s: Memory alloc fail\n", __func__);
		return JD_MEM_ALLOC_FAIL;
	}

	/* Read start */
	if (len > 0) {
		JD_I("%s: Start read ram addr: 0x%08x\n", __func__, ramaddr);
		JD_I("%s: Read ram data %d bytes\n", __func__, ramlen);

		/* Read from ram */
		while (ramlen > 0) {
			if (ramlen > packsize)
				packlen = packsize;
			else
				packlen = (uint16_t)ramlen;

			/* Read Data from ram */
			ReCode = jd9365tx_ReadRegMulti(ramaddr, rBuf, packlen);
			if (ReCode < 0) {
				JD_E("%s: Read ram data fail\n", __func__);
				kfree(rBuf);
				return ReCode;
			} else {
				memcpy(pData + pos, rBuf, packlen);
			}

			/* Set next packet */
			ramlen -= packlen;
			pos += packlen;
			ramaddr += packlen;
		}
	}

	/* Set CRC initial value */
	ReCode = g_module_fp.fp_SetCRCInitialValue();
	if (ReCode < 0) {
		JD_E("%s: Set CRC initial value fail\n", __func__);
		kfree(rBuf);
		return ReCode;
	}

	/* Set CRC Initial */
	ReCode = g_module_fp.fp_SetCRCInitial();
	if (ReCode < 0) {
		JD_E("%s: Set CRC initial fail\n", __func__);
		kfree(rBuf);
		return ReCode;
	}

	/* Set DMA Start & Pulling DMA busy = JD9365TX_DMA_RELATED_SETTING_DMA_DONE */
	ReCode = jd9365tx_SetDMAStart(addr, len, addr, (uint8_t)JD9365TX_DMA_RELATED_SETTING_WRITE_TO_PRAM);
	if (ReCode < 0) {
		JD_E("%s: DMA start error\n", __func__);
		kfree(rBuf);
		return ReCode;
	} else {
		JD_I("%s: DMA start finish\n", __func__);
	}

	kfree(rBuf);
	return ReCode;
}

static int jd9365tx_HostReadPram(uint32_t ReadAddr, uint8_t *pFileData, uint32_t ReadLen)
{
	int ReCode;
	uint16_t romid = 0;
	uint16_t crc, softcrc;

	/* 1. Enter backdoor & 2. Read IC ID */
	ReCode = jd9365tx_EnterBackDoor(&romid);
	if (ReCode < 0)
		return ReCode;

	/* 3.1 Set DMA 1-byte mode */
	ReCode = jd9365tx_SetDMAByteMode();
	if (ReCode < 0)
		return ReCode;

	/* 3.2 Stop MCU */
	ReCode = jd9365tx_StopMCU();
	if (ReCode < 0)
		return ReCode;

	/* 4.1 Stop WDT */
	ReCode = jd9365tx_DisableWDTRun();
	if (ReCode < 0)
		return ReCode;

	/* 4.2 Stop MCU clock */
	ReCode = jd9365tx_StopMCUClock();
	if (ReCode < 0)
		return ReCode;

	/* 5. Read data from PRAM/DRAM */
	JD_I("%s: Read ram start\n", __func__);
	ReCode = jd9365tx_ReadPram(ReadAddr, pFileData, ReadLen);
	if (ReCode < 0) {
		JD_E("%s: Read ram error\n", __func__);
		return ReCode;
	} else {
		JD_I("%s: Read ram finish\n", __func__);
	}

	/* 6.1 Get IC crc */
	ReCode = g_module_fp.fp_GetCRCResult(&crc);
	if (ReCode < 0) {
		JD_E("%s: Read flash ecc fail\n", __func__);
		return ReCode;
	}

	/* 6.2 Calculate software crc */
	softcrc = JD_CRC_INFO_CRC_INITIAL_VALUE;
	jd9365tx_GetDataCRC16(pFileData, ReadLen, ReadAddr, ReadLen, &softcrc, JD_CRC_INFO_PolynomialCRC16);

	/* 6.3 Compare IC and software crc */
	if (softcrc != crc)
		JD_E("%s: softEcc is %04x, but read IC crc is %04x\n", __func__, softcrc, crc);

	/* 7. Start MCU clock */
	ReCode = jd9365tx_StartMCUClock();
	if (ReCode >= 0)
		JD_I("%s: Start MCU clock\n", __func__);

	/* 8. start MCU */
	ReCode = jd9365tx_StartMCU();
	if (ReCode < 0)
		return ReCode;
	else
		JD_I("%s: Start MCU\n", __func__);

	msleep(10);

	return ReCode;
}

static int jd9365tx_ReadTpInfoBuf(uint8_t *rdata, uint16_t rlen)
{
	return jd9365tx_ReadRegMulti(g_jd9365tx_chip_info.esram_info_content_addr.coordinate_report, rdata, rlen);
}

static void module_jd9365tx_chip_init(void)
{
	JD_I("%s\n", __func__);
	/* Jadard: Set FW and CFG Flash Address */
	g_common_variable.FW_SIZE		 = JD9365TX_SIZE;
	g_common_variable.RAM_LEN		 = JD9365TX_PRAM_MAX_SIZE;
	jd9365tx_ChipInfoInit();

#ifndef JD_ZERO_FLASH
	/* Reload section info. */
	if (jd9365tx_ReadSectionInfo(false) < 0)
		JD_E("Chip init fail\n");
	else
		g_module_fp.fp_read_fw_ver();
#endif
}

static void module_jd9365tx_EnterBackDoor(void)
{
	jd9365tx_EnterBackDoor(NULL);
}

static void module_jd9365tx_ExitBackDoor(void)
{
	jd9365tx_ExitBackDoor();
}

static void module_jd9365tx_SoftReset(void)
{
	/* Reset IC touch soc */
	if (jd9365tx_ResetSOC() < 0) {
		JD_E("%s: Reset touch soc fail\n", __func__);
		return;
	} else {
		JD_I("%s: Reset touch soc\n", __func__);
	}

	/* Wait flash reload to Pram, Add Flash */
	msleep(100);

	/* Enter back door */
	jd9365tx_EnterBackDoor(NULL);
}

static void module_jd9365tx_ResetMCU(void)
{
	jd9365tx_ResetMCU();
}

static void module_jd9365tx_PorInit(void)
{
	jd9365tx_PorInit();
}

static int module_jd9365tx_register_read(uint32_t ReadAddr, uint8_t *ReadData, uint32_t ReadLen)
{
	int ReCode;

	if (ReadLen & 0xFFFF0000)
		ReCode = JD_READ_LEN_OVERFLOW;
	else
		ReCode = jd9365tx_ReadRegMulti(ReadAddr, ReadData, (uint16_t)ReadLen);

	return ReCode;
}

static int module_jd9365tx_register_write(uint32_t WriteAddr, uint8_t *WriteData, uint32_t WriteLen)
{
	return jd9365tx_WriteRegMulti(WriteAddr, WriteData, (uint16_t)WriteLen);
}

static int module_jd9365tx_ram_read(uint32_t ReadAddr, uint8_t *ReadBuffer, uint32_t ReadLen)
{
	return jd9365tx_HostReadPram(ReadAddr, ReadBuffer, ReadLen);
}

#ifdef JD_ZERO_FLASH
static int module_jd9365tx_ram_write(uint32_t WriteAddr, uint8_t *WriteBuffer, uint32_t WriteLen)
{
	return jd9365tx_HostWritePram(WriteAddr, WriteBuffer, WriteLen);
}
#else

static int module_jd9365tx_flash_read(uint32_t ReadAddr, uint8_t *ReadBuffer, uint32_t ReadLen)
{
	return jd9365tx_HostReadFlash(ReadAddr, ReadBuffer, ReadLen);
}

static int module_jd9365tx_flash_write(uint32_t WriteAddr, uint8_t *WriteData, uint32_t WriteLen)
{
	return jd9365tx_HostWriteFlash(WriteAddr, WriteData, WriteLen);
}
#endif
//add for 9375 info start
/* extern int fix_tp_proc_info(void *tp_data,u8 data_len); */
/* Stub implementation since the function is not available */
int fix_tp_proc_info(void *tp_data, u8 data_len)
{
	/* This function appears to be platform-specific and not available.
	 * Adding a stub to avoid compilation errors.
	 */
	return 0;
}
//add for 9375 info end
static void module_jd9365tx_read_fw_ver(void)
{
	int ReCode_cid, ReCode_ver, ReCode_maker, ReCode_panel, maker_id;
	uint8_t rdata_cid[JD_FOUR_SIZE], rdata_fw[JD_FOUR_SIZE], rdata_maker[JD_ONE_SIZE], rdata_panel[JD_ONE_SIZE];
	char *maker_name;
	//add for 9375 info start   
	int len = 0;
	unsigned char tp_info[512];
	//add for 9375 info end

	ReCode_cid = module_jd9365tx_register_read(g_jd9365tx_chip_info.dsram_host_addr.fw_cid_version, rdata_cid, sizeof(rdata_cid));
	ReCode_ver = module_jd9365tx_register_read(g_jd9365tx_chip_info.dsram_host_addr.fw_version, rdata_fw, sizeof(rdata_fw));
	ReCode_maker = module_jd9365tx_register_read(g_jd9365tx_chip_info.dsram_host_addr.panel_maker, rdata_maker, sizeof(rdata_maker));
	ReCode_panel = module_jd9365tx_register_read(g_jd9365tx_chip_info.dsram_host_addr.panel_version, rdata_panel, sizeof(rdata_panel));

	if ((ReCode_cid < 0) || (ReCode_ver < 0) || (ReCode_maker < 0) || (ReCode_panel < 0)) {
		JD_E("%s: Read fw version fail\n", __func__);
		pjadard_ic_data->fw_ver = 0;
		pjadard_ic_data->fw_cid_ver = 0;
		scnprintf(pjadard_ic_data->panel_maker, sizeof(pjadard_ic_data->panel_maker), "%s", jd_panel_maker_list[JD_PANEL_MAKER_NUM]);
		pjadard_ic_data->panel_ver = 0;
	} else {
		pjadard_ic_data->fw_ver = ((rdata_fw[3] << 24) |
								   (rdata_fw[2] << 16) |
								   (rdata_fw[1] << 8) |
								   (rdata_fw[0]));
		pjadard_ic_data->fw_cid_ver = ((rdata_cid[3] << 24) |
									   (rdata_cid[2] << 16) |
									   (rdata_cid[1] << 8) |
									   (rdata_cid[0]));
		maker_id = (int)rdata_maker[0];
		pjadard_ic_data->panel_ver = rdata_panel[0];

		switch (maker_id) {
		case JD_PANEL_MAKER_CP:
			maker_name = "CP";
			break;
		case JD_PANEL_MAKER_COB:
			maker_name = "COB";
			break;
		default:
			if (maker_id < JD_PANEL_MAKER_NUM)
				maker_name = jd_panel_maker_list[maker_id];
			else
				maker_name = jd_panel_maker_list[JD_PANEL_MAKER_NUM];
			break;
		}
		scnprintf(pjadard_ic_data->panel_maker, sizeof(pjadard_ic_data->panel_maker), "%s", maker_name);
	}

	JD_I("FW_VER: %08x\n", pjadard_ic_data->fw_ver);
	JD_I("FW_CID_VER: %08x\n", pjadard_ic_data->fw_cid_ver);
	JD_I("PANEL_MAKER: %s\n", pjadard_ic_data->panel_maker);
	JD_I("PANEL_VER: %02x\n", pjadard_ic_data->panel_ver);
	//add for 9375 info start 
	len=sprintf(tp_info,"TP IC: jd9365TX,TP MODULE: dezhixin,TP I2C ADR: 0x68,SW FirmWare: 0x%06X,Sample FirmWare: 0x%06X", pjadard_ic_data->fw_ver,pjadard_ic_data->fw_ver);
	fix_tp_proc_info(tp_info, len);
	//add for 9375 info end
}

#if defined(JD_AUTO_UPGRADE_FW)
static int module_jd9365tx_read_fw_ver_bin(void)
{
	if (g_jadard_fw != NULL) {
		JD_I("Catch fw version in bin file\n");
		g_jadard_fw_ver = ((g_jadard_fw[JD9365TX_BIN_FW_VER + 0] << 16) |
						   (g_jadard_fw[JD9365TX_BIN_FW_VER + 1] <<	 8) |
						   (g_jadard_fw[JD9365TX_BIN_FW_VER + 2]));
		g_jadard_fw_cid_ver = ((g_jadard_fw[JD9365TX_BIN_FW_CID_VER + 0] << 24) |
							   (g_jadard_fw[JD9365TX_BIN_FW_CID_VER + 1] << 16) |
							   (g_jadard_fw[JD9365TX_BIN_FW_CID_VER + 2] <<	 8) |
							   (g_jadard_fw[JD9365TX_BIN_FW_CID_VER + 3]));
	} else {
		JD_I("FW data is null\n");
		return JD_FILE_OPEN_FAIL;
	}

	return JD_NO_ERR;
}
#endif

static void module_jd9365tx_mutual_data_set(uint8_t data_type)
{
	uint8_t wdata[JD_TWO_SIZE] = {0x00, 0x00};
	uint8_t status[JD_TWO_SIZE];

	if ((data_type >= JD_DATA_TYPE_RawData) && (data_type <= JD_DATA_TYPE_RESERVE_F))
		wdata[0] = data_type - 1;
	else
		wdata[0] = JD_DATA_TYPE_RawData - 1;

	module_jd9365tx_register_write(g_jd9365tx_chip_info.dsram_debug_addr.output_data_sel, wdata, sizeof(wdata));

#ifdef JD_RAWDATA_V2
	if (pjadard_ts_data->debug_diag_apk_enable) {
		status[0] = 0x55;
		status[1] = 0xAA;
		jd9365tx_WriteRegMulti(g_jd9365tx_chip_info.dsram_host_addr.data_output_v2, status, sizeof(status));
	} else {
		status[0] = 0x00;
		status[1] = 0x00;
		jd9365tx_WriteRegMulti(g_jd9365tx_chip_info.dsram_host_addr.data_output_v2, status, sizeof(status));
		/* Clear handshark status for START */
		status[0] = 0xA5;
		status[1] = 0x5A;
		jd9365tx_WriteRegMulti(g_jd9365tx_chip_info.dsram_debug_addr.output_data_handshake, status, sizeof(status));
	}
#else
	/* Clear handshark status for START */
	status[0] = 0xA5;
	status[1] = 0x5A;
	jd9365tx_WriteRegMulti(g_jd9365tx_chip_info.dsram_debug_addr.output_data_handshake, status, sizeof(status));
#endif
}

static int module_jd9365tx_get_mutual_data(uint8_t data_type, uint8_t *rdata, uint16_t rlen)
{
	int ReCode;
	int DataCnt = 0;
	uint8_t status[JD_TWO_SIZE];

	do {
		mdelay(1);

		if (DataCnt++ > JD9365TX_DATA_TIMEOUT) {
			JD_I("Scan data timeout, DataCnt = %d\n", DataCnt);
			jd9365tx_EnterBackDoor(NULL);
			return JD_TIME_OUT;
		}

		jd9365tx_ReadRegMulti(g_jd9365tx_chip_info.dsram_debug_addr.output_data_handshake, status, sizeof(status));
	} while (((status[1] << 8) + status[0]) != JD9365TX_OUTPUT_DATA_HANDSHAKE_DONE);

	/* Get data from IC */
	if ((data_type >= JD_DATA_TYPE_RawData) && (data_type <= JD_DATA_TYPE_RESERVE_F)) {
		ReCode = jd9365tx_ReadRegMulti(g_jd9365tx_chip_info.dsram_debug_addr.output_data_addr, rdata, rlen);
		/* Clear handshark status */
		status[0] = 0xA5;
		status[1] = 0x5A;
		jd9365tx_WriteRegMulti(g_jd9365tx_chip_info.dsram_debug_addr.output_data_handshake, status, sizeof(status));
#ifdef JD_FPGA_WORKAROUND
		/* FPGA need delay after handshake */
		mdelay(50);
#endif
	} else {
		JD_E("%s: Not support data type = %d\n", __func__, data_type);
		ReCode = -EINVAL;
	}

	return ReCode;
}

static bool module_jd9365tx_get_touch_data(uint8_t *rdata, uint8_t rlen)
{
	if (jd9365tx_ReadTpInfoBuf(rdata, rlen) < 0)
		return false;
	else
		return true;
}

static int module_jd9365tx_read_mutual_data(uint8_t *rdata, uint16_t rlen)
{
	int ReCode;

	if (g_jd9365tx_chip_info.dsram_debug_addr.output_data_addr > g_jd9365tx_chip_info.esram_info_content_addr.coordinate_report) {
		ReCode = jd9365tx_ReadRegMulti(g_jd9365tx_chip_info.esram_info_content_addr.coordinate_report, rdata, rlen);
		if (ReCode >= 0)
			ReCode = JD_REPORT_DATA;
	} else {
		ReCode = jd9365tx_ReadRegMulti(g_jd9365tx_chip_info.dsram_debug_addr.output_data_addr, rdata, rlen);
	}

	return ReCode;
}

#ifdef JD_HIGH_SENSITIVITY
static void module_jd9365tx_set_high_sensitivity(bool enable)
{
	uint8_t wdata[JD_TWO_SIZE] = {0x00, 0x00};

	if (enable) {
		wdata[0] = 0x3A;
		wdata[1] = 0xA3;
	}
	module_jd9365tx_register_write(g_jd9365tx_chip_info.dsram_host_addr.high_sensitivity_en, wdata, sizeof(wdata));
}
#endif

#ifdef JD_ROTATE_BORDER
static void module_jd9365tx_set_rotate_border(uint16_t rotate)
{
	uint8_t wdata[JD_TWO_SIZE] = {0x00, 0x00};

	wdata[0] = (uint8_t)((rotate & 0x00FF) >> 0);
	wdata[1] = (uint8_t)((rotate & 0xFF00) >> 8);
	module_jd9365tx_register_write(g_jd9365tx_chip_info.dsram_host_addr.border_en, wdata, sizeof(wdata));
}
#endif

#ifdef JD_EARPHONE_DETECT
static void module_jd9365tx_set_earphone_enable(uint8_t status)
{
	uint8_t wdata[JD_TWO_SIZE] = {0x00, 0x00};

	switch (status) {
	case 1: /* earphone removed */
		pjadard_ts_data->earphone_enable &=	 0xFF00;
		break;
	case 2: /* earphone detected */
		pjadard_ts_data->earphone_enable &=	 0xFF00;
		pjadard_ts_data->earphone_enable |=	 0x0001;
		break;
	case 3: /* type-c_earphone removed */
		pjadard_ts_data->earphone_enable &=	 0x00FF;
		break;
	case 4: /* type-c_earphone detected */
		pjadard_ts_data->earphone_enable &=	 0x00FF;
		pjadard_ts_data->earphone_enable |=	 0x0100;
		break;
	default:
		JD_E("%s: unknown status\n", __func__);
		break;
	}

	wdata[0] = (uint8_t)((pjadard_ts_data->earphone_enable & 0x00FF) >> 0);
	wdata[1] = (uint8_t)((pjadard_ts_data->earphone_enable & 0xFF00) >> 8);
	JD_D("%s: earphone = %d, typec_earphone = %d\n", __func__, wdata[0], wdata[1]);

	module_jd9365tx_register_write(g_jd9365tx_chip_info.dsram_host_addr.earphone_en, wdata, sizeof(wdata));
}
#endif

static void module_jd9365tx_set_SMWP_enable(bool enable)
{
	uint8_t wdata[JD_TWO_SIZE] = {0x00, 0x00};

	if (enable)
		wdata[0] = 1;
	else
		wdata[0] = 0;

	module_jd9365tx_register_write(g_jd9365tx_chip_info.dsram_host_addr.gesture_en, wdata, sizeof(wdata));
}

static void module_jd9365tx_usb_detect_set(uint8_t *usb_status)
{
	uint8_t wdata[JD_TWO_SIZE] = {0x00, 0x00};

	wdata[0] = usb_status[1];
	module_jd9365tx_register_write(g_jd9365tx_chip_info.dsram_host_addr.usb_en, wdata, sizeof(wdata));
}

static uint8_t module_jd9365tx_get_freq_band(void)
{
	uint8_t rdata[JD_TWO_SIZE];

	module_jd9365tx_register_read(g_jd9365tx_chip_info.dsram_debug_addr.freq_band, rdata, sizeof(rdata));
	return rdata[0];
}

#if (JD_PROXIMITY_MODE != JD_PROXIMITY_DISABLE)
static void module_jd9365tx_set_virtual_proximity(bool enable)
{
	uint8_t wdata[JD_TWO_SIZE] = {0x00, 0x00};

	if (enable) {
		wdata[0] = 0x3A;
		wdata[1] = 0xA3;
	}
	module_jd9365tx_register_write(g_jd9365tx_chip_info.dsram_host_addr.proximity_en, wdata, sizeof(wdata));
}
#endif

#ifdef CONFIG_TOUCHSCREEN_JADARD_SORTING
static void jd9365tx_SetMpapPwSync(uint8_t value)
{
	uint8_t wdata[JD_TWO_SIZE] = {0x00, 0x00};

	wdata[0] = value;
	module_jd9365tx_register_write(g_jd9365tx_chip_info.dsram_host_addr.mpap_pw_sync, wdata, sizeof(wdata));
}

static void module_jd9365tx_APP_SetSortingMode(uint8_t *value, uint8_t value_len)
{
	jd9365tx_SetMpapPwSync(0);
	module_jd9365tx_register_write(g_jd9365tx_chip_info.dsram_host_addr.mpap_pw, value, value_len);
	msleep(50);
	jd9365tx_SetMpapPwSync(1);
}

static void module_jd9365tx_APP_ReadSortingMode(uint8_t *pValue, uint8_t pValue_len)
{
	module_jd9365tx_register_read(g_jd9365tx_chip_info.dsram_host_addr.mpap_pw, pValue, pValue_len);
}

static void module_jd9365tx_APP_GetLcdSleep(uint8_t *pStatus)
{
	jd9365tx_ReadRegSingle((uint32_t)JD9365TX_SOC_REG_ADDR_SLPOUT, pStatus);
}

static void module_jd9365tx_APP_SetSortingSkipFrame(uint8_t value)
{
	jd9365tx_WriteRegSingle(g_jd9365tx_chip_info.dsram_host_addr.mpap_skip_frame, value);
}

static void module_jd9365tx_APP_SetSortingKeepFrame(uint8_t value)
{
	jd9365tx_WriteRegSingle(g_jd9365tx_chip_info.dsram_host_addr.mpap_keep_frame, value);
}

static bool module_jd9365tx_APP_ReadSortingBusyStatus(uint8_t mpap_handshake_finish, uint8_t *pStatus)
{
	jd9365tx_ReadRegSingle(g_jd9365tx_chip_info.dsram_host_addr.mpap_handshake, pStatus);

	if (*pStatus == mpap_handshake_finish)
		return true;
	else
		return false;
}

static void module_jd9365tx_GetSortingDiffDataMax(uint8_t *pValue, uint16_t pValue_len)
{
	module_jd9365tx_register_read(g_jd9365tx_chip_info.dsram_host_addr.mpap_diff_max, pValue, pValue_len);
}

static void module_jd9365tx_GetSortingDiffDataMin(uint8_t *pValue, uint16_t pValue_len)
{
	module_jd9365tx_register_read(g_jd9365tx_chip_info.dsram_host_addr.mpap_diff_min, pValue, pValue_len);
}

static void module_jd9365tx_Fw_DBIC_Off(void)
{
	uint8_t wBuf[JD_TWO_SIZE];

	wBuf[0] = 0x24;
	wBuf[1] = 0x02;
	module_jd9365tx_register_write(g_jd9365tx_chip_info.dsram_host_addr.fw_dbic_en, wBuf, sizeof(wBuf));
}

static void module_jd9365tx_Fw_DBIC_On(void)
{
	uint8_t wBuf[JD_TWO_SIZE];

	wBuf[0] = 0xFF;
	wBuf[1] = 0xFF;
	module_jd9365tx_register_write(g_jd9365tx_chip_info.dsram_host_addr.fw_dbic_en, wBuf, sizeof(wBuf));
}

static void module_jd9365tx_StartMCU(void)
{
	jd9365tx_StartMCU();
}

static void module_jd9365tx_ReadMpapErrorMsg(uint8_t *pValue, uint8_t pValue_len)
{
	module_jd9365tx_register_read(g_jd9365tx_chip_info.dsram_host_addr.mpap_error_msg, pValue, pValue_len);
}
#endif

static void jd9365tx_func_reinit(void)
{
	JD_D("%s:Entering!\n", __func__);
	g_module_fp.fp_EnterBackDoor		= module_jd9365tx_EnterBackDoor;
	g_module_fp.fp_ExitBackDoor			= module_jd9365tx_ExitBackDoor;
	g_module_fp.fp_soft_reset			= module_jd9365tx_SoftReset;
	g_module_fp.fp_ResetMCU				= module_jd9365tx_ResetMCU;
	g_module_fp.fp_PorInit				= module_jd9365tx_PorInit;
	g_module_fp.fp_register_read		= module_jd9365tx_register_read;
	g_module_fp.fp_register_write		= module_jd9365tx_register_write;
#ifndef JD_ZERO_FLASH
	g_module_fp.fp_flash_read			= module_jd9365tx_flash_read;
	g_module_fp.fp_flash_write			= module_jd9365tx_flash_write;
#endif
	g_module_fp.fp_ram_read				= module_jd9365tx_ram_read;
#ifdef JD_ZERO_FLASH
	g_module_fp.fp_ram_write			= module_jd9365tx_ram_write;
#endif
	g_module_fp.fp_read_fw_ver			= module_jd9365tx_read_fw_ver;
#ifdef JD_AUTO_UPGRADE_FW
	g_module_fp.fp_read_fw_ver_bin		= module_jd9365tx_read_fw_ver_bin;
#endif
	g_module_fp.fp_mutual_data_set		= module_jd9365tx_mutual_data_set;
	g_module_fp.fp_get_mutual_data		= module_jd9365tx_get_mutual_data;
	g_module_fp.fp_get_touch_data		= module_jd9365tx_get_touch_data;
	g_module_fp.fp_read_mutual_data		= module_jd9365tx_read_mutual_data;
#ifdef JD_HIGH_SENSITIVITY
	g_module_fp.fp_set_high_sensitivity = module_jd9365tx_set_high_sensitivity;
#endif
#ifdef JD_ROTATE_BORDER
	g_module_fp.fp_set_rotate_border	= module_jd9365tx_set_rotate_border;
#endif
#ifdef JD_EARPHONE_DETECT
	g_module_fp.fp_set_earphone_enable	= module_jd9365tx_set_earphone_enable;
#endif
	g_module_fp.fp_set_SMWP_enable		= module_jd9365tx_set_SMWP_enable;
	g_module_fp.fp_usb_detect_set		= module_jd9365tx_usb_detect_set;
	g_module_fp.fp_get_freq_band		= module_jd9365tx_get_freq_band;
#if (JD_PROXIMITY_MODE != JD_PROXIMITY_DISABLE)
	g_module_fp.fp_set_virtual_proximity = module_jd9365tx_set_virtual_proximity;
#endif
#ifdef CONFIG_TOUCHSCREEN_JADARD_SORTING
	g_module_fp.fp_APP_SetSortingMode		 = module_jd9365tx_APP_SetSortingMode;
	g_module_fp.fp_APP_ReadSortingMode		 = module_jd9365tx_APP_ReadSortingMode;
	g_module_fp.fp_APP_GetLcdSleep			 = module_jd9365tx_APP_GetLcdSleep;
	g_module_fp.fp_APP_SetSortingSkipFrame	 = module_jd9365tx_APP_SetSortingSkipFrame;
	g_module_fp.fp_APP_SetSortingKeepFrame	 = module_jd9365tx_APP_SetSortingKeepFrame;
	g_module_fp.fp_APP_ReadSortingBusyStatus = module_jd9365tx_APP_ReadSortingBusyStatus;
	/*
	 * Using new function g_module_fp.fp_GetSortingDiffDataMax &
	 *					  g_module_fp.fp_GetSortingDiffDataMin
	 * when g_module_fp.fp_GetSortingDiffData = NULL
	 */
	g_module_fp.fp_GetSortingDiffData		 = NULL;
	g_module_fp.fp_GetSortingDiffDataMax	 = module_jd9365tx_GetSortingDiffDataMax;
	g_module_fp.fp_GetSortingDiffDataMin	 = module_jd9365tx_GetSortingDiffDataMin;
	g_module_fp.fp_Fw_DBIC_Off				 = module_jd9365tx_Fw_DBIC_Off;
	g_module_fp.fp_Fw_DBIC_On				 = module_jd9365tx_Fw_DBIC_On;
	g_module_fp.fp_StartMCU					 = module_jd9365tx_StartMCU;
	g_module_fp.fp_ReadMpapErrorMsg			 = module_jd9365tx_ReadMpapErrorMsg;
#endif
}

static bool module_jd9365tx_chip_detect(void)
{
	int i = 0;
	bool ret = false;
	uint16_t romid = 0;
	uint16_t CHIP_ID = JD9365TX_ID;

	g_common_variable.dbi_dd_reg_mode = JD9365TX_DDREG_MODE;
	if (g_common_variable.dbi_dd_reg_mode != JD_DDREG_MODE_1) {
		JD_E("%s:Don't support this mode ,dbi_dd_reg_mode %02X not equal %02X\n",
			__func__, g_common_variable.dbi_dd_reg_mode, JD_DDREG_MODE_1);
		return false;
	}

	do {
		/* EnterBackDoor get IC ID */
		jd9365tx_EnterBackDoor(&romid);

		if (romid == CHIP_ID) {
			JD_I("%s: detect IC %s successfully\n", __func__, JD9365TX_OUTER_ID);
			scnprintf(pjadard_ic_data->chip_id, sizeof(pjadard_ic_data->chip_id), JD9365TX_OUTER_ID);
			jd9365tx_func_reinit();
			ret = true;
		} else {
			JD_E("%s:Read driver ID %04X not equal %04X\n", __func__, romid, CHIP_ID);
			msleep(500);
		}
	} while ((++i < 3) && (ret == false));

	return ret;
}

#ifdef CONFIG_CHIP_DTCFG
static int jadard_jd9365tx_probe(struct platform_device *pdev)
{
	struct jadard_support_chip *pChip = NULL;
	struct jadard_support_chip *ptr = NULL;

	JD_I("%s:Enter\n", __func__);

	pChip = kzalloc(sizeof(struct jadard_support_chip), GFP_KERNEL);
	if (pChip == NULL) {
		JD_E("%s: allocate jadard_ts_data failed\n", __func__);
		return -ENOMEM;
	}

	/* Store chip init and detect funtion */
	pChip->chip_init = module_jd9365tx_chip_init;
	pChip->chip_detect = module_jd9365tx_chip_detect;
	pChip->next = NULL;

	if (g_module_fp.head_support_chip == NULL) {
		g_module_fp.head_support_chip = pChip;
	} else {
		ptr = g_module_fp.head_support_chip;
		/* Find the last one */
		while (ptr->next != NULL)
			ptr = ptr->next;
		/* Link to the last */
		ptr->next = pChip;
	}

	return 0;
}

static int jadard_jd9365tx_remove(struct platform_device *pdev)
{
	struct jadard_support_chip *ptr = g_module_fp.head_support_chip;
	struct jadard_support_chip *pre_ptr = NULL;

	while (ptr != NULL) {
		if ((ptr->chip_init == module_jd9365tx_chip_init) &&
			(ptr->chip_detect == module_jd9365tx_chip_detect)) {
			if (ptr == g_module_fp.head_support_chip)
				g_module_fp.head_support_chip = ptr->next;
			else
				pre_ptr->next = ptr->next;

			kfree(ptr);
			break;
		}

		pre_ptr = ptr;
		ptr = ptr->next;
	}

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id jadard_jd9365tx_mttable[] = {
	{ .compatible = "jadard,jd9365tx"},
	{ },
};
#else
#define jadard_jd9365tx_mttable NULL
#endif

static struct platform_driver jadard_jd9365tx_driver = {
	.probe	= jadard_jd9365tx_probe,
	.remove = jadard_jd9365tx_remove,
	.driver = {
		.name			= "JADARD_JD9365TX",
		.owner			= THIS_MODULE,
		.of_match_table = jadard_jd9365tx_mttable,
	},
};

static int __init jadard_jd9365tx_init(void)
{
	JD_I("%s\n", __func__);
	platform_driver_register(&jadard_jd9365tx_driver);

	return 0;
}

static void __exit jadard_jd9365tx_exit(void)
{
	platform_driver_unregister(&jadard_jd9365tx_driver);
}

#else
static int jadard_jd9365tx_probe(void)
{
	struct jadard_support_chip *pChip = NULL;
	struct jadard_support_chip *ptr = NULL;

	JD_I("%s:Enter\n", __func__);

	pChip = kzalloc(sizeof(struct jadard_support_chip), GFP_KERNEL);
	if (pChip == NULL) {
		JD_E("%s: allocate jadard_ts_data failed\n", __func__);
		return -ENOMEM;
	}

	/* Store chip init and detect funtion */
	pChip->chip_init = module_jd9365tx_chip_init;
	pChip->chip_detect = module_jd9365tx_chip_detect;
	pChip->next = NULL;

	if (g_module_fp.head_support_chip == NULL) {
		g_module_fp.head_support_chip = pChip;
	} else {
		ptr = g_module_fp.head_support_chip;
		/* Find the last one */
		while (ptr->next != NULL)
			ptr = ptr->next;
		/* Link to the last */
		ptr->next = pChip;
	}

	return 0;
}

static int jadard_jd9365tx_remove(void)
{
	struct jadard_support_chip *ptr = g_module_fp.head_support_chip;
	struct jadard_support_chip *pre_ptr = NULL;

	while (ptr != NULL) {
		if ((ptr->chip_init == module_jd9365tx_chip_init) &&
			(ptr->chip_detect == module_jd9365tx_chip_detect)) {
			if (ptr == g_module_fp.head_support_chip)
				g_module_fp.head_support_chip = ptr->next;
			else
				pre_ptr->next = ptr->next;

			kfree(ptr);
			break;
		}

		pre_ptr = ptr;
		ptr = ptr->next;
	}

	return 0;
}

static int __init jadard_jd9365tx_init(void)
{
	JD_I("%s\n", __func__);
	jadard_jd9365tx_probe();
#if defined(__JADARD_KMODULE__)
	jadard_common_init();
#endif

	return 0;
}

static void __exit jadard_jd9365tx_exit(void)
{
	jadard_jd9365tx_remove();
#if defined(__JADARD_KMODULE__)
	jadard_common_exit();
#endif
}
#endif

module_init(jadard_jd9365tx_init);
module_exit(jadard_jd9365tx_exit);

MODULE_DESCRIPTION("JADARD JD9365TX touch driver");
MODULE_LICENSE("GPL");
