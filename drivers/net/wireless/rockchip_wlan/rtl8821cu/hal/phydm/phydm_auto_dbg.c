/******************************************************************************
 *
 * Copyright(c) 2007 - 2017  Realtek Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * The full GNU General Public License is included in this distribution in the
 * file called LICENSE.
 *
 * Contact Information:
 * wlanfae <wlanfae@realtek.com>
 * Realtek Corporation, No. 2, Innovation Road II, Hsinchu Science Park,
 * Hsinchu 300, Taiwan.
 *
 * Larry Finger <Larry.Finger@lwfinger.net>
 *
 *****************************************************************************/

/*************************************************************
 * include files
 ************************************************************/

#include "mp_precomp.h"
#include "phydm_precomp.h"

#ifdef PHYDM_AUTO_DEGBUG

void phydm_check_hang_reset(
	void *dm_void)
{
	struct dm_struct *dm = (struct dm_struct *)dm_void;
	struct phydm_auto_dbg_struct *atd_t = &dm->auto_dbg_table;

	atd_t->dbg_step = 0;
	atd_t->auto_dbg_type = AUTO_DBG_STOP;
	phydm_pause_dm_watchdog(dm, PHYDM_RESUME);
	dm->debug_components &= (~ODM_COMP_API);
}

void phydm_check_hang_init(
	void *dm_void)
{
	struct dm_struct *dm = (struct dm_struct *)dm_void;
	struct phydm_auto_dbg_struct *atd_t = &dm->auto_dbg_table;

	atd_t->dbg_step = 0;
	atd_t->auto_dbg_type = AUTO_DBG_STOP;
	phydm_pause_dm_watchdog(dm, PHYDM_RESUME);
}

#if (ODM_IC_11N_SERIES_SUPPORT == 1)
void phydm_auto_check_hang_engine_n(
	void *dm_void)
{
	struct dm_struct *dm = (struct dm_struct *)dm_void;
	struct phydm_auto_dbg_struct *atd_t = &dm->auto_dbg_table;
	struct n_dbgport_803 dbgport_803 = {0};
	u32 value32_tmp = 0, value32_tmp_2 = 0;
	u8 i;
	u32 curr_dbg_port_val[DBGPORT_CHK_NUM] = {0, 0, 0, 0, 0, 0};
	u16 curr_ofdm_t_cnt;
	u16 curr_ofdm_r_cnt;
	u16 curr_cck_t_cnt;
	u16 curr_cck_r_cnt;
	u16 curr_ofdm_crc_error_cnt;
	u16 curr_cck_crc_error_cnt;
	u16 diff_ofdm_t_cnt;
	u16 diff_ofdm_r_cnt;
	u16 diff_cck_t_cnt;
	u16 diff_cck_r_cnt;
	u16 diff_ofdm_crc_error_cnt;
	u16 diff_cck_crc_error_cnt;
	u8 rf_mode;

	if (atd_t->auto_dbg_type == AUTO_DBG_STOP)
		return;

	if (dm->support_ic_type & ODM_IC_11AC_SERIES) {
		phydm_check_hang_reset(dm);
		return;
	}

	if (atd_t->dbg_step == 0) {
		pr_debug("dbg_step=0\n\n");

		/*Reset all packet counter*/
		odm_set_bb_reg(dm, R_0xf14, BIT(16), 1);
		odm_set_bb_reg(dm, R_0xf14, BIT(16), 0);

	} else if (atd_t->dbg_step == 1) {
		pr_debug("dbg_step=1\n\n");

		/*Check packet counter Register*/
		atd_t->ofdm_t_cnt = (u16)odm_get_bb_reg(dm, R_0x9cc, MASKHWORD);
		atd_t->ofdm_r_cnt = (u16)odm_get_bb_reg(dm, R_0xf94, MASKLWORD);
		atd_t->ofdm_crc_error_cnt = (u16)odm_get_bb_reg(dm, R_0xf94,
								MASKHWORD);

		atd_t->cck_t_cnt = (u16)odm_get_bb_reg(dm, R_0x9d0, MASKHWORD);
		atd_t->cck_r_cnt = (u16)odm_get_bb_reg(dm, R_0xfa0, MASKHWORD);
		atd_t->cck_crc_error_cnt = (u16)odm_get_bb_reg(dm, R_0xf84,
							       0x3fff);

		/*Check Debug Port*/
		for (i = 0; i < DBGPORT_CHK_NUM; i++) {
			if (phydm_set_bb_dbg_port(dm, DBGPORT_PRI_3,
						  (u32)atd_t->dbg_port_table[i])
						  ) {
				atd_t->dbg_port_val[i] =
					phydm_get_bb_dbg_port_val(dm);
				phydm_release_bb_dbg_port(dm);
			}
		}

	} else if (atd_t->dbg_step == 2) {
		pr_debug("dbg_step=2\n\n");

		/*Check packet counter Register*/
		curr_ofdm_t_cnt = (u16)odm_get_bb_reg(dm, R_0x9cc, MASKHWORD);
		curr_ofdm_r_cnt = (u16)odm_get_bb_reg(dm, R_0xf94, MASKLWORD);
		curr_ofdm_crc_error_cnt = (u16)odm_get_bb_reg(dm, R_0xf94,
							      MASKHWORD);

		curr_cck_t_cnt = (u16)odm_get_bb_reg(dm, R_0x9d0, MASKHWORD);
		curr_cck_r_cnt = (u16)odm_get_bb_reg(dm, R_0xfa0, MASKHWORD);
		curr_cck_crc_error_cnt = (u16)odm_get_bb_reg(dm, R_0xf84,
							     0x3fff);

		/*Check Debug Port*/
		for (i = 0; i < DBGPORT_CHK_NUM; i++) {
			if (phydm_set_bb_dbg_port(dm, DBGPORT_PRI_3,
						  (u32)atd_t->dbg_port_table[i])
						  ) {
				curr_dbg_port_val[i] =
					phydm_get_bb_dbg_port_val(dm);
				phydm_release_bb_dbg_port(dm);
			}
		}

		/*=== Make check hang decision ===============================*/
		pr_debug("Check Hang Decision\n\n");

		/* ----- Check RF Register -----------------------------------*/
		for (i = 0; i < dm->num_rf_path; i++) {
			rf_mode = (u8)odm_get_rf_reg(dm, i, RF_0x0, 0xf0000);
			pr_debug("RF0x0[%d] = 0x%x\n", i, rf_mode);
			if (rf_mode > 3) {
				pr_debug("Incorrect RF mode\n");
				pr_debug("ReasonCode:RHN-1\n");
			}
		}
		value32_tmp = odm_get_rf_reg(dm, 0, RF_0xb0, 0xf0000);
		if (dm->support_ic_type == ODM_RTL8188E) {
			if (value32_tmp != 0xff8c8) {
				pr_debug("ReasonCode:RHN-3\n");
			}
		}
		/* ----- Check BB Register ----------------------------------*/
		/*BB mode table*/
		value32_tmp = odm_get_bb_reg(dm, R_0x824, 0xe);
		value32_tmp_2 = odm_get_bb_reg(dm, R_0x82c, 0xe);
		pr_debug("BB TX mode table {A, B}= {%d, %d}\n",
			 value32_tmp, value32_tmp_2);

		if (value32_tmp > 3 || value32_tmp_2 > 3) {
			pr_debug("ReasonCode:RHN-2\n");
		}

		value32_tmp = odm_get_bb_reg(dm, R_0x824, 0x700000);
		value32_tmp_2 = odm_get_bb_reg(dm, R_0x82c, 0x700000);
		pr_debug("BB RX mode table {A, B}= {%d, %d}\n", value32_tmp,
			 value32_tmp_2);

		if (value32_tmp > 3 || value32_tmp_2 > 3) {
			pr_debug("ReasonCode:RHN-2\n");
		}

		/*BB HW Block*/
		value32_tmp = odm_get_bb_reg(dm, R_0x800, MASKDWORD);

		if (!(value32_tmp & BIT(24))) {
			pr_debug("Reg0x800[24] = 0, CCK BLK is disabled\n");
			pr_debug("ReasonCode: THN-3\n");
		}

		if (!(value32_tmp & BIT(25))) {
			pr_debug("Reg0x800[24] = 0, OFDM BLK is disabled\n");
			pr_debug("ReasonCode:THN-3\n");
		}

		/*BB Continue TX*/
		value32_tmp = odm_get_bb_reg(dm, R_0xd00, 0x70000000);
		pr_debug("Continue TX=%d\n", value32_tmp);
		if (value32_tmp != 0) {
			pr_debug("ReasonCode: THN-4\n");
		}

		/* ----- Check Packet Counter --------------------------------*/
		diff_ofdm_t_cnt = curr_ofdm_t_cnt - atd_t->ofdm_t_cnt;
		diff_ofdm_r_cnt = curr_ofdm_r_cnt - atd_t->ofdm_r_cnt;
		diff_ofdm_crc_error_cnt = curr_ofdm_crc_error_cnt -
					  atd_t->ofdm_crc_error_cnt;

		diff_cck_t_cnt = curr_cck_t_cnt - atd_t->cck_t_cnt;
		diff_cck_r_cnt = curr_cck_r_cnt - atd_t->cck_r_cnt;
		diff_cck_crc_error_cnt = curr_cck_crc_error_cnt -
					 atd_t->cck_crc_error_cnt;

		pr_debug("OFDM[t=0~1] {TX, RX, CRC_error} = {%d, %d, %d}\n",
			 atd_t->ofdm_t_cnt, atd_t->ofdm_r_cnt,
			 atd_t->ofdm_crc_error_cnt);
		pr_debug("OFDM[t=1~2] {TX, RX, CRC_error} = {%d, %d, %d}\n",
			 curr_ofdm_t_cnt, curr_ofdm_r_cnt,
			 curr_ofdm_crc_error_cnt);
		pr_debug("OFDM_diff {TX, RX, CRC_error} = {%d, %d, %d}\n",
			 diff_ofdm_t_cnt, diff_ofdm_r_cnt,
			 diff_ofdm_crc_error_cnt);

		pr_debug("CCK[t=0~1] {TX, RX, CRC_error} = {%d, %d, %d}\n",
			 atd_t->cck_t_cnt, atd_t->cck_r_cnt,
			 atd_t->cck_crc_error_cnt);
		pr_debug("CCK[t=1~2] {TX, RX, CRC_error} = {%d, %d, %d}\n",
			 curr_cck_t_cnt, curr_cck_r_cnt,
			 curr_cck_crc_error_cnt);
		pr_debug("CCK_diff {TX, RX, CRC_error} = {%d, %d, %d}\n",
			 diff_cck_t_cnt, diff_cck_r_cnt,
			 diff_cck_crc_error_cnt);

		/* ----- Check Dbg Port --------------------------------*/

		for (i = 0; i < DBGPORT_CHK_NUM; i++) {
			pr_debug("Dbg_port=((0x%x))\n",
				 atd_t->dbg_port_table[i]);
			pr_debug("Val{pre, curr}={0x%x, 0x%x}\n",
				 atd_t->dbg_port_val[i], curr_dbg_port_val[i]);

			if (atd_t->dbg_port_table[i] == 0) {
				if (atd_t->dbg_port_val[i] ==
				    curr_dbg_port_val[i]) {
					pr_debug("BB state hang\n");
					pr_debug("ReasonCode:\n");
				}

			} else if (atd_t->dbg_port_table[i] == 0x803) {
				if (atd_t->dbg_port_val[i] ==
				    curr_dbg_port_val[i]) {
					/* dbgport_803 =  */
					/* (struct n_dbgport_803 )   */
					/* (atd_t->dbg_port_val[i]); */
					odm_move_memory(dm, &dbgport_803,
							&atd_t->dbg_port_val[i],
							sizeof(struct n_dbgport_803));
					pr_debug("RSTB{BB, GLB, OFDM}={%d, %d,%d}\n",
						 dbgport_803.bb_rst_b,
						 dbgport_803.glb_rst_b,
						 dbgport_803.ofdm_rst_b);
					pr_debug("{ofdm_tx_en, cck_tx_en, phy_tx_on}={%d, %d, %d}\n",
						 dbgport_803.ofdm_tx_en,
						 dbgport_803.cck_tx_en,
						 dbgport_803.phy_tx_on);
					pr_debug("CCA_PP{OFDM, CCK}={%d, %d}\n",
						 dbgport_803.ofdm_cca_pp,
						 dbgport_803.cck_cca_pp);

					if (dbgport_803.phy_tx_on)
						pr_debug("Maybe TX Hang\n");
					else if (dbgport_803.ofdm_cca_pp ||
						 dbgport_803.cck_cca_pp)
						pr_debug("Maybe RX Hang\n");
				}

			} else if (atd_t->dbg_port_table[i] == 0x208) {
				if ((atd_t->dbg_port_val[i] & BIT(30)) &&
				    (curr_dbg_port_val[i] & BIT(30))) {
					pr_debug("EDCCA Pause TX\n");
					pr_debug("ReasonCode: THN-2\n");
				}

			} else if (atd_t->dbg_port_table[i] == 0xab0) {
				/* atd_t->dbg_port_val[i] & 0xffffff == 0 */
				/* curr_dbg_port_val[i] & 0xffffff == 0 */
				if (((atd_t->dbg_port_val[i] &
				      MASK24BITS) == 0) ||
				    ((curr_dbg_port_val[i] &
				      MASK24BITS) == 0)) {
					pr_debug("Wrong L-SIG formate\n");
					pr_debug("ReasonCode: THN-1\n");
				}
			}
		}

		phydm_check_hang_reset(dm);
	}

	atd_t->dbg_step++;
}

void phydm_bb_auto_check_hang_start_n(
	void *dm_void,
	u32 *_used,
	char *output,
	u32 *_out_len)
{
	u32 value32 = 0;
	struct dm_struct *dm = (struct dm_struct *)dm_void;
	struct phydm_auto_dbg_struct *atd_t = &dm->auto_dbg_table;
	u32 used = *_used;
	u32 out_len = *_out_len;

	if (dm->support_ic_type & ODM_IC_11AC_SERIES)
		return;

	PDM_SNPF(out_len, used, output + used, out_len - used,
		 "PHYDM auto check hang (N-series) is started, Please check the system log\n");

	dm->debug_components |= ODM_COMP_API;
	atd_t->auto_dbg_type = AUTO_DBG_CHECK_HANG;
	atd_t->dbg_step = 0;

	phydm_pause_dm_watchdog(dm, PHYDM_PAUSE);

	*_used = used;
	*_out_len = out_len;
}

void phydm_dbg_port_dump_n(void *dm_void, u32 *_used, char *output,
			   u32 *_out_len)
{
	u32 value32 = 0;
	struct dm_struct *dm = (struct dm_struct *)dm_void;
	u32 used = *_used;
	u32 out_len = *_out_len;

	if (dm->support_ic_type & ODM_IC_11AC_SERIES)
		return;

	PDM_SNPF(out_len, used, output + used, out_len - used,
		 "not support now\n");

	*_used = used;
	*_out_len = out_len;
}

#endif

#if (ODM_IC_11AC_SERIES_SUPPORT == 1)
void phydm_dbg_port_dump_ac(void *dm_void, u32 *_used, char *output,
			    u32 *_out_len)
{
	u32 value32 = 0;
	struct dm_struct *dm = (struct dm_struct *)dm_void;
	u32 used = *_used;
	u32 out_len = *_out_len;

	if (dm->support_ic_type & ODM_IC_11N_SERIES)
		return;

	value32 = odm_get_bb_reg(dm, R_0xf80, MASKDWORD);
	PDM_SNPF(out_len, used, output + used, out_len - used,
		 "\r\n %-35s = 0x%x", "rptreg of sc/bw/ht/...", value32);

	if (dm->support_ic_type & ODM_RTL8822B)
		odm_set_bb_reg(dm, R_0x198c, BIT(2) | BIT(1) | BIT(0), 7);

	/* dbg_port = basic state machine */
	{
		odm_set_bb_reg(dm, ODM_REG_DBG_RPT_11AC, MASKDWORD, 0x000);
		value32 = odm_get_bb_reg(dm, ODM_REG_DBG_RPT_11AC, MASKDWORD);
		PDM_SNPF(out_len, used, output + used, out_len - used,
			 "\r\n %-35s = 0x%x", "0x8fc", value32);

		value32 = odm_get_bb_reg(dm, ODM_REG_RPT_11AC, MASKDWORD);
		PDM_SNPF(out_len, used, output + used, out_len - used,
			 "\r\n %-35s = 0x%x", "basic state machine", value32);
	}

	/* dbg_port = state machine */
	{
		odm_set_bb_reg(dm, ODM_REG_DBG_RPT_11AC, MASKDWORD, 0x007);
		value32 = odm_get_bb_reg(dm, ODM_REG_DBG_RPT_11AC, MASKDWORD);
		PDM_SNPF(out_len, used, output + used, out_len - used,
			 "\r\n %-35s = 0x%x", "0x8fc", value32);

		value32 = odm_get_bb_reg(dm, ODM_REG_RPT_11AC, MASKDWORD);
		PDM_SNPF(out_len, used, output + used, out_len - used,
			 "\r\n %-35s = 0x%x", "state machine", value32);
	}

	/* dbg_port = CCA-related*/
	{
		odm_set_bb_reg(dm, ODM_REG_DBG_RPT_11AC, MASKDWORD, 0x204);
		value32 = odm_get_bb_reg(dm, ODM_REG_DBG_RPT_11AC, MASKDWORD);
		PDM_SNPF(out_len, used, output + used, out_len - used,
			 "\r\n %-35s = 0x%x", "0x8fc", value32);

		value32 = odm_get_bb_reg(dm, ODM_REG_RPT_11AC, MASKDWORD);
		PDM_SNPF(out_len, used, output + used, out_len - used,
			 "\r\n %-35s = 0x%x", "CCA-related", value32);
	}

	/* dbg_port = edcca/rxd*/
	{
		odm_set_bb_reg(dm, ODM_REG_DBG_RPT_11AC, MASKDWORD, 0x278);
		value32 = odm_get_bb_reg(dm, ODM_REG_DBG_RPT_11AC, MASKDWORD);
		PDM_SNPF(out_len, used, output + used, out_len - used,
			 "\r\n %-35s = 0x%x", "0x8fc", value32);

		value32 = odm_get_bb_reg(dm, ODM_REG_RPT_11AC, MASKDWORD);
		PDM_SNPF(out_len, used, output + used, out_len - used,
			 "\r\n %-35s = 0x%x", "edcca/rxd", value32);
	}

	/* dbg_port = rx_state/mux_state/ADC_MASK_OFDM*/
	{
		odm_set_bb_reg(dm, ODM_REG_DBG_RPT_11AC, MASKDWORD, 0x290);
		value32 = odm_get_bb_reg(dm, ODM_REG_DBG_RPT_11AC, MASKDWORD);
		PDM_SNPF(out_len, used, output + used, out_len - used,
			 "\r\n %-35s = 0x%x", "0x8fc", value32);

		value32 = odm_get_bb_reg(dm, ODM_REG_RPT_11AC, MASKDWORD);
		PDM_SNPF(out_len, used, output + used, out_len - used,
			 "\r\n %-35s = 0x%x",
			 "rx_state/mux_state/ADC_MASK_OFDM", value32);
	}

	/* dbg_port = bf-related*/
	{
		odm_set_bb_reg(dm, ODM_REG_DBG_RPT_11AC, MASKDWORD, 0x2B2);
		value32 = odm_get_bb_reg(dm, ODM_REG_DBG_RPT_11AC, MASKDWORD);
		PDM_SNPF(out_len, used, output + used, out_len - used,
			 "\r\n %-35s = 0x%x", "0x8fc", value32);

		value32 = odm_get_bb_reg(dm, ODM_REG_RPT_11AC, MASKDWORD);
		PDM_SNPF(out_len, used, output + used, out_len - used,
			 "\r\n %-35s = 0x%x", "bf-related", value32);
	}

	/* dbg_port = bf-related*/
	{
		odm_set_bb_reg(dm, ODM_REG_DBG_RPT_11AC, MASKDWORD, 0x2B8);
		value32 = odm_get_bb_reg(dm, ODM_REG_DBG_RPT_11AC, MASKDWORD);
		PDM_SNPF(out_len, used, output + used, out_len - used,
			 "\r\n %-35s = 0x%x", "0x8fc", value32);

		value32 = odm_get_bb_reg(dm, ODM_REG_RPT_11AC, MASKDWORD);
		PDM_SNPF(out_len, used, output + used, out_len - used,
			 "\r\n %-35s = 0x%x", "bf-related", value32);
	}

	/* dbg_port = txon/rxd*/
	{
		odm_set_bb_reg(dm, ODM_REG_DBG_RPT_11AC, MASKDWORD, 0xA03);
		value32 = odm_get_bb_reg(dm, ODM_REG_DBG_RPT_11AC, MASKDWORD);
		PDM_SNPF(out_len, used, output + used, out_len - used,
			 "\r\n %-35s = 0x%x", "0x8fc", value32);

		value32 = odm_get_bb_reg(dm, ODM_REG_RPT_11AC, MASKDWORD);
		PDM_SNPF(out_len, used, output + used, out_len - used,
			 "\r\n %-35s = 0x%x", "txon/rxd", value32);
	}

	/* dbg_port = l_rate/l_length*/
	{
		odm_set_bb_reg(dm, ODM_REG_DBG_RPT_11AC, MASKDWORD, 0xA0B);
		value32 = odm_get_bb_reg(dm, ODM_REG_DBG_RPT_11AC, MASKDWORD);
		PDM_SNPF(out_len, used, output + used, out_len - used,
			 "\r\n %-35s = 0x%x", "0x8fc", value32);

		value32 = odm_get_bb_reg(dm, ODM_REG_RPT_11AC, MASKDWORD);
		PDM_SNPF(out_len, used, output + used, out_len - used,
			 "\r\n %-35s = 0x%x", "l_rate/l_length", value32);
	}

	/* dbg_port = rxd/rxd_hit*/
	{
		odm_set_bb_reg(dm, ODM_REG_DBG_RPT_11AC, MASKDWORD, 0xA0D);
		value32 = odm_get_bb_reg(dm, ODM_REG_DBG_RPT_11AC, MASKDWORD);
		PDM_SNPF(out_len, used, output + used, out_len - used,
			 "\r\n %-35s = 0x%x", "0x8fc", value32);

		value32 = odm_get_bb_reg(dm, ODM_REG_RPT_11AC, MASKDWORD);
		PDM_SNPF(out_len, used, output + used, out_len - used,
			 "\r\n %-35s = 0x%x", "rxd/rxd_hit", value32);
	}

	/* dbg_port = dis_cca*/
	{
		odm_set_bb_reg(dm, ODM_REG_DBG_RPT_11AC, MASKDWORD, 0xAA0);
		value32 = odm_get_bb_reg(dm, ODM_REG_DBG_RPT_11AC, MASKDWORD);
		PDM_SNPF(out_len, used, output + used, out_len - used,
			 "\r\n %-35s = 0x%x", "0x8fc", value32);

		value32 = odm_get_bb_reg(dm, ODM_REG_RPT_11AC, MASKDWORD);
		PDM_SNPF(out_len, used, output + used, out_len - used,
			 "\r\n %-35s = 0x%x", "dis_cca", value32);
	}

	/* dbg_port = tx*/
	{
		odm_set_bb_reg(dm, ODM_REG_DBG_RPT_11AC, MASKDWORD, 0xAB0);
		value32 = odm_get_bb_reg(dm, ODM_REG_DBG_RPT_11AC, MASKDWORD);
		PDM_SNPF(out_len, used, output + used, out_len - used,
			 "\r\n %-35s = 0x%x", "0x8fc", value32);

		value32 = odm_get_bb_reg(dm, ODM_REG_RPT_11AC, MASKDWORD);
		PDM_SNPF(out_len, used, output + used, out_len - used,
			 "\r\n %-35s = 0x%x", "tx", value32);
	}

	/* dbg_port = rx plcp*/
	{
		odm_set_bb_reg(dm, ODM_REG_DBG_RPT_11AC, MASKDWORD, 0xAD0);
		value32 = odm_get_bb_reg(dm, ODM_REG_DBG_RPT_11AC, MASKDWORD);
		PDM_SNPF(out_len, used, output + used, out_len - used,
			 "\r\n %-35s = 0x%x", "0x8fc", value32);

		value32 = odm_get_bb_reg(dm, ODM_REG_RPT_11AC, MASKDWORD);
		PDM_SNPF(out_len, used, output + used, out_len - used,
			 "\r\n %-35s = 0x%x", "rx plcp", value32);

		odm_set_bb_reg(dm, ODM_REG_DBG_RPT_11AC, MASKDWORD, 0xAD1);
		value32 = odm_get_bb_reg(dm, ODM_REG_DBG_RPT_11AC, MASKDWORD);
		PDM_SNPF(out_len, used, output + used, out_len - used,
			 "\r\n %-35s = 0x%x", "0x8fc", value32);

		value32 = odm_get_bb_reg(dm, ODM_REG_RPT_11AC, MASKDWORD);
		PDM_SNPF(out_len, used, output + used, out_len - used,
			 "\r\n %-35s = 0x%x", "rx plcp", value32);

		odm_set_bb_reg(dm, ODM_REG_DBG_RPT_11AC, MASKDWORD, 0xAD2);
		value32 = odm_get_bb_reg(dm, ODM_REG_DBG_RPT_11AC, MASKDWORD);
		PDM_SNPF(out_len, used, output + used, out_len - used,
			 "\r\n %-35s = 0x%x", "0x8fc", value32);

		value32 = odm_get_bb_reg(dm, ODM_REG_RPT_11AC, MASKDWORD);
		PDM_SNPF(out_len, used, output + used, out_len - used,
			 "\r\n %-35s = 0x%x", "rx plcp", value32);

		odm_set_bb_reg(dm, ODM_REG_DBG_RPT_11AC, MASKDWORD, 0xAD3);
		value32 = odm_get_bb_reg(dm, ODM_REG_DBG_RPT_11AC, MASKDWORD);
		PDM_SNPF(out_len, used, output + used, out_len - used,
			 "\r\n %-35s = 0x%x", "0x8fc", value32);

		value32 = odm_get_bb_reg(dm, ODM_REG_RPT_11AC, MASKDWORD);
		PDM_SNPF(out_len, used, output + used, out_len - used,
			 "\r\n %-35s = 0x%x", "rx plcp", value32);
	}
	*_used = used;
	*_out_len = out_len;
}
#endif

#ifdef PHYDM_IC_JGR3_SERIES_SUPPORT
void phydm_dbg_port_dump_jgr3(void *dm_void, u32 *_used, char *output,
			      u32 *_out_len)
{
	struct dm_struct *dm = (struct dm_struct *)dm_void;
	u32 used = *_used;
	u32 out_len = *_out_len;
	/*u32 dbg_port_idx_all[3] = {0x000, 0x001, 0x002};*/
	u32 val = 0;
	u32 dbg_port_idx = 0;
	u32 i = 0;

	if (!(dm->support_ic_type & ODM_IC_JGR3_SERIES))
		return;

	PDM_VAST_SNPF(out_len, used, output + used, out_len - used,
		      "%-17s = %s\n", "DbgPort index", "Value");

#if 0
	/*0x000/0x001/0x002*/
	for (i = 0; i < 3; i++) {
		dbg_port_idx = dbg_port_idx_all[i];
		if (phydm_set_bb_dbg_port(dm, DBGPORT_PRI_3, dbg_port_idx)) {
			val = phydm_get_bb_dbg_port_val(dm);
			PDM_SNPF(out_len, used, output + used, out_len - used,
				 "0x%-15x = 0x%x\n", dbg_port_idx, val);
			phydm_release_bb_dbg_port(dm);
		}
	}
#endif
	for (dbg_port_idx = 0x0; dbg_port_idx <= 0xfff; dbg_port_idx++) {
		if (phydm_set_bb_dbg_port(dm, DBGPORT_PRI_3, dbg_port_idx)) {
			val = phydm_get_bb_dbg_port_val(dm);
			PDM_VAST_SNPF(out_len, used, output + used,
				      out_len - used,
				      "0x%-15x = 0x%x\n", dbg_port_idx, val);
			phydm_release_bb_dbg_port(dm);
		}
	}
	*_used = used;
	*_out_len = out_len;
}
#endif

void phydm_dbg_port_dump(void *dm_void, u32 *_used, char *output, u32 *_out_len)
{
	struct dm_struct *dm = (struct dm_struct *)dm_void;
	u32 used = *_used;
	u32 out_len = *_out_len;

	PDM_VAST_SNPF(out_len, used, output + used, out_len - used,
		      "------ BB debug port start ------\n");

	switch (dm->ic_ip_series) {
	#ifdef PHYDM_IC_JGR3_SERIES_SUPPORT
	case PHYDM_IC_JGR3:
		phydm_dbg_port_dump_jgr3(dm, &used, output, &out_len);
		break;
	#endif

	#if (ODM_IC_11AC_SERIES_SUPPORT == 1)
	case PHYDM_IC_AC:
		phydm_dbg_port_dump_ac(dm, &used, output, &out_len);
		break;
	#endif

	#if (ODM_IC_11N_SERIES_SUPPORT == 1)
	case PHYDM_IC_N:
		phydm_dbg_port_dump_n(dm, &used, output, &out_len);
		break;
	#endif

	default:
		break;
	}
	*_used = used;
	*_out_len = out_len;
}

void phydm_auto_dbg_console(
	void *dm_void,
	char input[][16],
	u32 *_used,
	char *output,
	u32 *_out_len)
{
	struct dm_struct *dm = (struct dm_struct *)dm_void;
	char help[] = "-h";
	u32 var1[10] = {0};
	u32 used = *_used;
	u32 out_len = *_out_len;

	PHYDM_SSCANF(input[1], DCMD_DECIMAL, &var1[0]);

	if ((strcmp(input[1], help) == 0)) {
		PDM_SNPF(out_len, used, output + used, out_len - used,
			 "hang: {1} {1:Show DbgPort, 2:Auto check hang}\n");
		return;
	} else if (var1[0] == 1) {
		PHYDM_SSCANF(input[2], DCMD_DECIMAL, &var1[1]);
		if (var1[1] == 1) {
			phydm_dbg_port_dump(dm, &used, output, &out_len);
		} else if (var1[1] == 2) {
			if (dm->support_ic_type & ODM_IC_11AC_SERIES) {
				PDM_SNPF(out_len, used, output + used,
					 out_len - used, "Not support\n");
			} else {
				#if (ODM_IC_11N_SERIES_SUPPORT == 1)
				phydm_bb_auto_check_hang_start_n(dm, &used,
								 output,
								 &out_len);
				#else
				PDM_SNPF(out_len, used, output + used,
					 out_len - used, "Not support\n");
				#endif
			}
		}
	}

	*_used = used;
	*_out_len = out_len;
}

void phydm_auto_dbg_engine(void *dm_void)
{
	u32 value32 = 0;

	struct dm_struct *dm = (struct dm_struct *)dm_void;
	struct phydm_auto_dbg_struct *atd_t = &dm->auto_dbg_table;

	if (atd_t->auto_dbg_type == AUTO_DBG_STOP)
		return;

	pr_debug("%s ======>\n", __func__);

	if (atd_t->auto_dbg_type == AUTO_DBG_CHECK_HANG) {
		if (dm->support_ic_type & ODM_IC_11AC_SERIES) {
			pr_debug("Not Support\n");
		} else {
			#if (ODM_IC_11N_SERIES_SUPPORT == 1)
			phydm_auto_check_hang_engine_n(dm);
			#else
			pr_debug("Not Support\n");
			#endif
		}

	} else if (atd_t->auto_dbg_type == AUTO_DBG_CHECK_RA) {
		pr_debug("Not Support\n");
	}
}

void phydm_auto_dbg_engine_init(void *dm_void)
{
	struct dm_struct *dm = (struct dm_struct *)dm_void;
	struct phydm_auto_dbg_struct *atd_t = &dm->auto_dbg_table;
	u16 dbg_port_table[DBGPORT_CHK_NUM] = {0x0, 0x803, 0x208, 0xab0,
					       0xab1, 0xab2};

	PHYDM_DBG(dm, ODM_COMP_API, "%s ======>\n", __func__);

	odm_move_memory(dm, &atd_t->dbg_port_table[0],
			&dbg_port_table[0], (DBGPORT_CHK_NUM * 2));

	phydm_check_hang_init(dm);
}

void phydm_store_phy_utility(void *dm_void)
{
	struct dm_struct *dm = (struct dm_struct *)dm_void;
	struct phydm_auto_dbg_info *a_dbg = &dm->auto_dbg_i;
	struct phydm_bkp_phy_utility_info *bkp_phy_utility = &a_dbg->bkp_phy_utility_i;
	struct phydm_cfo_track_struct *cfo_t = &dm->dm_cfo_track;
	struct odm_phy_dbg_info *dbg_i = &dm->phy_dbg_info;
	struct phydm_phystatus_avg *dbg_avg = &dbg_i->phystatus_statistic_avg;

	PHYDM_DBG(dm, ODM_COMP_API, "%s ======>\n", __func__);

	bkp_phy_utility->rx_utility = dm->rx_utility;
	bkp_phy_utility->avg_phy_rate = dm->avg_phy_rate;
	bkp_phy_utility->rx_rate_plurality = dm->rx_rate_plurality;
	bkp_phy_utility->tx_rate = dm->curr_tx_rate;
	bkp_phy_utility->cfo_avg = cfo_t->CFO_ave_pre;
	#ifdef PHYDM_PHYSTAUS_AUTO_SWITCH
	bkp_phy_utility->cn_avg = dbg_i->cn_avg;
	#endif
	odm_move_memory(dm, &bkp_phy_utility->physts_avg_i, dbg_avg, sizeof(struct phydm_phystatus_avg_info));
	
}

void phydm_store_pmac_info(void *dm_void)
{
	struct dm_struct *dm = (struct dm_struct *)dm_void;
	struct phydm_auto_dbg_info *a_dbg = &dm->auto_dbg_i;
	struct phydm_bkp_pmac_info *bkp_pmac = &a_dbg->bkp_pmac_i;
	struct phydm_fa_struct *fa_t = &dm->false_alm_cnt;

	PHYDM_DBG(dm, ODM_COMP_API, "%s ======>\n", __func__);

	/* CCK/OFDM Tx counter */
	bkp_pmac->tx_cnt_i.cck_mac_txen= fa_t->cnt_cck_txen;
	bkp_pmac->tx_cnt_i.cck_phy_txon= fa_t->cnt_cck_txon;
	bkp_pmac->tx_cnt_i.ofdm_mac_txen= fa_t->cnt_ofdm_txen;
	bkp_pmac->tx_cnt_i.ofdm_phy_txon= fa_t->cnt_ofdm_txon;

	/* CCK/OFDM CCA counter */
	bkp_pmac->cca_i.cnt_cck_cca = fa_t->cnt_cck_cca;
	bkp_pmac->cca_i.cnt_ofdm_cca = fa_t->cnt_ofdm_cca;
	bkp_pmac->cca_i.cnt_cca_all = fa_t->cnt_cca_all;

	/* CCK/OFDM FA counter */
	bkp_pmac->fa_i.cnt_cck_fail = fa_t->cnt_cck_fail;
	bkp_pmac->fa_i.cnt_ofdm_fail = fa_t->cnt_ofdm_fail;
	bkp_pmac->fa_i.cnt_fail_all = fa_t->cnt_all;
	bkp_pmac->fa_i.legacy_fa_i.cnt_parity_fail = fa_t->cnt_parity_fail;
	bkp_pmac->fa_i.legacy_fa_i.cnt_rate_illegal = fa_t->cnt_rate_illegal;
	bkp_pmac->fa_i.legacy_fa_i.cnt_fast_fsync = fa_t->cnt_fast_fsync;
	bkp_pmac->fa_i.legacy_fa_i.cnt_sb_search_fail = fa_t->cnt_sb_search_fail;
	bkp_pmac->fa_i.ht_fa_i.cnt_crc8_fail = fa_t->cnt_crc8_fail;
	bkp_pmac->fa_i.ht_fa_i.cnt_mcs_fail = fa_t->cnt_mcs_fail;
	bkp_pmac->fa_i.vht_fa_i.cnt_crc8_fail_vht = fa_t->cnt_crc8_fail_vhta + fa_t->cnt_crc8_fail_vhtb;
	bkp_pmac->fa_i.vht_fa_i.cnt_mcs_fail_vht = fa_t->cnt_mcs_fail_vht;

	/* CRC32 counter */
	bkp_pmac->crc_i.cnt_cck_crc32_error = fa_t->cnt_cck_crc32_error;
	bkp_pmac->crc_i.cnt_cck_crc32_ok = fa_t->cnt_cck_crc32_ok;
	bkp_pmac->crc_i.cnt_ofdm_crc32_error = fa_t->cnt_ofdm_crc32_error;
	bkp_pmac->crc_i.cnt_ofdm_crc32_ok = fa_t->cnt_ofdm_crc32_ok;
	bkp_pmac->crc_i.cnt_ht_crc32_error = fa_t->cnt_ht_crc32_error;
	bkp_pmac->crc_i.cnt_ht_crc32_ok = fa_t->cnt_ht_crc32_ok;
	bkp_pmac->crc_i.cnt_vht_crc32_error = fa_t->cnt_vht_crc32_error;
	bkp_pmac->crc_i.cnt_vht_crc32_ok = fa_t->cnt_vht_crc32_ok;
	bkp_pmac->crc_i.cnt_crc32_error_all = fa_t->cnt_crc32_error_all;
	bkp_pmac->crc_i.cnt_crc32_ok_all = fa_t->cnt_crc32_ok_all;
	bkp_pmac->crc_i.cnt_mpdu_crc32_error = fa_t->cnt_mpdu_crc32_error;
	bkp_pmac->crc_i.cnt_mpdu_crc32_ok = fa_t->cnt_mpdu_crc32_ok;
	bkp_pmac->crc_i.cnt_mpdu_miss = fa_t->cnt_mpdu_miss;

	/* CRC32 counter2 */
	bkp_pmac->crc2_i.ofdm2_rate_idx = fa_t->ofdm2_rate_idx;
	bkp_pmac->crc2_i.cnt_ofdm2_crc32_error = fa_t->cnt_ofdm2_crc32_error;
	bkp_pmac->crc2_i.cnt_ofdm2_crc32_ok = fa_t->cnt_ofdm2_crc32_ok;
	bkp_pmac->crc2_i.ofdm2_pcr = fa_t->ofdm2_pcr;
	bkp_pmac->crc2_i.ht2_rate_idx = fa_t->ht2_rate_idx;
	bkp_pmac->crc2_i.cnt_ht2_crc32_error = fa_t->cnt_ht2_crc32_error;
	bkp_pmac->crc2_i.cnt_ht2_crc32_ok = fa_t->cnt_ht2_crc32_ok;
	bkp_pmac->crc2_i.ht2_pcr = fa_t->ht2_pcr;
	bkp_pmac->crc2_i.vht2_rate_idx = fa_t->vht2_rate_idx;
	bkp_pmac->crc2_i.cnt_vht2_crc32_error = fa_t->cnt_vht2_crc32_error;
	bkp_pmac->crc2_i.cnt_vht2_crc32_ok = fa_t->cnt_vht2_crc32_ok;
	bkp_pmac->crc2_i.vht2_pcr = fa_t->vht2_pcr;
	/* BT polluted counter */
	bkp_pmac->cnt_bt_polluted = fa_t->cnt_bt_polluted;


}

void phydm_auto_chk_hang(void *dm_void)
{
	struct dm_struct *dm = (struct dm_struct *)dm_void;
	struct phydm_auto_dbg_info *a_dbg = &dm->auto_dbg_i;
	struct phydm_chk_hang_info *chk_hang = &a_dbg->chk_hang_i;
	struct phydm_fa_struct *fa_t = &dm->false_alm_cnt;

	PHYDM_DBG(dm, ODM_COMP_API, "%s ======>\n", __func__);

	if (fa_t->cnt_cca_all == 0)
		chk_hang->consecutive_no_rx_cnt++;
	else
		chk_hang->consecutive_no_rx_cnt = 0;

	if (fa_t->cnt_ofdm_txen == 0 && fa_t->cnt_cck_txen == 0)
		chk_hang->consecutive_no_tx_cnt++;
	else
		chk_hang->consecutive_no_tx_cnt = 0;

	if (chk_hang->consecutive_no_tx_cnt > 30 && chk_hang->consecutive_no_rx_cnt > 30)
		chk_hang->hang_occur = true;
	else
		chk_hang->hang_occur = false;
	

}
void phydm_auto_debug_en(void *dm_void, enum phydm_auto_dbg_type type, boolean en){

	struct dm_struct *dm = (struct dm_struct *)dm_void;
	struct phydm_auto_dbg_info *a_dbg = &dm->auto_dbg_i;

	if(en)
		a_dbg->auto_dbg_type_i |= (u32)BIT(type);
	else
		a_dbg->auto_dbg_type_i &= (u32)~(BIT(type));

	PHYDM_DBG(dm, ODM_COMP_API, "[%s] en=%d, type=0x%x, auto_dbg_type=0x%x\n",
		  __func__, en, type, a_dbg->auto_dbg_type_i);
	
	#ifdef PHYDM_PHYSTAUS_AUTO_SWITCH
	if (type == AUTO_DBG_PHY_UTILITY) {
		if (en == true)
			phydm_physts_auto_switch_jgr3_set(dm, true, BIT(4) | BIT(1));
		else {
			if(!(dm->debug_components & DBG_CMN))
				phydm_physts_auto_switch_jgr3_set(dm, false, BIT(1));
		}
	}
	#endif
		

}

void phydm_query_phy_utility_info(void *dm_void, struct phydm_bkp_phy_utility_info *rpt)
{
	struct dm_struct *dm = (struct dm_struct *)dm_void;
	struct phydm_auto_dbg_info *a_dbg = &dm->auto_dbg_i;

	PHYDM_DBG(dm, ODM_COMP_API, "%s ======>\n", __func__);

	odm_move_memory(dm, rpt, &a_dbg->bkp_phy_utility_i, sizeof(struct phydm_bkp_phy_utility_info));
}

void phydm_query_pmac_info(void *dm_void, struct phydm_bkp_pmac_info *rpt)
{
	struct dm_struct *dm = (struct dm_struct *)dm_void;
	struct phydm_auto_dbg_info *a_dbg = &dm->auto_dbg_i;

	PHYDM_DBG(dm, ODM_COMP_API, "%s ======>\n", __func__);
	odm_move_memory(dm, rpt, &a_dbg->bkp_pmac_i, sizeof(struct phydm_bkp_pmac_info));

}

void phydm_query_hang_info(void *dm_void, struct phydm_chk_hang_info *rpt)
{
	struct dm_struct *dm = (struct dm_struct *)dm_void;
	struct phydm_auto_dbg_info *a_dbg = &dm->auto_dbg_i;

	PHYDM_DBG(dm, ODM_COMP_API, "%s ======>\n", __func__);
	odm_move_memory(dm, rpt, &a_dbg->chk_hang_i, sizeof(struct phydm_chk_hang_info));
}


void phydm_auto_debug_watchdog(void *dm_void)
{
	struct dm_struct *dm = (struct dm_struct *)dm_void;
	struct phydm_auto_dbg_info *a_dbg = &dm->auto_dbg_i;

	if(a_dbg->auto_dbg_type_i & BIT(AUTO_DBG_STORE_PMAC))
		phydm_store_pmac_info(dm);

	if(a_dbg->auto_dbg_type_i & BIT(AUTO_DBG_PHY_UTILITY))
		phydm_store_phy_utility(dm);

	if(a_dbg->auto_dbg_type_i & BIT(AUTO_DBG_CHK_HANG))
		phydm_auto_chk_hang(dm);
}

void phydm_auto_debug_init(void *dm_void)
{
	struct dm_struct *dm = (struct dm_struct *)dm_void;
	struct phydm_auto_dbg_info *a_dbg = &dm->auto_dbg_i;
	struct phydm_chk_hang_info *chk_hang = &a_dbg->chk_hang_i;

	chk_hang->consecutive_no_tx_cnt = 0;
	chk_hang->consecutive_no_rx_cnt = 0;
	chk_hang->hang_occur = false;

	a_dbg->auto_dbg_type_i = 0; /*default Close*/
}

void phydm_auto_debug_pmac_print(void *dm_void, u32 *_used,
	char *output, u32 *_out_len, struct phydm_bkp_pmac_info *pmac_rpt)
{
	u32 used = *_used;
	u32 out_len = *_out_len;
	struct phydm_tx_cnt_info *tx = &pmac_rpt->tx_cnt_i;
	struct phydm_cca_info *cca = &pmac_rpt->cca_i;
	struct phydm_crc_info *crc = &pmac_rpt->crc_i;
	struct phydm_fa_info *fa = &pmac_rpt->fa_i;

	PDM_SNPF(out_len, used, output + used, out_len - used,
		"[Tx]{CCK_TxEN, CCK_TxON, OFDM_TxEN, OFDM_TxON}: {%d, %d, %d, %d}\n",
		tx->cck_mac_txen, tx->cck_phy_txon, tx->ofdm_mac_txen, tx->ofdm_phy_txon);
	PDM_SNPF(out_len, used, output + used, out_len - used,
		"[CRC]{B/G/N/AC/All/MPDU} OK:{%d, %d, %d, %d, %d, %d} Err:{%d, %d, %d, %d, %d, %d}\n",
		crc->cnt_cck_crc32_ok, crc->cnt_ofdm_crc32_ok, crc->cnt_ht_crc32_ok,
		crc->cnt_vht_crc32_ok, crc->cnt_crc32_ok_all, crc->cnt_mpdu_crc32_ok,
		crc->cnt_cck_crc32_error, crc->cnt_ofdm_crc32_error, crc->cnt_ht_crc32_error,
		crc->cnt_vht_crc32_error, crc->cnt_crc32_error_all, crc->cnt_mpdu_crc32_error);
	PDM_SNPF(out_len, used, output + used, out_len - used,
		"[CCA]{CCK, OFDM, All}: {%d, %d, %d}\n",
		cca->cnt_cck_cca, cca->cnt_ofdm_cca, cca->cnt_cca_all);
	PDM_SNPF(out_len, used, output + used, out_len - used,
		"[FA]{CCK, OFDM, All}: {%d, %d, %d}\n",
		fa->cnt_cck_fail, fa->cnt_ofdm_fail, fa->cnt_fail_all);
	PDM_SNPF(out_len, used, output + used, out_len - used,
		"*[AMPDU Miss] = {%d}\n", crc->cnt_mpdu_miss);
	PDM_SNPF(out_len, used, output + used, out_len - used,
		"*[OFDM]Prty=%d, Rate=%d, SBD=%d\n",
		fa->legacy_fa_i.cnt_parity_fail, fa->legacy_fa_i.cnt_rate_illegal, fa->legacy_fa_i.cnt_sb_search_fail);
	PDM_SNPF(out_len, used, output + used, out_len - used,
		" *[HT]CRC8=%d, MCS=%d, *[VHT]SIGA_CRC8=%d, MCS=%d\n",
		fa->ht_fa_i.cnt_crc8_fail, fa->ht_fa_i.cnt_mcs_fail, 
		fa->vht_fa_i.cnt_crc8_fail_vht, fa->vht_fa_i.cnt_mcs_fail_vht);
	*_used = used;
	*_out_len = out_len;
	
}

void phydm_auto_debug_pmac_print2(void *dm_void, u32 *_used,
	char *output, u32 *_out_len, struct phydm_bkp_pmac_info *pmac_rpt)
{
	struct dm_struct *dm = (struct dm_struct *)dm_void;
	struct phydm_crc2_info *crc2 = &pmac_rpt->crc2_i;
	char dbg_buf[PHYDM_SNPRINT_SIZE] = {0};

	u32 used = *_used;
	u32 out_len = *_out_len;
	
	if (crc2->ofdm2_rate_idx) {
		phydm_print_rate_2_buff(dm, crc2->ofdm2_rate_idx, dbg_buf,
					PHYDM_SNPRINT_SIZE);
		PDM_SNPF(out_len, used, output + used, out_len - used,
			  "[OFDM:%s CRC32 Cnt] {error, ok}= {%d, %d} (%d percent)\n",
			  dbg_buf, crc2->cnt_ofdm2_crc32_error,
			  crc2->cnt_ofdm2_crc32_ok, crc2->ofdm2_pcr);
	}
	if (crc2->ht2_rate_idx) {
		phydm_print_rate_2_buff(dm, crc2->ht2_rate_idx, dbg_buf,
					PHYDM_SNPRINT_SIZE);
		PDM_SNPF(out_len, used, output + used, out_len - used,
			  "[HT:%s CRC32 Cnt] {error, ok}= {%d, %d} (%d percent)\n",
			  dbg_buf, crc2->cnt_ht2_crc32_error,
			  crc2->cnt_ht2_crc32_ok, crc2->ht2_pcr);
	}
	if (crc2->vht2_rate_idx) {
		phydm_print_rate_2_buff(dm, crc2->vht2_rate_idx, dbg_buf,
					PHYDM_SNPRINT_SIZE);
		PDM_SNPF(out_len, used, output + used, out_len - used,
			  "[VHT:%s CRC32 Cnt] {error, ok}= {%d, %d} (%d percent)\n",
			  dbg_buf, crc2->cnt_vht2_crc32_error,
			  crc2->cnt_vht2_crc32_ok, crc2->vht2_pcr);
	}
	*_used = used;
	*_out_len = out_len;
}

void phydm_auto_debug_nss_print(void *dm_void, u8 nss, struct phydm_bkp_phy_utility_info *phy_rpt,
	u32 *_used, char *output, u32 *_out_len)
{
	struct dm_struct *dm = (struct dm_struct *)dm_void;
	struct phydm_phystatus_avg_info *phy_avg = &phy_rpt->physts_avg_i;
	u32 used = *_used;
	u32 out_len = *_out_len;
	char *rate_type = NULL;
	u8 *tmp_rssi_avg = NULL;
	u8 *tmp_snr_avg = NULL;
	u8 *tmp_evm_avg = NULL;
	u8 evm_rpt_show[RF_PATH_MEM_SIZE];
	u8 i = 0;

	odm_memory_set(dm, &evm_rpt_show[0], 0, RF_PATH_MEM_SIZE);

	switch (nss) {
	#if (defined(PHYDM_COMPILE_ABOVE_4SS))
	case 4:
		rate_type = "[4-SS]";
		tmp_rssi_avg = &phy_avg->rssi_4ss_avg[0];
		tmp_snr_avg = &phy_avg->snr_4ss_avg[0];
		tmp_evm_avg = &phy_avg->evm_4ss_avg[0];
		break;
	#endif
	#if (defined(PHYDM_COMPILE_ABOVE_3SS))
	case 3:
		rate_type = "[3-SS]";
		tmp_rssi_avg = &phy_avg->rssi_3ss_avg[0];
		tmp_snr_avg = &phy_avg->snr_3ss_avg[0];
		tmp_evm_avg = &phy_avg->evm_3ss_avg[0];
		break;
	#endif
	#if (defined(PHYDM_COMPILE_ABOVE_2SS))
	case 2:
		rate_type = "[2-SS]";
		tmp_rssi_avg = &phy_avg->rssi_2ss_avg[0];
		tmp_snr_avg = &phy_avg->snr_2ss_avg[0];
		tmp_evm_avg = &phy_avg->evm_2ss_avg[0];
		break;
	#endif
	case 1:
		rate_type = "[1-SS]";
		tmp_rssi_avg = &phy_avg->rssi_1ss_avg[0];
		tmp_snr_avg = &phy_avg->snr_1ss_avg[0];
		tmp_evm_avg = &phy_avg->evm_1ss_avg;
		break;
	case 0:
		rate_type = "[L-OFDM]";
		tmp_rssi_avg = &phy_avg->rssi_ofdm_avg[0];
		tmp_snr_avg = &phy_avg->snr_ofdm_avg[0];
		tmp_evm_avg = &phy_avg->evm_ofdm_avg;
		break;
	default:
		PHYDM_DBG(dm, DBG_CMN, "[warning] %s\n", __func__);
		return;
	}

	if (nss == 0 || nss == 1) 
		evm_rpt_show[0] = *tmp_evm_avg;
#if (defined(PHYDM_COMPILE_ABOVE_2SS))
	else {
		for (i = 0; i < nss; i++) 
			evm_rpt_show[i] = tmp_evm_avg[i];
	}
			
#endif
		

#if (defined(PHYDM_COMPILE_ABOVE_4SS))
	PDM_SNPF(out_len, used, output + used, out_len - used,
		  "* %-8s  RSSI:{%.2d, %.2d, %.2d, %.2d} SNR:{%.2d, %.2d, %.2d, %.2d} EVM:{-%.2d, -%.2d, -%.2d, -%.2d}\n",
		  rate_type,
		  tmp_rssi_avg[0], tmp_rssi_avg[1], tmp_rssi_avg[2],
		  tmp_rssi_avg[3], tmp_snr_avg[0], tmp_snr_avg[1],
		  tmp_snr_avg[2], tmp_snr_avg[3], evm_rpt_show[0],
		  evm_rpt_show[1], evm_rpt_show[2], evm_rpt_show[3]);
#elif (defined(PHYDM_COMPILE_ABOVE_3SS))
	PDM_SNPF(out_len, used, output + used, out_len - used,
		  "* %-8s  RSSI:{%.2d, %.2d, %.2d} SNR:{%.2d, %.2d, %.2d} EVM:{-%.2d, -%.2d, -%.2d}\n",
		  rate_type,
		  tmp_rssi_avg[0], tmp_rssi_avg[1], tmp_rssi_avg[2],
		  tmp_snr_avg[0], tmp_snr_avg[1], tmp_snr_avg[2],
		  evm_rpt_show[0], evm_rpt_show[1], evm_rpt_show[2]);
#elif (defined(PHYDM_COMPILE_ABOVE_2SS))
	PDM_SNPF(out_len, used, output + used, out_len - used,
		  "* %-8s  RSSI:{%.2d, %.2d} SNR:{%.2d, %.2d} EVM:{-%.2d, -%.2d}\n",
		  rate_type,
		  tmp_rssi_avg[0], tmp_rssi_avg[1],
		  tmp_snr_avg[0], tmp_snr_avg[1],
		  evm_rpt_show[0], evm_rpt_show[1]);
#else
	PDM_SNPF(out_len, used, output + used, out_len - used,
		  "* %-8s  RSSI:{%.2d} SNR:{%.2d} EVM:{-%.2d}\n",
		  rate_type,
		  tmp_rssi_avg[0], tmp_snr_avg[0], evm_rpt_show[0]);
#endif

	*_used = used;
	*_out_len = out_len;
}

void phydm_auto_debug_phy_utility_print(void *dm_void, struct phydm_bkp_phy_utility_info *phy_rpt, 
	u32 *_used, char *output, u32 *_out_len)
{
	struct dm_struct *dm = (struct dm_struct *)dm_void;
	struct phydm_phystatus_avg_info *phy_avg = &phy_rpt->physts_avg_i;
	u8 i = 0;
	u32 used = *_used;
	u32 out_len = *_out_len;
	PDM_SNPF(out_len, used, output + used, out_len - used,
		"CFO_avg = %d\n", phy_rpt->cfo_avg);
	#ifdef PHYDM_PHYSTAUS_AUTO_SWITCH
	PDM_SNPF(out_len, used, output + used, out_len - used,
		"CN_avg = (%d.%4d)\n", phy_rpt->cn_avg >> 3, phydm_show_fraction_num(phy_rpt->cn_avg & 0x7, 3));
	#endif

	switch (dm->num_rf_path) {
#if (defined(PHYDM_COMPILE_ABOVE_4SS))
	case 4:
		PDM_SNPF(out_len, used, output + used, out_len - used,
			  "* %-8s RSSI:{%.2d, %.2d, %.2d, %.2d}\n",
			  "[Beacon]", phy_avg->rssi_beacon_avg[0], phy_avg->rssi_beacon_avg[1],
			  phy_avg->rssi_beacon_avg[2], phy_avg->rssi_beacon_avg[3]);
		PDM_SNPF(out_len, used, output + used, out_len - used,
			  "* [SNR per path] SNR:{%.2d, %.2d, %.2d, %.2d}\n",
			  phy_avg->snr_per_path_avg[0], phy_avg->snr_per_path_avg[1],
			  phy_avg->snr_per_path_avg[2], phy_avg->snr_per_path_avg[3]);
		break;
#endif
#if (defined(PHYDM_COMPILE_ABOVE_3SS))
	case 3:
		PDM_SNPF(out_len, used, output + used, out_len - used,
			  "* %-8s RSSI:{%.2d, %.2d, %.2d}\n",
			  "[Beacon]", phy_avg->rssi_beacon_avg[0],
			  phy_avg->rssi_beacon_avg[1], phy_avg->rssi_beacon_avg[2]);
		PDM_SNPF(out_len, used, output + used, out_len - used,
			  "* [SNR per path] SNR:{%.2d, %.2d, %.2d}\n",
			  phy_avg->snr_per_path_avg[0], phy_avg->snr_per_path_avg[1], phy_avg->snr_per_path_avg[2]);
		break;
#endif
#if (defined(PHYDM_COMPILE_ABOVE_2SS))
	case 2:
		PDM_SNPF(out_len, used, output + used, out_len - used,
			  "* %-8s RSSI:{%.2d, %.2d}\n",
			  "[Beacon]", phy_avg->rssi_beacon_avg[0], phy_avg->rssi_beacon_avg[1]);
		PDM_SNPF(out_len, used, output + used, out_len - used,
			  "* [SNR per path] SNR:{%.2d, %.2d}\n",
			  phy_avg->snr_per_path_avg[0], phy_avg->snr_per_path_avg[1]);
		break;
#endif
	default:
		PDM_SNPF(out_len, used, output + used, out_len - used, "* %-8s RSSI:{%.2d}\n",
			  "[Beacon]", phy_avg->rssi_beacon_avg[0]);
		PDM_SNPF(out_len, used, output + used, out_len - used,
			  "* [SNR per path] SNR:{%.2d}\n", phy_avg->snr_per_path_avg[0]);
		break;
	}

	switch (dm->num_rf_path) {
#ifdef PHYSTS_3RD_TYPE_SUPPORT
	#if (defined(PHYDM_COMPILE_ABOVE_4SS))
	case 4:
		PDM_SNPF(out_len, used, output + used, out_len - used,
			  "* %-8s RSSI:{%.2d, %.2d, %.2d, %.2d}\n",
			  "[CCK]",phy_avg->rssi_cck_avg, phy_avg->rssi_cck_avg_abv_2ss[0],
			  phy_avg->rssi_cck_avg_abv_2ss[1], phy_avg->rssi_cck_avg_abv_2ss[2]);
		break;
	#endif
	#if (defined(PHYDM_COMPILE_ABOVE_3SS))
	case 3:
		PDM_SNPF(out_len, used, output + used, out_len - used,
			  "* %-8s RSSI:{%.2d, %.2d, %.2d}\n",
			  "[CCK]", phy_avg->rssi_cck_avg, phy_avg->rssi_cck_avg_abv_2ss[0],
			  phy_avg->rssi_cck_avg_abv_2ss[1]);
		break;
	#endif
	#if (defined(PHYDM_COMPILE_ABOVE_2SS))
	case 2:
		PDM_SNPF(out_len, used, output + used, out_len - used,
			  "* %-8s RSSI:{%.2d, %.2d}\n",
			  "[CCK]", phy_avg->rssi_cck_avg, phy_avg->rssi_cck_avg_abv_2ss[0]);
		break;
	#endif
#endif
	default:
		PDM_SNPF(out_len, used, output + used, out_len - used, "* %-8s RSSI:{%.2d}\n",
			  "[CCK]", phy_avg->rssi_cck_avg);
		break;
	}

	for (i = 0; i <= dm->num_rf_path; i++)
		phydm_auto_debug_nss_print(dm, i, phy_rpt, &used, output, &out_len);

	for (i = 0; i < dm->num_max_ss; i++){
		PDM_SNPF(out_len, used, output + used, out_len - used,
			"evm_ss_avg[%d] = %d\n", i, phy_avg->evm_ss_avg[i]);
	}

	*_used = used;
	*_out_len = out_len;
}
void phydm_auto_debug_dbg(
	void *dm_void,
	char input[][16],
	u32 *_used,
	char *output,
	u32 *_out_len)
{
	struct dm_struct *dm = (struct dm_struct *)dm_void;
	struct phydm_bkp_phy_utility_info phy_rpt;
	struct phydm_bkp_pmac_info pmac_rpt;
	struct phydm_chk_hang_info chk_hang_rpt;
	
	u32 var1[10] = {0};
	u32 used = *_used;
	u32 out_len = *_out_len;


	if ((strcmp(input[1], "-h") == 0)) {
		PDM_SNPF(out_len, used, output + used, out_len - used,
			 "type {0:chk_hang, 1:chk_tx, 2:PMAC, 3:phy_utility} {en}\n");
		PDM_SNPF(out_len, used, output + used, out_len - used,
			 "query {0:chk_hang, 1:chk_tx, 2:PMAC, 3:phy_utility}\n");
		
		return;
	}

	if ((strcmp(input[1], "type") == 0)) {
		PHYDM_SSCANF(input[2], DCMD_DECIMAL, &var1[0]);
		PHYDM_SSCANF(input[3], DCMD_DECIMAL, &var1[1]);

		phydm_auto_debug_en(dm, (enum phydm_auto_dbg_type)var1[0], (boolean)var1[1]);
		PDM_SNPF(out_len, used, output + used, out_len - used,
			 "type_idx=0x%x, en=%d, auto_dbg_type=0x%x\n", var1[0], var1[1], dm->auto_dbg_i.auto_dbg_type_i);
	} else if ((strcmp(input[1], "query") == 0)){
		PHYDM_SSCANF(input[2], DCMD_DECIMAL, &var1[0]);

		if (var1[0] == 3) {
			phydm_query_phy_utility_info(dm, &phy_rpt);
			PDM_SNPF(out_len, used, output + used, out_len - used,
				 "tx_rate=0x%x, avg_rx_rate=0x%x, rx_rate_plurality=0x%x, rx_utility=%d\n",
			    	phy_rpt.tx_rate, phy_rpt.avg_phy_rate, phy_rpt.rx_rate_plurality,
			    	phy_rpt.rx_utility);
			PDM_SNPF(out_len, used, output + used, out_len - used,
				 "evm_min=-%d, evm_max=-%d, evm_1ss=-%d\n",
			    	phy_rpt.physts_avg_i.evm_min_avg, phy_rpt.physts_avg_i.evm_max_avg,
			    	phy_rpt.physts_avg_i.evm_1ss_avg_all);
			phydm_auto_debug_phy_utility_print(dm, &phy_rpt, &used, output, &out_len);
		} else if (var1[0] == 2){
			phydm_query_pmac_info(dm, &pmac_rpt);
			phydm_auto_debug_pmac_print(dm, &used, output, &out_len, &pmac_rpt);
			phydm_auto_debug_pmac_print2(dm, &used, output, &out_len, &pmac_rpt);
		} else if (var1[0] == 0){
			phydm_query_hang_info(dm, &chk_hang_rpt);
			PDM_SNPF(out_len, used, output + used, out_len - used,
			"consecutive_no_tx_cnt=%d, consecutive_no_rx_cnt=%d, hang_occur=%d\n",
			chk_hang_rpt.consecutive_no_tx_cnt, chk_hang_rpt.consecutive_no_rx_cnt, 
			chk_hang_rpt.hang_occur);
		} else {
			PDM_SNPF(out_len, used, output + used, out_len - used,
				"Err\n");
		}

	}

	*_used = used;
	*_out_len = out_len;
}

#endif
