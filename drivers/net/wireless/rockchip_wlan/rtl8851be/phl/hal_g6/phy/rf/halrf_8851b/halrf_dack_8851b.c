/******************************************************************************
 *
 * Copyright(c) 2019 Realtek Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 *****************************************************************************/
#include "../halrf_precomp.h"
#ifdef RF_8851B_SUPPORT

#define t_avg 100


void halrf_afe_init_8851b(struct rf_info *rf)
{
	halrf_wreg(rf, 0xc0dc, 0x1f0, 0xc);
}

void halrf_dack_init_8851b(struct rf_info *rf)
{
}

void halrf_dack_reset_8851b(struct rf_info *rf, enum rf_path path)
{
	halrf_wreg(rf, 0xc000, BIT(17), 0x0);
	halrf_wreg(rf, 0xc000, BIT(17), 0x1);
}

void halrf_drck_8851b(struct rf_info *rf)
{
	u32 c;
	u32 rck_d;

	RF_DBG(rf, DBG_RF_DACK, "[DACK]Ddie RCK start!!!\n");
	/*RCK_SEL=1*/
	halrf_wreg(rf, 0xc0c4, BIT(9), 0x1);

	halrf_wreg(rf, 0xc0c4, BIT(6), 0x1);
	c = 0;
	while (halrf_rreg(rf, 0xc0c8, BIT(3)) == 0) {
		c++;
		halrf_delay_us(rf, 1);
		if (c > 10000) {
			RF_DBG(rf, DBG_RF_DACK, "[DACK]DRCK timeout\n");
			break;
		}
	}

	halrf_wreg(rf, 0xc0c4, BIT(6), 0x0);
	halrf_wreg(rf, 0xc094, BIT(12), 0x1);
	halrf_delay_us(rf, 1);
	halrf_wreg(rf, 0xc094, BIT(12), 0x0);

	/*manual write for LPS*/
	//rck_d = halrf_rreg(rf, 0xc0c8, 0xf8000);
	rck_d = halrf_rreg(rf, 0xc0c8, 0x7c00);
	/*RCK_SEL=0*/
	halrf_wreg(rf, 0xc0c4, BIT(9), 0x0);
	halrf_wreg(rf, 0xc0c4, 0x1f, rck_d);
	RF_DBG(rf, DBG_RF_DACK, "[DACK]0xc0c4 = 0x%x\n", halrf_rreg(rf, 0xc0c4, MASKDWORD));
}

void halrf_addck_backup_8851b(struct rf_info *rf)
{
	struct halrf_dack_info *dack = &rf->dack;

	halrf_wreg(rf, 0xc0f4, 0x300, 0x0);
	dack->addck_d[0][0] = (u16)halrf_rreg(rf, 0xc0fc,0xffc00) ;
	dack->addck_d[0][1] = (u16)halrf_rreg(rf, 0xc0fc,0x003ff) ;
}

void halrf_addck_reload_8851b(struct rf_info *rf)
{
	struct halrf_dack_info *dack = &rf->dack;
	/*S0*/
	halrf_wreg(rf, 0xc0f8, 0x0ffc0000, dack->addck_d[0][0]);
	halrf_wreg(rf, 0xc0f8, 0x0003ff00, dack->addck_d[0][1]);
	/*manual*/
	halrf_wreg(rf, 0xc0f8, 0x30000000, 0x3);
}

void halrf_dack_backup_s0_8851b(struct rf_info *rf)
{
	struct halrf_dack_info *dack = &rf->dack;
	u8 i;
	u32 temp;

	halrf_wreg(rf, 0x12b8, BIT(30), 0x1);
	/*MSBK*/
#if 0
	for (i = 0; i < 0x10; i++) {
		/*S0*/
		halrf_wreg(rf, 0xc000, 0x1e, i);
		temp = (u8)halrf_rreg(rf, 0xc04c, 0x7fc0);
		RF_DBG(rf, DBG_RF_DACK, "[DACK]0xc04c %d = 0x%x\n",
		i, temp);
	}

	for (i = 0; i < 0x10; i++) {
		/*S0*/
		halrf_wreg(rf, 0xc020, 0x1e, i);
		temp = (u8)halrf_rreg(rf, 0xc070, 0x7fc0);
		RF_DBG(rf, DBG_RF_DACK, "[DACK]0xc070 %d = 0x%x\n",
		i, temp);
	}
#endif
	for (i = 0; i < 0x10; i++) {
		halrf_wreg(rf, 0xc000, 0x1e, i);
		dack->msbk_d[0][0][i] = (u8)halrf_rreg(rf, 0xc05c, 0xff000000);
		halrf_wreg(rf, 0xc020, 0x1e, i);
		dack->msbk_d[0][1][i] = (u8)halrf_rreg(rf, 0xc080, 0xff000000);
	}


	/*biasK*/
//	dack->biask_d[0][0] = (u16)halrf_rreg(rf, 0xc048, 0xffc);
//	dack->biask_d[0][1] = (u16)halrf_rreg(rf, 0xc06c, 0xffc);
	dack->biask_d[0][0] = (u16)halrf_rreg(rf, 0xc088, 0xffc00000);
	/*DADCK*/
	temp = (u8)halrf_rreg(rf, 0xc060, 0xff000000);
	if (temp >= 0xe7)
		dack->dadck_d[0][0] = 0xff;
	else
		dack->dadck_d[0][0] = (u8)temp + 24;

	temp = (u8)halrf_rreg(rf, 0xc084, 0xff000000);
	if (temp >= 0xe7)
		dack->dadck_d[0][1] = 0xff;
	else
		dack->dadck_d[0][1] = (u8)temp + 24;
}

void halrf_dack_backup_s1_8851b(struct rf_info *rf)
{
}

void halrf_dack_reload_by_path_8851b(struct rf_info *rf, enum rf_path path, u8 index)
{
	struct halrf_dack_info *dack = &rf->dack;
	u32 temp = 0 , temp_offset, temp_reg;
	u8 i;
	u32 idx_offset, path_offset;

	if (index ==0)
		idx_offset = 0;
	else
		idx_offset = 0x14;

	if (path == RF_PATH_A)
		path_offset = 0;
	else
		path_offset = 0x28;

	temp_offset = idx_offset + path_offset;

	halrf_wreg(rf, 0xc004, BIT(17), 0x1);
	halrf_wreg(rf, 0xc024, BIT(17), 0x1);

	/*msbk_d: 15/14/13/12*/
	temp = 0x0;
	for (i = 0; i < 4; i++) {
		temp |= dack->msbk_d[path][index][i+12] << (i * 8);
	}
	temp_reg = 0xc200 + temp_offset;
	halrf_w32(rf, temp_reg, temp);
	RF_DBG(rf, DBG_RF_DACK, "[DACK]0x%x=0x%x\n", temp_reg, halrf_rreg(rf, temp_reg, MASKDWORD));
	/*msbk_d: 11/10/9/8*/
	temp = 0x0;
	for (i = 0; i < 4; i++) {
		temp |= dack->msbk_d[path][index][i+8] << (i * 8);
	}
	temp_reg = 0xc204 + temp_offset;
	halrf_w32(rf, temp_reg, temp);
	RF_DBG(rf, DBG_RF_DACK, "[DACK]0x%x=0x%x\n", temp_reg, halrf_rreg(rf, temp_reg, MASKDWORD));
	/*msbk_d: 7/6/5/4*/
	temp = 0x0;
	for (i = 0; i < 4; i++) {
		temp |= dack->msbk_d[path][index][i+4] << (i * 8);
	}
	temp_reg = 0xc208 + temp_offset;
	halrf_w32(rf, temp_reg, temp);
	RF_DBG(rf, DBG_RF_DACK, "[DACK]0x%x=0x%x\n", temp_reg, halrf_rreg(rf, temp_reg, MASKDWORD));
	/*msbk_d: 3/2/1/0*/
	temp = 0x0;
	for (i = 0; i < 4; i++) {
		temp |= dack->msbk_d[path][index][i] << (i * 8);
	}
	temp_reg = 0xc20c + temp_offset;
	halrf_w32(rf, temp_reg, temp);
	RF_DBG(rf, DBG_RF_DACK, "[DACK]0x%x=0x%x\n", temp_reg, halrf_rreg(rf,temp_reg, MASKDWORD));
	/*dadak_d/biask_d*/
	temp = 0x0;
	temp = (dack->biask_d[0][0] << 22) |
		 (dack->dadck_d[path][index] << 14);
	temp_reg = 0xc210 + temp_offset;
	halrf_w32(rf, temp_reg, temp);
	RF_DBG(rf, DBG_RF_DACK, "[DACK]0x%x=0x%x\n", temp_reg, halrf_rreg(rf,temp_reg, MASKDWORD));
	/*enable DACK result from reg */
	halrf_wreg(rf, 0xc210 + temp_offset, BIT(0), 0x1);
}

void halrf_dack_reload_8851b(struct rf_info *rf, enum rf_path path)
{
	u8 i;

	for (i = 0; i < 2; i++)
		halrf_dack_reload_by_path_8851b(rf, path, i);
}

void halrf_check_addc_8851b(struct rf_info *rf, enum rf_path path)
{
	u32 temp, dc_re, dc_im;
	u32 i, m, p, t;
	u32 re[t_avg], im[t_avg];
#if 1
	halrf_wreg(rf, 0x20f4, BIT(24), 0x0);
	halrf_wreg(rf, 0x20f8, 0x80000000, 0x1);
	halrf_wreg(rf, 0x20f0, 0xff0000, 0x1);
	halrf_wreg(rf, 0x20f0, 0xf00, 0x2);
	halrf_wreg(rf, 0x20f0, 0xf, 0x0);
	if (path  == RF_PATH_A)
		halrf_wreg(rf, 0x20f0, 0xc0, 0x2);
	else
		halrf_wreg(rf, 0x20f0, 0xc0, 0x3);
	for (i = 0; i < t_avg; i++) {
		temp = halrf_rreg(rf, 0x1730, 0xffffffff);
		re[i] = (temp & 0xfff000) >> 12;
		im[i] = temp & 0xfff;
//		RF_DBG(rf, DBG_RF_DACK, "[DACK]S%d,re[i]= 0x%x,im[i] =0x%x\n",
//		path, re[i], im[i]);
	}
#else
	for (i = 0; i < t_avg; i++) {
		if (path  == RF_PATH_A)
			temp = halrf_rreg(rf, 0x1c8c, MASKDWORD);
		else
			temp = halrf_rreg(rf, 0x3c8c, MASKDWORD);

		re[i] = (temp & 0xfff000) >> 12;
		im[i] = temp & 0xfff;
//		RF_DBG(rf, DBG_RF_DACK, "[DACK]S%d,re[i]= 0x%x,im[i] =0x%x\n",
//		path, re[i], im[i]);
	}

#endif
	m = 0;
	p = 0;
	for (i = 0; i < t_avg; i++) {
		if (re[i] > 0x800)
			m = (0x1000 - re[i]) + m;
		else
			p = re[i] + p;
	}

	if (p > m) {
		t = p - m;
		t = t / t_avg;
	} else {
		t = m - p;
		t = t / t_avg;
		if (t != 0x0)
			t = 0x1000 - t;
	}
	dc_re = t;

	m = 0;
	p = 0;
	for (i = 0; i < t_avg; i++) {
		if (im[i] > 0x800)
			m = (0x1000 - im[i]) + m;
		else
			p = im[i] + p;
	}

	if (p > m) {
		t = p - m;
		t = t / t_avg;
	} else {
		t = m - p;
		t = t / t_avg;
		if (t != 0x0)
			t = 0x1000 - t;
	}
	dc_im = t;

	RF_DBG(rf, DBG_RF_DACK, "[DACK]S%d,dc_re = 0x%x,dc_im =0x%x\n",
		path, dc_re, dc_im);
}

void halrf_addck_8851b(struct rf_info *rf)
{
	struct halrf_dack_info *dack = &rf->dack;
	u32 c;
	/*S0*/

	/*0.set one_shot_k_mode*/
	halrf_wreg(rf, 0xc0f4, BIT(2), 0x1);
	/*1.Enable on_shot_k_en*/
	halrf_wreg(rf, 0xc0f4, BIT(4), 0x1);
	halrf_wreg(rf, 0xc0f4, BIT(4), 0x0);
	halrf_delay_us(rf, 1);
	/*check if cal done*/
	halrf_wreg(rf, 0xc0f4, 0x300, 0x1);
	c = 0;
	while (halrf_rreg(rf, 0xc0fc, BIT(0)) == 0) {
		c++;
		halrf_delay_us(rf, 1);
		if (c > 10000) {
			RF_DBG(rf, DBG_RF_DACK, "[DACK]S0 ADDCK timeout\n");
			dack->addck_timeout[0] = true;
			break;
		}
	}
	RF_DBG(rf, DBG_RF_DACK, "[DACK]ADDCK c = %d\n", c);
	/*Disable one_shot_k mode*/
	halrf_wreg(rf, 0xc0f4, BIT(2), 0x0);
}

void halrf_new_dadck_8851b(struct rf_info *rf)
{
	struct halrf_dack_info *dack = &rf->dack;
	u32 c, i_dc, q_dc, ic, qc;
	/*S0*/

	/*0.reset DADCK value*/
	halrf_wreg(rf, 0xc210, 0x003fc000, 0x80);
	halrf_wreg(rf, 0xc224, 0x003fc000, 0x80);
	/*1.ADDCK manual on*/
	halrf_wreg(rf, 0xc0f8, 0x30000000, 0x3);
	/*2.eanble ADDA clock*/
	halrf_wreg(rf, 0x12b8, BIT(30), 0x1);
	halrf_wreg(rf, 0x030c, 0x1f000000,0x1f);
	halrf_wreg(rf, 0x032c, 0xc0000000, 0x0);
	halrf_delay_us(rf, 100);
	/*3.reset K machine*/
	halrf_wreg(rf, 0x032c, BIT(22), 0x0);
	halrf_wreg(rf, 0x032c, BIT(22), 0x1);
	/*4.ADC input not from RXBB & ADC input from DAC*/
	halrf_wreg(rf, 0x032c, BIT(16), 0x0);
	halrf_wreg(rf, 0x032c, BIT(20), 0x1);
	/*5.release ADC reset*/
	halrf_wreg(rf, 0x030c, 0x0f000000,0x3);
	/*6.one shot mode off*/
	halrf_wreg(rf, 0xc0f4, BIT(2), 0x0);
	halrf_wreg(rf, 0xc0f4, BIT(4), 0x0);
	/*7enable dc offset calibration*/
	halrf_wreg(rf, 0xc0f4, BIT(11), 0x1);
	halrf_wreg(rf, 0xc0f4, BIT(11), 0x0);
	halrf_delay_us(rf, 1);
	/*check if cal done*/
	halrf_wreg(rf, 0xc0f4, 0x300, 0x1);
	c = 0;
	while (halrf_rreg(rf, 0xc0fc, BIT(0)) == 0) {
		c++;
		halrf_delay_us(rf, 1);
		if (c > 10000) {
			RF_DBG(rf, DBG_RF_DACK, "[DACK]S0 DADCK timeout\n");
			dack->addck_timeout[0] = true;
			break;
		}
	}
	RF_DBG(rf, DBG_RF_DACK, "[DACK]DADCK c = %d\n", c);
	halrf_wreg(rf, 0xc0f4, BIT(10), 0x0);
	i_dc = halrf_rreg(rf, 0xc0fc, 0xfff0);
	halrf_wreg(rf, 0xc0f4, BIT(10), 0x1);
	q_dc = halrf_rreg(rf, 0xc0fc, 0xfff0);	
	if (i_dc > 0x800)
		ic = 0x80 + (0x1000 - i_dc) * 6;
	else
		ic = 0x80 - (i_dc * 6);

	if (q_dc > 0x800)
		qc = 0x80 + (0x1000 - q_dc) * 6;
	else
		qc = 0x80 - (q_dc * 6);
	RF_DBG(rf, DBG_RF_DACK, "[DACK]before DADCK, i_dc=0x%x, q_dc=0x%x\n", i_dc, q_dc);
	dack->dadck_d[0][0] = (u8)ic;
	dack->dadck_d[0][1] = (u8)qc;
	halrf_wreg(rf, 0xc210, 0x003fc000, dack->dadck_d[0][0]);
	halrf_wreg(rf, 0xc224, 0x003fc000, dack->dadck_d[0][1]);
	RF_DBG(rf, DBG_RF_DACK, "[DACK]after DADCK, 0xc210=0x%x, 0xc224=0x%x\n", halrf_rreg(rf, 0xc210, 0xffffffff), halrf_rreg(rf, 0xc224, 0xffffffff));
#if 0

	/*check DADC*/
	/*3.reset K machine*/
	halrf_wreg(rf, 0x032c, BIT(22), 0x0);
	halrf_wreg(rf, 0x032c, BIT(22), 0x1);
	/*4.ADC input not from RXBB & ADC input from DAC*/
	halrf_wreg(rf, 0x032c, BIT(16), 0x0);
	halrf_wreg(rf, 0x032c, BIT(20), 0x1);
	/*5.release ADC reset*/
	halrf_wreg(rf, 0x030c, 0x0f000000,0x3);
	/*6.one shot mode off*/
	halrf_wreg(rf, 0xc0f4, BIT(4), 0x0);
	/*7.enable dc offset calibration*/
	halrf_wreg(rf, 0xc0f4, BIT(11), 0x1);
	halrf_wreg(rf, 0xc0f4, BIT(11), 0x0);
	halrf_delay_us(rf, 1);
	/*check if cal done*/
	halrf_wreg(rf, 0xc0f4, 0x300, 0x1);
	c = 0;
	while (halrf_rreg(rf, 0xc0fc, BIT(0)) == 0) {
		c++;
		halrf_delay_us(rf, 1);
		if (c > 10000) {
			RF_DBG(rf, DBG_RF_DACK, "[DACK]S0 DADCK timeout\n");
			dack->addck_timeout[0] = true;
			break;
		}
	}
	RF_DBG(rf, DBG_RF_DACK, "[DACK]DADCK c = %d\n", c);
	halrf_wreg(rf, 0xc0f4, BIT(10), 0x0);
	i_dc = halrf_rreg(rf, 0xc0fc, 0xfff0);
	halrf_wreg(rf, 0xc0f4, BIT(10), 0x1);
	q_dc = halrf_rreg(rf, 0xc0fc, 0xfff0);
	RF_DBG(rf, DBG_RF_DACK, "[DACK]after DADCK, i_dc=0x%x, q_dc=0x%x\n", i_dc, q_dc);
#endif
	halrf_wreg(rf, 0x032c, BIT(16), 0x1);
	halrf_wreg(rf, 0x032c, BIT(20), 0x0);
	halrf_wreg(rf, 0x030c, 0x1f000000, 0xc);
	halrf_wreg(rf, 0x032c, 0xc0000000, 0x1);
	halrf_wreg(rf, 0x12b8, BIT(30), 0x0);
}

void halrf_check_dadc_8851b(struct rf_info *rf, enum rf_path path)
{
	halrf_wreg(rf, 0x032c, BIT(30), 0x0);
	halrf_wreg(rf, 0x030c, 0x0f000000, 0xf);
	halrf_wreg(rf, 0x030c, 0x0f000000, 0x3);
	halrf_wreg(rf, 0x032c, BIT(16), 0x0);
	if (path == RF_PATH_A) {
		halrf_wreg(rf, 0x12dc, BIT(0), 0x1);
		halrf_wreg(rf, 0x12e8, BIT(2), 0x1);
		halrf_wrf(rf, RF_PATH_A, 0x8f, BIT(13), 0x1);
	} else {
		halrf_wreg(rf, 0x32dc, BIT(0), 0x1);
		halrf_wreg(rf, 0x32e8, BIT(2), 0x1);
		halrf_wrf(rf, RF_PATH_B, 0x8f, BIT(13), 0x1);
	}
	halrf_check_addc_8851b(rf, path);
	if (path == RF_PATH_A) {
		halrf_wreg(rf, 0x12dc, BIT(0), 0x0);
		halrf_wreg(rf, 0x12e8, BIT(2), 0x0);
		halrf_wrf(rf, RF_PATH_A, 0x8f, BIT(13), 0x0);
	} else {
		halrf_wreg(rf, 0x32dc, BIT(0), 0x0);
		halrf_wreg(rf, 0x32e8, BIT(2), 0x0);
		halrf_wrf(rf, RF_PATH_B, 0x8f, BIT(13), 0x0);
	}
	halrf_wreg(rf, 0x032c, BIT(16), 0x1);
}

void halrf_dack_8851b_s0(struct rf_info *rf)
{
	struct halrf_dack_info *dack = &rf->dack;
	u32 c = 0;

	/*step 1: Set clk to 160MHz for calibration  */
	halrf_wreg(rf, 0x12a0, BIT(15), 0x1);
	halrf_wreg(rf, 0x12a0, 0x7000, 0x3);
	/*step 2: DAC & clk enable */
	halrf_wreg(rf, 0x12b8, BIT(30), 0x1);
	halrf_wreg(rf, 0x030c, BIT(28), 0x1);
	halrf_wreg(rf, 0x032c, 0x80000000, 0x0);
//	halrf_wreg(rf, 0xc004, 0xfff00000, 0x30);
//	halrf_wreg(rf, 0xc024, 0xfff00000, 0x30);

	halrf_dack_reset_8851b(rf, RF_PATH_A);
	halrf_delay_us(rf, 100);
	/*step 3: Enable DACK*/
	halrf_wreg(rf, 0xc004, BIT(0), 0x1);

	//polling done
	c = 0x0;
	while ((halrf_rreg(rf, 0xc040, BIT(31)) == 0) || (halrf_rreg(rf, 0xc064, BIT(31)) == 0) ||
		(halrf_rreg(rf, 0xc05c, BIT(2)) == 0) || (halrf_rreg(rf, 0xc080, BIT(2)) == 0)) {
		c++;
		halrf_delay_us(rf, 1);
		if (c > 10000) {
			RF_DBG(rf, DBG_RF_DACK, "[DACK]S0 DACK timeout\n");
			dack->msbk_timeout[0] = true;
			break;
		}
	}
	RF_DBG(rf, DBG_RF_DACK, "[DACK]DACK c = %d\n", c);
	/*step 4: disableDACK */
	halrf_wreg(rf, 0xc004, BIT(0), 0x0);
	/*step 5: Set clk to 960MHz for normal mode */
	halrf_wreg(rf, 0x12a0, BIT(15), 0x0);
	halrf_wreg(rf, 0x12a0, 0x7000, 0x7);
	RF_DBG(rf, DBG_RF_DACK, "[DACK]after S0 DADCK\n");
//	halrf_check_dadc_8851b(rf, RF_PATH_A);

	/*backup here*/
	halrf_dack_backup_s0_8851b(rf);
	halrf_dack_reload_8851b(rf, RF_PATH_A);
	/*step 6: Set DAC & clk to normal mode */
	halrf_wreg(rf, 0x12b8, BIT(30), 0x0);
}

void halrf_step_dack_8851b_s0(struct rf_info *rf)
{
	struct halrf_dack_info *dack = &rf->dack;
	u32 c = 0;

	halrf_wreg(rf, 0xc210, BIT(0), 0x0);
	halrf_wreg(rf, 0xc224, BIT(0), 0x0);
	/*step 1: Set clk to 160MHz for calibration  */
	halrf_wreg(rf, 0x12a0, BIT(15), 0x1);
	halrf_wreg(rf, 0x12a0, 0x7000, 0x3);
	/*step 2: DAC & clk enable */
	halrf_wreg(rf, 0x12b8, BIT(30), 0x1);
	halrf_wreg(rf, 0x030c, BIT(28), 0x1);
	halrf_wreg(rf, 0x032c, 0x80000000, 0x0);
	halrf_dack_reset_8851b(rf, RF_PATH_A);
	halrf_delay_us(rf, 100);
	halrf_wreg(rf, 0xC018, BIT(2), 0x1);
	halrf_wreg(rf, 0xC0d8, BIT(16), 0x1);
	/*4*/
	/*osk sample cycles*/
	halrf_wreg(rf, 0xc018, 0x38000, 0x7);
	halrf_wreg(rf, 0xc038, 0x38000, 0x7);	
	/*4 OSK_din_offset*/
//	halrf_wreg(rf, 0xc00c, BIT(28), 0x1);
//	halrf_wreg(rf, 0xc02c, BIT(28), 0x1);
//	halrf_wreg(rf, 0xc00c, 0xf00000, 0xf);
//	halrf_wreg(rf, 0xc02c, 0xf00000, 0xf);

	halrf_wreg(rf, 0xc0dc, 0x0c000000, 0x3);
	halrf_wreg(rf, 0xc018, 0x3, 0x3);
	/*6*/
	halrf_wreg(rf, 0xC004, 0x3ff00000, dack->biask_d[0][1]);
	halrf_wreg(rf, 0xC024, 0x3ff00000, dack->biask_d[0][1]);
	/*7*/
	halrf_wreg(rf, 0xC004, 0xc0000000, 0x1);
	halrf_wreg(rf, 0xC024, 0xc0000000, 0x1);
	/*8*/
	halrf_wreg(rf, 0xC004, BIT(17), 0x1);
	halrf_wreg(rf, 0xC024, BIT(17), 0x1);
	halrf_wreg(rf, 0xc00c, BIT(2), 0x0);
	/*9*/
	halrf_wreg(rf, 0xc004, BIT(0), 0x1);
	c = 0x0;
	while ((halrf_rreg(rf, 0xc040, BIT(31)) == 0) || (halrf_rreg(rf, 0xc064, BIT(31)) == 0)) {
		c++;
		halrf_delay_us(rf, 1);
		if (c > 10000) {
			RF_DBG(rf, DBG_RF_DACK, "[DACK]S0 MASK timeout\n");
			dack->msbk_timeout[0] = true;
			break;
		}
	}
	RF_DBG(rf, DBG_RF_DACK, "[DACK]DACK c = %d\n", c);
	/*11*/
	halrf_wreg(rf, 0xc0dc, 0x0c000000, 0x0);
	/*12*/
	halrf_wreg(rf, 0xc00c, BIT(2), 0x1);
	c = 0x0;
	while ((halrf_rreg(rf, 0xc05c, BIT(2)) == 0) || (halrf_rreg(rf, 0xc080, BIT(2)) == 0)) {
		c++;
		halrf_delay_us(rf, 1);
		if (c > 10000) {
			RF_DBG(rf, DBG_RF_DACK, "[DACK]S0 DADCK timeout\n");
			dack->msbk_timeout[0] = true;
			break;
		}
	}
	/*14*/
	halrf_wreg(rf, 0xc004, BIT(0), 0x0);
	/*15*/
	halrf_wreg(rf, 0xC0d8, BIT(16), 0x0);
	halrf_wreg(rf, 0xc004, BIT(0), 0x0);

	halrf_wreg(rf, 0x12a0, BIT(15), 0x0);
	halrf_wreg(rf, 0x12a0, 0x7000, 0x7);
	RF_DBG(rf, DBG_RF_DACK, "[DACK]after S0 DADCK\n");

	/*backup here*/
	halrf_dack_backup_s0_8851b(rf);
	halrf_dack_reload_8851b(rf, RF_PATH_A);
	/*Set DAC & clk to normal mode */
	halrf_wreg(rf, 0x12b8, BIT(30), 0x0);
}
void halrf_dack_8851b_s1(struct rf_info *rf)
{

}

void halrf_dack_new_8851b_s0(struct rf_info *rf)
{
	struct halrf_dack_info *dack = &rf->dack;

//	halrf_step_dack_8851b_s0(rf);
//	halrf_step_dack_8851b_s0(rf);	
	dack->biask_d[0][0] = 0x60;
	dack->biask_d[0][1] = 0x60;
	halrf_step_dack_8851b_s0(rf);	
	dack->biask_d[0][1] = dack->biask_d[0][0];
//	update mbias
	halrf_step_dack_8851b_s0(rf);
#if 0
	if (dack->biask_d[0][0] < 0x50) {
		halrf_wreg(rf, 0xc0dc, 0x1f0, 0xd);
		halrf_delay_us(rf, 1000);
		halrf_step_dack_8851b_s0(rf);
	}
#endif
}

void halrf_dack_8851b(struct rf_info *rf)
{

//	u8 c = 0;
//	u32 temp;

	halrf_dack_new_8851b_s0(rf);
#if 0
	while (c < 10) {
		temp = halrf_rreg(rf, 0xc208, 0xffffffff);
		if ((c == 0) || (temp == 0x0) || (temp ==0x80808080))
			halrf_dack_check_bias_8851b_s0(rf);
//			halrf_step_dack_8851b_s0(rf);
		else
			break;
		c++;
		RF_DBG(rf, DBG_RF_DACK, "c=%d, SER L0(0x8424)=0x%x, 0x0704 = 0x%x\n",
			c, halrf_rmac32(rf, 0x8424), halrf_rreg(rf, 0x0704, MASKDWORD));
	}
#endif
//	halrf_dack_8851b_s0(rf);
}

void halrf_dack_dump_8851b(struct rf_info *rf)
{
	struct halrf_dack_info *dack = &rf->dack;
	u8 i;
	u8 t;

	RF_DBG(rf, DBG_RF_DACK, "[DACK]S0 ADC_DCK ic = 0x%x, qc = 0x%x\n",
	                    dack->addck_d[0][0], dack->addck_d[0][1] );
	RF_DBG(rf, DBG_RF_DACK, "[DACK]S0 DAC_DCK ic = 0x%x, qc = 0x%x\n",
	       dack->dadck_d[0][0], dack->dadck_d[0][1] );
	RF_DBG(rf, DBG_RF_DACK, "[DACK]S0 biask ic = 0x%x, qc = 0x%x\n",
	       dack->biask_d[0][0], dack->biask_d[0][1] );

	RF_DBG(rf, DBG_RF_DACK, "[DACK]S0 MSBK ic:\n");
	for (i = 0; i < 0x10; i++) {
		t = dack->msbk_d[0][0][i];
		RF_DBG(rf, DBG_RF_DACK, "[DACK]0x%x\n", t);
	}
	RF_DBG(rf, DBG_RF_DACK, "[DACK]S0 MSBK qc:\n");
	for (i = 0; i < 0x10; i++) {
		t = dack->msbk_d[0][1][i];
		RF_DBG(rf, DBG_RF_DACK, "[DACK]0x%x\n", t);
	}
}

void halrf_dack_manual_off_8851b(struct rf_info *rf)
{
	/*disable ADDCK manual mode*/
	halrf_wreg(rf, 0xc0f8, 0x30000000, 0x0);
	/*disable DACK result from reg */
	halrf_wreg(rf, 0xc210, BIT(0), 0x0);
	halrf_wreg(rf, 0xc224, BIT(0), 0x0);
}

void halrf_dac_cal_8851b(struct rf_info *rf, bool force)
{
	struct halrf_dack_info *dack = &rf->dack;
	u8 phy_map;

	phy_map = (BIT(HW_PHY_0) << 4) | RF_AB;

	dack->dack_done = false;
	RF_DBG(rf, DBG_RF_DACK, "[DACK]DACK start!!!\n");	

	halrf_btc_rfk_ntfy(rf, phy_map, RF_BTC_DACK, RFK_ONESHOT_START);
#ifdef HALRF_CONFIG_FW_IO_OFLD_SUPPORT
	if (rf->phl_com->dev_cap.io_ofld)
		halrf_write_fwofld_start(rf);		
#endif
	halrf_afe_init_8851b(rf);
	halrf_drck_8851b(rf);
	halrf_dack_manual_off_8851b(rf);
	halrf_wrf(rf, RF_PATH_A, 0x0, MASKRF, 0x337e1);
	halrf_wrf(rf, RF_PATH_A, 0x5, BIT(0), 0x0);
	halrf_addck_8851b(rf);
	halrf_addck_backup_8851b(rf);
	halrf_addck_reload_8851b(rf);
	halrf_wrf(rf, RF_PATH_A, 0x0, MASKRF, 0x40001);
	halrf_dack_8851b(rf);
	halrf_dack_dump_8851b(rf);
	halrf_wrf(rf, RF_PATH_A, 0x5, BIT(0), 0x1);
	dack->dack_done = true;
#ifdef HALRF_CONFIG_FW_IO_OFLD_SUPPORT
	if (rf->phl_com->dev_cap.io_ofld)
		halrf_write_fwofld_end(rf);
#endif
	halrf_btc_rfk_ntfy(rf, phy_map, RF_BTC_DACK, RFK_ONESHOT_STOP);
	dack->dack_cnt++;
	RF_DBG(rf, DBG_RF_DACK, "[DACK]DACK finish!!!\n");
}
#endif
