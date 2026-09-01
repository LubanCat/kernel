/******************************************************************************
 *
 * Copyright(c) 2007 - 2020  Realtek Corporation.
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
#include "halbb_precomp.h"

#ifdef HALBB_PMAC_TX_SUPPORT
#ifdef HALBB_COMPILE_AX_SERIES

void halbb_bs_write(struct bb_info *bb, u8 *buf, u32 *byte_ptr, u32 *bit_ptr,
		    u32 data, u32 len)
{
	u32 w_size;
	u32 bmask;
	u8 d_tmp;
	do {
		if (len >= (*bit_ptr)) {
			w_size = (*bit_ptr);
			len = len - w_size;
			bmask = ((u32)1 << w_size) - 1;
			d_tmp = (u8)((data >> len) & bmask);
			buf[(*byte_ptr)] = buf[(*byte_ptr)] | d_tmp;
			*bit_ptr = 8;
			(*byte_ptr)++;
		} else {
			w_size = len;
			len = 0;
			bmask = ((u32)1 << w_size) - 1;
			d_tmp = (u8)((data & bmask) << ((*bit_ptr) - w_size));
			buf[(*byte_ptr)] = buf[(*byte_ptr)] | d_tmp;
			*bit_ptr = *bit_ptr - w_size;
		}
	} while (len > 0) ;
}

u32 halbb_msb2lsb(struct bb_info *bb, u32 x, u32 nbits)
{
	u32 lsb = 0;
	u32 b;

	for (b = 0; b < nbits; b++) {
		lsb = (lsb << 1) | (x & 0x1);
		x = x >> 1;
	}

	return lsb;
}

u8 halbb_msb2lsb_tab_256(struct bb_info *bb, u8 val)
{
	return (msb2lsb_tab16[val & 0xf] << 4) | msb2lsb_tab16[val >> 4];
}

u8 halbb_crc8_impl_tab(struct bb_info *bb, u8 *p, u32 len, u8 mode)
{
	u8 crc = 0xff;
	u8 bit = 0, j = 0, c = 0, min_j = 0;

	while (len--)
		crc = (crc << 8) ^ crctab_mp[crc ^ *p++];

	// mode:
	//  0: No c26t
	//  1: c26t exists, c26t = 0;
	//  2: c26t exists, c26t = 1;
	//  3: one user field (last_empty = 1)
	//  4: two user field (Last_empty = 0)
	if (mode == 1) {
		bit = crc & 0x80;
		crc <<= 1;
		if (bit)
			crc ^= 0x7;
	} else if (mode == 2) {
		bit = crc & 0x80;
		crc <<= 1;
		bit ^= 0x80;
		if (bit)
			crc ^= 0x7;
	} else {
		c = *p++;
		if (mode == 3)
			min_j = 0x4;
		else if (mode == 4)
			min_j = 0x20;
		else
			min_j = 0x80;

		for (j = 0x80; j > min_j; j >>= 1) {
			bit = crc & 0x80;
			crc <<= 1;
			if (c & j)
				bit ^= 0x80;
			if (bit)
				crc ^= 0x7;
		}
	}

	crc ^= 0xff;
	crc = (crc >> 4) & 0xf;
	return crc;
}

u8 halbb_crc8_comm_field(struct bb_info *bb, u8 *ru_alloc, u8 c26t, u8 l)
{
	u8 n = 0, crc_out = 0, mode = 0;

	switch (l) {
	case 1: // BW20
		n = 1;
		break;
	case 2: // BW40
		n = 1;
		break;
	case 3: // BW80
		n = 2;
		break;
	default: // case 4: // BW160 or BW80_80
		n = 4;
		break;
	}

	if (l >= 3) {
		if (c26t == 0)
			mode = 1;
		else
			mode = 2;
	} else {
		mode = 0;
	}

	crc_out = halbb_crc8_impl_tab(bb, ru_alloc, n, mode);

	return crc_out;
}

u8 halbb_crc8_user_field(struct bb_info *bb, u32 *user_field, u8 last_empty)
{
	u8 crc_out = 0;
	u8 d_buf[6] = {0};
	u8 byte_len;
	u8 mode;

	d_buf[0] = (user_field[0] >> 13) & 0xff;
	d_buf[1] = (user_field[0] >> 5) & 0xff;
	d_buf[2] = (user_field[0] << 3) & 0xf8;

	if (last_empty == 0) {
		d_buf[2] = d_buf[2] | ((user_field[1] >> 18) & 0x7);
		d_buf[3] = (user_field[1] >> 10) & 0xff;
		d_buf[4] = (user_field[1] >> 2) & 0xff;
		d_buf[5] = (user_field[1] << 6) & 0xc0;
	}

	if (last_empty == 0) {
		byte_len = 5;
		mode = 4;
	} else {
		byte_len = 2;
		mode = 3;
	}

	if (last_empty == 1)
		mode = 3;
	else
		mode = 4;

	crc_out = halbb_crc8_impl_tab(bb, d_buf, byte_len, mode);

	return crc_out;
}

void halbb_sigb_gen(struct bb_info *bb, struct halbb_plcp_info *in,
		    u8 *cc1_sigb_content_ptr, u8 *cc2_sigb_content_ptr,
		    u16 *cc_len, u8 *sigb_n_sym)
{
	u32 U_ppdu_bw = in->dbw;
	// u32 U_ru_position = ws->fixed_rate;
	u32 U_ru_position = in->usr[0].ru_alloc;
	u32 U_mac_id = 0;
	u32 U_nsts = in->usr[0].nss - 1;
	u32 U_txbf = in->usr[0].txbf;
	u32 U_mcs = in->usr[0].mcs;
	u32 U_dcm = in->usr[0].dcm;
	u32 U_coding = in->usr[0].fec;

	u8 sigb_compression = (u8)in->he_sigb_compress_en;
	u8 sigb_mcs = (u8)in->he_mcs_sigb, sigb_dcm = (u8)in->he_dcm_sigb;
	//------------------------------------
	u8 symbolnum = 0;
	u16 sigb_len[4][NUM_RU_SIZE] = {
		{257, 122, 70, 49, 0xffff, 0xffff},  // BW20
		{257, 122, 70, 49, 49, 0xffff},      // BW40
		{266, 131, 79, 58, 58, 58},          // BW80
		{282, 147, 95, 74, 74, 74}           // BW160
	};
	u8 cc1_padding[32] = {
		0x57, 0xDE, 0x5B, 0x9B,
		0xD5, 0x1B, 0x8C, 0x58,
		0xA1, 0x72, 0xDE, 0x45,
		0x3A, 0x3A, 0x3A, 0x3A,
		0x57, 0xDE, 0x5B, 0x9B,
		0xD5, 0x1B, 0x8C, 0x58,
		0xA1, 0x72, 0xDE, 0x45,
		0x3A, 0x3A, 0x3A, 0x3A
	};
	u8 cc2_padding[32] = {
		0xDE, 0x5B, 0x9B, 0xD5,
		0x1B, 0x8C, 0x58, 0xA1,
		0x72, 0xDE, 0x45, 0x3A,
		0x3A, 0x3A, 0x3A, 0x57,
		0xDE, 0x5B, 0x9B, 0xD5,
		0x1B, 0x8C, 0x58, 0xA1,
		0x72, 0xDE, 0x45, 0x3A,
		0x3A, 0x3A, 0x3A, 0x57
	};
	u16 sigb_len_wo[4] = {18, 18, 27, 43};
	u16 N_dbps_per_MCS[6] = {26,52,78,104,156,208};
	u16 n_dbps = 0;
	u16 data_len = 0;
	u16 data_len_wo = 0;

	u32 uf = (u32)((U_coding & 0x1) | (u32)((U_dcm & 0x1) << 1) | (u32)((halbb_msb2lsb(bb, U_mcs,4)&0xf) << 2) | (u32)((U_txbf & 0x1) << 6) | (u32)((halbb_msb2lsb(bb, U_nsts,3) & 0x7) << 7) | (u32)((halbb_msb2lsb(bb, U_mac_id,11) & 0x7ff) << 10));
	u8 pos_b0 = (u8)(U_ru_position & 0x1);
	u8 pos_b1b7 = (u8)((U_ru_position>>1) & 0x7f);

	u8 ru_alloc_sf_w[NUM_RU_SIZE] = {0,112,96,192,200,208}; // RU26~RU996
	u8 ru_alloc_sf_wo[NUM_RU_SIZE] = {113,113,113,113,114,115}; // RU26~RU996

	u8 ru_alloc_sf[8] = {0}; // RU996 to RU26
	u8 pos_ofst[NUM_RU_SIZE] = {67,65,61,53,37,0}; // RU996 to RU26
	u8 ru_size = 0, N_sub20 = 0, N_uf = 0, N_cf = 0, ru_vld = 0, i = 0, su20_idx = 0, su20_idx_wo = 0xff, c26_ex = 0;
	u16 cc2_byte_offset = 0, max_num_cc_bits_w_data_padding = 0, byte_inc = 0;
	u8 cc1_cf_stack[4] = {0}, cc2_cf_stack[4] = {0};
	u8 j = 0, cc1_cf_crc = 0, cc2_cf_crc = 0, N_ub = 0, uf_last = 0;
	u8 *cc1_cf_stack_ptr = NULL, *cc2_cf_stack_ptr = NULL;
	u32 uf_data_stack[MAX_DATA_STACK_SIZE];
	u32 *uf_data_stack_ptr = NULL;
	u8 ub_crc[MAX_UB_NUM] = {0};
	u8 *ub_crc_ptr = NULL, *cont_chan_1_bs = NULL, *cc1_padding_ptr = NULL, *cont_chan_2_bs = NULL, *cc2_padding_ptr = NULL;
	u32 byte_ptr = 0, bit_ptr = 8;
	u32 num_cc_pad_bits = 0, num_cc_pad_bits_wo = 0;
	u8 num_cc1_pad_wbytes = 0, num_cc2_pad_wbytes = 0, num_cc1_pad_wbits = 0, num_cc2_pad_wbits = 0;
	u8 sigb_content[96] = {0};

	for (i = 0; i < NUM_RU_SIZE; i++) {
		if (pos_b1b7 >= pos_ofst[i]) {
			ru_vld = 1;
			ru_size = 5-i;
			break;
		}
	}

	if (ru_vld == 0) {
		BB_WARNING("Invalid RU alloc !!\n");
		return;
	} else {
		switch (ru_size) {
		case 0: // RU26
			N_uf = 9;
			if (pos_b1b7 >= 28)
				su20_idx = 3;
			else if (pos_b1b7 >= 18)
				su20_idx = 2;
			else if (pos_b1b7 >= 9)
				su20_idx = 1;
			else // 0
				su20_idx = 0;

			if ( (U_ppdu_bw > 1) && (pos_b1b7 == 18) ) {
				c26_ex = 1;
				N_uf = 1;
				su20_idx = 0xff;
			}
			break;
		case 1: // RU52
			N_uf = 4;
			if (pos_b1b7 >= 49)
				su20_idx = 3;
			else if (pos_b1b7 >= 45)
				su20_idx = 2;
			else if (pos_b1b7 >= 41)
				su20_idx = 1;
			else // 37
				su20_idx = 0;
			break;
		case 2: // RU106
			N_uf = 2;
			if (pos_b1b7 >= 59)
				su20_idx = 3;
			else if (pos_b1b7 >= 57)
				su20_idx = 2;
			else if (pos_b1b7 >= 55)
				su20_idx = 1;
			else // 53
				su20_idx = 0;
			break;
		case 3: // RU242
			N_uf = 1;
			if (pos_b1b7 >= 64)
				su20_idx = 3;
			else if (pos_b1b7 >= 63)
				su20_idx = 2;
			else if (pos_b1b7 >= 62)
				su20_idx = 1;
			else // 61
				su20_idx = 0;
			break;
		case 4: // RU484
			N_uf = 1;
			if (pos_b1b7 >= 66)
				su20_idx = 2;
			else // 65
				su20_idx = 0;
			break;
		case 5: // RU996
		default:
			su20_idx = 0; // 67
			N_uf = 1;
			break;
		}

		if ((sigb_mcs == 2) || (sigb_mcs == 5))
			sigb_dcm = 0;

		n_dbps = (u16)(N_dbps_per_MCS[sigb_mcs] >> sigb_dcm);
		if (su20_idx == 0xff) {
			data_len = sigb_len[U_ppdu_bw][5];
			data_len_wo = sigb_len_wo[U_ppdu_bw];
		} else {
			data_len = sigb_len[U_ppdu_bw][ru_size];
			data_len_wo = sigb_len_wo[U_ppdu_bw];
		}
		if (n_dbps > 0) {
			symbolnum = data_len/n_dbps;
			symbolnum += ((data_len % n_dbps) == 0) ? 0 : 1;
		}
		max_num_cc_bits_w_data_padding = (u16)(symbolnum) * n_dbps;
		byte_inc = (max_num_cc_bits_w_data_padding & 0x7) != 0;
		cc2_byte_offset = (max_num_cc_bits_w_data_padding >> 3) + byte_inc;

		switch (U_ppdu_bw) {
		case 0: // BW20
			N_sub20 = 1;
			N_cf = 1;
			ru_alloc_sf[su20_idx] = ru_alloc_sf_w[ru_size];
			break;
		case 1: // BW40
			N_sub20 = 2;
			N_cf = 1;
			su20_idx_wo = 1- (su20_idx &0x1);
			ru_alloc_sf[su20_idx] = ru_alloc_sf_w[ru_size];
			ru_alloc_sf[su20_idx_wo] = ru_alloc_sf_wo[ru_size];
			break;
		case 2: // BW80
			N_sub20 = 4;
			N_cf = 2;
			if (su20_idx != 0xff)
				su20_idx_wo = ((su20_idx & 0x1)==0) ? (su20_idx+1) : (su20_idx-1);
			else
				su20_idx_wo = 0xff;
			for (i = 0; i < N_sub20; i++) {
				if (i == su20_idx) {
					ru_alloc_sf[i] = ru_alloc_sf_w[ru_size];
				} else if (i == su20_idx_wo) {
					ru_alloc_sf[i] = ru_alloc_sf_wo[ru_size];
				} else {
					if ((ru_size == 5) || ((su20_idx == 0xff) && (su20_idx_wo == 0xff))) // RU996
						ru_alloc_sf[i] = ru_alloc_sf_wo[5];
					else
						ru_alloc_sf[i] = ru_alloc_sf_wo[4];
				}
			}
			break;
		default: // case 3: // BW160 or BW80_80
			N_sub20 = 8;
			N_cf = 4;
			if (su20_idx != 0xff) {
				su20_idx = su20_idx + (pos_b0 << 2);
				su20_idx_wo = ((su20_idx & 0x1)==0) ? (su20_idx+1) : (su20_idx-1);
			} else {
				su20_idx_wo = 0xff;
			}
			for (i=0; i < N_sub20; i++) {
				if (i == su20_idx) {
					ru_alloc_sf[i] = ru_alloc_sf_w[ru_size];
				} else if (i == su20_idx_wo) {
					ru_alloc_sf[i] = ru_alloc_sf_wo[ru_size];
				} else {
					if ((ru_size == 5) || ((su20_idx == 0xff) && (su20_idx_wo == 0xff))) // RU996
						ru_alloc_sf[i] = ru_alloc_sf_wo[5];
					else
						ru_alloc_sf[i] = ru_alloc_sf_wo[4];
				}
			}
			break;
		}
	}

	// --- got ru_alloc_sf ---
	for (i = 0; i < 4; i++) {
		j = i << 1;
		cc1_cf_stack[i] = (u8)halbb_msb2lsb(bb, ru_alloc_sf[j],8);
		cc2_cf_stack[i] = (u8)halbb_msb2lsb(bb, ru_alloc_sf[j + 1],8);
	}
	cc1_cf_stack_ptr = cc1_cf_stack;
	cc2_cf_stack_ptr = cc2_cf_stack;
	cc1_cf_crc = halbb_crc8_comm_field(bb, cc1_cf_stack_ptr, c26_ex, (u8)(U_ppdu_bw + 1));
	cc2_cf_crc = halbb_crc8_comm_field(bb, cc2_cf_stack_ptr, 0, (u8)(U_ppdu_bw + 1));

	for (i = 0; i < 9; i++) {
		if (i==0) {
			uf_data_stack[i] = uf;
		} else {
			if (i < N_uf)
				uf_data_stack[i] = EMPTY_UF;
			else
				uf_data_stack[i] = 0;
		}
	}
	uf_data_stack_ptr = uf_data_stack;
	ub_crc_ptr = ub_crc;
	N_ub = N_uf >> 1;
	uf_last = (N_uf & 0x1);
	for (i = 0; i < N_ub; i++) {
		*ub_crc_ptr = halbb_crc8_user_field(bb, uf_data_stack_ptr, 0);
		ub_crc_ptr++;
		uf_data_stack_ptr += 2;
	}
	if (uf_last == 1)
		*ub_crc_ptr++ = halbb_crc8_user_field(bb, uf_data_stack_ptr, 1);

	// bs_write
	cc1_cf_stack_ptr = cc1_cf_stack;
	uf_data_stack_ptr = uf_data_stack;
	ub_crc_ptr = ub_crc;
	cont_chan_1_bs = &cc1_sigb_content_ptr[0];
	if (sigb_compression == 0) {
		for (i = 0; i < N_cf; i++)
			halbb_bs_write(bb,cont_chan_1_bs, &byte_ptr, &bit_ptr, *cc1_cf_stack_ptr++, 8);
		if (N_cf >= 2)
			halbb_bs_write(bb,cont_chan_1_bs, &byte_ptr, &bit_ptr, c26_ex, 1);
		halbb_bs_write(bb,cont_chan_1_bs, &byte_ptr, &bit_ptr, (u32)(cc1_cf_crc), 4);
		halbb_bs_write(bb,cont_chan_1_bs, &byte_ptr, &bit_ptr, 0, 6);
	}

	if ((su20_idx & 0x1) == 0) { // cc1
		for (i = 0; i < N_ub; i++) {
			halbb_bs_write(bb,cont_chan_1_bs, &byte_ptr, &bit_ptr, *uf_data_stack_ptr++, 21);
			halbb_bs_write(bb,cont_chan_1_bs, &byte_ptr, &bit_ptr, *uf_data_stack_ptr++, 21);

			halbb_bs_write(bb,cont_chan_1_bs, &byte_ptr, &bit_ptr, (u32)((*ub_crc_ptr++) << 6), 10);
		}

		if (uf_last == 1) {
			halbb_bs_write(bb,cont_chan_1_bs, &byte_ptr, &bit_ptr, *uf_data_stack_ptr, 21);
			halbb_bs_write(bb,cont_chan_1_bs, &byte_ptr, &bit_ptr, (u32)((*ub_crc_ptr) << 6), 10);
		}
	}
	// Perform padding
	num_cc_pad_bits = max_num_cc_bits_w_data_padding - data_len;
	num_cc_pad_bits_wo = max_num_cc_bits_w_data_padding - data_len_wo;

	if ( ((su20_idx & 0x1) == 0) || (su20_idx == 0xff) ) {
		num_cc1_pad_wbytes = (u8)(num_cc_pad_bits >> 3);
		num_cc1_pad_wbits = (u8)(num_cc_pad_bits & 0x7);
		num_cc2_pad_wbytes = (u8)(num_cc_pad_bits_wo >> 3);
		num_cc2_pad_wbits = (u8)(num_cc_pad_bits_wo & 0x7);
	} else {
		num_cc1_pad_wbytes = (u8)(num_cc_pad_bits_wo >> 3);
		num_cc1_pad_wbits = (u8)(num_cc_pad_bits_wo & 0x7);
		num_cc2_pad_wbytes = (u8)(num_cc_pad_bits >> 3);
		num_cc2_pad_wbits = (u8)(num_cc_pad_bits & 0x7);
	}
	if (U_ppdu_bw == 0) {
		num_cc2_pad_wbytes = 0;
		num_cc2_pad_wbits = 0;
	}
    	cc1_padding_ptr = &cc1_padding[0];
	for (i = 0; i < num_cc1_pad_wbytes; i++)
		halbb_bs_write(bb,cont_chan_1_bs, &byte_ptr, &bit_ptr, *cc1_padding_ptr++, 8);
	halbb_bs_write(bb,cont_chan_1_bs, &byte_ptr, &bit_ptr, *cc1_padding_ptr, num_cc1_pad_wbits);


	cont_chan_2_bs = &cc2_sigb_content_ptr[0];
	byte_ptr = 0;
	bit_ptr = 8;
	cc2_cf_stack_ptr = cc2_cf_stack;
	uf_data_stack_ptr = uf_data_stack;
	ub_crc_ptr = ub_crc;
	if (sigb_compression == 0) {
		for (i = 0; i < N_cf; i++)
			halbb_bs_write(bb,cont_chan_2_bs, &byte_ptr, &bit_ptr, *cc2_cf_stack_ptr++, 8);
		if (N_cf >= 2)
			halbb_bs_write(bb,cont_chan_2_bs, &byte_ptr, &bit_ptr, 0, 1); // c26_ex = 0;
		halbb_bs_write(bb,cont_chan_2_bs, &byte_ptr, &bit_ptr, (u32)(cc2_cf_crc << 6), 10);
	}

	if ((su20_idx&0x1) == 1) { // cc2
		for (i = 0; i < N_ub; i++) {
			halbb_bs_write(bb,cont_chan_2_bs, &byte_ptr, &bit_ptr, *uf_data_stack_ptr++, 21);
			halbb_bs_write(bb,cont_chan_2_bs, &byte_ptr, &bit_ptr, *uf_data_stack_ptr++, 21);

			halbb_bs_write(bb,cont_chan_2_bs, &byte_ptr, &bit_ptr, (u32)((*ub_crc_ptr++) << 6), 10);
		}

		if (uf_last == 1) {
			halbb_bs_write(bb,cont_chan_2_bs, &byte_ptr, &bit_ptr, *uf_data_stack_ptr, 21);
			halbb_bs_write(bb,cont_chan_2_bs, &byte_ptr, &bit_ptr, (u32)((*ub_crc_ptr) << 6), 10);
		}
	}

	cc2_padding_ptr = &cc2_padding[0];
	for (i = 0; i < num_cc2_pad_wbytes; i++)
		halbb_bs_write(bb,cont_chan_2_bs, &byte_ptr, &bit_ptr, *cc2_padding_ptr++, 8);
	halbb_bs_write(bb,cont_chan_2_bs, &byte_ptr, &bit_ptr, *cc2_padding_ptr, num_cc2_pad_wbits);

	for (i = 0; i < 96; i++)
		sigb_content[i] = halbb_msb2lsb_tab_256(bb, sigb_content[i]);

	*sigb_n_sym = symbolnum;
	*cc_len = cc2_byte_offset;
}

#endif
#endif
