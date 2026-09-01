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
#ifndef __HALBB_DFS_H__
#define __HALBB_DFS_H__
/*@--------------------------[Define] ---------------------------------------*/
#define DFS_RPT_LENGTH 8
#define DFS_RDR_TYP_NUM 8
#define DFS_L_RDR_IDX 6
#define DFS_SPCL_RDR_IDX_ETSI 3
#define PW_FTR_IDLE 1
#define PRI_FTR_IDLE 1
#define PW_FTR 3
#define PRI_FTR 3
#define DFS_PPB_PRCNT 4
#define DFS_PPB_IDLE_PRCNT 5

#define DFS_FCC_LP_LNGTH 12	/*Real Waveform length of FCC-LP is 12 secs*/
#define DFS_MAX_SEQ_NUM 127

#define DFS_ADPTV_CNT1 1
#define DFS_ADPTV_CNT2 1
#define DFS_ADPTV_CNT_TH 30

#define DFS_Normal_State 0
#define DFS_Adaptive_State 1

#ifdef HALBB_DFS_GEN2_SERIES
#define REF_TYPE_PRI 111                /* 1428us / 12.8(unit) */
#define SRRC_RFE_TYPE_PRI 78      /* 1000us / 12.8(unit) */
#define W56_Type1_PRI 108            /* 1388us / 12.8(unit) */
#endif

#ifdef HALBB_DBG_TRACE_SUPPORT
#ifdef HALBB_DBCC_SUPPORT
#define BB_DFS_DBG(bb, case, fmt, ...)\
	do {\
		if((bb->dbg_component & BB_DFS) && (bb->bb_dfs_i.dbg_print_component & case)) {\
			_os_dbgdump("[BB][%d]" fmt, bb->bb_phy_idx, ##__VA_ARGS__);\
		}\
	} while (0)
#else
#define BB_DFS_DBG(bb, case, fmt, ...)\
	do {\
		if((bb->dbg_component & BB_DFS) && (bb->bb_dfs_i.dbg_print_component & case)) {\
			_os_dbgdump("[BB]" fmt, ##__VA_ARGS__);\
		}\
	} while (0)
#endif
#else
#define BB_DFS_DBG(bb, case, fmt, ...)
#endif

/*@--------------------------[Enum]------------------------------------------*/
enum dfs_dbg_print_case {
	HWDET_PRINT		= BIT(1),
	SWDET_PRINT		= BIT(2),
	BRK_PRINT		= BIT(3),
	DYN_PRINT		= BIT(4),
	TRIVIL_PRINT		= BIT(5)
};
/*@--------------------------[Structure]-------------------------------------*/
struct bb_dfs_cr_info {
	u32 dfs_en;
	u32 dfs_en_m;
	u32 tw_dfs_en;
	u32 tw_dfs_en_m;
	u32 dfs_en_p1;
	u32 dfs_en_p1_m;
	u32 dfs_l2h_th;
	u32 dfs_l2h_th_m;
};

struct bb_dfs_info {
//============= Used by TW DFS ==============//
	bool is_tw_en;
	bool bypass_seg0;
//===========================================//

	u8 dbg_print_component;
	u8 dfs_rgn_domain;
	u8 ppb_prcnt;
	u16 pw_rpt[DFS_MAX_SEQ_NUM];
	u8 pri_rpt[DFS_MAX_SEQ_NUM];
	u8 seq_num_rpt[DFS_MAX_SEQ_NUM];
	u8 seq_num_rpt_all[DFS_MAX_SEQ_NUM];
	bool chrp_rpt[DFS_MAX_SEQ_NUM];

#ifdef HALBB_DFS_GEN2_SERIES
	bool dfs_more[DFS_MAX_SEQ_NUM];
	bool dfs_phy_def[DFS_MAX_SEQ_NUM];
	u8 format[DFS_MAX_SEQ_NUM];
	u8 bbid[DFS_MAX_SEQ_NUM];
	u16 seq_num[DFS_MAX_SEQ_NUM];
	u16 RSVD[DFS_MAX_SEQ_NUM];
	u16 sub20_1_pw[DFS_MAX_SEQ_NUM];
	u16 sub20_1_pri[DFS_MAX_SEQ_NUM];
	bool sub20_1_chirp_flag[DFS_MAX_SEQ_NUM];
	bool sub20_1_pres[DFS_MAX_SEQ_NUM];
	u16 sub20_0_pw[DFS_MAX_SEQ_NUM];
	u16 sub20_0_pri[DFS_MAX_SEQ_NUM];
	bool sub20_0_chirp_flag[DFS_MAX_SEQ_NUM];
	bool sub20_0_pres[DFS_MAX_SEQ_NUM];

	u8 sub20_0_srt_rdr_cnt[DFS_RDR_TYP_NUM];
	u8 sub20_1_srt_rdr_cnt[DFS_RDR_TYP_NUM];
	u8 sub20_2_srt_rdr_cnt[DFS_RDR_TYP_NUM];
	u8 sub20_3_srt_rdr_cnt[DFS_RDR_TYP_NUM];
	u8 sub20_4_srt_rdr_cnt[DFS_RDR_TYP_NUM];
	u8 sub20_5_srt_rdr_cnt[DFS_RDR_TYP_NUM];
	u8 sub20_6_srt_rdr_cnt[DFS_RDR_TYP_NUM];
	u8 sub20_7_srt_rdr_cnt[DFS_RDR_TYP_NUM];
	u8 sub20_0_lng_rdr_cnt;
	u8 sub20_1_lng_rdr_cnt;
	u8 sub20_2_lng_rdr_cnt;
	u8 sub20_3_lng_rdr_cnt;
	u8 sub20_4_lng_rdr_cnt;
	u8 sub20_5_lng_rdr_cnt;
	u8 sub20_6_lng_rdr_cnt;
	u8 sub20_7_lng_rdr_cnt;

	u8 adv_pri_cnt[DFS_RDR_TYP_NUM];
	bool sub20_detect_en;
#endif
	u16 pw_ofst;
	u16 pri_ofst;
	u16 adv_pri_ofst;
	u32 l2h_init_val;
	u32 l2h_val;  /* for FCC detection BW */

//===========================================//
//===========================================//
//============= Used by TW DFS ==============//
	bool seg_rpt[DFS_MAX_SEQ_NUM];
	bool seg_rpt_all[DFS_MAX_SEQ_NUM];
//===========================================//
	//u8 chrp_cnt;
	//u8 chrp_th;
	u32 chrp_srt_t;
	u8 lng_rdr_cnt;
	u8 chrp_rdr_cnt;
	u8 lng_rdr_cnt_pre;

//============= Used by TW DFS ==============//
	u32 chrp_srt_t_sg1;
	u8 n_cnfd_lvl_sg1;
	u8 chrp_rdr_cnt_sg1;
	u8 lng_rdr_cnt_sg1;
	u8 lng_rdr_cnt_sg1_pre;
//===========================================//

	u8 srt_rdr_cnt[DFS_RDR_TYP_NUM];
	u16 pw_lbd[DFS_RDR_TYP_NUM];
	u16 pw_ubd[DFS_RDR_TYP_NUM];
	u16 pri_lbd[DFS_RDR_TYP_NUM];
	u16 pri_ubd[DFS_RDR_TYP_NUM];
	u16 pw_min_tab[DFS_RDR_TYP_NUM];
	u16 pw_max_tab[DFS_RDR_TYP_NUM];
	u16 pri_min_tab[DFS_RDR_TYP_NUM];
	u16 pri_max_tab[DFS_RDR_TYP_NUM];
	u8 ppb_tab[DFS_RDR_TYP_NUM];
	u8 ppb_tp_tab[DFS_RDR_TYP_NUM];
	u8 ppb_typ_th[DFS_RDR_TYP_NUM];
	u8 lst_seq_num;
	u8 lst_seg_idx;
	u8 pw_factor;
	u8 pri_factor;
//============= Used by TW DFS ==============//
	u8 srt_rdr_cnt_sg1[DFS_RDR_TYP_NUM];
	u8 lst_seq_num_sg1;
	u8 rpt_sg_history;
	u8 rpt_sg_history_all;
//===========================================//
	bool is_mic_w53;
	bool is_mic_w56;
	bool l_rdr_exst_flag;
	bool chrp_obsrv_flag;
	bool n_seq_flag;
//============= Used by TW DFS ==============//
	bool chrp_obsrv_flag_sg1;
	bool n_seq_flag_sg1;
//===========================================//
	bool idle_flag;
	bool first_dyn_set_flag;
	bool dyn_reset_flag;

	bool dfs_sw_trgr_mode;
	bool dfs_sw_trgr_mode_extra;
	bool dfs_dbg_mode;
	bool mask_fake_rpt_en;
	bool dfs_dyn_setting_en;
	u16 dfs_fa_th;
	u8 fk_dfs_num_th;
	u8 dfs_tp_th;
	u8 dfs_idle_prd_th;
	u8 dfs_rx_rto_th;
	u8 dfs_nhm_th;
	u8 dfs_rssi_th;

	u8 adap_detect_cnt;
	u8 adap_detect_cnt_init;
	u8 adap_detect_cnt_add;
	u8 adap_detect_cnt_all;
	u8 adap_detect_cnt_th;
	u8 detect_state;
	bool adap_detect_brk_en;

	u8 pri_mask_th;
	u8 rpt_rdr_cnt;
	u8 pri_full_cnt;
	u8 fake_rdr_cnt_th;

	bool In_CAC_Flag;

	u16 pw_diff_th;
	u16 pw_lng_chrp_diff_th;
	u16 pri_diff_th;
	u16 pw_max_th;
	u16 invalid_lng_pulse_h_th;
	u16 invalid_lng_pulse_l_th;

	u16 min_pw_shrt[DFS_RDR_TYP_NUM];
	u16 min_pw_lng;
	u16 min_pw_chrp;
	u16 min_pri_shrt[DFS_RDR_TYP_NUM];

	u16 max_pw_shrt[DFS_RDR_TYP_NUM];
	u16 max_pw_lng;
	u16 max_pw_chrp;
	u16 max_pri_shrt[DFS_RDR_TYP_NUM];

	u16 pw_diff_shrt[DFS_RDR_TYP_NUM];
	u16 pri_diff_shrt[DFS_RDR_TYP_NUM];

//============= Used by TW DFS ==============//
	u16 min_pw_shrt_sg1[DFS_RDR_TYP_NUM];
	u16 min_pw_lng_sg1;
	u16 min_pw_chrp_sg1;
	u16 min_pri_shrt_sg1[DFS_RDR_TYP_NUM];

	u16 max_pw_shrt_sg1[DFS_RDR_TYP_NUM];
	u16 max_pw_lng_sg1;
	u16 max_pw_chrp_sg1;
	u16 max_pri_shrt_sg1[DFS_RDR_TYP_NUM];

	u16 pw_diff_shrt_sg1[DFS_RDR_TYP_NUM];
	u16 pw_diff_lng_sg1;
	u16 pri_diff_shrt_sg1[DFS_RDR_TYP_NUM];
//===========================================//

};

struct bb_dfs_rpt {
	u8 *dfs_ptr;
	u16 dfs_num;
	u8 phy_idx; /*phy0,phy1*/
};

#ifdef HALBB_DFS_GEN2_SERIES
#if (PLATFOM_IS_LITTLE_ENDIAN)
struct bb_rdr_info_gen2 {
	u8 dfs_sub20_0_pres : 1;  /*whether rpt is valid*/
	u8 dfs_sub20_0_chirp_flag : 1;
	u8 dfs_sub20_0_pri_l : 6;

	u8 dfs_sub20_0_pri_m : 3;
	u8 dfs_sub20_0_pw_l : 5;

	u8 dfs_sub20_0_pw_m : 4;
	u8 dfs_sub20_1_pres : 1;
	u8 dfs_sub20_1_chirp_flag : 1;
	u8 dfs_sub20_1_pri_l : 2;

	u8 dfs_sub20_1_pri_m : 7;
	u8 dfs_sub20_1_pw_l : 1;

	u8 dfs_sub20_1_pw_m;

	u8 RSVD_l;
	
	u8 RSVD_m : 4;
	u8 dfs_seq_num_l : 4;

	u8 dfs_seq_num_m : 3;
	u8 dfs_bbid : 1; /*BB0,BB1*/
	u8 dfs_format : 2;
	u8 dfs_phy_def : 1; /*phy0,phy1*/
	u8 dfs_more : 1;
};
#else
struct bb_rdr_info_gen2 {
	u8 dfs_sub20_0_pri_l : 6;
	u8 dfs_sub20_0_chirp_flag : 1;
	u8 dfs_sub20_0_pres : 1;

	u8 dfs_sub20_0_pw_l : 5;
	u8 dfs_sub20_0_pri_m : 3;

	u8 dfs_sub20_1_pri_l : 2;
	u8 dfs_sub20_1_chirp_flag : 1;
	u8 dfs_sub20_1_pres : 1;
	u8 dfs_sub20_0_pw_m : 4;

	u8 dfs_sub20_1_pw_l : 1;
	u8 dfs_sub20_1_pri_m : 7;

	u8 dfs_sub20_1_pw_m;

	u8 RSVD_l;
	
	u8 dfs_seq_num_l : 4;
	u8 RSVD_m : 4;

	u8 dfs_more : 1;
	u8 dfs_phy_def : 1;
	u8 dfs_format : 2;
	u8 dfs_bbid : 1;
	u8 dfs_seq_num_m : 3;
};
#endif
#endif

#ifdef HALBB_TW_DFS_SERIES
#if (PLATFOM_IS_LITTLE_ENDIAN)
struct bb_rdr_info_gen1 {
	u8 rdr_info_sg1_chirp_flag : 1; /*[18:9],[8:1],[0]*/
	u8 rdr_info_sg1_pri_l : 7;

	u8 rdr_info_sg1_pri_m : 1;
	u8 rdr_info_sg1_pw_l : 7;

	u8 rdr_info_sg1_pw_m : 3;
	u8 rdr_info_sg0_chirp_flag : 1; /*[37:28],[27:20],[19]*/
	u8 rdr_info_sg0_pri_l : 4;

	u8 rdr_info_sg0_pri_m : 4;
	u8 rdr_info_sg0_pw_l : 4;

	u8 rdr_info_sg0_pw_m : 6;
	u8 rdr_info_rsvd_l : 2;			/*[48:38]*/

	u8 rdr_info_rsvd_m;

	u8 rdr_info_rsvd_h : 1;
	u8 rdr_info_sg1_seq : 7;		/*[55:49]*/

	u8 rdr_info_sg0_seq : 7;		/*[62:56]*/
	u8 rdr_info_path_opt : 1;		/*[63]*/

};
#else
struct bb_rdr_info_gen1 {
	u8 rdr_info_sg1_pri_l : 7;
	u8 rdr_info_sg1_chirp_flag : 1;

	u8 rdr_info_sg1_pw_l : 7;
	u8 rdr_info_sg1_pri_m : 1;

	u8 rdr_info_sg0_pri_l : 4;
	u8 rdr_info_sg0_chirp_flag : 1;
	u8 rdr_info_sg1_pw_m : 3;

	u8 rdr_info_sg0_pw_l : 4;
	u8 rdr_info_sg0_pri_m : 4;

	u8 rdr_info_rsvd_l : 2;
	u8 rdr_info_sg0_pw_m : 6;

	u8 rdr_info_rsvd_m;

	u8 rdr_info_sg1_seq : 7;
	u8 rdr_info_rsvd_h : 1;

	u8 rdr_info_path_opt : 1;
	u8 rdr_info_sg0_seq : 7;
};
#endif
#endif

#if (PLATFOM_IS_LITTLE_ENDIAN)
struct bb_rdr_info_gen0{
	u8 rdr_info_sg0_chirp_flag:1;	/*[18:9],[8:1],[0]*/
	u8 rdr_info_sg0_pri_l:7;

	u8 rdr_info_sg0_pri_m:1;
	u8 rdr_info_sg0_pw_l:7;

	u8 rdr_info_sg0_pw_m:3;
	u8 rdr_info_sg1_chirp_flag:1;	/*[37:28],[27:20],[19]*/
	u8 rdr_info_sg1_pri_l:4;

	u8 rdr_info_sg1_pri_m:4;
	u8 rdr_info_sg1_pw_l:4;

	u8 rdr_info_sg1_pw_m:6;
	u8 rdr_info_zw_chirp_flag:1;	/*[56:47],[46:39],[38]*/
	u8 rdr_info_zw_pri_l:1;

	u8 rdr_info_zw_pri_m:7;
	u8 rdr_info_zw_pw_l:1;

	u8 rdr_info_zw_pw_m;

	u8 rdr_info_zw_pw_h:1;
	u8 rdr_info_seq:7;
};
#else
struct bb_rdr_info_gen0 {
	u8 rdr_info_sg0_pri_l:7;
	u8 rdr_info_sg0_chirp_flag:1;

	u8 rdr_info_sg0_pw_l:7;
	u8 rdr_info_sg0_pri_m:1;

	u8 rdr_info_sg1_pri_l:4;
	u8 rdr_info_sg1_chirp_flag:1;
	u8 rdr_info_sg0_pw_m:3;

	u8 rdr_info_sg1_pw_l:4;
	u8 rdr_info_sg1_pri_m:4;

	u8 rdr_info_zw_pri_l:1;
	u8 rdr_info_zw_chirp_flag:1;
	u8 rdr_info_sg1_pw_m:6;

	u8 rdr_info_zw_pw_l:1;
	u8 rdr_info_zw_pri_m:7;

	u8 rdr_info_zw_pw_m;

	u8 rdr_info_seq:7;
	u8 rdr_info_zw_pw_h:1;
};
#endif
/*@--------------------------[Prptotype]-------------------------------------*/
struct bb_info;
struct hal_dfs_rpt;
void halbb_dfs(struct bb_info *bb);
void halbb_mac_cfg_dfs_rpt(struct bb_info *bb, bool rpt_en);
void halbb_dfs_rgn_dmn_dflt_cnfg(struct bb_info *bb);
void halbb_dfs_rgn_dmn_cnfg_by_ch(struct bb_info *bb, bool w53_band,
				  bool w56_band);
void halbb_radar_chrp_mntr(struct bb_info *bb, bool chrp_flag, bool is_sg1);
void halbb_radar_seq_inspctn(struct bb_info *bb, u16 dfs_rpt_idx,
			     u8 c_num, u8 p_num, bool is_sg1, u8 c_seg, u8 p_seg);
void halbb_radar_ptrn_cmprn(struct bb_info *bb, u16 dfs_rpt_idx,
			    u8 pri, u16 pw, bool chrp_flag, bool is_sg1);
void halbb_radar_info_processing(struct bb_info *bb,
				 struct hal_dfs_rpt *dfs_rpt, u16 dfs_rpt_idx);
void halbb_dfs_dyn_setting(struct bb_info *bb);
void halbb_dfs_debug(struct bb_info *bb, char input[][16], u32 *_used,
		     char *output, u32 *_out_len);
void halbb_cr_cfg_dfs_init(struct bb_info *bb);
#ifdef HALBB_DFS_GEN2_SERIES
void halbb_radar_info_processing_gen2(struct bb_info *bb, struct hal_dfs_rpt *rpt, u16 dfs_rpt_idx);
void halbb_radar_cnt_accumulate(struct bb_info *bb, u8 rpt_idx, bool sub20_1, u8 rdr_type_idx, bool is_long);
void halbb_radar_ptrn_cmprn_gen2(struct bb_info *bb);
u16 halbb_radar_judge_gen2(struct bb_info *bb);
u16 halbb_radar_detect_gen2(struct bb_info *bb, struct hal_dfs_rpt *dfs_rpt);
void halbb_dfs_dyn_setting_gen2(struct bb_info *bb);
void halbb_radar_show_log(struct bb_info *bb, struct hal_dfs_rpt *dfs_rp, u16 rdr_detected);
#endif
#endif
