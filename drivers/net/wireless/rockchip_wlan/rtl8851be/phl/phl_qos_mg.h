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
#ifdef CONFIG_QOS_MG
#ifndef _PHL_QOS_MG_H_
#define _PHL_QOS_MG_H_

#define STS_CODE_SZ 2
#define ELE_SZ 1
#define LEN_SZ 1
#define EX_ELE_SZ 1
#define SCSID_SZ 1
#define REQ_TP_SZ 1
#define INTRA_ACC_PRI_SZ 1
#define TCLAS_UP_SZ 1
#define TCLAS_CLF_TP_SZ 1
#define TCLAS_CLF_TP4_MASK_SZ 1
#define TCLAS_CLF_TP4_VER_SZ 1
#define TCLAS_CLF_TP4_IPV4_IP_SZ 4
#define TCLAS_CLF_TP4_IPV4_PORT_SZ 2
#define TCLAS_CLF_TP4_IPV4_DSCP_SZ 1
#define TCLAS_CLF_TP4_IPV4_PROT_SZ 1
#define TCLAS_CLF_TP4_IPV4_RSVD_SZ 1
#define TCLAS_CLF_TP10_PROT_INST_SZ 1
#define TCLAS_CLF_TP10_PROT_NUM_NEXT_HDR_SZ 1
#define TCLAS_PROCING_SZ 1
#define SCS_RSP_CNT_SZ 1
#define SCS_RSP_STS_SZ 2
#define UP_BITMAP_SZ 1
#define UP_CTRL_SZ 2
#define STM_TIMEOUT_SZ 4
#define MSCS_SUB_ELE_SZ 1
#define MSCS_SUB_LEN_SZ 1
#define QOS_CHAR_CTRL_SZ 4
#define QOS_CHAR_MIN_SER_INTVL_SZ 4
#define QOS_CHAR_MAX_SER_INTVL_SZ 4
#define QOS_CHAR_MIN_DATA_RATE_SZ 3
#define QOS_CHAR_DELAY_BOUND_SZ 3
#define QOS_CHAR_MAX_MSDU_SZ 2
#define QOS_CHAR_SER_ST_SZ 4
#define QOS_CHAR_SER_ST_LNKID_SZ 1
#define QOS_CHAR_MEAN_DATA_RATE_SZ 3
#define QOS_CHAR_BURST_SZ 4
#define QOS_CHAR_MSDU_LIFETIME_SZ 2
#define QOS_CHAR_MSDU_DELIVERY_RATIO_SZ 1
#define QOS_CHAR_MSDU_CNT_EXP_SZ 1
#define QOS_CHAR_MED_T_SZ 2

#define QM_OUI_SUBTP_SZ 1
#define DSCP_REQ_CTRL_SZ 1
#define DSCP_RSP_CTRL_SZ 1
#define DSCP_POL_ID_SZ 1
#define DSCP_POL_STS_SZ 1
#define DSCP_POL_STS_CNT_SZ 1
#define ATTR_ID_SZ 1
#define PORT_SZ 2
#define DSCP_REQ_TP_SZ 1
#define DSCP_SZ 1





/* Wi-Fi Alliance Capabilities element */
#define WFA_CAP_LEN_SZ 1



/* SCS Desc. Start*/
#define SET_SCS_DESC_ELE_ID(_ele) \
	SET_BITS_TO_LE_1BYTE(_ele, 0, 8, EID_SCS_DESCRIPTOR)
#define SET_SCS_DESC_LEN(_ele, _val) \
	SET_BITS_TO_LE_1BYTE(_ele, 0, 8, _val)
#define SET_SCS_DESC_SCSID(_ele, _val) \
	SET_BITS_TO_LE_1BYTE(_ele, 0, 8, _val)
#define SET_SCS_DESC_REQ_TP(_ele, _val) \
	SET_BITS_TO_LE_1BYTE(_ele, 0, 8, _val)

/* Intra-Access Category Priority Ele. */
#define SET_INTRA_ACC_CAT_ELE_ID(_ele) \
	SET_BITS_TO_LE_1BYTE(_ele, 0, 8, EID_INTER_AC_PRIO)
#define SET_INTRA_ACC_CAT_LEN(_ele) \
	SET_BITS_TO_LE_1BYTE(_ele, 0, 8, ELE_SZ + LEN_SZ + INTRA_ACC_PRI_SZ)
#define SET_INTRA_ACC_CAT_UP(_ele, _val) \
	SET_BITS_TO_LE_1BYTE(_ele, 0, 3, _val)
#define SET_INTRA_ACC_ALT_Q(_ele, _val) \
	SET_BITS_TO_LE_1BYTE(_ele, 3, 1, _val)
#define SET_INTRA_ACC_DEOP_ELG(_ele, _val) \
	SET_BITS_TO_LE_1BYTE(_ele, 4, 1, _val)


/* TCLAS Ele. */
#define SET_TCLAS_ELE_ID(_ele) \
	SET_BITS_TO_LE_1BYTE(_ele, 0, 8, EID_TCLAS)
#define SET_TCLAS_LEN(_ele, _val) \
	SET_BITS_TO_LE_1BYTE(_ele, 0, 8, _val)
#define SET_TCLAS_UP(_ele, _val) \
	SET_BITS_TO_LE_1BYTE(_ele, 0, 8, _val)
/* Classifier */
#define SET_TCLAS_CLF_TP(_ele, _val) \
	SET_BITS_TO_LE_1BYTE(_ele, 0, 8, _val)

/* TCLAS Type4 for IPV4 */
#define SET_TCLAS_TP4_CLF_MASK(_ele, _val) \
	SET_BITS_TO_LE_1BYTE(_ele, 0, 8, _val)
#define SET_TCLAS_TP4_VER(_ele, _val) \
	SET_BITS_TO_LE_1BYTE(_ele, 0, 8, _val)
#define SET_TCLAS_TP4_IPV4_IP(_ele, _val) \
	do { \
		SET_BITS_TO_LE_1BYTE(_ele, 0, 8, _val[0]);\
		SET_BITS_TO_LE_1BYTE(_ele + 1, 0, 8, _val[1]);\
		SET_BITS_TO_LE_1BYTE(_ele + 2, 0, 8, _val[2]);\
		SET_BITS_TO_LE_1BYTE(_ele + 3, 0, 8, _val[3]);\
	} while(false)
#define SET_TCLAS_TP4_IPV4_PORT(_ele, _val) \
	do { \
		SET_BITS_TO_LE_2BYTE(_ele, 0, 16, _val);\
	} while(false)
#define SET_TCLAS_TP4_IPV4_DSCP(_ele, _val) \
	SET_BITS_TO_LE_1BYTE(_ele, 0, 8, _val)
#define SET_TCLAS_TP4_IPV4_PROC(_ele, _val) \
	SET_BITS_TO_LE_1BYTE(_ele, 0, 8, _val)
#define SET_TCLAS_TP4_IPV4_RSVD(_ele, _val) \
	SET_BITS_TO_LE_1BYTE(_ele, 0, 8, _val)

/* TCLAS Type10 for IPV4 or IPV6*/
#define SET_TCLAS_TP10_PROT_INST(_ele, _val) \
	SET_BITS_TO_LE_1BYTE(_ele, 0, 8, _val)
#define SET_TCLAS_TP10_PROT_NUM_NEXT_HDR(_ele, _val) \
	SET_BITS_TO_LE_1BYTE(_ele, 0, 8, _val)
#define SET_TCLAS_TP10_FILTER(_ele, _val) \
	SET_BITS_TO_LE_1BYTE(_ele, 0, 8, _val)


/* TCLAS Processing Ele. */
#define SET_TCLAS_PROC_ELE_ID(_ele) \
	SET_BITS_TO_LE_1BYTE(_ele, 0, 8, EID_TCLAS_PROCESSING)
#define SET_TCLAS_PROC_LEN(_ele, _val) \
	SET_BITS_TO_LE_1BYTE(_ele, 0, 8, _val)
#define SET_TCLAS_PROC_PROCING(_ele, _val) \
	SET_BITS_TO_LE_1BYTE(_ele, 0, 8, _val)



#define GET_CLF_TP(_ele) \
	LE_BITS_TO_1BYTE(_ele, 0, 8)


/************************************************************
GET QoS Characteristics - Control Info subfield: _pStart = start of Control Info subfield
************************************************************/
#define GET_QOS_CONTROL_INFO_DIRECTION(_pStart) \
		LE_BITS_TO_4BYTE(_pStart, 0, 2)
#define GET_QOS_CONTROL_INFO_TID(_pStart) \
		LE_BITS_TO_4BYTE(_pStart, 2, 4)
#define GET_QOS_CONTROL_INFO_USER_PRIORITY(_pStart) \
		LE_BITS_TO_4BYTE(_pStart, 6, 3)
#define GET_QOS_CONTROL_INFO_BITMAP(_pStart) \
		LE_BITS_TO_4BYTE(_pStart, 9, 16)
#define GET_QOS_CONTROL_INFO_LINKID(_pStart) \
		LE_BITS_TO_4BYTE(_pStart, 25, 4)
/************************************************************
SET QoS Characteristics - Control Info subfield: _pStart = start of Control Info subfield
************************************************************/
#define SET_QOS_CONTROL_INFO_DIRECTION(_pStart, _val) \
		SET_BITS_TO_LE_4BYTE(_pStart, 0, 2, _val)
#define SET_QOS_CONTROL_INFO_TID(_pStart, _val) \
		SET_BITS_TO_LE_4BYTE(_pStart, 2, 4, _val)
#define SET_QOS_CONTROL_INFO_USER_PRIORITY(_pStart, _val) \
		SET_BITS_TO_LE_4BYTE(_pStart, 6, 3, _val)
#define SET_QOS_CONTROL_INFO_BITMAP(_pStart, _val) \
		SET_BITS_TO_LE_4BYTE(_pStart, 9, 16, _val)
#define SET_QOS_CONTROL_INFO_LINKID(_pStart, _val) \
		SET_BITS_TO_LE_4BYTE(_pStart, 25, 4, _val)

/************************************************************
GET QoS Characteristics Presence: _pStart = start of Control Info subfield
************************************************************/
#define GET_QOS_ELE_MAX_MSDU_SIZE_PRESENT(_pStart) \
	LE_BITS_TO_4BYTE(_pStart, 9, 1)
#define GET_QOS_ELE_SERVICE_START_TIME_PRESENT(_pStart) \
	LE_BITS_TO_4BYTE(_pStart, 10, 1)
#define GET_QOS_ELE_MEAN_DATA_RATE_PRESENT(_pStart) \
	LE_BITS_TO_4BYTE(_pStart, 11, 1)
#define GET_QOS_ELE_BURST_SIZE_PRESENT(_pStart) \
	LE_BITS_TO_4BYTE(_pStart, 12, 1)
#define GET_QOS_ELE_MSDU_LIFETIME_PRESENT(_pStart) \
	LE_BITS_TO_4BYTE(_pStart, 13, 1)
#define GET_QOS_ELE_MSDU_DELIVERY_RATIO_PRESENT(_pStart) \
	LE_BITS_TO_4BYTE(_pStart, 14, 1)
#define GET_QOS_ELE_MSDU_COUNT_EXPONENT_PRESENT(_pStart) \
	LE_BITS_TO_4BYTE(_pStart, 15, 1)
#define GET_QOS_ELE_MEDIUM_TIME_PRESENT(_pStart) \
	LE_BITS_TO_4BYTE(_pStart, 16, 1)
/************************************************************
GET QoS Characteristics subfield: _pStart = start of QoS Characteristics subfield
************************************************************/
#define GET_QOS_ELE_LENGTH(_pStart) \
		LE_BITS_TO_1BYTE(_pStart, 0, 8)
#define GET_QOS_ELE_MINIMUM_SERVICE_INTERVAL(_pStart) \
		LE_BITS_TO_4BYTE(_pStart, 0, 32)
#define GET_QOS_ELE_MAXIMUM_SERVICE_INTERVAL(_pStart) \
		LE_BITS_TO_4BYTE(_pStart, 0, 32)
#define GET_QOS_ELE_MINIMUM_DATA_RATE(_pStart) \
		LE_BITS_TO_4BYTE(_pStart, 0, 24)
#define GET_QOS_ELE_DELAY_BOUND(_pStart) \
		LE_BITS_TO_4BYTE(_pStart, 0, 24)
#define GET_QOS_ELE_MAXIMUM_MSDU_SIZE(_pStart) \
		LE_BITS_TO_2BYTE(_pStart, 0, 16)
#define GET_QOS_ELE_SERVICE_START_TIME(_pStart) \
		LE_BITS_TO_4BYTE(_pStart, 0, 32)
#define GET_QOS_ELE_SERVICE_START_TIME_LNKID(_pStart) \
		LE_BITS_TO_1BYTE(_pStart, 0, 8)
#define GET_QOS_ELE_MEAN_DATA_RATE(_pStart) \
		LE_BITS_TO_4BYTE(_pStart, 0, 24)
#define GET_QOS_ELE_BURST_SIZE(_pStart) \
		LE_BITS_TO_4BYTE(_pStart, 0, 32)
#define GET_QOS_ELE_MSDU_LIFETIME(_pStart) \
		LE_BITS_TO_2BYTE(_pStart, 0, 16)
#define GET_QOS_ELE_MSDU_DELIVERY_RATIO(_pStart) \
		LE_BITS_TO_1BYTE(_pStart, 0, 8)
#define GET_QOS_ELE_MSDU_COUNT_EXPONENT(_pStart) \
		LE_BITS_TO_1BYTE(_pStart, 0, 8)
#define GET_QOS_ELE_MEDIUM_TIME(_pStart) \
		LE_BITS_TO_2BYTE(_pStart, 0, 16)
/************************************************************
SET QoS Characteristics subfield: _pStart = start of QoS Characteristics subfield
************************************************************/
#define SET_QOS_ELE_MINIMUM_SERVICE_INTERVAL(_pStart, _val) \
	SET_BITS_TO_LE_4BYTE(_pStart, 0, 32, _val)
#define SET_QOS_ELE_MAXIMUM_SERVICE_INTERVAL(_pStart, _val) \
	SET_BITS_TO_LE_4BYTE(_pStart, 0, 32, _val)
#define SET_QOS_ELE_MINIMUM_DATA_RATE(_pStart, _val) \
	SET_BITS_TO_LE_4BYTE(_pStart, 0, 24, _val)
#define SET_QOS_ELE_DELAY_BOUND(_pStart, _val) \
	SET_BITS_TO_LE_4BYTE(_pStart, 0, 24, _val)
#define SET_QOS_ELE_MAXIMUM_MSDU_SIZE(_pStart, _val) \
	SET_BITS_TO_LE_2BYTE(_pStart, 0, 16, _val)
#define SET_QOS_ELE_SERVICE_START_TIME(_pStart, _val) \
	SET_BITS_TO_LE_4BYTE(_pStart, 0, 32, _val)
#define SET_QOS_ELE_SERVICE_START_TIME_LNKID(_pStart, _val) \
	SET_BITS_TO_LE_1BYTE(_pStart, 0, 8, _val)
#define SET_QOS_ELE_MEAN_DATA_RATE(_pStart, _val) \
	SET_BITS_TO_LE_4BYTE(_pStart, 0, 24, _val)
#define SET_QOS_ELE_BURST_SIZE(_pStart, _val) \
	SET_BITS_TO_LE_4BYTE(_pStart, 0, 32, _val)
#define SET_QOS_ELE_MSDU_LIFETIME(_pStart, _val) \
	SET_BITS_TO_LE_2BYTE(_pStart, 0, 16, _val)
#define SET_QOS_ELE_MSDU_DELIVERY_INFO(_pStart, _val) \
	SET_BITS_TO_LE_1BYTE(_pStart, 0, 8, _val)
#define SET_QOS_ELE_MSDU_DELIVERY_RATIO(_pStart, _val) \
	SET_BITS_TO_LE_1BYTE(_pStart, 0, 8, _val)
#define SET_QOS_ELE_MSDU_COUNT_EXPONENT(_pStart, _val) \
	SET_BITS_TO_LE_1BYTE(_pStart, 0, 8, _val)
#define SET_QOS_ELE_MEDIUM_TIME(_pStart, _val) \
	SET_BITS_TO_LE_2BYTE(_pStart, 0, 16, _val)

/* SCS Desc. End*/

/* SCS Rsp Start */
#define SET_SCS_RSP_CNT(_ele, _val) \
	SET_BITS_TO_LE_1BYTE(_ele, 0, 8, _val)
#define SET_SCS_RSP_SCSID(_ele, _val) \
	SET_BITS_TO_LE_1BYTE(_ele, 0, 8, _val)
#define SET_SCS_RSP_STS(_ele, _val) \
	SET_BITS_TO_LE_2BYTE(_ele, 0, 16, _val)

#define GET_SCS_RSP_CNT(_ele) \
	LE_BITS_TO_1BYTE(_ele, 0, 8)
#define GET_SCS_RSP_SCSID(_ele) \
	LE_BITS_TO_1BYTE(_ele, 0, 8)
#define GET_SCS_RSP_STS(_ele) \
	LE_BITS_TO_2BYTE(_ele, 0, 16)
/* SCS Rsp End */

/*  */
#define SET_WFA_CAP_TOTAL_LEN(_ele, _val) \
	SET_BITS_TO_LE_1BYTE(_ele, 0, 8, _val)
#define SET_WFA_CAP_LEN(_ele, _val) \
	SET_BITS_TO_LE_1BYTE(_ele, 0, 8, _val)
#define SET_WFA_CAP(_ele, _val) \
	SET_BITS_TO_LE_1BYTE(_ele, 0, 8, _val)


/* QoSMap Start */
#define SET_DSCP_EXC_DSCP(_ele, _val) \
	SET_BITS_TO_LE_1BYTE(_ele, 0, 8, _val)
#define SET_DSCP_EXC_UP(_ele, _val) \
	SET_BITS_TO_LE_1BYTE(_ele, 0, 8, _val)
#define SET_DSCP_RNG_DSCP_L(_ele, _val) \
	SET_BITS_TO_LE_1BYTE(_ele, 0, 8, _val)
#define SET_DSCP_RNG_DSCP_H(_ele, _val) \
	SET_BITS_TO_LE_1BYTE(_ele, 0, 8, _val)

#define GET_DSCP_EXC_DSCP(_ele) \
	LE_BITS_TO_1BYTE(_ele, 0, 8)
#define GET_DSCP_EXC_UP(_ele) \
	LE_BITS_TO_1BYTE(_ele, 0, 8)
#define GET_DSCP_RNG_DSCP_L(_ele) \
	LE_BITS_TO_1BYTE(_ele, 0, 8)
#define GET_DSCP_RNG_DSCP_H(_ele) \
	LE_BITS_TO_1BYTE(_ele, 0, 8)
/* QoSMap End*/

/* MSCS Start */
/* MSCS Desc. */
#define SET_MSCS_REQ_TP(_ele, _val) \
	SET_BITS_TO_LE_1BYTE(_ele, 0, 8, _val)
#define SET_MSCS_UP_CTRL_BITMAP(_ele, _val) \
	SET_BITS_TO_LE_1BYTE(_ele, 0, 8, _val)
#define SET_MSCS_UP_CTRL_LIM(_ele, _val) \
	SET_BITS_TO_LE_1BYTE(_ele, 0, 3, _val)
#define SET_MSCS_STREAM_TIMEOUT(_ele, _val) \
	SET_BITS_TO_LE_4BYTE(_ele, 0, 32, _val)

#define SET_MSCS_SUB_ELE_ID(_ele, _id) \
	SET_BITS_TO_LE_1BYTE(_ele, 0, 8, _id)
#define SET_MSCS_SUB_ELE_LEN(_ele, _val) \
	SET_BITS_TO_LE_1BYTE(_ele, 0, 8, _val)

#define SET_MSCS_SUB_ELE_MCSC_STS(_ele, _val) \
	SET_BITS_TO_LE_2BYTE(_ele, 0, 16, _val)

/* MSCS Rsp action frame */
#define SET_MSCS_RSP_STS(_ele, _val) \
	SET_BITS_TO_LE_2BYTE(_ele, 0, 16, _val)

#define GET_MSCS_RSP_STS(_ele) \
	LE_BITS_TO_2BYTE(_ele, 0, 16)
/* Element ID */
enum mscs_desc_sub_eid {
	MSCS_SUBEID_MSCS_STS = 1 /* MSCS Status */
};


/* MSCS End */

/* DSCP Policy start */

enum qm_attr_id {
	QM_ATTR_PORT = 1, /* Port Range attribute */
	QM_ATTR_DSCP = 2, /* DSCP Policy attribute */
	QM_ATTR_TCLAS = 3, /* TCLAS attribute */
	QM_ATTR_DOMAIN = 4, /* Domain Name attribute */
};




#define SET_QM_OUI_SUBTP(_ele, _val) \
	SET_BITS_TO_LE_1BYTE(_ele, 0, 1, _val)

#define SET_DSCP_REQ_CTRL_MORE(_ele, _val) \
	SET_BITS_TO_LE_1BYTE(_ele, 0, 1, _val)
#define SET_DSCP_REQ_CTRL_RESET(_ele, _val) \
	SET_BITS_TO_LE_1BYTE(_ele, 1, 1, _val)
#define SET_DSCP_POL_ID(_ele, _val) \
	SET_BITS_TO_LE_1BYTE(_ele, 0, 8, _val)

#define SET_QM_ATTR_ID(_ele, _val) \
	SET_BITS_TO_LE_1BYTE(_ele, 0, 8, _val)
#define SET_QM_ATTR_LEN(_ele, _val) \
	SET_BITS_TO_LE_1BYTE(_ele, 0, 8, _val)
/* Port Range Attribute */
#define SET_PORT_RNG_ATTR_PORT(_pStart, _val) \
	SET_BITS_TO_BE_2BYTE(_pStart, 0, 16, _val)
/* DSCP Policy Attribute */
#define SET_DSCP_POL_REQ_TP(_ele, _val) \
	SET_BITS_TO_LE_1BYTE(_ele, 0, 8, _val)
#define SET_DSCP_POL_DSCP(_ele, _val) \
	SET_BITS_TO_LE_1BYTE(_ele, 0, 8, _val)

/* DSCP Policy Response Action frame */
#define SET_DSCP_RSP_CTRL_MORE(_ele, _val) \
	SET_BITS_TO_LE_1BYTE(_ele, 0, 1, _val)
#define SET_DSCP_RSP_CTRL_RESET(_ele, _val) \
	SET_BITS_TO_LE_1BYTE(_ele, 1, 1, _val)
#define SET_DSCP_POL_STS_CNT(_ele, _val) \
	SET_BITS_TO_LE_1BYTE(_ele, 0, 8, _val)
#define SET_DSCP_POL_STS(_ele, _val) \
	SET_BITS_TO_LE_1BYTE(_ele, 0, 8, _val)

#define GET_DSCP_REQ_CTRL_MORE(_ele) \
	LE_BITS_TO_1BYTE(_ele, 0, 1)
#define GET_DSCP_REQ_CTRL_RESET(_ele) \
	LE_BITS_TO_1BYTE(_ele, 1, 1)
#define GET_DSCP_POL_ID(_ele) \
	LE_BITS_TO_1BYTE(_ele, 0, 8)

#define GET_QM_ATTR_ID(_ele) \
	LE_BITS_TO_1BYTE(_ele, 0, 8)

/* Port Range Attribute */
#define GET_PORT_RNG_ATTR_PORT(_pStart) \
	LE_BITS_TO_2BYTE(_pStart, 0, 16)
/* DSCP Policy Attribute */
#define GET_DSCP_POL_REQ_TP(_ele) \
	LE_BITS_TO_1BYTE(_ele, 0, 8)
#define GET_DSCP_POL_DSCP(_ele) \
	LE_BITS_TO_1BYTE(_ele, 0, 8)

/* DSCP Policy Response Action frame */
#define GET_DSCP_RSP_CTRL_MORE(_ele) \
	LE_BITS_TO_1BYTE(_ele, 0, 1)
#define GET_DSCP_RSP_CTRL_RESET(_ele) \
	LE_BITS_TO_1BYTE(_ele, 1, 1)
#define GET_DSCP_POL_STS_CNT(_ele) \
	LE_BITS_TO_1BYTE(_ele, 0, 8)
#define GET_DSCP_POL_STS(_ele) \
	LE_BITS_TO_1BYTE(_ele, 0, 8)

/* DSCP Policy end */

void phl_qm_deinit(struct phl_info_t *phl);

enum rtw_phl_status phl_qm_init(struct phl_info_t *phl);

#endif /*_PHL_QOS_MG_H_*/
#endif /* CONFIG_QOS_MG */
