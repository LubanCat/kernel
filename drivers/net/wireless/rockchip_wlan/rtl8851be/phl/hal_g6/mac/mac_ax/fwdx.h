/******************************************************************************
 *
 * Copyright(c) 2019 Realtek Corporation. All rights reserved.
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
 ******************************************************************************/
#ifndef _MAC_FWDX_H_
#define _MAC_FWDX_H_
#include "../type.h"

#if MAC_SELF_DIAG_INFO

#define ENV_IS_PXP (adapter->env_info.env == DUT_ENV_PXP)

#define FWDX_MAX_ITEM 96UL
#define FWDX_CTRL_ACK_TIMEOUT (ENV_IS_PXP ? 300UL : 50UL)
#define FWDX_TEST_ACK_TIMEOUT (ENV_IS_PXP ? 300UL : 50UL)

#define FWDX_EXIST (adapter->fw_info.fwdx_info.fwdx_exist)
#define FWDX_INIT_DONE (adapter->fw_info.fwdx_info.fwdx_init_done)
#define FWDX_TIMER_RUNNING (adapter->fw_info.fwdx_info.fwdx_timer_running)
#define FWDX_CTRL_COUNT (adapter->fw_info.fwdx_info.fwdx_ctrl_count)
#define FWDX_ITEM_CTRL_BLOCK_ARRAY (adapter->fw_info.fwdx_info.fwdx_item_ctrl_block_array)
#define FWDX_CONFIG (adapter->fw_info.fwdx_info.fwdx_config)
#define FWDX_ITEM_CTRL_BLOCK_LEN sizeof(struct fwdx_item_ctrl_block)
#define FWDX_SUPPORT_ITEM_NUM FWDX_ITEM_ID_MAX

#define FWDX_NEXT_RECORD_OF_RECORD_ON_RING(CTRL, RECORD) \
((RECORD) + (CTRL)->record_len >= (CTRL)->ring_buffer_end ? \
(CTRL)->ring_buffer_head : (RECORD) + (CTRL)->record_len)

#define FWDX_NEXT_RECORD_ON_RING(CTRL) \
FWDX_NEXT_RECORD_OF_RECORD_ON_RING((CTRL), (CTRL)->ring_buffer_curr)

#define FWDX_PREVIOUS_RECORD_OF_RECORD_ON_RING(CTRL, RECORD) \
((RECORD) == (CTRL)->ring_buffer_head ? \
(CTRL)->ring_buffer_end - (CTRL)->record_len : (RECORD) - (CTRL)->record_len)

#define FWDX_PREVIOUS_RECORD_ON_RING(CTRL) \
FWDX_PREVIOUS_RECORD_OF_RECORD_ON_RING(CTRL, (CTRL)->ring_buffer_curr)

#define FWDX_LATEST_RECORD_ON_RING(CTRL) FWDX_PREVIOUS_RECORD_ON_RING(CTRL)

#define FWDX_GET_RECORD_IDX_ON_RING(CTRL) \
(((CTRL)->ring_buffer_curr - (CTRL)->ring_buffer_head) / (CTRL)->record_len)

#if PLATFOM_IS_LITTLE_ENDIAN
#define ENDIAN_REVERTER(buff, buf_size)
#else // PLATFORM_IS_BIG_ENDIAN
#define ENDIAN_REVERTER(buff, buf_size)				\
{								\
	u8 idx;							\
	u32 *temp;						\
								\
	temp = (u32 *)&(buff);					\
	for (idx = 0; idx < (u8)((buf_size) >> 2); idx++)	\
		temp[idx] = le32_to_cpu(temp[idx]);		\
}
#endif

struct fwdx_ctrl_containter {
#if PLATFOM_IS_LITTLE_ENDIAN
	u32 fwdx_time_interval : 16;
	u32 rsvd0 : 16;
	u32 fwdx_heap_remain : 1;
	u32 fwdx_irq_cnt : 1;
	u32 fwdx_axidma_debug_info : 1;
	u32 fwdx_cpuio_debug_info : 1;
	u32 rsvd1 : 28;
	u32 rsvd2 : 32;
	u32 rsvd3 : 32;
#else // PLATFORM_IS_BIG_ENDIAN
	u32 rsvd0 : 16;
	u32 fwdx_time_interval : 16;
	u32 rsvd1 : 28;
	u32 fwdx_cpuio_debug_info : 1;
	u32 fwdx_axidma_debug_info : 1;
	u32 fwdx_irq_cnt : 1;
	u32 fwdx_heap_remain : 1;
	u32 rsvd2 : 32;
	u32 rsvd3 : 32;
#endif
};

struct fwdx_item_hdr {
#if PLATFOM_IS_LITTLE_ENDIAN
	u32 total_len : 16; // include self in bytes
	u32 fwdx_item_seq_num : 8;
	u32 fwdx_item_id : 8;
	u32 record_cnt : 16;
	u32 record_len : 16; // in bytes
#else // PLATFORM_IS_BIG_ENDIAN
	u32 fwdx_item_id : 8;
	u32 fwdx_item_seq_num : 8;
	u32 total_len : 16; // include self in bytes
	u32 record_len : 16; // in bytes
	u32 record_cnt : 16;
#endif
};

typedef u32(*handler_cb)(struct mac_ax_adapter *adapter, struct fwdx_item_ctrl_block *ctrl,
			  u32 new_item_num);

typedef u32(*detecter_cb)(struct mac_ax_adapter *adapter, struct fwdx_item_ctrl_block *ctrl,
			  u32 input_len, void *input, u32 *output_len, void *output);

struct fwdx_item_ctrl_block {
	u32 record_received;
	u32 record_num;
	u32 record_len;
	//Ring Buffer for record, use to store the history of fwdx item
	u32 ring_buffer_size; // = item_count * item_len
	u8 *ring_buffer_curr; // Point to addr which last known usable
	u8 *ring_buffer_end; // = record_ring_buff + record_ring_buff_size
	u8 *ring_buffer_head;
	handler_cb handler;
	detecter_cb detecter;
};

struct fwdx_heap_info {
	u32 timestamp;
	u32 heap_remain;
};

struct fwdx_irq_counter {
	u32 timestamp;
	u32 counter[16];
};

struct fwdx_axidma_info {
	u32 timestamp;
	u8 info[128];
};

struct fwdx_cpuio_info {
	u8 info[4000];
};

u32 mac_fwdx_c2h_handler(struct mac_ax_adapter *, u8 *, u32);

u32 mac_fwdx_init(struct mac_ax_adapter *adapter, u32 *ring_buffer_num,
		  u8 ring_buffer_num_size);

u32 mac_fwdx_deinit(struct mac_ax_adapter *adapter);

u32 mac_fwdx_start(struct mac_ax_adapter *adapter, struct mac_fwdx_config *item);

u32 mac_fwdx_stop(struct mac_ax_adapter *adapter);

u32 mac_fwdx_detect_error(struct mac_ax_adapter *adapter, enum fwdx_item_id item,
			  u32 input_len, void *input, u32 *output_len, void *output);

#endif // MAC_SELF_DIAG_INFO
#endif // _MAC_FWDX_H_
