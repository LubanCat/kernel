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
#include "fwdx.h"
#include "mac_priv.h"
#if MAC_SELF_DIAG_INFO
static u32 fwdx_info_struct_size[FWDX_MAX_ITEM] = {
	sizeof(struct fwdx_heap_info),
	sizeof(struct fwdx_irq_counter),
	sizeof(struct fwdx_axidma_info),
	sizeof(struct fwdx_cpuio_info)
};

/* Handler Callback will run every time a FWDX C2H received */
static u32 fwdx_heap_remain_handler(struct mac_ax_adapter *adapter,
				    struct fwdx_item_ctrl_block *ctrl, u32 new_item_num)
{
	struct fwdx_heap_info *heap_info;

	heap_info = (struct fwdx_heap_info *)FWDX_LATEST_RECORD_ON_RING(ctrl);
	PLTFM_MSG_TRACE("[FWDX] Timestamp = %d, heap remain = %d\n", heap_info->timestamp,
			 heap_info->heap_remain);
	return MACSUCCESS;
}

static u32 fwdx_irq_handler(struct mac_ax_adapter *adapter,
			    struct fwdx_item_ctrl_block *ctrl, u32 new_item_num)
{
	struct fwdx_irq_counter *irq_info;

	irq_info = (struct fwdx_irq_counter *)FWDX_LATEST_RECORD_ON_RING(ctrl);
	PLTFM_MSG_TRACE("[FWDX] Timestamp = %d, %x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x\n",
			 irq_info->timestamp, irq_info->counter[0], irq_info->counter[1],
			 irq_info->counter[2], irq_info->counter[3], irq_info->counter[4],
			 irq_info->counter[5], irq_info->counter[6], irq_info->counter[7],
			 irq_info->counter[8], irq_info->counter[9], irq_info->counter[10],
			 irq_info->counter[11], irq_info->counter[12], irq_info->counter[13],
			 irq_info->counter[14], irq_info->counter[15]);
	return MACSUCCESS;
}

static u32 fwdx_axidma_debug_handler(struct mac_ax_adapter *adapter,
				     struct fwdx_item_ctrl_block *ctrl, u32 new_item_num)
{
	return MACSUCCESS;
}

static u32 fwdx_cpu_io_handler(struct mac_ax_adapter *adapter,
			       struct fwdx_item_ctrl_block *ctrl, u32 new_item_num)
{
	return MACSUCCESS;
}

static handler_cb handler_cb_array[FWDX_MAX_ITEM] = {
	fwdx_heap_remain_handler,
	fwdx_irq_handler,
	fwdx_axidma_debug_handler,
	fwdx_cpu_io_handler
};

u8 fwdx_memleak_detect_start;
u32 start_heap;
u32 start_counter;
/* Detecter Callback will run when someone call it */
static u32 fwdx_memory_leak_detecter(struct mac_ax_adapter *adapter,
				     struct fwdx_item_ctrl_block *ctrl,
				     u32 input_len, void *input, u32 *output_len, void *output)
{
	struct mac_fwdx_config new_fwdx_config = { 0 };
	struct mac_ax_ops *mac_ops = adapter->ops;
	u32 start_record = *(u32 *)input;
	u32 leak_tolerance = *((u32 *)input + 1);
	u32 *out_result = (u32 *)output;
	u32 *out_heap_lost = (u32 *)output + 1;
	u32 time = FWDX_CTRL_ACK_TIMEOUT;
	u32 counter0 = ctrl->record_received;
	u32 counter1 = ctrl->record_received;
	s32 diff;
	struct fwdx_heap_info *heap_info;

	if (start_record) {
		//Start Record
		start_heap = 0;
		if (!FWDX_TIMER_RUNNING) {
			//There isn't any FWDX running, start a single shot one.
			new_fwdx_config.fwdx_enable_item.fwdx_heap_remain = 1;
			mac_ops->fwdx_start(adapter, &new_fwdx_config);
		} else if (!FWDX_CONFIG.fwdx_enable_item.fwdx_heap_remain) {
			//There is a FWDX running, but needed item is not enable
			//copy the old one config, enable the needed and restart FWDX
			PLTFM_MEMCPY(&new_fwdx_config, &FWDX_CONFIG,
				     sizeof(struct mac_fwdx_config));
			new_fwdx_config.fwdx_enable_item.fwdx_heap_remain = 1;
			mac_ops->fwdx_stop(adapter);
			mac_ops->fwdx_start(adapter, &new_fwdx_config);
		}
		//Make sure we received at least one record
		while (counter1 != counter0 + 1 && time--) {
			counter1 = ctrl->record_received;
			PLTFM_DELAY_MS(1);
		}
		if (!time)
			return MACPOLLTO;
		heap_info = (struct fwdx_heap_info *)FWDX_LATEST_RECORD_ON_RING(ctrl);
		start_heap = heap_info->heap_remain;
		start_counter = ctrl->record_received;
		fwdx_memleak_detect_start = 1;
	} else {
		//Stop Record
		if (!fwdx_memleak_detect_start)
			return MACPROCERR;
		if (!FWDX_TIMER_RUNNING) {
			//There isn't any FWDX running, start a single shot one.
			new_fwdx_config.fwdx_enable_item.fwdx_heap_remain = 1;
			mac_ops->fwdx_start(adapter, &new_fwdx_config);
		} else if (!FWDX_CONFIG.fwdx_enable_item.fwdx_heap_remain) {
			//There is a FWDX running, but needed item is not enable
			//copy the old one config, enable the needed and restart FWDX
			PLTFM_MEMCPY(&new_fwdx_config, &FWDX_CONFIG,
				     sizeof(struct mac_fwdx_config));
			new_fwdx_config.fwdx_enable_item.fwdx_heap_remain = 1;
			mac_ops->fwdx_stop(adapter);
			mac_ops->fwdx_start(adapter, &new_fwdx_config);
		}
		//Make sure we received at least one record
		while (counter1 != counter0 + 1 && time--) {
			counter1 = ctrl->record_received;
			PLTFM_DELAY_MS(1);
		}
		if (!time)
			return MACPOLLTO;
		heap_info = (struct fwdx_heap_info *)FWDX_LATEST_RECORD_ON_RING(ctrl);
		if (start_counter == ctrl->record_received)
			PLTFM_MSG_WARN("[FWDX][WARN] Heap record nothing after detect start.\n");
		diff = start_heap - heap_info->heap_remain;
		*out_heap_lost = (u32)diff;
		if (diff >= (s32)leak_tolerance)
			*out_result = MACMEMLEAK;
		else
			*out_result = MACSUCCESS;
		*output_len = 2;
		fwdx_memleak_detect_start = 0;
	}
	return MACSUCCESS;
}

static u32 fwdx_irq_detecter(struct mac_ax_adapter *adapter,
			     struct fwdx_item_ctrl_block *ctrl,
			     u32 input_len, void *input, u32 *output_len, void *output)
{
	return MACSUCCESS;
}

static u32 fwdx_axidma_debug_detecter(struct mac_ax_adapter *adapter,
				      struct fwdx_item_ctrl_block *ctrl,
				      u32 input_len, void *input, u32 *output_len, void *output)
{
	return MACSUCCESS;
}

static u32 fwdx_cpu_io_detecter(struct mac_ax_adapter *adapter,
				struct fwdx_item_ctrl_block *ctrl,
				u32 input_len, void *input, u32 *output_len, void *output)
{
	return MACSUCCESS;
}

static detecter_cb detecter_cb_array[FWDX_MAX_ITEM] = {
	fwdx_memory_leak_detecter,
	fwdx_irq_detecter,
	fwdx_axidma_debug_detecter,
	fwdx_cpu_io_detecter
};

static u32 fwdx_item_parser(struct mac_ax_adapter *adapter, struct fwdx_item_hdr *item_hdr,
			    u8 *item_buff)
{
	u32 idx;
	u32 item_info_len = item_hdr->total_len - sizeof(struct fwdx_item_hdr);
	u32 ret = MACSUCCESS;
	u32 item_record_count;
	struct fwdx_item_ctrl_block *ctrl;

	if (item_hdr->fwdx_item_id >= FWDX_ITEM_ID_MAX) {
		PLTFM_MSG_ERR("[FWDX][ERROR] fwdx_item_id Error %d\n", item_hdr->fwdx_item_id);
		return MACNOITEM;
	}
	ctrl = &FWDX_ITEM_CTRL_BLOCK_ARRAY[item_hdr->fwdx_item_id];
	if (!item_hdr->fwdx_item_seq_num) {
		// Check if item record len from c2h as same as the size store in halmac
		if (item_hdr->record_len != ctrl->record_len) {
			PLTFM_MSG_ERR("[FWDX][ERROR] item length miss match %d != %d\n",
				      item_hdr->record_len, ctrl->record_len);
			return MACCMP;
		}
		// Check if all record size equal to item_info from c2h
		if (item_hdr->record_len * item_hdr->record_cnt != item_info_len) {
			PLTFM_MSG_ERR("[FWDX][ERROR] item total length miss match %d * %d != %d\n",
				      item_hdr->record_len, item_hdr->record_cnt, item_info_len);
			return MACCMP;
		}
		//Loop all item record from c2h and memcpy to record ring buffer in ctrl.
		for (idx = 0; idx < item_hdr->record_cnt; idx++) {
			PLTFM_MEMCPY(ctrl->ring_buffer_curr, item_buff, ctrl->record_len);
			ENDIAN_REVERTER(ctrl->ring_buffer_curr, ctrl->record_len);
			ctrl->ring_buffer_curr = FWDX_NEXT_RECORD_ON_RING(ctrl);
			item_buff += ctrl->record_len;
		}
	} else {
		PLTFM_MSG_ERR("[FWDX][ERROR] Long packet type not support\n");
	}
	item_record_count = item_hdr->record_cnt;
	ctrl->record_received += item_record_count;
	if (item_hdr->record_cnt > ctrl->record_num) {
		PLTFM_MSG_WARN("[FWDX][WARN] C2H Entity Count outrun buffer design max\n");
		PLTFM_MSG_WARN("[FWDX][WARN] %d > %d\n", item_hdr->record_cnt, ctrl->record_num);
		PLTFM_MSG_WARN("[FWDX][WARN] FWDX Information Lost\n");
		PLTFM_MSG_WARN("[FWDX][WARN] Consider incresss ring buffer len\n");
		item_record_count = ctrl->record_num;
	}
	//Call handler every time c2h received and fwdx item received.
	ret = ctrl->handler(adapter, ctrl, item_record_count);
	if (ret)
		return ret;
	return MACSUCCESS;
}

static u32 fwdx_ctrl_handler(struct mac_ax_adapter *adapter, struct fwdx_ctrl_containter *fwdx_ctrl)
{
	PLTFM_MSG_TRACE("[FWDX][INFO] Get FWDX Ctrl Packet.\n");
	FWDX_CONFIG.fwdx_time_interval = (u16)fwdx_ctrl->fwdx_time_interval;
	FWDX_CONFIG.fwdx_enable_item.fwdx_heap_remain = (u8)fwdx_ctrl->fwdx_heap_remain;
	FWDX_CONFIG.fwdx_enable_item.fwdx_irq_cnt = (u8)fwdx_ctrl->fwdx_irq_cnt;
	FWDX_CONFIG.fwdx_enable_item.fwdx_axidma_debug_info = (u8)fwdx_ctrl->fwdx_axidma_debug_info;
	FWDX_CONFIG.fwdx_enable_item.fwdx_cpuio_debug_info = (u8)fwdx_ctrl->fwdx_cpuio_debug_info;
	FWDX_CTRL_COUNT++;
	if (!FWDX_TIMER_RUNNING && FWDX_CONFIG.fwdx_time_interval) {
		// Case 1: FWDX FW Timer Task Start with Auto Reload
		PLTFM_MSG_TRACE("[FWDX][INFO] FWDX Start ACK from FW\n");
		PLTFM_MSG_TRACE("[FWDX][INFO] FWDX FW Time Interval = %d ms\n",
				 FWDX_CONFIG.fwdx_time_interval);
		FWDX_TIMER_RUNNING = 1;
	} else if (FWDX_TIMER_RUNNING && !FWDX_CONFIG.fwdx_time_interval) {
		// Case 2: FWDX FW Timer Task Stop
		PLTFM_MSG_TRACE("[FWDX][INFO] FWDX Stop ACK from FW\n");
		FWDX_TIMER_RUNNING = 0;
	} else if (!FWDX_TIMER_RUNNING && !FWDX_CONFIG.fwdx_time_interval) {
		// Case 3 : FWDX FW Timer Task Single Shot
		PLTFM_MSG_TRACE("[FWDX][INFO] FWDX Start ACK from FW\n");
		PLTFM_MSG_TRACE("[FWDX][INFO] FWDX Single Shot.\n");
		FWDX_TIMER_RUNNING = 0;
	} else { //(FWDX_TIMER_RUNNING && FWDX_CONFIG.fwdx_time_interval)
		// Case 4 : Sepcial Case, should NOT enter here
		// If you stop cpu without stop fwdx, than fwdl and start fwdx, you will enter here
		PLTFM_MSG_WARN("[FWDX][WARN] FWDX Ctrl Block Received During FWDX Enable.\n");
		FWDX_TIMER_RUNNING = 1;
	}
	return MACSUCCESS;
}

u32 mac_fwdx_c2h_handler(struct mac_ax_adapter *adapter, u8 *info_buffer, u32 length)
{
	u8 *item_ptr;
	u32 ret = MACSUCCESS;
	struct fwdx_ctrl_containter fwdx_ctrl;
	struct fwdx_item_hdr fwdx_item_hdr;

	PLTFM_MSG_TRACE("[FWDX][INFO] Get Info Buffer Length = 0x%08X\n", length);
	if (!FWDX_INIT_DONE)
		return MACPROCERR;
	//Copy Fwdx ctrl block from buffer into local structure and handle endian issue
	PLTFM_MEMCPY(&fwdx_ctrl, info_buffer, sizeof(struct fwdx_ctrl_containter));
	ENDIAN_REVERTER(fwdx_ctrl, sizeof(struct fwdx_ctrl_containter));
	if (length == sizeof(struct fwdx_ctrl_containter)) {
		// This packet contain only FWDX Ctrl Block, update fwdx ctrl in adapter and return
		return fwdx_ctrl_handler(adapter, &fwdx_ctrl);
	} else {
		// This packet contain FWDX Ctrl Block AND FWDX Item Info.
		if (FWDX_CONFIG.fwdx_time_interval != (u16)fwdx_ctrl.fwdx_time_interval) {
			// At lease check fwdx_time_interval equal to the one store in adapter
			PLTFM_MSG_ERR("[FWDX][ERR][%s] Timer Interval Check Fail\n", __func__);
			PLTFM_MSG_ERR("[FWDX][ERR][%s] %d (adapter) != %d (c2h)\n", __func__,
				      FWDX_CONFIG.fwdx_time_interval, fwdx_ctrl.fwdx_time_interval);
			return MACCMP;
		}
	}
	length -= sizeof(struct fwdx_ctrl_containter);
	item_ptr = info_buffer + sizeof(struct fwdx_ctrl_containter);
	while (length) {
		PLTFM_MEMCPY(&fwdx_item_hdr, item_ptr, sizeof(struct fwdx_item_hdr));
		ENDIAN_REVERTER(fwdx_item_hdr, sizeof(struct fwdx_item_hdr));
		ret = fwdx_item_parser(adapter, &fwdx_item_hdr,
				       item_ptr + sizeof(struct fwdx_item_hdr));
		if (ret)
			return ret;
		length -= fwdx_item_hdr.total_len;
		item_ptr += fwdx_item_hdr.total_len;
	}
	return MACSUCCESS;
}

static u32 fwdx_ctrl_send(struct mac_ax_adapter *adapter, struct fwdx_ctrl_containter *fwdx_ctrl)
{
	struct h2c_info h2c_info = { 0 };

	h2c_info.agg_en = 0;
	h2c_info.content_len = cpu_to_le16(sizeof(struct fwcmd_fw_self_dx));
	h2c_info.h2c_cat = FWCMD_H2C_CAT_MAC;
	h2c_info.h2c_class = FWCMD_H2C_CL_FW_INFO;
	h2c_info.h2c_func = FWCMD_H2C_FUNC_FW_SELF_DX;
	h2c_info.rec_ack = 0;
	h2c_info.done_ack = 0;
	return mac_h2c_common(adapter, &h2c_info, (u32 *)fwdx_ctrl);
}

static u32 fwdx_check_exist(struct mac_ax_adapter *adapter)
{
	u32 ret = MACSUCCESS;
	struct h2c_info h2c_info = { 0 };
	u8 magic_content[16] = {'F', 'W', 'D', 'X', 'T', 'E', 'S', 'T',
				'F', 'W', 'D', 'X', 'T', 'E', 'S', 'T'};
	u32 time = FWDX_TEST_ACK_TIMEOUT;
	struct rtw_phl_com_t *phl_com = (struct rtw_phl_com_t *)adapter->phl_adapter;
	struct rtw_wcpu_cap_t *wcpu_cap = &phl_com->dev_cap.wcpu_cap;

	if (wcpu_cap->valid) {
		if (wcpu_cap->mac_ofld_cap.fwdx)
			FWDX_EXIST = 1;
		else
			FWDX_EXIST = 0;
		return ret;
	}

	h2c_info.agg_en = 0;
	h2c_info.content_len = cpu_to_le16(16);
	h2c_info.h2c_cat = FWCMD_H2C_CAT_MAC;
	h2c_info.h2c_class = FWCMD_H2C_CL_FW_INFO;
	h2c_info.h2c_func = FWCMD_H2C_FUNC_FW_SELF_DX;
	h2c_info.rec_ack = 0;
	h2c_info.done_ack = 1;

	ret = mac_h2c_common(adapter, &h2c_info, (u32 *)magic_content);
	if (ret) {
		PLTFM_MSG_ERR("[FWDX][ERR][%s] Send H2C Fail\n", __func__);
		return ret;
	}

	// Polling and check fwdl exist
	while (!FWDX_EXIST && time--)
		PLTFM_DELAY_MS(1);
	return ret;
}

u32 mac_fwdx_init(struct mac_ax_adapter *adapter, u32 *ring_buffer_num,
		  u8 ring_buffer_num_size)
{
	u32 idx;
	struct fwdx_item_ctrl_block *ctrl;
	u32 ctb_array_len = FWDX_ITEM_CTRL_BLOCK_LEN * FWDX_SUPPORT_ITEM_NUM;
	u32 ret = MACSUCCESS;

	// First Check if FWDX exist in FW
	ret = fwdx_check_exist(adapter);
	if (ret) {
		PLTFM_MSG_ERR("[FWDX][ERR][%s] Check FWDX Exist Fail\n", __func__);
		return ret;
	}

	if (!FWDX_EXIST) {
		PLTFM_MSG_TRACE("[FWDX][INFO] FWDX Not Exist\n");
		return MACPROCERR;
	}
	if (FWDX_INIT_DONE)
		mac_fwdx_deinit(adapter);
	if (ring_buffer_num_size != FWDX_SUPPORT_ITEM_NUM) {
		PLTFM_MSG_ERR("[FWDX][ERR][%s] Size of Ring Buffer Num should be exactily %d\n",
			      __func__, FWDX_SUPPORT_ITEM_NUM);
		PLTFM_MSG_ERR("[FWDX][ERR][%s] Get %d instead\n", __func__, ring_buffer_num_size);
		return MACBUFSZ;
	}
	// Allocate an array of fwdx_item_ctrl, each item have it own ctrl block and ring buffer
	FWDX_ITEM_CTRL_BLOCK_ARRAY = (struct fwdx_item_ctrl_block *)PLTFM_MALLOC(ctb_array_len);
	if (!FWDX_ITEM_CTRL_BLOCK_ARRAY) {
		PLTFM_MSG_ERR("[FWDX][ERR][%s] Init Malloc Fail\n", __func__);
		mac_fwdx_deinit(adapter);
		return MACBUFALLOC;
	}
	/*
	 * Init a ring buffer in fwdx_item_ctrl
	 * Ring Buffer srore the pass information of FWDX item
	 */
	for (idx = 0; idx < ring_buffer_num_size; idx++) {
		ctrl = &FWDX_ITEM_CTRL_BLOCK_ARRAY[idx];
		ctrl->record_num = ring_buffer_num[idx];
		//Make sure there are at least one buffer
		ctrl->record_num = ctrl->record_num == 0 ? 1 : ctrl->record_num;
		ctrl->record_len = fwdx_info_struct_size[idx];
		ctrl->handler = handler_cb_array[idx];
		ctrl->detecter = detecter_cb_array[idx];
		ctrl->ring_buffer_size = ctrl->record_num * ctrl->record_len;
		ctrl->ring_buffer_head = (u8 *)PLTFM_MALLOC(ctrl->ring_buffer_size);
		if (!ctrl->ring_buffer_head) {
			PLTFM_MSG_ERR("[FWDX][ERR][%s] Init Malloc Fail at index %d\n", __func__,
				      idx);
			PLTFM_MSG_ERR("[FWDX][ERR][%s] Request Size = %d = %d * %d\n",
				      __func__, ctrl->ring_buffer_size, ctrl->record_num,
				      ctrl->record_len);
			mac_fwdx_deinit(adapter);
			return MACBUFALLOC;
		}
		PLTFM_MEMSET(ctrl->ring_buffer_head, 0, ctrl->ring_buffer_size);
		ctrl->ring_buffer_curr = ctrl->ring_buffer_head;
		ctrl->ring_buffer_end = ctrl->ring_buffer_head + ctrl->ring_buffer_size;
	}
	FWDX_CTRL_COUNT = 0;
	FWDX_INIT_DONE = 1;
	return MACSUCCESS;
}

u32 mac_fwdx_deinit(struct mac_ax_adapter *adapter)
{
	u32 idx;
	struct fwdx_item_ctrl_block *ctrl;

	if (!FWDX_INIT_DONE)
		return MACSUCCESS;
	if (FWDX_TIMER_RUNNING)
		mac_fwdx_stop(adapter);
	if (FWDX_ITEM_CTRL_BLOCK_ARRAY) {
		for (idx = 0; idx < FWDX_SUPPORT_ITEM_NUM; idx++) {
			ctrl = &FWDX_ITEM_CTRL_BLOCK_ARRAY[idx];
			PLTFM_FREE(ctrl->ring_buffer_head, ctrl->ring_buffer_size);
		}
		PLTFM_FREE(FWDX_ITEM_CTRL_BLOCK_ARRAY, FWDX_ITEM_CTRL_BLOCK_LEN *
			   FWDX_SUPPORT_ITEM_NUM);
	}
	FWDX_INIT_DONE = 0;
	return MACSUCCESS;
}

u32 mac_fwdx_start(struct mac_ax_adapter *adapter, struct mac_fwdx_config *item)
{
	struct fwdx_ctrl_containter fwdx_ctrl = { 0 };
	u32 ret = MACSUCCESS;
	u32 time = FWDX_CTRL_ACK_TIMEOUT;
	u32 ctrl_count0 = FWDX_CTRL_COUNT;
	u32 ctrl_count1 = FWDX_CTRL_COUNT;

	if (!FWDX_INIT_DONE)
		return MACSUCCESS;
	fwdx_ctrl.fwdx_time_interval = item->fwdx_time_interval;
	fwdx_ctrl.fwdx_heap_remain = item->fwdx_enable_item.fwdx_heap_remain;
	fwdx_ctrl.fwdx_irq_cnt = item->fwdx_enable_item.fwdx_irq_cnt;
	fwdx_ctrl.fwdx_axidma_debug_info = item->fwdx_enable_item.fwdx_axidma_debug_info;
	fwdx_ctrl.fwdx_cpuio_debug_info = item->fwdx_enable_item.fwdx_cpuio_debug_info;
	PLTFM_MSG_TRACE("[FWDX][INFO] Start FWDX from Halmac.\n");
	ret = fwdx_ctrl_send(adapter, &fwdx_ctrl);
	if (ret)
		return ret;
	//Polling until received FWDX Start Ctrl Block
	while (ctrl_count1 != ctrl_count0 + 1 && time--) {
		ctrl_count1 = FWDX_CTRL_COUNT;
		PLTFM_DELAY_MS(1);
	}
	return time ? MACSUCCESS : MACPOLLTO;
}

u32 mac_fwdx_stop(struct mac_ax_adapter *adapter)
{
	struct fwdx_ctrl_containter fwdx_ctrl = { 0 };
	u32 ret = MACSUCCESS;
	u32 time = FWDX_CTRL_ACK_TIMEOUT;
	u32 ctrl_count0 = FWDX_CTRL_COUNT;
	u32 ctrl_count1 = FWDX_CTRL_COUNT;

	if (!FWDX_INIT_DONE)
		return MACPROCERR;
	PLTFM_MSG_TRACE("[FWDX][INFO] Stop FWDX from Halmac.\n");
	ret = fwdx_ctrl_send(adapter, &fwdx_ctrl);
	if (ret)
		return ret;
	//Polling until received FWDX Stop Ctrl Block
	while (ctrl_count1 != ctrl_count0 + 1 && time--) {
		ctrl_count1 = FWDX_CTRL_COUNT;
		PLTFM_DELAY_MS(1);
	}
	return time ? MACSUCCESS : MACPOLLTO;
}

u32 mac_fwdx_detect_error(struct mac_ax_adapter *adapter, enum fwdx_item_id item,
			  u32 input_len, void *input, u32 *output_len, void *output)
{
	struct fwdx_item_ctrl_block *ctrl;

	if (!FWDX_INIT_DONE)
		return MACPROCERR;
	if (item >= FWDX_ITEM_ID_MAX || item < 0) {
		PLTFM_MSG_ERR("[FWDX][ERROR] fwdx_item_id Error %d\n", item);
		return MACNOITEM;
	}
	ctrl = &FWDX_ITEM_CTRL_BLOCK_ARRAY[item];
	return ctrl->detecter(adapter, ctrl, input_len, input, output_len, output);
}
#endif //MAC_SELF_DIAG_INFO