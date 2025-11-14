#ifndef JADARD_MODULE_H
#define JADARD_MODULE_H

#define JD_ONE_SIZE    1
#define JD_TWO_SIZE    2
#define JD_THREE_SIZE  3
#define JD_FOUR_SIZE   4
#define JD_FIVE_SIZE   5
#define JD_SIX_SIZE    6
#define JD_SEVEN_SIZE  7
#define JD_EIGHT_SIZE  8

struct jadard_common_variable {
    uint32_t FW_SIZE;
    uint32_t RAM_LEN;
    uint8_t dbi_dd_reg_mode;
};

struct jadard_support_chip {
    void (*chip_init)(void);
    bool (*chip_detect)(void);
    struct jadard_support_chip *next;
};

enum JD_SIZE_DEF {
	JD_SIZE_DEF_PAGE_SIZE	= 0x100,
	JD_SIZE_DEF_SECTOR_SIZE = 0x1000,
	JD_SIZE_DEF_BLOCK_32K	= 0x8000,
	JD_SIZE_DEF_BLOCK_64K	= 0x10000,
};

enum JD_FLASH_RUN_TIME {
	/* unit: ms */
	JD_FLASH_RUN_TIME_PAGE_WRITE_WAIT_TIME = 3,
	JD_FLASH_RUN_TIME_PAGE_WRITE_TIME	   = 50,
	JD_FLASH_RUN_TIME_ERASE_SECTOR_TIME	   = 80,
	JD_FLASH_RUN_TIME_ERASE_BLOCK_32K_TIME = 250,
	JD_FLASH_RUN_TIME_ERASE_BLOCK_64K_TIME = 400,
	JD_FLASH_RUN_TIME_ERASE_CHIP_TIME	   = 3000,
};

enum JD_EX_FLASH_ADDR {
	JD_EX_FLASH_ADDR_WP_UNLOCK				= 0x00,
	JD_EX_FLASH_ADDR_WRITE_STATUS_REGISTER1 = 0x01,
	JD_EX_FLASH_ADDR_READ					= 0x03,
	JD_EX_FLASH_ADDR_STATUS_REGISTER1		= 0x05,
	JD_EX_FLASH_ADDR_WRITE_ENABLE			= 0x06,
	JD_EX_FLASH_ADDR_WRITE_DISABLE			= 0x04,
	JD_EX_FLASH_ADDR_ERASE_SECTOR			= 0x20,
	JD_EX_FLASH_ADDR_STATUS_REGISTER2		= 0x35,
	JD_EX_FLASH_ADDR_ERASE_BLOCK_32K		= 0x52,
	JD_EX_FLASH_ADDR_ERASE_BLOCK_64K		= 0xD8,
	JD_EX_FLASH_ADDR_ERASE_CHIP				= 0xC7,
	JD_EX_FLASH_ADDR_WP_LOCK				= 0x9C,
	JD_EX_FLASH_ADDR_READ_ID				= 0x9F,
};

enum JD_DMA_RELATED_SETTING {
	JD_DMA_RELATED_SETTING_READ_FLASH_STATUS		 = 0x08,
};

enum JD_FLASH_REG_ADDR {
	JD_FLASH_BASE_ADDR							  = 0x40000200,
	JD_FLASH_REG_ADDR_DMA_FLASH_ADDR0			  = JD_FLASH_BASE_ADDR + 0x00,
	JD_FLASH_REG_ADDR_DMA_2WFLASHEN_1WRITE_0START = JD_FLASH_BASE_ADDR + 0x0C,
	JD_FLASH_REG_ADDR_DMA_0ABORT				  = JD_FLASH_BASE_ADDR + 0x0D,
	JD_FLASH_REG_ADDR_DMA_BYTE_MODE				  = JD_FLASH_BASE_ADDR + 0x0E,
	JD_FLASH_REG_ADDR_SPI_BAUD_RATE				  = JD_FLASH_BASE_ADDR + 0x11,
	JD_FLASH_REG_ADDR_SPI_CSN_OUT				  = JD_FLASH_BASE_ADDR + 0x12,
	JD_FLASH_REG_ADDR_SPI_2CPHA_1CPOL_0SPEN		  = JD_FLASH_BASE_ADDR + 0x13,
	JD_FLASH_REG_ADDR_SPI_WDATA					  = JD_FLASH_BASE_ADDR + 0x14,
	JD_FLASH_REG_ADDR_SPI_STATUS				  = JD_FLASH_BASE_ADDR + 0x15,
	JD_FLASH_REG_ADDR_SPI_RDATA					  = JD_FLASH_BASE_ADDR + 0x16,
	JD_FLASH_REG_ADDR_CRC_INIT					  = JD_FLASH_BASE_ADDR + 0x17,
	JD_FLASH_REG_ADDR_CRC_INIT_CODE0			  = JD_FLASH_BASE_ADDR + 0x18,
	JD_FLASH_REG_ADDR_CRC_CHK0					  = JD_FLASH_BASE_ADDR + 0x1A,
	JD_FLASH_REG_ADDR_CRC_ENABLE				  = JD_FLASH_BASE_ADDR + 0x1C,
	JD_FLASH_REG_ADDR_DMA_BUSY_OR_START			  = JD_FLASH_BASE_ADDR + 0x1D,
	JD_FLASH_REG_ADDR_PRAM_CKSUM_EN				  = JD_FLASH_BASE_ADDR + 0x30,
};

enum JD_MASTER_SPI_RELATED_SETTING {
	/* 0x11[2:0] */
	JD_MASTER_SPI_RELATED_SETTING_SPEED_24MHz			= 0x00,
	JD_MASTER_SPI_RELATED_SETTING_SPEED_12MHz			= 0x01,
	JD_MASTER_SPI_RELATED_SETTING_SPEED_6MHz			= 0x02,
	JD_MASTER_SPI_RELATED_SETTING_SPEED_3MHz			= 0x03,
	JD_MASTER_SPI_RELATED_SETTING_SPEED_1500KHz			= 0x04,
	JD_MASTER_SPI_RELATED_SETTING_SPEED_750KHz			= 0x05,
	JD_MASTER_SPI_RELATED_SETTING_SPEED_375KHz			= 0x06,
	/* 0x12[0] */
	JD_MASTER_SPI_RELATED_SETTING_CSN_H					= 0x00,
	JD_MASTER_SPI_RELATED_SETTING_CSN_L					= 0x01,
	/* 0x15[7] */
	JD_MASTER_SPI_RELATED_SETTING_SPIF_BUSY				= 0x00,
	JD_MASTER_SPI_RELATED_SETTING_SPIF_DONE				= 0x80,
	/* 0x1D[4] */
	JD_MASTER_SPI_RELATED_SETTING_FLASH_DONE			= 0x00,
	JD_MASTER_SPI_RELATED_SETTING_FLASH_BUSY			= 0x10,
	/* 0x30[0] */
	JD_MASTER_SPI_RELATED_SETTING_PRAM_CKSUM_EN_DISABLE = 0x00,
	JD_MASTER_SPI_RELATED_SETTING_PRAM_CKSUM_EN_ENABLE	= 0x01,
};

enum JD_CRC_INFO {
	JD_CRC_INFO_PolynomialCRC16	  = 0x8005,
	JD_CRC_INFO_CRC_INITIAL_VALUE = 0xFFFF,
};

enum JD_CRC_CODE_POSITION {
	JD_CRC_CODE_POSITION_LOW_BYTE = 0,
	JD_CRC_CODE_POSITION_HIGH_BYTE,
};

struct jadard_module_fp {
    /* Support multiple chip */
    struct jadard_support_chip *head_support_chip;

	int (*fp_register_read)(uint32_t ReadAddr, uint8_t *ReadData, uint32_t ReadLen);
	int (*fp_register_write)(uint32_t WriteAddr, uint8_t *WriteData, uint32_t WriteLen);
	uint8_t (*fp_dd_register_read)(uint8_t page, uint8_t cmd, uint8_t *rpar, uint8_t rpar_len, uint32_t offset);
	uint8_t (*fp_dd_register_write)(uint8_t page, uint8_t cmd, uint8_t *par, uint8_t par_len, uint32_t offset);
	void (*fp_set_sleep_mode)(uint8_t *value, uint8_t value_len);
	void (*fp_read_fw_ver)(void);
	void (*fp_mutual_data_set)(uint8_t data_type);
	int (*fp_get_mutual_data)(uint8_t data_type, uint8_t *rdata, uint16_t rlen);
	bool (*fp_get_touch_data)(uint8_t *buf, uint8_t length);
	int (*fp_read_mutual_data)(uint8_t *rdata, uint16_t rlen);
	void (*fp_report_points)(struct jadard_ts_data *ts);
	int (*fp_parse_report_data)(struct jadard_ts_data *ts, int irq_event, int ts_status);
	int (*fp_distribute_touch_data)(struct jadard_ts_data *ts, uint8_t *buf, int irq_event, int ts_status);
#ifndef JD_ZERO_FLASH
	int (*fp_flash_read)(uint32_t start_addr, uint8_t *rdata, uint32_t rlen);
	int (*fp_flash_write)(uint32_t start_addr, uint8_t *wdata, uint32_t wlen);
	int (*fp_flash_erase)(void);
#endif
	void (*fp_EnterBackDoor)(void);
	void (*fp_ExitBackDoor)(void);
	void (*fp_pin_reset)(uint8_t reset_type);
	void (*fp_soft_reset)(void);
	void (*fp_ResetMCU)(void);
	void (*fp_PorInit)(void);
#ifdef JD_RST_PIN_FUNC
    void (*fp_ic_reset)(bool reload_cfg, bool int_off_on);
#endif

    void (*fp_ic_soft_reset)(void);
    void (*fp_touch_info_set)(void);
    void (*fp_report_data_reinit)(void);
    void (*fp_usb_detect_set)(uint8_t *usb_status);
    uint8_t (*fp_get_freq_band)(void);
    uint8_t (*fp_ReadDbicPageEn)(void);
    int (*fp_SetDbicPage)(uint8_t page);
    void (*fp_log_touch_state)(void);
#if defined(JD_SMART_WAKEUP) || defined(JD_USB_DETECT_GLOBAL) || defined(JD_USB_DETECT_CALLBACK) ||\
    defined(JD_HIGH_SENSITIVITY) || defined(JD_ROTATE_BORDER) || defined(JD_EARPHONE_DETECT)
    void (*fp_resume_set_func)(bool suspended);
#endif
#ifdef JD_HIGH_SENSITIVITY
	void (*fp_set_high_sensitivity)(bool enable);
#endif
#ifdef JD_ROTATE_BORDER
	void (*fp_set_rotate_border)(uint16_t rotate);
#endif
#ifdef JD_EARPHONE_DETECT
	void (*fp_set_earphone_enable)(uint8_t status);
#endif
	void (*fp_set_SMWP_enable)(bool enable);
#if (JD_PROXIMITY_MODE != JD_PROXIMITY_DISABLE)
	void (*fp_set_virtual_proximity)(bool enable);
#endif
#ifdef JD_AUTO_UPGRADE_FW
    int (*fp_read_fw_ver_bin)(void);
#endif

    int (*fp_ram_read)(uint32_t start_addr, uint8_t *rdata, uint32_t rlen);
#ifdef JD_ZERO_FLASH
    int (*fp_ram_write)(uint32_t start_addr, uint8_t *wdata, uint32_t wlen);
    int (*fp_0f_upgrade_fw)(char *file_name);
#ifdef JD_ESD_CHECK
    int (*fp_0f_esd_upgrade_fw)(char *file_name);
    int (*fp_esd_ram_write)(uint32_t start_addr, uint8_t *wdata, uint32_t wlen);
#endif
    void (*fp_0f_operation)(struct work_struct *work);
#endif

#ifdef CONFIG_TOUCHSCREEN_JADARD_SORTING
	int (*fp_sorting_test)(char *json_buf, size_t len);
	void (*fp_APP_SetSortingMode)(uint8_t *value, uint8_t value_len);
	void (*fp_APP_ReadSortingMode)(uint8_t *pValue, uint8_t pValue_len);
	void (*fp_APP_GetLcdSleep)(uint8_t *pStatus);
	void (*fp_APP_SetSortingSkipFrame)(uint8_t value);
	void (*fp_APP_SetSortingKeepFrame)(uint8_t value);
	bool (*fp_APP_ReadSortingBusyStatus)(uint8_t mpap_handshake_finish, uint8_t *pStatus);
	void (*fp_GetSortingDiffData)(uint8_t *pValue, uint16_t pValue_len);
	void (*fp_GetSortingDiffDataMax)(uint8_t *pValue, uint16_t pValue_len);
	void (*fp_GetSortingDiffDataMin)(uint8_t *pValue, uint16_t pValue_len);
	void (*fp_Fw_DBIC_Off)(void);
	void (*fp_Fw_DBIC_On)(void);
	void (*fp_StartMCU)(void);
	void (*fp_SetMpBypassMain)(void);
	void (*fp_ClearMpBypassMain)(void);
	void (*fp_ReadMpapErrorMsg)(uint8_t *pValue, uint8_t pValue_len);
#endif
	bool (*fp_DoneStatusIsHigh)(uint8_t usValue, uint8_t DoneStatus);
	bool (*fp_DoneStatusIsLow)(uint8_t usValue, uint8_t BusyStatus, uint8_t DoneStatus);
#ifndef JD_ZERO_FLASH
#ifdef JD_FLASH_WP_EN
	int (*fp_FlashUnlock)(void);
	int (*fp_FlashLock)(void);
#endif
	int (*fp_SetFlashSPISpeed)(uint8_t flash_spi_speed_level);
	int (*fp_ICReadExFlashFlow)(uint8_t cmd, uint32_t addr, uint16_t addr_len,
								uint8_t dummy_par, uint16_t dummy_par_len, uint8_t *rdata, uint16_t rdata_len);
	int (*fp_ICGetExFlashStatus)(uint16_t usTimeOut);
	int (*fp_EraseSector)(uint32_t addr);
	int (*fp_EraseBlock_32K)(uint32_t addr);
	int (*fp_EraseBlock_64K)(uint32_t addr);
	int (*fp_EraseChipFlash)(uint32_t addr, uint32_t len);
	int (*fp_SetCRCEnable)(bool enable);
	uint32_t (*fp_Pow)(uint8_t base, uint32_t pow);
	int (*fp_JEDEC_ID)(uint8_t *id);
	int (*fp_CheckSize)(uint8_t data, uint32_t length);
#endif
	int (*fp_GetFlashSPIFStatus)(uint16_t usTimeOut, uint8_t *rdata);
	int (*fp_ICSetExFlashCSNOutDisable)(void);
	int (*fp_ICSetExFlashCSNOutEnable)(void);
	int (*fp_ICWriteExFlashFlow)(uint8_t cmd, uint32_t addr, uint16_t addr_len, uint8_t *wdata, uint16_t wdata_len);
	int (*fp_SetCRCInitial)(void);
	int (*fp_SetCRCInitialValue)(void);
	int (*fp_GetCRCResult)(uint16_t *crc);
	int (*fp_SetPramCksumEn)(bool enable);
};

void jadard_mcu_cmd_struct_init(void);

#endif
