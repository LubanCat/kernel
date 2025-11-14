#ifndef JADARD_COMMON_H
#define JADARD_COMMON_H

/* #include <asm/segment.h> */
#include <linux/uaccess.h>
#include <linux/atomic.h>
#include <linux/vmalloc.h>
#include <linux/version.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/async.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/input/mt.h>
#include <linux/firmware.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/buffer_head.h>
#include <linux/pm_wakeup.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#include "jadard_platform.h"

#ifdef CONFIG_OF
    #include <linux/of_gpio.h>
#endif

#ifdef JD_SPRD_EIC_MODE
#include <linux/errno.h>
#endif

#if defined(__JADARD_KMODULE__)
#include <linux/kallsyms.h>
#endif

#define JADARD_DRIVER_VER		 "01.FD" /* MAJOR_VER.MINOR_VER */
#define JADARD_DRIVER_CID_VER	 "00.01" /* MAJOR_CID_VER.MINOR_CID_VER */
#define JADARD_PROC_TOUCH_FOLDER "jadard_touch"

#define JD_TS_WAKE_LOCK_TIMEOUT  (5000)
#define JD_READ_LEN_OVERFLOW     (1)
#define JD_NO_ERR                (0)
#define JD_WRITE_OVERFLOW        (-1)
#define JD_MEM_ALLOC_FAIL        (-2)
#define JD_UPGRADE_CONFLICT      (-3)
#define JD_TIME_OUT              (-4)
#define JD_CHIP_ID_ERROR         (-5)
#define JD_APP_START_FAIL        (-6)
#define JD_CHECK_DATA_ERROR      (-7)
#define JD_FILE_OPEN_FAIL        (-8)
#define JD_INPUT_REG_FAIL        (-9)
#define JD_PRAM_CRC_PASS         (-10)
/* Report format info. */
/* #define JD_REPORT_FORMAT_V2 */
/* #define JD_SANSUMG_PALM_EN */
#if defined(JD_REPORT_FORMAT_V2)
	#if defined(JD_SANSUMG_PALM_EN)
	#define JD_PALM_EN
	#define JD_TOUCH_DATA_SIZE		  (98)
	#define JD_PALM_DATA_SIZE		  (2)
	#define JD_TOUCH_PALM_INFO_SIZE	  (JD_PALM_DATA_SIZE * 10)
	#else
	#define JD_TOUCH_DATA_SIZE		  (73)
	#endif
#else
    #if defined(JD_SANSUMG_PALM_EN)
    #define JD_PALM_EN
    #define JD_TOUCH_DATA_SIZE        (124)
    #define JD_PALM_DATA_SIZE         (2)
    #define JD_TOUCH_PALM_INFO_ADDR   (104)
    #else
    #define JD_TOUCH_DATA_SIZE        (78)
    #endif
#endif

#define JD_TOUCH_MAX_DATA_SIZE    (256)
#define JD_HID_TOUCH_DATA_SIZE    (JD_TOUCH_DATA_SIZE + 4)
#define JD_FINGER_DATA_SIZE       (5)
#define JD_TOUCH_STATE_INFO_SIZE  (5) /* Touch_State*4+Touch_Application*1 */
#define JD_TOUCH_STYLUS_SIZE	  (14)
#define JD_GEST_SUP_NUM			  (24)

#if defined(JD_REPORT_FORMAT_V2)
    #define JD_TOUCH_HEADER_SIZE      (3)
    #define JD_TOUCH_STATE_ALL_SIZE   (JD_TOUCH_STATE_INFO_SIZE + 6)
    #define JD_FINGER_FRAME_ADDR      (1)
    #define JD_TOUCH_STYLUS_INFO_ADDR (3)
#else
    #define JD_FINGER_NUM_ADDR        (0)
    #define JD_FINGER_FRAME_ADDR      (1)
    #define JD_TOUCH_EVENT_INFO_ADDR  (2)
    #define JD_TOUCH_COORD_INFO_ADDR  (3)
    #define JD_TOUCH_STATE_INFO_ADDR  (53)
    #define JD_TOUCH_STYLUS_INFO_ADDR (64)
#endif

/*
 * Thread delay time setting
 */
#define JD_UPGRADE_FW_RETRY_TIME (5)
#if defined(CONFIG_JD_DB)
#define JD_UPGRADE_DELAY_TIME	 (3000)
#else
#define JD_UPGRADE_DELAY_TIME	 (1500)
#endif
#define JD_FB_DELAY_TIME		 (15000)
#define JD_USB_DETECT_DELAY_TIME (20000)
#define JD_ESD_CHECK_DELAY_TIME	 (50000)
#define JD_RESUME_DELAY_TIME	 (150)

/*=========== Config select start ===========*/
#define JD_REPORT_CHECKSUM 
/* #define JD_REPORT_FRAME_CHECK */

/*
 *  0 : Cell phone
 *  1 : JD9366T Tablet
 *  2 : JD9366TP/JD9366TS Tablet
 */
#define JD_PRODUCT_TYPE (0)

/*
 *  Can't enable JD_AUTO_UPGRADE_FW and JD_ZERO_FLASH
 *  at the same time
 */
//#define JD_AUTO_UPGRADE_FW
//#define JD_ZERO_FLASH

/*
 * All *.bin content store in an array (Jadard_firmware.i) and write to ram
 */
/* #define JD_UPGRADE_FW_ARRAY */

/*
 * Program Jadard_ito_firmware.bin before sorting_test
 * and program Jadard_firmware.bin after sorting_test
 */
#if (JD_PRODUCT_TYPE == 1)
#define JD_UPGRADE_ITO_FW
#endif

/*
 *  USB detect function
 *  JD_USB_DETECT_CALLBACK support after Android10
 */
//#define JD_USB_DETECT_GLOBAL
//#define JD_USB_DETECT_CALLBACK

/*
 *  JD_PROTOCOL_A : Report specific format of MTK platform
 *  JD_PROTOCOL_B(undef JD_PROTOCOL_A) : input_mt_init_slots(A, B) format of QCT platform
 *  JD_PROTOCOL_B_3PA : input_mt_init_slots(A, B, C) format of QCT platform
 */
/* #define JD_PROTOCOL_A */
#define JD_PROTOCOL_B_3PA
/*
 * Report Pressure in multitouch
 * 1:enable(default), 0:disable
*/
#define JD_REPORT_PRESSURE (1)

/* Other config */
#define JD_RST_PIN_FUNC
/* #define JD_SMART_WAKEUP */
/* #define JD_SYS_CLASS_SMWP_EN */
/* #define JD_HIGH_SENSITIVITY */
/* #define JD_ROTATE_BORDER */
/* #define JD_EARPHONE_DETECT */
/* #define JD_PEN_BATTERY */
/* #define JD_PALM_EN */
#if defined(JD_PALM_EN)
#define JD_BTN_PALM  (0x118)
#endif

/*
 * Proximity mode options
 * Disable: JD_PROXIMITY_DISABLE
 * Enable:  JD_PROXIMITY_SUSPEND_RESUME when sleep in/out
 *          JD_PROXIMITY_BACKLIGHT when backlight on/off
 */
#define JD_PROXIMITY_DISABLE        0 /* Don`t modify */
#define JD_PROXIMITY_SUSPEND_RESUME 1 /* Don`t modify */
#define JD_PROXIMITY_BACKLIGHT      2 /* Don`t modify */
#define JD_PROXIMITY_MODE          (JD_PROXIMITY_DISABLE)
/* #define JD_SYS_CLASS_PSENSOR_EN */

/* #define CONFIG_CHIP_DTCFG */

/*
 *  Customer/platform config
 */
/* #define JD_OPPO_FUNC */

/*
 *  Suspend/Resume function
 *
 * Frame Buffer (FB): JD_CONFIG_FB
 * Direct Rendering Manager (DRM): JD_CONFIG_FB + JD_CONFIG_DRM
 * Qualcomm MSM DRM: JD_CONFIG_FB + JD_CONFIG_DRM_MSM
 * MTK Platform DRM V2: JD_CONFIG_DRM_MTK_V2
 * SPRD Platform DRM: JD_CONFIG_SPRD_DRM
 * System Node: JD_CONFIG_NODE
 * Notice: Can`t enable JD_CONFIG_SPRD_DRM and JD_RESUME_NOT_WAIT_FW at the same time
 */
/* #define JD_RESUME_NOT_WAIT_FW */
/* #define JD_CONFIG_FB */
/* #define JD_CONFIG_DRM */
/* #define JD_CONFIG_DRM_V2 */
/* #define JD_CONFIG_DRM_MSM */
//#define JD_CONFIG_DRM_MTK_V2
/* #define JD_CONFIG_SPRD_DRM */
/* #define JD_CONFIG_NODE */

/*
 *	For Zero-Flash only
 *	Reduce the time of upgrade FW when resume and IOVCC is not drop
 */
//#define JD_RESUME_TIME_REDUCE

/*
 *	Enable I2C single mode for with flash only
 */
/* #define JD_I2C_SINGLE_MODE */

/*
 *	Enable esd check for Zero-Flash only
 */
/* #define JD_ESD_CHECK */
/*
 *	Parse the jadard_sorting_threshold.h file and set sorting threshold
 */
/* #define JD_SORTING_THRES_BUILT_IN */

/*
 * Enable Spreadtrum EIC (external interrupt controller) mode for Spreadtrum platform only
 * It can be used only in input mode
 * If enable, must add PINCTRL configuration in dts file
 */
/* #define JD_SPRD_EIC_MODE */

/*
 *	Enable each workarounds for FPGA only
 */
/* #define JD_FPGA_WORKAROUND */
/*=========== Config select end =============*/

#if defined(JD_CONFIG_FB)
#ifdef JD_CONFIG_DRM_V2
	#include <linux/soc/qcom/panel_event_notifier.h>
#else
	#include <linux/notifier.h>
	#include <linux/fb.h>
#if defined(JD_CONFIG_DRM)
	#include <drm/drm_panel.h>
#endif
#if defined(JD_CONFIG_DRM_MSM)
	#include <linux/msm_drm_notify.h>
#endif
#endif
#endif

#if defined(JD_CONFIG_SPRD_DRM)
    #include <linux/sprd_drm_notifier.h>
#endif

#if defined(JD_CONFIG_DRM_MTK_V2)
    #include "mtk_disp_notify.h"
#endif

#if defined(JD_USB_DETECT_CALLBACK)
    #include <linux/power_supply.h>
#endif

#if defined(CONFIG_JD_HID)
    #include "jadard_hid.h"
#endif

enum JD_DATA_TYPE {
    JD_DATA_TYPE_RawData = 1,
    JD_DATA_TYPE_Baseline,
    JD_DATA_TYPE_Difference,
    JD_DATA_TYPE_LISTEN,
    JD_DATA_TYPE_LABEL,
    JD_DATA_TYPE_LAPLACE,
    JD_DATA_TYPE_RESERVE_7,
    JD_DATA_TYPE_RESERVE_8,
    JD_DATA_TYPE_RESERVE_9,
    JD_DATA_TYPE_RESERVE_A,
    JD_DATA_TYPE_RESERVE_B,
    JD_DATA_TYPE_RESERVE_C,
    JD_DATA_TYPE_RESERVE_D,
    JD_DATA_TYPE_RESERVE_E,
    JD_DATA_TYPE_RESERVE_F
};

enum JD_KEEP_TYPE {
    JD_KEEP_TYPE_NormalValue = 1,
    JD_KEEP_TYPE_MaxValue,
    JD_KEEP_TYPE_MinValue,
    JD_KEEP_TYPE_PeakValue
};

enum JD_IRQ_EVENT {
    JD_REPORT_COORD = 1,
    JD_REPORT_SMWP_EVENT,
};

enum JD_TS_STATUS {
    JD_TS_UNUSUAL_DATA_FAIL = -4,
    JD_TS_CHECKSUM_FAIL = -3,
    JD_TS_GET_DATA_FAIL = -2,
    JD_IRQ_EVENT_FAIL = -1,
    JD_TS_NORMAL_START = 0,
    JD_REPORT_DATA,
    JD_RST_OK,
};

enum JD_DDREG_MODE {
    JD_DDREG_MODE_0 = 0x00,
    JD_DDREG_MODE_1,
};

enum JD_DBIC_REG_ADDR {
    JD_DBIC_BASE_ADDR  = 0x400001,
    JD_DBIC_CMD        = (JD_DBIC_BASE_ADDR << 8) + 0x40,
    JD_DBIC_WRITE_DATA = (JD_DBIC_BASE_ADDR << 8) + 0x41,
    JD_DBIC_READ_DATA  = (JD_DBIC_BASE_ADDR << 8) + 0x42,
    JD_DBIC_DUMMY1     = (JD_DBIC_BASE_ADDR << 8) + 0x43,
    JD_DBIC_INT_EN     = (JD_DBIC_BASE_ADDR << 8) + 0x44,
    JD_DBIC_INT_CLR    = (JD_DBIC_BASE_ADDR << 8) + 0x45,
    JD_DBIC_SPI_RUN    = (JD_DBIC_BASE_ADDR << 8) + 0x46,
    JD_DBIC_CMD_LEN    = (JD_DBIC_BASE_ADDR << 8) + 0x47,
    JD_DBIC_SPI_FREQ   = (JD_DBIC_BASE_ADDR << 8) + 0x48,
    JD_DBIC_STATUS     = (JD_DBIC_BASE_ADDR << 8) + 0x49,
};

enum JD_DBIC_RELATED_SETTING {
    JD_DBIC_READ_WRITE_SUCCESS  = 0x00,
    JD_DBIC_READ_WRITE_FAIL     = 0x01,
    JD_DBIC_IRQ_VALID_MSK       = 0x03,
    JD_DBIC_WRITE_RUN           = 0x01,
    JD_DBIC_READ_RUN            = 0x03,
    JD_DBIC_INT_CLR_BUSY_MSK    = 0x40,
    JD_DBIC_STATUS_BUSY_MSK     = 0x40,
    JD_DBIC_STATUS_RDY_MSK      = 0x80,
    JD_DBIC_STATUS_RDY_BUSY_MSK = 0xC0,
    JD_DBIC_READ_SET_CMD        = 0xF1,
    JD_DBIC_READ_DATA_CMD       = 0xF2,
};

enum JD_DBI_DDREG_ADDR {
    JD_DBI_DDREG_BASE_ADDR           = 0x50000000,
    JD_DBI_DDREG_STD_CMD_BASE_ADDR   = 0x50800000,
    JD_S_DBI_DDREG_BASE_ADDR         = 0x51000000,
    JD_S_DBI_DDREG_STD_CMD_BASE_ADDR = 0x51800000,
};

struct jadard_ts_data {
    bool suspended;
    bool rawdata_little_endian;
    atomic_t suspend_mode;
    uint8_t protocol_type;
    uint8_t irq_enabled;
    uint8_t debug_log_touch_level;
    bool debug_fw_package_enable;
    bool debug_diag_apk_enable;
    bool dbi_std_mode_enable;
    int rst_gpio;
    int jd_irq;
    bool rst_active;
    bool diag_thread_active;

	struct device *dev;
	struct input_dev *input_dev;
	struct input_dev *stylus_dev;
	struct i2c_client *client;
	struct jadard_platform_data *pdata;
	struct mutex rw_lock;
	spinlock_t irq_active;
#ifdef CONFIG_TOUCHSCREEN_JADARD_SORTING
	struct mutex sorting_active;
	bool ito_sorting_active;
#endif

/******* SPI-start *******/
	struct mutex spi_lock;
	struct spi_device *spi;
/******* SPI-end *******/

#if defined(JD_CONFIG_FB)
#if !defined(JD_CONFIG_DRM_V2)
	struct notifier_block fb_notif;
#endif
	struct workqueue_struct *jadard_fb_wq;
	struct delayed_work work_fb;
#if defined(JD_CONFIG_DRM) || defined(JD_CONFIG_DRM_V2)
	struct drm_panel *active_panel;
#if defined(JD_CONFIG_DRM_V2)
	void *cookie;
#endif
#endif
#endif

    struct workqueue_struct *fw_dump_wq;
    struct work_struct fw_dump_work;

#ifdef JD_AUTO_UPGRADE_FW
    struct workqueue_struct *jadard_upgrade_wq;
    struct delayed_work work_upgrade;
#endif

    bool fw_ready;
#ifdef JD_ZERO_FLASH
    struct workqueue_struct *jadard_0f_upgrade_wq;
    struct delayed_work work_0f_upgrade;
    bool power_on_upgrade;
#endif

#ifdef JD_ESD_CHECK
#ifdef JD_ZERO_FLASH
	const struct firmware *esd_fw;
	bool esd_fw_reload;
#endif
	bool esd_check_running;
	struct workqueue_struct *jadard_esd_check_wq;
	struct delayed_work work_esd_check;
#endif

#ifdef JD_FW_ESD_EVENT
	uint8_t esd_detect;
	uint8_t pre_esd_detect;
#ifdef JD_FW_ESD_EVENT_CRC_CHECK
	bool upgrade_request;
	uint16_t checksum_error_count;
#endif
#endif

    struct workqueue_struct *jadard_diag_wq;
    struct delayed_work jadard_diag_delay_wrok;

#ifdef JD_SMART_WAKEUP
    bool SMWP_enable;
    bool gesture_cust_en[JD_GEST_SUP_NUM];
    struct wakeup_source *ts_SMWP_wake_lock;
#endif

#if defined(JD_USB_DETECT_CALLBACK) || defined(JD_USB_DETECT_GLOBAL)
    uint8_t usb_connected;
    uint8_t *usb_status;
#if defined(JD_USB_DETECT_GLOBAL)
    int update_usb_status;
#endif
#if defined(JD_USB_DETECT_CALLBACK)
    struct notifier_block charger_notif;
    struct workqueue_struct *jadard_usb_detect_wq;
    struct delayed_work work_usb_detect;
#endif
#endif

#ifdef JD_HIGH_SENSITIVITY
    bool high_sensitivity_enable;
#endif

#ifdef JD_ROTATE_BORDER
    int rotate_switch_mode;
    uint16_t rotate_border;
#endif

#ifdef JD_EARPHONE_DETECT
    /*     High byte      Low byte
     * ------------------------------
     *| Type-c status | 3.5mm status |
     * ------------------------------
     */
    uint16_t earphone_enable;
#endif

#if (JD_PROXIMITY_MODE != JD_PROXIMITY_DISABLE)
    uint8_t proximity_enable;
    uint8_t proximity_detect;
    uint8_t pre_proximity_detect;
#endif

#ifdef JD_PALM_EN
	uint8_t palm_detect;
	uint8_t pre_palm_detect;
#endif

#ifdef JD_RESUME_NOT_WAIT_FW
	/* Using jadard_resume_wq the same name was Fool-proof mechanism
	 * when define JD_CONFIG_SPRD_DRM and JD_RESUME_NOT_WAIT_FW at the same time
	 */
	struct workqueue_struct *jadard_resume_wq;
	struct delayed_work jadard_resume_work;
#endif

#if defined(JD_CONFIG_SPRD_DRM)
    struct notifier_block drm_notify;
    struct workqueue_struct *jadard_resume_wq;
    struct work_struct jadard_resume_work;
#endif

#if defined(JD_CONFIG_DRM_MTK_V2)
    struct notifier_block disp_notifier;
#endif

#ifdef CONFIG_JD_HID
    struct workqueue_struct *jadard_hid_mode_wq;
    struct delayed_work work_hid_mode;
#endif
};

struct jadard_ic_data {
	char	 chip_id[15];
	uint32_t fw_ver;
	uint32_t fw_cid_ver;
	char	 panel_maker[10];
	uint8_t	 panel_ver;
	int		 JD_X_NUM;
	int		 JD_Y_NUM;
	int		 JD_X_RES;
	int		 JD_Y_RES;
	int		 JD_MAX_PT;
	bool	 JD_INT_EDGE;
	bool	 JD_STYLUS_EN;
	uint8_t	 JD_STYLUS_ID_EN;
	uint8_t	 JD_STYLUS_RATIO;
	uint8_t	 JD_MODULE_CASCADE_MODE;
	uint32_t JD_MASTER_ADDR_OFFSET;
	uint32_t JD_SECOND_ADDR_OFFSET;
	uint32_t JD_BOTH_ADDR_OFFSET;
	bool	 JD_DISABLE_MASTER_TO_SECOND;
};

enum jadard_input_protocol_type {
    PROTOCOL_TYPE_A = 0,
    PROTOCOL_TYPE_B,
};

struct jadard_report_data {
    int touch_data_size;
    int touch_coord_size;
    uint8_t *touch_coord_info;
#ifdef JD_SANSUMG_PALM_EN
    int touch_palm_size;
    uint8_t *touch_palm_info;
#endif
    uint8_t touch_state_info[JD_TOUCH_STATE_INFO_SIZE];
    uint8_t touch_stylus_info[JD_TOUCH_STYLUS_SIZE];
#if defined(JD_SMART_WAKEUP)
    uint8_t touch_event_info;
#endif
#if defined(CONFIG_TOUCHSCREEN_JADARD_DEBUG)
    uint32_t report_rate_count;
    uint8_t touch_all_info[JD_HID_TOUCH_DATA_SIZE];
#endif
#if defined(JD_ESD_CHECK)
    uint8_t crc_retry;
    uint8_t crc_start;
#endif
};

struct jadard_stylus_host_data {
#ifdef JD_REPORT_FRAME_CHECK
    uint8_t pen_frame;
    uint8_t pre_pen_frame;
#endif
	uint64_t pen_id;
	int x;
	int y;
	int w;
	int event;
	int tilt_x;
	int tilt_y;
	uint32_t hover;
	uint32_t tail;
	uint32_t btn;
	uint32_t btn2;
	uint32_t pre_tail;
	uint32_t pre_btn;
	uint32_t pre_btn2;
	uint8_t battery_info;
};

struct jadard_host_data {
#ifdef JD_REPORT_FRAME_CHECK
    uint8_t finger_frame;
    uint8_t pre_finger_frame;
#endif
    uint8_t finger_num;
    uint8_t *id;
    int *x;
    int *y;
    int *w;
#ifdef JD_SANSUMG_PALM_EN
    int *maj;
    int *min;
#endif
    int *event;
#ifdef JD_SMART_WAKEUP
    int SMWP_event_chk;
#endif
    struct jadard_stylus_host_data stylus;
};

enum jadard_host_data_event{
    JD_DOWN_EVENT = 0,
    JD_UP_EVENT,
    JD_CONTACT_EVENT,
    JD_NO_EVENT
};

enum jadard_finger_event{
    JD_ATTN_IN = 1,
    JD_ATTN_OUT,
};

struct jadard_debug {
    bool proc_send_flag;
    bool *fw_dump_going;
    void (*fp_touch_dbg_func)(struct jadard_ts_data *ts, uint8_t start);
};

enum jadard_panel_maker{
    JD_PANEL_MAKER_NUM  = 0x12,
    JD_PANEL_MAKER_LIST_SIZE = JD_PANEL_MAKER_NUM + 1,
    JD_PANEL_MAKER_CP   = 0xFE,
    JD_PANEL_MAKER_COB  = 0xFF,
};

enum JD_LCD_STATUS {
    JD_LCD_STATUS_SLEEP_IN = 0,
    JD_LCD_STATUS_SLEEP_OUT,
    JD_LCD_STATUS_DISPLAY_OFF,
    JD_LCD_STATUS_DISPLAY_ON,
    JD_LCD_STATUS_ERROR
};

enum jadard_cascade_mode {
    JD_CASCADE_MODE_DISABLE = 0,
    JD_CASCADE_MODE_ENABLE = 1,
};

int jadard_chip_common_init(void);
void jadard_chip_common_deinit(void);
void jadard_common_proc_deinit(void);
void jadard_ts_work(struct jadard_ts_data *ts);
uint8_t jadard_DbicWriteDDReg(uint8_t cmd, uint8_t *rpar, uint8_t rpar_len);
uint8_t jadard_DbicReadDDReg(uint8_t cmd, uint8_t *rpar, uint8_t rpar_len);
uint8_t jadard_DbiWriteDDReg(uint8_t page, uint8_t cmd, uint8_t *par, uint8_t par_len, uint32_t offset);
uint8_t jadard_DbiReadDDReg(uint8_t page, uint8_t cmd, uint8_t *rpar, uint8_t rpar_len, uint32_t offset);
int jadard_report_data_init(void);
void jadard_report_points(struct jadard_ts_data *ts);
int jadard_distribute_touch_data(struct jadard_ts_data *ts, uint8_t *buf, int irq_event, int ts_status);
int jadard_parse_report_data(struct jadard_ts_data *ts, int irq_event, int ts_status);
void jadard_log_touch_state(const char **bit_map);
int jadard_input_register(struct jadard_ts_data *ts);

#if defined(JD_USB_DETECT_GLOBAL)
void jadard_cable_detect(bool renew);
#endif

#if defined(JD_USB_DETECT_CALLBACK)
void jadard_usb_status(int status, bool renew);
#endif

extern int jadard_chip_common_suspend(struct jadard_ts_data *ts);
extern int jadard_chip_common_resume(struct jadard_ts_data *ts);
extern int jadard_parse_dt(struct jadard_ts_data *ts,
						struct jadard_platform_data *pdata);
extern int jadard_dev_set(struct jadard_ts_data *ts);
extern int jadard_input_register_device(struct input_dev *input_dev);

#if defined(JD_CONFIG_DRM) || defined(JD_CONFIG_DRM_V2)
extern int jadard_drm_check_dt(struct jadard_ts_data *ts);
#endif

#ifdef CONFIG_TOUCHSCREEN_JADARD_DEBUG
extern int jadard_debug_init(void);
extern int jadard_debug_remove(void);
#endif

#if defined(__JADARD_KMODULE__)
extern int jadard_common_init(void);
extern void jadard_common_exit(void);
#endif

#ifdef CONFIG_JD_HID
extern int jadard_spi_hid_init(void);
extern void jadard_spi_hid_exit(void);
void jadard_spi_hid_work(struct jadard_ts_data *ts);
#endif

#endif
