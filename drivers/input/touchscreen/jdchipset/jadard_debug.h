#ifndef H_JADARD_DEBUG
#define H_JADARD_DEBUG

#define JADARD_PROC_REPORT_DEBUG_FILE "report_debug"
#define JADARD_PROC_FW_PACKAGE_FILE   "fw_package"
#define JADARD_PROC_RESET_FILE        "reset"
#define JADARD_PROC_ATTN_FILE         "attn"
#define JADARD_PROC_INT_EN_FILE       "int_en"
#define JADARD_PROC_DIAG_FILE         "diag"
#define JADARD_PROC_DIAG_ARR_FILE     "diag_arr"
#define JADARD_PROC_DIAG_APK_FILE     "diag_apk"
#define JADARD_PROC_FW_DUMP_FILE      "fw_dump"
#define JADARD_PROC_REGISTER_FILE     "register"
#define JADARD_PROC_DISPLAY_FILE      "display"
#define JADARD_PROC_DISPLAY_STD_FILE  "display_std"
#define JADARD_PROC_DEBUG_FILE        "debug"
#define JADARD_PROC_BUF_RD_FILE       "buf_rd"

static struct proc_dir_entry *jadard_proc_report_debug_file;
static struct proc_dir_entry *jadard_proc_fw_package_file;
static struct proc_dir_entry *jadard_proc_reset_file;
static struct proc_dir_entry *jadard_proc_attn_file;
static struct proc_dir_entry *jadard_proc_int_en_file;
static struct proc_dir_entry *jadard_proc_diag_file;
static struct proc_dir_entry *jadard_proc_diag_arr_file;
static struct proc_dir_entry *jadard_proc_diag_apk_file;
static struct proc_dir_entry *jadard_proc_fw_dump_file;
static struct proc_dir_entry *jadard_proc_register_file;
static struct proc_dir_entry *jadard_proc_display_file;
static struct proc_dir_entry *jadard_proc_display_std_file;
static struct proc_dir_entry *jadard_proc_debug_file;
static struct proc_dir_entry *jadard_proc_buf_rd_file;

#define JD_MASTER_FW_DUMP_FILE "/sdcard/JD_MASTER_FW_Dump.bin"
#define JD_SECOND_FW_DUMP_FILE "/sdcard/JD_SECOND_FW_Dump.bin"
#define JD_RAWDATA_DUMP_FILE   "/sdcard/JD_RAWDATA_Dump.txt"
#define JD_DIFF_DUMP_FILE	   "/sdcard/JD_DIFF_Dump.txt"
#define JD_BASE_DUMP_FILE	   "/sdcard/JD_BASE_Dump.txt"
#define JD_LISTEN_DUMP_FILE	   "/sdcard/JD_LISTEN_Dump.txt"
#define JD_LABEL_DUMP_FILE	   "/sdcard/JD_LABEL_Dump.txt"
#define JD_LAPLACE_DUMP_FILE   "/sdcard/JD_LAPLACE_Dump.txt"

struct jadard_diag_mutual_data {
    char *buf;
    int  buf_len;
    int  write_len;
};

int DataType;
static int KeepType;
int KeepFrame;

#ifdef JD_RAWDATA_V2
static char ApkOrManual;
#endif

struct file *jd_diag_mutual_fn;
int				   *jd_diag_mutual;
int					jd_diag_mutual_cnt;
uint8_t			   *jd_buf;
static uint8_t	   diag_arr_num;
static uint8_t	   reg_cmd[4];
static uint8_t	   reg_cmd_len;
static uint16_t	   reg_read_len;
static uint8_t	   dd_reg_m_s_b;
static uint8_t	   dd_reg_page;
static uint8_t	   dd_reg_cmd;
static uint8_t	   dd_reg_read_len;

static uint8_t *fw_buffer;
static bool    fw_dump_busy;
static bool    fw_dump_complete;
static bool    fw_dump_going;

static char debug_cmd;
static bool fw_upgrade_complete;
bool jd_g_dbg_enable;
static bool        jd_g_buf_rd_enable;
static uint8_t     buf_rd_byte_num;
int jadard_touch_proc_init(void);
void jadard_touch_proc_deinit(void);

#endif
