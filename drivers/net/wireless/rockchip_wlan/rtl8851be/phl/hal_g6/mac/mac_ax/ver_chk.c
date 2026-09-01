#include "ver_chk.h"

#if MAC_FEAT_COLLOCATION
u32 chk_mac_collocation_ver(struct mac_ax_adapter *adapter)
{
#if !(defined(CONFIG_HVTOOL) || defined(HV_SERVICE_CFG) || defined(__cplusplus))
	u16 phl_depd_ver_high = PHL_MAC_DEPD_VER / 10000;
	u16 phl_depd_ver_low = PHL_MAC_DEPD_VER % 10000;
	u8 err_flag = 0;

	if (adapter->env_info.env != DUT_ENV_ASIC)
		return MACSUCCESS;
	if (phl_depd_ver_high == 19) {
		if (phl_depd_ver_low < 1)
			err_flag = 1;
	} else {
		err_flag = 1;
	}
	if (adapter->fw_info.minor_ver == 29) {
		if (adapter->fw_info.collocation_ver < 1)
			err_flag = 1;
	} else {
		err_flag = 1;
	}
	if (err_flag) {
		PLTFM_MSG_ALWAYS("[MAC][ERR] ==========Collocation Error==========\n");
		PLTFM_MSG_ALWAYS("[MAC][ERR] Collocation Ver PHL: %d, MAC: %d.%d, FW: %d.%d\n",
				 PHL_MAC_DEPD_VER,
				 MAC_AX_COLLOCATION_PROTOTYPE_VER, MAC_AX_COLLOCATION_SUB_VER,
				 adapter->fw_info.minor_ver, adapter->fw_info.collocation_ver);
		PLTFM_MSG_ALWAYS("[MAC][ERR] =====================================\n");
		return MACVERERR;
	}
	phl_depd_ver_high = 0;
	phl_depd_ver_low = 0;
#endif
	return MACSUCCESS;
}
#endif
