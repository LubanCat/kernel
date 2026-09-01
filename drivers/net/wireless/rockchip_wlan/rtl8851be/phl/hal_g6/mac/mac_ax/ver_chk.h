#include "../pltfm_cfg.h"
#include "../errors.h"
#include "../mac_def.h"
#include "../mac_ax.h"
#include "../../hal_headers_le.h"

#if MAC_FEAT_COLLOCATION
u32 chk_mac_collocation_ver(struct mac_ax_adapter *adapter);
#endif