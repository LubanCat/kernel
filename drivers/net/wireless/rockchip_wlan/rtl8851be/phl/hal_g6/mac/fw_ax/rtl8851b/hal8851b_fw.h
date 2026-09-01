/******************************************************************************
 *
 * Copyright(c) 2012 - 2020 Realtek Corporation.
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
#ifdef PHL_FEATURE_NIC

#if defined(MAC_8851B_SUPPORT)
#ifdef MAC_FW_8851B_U1
#ifdef MAC_FW_CATEGORY_NICCE
extern u32 array_length_8851b_u1_nicce;
extern u8 array_8851b_u1_nicce[335784];
#endif /* MAC_FW_CATEGORY_NICCE */

#ifdef MAC_FW_CATEGORY_NIC_PLE
extern u32 array_length_8851b_u1_nic_ple;
extern u8 array_8851b_u1_nic_ple[272696];
#endif /* MAC_FW_CATEGORY_NIC_PLE */

#ifdef MAC_FW_CATEGORY_NIC
extern u32 array_length_8851b_u1_nic;
extern u8 array_8851b_u1_nic[270784];
#endif /* MAC_FW_CATEGORY_NIC */

#ifdef MAC_FW_CATEGORY_WOWLAN_PLE
extern u32 array_length_8851b_u1_wowlan_ple;
extern u8 array_8851b_u1_wowlan_ple[272360];
#endif /* MAC_FW_CATEGORY_WOWLAN_PLE */

#ifdef MAC_FW_CATEGORY_WOWLAN
extern u32 array_length_8851b_u1_wowlan;
extern u8 array_8851b_u1_wowlan[269352];
#endif /* MAC_FW_CATEGORY_WOWLAN */

#endif /* MAC_FW_8851B_U1 */
#ifdef MAC_FW_8851B_U2
#ifdef MAC_FW_CATEGORY_NICCE
extern u32 array_length_8851b_u2_nicce;
extern u8 array_8851b_u2_nicce[335832];
#endif /* MAC_FW_CATEGORY_NICCE */

#ifdef MAC_FW_CATEGORY_NIC_PLE
extern u32 array_length_8851b_u2_nic_ple;
extern u8 array_8851b_u2_nic_ple[272752];
#endif /* MAC_FW_CATEGORY_NIC_PLE */

#ifdef MAC_FW_CATEGORY_NIC
extern u32 array_length_8851b_u2_nic;
extern u8 array_8851b_u2_nic[270824];
#endif /* MAC_FW_CATEGORY_NIC */

#ifdef MAC_FW_CATEGORY_WOWLAN_PLE
extern u32 array_length_8851b_u2_wowlan_ple;
extern u8 array_8851b_u2_wowlan_ple[272440];
#endif /* MAC_FW_CATEGORY_WOWLAN_PLE */

#ifdef MAC_FW_CATEGORY_WOWLAN
extern u32 array_length_8851b_u2_wowlan;
extern u8 array_8851b_u2_wowlan[269448];
#endif /* MAC_FW_CATEGORY_WOWLAN */

#endif /* MAC_FW_8851B_U2 */
#endif /* #if MAC_XXXX_SUPPORT */
#endif /* PHL_FEATURE_NIC */
