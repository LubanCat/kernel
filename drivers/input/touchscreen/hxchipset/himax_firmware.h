#ifndef __HIMAX_FIRMWARE_HEADER__
#define __HIMAX_FIRMWARE_HEADER__

extern const unsigned char *gHimax_firmware_list[];

extern const unsigned int gHimax_firmware_id[];

extern const unsigned int gHimax_firmware_size[];

enum firmware_type {
	HX_FWTYPE_NORMAL = 0,
	HX_FWTYPE_MP
};

extern const enum firmware_type gHimax_firmware_type[];

#define gTotal_N_firmware 1
#endif
