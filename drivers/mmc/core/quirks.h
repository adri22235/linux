/* SPDX-License-Identifier: GPL-2.0 */
/*
 * mmc_core_quirks.h - workarounds for known hardware bugs in SD/MMC and SDIO
 */

#ifndef LINUX_MMC_CORE_QUIRKS_H
#define LINUX_MMC_CORE_QUIRKS_H

#include <linux/mmc/card.h>

#define CID_MANFID_SANDISK	0x2
#define CID_MANFID_TOSHIBA	0x11
#define CID_MANFID_KINGSTON	0x70

#define SDIO_VENDOR_ID_STE	0x0020
#define SDIO_DEVICE_ID_STE_CW1200	0x2280

#define SDIO_VENDOR_ID_MARVELL	0x02df
#define SDIO_DEVICE_ID_MARVELL_LIBERTAS	0x9103
#define SDIO_DEVICE_ID_MARVELL_8688_WLAN	0x9104

struct mmc_fixup {
	const char *name;
	unsigned int manfid;
	unsigned int oemid;
	unsigned short rev_start;
	unsigned short rev_end;
	unsigned int year;
	void (*vendor_fixup)(struct mmc_card *card, int data);
	int data;
};

#define MMC_FIXUP_REV(_name, _manfid, _oemid, _rev_start, _rev_end, _fixup, _data) \
	{						\
		.name = (_name),			\
		.manfid = (_manfid),			\
		.oemid = (_oemid),			\
		.rev_start = (_rev_start),		\
		.rev_end = (_rev_end),			\
		.vendor_fixup = (_fixup),		\
		.data = (_data),			\
	}

#define MMC_FIXUP(_name, _manfid, _oemid, _fixup, _data) \
	MMC_FIXUP_REV(_name, _manfid, _oemid, 0, 0xffff, _fixup, _data)

#define SDIO_FIXUP(_vendor, _device, _fixup, _data)	\
	{						\
		.manfid = (_vendor),			\
		.oemid = (_device),			\
		.rev_start = 0,				\
		.rev_end = 0xffff,			\
		.vendor_fixup = (_fixup),		\
		.data = (_data),			\
	}

#define _FIXUP_EXT(_name, _manfid, _oemid, _rev_start, _rev_end, _fixup, _data, _year) \
	{						\
		.name = (_name),			\
		.manfid = (_manfid),			\
		.oemid = (_oemid),			\
		.rev_start = (_rev_start),		\
		.rev_end = (_rev_end),			\
		.vendor_fixup = (_fixup),		\
		.data = (_data),			\
		.year = (_year),			\
	}

#define MMC_FIXUP_YEAR(_name, _manfid, _oemid, _year, _fixup, _data) \
	_FIXUP_EXT(_name, _manfid, _oemid, 0, 0xffff, _fixup, _data, _year)

static const struct mmc_fixup mmc_ext_fixups[] = {
	/* Kingston cards with specific CID redefinitions */
	_FIXUP_EXT(\"S70632\", CID_MANFID_KINGSTON, 0x4442, 0, 0,
		   NULL, 0, 2024),

	{ /* sentinel */ }
};

#endif /* LINUX_MMC_CORE_QUIRKS_H */
