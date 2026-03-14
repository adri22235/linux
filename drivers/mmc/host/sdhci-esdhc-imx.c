// SPDX-License-Identifier: GPL-2.0
/*
 * Freescale eSDHC i.MX controller driver for the platform bus.
 *
 * derived from the OF-version.
 *
 * Copyright (c) 2010 Pengutronix e.K.
 *   Author: Wolfram Sang <kernel@pengutronix.de>
 */

#include <linux/bitfield.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/pm_qos.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/slot-gpio.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pm_runtime.h>
#include "sdhci-cqhci.h"
#include "sdhci-pltfm.h"
#include "sdhci-esdhc.h"
#include "cqhci.h"

#define ESDHC_SYS_CTRL_DTOCV_MASK	GENMASK(19, 16)
#define ESDHC_SYS_CTRL_RST_FIFO		BIT(22)
#define ESDHC_SYS_CTRL_IPP_RST_N	BIT(23)
#define ESDHC_SYS_CTRL_RESET_TUNING	BIT(28)
#define	ESDHC_CTRL_D3CD			0x08
#define ESDHC_BURST_LEN_EN_INCR		(1 << 27)
/* VENDOR SPEC register */
#define ESDHC_VENDOR_SPEC		0xc0
#define  ESDHC_VENDOR_SPEC_SDIO_QUIRK	(1 << 1)
#define  ESDHC_VENDOR_SPEC_VSELECT	(1 << 1)
#define  ESDHC_VENDOR_SPEC_FRC_SDCLK_ON	(1 << 8)
#define ESDHC_DEBUG_SEL_AND_STATUS_REG		0xc2
#define ESDHC_DEBUG_SEL_REG			0xc3
#define ESDHC_DEBUG_SEL_MASK			0xf
#define ESDHC_DEBUG_SEL_CMD_STATE		1
#define ESDHC_DEBUG_SEL_DATA_STATE		2
#define ESDHC_DEBUG_SEL_TRANS_STATE		3
#define ESDHC_DEBUG_SEL_DMA_STATE		4
#define ESDHC_DEBUG_SEL_ADMA_STATE		5
#define ESDHC_DEBUG_SEL_FIFO_STATE		6
#define ESDHC_DEBUG_SEL_ASYNC_FIFO_STATE	7
#define ESDHC_WTMK_LVL			0x44
#define  ESDHC_WTMK_DEFAULT_VAL		0x10401040
#define  ESDHC_WTMK_LVL_RD_WML_MASK	0x000000FF
#define  ESDHC_WTMK_LVL_RD_WML_SHIFT	0
#define  ESDHC_WTMK_LVL_WR_WML_MASK	0x00FF0000
#define  ESDHC_WTMK_LVL_WR_WML_SHIFT	16
#define  ESDHC_WTMK_LVL_WML_VAL_DEF	64
#define  ESDHC_WTMK_LVL_WML_VAL_MAX	128
#define ESDHC_MIX_CTRL			0x48
#define  ESDHC_MIX_CTRL_DDREN		(1 << 3)
#define  ESDHC_MIX_CTRL_AC23EN		(1 << 7)
#define  ESDHC_MIX_CTRL_EXE_TUNE	(1 << 22)
#define  ESDHC_MIX_CTRL_SMPCLK_SEL	(1 << 23)
#define  ESDHC_MIX_CTRL_AUTO_TUNE_EN	(1 << 24)
#define  ESDHC_MIX_CTRL_FBCLK_SEL	(1 << 25)
#define  ESDHC_MIX_CTRL_HS400_EN	(1 << 26)
#define  ESDHC_MIX_CTRL_HS400_ES_EN	(1 << 27)
/* Bits 3 and 6 are not SDHCI standard definitions */
#define  ESDHC_MIX_CTRL_SDHCI_MASK	0xb7
/* Tuning bits */
#define  ESDHC_MIX_CTRL_TUNING_MASK	0x03c00000

/* dll control register */
#define ESDHC_DLL_CTRL			0x60
#define ESDHC_DLL_OVERRIDE_VAL_SHIFT	9
#define ESDHC_DLL_OVERRIDE_EN_SHIFT	8

/* tune control register */
#define ESDHC_TUNE_CTRL_STATUS		0x68
#define  ESDHC_TUNE_CTRL_STEP		1
#define  ESDHC_TUNE_CTRL_MIN		0
#define  ESDHC_TUNE_CTRL_MAX		((1 << 7) - 1)
#define  ESDHC_TUNE_CTRL_STATUS_TAP_SEL_MASK		GENMASK(30, 16)
#define  ESDHC_TUNE_CTRL_STATUS_TAP_SEL_PRE_MASK	GENMASK(30, 24)
#define  ESDHC_TUNE_CTRL_STATUS_DLY_CELL_SET_PRE_MASK	GENMASK(14, 8)
#define  ESDHC_TUNE_CTRL_STATUS_DLY_CELL_SET_OUT_MASK	GENMASK(7, 4)
#define  ESDHC_TUNE_CTRL_STATUS_DLY_CELL_SET_POST_MASK	GENMASK(3, 0)
/* strobe dll register */
#define ESDHC_STROBE_DLL_CTRL		0x70
#define ESDHC_STROBE_DLL_CTRL_ENABLE	(1 << 0)
#define ESDHC_STROBE_DLL_CTRL_RESET	(1 << 1)
#define ESDHC_STROBE_DLL_CTRL_SLV_DLY_TARGET_DEFAULT	0x7
#define ESDHC_STROBE_DLL_CTRL_SLV_DLY_TARGET_SHIFT	3
#define ESDHC_STROBE_DLL_CTRL_SLV_UPDATE_INT_DEFAULT	(4 << 20)

#define ESDHC_STROBE_DLL_STATUS		0x74
#define ESDHC_STROBE_DLL_STS_REF_LOCK	(1 << 1)
#define ESDHC_STROBE_DLL_STS_SLV_LOCK	0x1

#define ESDHC_VEND_SPEC2		0xc8
#define ESDHC_VEND_SPEC2_EN_BUSY_IRQ	(1 << 8)
#define ESDHC_VEND_SPEC2_AUTO_TUNE_8BIT_EN	(1 << 4)
#define ESDHC_VEND_SPEC2_AUTO_TUNE_4BIT_EN	(0 << 4)
#define ESDHC_VEND_SPEC2_AUTO_TUNE_1BIT_EN	(2 << 4)
#define ESDHC_VEND_SPEC2_AUTO_TUNE_CMD_EN	(1 << 6)
#define ESDHC_VEND_SPEC2_AUTO_TUNE_MODE_MASK	(7 << 4)

#define ESDHC_TUNING_CTRL		0xcc
#define ESDHC_STD_TUNING_EN		(1 << 24)
#define ESDHC_TUNING_WINDOW_MASK	GENMASK(22, 20)
/* NOTE: the minimum valid tuning start tap for mx6sl is 1 */
#define ESDHC_TUNING_START_TAP_DEFAULT	0x1
#define ESD_TUNING_START_TAP_MASK	0x7f
#define ESDHC_TUNING_CMD_CRC_CHECK_DISABLE	(1 << 7)
#define ESDHC_TUNING_STEP_DEFAULT	0x1
#define ESDHC_TUNING_STEP_MASK		0x00070000
#define ESDHC_TUNING_STEP_SHIFT		16

/* pinctrl state */
#define ESDHC_PINCTRL_STATE_100MHZ	\"state_100mhz\"
#define ESDHC_PINCTRL_STATE_200MHZ	\"state_200mhz\"

/*
 * Our interpretation of the SDHCI_HOST_CONTROL register
 */
#define ESDHC_CTRL_4BITBUS		(0x1 << 1)
#define ESDHC_CTRL_8BITBUS		(0x2 << 1)
#define ESDHC_CTRL_BUSWIDTH_MASK	(0x3 << 1)
#define USDHC_GET_BUSWIDTH(c) (c & ESDHC_CTRL_BUSWIDTH_MASK)

/*
 * There is an INT DMA ERR mismatch between eSDHC and STD SDHC SPEC:
 * Bit25 is used in STD SPEC, and is reserved in fsl eSDHC design,
 * but bit28 is used as the INT DMA ERR in fsl eSDHC design.
 * Define this macro DMA error INT for fsl eSDHC
 */
#define ESDHC_INT_VENDOR_SPEC_DMA_ERR	(1 << 28)

/* the address offset of CQHCI */
#define ESDHC_CQHCI_ADDR_OFFSET		0x100

/*
 * The CMDTYPE of the CMD register (offset 0xE) should be set to
 * \"11\" when the STOP CMD12 is issued on imx53 to abort one
 * open ended multi-blk IO. Otherwise the TC INT wouldn't
 * be generated.
 * In exact block transfer, the controller doesn't complete the
 * operations automatically as required at the end of the
 * transfer and remains on hold if the abort command is not sent.
 * As a result, the TC flag is not asserted and SW received timeout
 * exception. Bit1 of Vendor Spec register is used to fix it.
 */
#define ESDHC_FLAG_MULTIBLK_NO_INT	BIT(1)
/*
 * The flag tells that the ESDHC controller is an USDHC block that is
 * integrated on the i.MX6 series.
 */
#define ESDHC_FLAG_USDHC		BIT(3)
/* The IP supports manual tuning process */
#define ESDHC_FLAG_MAN_TUNING		BIT(4)
/* The IP supports standard tuning process */
#define ESDHC_FLAG_STD_TUNING		BIT(5)
/* The IP has SDHCI_CAPABILITIES_1 register */
#define ESDHC_FLAG_HAVE_CAP1		BIT(6)
/*
 * The IP has erratum ERR004536
 * uSDHC: ADMA Length Mismatch Error occurs if the AHB read access is slow,
 * when reading data from the card
 * This flag is also set for i.MX25 and i.MX35 in order to get
 * SDHCI_QUIRK_BROKEN_ADMA, but for different reasons (ADMA capability bits).
 */
#define ESDHC_FLAG_ERR004536		BIT(7)
/* The IP supports HS200 mode */
#define ESDHC_FLAG_HS200		BIT(8)
/* The IP supports HS400 mode */
#define ESDHC_FLAG_HS400		BIT(9)
/*
 * The IP has errata ERR010450
 * uSDHC: At 1.8V due to the I/O timing limit, for SDR mode, SD card
 * clock can't exceed 150MHz, for DDR mode, SD card clock can't exceed 45MHz.
 */
#define ESDHC_FLAG_ERR010450		BIT(10)
/* The IP supports HS400ES mode */
#define ESDHC_FLAG_HS400_ES		BIT(11)
/* The IP has Host Controller Interface for Command Queuing */
#define ESDHC_FLAG_CQHCI		BIT(12)
/* need request pmqos during low power */
#define ESDHC_FLAG_PMQOS		BIT(13)
/* The IP state got lost in low power mode */
#define ESDHC_FLAG_STATE_LOST_IN_LPMODE		BIT(14)
/* The IP lost clock rate in PM_RUNTIME */
#define ESDHC_FLAG_CLK_RATE_LOST_IN_PM_RUNTIME	BIT(15)
/*
 * The IP do not support the ACMD23 feature completely when use ADMA mode.
 * In ADMA mode, it only use the 16 bit block count of the register 0x4
 * (BLOCK_ATT) as the CMD23's argument for ACMD23 mode, which means it will
 * ignore the upper 16 bit of the CMD23's argument. This will block the reliable
 * write operation in RPMB, because RPMB reliable write need to set the bit31
 * of the CMD23's argument.
 * imx6qpdl/imx6sx/imx6sl/imx7d has this limitation only for ADMA mode, SDMA
 * do not has this limitation. so when these SoC use ADMA mode, it need to
 * disable the ACMD23 feature.
 */
#define ESDHC_FLAG_BROKEN_AUTO_CMD23	BIT(16)

/* ERR004536 is not applicable for the IP  */
#define ESDHC_FLAG_SKIP_ERR004536	BIT(17)

/* The IP does not have GPIO CD wake capabilities */
#define ESDHC_FLAG_SKIP_CD_WAKE		BIT(18)

/* the controller has dummy pad for clock loopback */
#define ESDHC_FLAG_DUMMY_PAD		BIT(19)

#define ESDHC_AUTO_TUNING_WINDOW	3

enum wp_types {
	ESDHC_WP_NONE,		/* no WP, neither controller nor gpio */
	ESDHC_WP_CONTROLLER,	/* mmc controller internal WP */
	ESDHC_WP_GPIO,		/* external gpio pin for WP */
};

enum cd_types {
	ESDHC_CD_NONE,		/* no CD, neither controller nor gpio */
	ESDHC_CD_CONTROLLER,	/* mmc controller internal CD */
	ESDHC_CD_GPIO,		/* external gpio pin for CD */
	ESDHC_CD_PERMANENT,	/* no CD, card permanently wired to host */
};

/*
 * struct esdhc_platform_data - platform data for esdhc on i.MX
 *
 * ESDHC_WP(CD)_CONTROLLER type is not available on i.MX25/35.
 *
 * @wp_type:	type of write_protect method (see wp_types enum above)
 * @cd_type:	type of card_detect method (see cd_types enum above)
 */

struct esdhc_platform_data {
	enum wp_types wp_type;
	enum cd_types cd_type;
	int max_bus_width;
	unsigned int delay_line;
	unsigned int tuning_step;	/* The delay cell steps in tuning procedure */
	unsigned int tuning_start_tap;	/* The start delay cell point in tuning procedure */
	unsigned int strobe_dll_delay_target;	/* The delay cell for strobe pad (read clock) */
	unsigned int saved_tuning_delay_cell;	/* save the value of tuning delay cell */
	unsigned int saved_auto_tuning_window;	/* save the auto tuning window width */
};

struct esdhc_soc_data {
	u32 flags;
	u32 quirks;
};

static const struct esdhc_soc_data esdhc_imx25_35_data = {
	.flags = ESDHC_FLAG_ERR004536,
};

static const struct esdhc_soc_data esdhc_imx51_data = {
	.flags = 0,
};

static const struct esdhc_soc_data esdhc_imx53_data = {
	.flags = ESDHC_FLAG_MULTIBLK_NO_INT,
};

static const struct esdhc_soc_data usdhc_imx6q_data = {
	.flags = ESDHC_FLAG_USDHC | ESDHC_FLAG_MAN_TUNING
			| ESDHC_FLAG_BROKEN_AUTO_CMD23,
};

static const struct esdhc_soc_data usdhc_imx6sl_data = {
	.flags = ESDHC_FLAG_USDHC | ESDHC_FLAG_MAN_TUNING
			| ESDHC_FLAG_HAVE_CAP1 | ESDHC_FLAG_ERR004536
			| ESDHC_FLAG_HS200
			| ESDHC_FLAG_BROKEN_AUTO_CMD23,
};

static const struct esdhc_soc_data usdhc_imx6sll_data = {
	.flags = ESDHC_FLAG_USDHC | ESDHC_FLAG_MAN_TUNING
			| ESDHC_FLAG_HAVE_CAP1 | ESDHC_FLAG_HS200
			| ESDHC_FLAG_HS400
			| ESDHC_FLAG_STATE_LOST_IN_LPMODE,
};

static const struct esdhc_soc_data usdhc_imx6sx_data = {
	.flags = ESDHC_FLAG_USDHC | ESDHC_FLAG_MAN_TUNING
			| ESDHC_FLAG_HAVE_CAP1 | ESDHC_FLAG_HS200
			| ESDHC_FLAG_STATE_LOST_IN_LPMODE
			| ESDHC_FLAG_BROKEN_AUTO_CMD23,
};

static const struct esdhc_soc_data usdhc_imx6ull_data = {
	.flags = ESDHC_FLAG_USDHC | ESDHC_FLAG_MAN_TUNING
			| ESDHC_FLAG_HAVE_CAP1 | ESDHC_FLAG_HS200
			| ESDHC_FLAG_ERR010450
			| ESDHC_FLAG_STATE_LOST_IN_LPMODE,
};

static const struct esdhc_soc_data usdhc_imx7d_data = {
	.flags = ESDHC_FLAG_USDHC | ESDHC_FLAG_MAN_TUNING
			| ESDHC_FLAG_HAVE_CAP1 | ESDHC_FLAG_HS200
			| ESDHC_FLAG_HS400
			| ESDHC_FLAG_STATE_LOST_IN_LPMODE
			| ESDHC_FLAG_BROKEN_AUTO_CMD23,
};

static struct esdhc_soc_data usdhc_s32g2_data = {
	.flags = ESDHC_FLAG_USDHC | ESDHC_FLAG_MAN_TUNING
			| ESDHC_FLAG_HAVE_CAP1 | ESDHC_FLAG_HS200
			| ESDHC_FLAG_HS400 | ESDHC_FLAG_HS400_ES
			| ESDHC_FLAG_SKIP_ERR004536 | ESDHC_FLAG_SKIP_CD_WAKE,
	.quirks = SDHCI_QUIRK_NO_LED,
};

static struct esdhc_soc_data usdhc_imx7ulp_data = {
	.flags = ESDHC_FLAG_USDHC | ESDHC_FLAG_MAN_TUNING
			| ESDHC_FLAG_HAVE_CAP1 | ESDHC_FLAG_HS200
			| ESDHC_FLAG_PMQOS | ESDHC_FLAG_HS400
			| ESDHC_FLAG_STATE_LOST_IN_LPMODE,
	.quirks = SDHCI_QUIRK_NO_LED,
};
static struct esdhc_soc_data usdhc_imxrt1050_data = {
	.flags = ESDHC_FLAG_USDHC | ESDHC_FLAG_MAN_TUNING
			| ESDHC_FLAG_HAVE_CAP1 | ESDHC_FLAG_HS200,
	.quirks = SDHCI_QUIRK_NO_LED,
};

static struct esdhc_soc_data usdhc_imx8qxp_data = {
	.flags = ESDHC_FLAG_USDHC | ESDHC_FLAG_MAN_TUNING
			| ESDHC_FLAG_HAVE_CAP1 | ESDHC_FLAG_HS200
			| ESDHC_FLAG_HS400 | ESDHC_FLAG_HS400_ES
			| ESDHC_FLAG_STATE_LOST_IN_LPMODE
			| ESDHC_FLAG_CLK_RATE_LOST_IN_PM_RUNTIME,
	.quirks = SDHCI_QUIRK_NO_LED,
};

static struct esdhc_soc_data usdhc_imx8mm_data = {
	.flags = ESDHC_FLAG_USDHC | ESDHC_FLAG_MAN_TUNING
			| ESDHC_FLAG_HAVE_CAP1 | ESDHC_FLAG_HS200
			| ESDHC_FLAG_HS400 | ESDHC_FLAG_HS400_ES
			| ESDHC_FLAG_STATE_LOST_IN_LPMODE,
	.quirks = SDHCI_QUIRK_NO_LED,
};

static struct esdhc_soc_data usdhc_imx95_data = {
	.flags = ESDHC_FLAG_USDHC | ESDHC_FLAG_MAN_TUNING
			| ESDHC_FLAG_HAVE_CAP1 | ESDHC_FLAG_HS200
			| ESDHC_FLAG_HS400 | ESDHC_FLAG_HS400_ES
			| ESDHC_FLAG_STATE_LOST_IN_LPMODE
			| ESDHC_FLAG_DUMMY_PAD,
	.quirks = SDHCI_QUIRK_NO_LED,
};

struct pltfm_imx_data {
	u32 scratchpad;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_100mhz;
	struct pinctrl_state *pins_200mhz;
	const struct esdhc_soc_data *socdata;
	struct esdhc_platform_data boarddata;
	struct clk *clk_ipg;
	struct clk *clk_ahb;
	struct clk *clk_per;
	unsigned int actual_clock;

	/*
	 * USDHC has one limition, require the SDIO device a different
	 * register setting. Driver has to recognize card type during
	 * the card init, but at this stage, mmc_host->card is not
	 * available. So involve this field to save the card type
	 * during card init through usdhc_init_card().
	 */
	unsigned int init_card_type;

	enum {
		NO_CMD_PENDING,      /* no multiblock command pending */
		MULTIBLK_IN_PROCESS, /* exact multiblock cmd in process */
		WAIT_FOR_INT,        /* sent CMD12, waiting for response INT */
	} multiblock_status;
	u32 is_ddr;
	struct pm_qos_request pm_qos_req;
};

static const struct of_device_id imx_esdhc_dt_ids[] = {
	{ .compatible = \"fsl,imx25-esdhc\", .data = &esdhc_imx25_35_data, },
	{ .compatible = \"fsl,imx35-esdhc\", .data = &esdhc_imx25_35_data, },
	{ .compatible = \"fsl,imx51-esdhc\", .data = &esdhc_imx51_data, },
	{ .compatible = \"fsl,imx53-esdhc\", .data = &esdhc_imx53_data, },
	{ .compatible = \"fsl,imx6sx-usdhc\", .data = &usdhc_imx6sx_data, },
	{ .compatible = \"fsl,imx6sl-usdhc\", .data = &usdhc_imx6sl_data, },
	{ .compatible = \"fsl,imx6sll-usdhc\", .data = &usdhc_imx6sll_data, },
	{ .compatible = \"fsl,imx6q-usdhc\", .data = &usdhc_imx6q_data, },
	{ .compatible = \"fsl,imx6ull-usdhc\", .data = &usdhc_imx6ull_data, },
	{ .compatible = \"fsl,imx7d-usdhc\", .data = &usdhc_imx7d_data, },
	{ .compatible = \"fsl,imx7ulp-usdhc\", .data = &usdhc_imx7ulp_data, },
	{ .compatible = \"fsl,imx8qxp-usdhc\", .data = &usdhc_imx8qxp_data, },
	{ .compatible = \"fsl,imx8mm-usdhc\", .data = &usdhc_imx8mm_data, },
	{ .compatible = \"fsl,imx94-usdhc\", .data = &usdhc_imx95_data, },
	{ .compatible = \"fsl,imx95-usdhc\", .data = &usdhc_imx95_data, },
	{ .compatible = \"fsl,imxrt1050-usdhc\", .data = &usdhc_imrt1050_data, },
	{ .compatible = \"nxp,s32g2-usdhc\", .data = &usdhc_s32g2_data, },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_esdhc_dt_ids);

static inline int is_imx25_esdhc(struct pltfm_imx_data *data)
{
	return data->socdata == &esdhc_imx25_35_data;
}

static inline int is_imx53_esdhc(struct pltfm_imx_data *data)
{
	return data->socdata == &esdhc_imx53_data;
}

static inline int esdhc_is_usdhc(struct pltfm_imx_data *data)
{
	return !!(data->socdata->flags & ESDHC_FLAG_USDHC);
}

static inline void esdhc_clrset_le(struct sdhci_host *host, u32 mask, u32 val, int reg)
{
	void __iomem *base = host->ioaddr + (reg & ~0x3);
	u32 shift = (reg & 0x3) * 8;

	writel(((readl(base) & ~(mask << shift)) | (val << shift)), base);
}

#define DRIVER_NAME \"sdhci-esdhc-imx\"
#define ESDHC_IMX_DUMP(f, x...) \
	pr_err(\"%s: \" DRIVER_NAME \": \" f, mmc_hostname(host->mmc), ## x)
static void esdhc_dump_debug_regs(struct sdhci_host *host)
{
	int i;
	char *debug_status[7] = {
				 \"cmd debug status\",
				 \"data debug status\",
				 \"trans debug status\",
				 \"dma debug status\",
				 \"adma debug status\",
				 \"fifo debug status\",
				 \"async fifo debug status\"
	};

	ESDHC_IMX_DUMP(\"========= ESDHC IMX DEBUG STATUS DUMP =========\\n\");
	for (i = 0; i < 7; i++) {
		esdhc_clrset_le(host, ESDHC_DEBUG_SEL_MASK,
			ESDHC_DEBUG_SEL_CMD_STATE + i, ESDHC_DEBUG_SEL_REG);
		ESDHC_IMX_DUMP(\"%s:  0x%04x\\n\", debug_status[i],
			readw(host->ioaddr + ESDHC_DEBUG_SEL_AND_STATUS_REG));
	}

	esdhc_clrset_le(host, ESDHC_DEBUG_SEL_MASK, 0, ESDHC_DEBUG_SEL_REG);

}

static inline void esdhc_wait_for_card_clock_gate_off(struct sdhci_host *host)
{
	u32 present_state;
	int ret;

	ret = readl_poll_timeout(host->ioaddr + ESDHC_PRSSTAT, present_state,
				(present_state & ESDHC_CLOCK_GATE_OFF), 2, 100);
	if (ret == -ETIMEDOUT)
		dev_warn(mmc_dev(host->mmc), \"%s: card clock still not gate off in 100us!.\\n\", __func__);
}

/* Enable the auto tuning circuit to check the CMD line and BUS line */
static inline void usdhc_auto_tuning_mode_sel_and_en(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct pltfm_imx_data *imx_data = sdhci_pltfm_priv(pltfm_host);
	u32 buswidth, auto_tune_buswidth;
	u32 reg;

	buswidth = USDHC_GET_BUSWIDTH(readl(host->ioaddr + SDHCI_HOST_CONTROL));

	switch (buswidth) {
	case ESDHC_CTRL_8BITBUS:
		auto_tune_buswidth = ESDHC_VEND_SPEC2_AUTO_TUNE_8BIT_EN;
		break;
	case ESDHC_CTRL_4BITBUS:
		auto_tune_buswidth = ESDHC_VEND_SPEC2_AUTO_TUNE_4BIT_EN;
		break;
	default:	/* 1BITBUS */
		auto_tune_buswidth = ESDHC_VEND_SPEC2_AUTO_TUNE_1BIT_EN;
		break;
	}

	/*
	 * For USDHC, auto tuning circuit can not handle the async sdio
	 * device interrupt correctly. When sdio device use 4 data lines,
	 * async sdio interrupt will use the shared DAT[1], if enable auto
	 * tuning circuit check these 4 data lines, include the DAT[1],
	 * this circuit will detect this interrupt, take this as a data on
	 * DAT[1], and adjust the delay cell wrongly.
	 * This is the hardware design limitation, to avoid this, for sdio
	 * device, config the auto tuning circuit only check DAT[0] and CMD
	 * line.
	 */
	if (imx_data->init_card_type == MMC_TYPE_SDIO)
		auto_tune_buswidth = ESDHC_VEND_SPEC2_AUTO_TUNE_1BIT_EN;

	esdhc_clrset_le(host, ESDHC_VEND_SPEC2_AUTO_TUNE_MODE_MASK,
			auto_tune_buswidth | ESDHC_VEND_SPEC2_AUTO_TUNE_CMD_EN,
			ESDHC_VEND_SPEC2);

	reg = readl(host->ioaddr + ESDHC_MIX_CTRL);
	reg |= ESDHC_MIX_CTRL_AUTO_TUNE_EN;
	writel(reg, host->ioaddr + ESDHC_MIX_CTRL);
}

static u32 esdhc_readl_le(struct sdhci_host *host, int reg)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct pltfm_imx_data *imx_data = sdhci_pltfm_priv(pltfm_host);
	u32 val = readl(host->ioaddr + reg);

	if (unlikely(reg == SDHCI_PRESENT_STATE)) {
		u32 fsl_prss = val;
		/* save the least 20 bits */
		val = fsl_prss & 0x000FFFFF;
		/* move dat[0-3] bits */
		val |= (fsl_prss & 0x0F000000) >> 4;
		/* move cmd line bit */
		val |= (fsl_prss & 0x00800000) << 1;
	}

	if (unlikely(reg == SDHCI_CAPABILITIES)) {
		/* ignore bit[0-15] as it stores cap_1 register val for mx6sl */
		if (imx_data->socdata->flags & ESDHC_FLAG_HAVE_CAP1)
			val &= 0xffff0000;

		/*
		 * In FSL esdhc IC module, only bit20 is used to indicate the
		 * ADMA2 capability of esdhc, but this bit is messed up on
		 * some SOCs (e.g. on MX25, MX35 this bit is set, but they
		 * don't actually support ADMA2). So set the BROKEN_ADMA
		 * quirk on MX25/35 platforms.
		 */

		if (val & SDHCI_CAN_DO_ADMA1) {
			val &= ~SDHCI_CAN_DO_ADMA1;
			val |= SDHCI_CAN_DO_ADMA2;
		}
	}

	if (unlikely(reg == SDHCI_CAPABILITIES_1)) {
		if (esdhc_is_usdhc(imx_data)) {
			if (imx_data->socdata->flags & ESDHC_FLAG_HAVE_CAP1)
				val = readl(host->ioaddr + SDHCI_CAPABILITIES) & 0xFFFF;
			else
				/* imx6q/dl does not have cap_1 register, fake one */
				val = SDHCI_SUPPORT_DDR50 | SDHCI_SUPPORT_SDR104
					| SDHCI_SUPPORT_SDR50
					| SDHCI_USE_SDR50_TUNING
					| FIELD_PREP(SDHCI_RETUNING_MODE_MASK,
						     SDHCI_TUNING_MODE_3);

			/*
			 * Do not advertise faster UHS modes if there are no
			 * pinctrl states for 100MHz/200MHz.
			 */
			if (IS_ERR_OR_NULL(imx_data->pins_100mhz))
				val &= ~(SDHCI_SUPPORT_SDR50 | SDHCI_SUPPORT_DDR50);
			if (IS_ERR_OR_NULL(imx_data->pins_200mhz))
				val &= ~(SDHCI_SUPPORT_SDR104 | SDHCI_SUPPORT_HS400);
		}
	}

	if (unlikely(reg == SDHCI_MAX_CURRENT) && esdhc_is_usdhc(imx_data)) {
		val = 0;
		val |= FIELD_PREP(SDHCI_MAX_CURRENT_330_MASK, 0xFF);
		val |= FIELD_PREP(SDHCI_MAX_CURRENT_300_MASK, 0xFF);
		val |= FIELD_PREP(SDHCI_MAX_CURRENT_180_MASK, 0xFF);
	}

	if (unlikely(reg == SDHCI_INT_STATUS)) {
		if (val & ESDHC_INT_VENDOR_SPEC_DMA_ERR) {
			val &= ~ESDHC_INT_VENDOR_SPEC_DMA_ERR;
			val |= SDHCI_INT_ADMA_ERROR;
		}

		/*
		 * mask off the interrupt we get in response to the manually
		 * sent CMD12
		 */
		if ((imx_data->multiblock_status == WAIT_FOR_INT) &&
		    ((val & SDHCI_INT_RESPONSE) == SDHCI_INT_RESPONSE)) {
			val &= ~SDHCI_INT_RESPONSE;
			writel(SDHCI_INT_RESPONSE, host->ioaddr +
						   SDHCI_INT_STATUS);
			imx_data->multiblock_status = NO_CMD_PENDING;
		}
	}

	return val;
}

static void esdhc_writel_le(struct sdhci_host *host, u32 val, int reg)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct pltfm_imx_data *imx_data = sdhci_pltfm_priv(pltfm_host);
	u32 data;

	if (unlikely(reg == SDHCI_INT_ENABLE || reg == SDHCI_SIGNAL_ENABLE ||
			reg == SDHCI_INT_STATUS)) {
		if ((val & SDHCI_INT_CARD_INT) && !esdhc_is_usdhc(imx_data)) {
			/*
			 * Clear and then set D3CD bit to avoid missing the
			 * card interrupt. This is an eSDHC controller problem
			 * so we need to apply the following workaround: clear
			 * and set D3CD bit will make eSDHC re-sample the card
			 * interrupt. In case a card interrupt was lost,
			 * re-sample it by the following steps.
			 */
			data = readl(host->ioaddr + SDHCI_HOST_CONTROL);
			data &= ~ESDHC_CTRL_D3CD;
			writel(data, host->ioaddr + SDHCI_HOST_CONTROL);
			data |= ESDHC_CTRL_D3CD;
			writel(data, host->ioaddr + SDHCI_HOST_CONTROL);
		}

		if (val & SDHCI_INT_ADMA_ERROR) {
			val &= ~SDHCI_INT_ADMA_ERROR;
			val |= ESDHC_INT_VENDOR_SPEC_DMA_ERR;
		}
	}

	if (unlikely((imx_data->socdata->flags & ESDHC_FLAG_MULTIBLK_NO_INT) &&
		     (reg == SDHCI_INT_STATUS) &&
		     (val & SDHCI_INT_DATA_END))) {
		u32 v;

		v = readl(host->ioaddr + ESDHC_VENDOR_SPEC);
		v &= ~ESDHC_VENDOR_SPEC_SDIO_QUIRK;
		writel(v, host->ioaddr + ESDHC_VENDOR_SPEC);

		if (imx_data->multiblock_status == MULTIBLK_IN_PROCESS) {
			/* send a manual CMD12 with RESPTYP=none */
			data = MMC_STOP_TRANSMISSION << 24 |
			       SDHCI_CMD_ABORTCMD << 16;
			writel(data, host->ioaddr + SDHCI_TRANSFER_MODE);
			imx_data->multiblock_status = WAIT_FOR_INT;
		}
	}

	writel(val, host->ioaddr + reg);
}

static u16 esdhc_readw_le(struct sdhci_host *host, int reg)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct pltfm_imx_data *imx_data = sdhci_pltfm_priv(pltfm_host);
	u16 ret = 0;
	u32 val;

	if (unlikely(reg == SDHCI_HOST_VERSION)) {
		reg ^= 2;
		if (esdhc_is_usdhc(imx_data)) {
			/*
			 * The usdhc register returns a wrong host version.
			 * Correct it here.
			 */
			return SDHCI_SPEC_300;
		}
	}

	if (unlikely(reg == SDHCI_HOST_CONTROL2)) {
		val = readl(host->ioaddr + ESDHC_VENDOR_SPEC);
		if (val & ESDHC_VENDOR_SPEC_VSELECT)
			ret |= SDHCI_CTRL_VDD_180;

		if (esdhc_is_usdhc(imx_data)) {
			if (imx_data->socdata->flags & ESDHC_FLAG_MAN_TUNING)
				val = readl(host->ioaddr + ESDHC_MIX_CTRL);
			else if (imx_data->socdata->flags & ESDHC_FLAG_STD_TUNING)
				/* the std tuning bits is in ACMD12_ERR for imx6sl */
				val = readl(host->ioaddr + SDHCI_AUTO_CMD_STATUS);
		}

		if (val & ESDHC_MIX_CTRL_EXE_TUNE)
			ret |= SDHCI_CTRL_EXEC_TUNING;
		if (val & ESDHC_MIX_CTRL_SMPCLK_SEL)
			ret |= SDHCI_CTRL_TUNED_CLK;

		ret &= ~SDHCI_CTRL_PRESET_VAL_ENABLE;

		return ret;
	}

	if (unlikely(reg == SDHCI_TRANSFER_MODE)) {
		if (esdhc_is_usdhc(imx_data)) {
			u32 m = readl(host->ioaddr + ESDHC_MIX_CTRL);

			ret = m & ESDHC_MIX_CTRL_SDHCI_MASK;
			/* Swap AC23 bit */
			if (m & ESDHC_MIX_CTRL_AC23EN) {
				ret &= ~ESDHC_MIX_CTRL_AC23EN;
				ret |= SDHCI_TRNS_AUTO_CMD23;
			}
		} else {
			ret = readw(host->ioaddr + SDHCI_TRANSFER_MODE);
		}

		return ret;
	}

	return readw(host->ioaddr + reg);
}

static void esdhc_writew_le(struct sdhci_host *host, u16 val, int reg)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct pltfm_imx_data *imx_data = sdhci_pltfm_priv(pltfm_host);
	u32 new_val = 0;

	switch (reg) {
	case SDHCI_CLOCK_CONTROL:
		new_val = readl(host->ioaddr + ESDHC_VENDOR_SPEC);
		if (val & SDHCI_CLOCK_CARD_EN)
			new_val |= ESDHC_VENDOR_SPEC_FRC_SDCLK_ON;
		else
			new_val &= ~ESDHC_VENDOR_SPEC_FRC_SDCLK_ON;
		writel(new_val, host->ioaddr + ESDHC_VENDOR_SPEC);
		if (!(new_val & ESDHC_VENDOR_SPEC_FRC_SDCLK_ON))
			esdhc_wait_for_card_clock_gate_off(host);
		return;
	case SDHCI_HOST_CONTROL2:
		new_val = readl(host->ioaddr + ESDHC_VENDOR_SPEC);
		if (val & SDHCI_CTRL_VDD_180)
			new_val |= ESDHC_VENDOR_SPEC_VSELECT;
		else
			new_val &= ~ESDHC_VENDOR_SPEC_VSELECT;
		writel(new_val, host->ioaddr + ESDHC_VENDOR_SPEC);
		if (imx_data->socdata->flags & ESDHC_FLAG_STD_TUNING) {
			u32 v = readl(host->ioaddr + SDHCI_AUTO_CMD_STATUS);

			if (val & SDHCI_CTRL_TUNED_CLK)
				v |= ESDHC_MIX_CTRL_SMPCLK_SEL;
			else
				v &= ~ESDHC_MIX_CTRL_SMPCLK_SEL;

			if (val & SDHCI_CTRL_EXEC_TUNING)
				v |= ESDHC_MIX_CTRL_EXE_TUNE;
			else
				v &= ~ESDHC_MIX_CTRL_EXE_TUNE;

			writel(v, host->ioaddr + SDHCI_AUTO_CMD_STATUS);
		}
		return;
	case SDHCI_TRANSFER_MODE:
		if ((imx_data->socdata->flags & ESDHC_FLAG_MULTIBLK_NO_INT) &&
		    (host->cmd->opcode == SD_IO_RW_EXTENDED) &&
		    (host->cmd->data->blocks > 1) &&
		    (host->cmd->data->flags & MMC_DATA_READ)) {
			u32 v;

			v = readl(host->ioaddr + ESDHC_VENDOR_SPEC);
			v |= ESDHC_VENDOR_SPEC_SDIO_QUIRK;
			writel(v, host->ioaddr + ESDHC_VENDOR_SPEC);
		}

		if (esdhc_is_usdhc(imx_data)) {
			u32 wml;
			u32 m = readl(host->ioaddr + ESDHC_MIX_CTRL);
			/* Swap AC23 bit */
			if (val & SDHCI_TRNS_AUTO_CMD23) {
				val &= ~SDHCI_TRNS_AUTO_CMD23;
				val |= ESDHC_MIX_CTRL_AC23EN;
			}
			m = val | (m & ~ESDHC_MIX_CTRL_SDHCI_MASK);
			writel(m, host->ioaddr + ESDHC_MIX_CTRL);

			/*
			 * Set watermark levels for PIO access to maximum value
			 * (128 words) to accommodate full 512 bytes buffer.
			 * For DMA access restore the levels to default value.
			 */
			m = readl(host->ioaddr + ESDHC_WTMK_LVL);
			if (val & SDHCI_TRNS_DMA) {
				wml = ESDHC_WTMK_LVL_WML_VAL_DEF;
			} else {
				u8 ctrl;

				wml = ESDHC_WTMK_LVL_WML_VAL_MAX;

				/*
				 * Since already disable DMA mode, so also need
				 * to clear the DMASEL. Otherwise, for standard
				 * tuning, when send tuning command, usdhc will
				 * still prefetch the ADMA script from wrong
				 * DMA address, then we will see IOMMU report
				 * some error which show lack of TLB mapping.
				 */
				ctrl = sdhci_readb(host, SDHCI_HOST_CONTROL);
				ctrl &= ~SDHCI_CTRL_DMA_MASK;
				sdhci_writeb(host, ctrl, SDHCI_HOST_CONTROL);
			}
			m &= ~(ESDHC_WTMK_LVL_RD_WML_MASK | ESDHC_WTMK_LVL_WR_WML_MASK);
			m |= (wml << ESDHC_WTMK_LVL_RD_WML_SHIFT) |
			     (wml << ESDHC_WTMK_LVL_WR_WML_SHIFT);
			writel(m, host->ioaddr + ESDHC_WTMK_LVL);
		} else {
			writew(val, host->ioaddr + SDHCI_TRANSFER_MODE);
		}
		return;
	case SDHCI_COMMAND:
		if (esdhc_is_usdhc(imx_data))
			writel(((u32)val) << 16, host->ioaddr + SDHCI_TRANSFER_MODE);
		else
			writew(val, host->ioaddr + SDHCI_COMMAND);
		return;
	}

	writew(val, host->ioaddr + reg);
}

static u8 esdhc_readb_le(struct sdhci_host *host, int reg)
{
	u8 ret;
	u32 val;

	switch (reg) {
	case SDHCI_HOST_CONTROL:
		val = readl(host->ioaddr + reg);

		ret = val & SDHCI_CTRL_LED;
		ret |= (val >> 5) & SDHCI_CTRL_DMA_MASK;
		ret |= (val & ESDHC_CTRL_4BITBUS);
		ret |= (val & ESDHC_CTRL_8BITBUS) << 2;
		return ret;
	}

	return readb(host->ioaddr + reg);
}

static void esdhc_writeb_le(struct sdhci_host *host, u8 val, int reg)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct pltfm_imx_data *imx_data = sdhci_pltfm_priv(pltfm_host);
	u32 new_val;
	u32 mask;

	switch (reg) {
	case SDHCI_HOST_CONTROL:
		/* FSL eSDHC doesn't support platform-wide power management */
		val &= ~SDHCI_CTRL_LED;

		new_val = val & SDHCI_CTRL_LED;
		new_val |= (val & SDHCI_CTRL_DMA_MASK) << 5;
		new_val |= (val & SDHCI_CTRL_4BITBUS);
		new_val |= (val >> 2) & ESDHC_CTRL_8BITBUS;

		mask = SDHCI_CTRL_LED | ESDHC_CTRL_BUSWIDTH_MASK | (SDHCI_CTRL_DMA_MASK << 5);
		esdhc_clrset_le(host, mask, new_val, reg);
		return;
	case SDHCI_POWER_CONTROL:
		if (esdhc_is_usdhc(imx_data))
			return;
	}
	writeb(val, host->ioaddr + reg);
}

static unsigned int esdhc_pltfm_get_max_clock(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct pltfm_imx_data *imx_data = sdhci_pltfm_priv(pltfm_host);

	return clk_get_rate(imx_data->clk_per);
}

static unsigned int esdhc_pltfm_get_min_clock(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct pltfm_imx_data *imx_data = sdhci_pltfm_priv(pltfm_host);

	return clk_get_rate(imx_data->clk_per) / 256 / 16;
}

static inline void esdhc_pltfm_set_clock(struct sdhci_host *host,
					unsigned int clock)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct pltfm_imx_data *imx_data = sdhci_pltfm_priv(pltfm_host);
	unsigned int host_clock = clk_get_rate(imx_data->clk_per);
	int pre_div = 1;
	int div = 1;
	u32 temp, mask;

	if (clock == 0) {
		host->clock = 0;
		return;
	}

	if (esdhc_is_usdhc(imx_data) && !imx_data->actual_clock)
		imx_data->actual_clock = host_clock;

	if (clock > host_clock)
		clock = host_clock;

	while (host_clock / pre_div / 16 > clock && pre_div < 256)
		pre_div <<= 1;

	while (host_clock / pre_div / div > clock && div < 16)
		div++;

	host->clock = host_clock / pre_div / div;

	temp = sdhci_readl(host, ESDHC_SYSTEM_CONTROL);
	mask = ~(ESDHC_CLOCK_IPGEN | ESDHC_CLOCK_HCKEN | ESDHC_CLOCK_PEREN
		| ESDHC_CLOCK_MASK);
	temp &= mask;
	temp |= ESDHC_CLOCK_IPGEN | ESDHC_CLOCK_HCKEN | ESDHC_CLOCK_PEREN
		| (div - 1) << ESDHC_VENDOR_V_23_DIVIDER_SHIFT
		| (pre_div >> 1) << ESDHC_SYSTEM_CONTROL_PRE_DIVIDER_SHIFT;
	sdhci_writel(host, temp, ESDHC_SYSTEM_CONTROL);

	mdelay(1);
}

static const struct sdhci_ops sdhci_esdhc_imx_ops = {
	.read_l = esdhc_readl_le,
	.read_w = esdhc_readw_le,
	.read_b = esdhc_readb_le,
	.write_l = esdhc_writel_le,
	.write_w = esdhc_writew_le,
	.write_b = esdhc_writeb_le,
	.set_clock = esdhc_pltfm_set_clock,
	.get_max_clock = esdhc_pltfm_get_max_clock,
	.get_min_clock = esdhc_pltfm_get_min_clock,
};

static const struct sdhci_pltfm_data sdhci_esdhc_imx_pdata = {
	.ops = &sdhci_esdhc_imx_ops,
	.quirks = SDHCI_QUIRK_NO_ENDATTR_IN_NOP_DESC,
	.quirks2 = SDHCI_QUIRK2_PRESET_VALUE_BROKEN,
};

static int sdhci_esdhc_imx_probe(struct platform_device *pdev)
{
	const struct of_device_id *of_id =
			of_match_device(imx_esdhc_dt_ids, &pdev->dev);
	struct sdhci_host *host;
	struct sdhci_pltfm_host *pltfm_host;
	struct pltfm_imx_data *imx_data;
	int err;

	host = sdhci_pltfm_init(pdev, &sdhci_esdhc_imx_pdata, 0);
	if (IS_ERR(host))
		return PTR_ERR(host);

	pltfm_host = sdhci_priv(host);
	imx_data = sdhci_pltfm_priv(pltfm_host);

	imx_data->socdata = of_id->data;

	if (imx_data->socdata->quirks)
		host->quirks2 |= imx_data->socdata->quirks;

	err = sdhci_add_host(host);
	if (err)
		goto err_add_host;

	return 0;
}

static int sdhci_esdhc_imx_remove(struct platform_device *pdev)
{
	struct sdhci_host *host = platform_get_drvdata(pdev);

	sdhci_remove_host(host, 0);
	sdhci_pltfm_free(pdev);

	return 0;
}

static struct platform_driver sdhci_esdhc_imx_driver = {
	.driver = {
		.name = \"sdhci-esdhc-imx\",
		.of_match_table = imx_esdhc_dt_ids,
		.pm = &sdhci_pltfm_pmops,
	},
	.probe = sdhci_esdhc_imx_probe,
	.remove = sdhci_esdhc_imx_remove,
};

module_platform_driver(sdhci_esdhc_imx_driver);

MODULE_DESCRIPTION(\"SDHCI platform driver for Freescale i.MX eSDHC\");
MODULE_AUTHOR(\"Wolfram Sang <kernel@pengutronix.de>\");
MODULE_LICENSE(\"GPL v2\");
