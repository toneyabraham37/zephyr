/*
 * Copyright (c) 2017 Google LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT mchp_v1_spi

#include "spi_context.h"
#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/spi/rtio.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/pinctrl.h>
#include <soc.h>

/* Device constant configuration parameters */
struct spi_mchp_v1_config {
	SercomSpi *regs;
	uint32_t pads;
	const struct pinctrl_dev_config *pcfg;
#ifdef MCLK
	volatile uint32_t *mclk;
	uint32_t mclk_mask;
	uint16_t gclk_core_id;
#else
	uint32_t pm_apbcmask;
	uint16_t gclk_clkctrl_id;
#endif
#ifdef CONFIG_SPI_ASYNC
	const struct device *dma_dev;
	uint8_t tx_dma_request;
	uint8_t tx_dma_channel;
	uint8_t rx_dma_request;
	uint8_t rx_dma_channel;
#endif
};

/* Device run time data */
struct spi_mchp_v1_data {
	struct spi_context ctx;
#ifdef CONFIG_SPI_ASYNC
	const struct device *dev;
	uint32_t dma_segment_len;
#endif
};
static int spi_mchp_v1_transceive_sync(const struct device *dev, const struct spi_config *config,
				       const struct spi_buf_set *tx_bufs,
				       const struct spi_buf_set *rx_bufs)
{
	// Write the spi transceive implementation
	printf("This is the spi transceive implementation");
    return 0;
}

static int spi_mchp_v1_release(const struct device *dev, const struct spi_config *config)
{
	// Release function
	printf("This is the Spi release function");
        return 0;

}

static int spi_mchp_v1_init(const struct device *dev)
{
	// Initialize the spi
	// printf("This is the spi Initialize");
        return 0;

}

static const struct spi_driver_api spi_mchp_v1_driver_api = {
	.transceive = spi_mchp_v1_transceive_sync,
	.release = spi_mchp_v1_release,
};

#define SPI_MCHP_V1_DMA_CHANNELS(n)

#define SPI_MCHP_V1_SERCOM_PADS(n)                                                                 \
	SERCOM_SPI_CTRLA_DIPO(DT_INST_PROP(n, dipo)) | SERCOM_SPI_CTRLA_DOPO(DT_INST_PROP(n, dopo))

#ifdef MCLK
#define SPI_MCHP_V1_DEFINE_CONFIG(n)					\
static const struct spi_mchp_v1_config spi_mchp_v1_config_##n = {		\
	.regs = (SercomSpi *)DT_INST_REG_ADDR(n),			\
	.mclk = (volatile uint32_t *)MCLK_MASK_DT_INT_REG_ADDR(n),	\
	.mclk_mask = BIT(DT_INST_CLOCKS_CELL_BY_NAME(n, mclk, bit)),	\
	.gclk_core_id = DT_INST_CLOCKS_CELL_BY_NAME(n, gclk, periph_ch),\
	.pads = SPI_MCHP_V1_SERCOM_PADS(n),				\
	.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),			\
	SPI_MCHP_V1_DMA_CHANNELS(n)					\
}
#else
#define SPI_MCHP_V1_DEFINE_CONFIG(n)					\
static const struct spi_mchp_v1_config spi_mchp_v1_config_##n = {		\
	.regs = (SercomSpi *)DT_INST_REG_ADDR(n),			\
	.pm_apbcmask = BIT(DT_INST_CLOCKS_CELL_BY_NAME(n, pm, bit)),	\
	.gclk_clkctrl_id = DT_INST_CLOCKS_CELL_BY_NAME(n, gclk, clkctrl_id),\
	.pads = SPI_MCHP_V1_SERCOM_PADS(n),				\
	.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),			\
	SPI_MCHP_V1_DMA_CHANNELS(n)					\
}
#endif /* MCLK */

#define SPI_MCHP_V1_DEVICE_INIT(n)                                                                 \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	SPI_MCHP_V1_DEFINE_CONFIG(n);                                                              \
	static struct spi_mchp_v1_data spi_mchp_v1_dev_data_##n = {                                \
		SPI_CONTEXT_INIT_LOCK(spi_mchp_v1_dev_data_##n, ctx),                              \
		SPI_CONTEXT_INIT_SYNC(spi_mchp_v1_dev_data_##n, ctx),                              \
		SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(n), ctx)};                             \
	DEVICE_DT_INST_DEFINE(n, spi_mchp_v1_init, NULL, &spi_mchp_v1_dev_data_##n,                \
			      &spi_mchp_v1_config_##n, POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,      \
			      &spi_mchp_v1_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SPI_MCHP_V1_DEVICE_INIT)

// PINCTRL_DT_INST_DEFINE(n);       define pin ctrl from dts, configures cs,mosi,miso,sck
// SPI_MCHP_V1_DEFINE_CONFIG(n);    config struct for spi, init hardware specific values like reg,
// clock, pads
