/*
 * Copyright (c) 2021 Antmicro <www.antmicro.com>
 * Copyright (c) 2025 Mikołaj Rosowski
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Derived from ov2640 I2C driver */

#define DT_DRV_COMPAT arducam_ov2640

#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel/thread.h>
#include <zephyr/kernel/thread_stack.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/video.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/toolchain.h>

LOG_MODULE_REGISTER(arducam, CONFIG_VIDEO_LOG_LEVEL);

/* DSP register bank FF=0x00*/
#define QS     0x44
#define HSIZE  0x51
#define VSIZE  0x52
#define XOFFL  0x53
#define YOFFL  0x54
#define VHYX   0x55
#define TEST   0x57
#define ZMOW   0x5A
#define ZMOH   0x5B
#define ZMHH   0x5C
#define BPADDR 0x7C
#define BPDATA 0x7D
#define SIZEL  0x8C
#define HSIZE8 0xC0
#define VSIZE8 0xC1
#define CTRL1  0xC3

#define CTRLI       0x50
#define CTRLI_LP_DP 0x80

#define CTRL0        0xC2
#define CTRL0_YUV422 0x08
#define CTRL0_YUV_EN 0x04
#define CTRL0_RGB_EN 0x02

#define CTRL2           0x86
#define CTRL2_DCW_EN    0x20
#define CTRL2_SDE_EN    0x10
#define CTRL2_UV_ADJ_EN 0x08
#define CTRL2_UV_AVG_EN 0x04
#define CTRL2_CMX_EN    0x01

#define CTRL3              0x87
#define CTRL3_BPC_EN       0x80
#define CTRL3_WPC_EN       0x40
#define R_DVP_SP           0xD3
#define R_DVP_SP_AUTO_MODE 0x80

#define R_BYPASS           0x05
#define R_BYPASS_DSP_EN    0x00
#define R_BYPASS_DSP_BYPAS 0x01

#define IMAGE_MODE         0xDA
#define IMAGE_MODE_JPEG_EN 0x10
#define IMAGE_MODE_RGB565  0x08

#define RESET      0xE0
#define RESET_JPEG 0x10
#define RESET_DVP  0x04

#define MC_BIST              0xF9
#define MC_BIST_RESET        0x80
#define MC_BIST_BOOT_ROM_SEL 0x40

#define BANK_SEL        0xFF
#define BANK_SEL_DSP    0x00
#define BANK_SEL_SENSOR 0x01

/* Sensor register bank FF=0x01*/
#define COM1        0x03
#define REG_PID     0x0A
#define REG_PID_VAL 0x26
#define REG_VER     0x0B
#define REG_VER_VAL 0x42
#define AEC         0x10
#define CLKRC       0x11
#define COM10       0x15
#define HSTART      0x17
#define HSTOP       0x18
#define VSTART      0x19
#define VSTOP       0x1A
#define AEW         0x24
#define AEB         0x25
#define ARCOM2      0x34
#define FLL         0x46
#define FLH         0x47
#define COM19       0x48
#define ZOOMS       0x49
#define BD50        0x4F
#define BD60        0x50
#define REG5D       0x5D
#define REG5E       0x5E
#define REG5F       0x5F
#define REG60       0x60
#define HISTO_LOW   0x61
#define HISTO_HIGH  0x62

#define REG04           0x04
#define REG04_DEFAULT   0x28
#define REG04_HFLIP_IMG 0x80
#define REG04_VFLIP_IMG 0x40
#define REG04_VREF_EN   0x10
#define REG04_HREF_EN   0x08
#define REG04_SET(x)    (REG04_DEFAULT | x)

#define COM2              0x09
#define COM2_OUT_DRIVE_3x 0x02

#define COM3             0x0C
#define COM3_DEFAULT     0x38
#define COM3_BAND_AUTO   0x02
#define COM3_BAND_SET(x) (COM3_DEFAULT | x)

#define COM7           0x12
#define COM7_SRST      0x80
#define COM7_RES_UXGA  0x00 /* UXGA */
#define COM7_ZOOM_EN   0x04 /* Enable Zoom */
#define COM7_COLOR_BAR 0x02 /* Enable Color Bar Test */

#define COM8         0x13
#define COM8_DEFAULT 0xC0
#define COM8_BNDF_EN 0x20 /* Enable Banding filter */
#define COM8_AGC_EN  0x04 /* AGC Auto/Manual control selection */
#define COM8_AEC_EN  0x01 /* Auto/Manual Exposure control */
#define COM8_SET(x)  (COM8_DEFAULT | x)

#define COM9             0x14 /* AGC gain ceiling */
#define COM9_DEFAULT     0x08
#define COM9_AGC_GAIN_8x 0x02 /* AGC:    8x */
#define COM9_AGC_SET(x)  (COM9_DEFAULT | (x << 5))

#define COM10 0x15

#define CTRL1_AWB 0x08 /* Enable AWB */

#define VV                  0x26
#define VV_AGC_TH_SET(h, l) ((h << 4) | (l & 0x0F))

#define REG32      0x32
#define REG32_UXGA 0x36

/* Configuration arrays */
#define QQVGA_HSIZE (160)
#define QQVGA_VSIZE (120)

#define SVGA_HSIZE (800)
#define SVGA_VSIZE (600)

#define UXGA_HSIZE (1600)
#define UXGA_VSIZE (1200)

#define QQVGA_RGB565_BUFSIZE (QQVGA_HSIZE * QQVGA_VSIZE * 2)

/* Arducam SPI interface */
#define ARDUCHIP_TEST1      0x00 /* constant 0x55 when silicon OK */
#define ARDUCHIP_FIFO       0x04
#define FIFO_CLEAR_MASK     0x01
#define FIFO_START_MASK     0x02
#define FIFO_RDPTR_RST_MASK 0x10
#define FIFO_WRPTR_RST_MASK 0x20
#define ARDUCHIP_TRIG       0x41
#define CAP_DONE_MASK       0x08
#define BURST_FIFO_READ     0x3C

#define ARDUCHIP_ID 0x55

#define FIFO_SIZE1 0x42
#define FIFO_SIZE2 0x43
#define FIFO_SIZE3 0x44

struct arducam_config {
	struct i2c_dt_spec i2c; /* Config endpoint */
	struct spi_dt_spec spi; /* Data endpoint */
	uint8_t clock_rate_control;
};

struct arducam_data {
	struct video_format fmt;
	struct video_buffer *buf_in_flight;
	struct k_sem sem_buf_in;
	struct k_sem sem_buf_out;
	struct k_thread thread;

	K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_ARDUCAM_CAPTURE_THREAD_STACK_SIZE);
};

struct arducam_reg {
	uint8_t addr;
	uint8_t value;
};

static const struct arducam_reg default_regs[] = {
	{BANK_SEL, BANK_SEL_DSP},
	{0x2c, 0xff},
	{0x2e, 0xdf},
	{BANK_SEL, BANK_SEL_SENSOR},
	{0x3c, 0x32},
	{CLKRC, 0x80},             /* Set PCLK divider */
	{COM2, COM2_OUT_DRIVE_3x}, /* Output drive x2 */
	{REG04, REG04_SET(REG04_HREF_EN)},
	{COM8, COM8_SET(COM8_BNDF_EN | COM8_AGC_EN | COM8_AEC_EN)},
	{COM9, COM9_AGC_SET(COM9_AGC_GAIN_8x)},
	{COM10, 0x00}, /* Invert VSYNC */
	{0x2c, 0x0c},
	{0x33, 0x78},
	{0x3a, 0x33},
	{0x3b, 0xfb},
	{0x3e, 0x00},
	{0x43, 0x11},
	{0x16, 0x10},
	{0x39, 0x02},
	{0x35, 0x88},
	{0x22, 0x0a},
	{0x37, 0x40},
	{0x23, 0x00},
	{ARCOM2, 0xa0},
	{0x06, 0x02},
	{0x06, 0x88},
	{0x07, 0xc0},
	{0x0d, 0xb7},
	{0x0e, 0x01},
	{0x4c, 0x00},
	{0x4a, 0x81},
	{0x21, 0x99},
	{AEW, 0x40},
	{AEB, 0x38},
	/* AGC/AEC fast mode operating region */
	{VV, VV_AGC_TH_SET(0x08, 0x02)},
	{COM19, 0x00}, /* Zoom control 2 LSBs */
	{ZOOMS, 0x00}, /* Zoom control 8 MSBs */
	{0x5c, 0x00},
	{0x63, 0x00},
	{FLL, 0x00},
	{FLH, 0x00},

	/* Set banding filter */
	{COM3, COM3_BAND_SET(COM3_BAND_AUTO)},
	{REG5D, 0x55},
	{REG5E, 0x7d},
	{REG5F, 0x7d},
	{REG60, 0x55},
	{HISTO_LOW, 0x70},
	{HISTO_HIGH, 0x80},
	{0x7c, 0x05},
	{0x20, 0x80},
	{0x28, 0x30},
	{0x6c, 0x00},
	{0x6d, 0x80},
	{0x6e, 0x00},
	{0x70, 0x02},
	{0x71, 0x94},
	{0x73, 0xc1},
	{0x3d, 0x34},
	/* { COM7,   COM7_RES_UXGA | COM7_ZOOM_EN }, */
	{0x5a, 0x57},
	{BD50, 0xbb},
	{BD60, 0x9c},

	{BANK_SEL, BANK_SEL_DSP},
	{0xe5, 0x7f},
	{MC_BIST, MC_BIST_RESET | MC_BIST_BOOT_ROM_SEL},
	{0x41, 0x24},
	{RESET, RESET_JPEG | RESET_DVP},
	{0x76, 0xff},
	{0x33, 0xa0},
	{0x42, 0x20},
	{0x43, 0x18},
	{0x4c, 0x00},
	{CTRL3, CTRL3_BPC_EN | CTRL3_WPC_EN | 0x10},
	{0x88, 0x3f},
	{0xd7, 0x03},
	{0xd9, 0x10},
	{R_DVP_SP, R_DVP_SP_AUTO_MODE | 0x2},
	{0xc8, 0x08},
	{0xc9, 0x80},
	{BPADDR, 0x00},
	{BPDATA, 0x00},
	{BPADDR, 0x03},
	{BPDATA, 0x48},
	{BPDATA, 0x48},
	{BPADDR, 0x08},
	{BPDATA, 0x20},
	{BPDATA, 0x10},
	{BPDATA, 0x0e},
	{0x90, 0x00},
	{0x91, 0x0e},
	{0x91, 0x1a},
	{0x91, 0x31},
	{0x91, 0x5a},
	{0x91, 0x69},
	{0x91, 0x75},
	{0x91, 0x7e},
	{0x91, 0x88},
	{0x91, 0x8f},
	{0x91, 0x96},
	{0x91, 0xa3},
	{0x91, 0xaf},
	{0x91, 0xc4},
	{0x91, 0xd7},
	{0x91, 0xe8},
	{0x91, 0x20},
	{0x92, 0x00},
	{0x93, 0x06},
	{0x93, 0xe3},
	{0x93, 0x03},
	{0x93, 0x03},
	{0x93, 0x00},
	{0x93, 0x02},
	{0x93, 0x00},
	{0x93, 0x00},
	{0x93, 0x00},
	{0x93, 0x00},
	{0x93, 0x00},
	{0x93, 0x00},
	{0x93, 0x00},
	{0x96, 0x00},
	{0x97, 0x08},
	{0x97, 0x19},
	{0x97, 0x02},
	{0x97, 0x0c},
	{0x97, 0x24},
	{0x97, 0x30},
	{0x97, 0x28},
	{0x97, 0x26},
	{0x97, 0x02},
	{0x97, 0x98},
	{0x97, 0x80},
	{0x97, 0x00},
	{0x97, 0x00},
	{0xa4, 0x00},
	{0xa8, 0x00},
	{0xc5, 0x11},
	{0xc6, 0x51},
	{0xbf, 0x80},
	{0xc7, 0x10},
	{0xb6, 0x66},
	{0xb8, 0xA5},
	{0xb7, 0x64},
	{0xb9, 0x7C},
	{0xb3, 0xaf},
	{0xb4, 0x97},
	{0xb5, 0xFF},
	{0xb0, 0xC5},
	{0xb1, 0x94},
	{0xb2, 0x0f},
	{0xc4, 0x5c},
	{0xa6, 0x00},
	{0xa7, 0x20},
	{0xa7, 0xd8},
	{0xa7, 0x1b},
	{0xa7, 0x31},
	{0xa7, 0x00},
	{0xa7, 0x18},
	{0xa7, 0x20},
	{0xa7, 0xd8},
	{0xa7, 0x19},
	{0xa7, 0x31},
	{0xa7, 0x00},
	{0xa7, 0x18},
	{0xa7, 0x20},
	{0xa7, 0xd8},
	{0xa7, 0x19},
	{0xa7, 0x31},
	{0xa7, 0x00},
	{0xa7, 0x18},
	{0x7f, 0x00},
	{0xe5, 0x1f},
	{0xe1, 0x77},
	{0xdd, 0x7f},
	{CTRL0, CTRL0_YUV422 | CTRL0_YUV_EN | CTRL0_RGB_EN},
	{0x00, 0x00},
};

static const struct arducam_reg uxga_regs[] = {
	{BANK_SEL, BANK_SEL_SENSOR},
	/* DSP input image resolution and window size control */
	{COM7, COM7_RES_UXGA},
	{COM1, 0x0F},        /* UXGA=0x0F, SVGA=0x0A, CIF=0x06 */
	{REG32, REG32_UXGA}, /* UXGA=0x36, SVGA/CIF=0x09 */

	{HSTART, 0x11}, /* UXGA=0x11, SVGA/CIF=0x11 */
	{HSTOP, 0x75},  /* UXGA=0x75, SVGA/CIF=0x43 */

	{VSTART, 0x01}, /* UXGA=0x01, SVGA/CIF=0x00 */
	{VSTOP, 0x97},  /* UXGA=0x97, SVGA/CIF=0x4b */
	{0x3d, 0x34},   /* UXGA=0x34, SVGA/CIF=0x38 */

	{0x35, 0x88},
	{0x22, 0x0a},
	{0x37, 0x40},
	{0x34, 0xa0},
	{0x06, 0x02},
	{0x0d, 0xb7},
	{0x0e, 0x01},
	{0x42, 0x83},

	/*
	 * Set DSP input image size and offset.
	 * The sensor output image can be scaled with OUTW/OUTH
	 */
	{BANK_SEL, BANK_SEL_DSP},
	{R_BYPASS, R_BYPASS_DSP_BYPAS},

	{RESET, RESET_DVP},
	{HSIZE8, (UXGA_HSIZE >> 3)}, /* Image Horizontal Size HSIZE[10:3] */
	{VSIZE8, (UXGA_VSIZE >> 3)}, /* Image Vertical Size VSIZE[10:3] */

	/* {HSIZE[11], HSIZE[2:0], VSIZE[2:0]} */
	{SIZEL, ((UXGA_HSIZE >> 6) & 0x40) | ((UXGA_HSIZE & 0x7) << 3) | (UXGA_VSIZE & 0x7)},

	{XOFFL, 0x00},                       /* OFFSET_X[7:0] */
	{YOFFL, 0x00},                       /* OFFSET_Y[7:0] */
	{HSIZE, ((UXGA_HSIZE >> 2) & 0xFF)}, /* H_SIZE[7:0] real/4 */
	{VSIZE, ((UXGA_VSIZE >> 2) & 0xFF)}, /* V_SIZE[7:0] real/4 */

	/* V_SIZE[8]/OFFSET_Y[10:8]/H_SIZE[8]/OFFSET_X[10:8] */
	{VHYX, ((UXGA_VSIZE >> 3) & 0x80) | ((UXGA_HSIZE >> 7) & 0x08)},
	{TEST, (UXGA_HSIZE >> 4) & 0x80}, /* H_SIZE[9] */

	{CTRL2, CTRL2_DCW_EN | CTRL2_SDE_EN | CTRL2_UV_AVG_EN | CTRL2_CMX_EN | CTRL2_UV_ADJ_EN},

	/* H_DIVIDER/V_DIVIDER */
	{CTRLI, CTRLI_LP_DP | 0x00},
	/* DVP prescaler */
	{R_DVP_SP, R_DVP_SP_AUTO_MODE | 0x04},

	{R_BYPASS, R_BYPASS_DSP_EN},
	{RESET, 0x00},
	{0, 0},
};

#define ARDUCAM_VIDEO_FORMAT_CAP(width, height, format)                                            \
	{                                                                                          \
		.pixelformat = (format),                                                           \
		.width_min = (width),                                                              \
		.width_max = (width),                                                              \
		.height_min = (width),                                                             \
		.height_max = (width),                                                             \
		.width_step = 0,                                                                   \
		.height_step = 0,                                                                  \
	}

static const struct video_format_cap fmts[] = {
	ARDUCAM_VIDEO_FORMAT_CAP(160, 120, VIDEO_PIX_FMT_RGB565), /* QQVGA */
	{0}};

static int arducam_i2c_write_reg(const struct i2c_dt_spec *spec, uint8_t reg_addr, uint8_t value)
{
	uint8_t tries = 3;

	/**
	 * It rarely happens that the camera does not respond with ACK signal.
	 * In that case it usually responds on 2nd try but there is a 3rd one
	 * just to be sure that the connection error is not caused by driver
	 * itself.
	 */
	while (tries-- > 0) {
		if (!i2c_reg_write_byte_dt(spec, reg_addr, value)) {
			return 0;
		}
		/* If writing failed wait 5ms before next attempt */
		k_msleep(5);
	}
	LOG_ERR("failed to write 0x%x to 0x%x", value, reg_addr);

	return -1;
}

static int arducam_i2c_read_reg(const struct i2c_dt_spec *spec, uint8_t reg_addr)
{
	uint8_t tries = 3;
	uint8_t value;

	/**
	 * It rarely happens that the camera does not respond with ACK signal.
	 * In that case it usually responds on 2nd try but there is a 3rd one
	 * just to be sure that the connection error is not caused by driver
	 * itself.
	 */
	while (tries-- > 0) {
		if (!i2c_reg_read_byte_dt(spec, reg_addr, &value)) {
			return value;
		}
		/* If reading failed wait 5ms before next attempt */
		k_msleep(5);
	}
	LOG_ERR("failed to read 0x%x register", reg_addr);

	return -1;
}

static int arducam_i2c_write_all(const struct device *dev, const struct arducam_reg *regs,
				 uint16_t reg_num)
{
	uint16_t i = 0;
	const struct arducam_config *cfg = dev->config;

	for (i = 0; i < reg_num; i++) {
		int err;

		err = arducam_i2c_write_reg(&cfg->i2c, regs[i].addr, regs[i].value);
		if (err) {
			return err;
		}
	}

	return 0;
}

static int arducam_spi_read(const struct device *dev, uint8_t reg, uint8_t *val)
{
	const struct arducam_config *cfg = dev->config;

	uint8_t tx = reg & 0x7F;

	struct spi_buf tx_buf = {.buf = &tx, .len = 1};
	struct spi_buf rx_bufs[3] = {
		{.buf = &tx, .len = 1}, /* throw‑away */
		{.buf = val, .len = 1},
	};
	struct spi_buf_set txs = {.buffers = &tx_buf, .count = 1};
	struct spi_buf_set rxs = {.buffers = rx_bufs, .count = 2};

	return spi_transceive_dt(&cfg->spi, &txs, &rxs);
}

static inline int arducam_spi_write(const struct device *dev, uint8_t reg, uint8_t val)
{
	const struct arducam_config *cfg = dev->config;

	uint8_t tx[2] = {(reg | 0x80), val};

	struct spi_buf buf = {.buf = tx, .len = 2};
	struct spi_buf_set txs = {.buffers = &buf, .count = 1};

	return spi_write_dt(&cfg->spi, &txs);
}

static int arducam_spi_burst_read(const struct device *dev, uint8_t *dst, size_t len)
{
	const struct arducam_config *cfg = dev->config;

	uint8_t cmd = BURST_FIFO_READ;
	struct spi_buf tx_buf = {.buf = &cmd, .len = 1};
	struct spi_buf rx_bufs[] = {
		{.buf = &cmd, .len = 1},
		{.buf = &cmd, .len = 1},
		{.buf = dst, .len = QQVGA_RGB565_BUFSIZE},
	};
	struct spi_buf_set txs = {.buffers = &tx_buf, .count = 1};
	struct spi_buf_set rxs = {.buffers = rx_bufs, .count = 3};

	return spi_transceive_dt(&cfg->spi, &txs, &rxs);
}

static int arducam_soft_reset(const struct device *dev)
{
	const struct arducam_config *cfg = dev->config;
	int ret = 0;

	/* Switch to DSP register bank */
	ret |= arducam_i2c_write_reg(&cfg->i2c, BANK_SEL, BANK_SEL_SENSOR);

	/* Initiate system reset */
	ret |= arducam_i2c_write_reg(&cfg->i2c, COM7, COM7_SRST);

	return ret;
}

static int arducam_set_output_format(const struct device *dev, int output_format)
{
	int ret = 0;
	const struct arducam_config *cfg = dev->config;

	/* Switch to DSP register bank */
	ret |= arducam_i2c_write_reg(&cfg->i2c, BANK_SEL, BANK_SEL_DSP);

	if (output_format != VIDEO_PIX_FMT_RGB565) {
		LOG_ERR("Image format not supported");
		return -ENOTSUP;
	}

	arducam_i2c_write_reg(&cfg->i2c, IMAGE_MODE, IMAGE_MODE_RGB565);
	k_msleep(30);

	return ret;
}

static int arducam_set_resolution(const struct device *dev, uint16_t img_width, uint16_t img_height)
{
	int ret = 0;
	const struct arducam_config *cfg = dev->config;

	uint16_t w = img_width;
	uint16_t h = img_height;

	/* Disable DSP */
	ret |= arducam_i2c_write_reg(&cfg->i2c, BANK_SEL, BANK_SEL_DSP);
	ret |= arducam_i2c_write_reg(&cfg->i2c, R_BYPASS, R_BYPASS_DSP_BYPAS);

	/* Write output width */
	ret |= arducam_i2c_write_reg(&cfg->i2c, ZMOW, (w >> 2) & 0xFF); /* OUTW[7:0] (real/4) */
	ret |= arducam_i2c_write_reg(&cfg->i2c, ZMOH, (h >> 2) & 0xFF); /* OUTH[7:0] (real/4) */
	ret |= arducam_i2c_write_reg(
		&cfg->i2c, ZMHH, ((h >> 8) & 0x04) | ((w >> 10) & 0x03)); /* OUTH[8]/OUTW[9:8] */

	/* Set CLKRC */
	ret |= arducam_i2c_write_reg(&cfg->i2c, BANK_SEL, BANK_SEL_SENSOR);
	ret |= arducam_i2c_write_reg(&cfg->i2c, CLKRC, cfg->clock_rate_control);

	/* Write DSP input registers */
	arducam_i2c_write_all(dev, uxga_regs, ARRAY_SIZE(uxga_regs));

	/* Enable DSP */
	ret |= arducam_i2c_write_reg(&cfg->i2c, BANK_SEL, BANK_SEL_DSP);
	ret |= arducam_i2c_write_reg(&cfg->i2c, R_BYPASS, R_BYPASS_DSP_EN);

	k_msleep(30);

	return ret;
}

int arducam_check_connection(const struct device *dev)
{
	int ret = 0;
	const struct arducam_config *cfg = dev->config;

	uint8_t reg_pid_val, reg_ver_val, reg_id_val;

	ret |= arducam_i2c_write_reg(&cfg->i2c, BANK_SEL, BANK_SEL_SENSOR);
	reg_pid_val = arducam_i2c_read_reg(&cfg->i2c, REG_PID);
	reg_ver_val = arducam_i2c_read_reg(&cfg->i2c, REG_VER);

	if (REG_PID_VAL != reg_pid_val || REG_VER_VAL != reg_ver_val) {
		LOG_ERR("Arducam configuration endpoint not detected\n");
		return -ENODEV;
	}

	ret = arducam_spi_write(dev, ARDUCHIP_TEST1, ARDUCHIP_ID);
	if (ret) {
		LOG_ERR("Write failed %u", -ret);
	}

	if (arducam_spi_read(dev, ARDUCHIP_TEST1, &reg_id_val) || reg_id_val != ARDUCHIP_ID) {
		LOG_ERR("Arducam data endpoint not responding");
		return -ENODEV;
	}

	return ret;
}

static int arducam_set_exposure_ctrl(const struct device *dev, int enable)
{
	int ret = 0;
	const struct arducam_config *cfg = dev->config;

	uint8_t reg;

	/* Switch to SENSOR register bank */
	ret |= arducam_i2c_write_reg(&cfg->i2c, BANK_SEL, BANK_SEL_SENSOR);

	/* Update COM8  to enable/disable automatic exposure control */
	reg = arducam_i2c_read_reg(&cfg->i2c, COM8);

	if (enable) {
		reg |= COM8_AEC_EN;
	} else {
		reg &= ~COM8_AEC_EN;
	}

	ret |= arducam_i2c_write_reg(&cfg->i2c, COM8, reg);

	return ret;
}

static int arducam_set_white_bal(const struct device *dev, int enable)
{
	int ret = 0;
	const struct arducam_config *cfg = dev->config;

	uint8_t reg;

	/* Switch to SENSOR register bank */
	ret |= arducam_i2c_write_reg(&cfg->i2c, BANK_SEL, BANK_SEL_SENSOR);

	/* Update CTRL1 to enable/disable automatic white balance*/
	reg = arducam_i2c_read_reg(&cfg->i2c, CTRL1);

	if (enable) {
		reg |= CTRL1_AWB;
	} else {
		reg &= ~CTRL1_AWB;
	}

	ret |= arducam_i2c_write_reg(&cfg->i2c, CTRL1, reg);

	return ret;
}

static int arducam_wait_done(const struct device *dev, k_timeout_t to)
{
	uint8_t trig;
	int64_t deadline = k_uptime_get() + to.ticks;
	do {
		if (arducam_spi_read(dev, ARDUCHIP_TRIG, &trig)) {
			LOG_ERR("Poll ready failed");
			return -EIO;
		}
		if (trig & CAP_DONE_MASK) {
			return 0;
		}
		k_msleep(2);
	} while (k_uptime_get() < deadline);

	LOG_ERR("Capture timeout");
	return -ETIMEDOUT;
}

static int arducam_read_fifo_len(const struct device *dev, uint32_t *len)
{
	uint8_t b1, b2, b3;

	if (arducam_spi_read(dev, FIFO_SIZE1, &b1) || arducam_spi_read(dev, FIFO_SIZE2, &b2) ||
	    arducam_spi_read(dev, FIFO_SIZE3, &b3)) {
		return -EIO;
	}

	*len = (((uint32_t)b3 << 16) | ((uint32_t)b2 << 8) | b1) & 0x07FFFFF;
	return 0;
}

static int arducam_capture_single(const struct device *dev, struct video_buffer *buf)
{
	uint32_t len;
	int ret = 0;

	/* 1. Reset FIFO */
	ret = arducam_spi_write(dev, ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
	if (ret) {
		LOG_ERR("FIFO reset failed");
		return ret;
	}

	/* 2. Trigger 1 frame capture */
	ret = arducam_spi_write(dev, ARDUCHIP_FIFO, FIFO_START_MASK);
	if (ret) {
		LOG_ERR("Capture trigger failed");
		return ret;
	}

	/* 3. Wait for completion */
	ret = arducam_wait_done(dev, K_MSEC(500));
	if (ret) {
		LOG_ERR("Capture finish failed");
		return ret;
	}

	/* 4. Read FIFO length */
	ret = arducam_read_fifo_len(dev, &len);
	if (ret) {
		LOG_ERR("FIFO read failed");
		return ret;
	}
	if (!len) {
		LOG_ERR("FIFO empty");
		return -EIO;
	}
	if (len < QQVGA_RGB565_BUFSIZE) {
		LOG_ERR("FIFO not full");
		return -EIO;
	}

	/* 5. Burst‑read into provided buffer */
	ret = arducam_spi_burst_read(dev, buf->buffer, len);
	if (ret) {
		LOG_ERR("FIFO read failed");
		return ret;
	}

	buf->bytesused = QQVGA_RGB565_BUFSIZE;
	buf->timestamp = k_ticks_to_us_floor64(k_uptime_ticks());

	return ret;
}

static void arducam_capture_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	const struct device *dev = p1;
	struct arducam_data *data = dev->data;
	int err;

	for (;;) {
		if (k_sem_take(&data->sem_buf_in, K_FOREVER)) {
			continue;
		}

		err = arducam_capture_single(dev, data->buf_in_flight);
		if (err) {
			LOG_ERR("Capture failed (%d)", err);
		}

		k_sem_give(&data->sem_buf_out);
	}
}

static int arducam_get_caps(const struct device *dev, enum video_endpoint_id ep,
			    struct video_caps *caps)
{
	caps->format_caps = fmts;
	caps->min_vbuf_count = 1;
	caps->min_line_count = LINE_COUNT_HEIGHT;
	caps->max_line_count = LINE_COUNT_HEIGHT;

	return 0;
}

static int arducam_set_fmt(const struct device *dev, enum video_endpoint_id ep,
			   struct video_format *fmt)
{
	struct arducam_data *drv_data = dev->data;
	int ret = 0;

	if (fmt->pixelformat != VIDEO_PIX_FMT_RGB565 || fmt->width != QQVGA_HSIZE ||
	    fmt->height != QQVGA_VSIZE) {
		LOG_ERR("Arducam camera supports only RGB565 pixelformat");
		return -ENOTSUP;
	}

	if (!memcmp(&drv_data->fmt, fmt, sizeof(drv_data->fmt))) {
		return 0;
	}

	drv_data->fmt = *fmt;

	ret |= arducam_set_output_format(dev, fmt->pixelformat);
	ret |= arducam_set_resolution(dev, fmt->width, fmt->height);

	return ret;
}

static int arducam_get_fmt(const struct device *dev, enum video_endpoint_id ep,
			   struct video_format *fmt)
{
	struct arducam_data *drv_data = dev->data;

	*fmt = drv_data->fmt;

	return 0;
}

static int arducam_set_stream(const struct device *dev, bool enable)
{
	struct arducam_data *drv_data = dev->data;

	if (enable) {
		k_sem_give(&drv_data->sem_buf_in);
	} else {
		k_sem_reset(&drv_data->sem_buf_in);
	}

	return 0;
}

static int arducam_enqueue(const struct device *dev, enum video_endpoint_id ep,
			   struct video_buffer *buf)
{
	ARG_UNUSED(ep);

	struct arducam_data *drv_data = dev->data;
	if (drv_data->buf_in_flight) {
		return -EBUSY;
	}
	drv_data->buf_in_flight = buf;

	return 0;
}

static int arducam_dequeue(const struct device *dev, enum video_endpoint_id ep,
			   struct video_buffer **buf, k_timeout_t timeout)
{
	ARG_UNUSED(ep);

	struct arducam_data *drv_data = dev->data;

	if (k_sem_take(&drv_data->sem_buf_out, timeout)) {
		return -EAGAIN;
	}

	*buf = drv_data->buf_in_flight;
	drv_data->buf_in_flight = NULL;

	return 0;
}

static DEVICE_API(video, arducam_driver_api) = {
	.set_format = arducam_set_fmt,
	.get_format = arducam_get_fmt,
	.get_caps = arducam_get_caps,
	.set_stream = arducam_set_stream,
	.enqueue = arducam_enqueue,
	.dequeue = arducam_dequeue,
};

static int arducam_init(const struct device *dev)
{
	struct video_format fmt;
	k_tid_t tid;
	const struct arducam_config *cfg = dev->config;
	struct arducam_data *data = dev->data;
	int ret = 0;

	if (!i2c_is_ready_dt(&cfg->i2c) || !spi_is_ready_dt(&cfg->spi)) {
		LOG_ERR("Bus device is not ready");
		return -EINVAL;
	}

	ret = arducam_check_connection(dev);
	if (ret) {
		return ret;
	}

	arducam_soft_reset(dev);
	k_sleep(K_MSEC(300));

	arducam_i2c_write_all(dev, default_regs, ARRAY_SIZE(default_regs));

	fmt.pixelformat = VIDEO_PIX_FMT_RGB565;
	fmt.width = QQVGA_HSIZE;
	fmt.height = QQVGA_VSIZE;
	fmt.pitch = QQVGA_HSIZE * 2;
	ret = arducam_set_fmt(dev, VIDEO_EP_OUT, &fmt);
	if (ret) {
		LOG_ERR("Unable to configure default format");
		return -EIO;
	}

	ret |= arducam_set_exposure_ctrl(dev, true);
	ret |= arducam_set_white_bal(dev, true);

	if (ret) {
		return -EIO;
	}

	k_sem_init(&data->sem_buf_in, 0, 1);
	k_sem_init(&data->sem_buf_out, 0, 1);

	tid = k_thread_create(&data->thread, data->thread_stack,
			      K_KERNEL_STACK_SIZEOF(data->thread_stack), arducam_capture_thread,
			      (void *)dev, NULL, NULL, CONFIG_ARDUCAM_CAPTURE_THREAD_PRIO,
			      K_ESSENTIAL, K_NO_WAIT);
	k_thread_name_set(tid, dev->name);

	return ret;
}

static const struct arducam_config arducam_cfg = {
	.i2c =
		{
			.bus = DEVICE_DT_GET(DT_INST_PHANDLE(0, control_bus)),
			.addr = 0x30,
		},
	.spi = SPI_DT_SPEC_INST_GET(0, SPI_OP_MODE_MASTER | SPI_WORD_SET(8), 0),
	.clock_rate_control = DT_INST_PROP(0, clock_rate_control),
};
static struct arducam_data arducam_data;

DEVICE_DT_INST_DEFINE(0, &arducam_init, NULL, &arducam_data, &arducam_cfg, POST_KERNEL,
		      CONFIG_VIDEO_INIT_PRIORITY, &arducam_driver_api);
