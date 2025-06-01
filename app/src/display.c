#include <stdbool.h>
#include <stdint.h>

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/display.h>
#include <zephyr/display/cfb.h>

#include "input.h"

#include "display.h"

LOG_MODULE_REGISTER(display);

#define DISPLAY_WIDTH  DT_PROP(DISPLAY_NODE, width)
#define DISPLAY_HEIGHT DT_PROP(DISPLAY_NODE, height)

#define BORDER_WIDTH  (MNIST_WIDTH + 2)
#define BORDER_HEIGHT (MNIST_HEIGHT + 2)

/* clang-format off */
#define CFB_POSITION(a, b) (struct cfb_position){.x = (a), .y = (b)}
/* clang-format on */

const static struct device *dev = DEVICE_DT_GET(DISPLAY_NODE);

static uint8_t font_height;

static int draw_border()
{
	int ret = 0;
	if ((ret = cfb_draw_line(dev, &CFB_POSITION(0, 0), &CFB_POSITION(BORDER_WIDTH, 0)))) {
		goto _border_exit;
	}
	if ((ret = cfb_draw_line(dev, &CFB_POSITION(0, BORDER_HEIGHT),
				 &CFB_POSITION(BORDER_WIDTH, BORDER_HEIGHT)))) {
		goto _border_exit;
	}
	if ((ret = cfb_draw_line(dev, &CFB_POSITION(0, 0), &CFB_POSITION(0, BORDER_HEIGHT)))) {
		goto _border_exit;
	}
	if ((ret = cfb_draw_line(dev, &CFB_POSITION(BORDER_WIDTH, 0),
				 &CFB_POSITION(BORDER_WIDTH, BORDER_HEIGHT)))) {
		goto _border_exit;
	}

	if (ret) {
		LOG_ERR("Failed to draw border");
	}

_border_exit:
	return ret;
}

static int draw_image(const uint8_t *mnist)
{
	for (uint8_t y = 0; y < MNIST_HEIGHT; ++y) {
		for (uint8_t x = 0; x < MNIST_WIDTH; ++x) {
			if (mnist[y * MNIST_WIDTH + x]) {
				if (cfb_draw_point(dev, &CFB_POSITION(x + 1, y + 1))) {
					LOG_ERR("Failed to draw mnist image");
					return -EIO;
				}
			}
		}
	}

	return 0;
}

int setup_display()
{
	uint8_t font_width;
	int ret = 0;

	if (!device_is_ready(dev)) {
		LOG_ERR("%s: display device is not ready", dev->name);
		ret = -ENODEV;
		goto _exit_setup;
	}

	if ((ret = display_set_pixel_format(dev, PIXEL_FORMAT_MONO01))) {
		LOG_ERR("Failed to set pixel format");
		goto _exit_setup;
	}

	if ((ret = cfb_framebuffer_init(dev))) {
		LOG_ERR("Framebuffer initialization failed");
		goto _exit_setup;
	}

	if ((ret = cfb_framebuffer_clear(dev, true))) {
		LOG_ERR("Failed to clear framebuffer");
		goto _exit_setup;
	}

	if ((ret = display_blanking_off(dev))) {
		LOG_ERR("Failed to disable display blanking");
		goto _exit_setup;
	}

	if ((ret = cfb_get_font_size(dev, 0, &font_width, &font_height))) {
		LOG_ERR("Failed to get display font size");
		goto _exit_setup;
	}

	if ((ret = cfb_framebuffer_set_font(dev, 0))) {
		LOG_ERR("Failed to set font size");
		goto _exit_setup;
	}

	LOG_DBG("Font height: %u", font_height);

	if ((ret = cfb_set_kerning(dev, 3))) {
		LOG_ERR("Failed to set font kerning");
		goto _exit_setup;
	}

_exit_setup:
	return ret;
}

int display_prediction(const uint8_t *mnist, const struct prediction *pred)
{
	int ret = 0;

	if ((ret = cfb_framebuffer_clear(dev, false))) {
		LOG_ERR("Failed to clear framebuffer");
		goto _exit_pred;
	}

	if ((ret = draw_border())) {
		goto _exit_pred;
	}

	if ((ret = draw_image(mnist))) {
		goto _exit_pred;
	}

	if ((ret = cfb_framebuffer_finalize(dev))) {
		LOG_ERR("Failed to draw framebuffer");
		goto _exit_pred;
	}

_exit_pred:
	return ret;
}
