#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/video.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>

LOG_MODULE_REGISTER(main);

/* USB-UART device */
const static struct device *console_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

/* Arducam camera device */
const static struct device *video_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_camera));

/* mnist grayscale image buffer */
static uint8_t tflite_buf[28 * 28];

#define SRC_W (160)
#define SRC_H (120)
#define DST_W (28)
#define DST_H (28)

/* Setup USB-UART console */
static int enable_usb_uart(void)
{
	uint32_t dtr = 0;
	if (usb_enable(NULL)) {
		return -ENODEV;
	}

	while (!dtr) {
		uart_line_ctrl_get(console_dev, UART_LINE_CTRL_DTR, &dtr);
		k_sleep(K_MSEC(100));
	}

	LOG_INF("Logger initialized");
	return 0;
}

/* Converts RGB565 160x120 buffer into 28x28 0-255 Grayscale image */
static void rgb565_to_f32(const uint8_t *src, uint8_t *dst)
{
	const float sx = (float)SRC_W / DST_W;
	const float sy = (float)SRC_H / DST_H;

	for (int y = 0; y < DST_H; ++y) {
		const int ys = (int)(y * sy * 0.5f);
		const uint16_t *sp = (const uint16_t *)src + ys * SRC_W;

		for (int x = 0; x < DST_H; ++x) {
			const int xs = (int)(x * sx * 0.5f);
			const uint16_t pxl = sp[xs];

			uint8_t r5 = (pxl >> 11) & 0x1F;
			uint8_t g6 = (pxl >> 5) & 0x3F;
			uint8_t b5 = (pxl >> 11) & 0x1F;

			uint8_t r = (r5 << 3) | (r5 >> 2);
			uint8_t g = (g6 << 2) | (g6 >> 4);
			uint8_t b = (b5 << 3) | (b5 >> 2);

			uint8_t gray = (uint8_t)((38 * r + 75 * g + 15 * b) >> 7);

			dst[y * DST_W + x] = gray;
		}
	}
}

int main(void)
{
	struct video_buffer *vbuf;
	struct video_format fmt;
	enable_usb_uart();

	if (!device_is_ready(video_dev)) {
		LOG_ERR("%s: video device is not ready", video_dev->name);
	}

	if (video_get_format(video_dev, VIDEO_EP_OUT, &fmt)) {
		LOG_ERR("Unable to retrieve video format");
		return -EIO;
	}

	LOG_INF("Pitch: %u, height: %u", fmt.pitch, fmt.height);
	vbuf = video_buffer_aligned_alloc(fmt.pitch * fmt.height + 4,
					  CONFIG_VIDEO_BUFFER_POOL_ALIGN, K_FOREVER);
	if (!vbuf) {
		LOG_ERR("Failed to allocate video buffer");
		return -ENOMEM;
	}

	for (;;) {
		/* Set target video buffer */
		video_enqueue(video_dev, VIDEO_EP_OUT, vbuf);

		/* Start frame capture */
		video_stream_start(video_dev);

		/* Wait for capture completion and retake ownership of video buffer */
		if (video_dequeue(video_dev, VIDEO_EP_OUT, &vbuf, K_MSEC(700))) {
			LOG_ERR("Failed to capture frame");
			return -EIO;
		}

		LOG_INF("Got frame! size: %u; timestamp %u ms", vbuf->bytesused, vbuf->timestamp);

		/* Skip 1st empty byte */
		rgb565_to_f32(&vbuf->buffer[1], tflite_buf);

		for (int i = 0; i < 28 * 28; ++i) {
			printk("%02X ", tflite_buf[i]);
		}
		printk("\n");

		/* Don't spam the console */
		k_sleep(K_SECONDS(1));
	}
}
