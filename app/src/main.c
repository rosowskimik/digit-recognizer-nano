#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/video.h>
#include <zephyr/logging/log.h>

#include "usb.h"
#include "camera.h"
#include "input.h"
#include "tflite.hpp"
#include "display.h"

LOG_MODULE_REGISTER(main);

/* Capture buffer */
static struct video_buffer *vbuf;

/* Pointer to input tensor data */
static uint8_t *mnist;

static uint8_t mnist_cp[28 * 28];

/* Output prediction */
static struct prediction pred;

int main(void)
{
	if (enable_usb_uart()) {
		return 0;
	}

	if (setup_camera()) {
		return 0;
	}

#if DT_NODE_HAS_STATUS(DISPLAY_NODE, okay)
	if (setup_display()) {
		return 0;
	}
#endif

	if (setup_tflite()) {
		return 0;
	}

	vbuf = video_buffer_aligned_alloc(FRAME_SIZE, CONFIG_VIDEO_BUFFER_POOL_ALIGN, K_MSEC(100));
	if (!vbuf) {
		LOG_ERR("Failed to allocate frame buffer");
		return 0;
	}

	mnist = get_input_buffer();
	if (!mnist) {
		LOG_ERR("Failed to get model input pointer");
		return 0;
	}

	for (;;) {
		if (capture_frame(vbuf, CAPTURE_TIMEOUT)) {
			return 0;
		}

		LOG_DBG("Got frame! size: %u; timestamp %u ms", vbuf->bytesused, vbuf->timestamp);

		prepare_mnist_input(vbuf->buffer, mnist);
		memcpy(mnist_cp, mnist, 28 * 28);

		if (run_prediction(&pred)) {
			continue;
		};

#if DT_NODE_HAS_STATUS(DISPLAY_NODE, okay)
		display_prediction(mnist_cp, &pred);
#endif

		LOG_INF("Prediction: %u (confidence: %03u/255)", pred.value, pred.confidence);

		/* Don't spam the console */
		k_sleep(K_SECONDS(1));
	}
}
