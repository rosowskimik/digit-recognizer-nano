#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/video.h>

#include "camera.h"

LOG_MODULE_REGISTER(camera);

/* Arducam camera device */
const static struct device *dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_camera));

int setup_camera()
{
	struct video_format fmt;

	if (!device_is_ready(dev)) {
		LOG_ERR("%s: video device is not ready", dev->name);
		return -ENODEV;
	}

	video_get_format(dev, VIDEO_EP_OUT, &fmt);

	LOG_DBG("width: %u, height: %u, pitch: %u", fmt.width, fmt.height, fmt.pitch);

	return 0;
}

int capture_frame(struct video_buffer *frame_buffer, k_timeout_t timeout)
{
	int err = 0;

	if (!frame_buffer) {
		LOG_ERR("Invalid frame buffer");
		return -EINVAL;
	}

	if ((err = video_enqueue(dev, VIDEO_EP_OUT, frame_buffer))) {
		LOG_ERR("Failed to queue frame capture (%d)", err);
		return err;
	}

	if ((err = video_stream_start(dev))) {
		LOG_ERR("Failed to start frame capture (%d)", err);
		return err;
	}

	if ((err = video_dequeue(dev, VIDEO_EP_OUT, &frame_buffer, timeout))) {
		LOG_ERR("Failed to dequeue captured frame (%d)", err);
		return err;
	}

	return 0;
}
