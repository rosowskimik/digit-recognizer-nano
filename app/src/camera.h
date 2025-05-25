#ifndef _CAMERA_H
#define _CAMERA_H

#include <zephyr/drivers/video.h>
#include <zephyr/sys_clock.h>
#include <zephyr/kernel.h>

#define FRAME_WIDTH     160
#define FRAME_HEIGHT    120
#define FRAME_SIZE      (FRAME_WIDTH * FRAME_HEIGHT * 2 + 4)
#define CAPTURE_TIMEOUT (K_MSEC(700))

int setup_camera();

int capture_frame(struct video_buffer *frame_buffer, k_timeout_t timeout);

#endif // _CAMERA_H
