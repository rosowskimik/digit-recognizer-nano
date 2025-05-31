#include <stdint.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include "camera.h"
#include "input.h"

void prepare_mnist_input(const uint8_t *rgb565, uint8_t *mnist)
{
	for (uint8_t oy = 0; oy < MNIST_HEIGHT; ++oy) {
		for (uint8_t ox = 0; ox < MNIST_WIDTH; ++ox) {
			/* Rotate 90 degrees right */
			int iy = FRAME_HEIGHT - 1 - ((ox * FRAME_HEIGHT) / MNIST_WIDTH);
			iy = CLAMP(iy, 0, FRAME_HEIGHT - 1);

			int ix = (oy * FRAME_WIDTH) / MNIST_HEIGHT;
			ix = CLAMP(ix, 0, FRAME_WIDTH - 1);

			// Get the 16-bit RGB565 pixel from the input buffer at (ix, iy)
			const uint8_t *pxl_ptr = rgb565 + (iy * FRAME_WIDTH + ix) * 2;
			uint16_t pxl = sys_be16_to_cpu(*(const uint16_t *)pxl_ptr);

			uint8_t r5 = (pxl >> 11) & 0x1F;
			uint8_t g6 = (pxl >> 5) & 0x3F;
			uint8_t b5 = pxl & 0x1F;

			uint8_t r = (r5 << 3) | (r5 >> 2);
			uint8_t g = (g6 << 2) | (g6 >> 4);
			uint8_t b = (b5 << 3) | (b5 >> 2);

			uint8_t gray = (uint8_t)((38 * r + 75 * g + 15 * b) >> 7);

			gray = gray >= 95 ? 0 : 255;

			mnist[oy * MNIST_WIDTH + ox] = gray;
		}
	}
}
