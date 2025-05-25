#include "camera.h"
#include "input.h"

void prepare_mnist_input(const uint8_t *rgb565, uint8_t *mnist)
{
	const float sx = (float)FRAME_WIDTH / MNIST_WIDTH;
	const float sy = (float)FRAME_HEIGHT / MNIST_HEIGHT;

	for (int y = 0; y < MNIST_HEIGHT; ++y) {
		const int ys = (int)(y * sy * 0.5f);
		const uint16_t *sp = (const uint16_t *)rgb565 + ys * FRAME_WIDTH;

		for (int x = 0; x < MNIST_HEIGHT; ++x) {
			const int xs = (int)(x * sx * 0.5f);
			const uint16_t pxl = sp[xs];

			uint8_t r5 = (pxl >> 11) & 0x1F;
			uint8_t g6 = (pxl >> 5) & 0x3F;
			uint8_t b5 = (pxl >> 11) & 0x1F;

			uint8_t r = (r5 << 3) | (r5 >> 2);
			uint8_t g = (g6 << 2) | (g6 >> 4);
			uint8_t b = (b5 << 3) | (b5 >> 2);

			uint8_t gray = (uint8_t)((38 * r + 75 * g + 15 * b) >> 7);

			mnist[y * MNIST_WIDTH + x] = (255 - gray);
		}
	}
}
