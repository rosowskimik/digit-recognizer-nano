#ifndef _INPUT_H
#define _INPUT_H

#include <stdint.h>

#define MNIST_WIDTH  28
#define MNIST_HEIGHT 28

void prepare_mnist_input(const uint8_t *rgb565, uint8_t *mnist);

#endif // _INPUT_H
