#ifndef _DISPLAY_H
#define _DISPLAY_H

#include <stdint.h>
#include <zephyr/devicetree.h>

#include "tflite.hpp"

#define DISPLAY_NODE DT_CHOSEN(zephyr_display)

int setup_display();

int display_prediction(const uint8_t *mnist, const struct prediction *pred);

#endif // _DISPLAY_H
