#ifndef _TFLITE_HPP
#define _TFLITE_HPP

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct prediction {
	uint8_t value;
	uint8_t confidence;
};

int setup_tflite();

uint8_t *get_input_buffer();

int run_prediction(struct prediction *pred);

#ifdef __cplusplus
}
#endif

#endif //_TFLITE_HPP
