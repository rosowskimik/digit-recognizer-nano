#include <tensorflow/lite/micro/micro_interpreter.h>
#include <tensorflow/lite/schema/schema_generated.h>
#include <tensorflow/lite/micro/micro_mutable_op_resolver.h>
#include <tensorflow/lite/c/common.h>
#include <tensorflow/lite/c/c_api_types.h>
#include <stdint.h>
#include <zephyr/logging/log.h>

#include "model.hpp"
#include "tflite.hpp"

LOG_MODULE_REGISTER(tflite);

static constexpr int tensor_arena_size = 20 * 1024;
static uint8_t tensor_arena[tensor_arena_size];

const static tflite::Model *model = nullptr;
static tflite::MicroInterpreter *interpreter = nullptr;
static TfLiteTensor *model_input = nullptr;
static TfLiteTensor *model_output = nullptr;

int setup_tflite()
{
	model = tflite::GetModel(g_model);
	if (model->version() != TFLITE_SCHEMA_VERSION) {
		return -EINVAL;
	}

	static tflite::MicroMutableOpResolver<7> micro_op_resolver;
	micro_op_resolver.AddQuantize();
	micro_op_resolver.AddConv2D();
	micro_op_resolver.AddMaxPool2D();
	micro_op_resolver.AddReshape();
	micro_op_resolver.AddFullyConnected();
	micro_op_resolver.AddSoftmax();
	micro_op_resolver.AddDequantize();

	static tflite::MicroInterpreter static_interpreter(model, micro_op_resolver, tensor_arena,
							   tensor_arena_size);
	interpreter = &static_interpreter;

	interpreter->AllocateTensors();

	model_input = interpreter->input(0);
	model_output = interpreter->output(0);

	if ((model_input->dims->size != 4) || (model_input->dims->data[0] != 1) ||
	    (model_input->dims->data[1] != 28) || (model_input->dims->data[2] != 28) ||
	    model_input->type != kTfLiteUInt8) {
		LOG_ERR("Wrong input tensor parameters");
		return -EINVAL;
	}

	if ((model_output->dims->size != 2) || (model_output->dims->data[0] != 1) ||
	    (model_output->dims->data[1] != 10) || (model_output->type != kTfLiteUInt8)) {
		LOG_ERR("Wrong output tensor parameters");
		return -EINVAL;
	}

	return 0;
}

uint8_t *get_input_buffer()
{
	if (!model_input) {
		return nullptr;
	}

	return model_input->data.uint8;
}

int run_prediction(struct prediction *pred)
{
	uint8_t *data = model_output->data.uint8;

	TfLiteStatus status = interpreter->Invoke();
	if (status != kTfLiteOk) {
		LOG_ERR("Model invoke failed");
		return -EIO;
	}

	// max_val = model_output->data.uint8[0];
	pred->value = 0;
	pred->confidence = data[0];
	for (uint8_t i = 1; i < model_output->bytes; ++i) {
		if (pred->confidence < data[i]) {
			pred->value = i;
			pred->confidence = data[i];
		}
	}

	return 0;
}
