#include "TinyMLManager.h"
#include "../Utilities/Logger.h"

#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"

// Include your trained model
#include "model_data.h"

using namespace JRDev;

// TensorFlow Lite globals
namespace {
    constexpr int tensorArenaSize = 2 * 1024;
    uint8_t tensorArena[tensorArenaSize];

    tflite::MicroInterpreter* interpreter;
    TfLiteTensor* inputTensor;
    TfLiteTensor* outputTensor;
}

void TinyMLManager::begin() {
    static tflite::MicroErrorReporter micro_error_reporter;
    const tflite::Model* model = tflite::GetModel(g_model);

    static tflite::AllOpsResolver resolver;
    static tflite::MicroInterpreter static_interpreter(
        model, resolver, tensorArena, tensorArenaSize, &micro_error_reporter
    );

    interpreter = &static_interpreter;

    TfLiteStatus allocate_status = interpreter->AllocateTensors();
    if (allocate_status != kTfLiteOk) {
        Logger::error("Failed to allocate tensors!");
        return;
    }

    inputTensor = interpreter->input(0);
    outputTensor = interpreter->output(0);

    Logger::info("TinyMLManager initialized.");
}

float* TinyMLManager::runInference(const float* input, size_t length) {
    if (!interpreter || !inputTensor || length != inputTensor->bytes / sizeof(float)) {
        Logger::error("Inference setup invalid.");
        return nullptr;
    }

    memcpy(inputTensor->data.f, input, length * sizeof(float));

    if (interpreter->Invoke() != kTfLiteOk) {
        Logger::error("Inference failed.");
        return nullptr;
    }

    return outputTensor->data.f;
}

void TinyMLManager::printOutput(float* output, size_t length) {
    Serial.print("[TinyML Output]: ");
    for (size_t i = 0; i < length; i++) {
        Serial.print(output[i], 5);
        Serial.print(" ");
    }
    Serial.println();
}
