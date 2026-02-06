#include "include/ai_core.h"
#include "model.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include <cmath>

// Configurações do Scaler 
const float SCALER_MEAN[14] = { 
    1934.83f, 
    1674.64f, 
    3704.06f, 
    3830.94f, 
    4841.40f, 
    3924.08f, 
    16553.61f, 
    8511.70f, 
    7386.97f, 
    6556.68f, 
    11602.06f, 
    4.70f, 
    4.76f, 
    4.09f 
};

const float SCALER_SCALE[14] = { 
    1901.21f, 
    1595.66f, 
    4633.72f, 
    3558.05f, 
    4998.32f, 
    3844.85f, 
    2659.00f, 
    7030.70f, 
    7495.96f, 
    6728.49f, 
    12850.84f, 
    2.20f, 
    1.97f, 
    2.04f 
};

const char* CLASSES[] = { "caminhando", "correndo", "parado", "pulando" };

// Variáveis Globais do TFLite
namespace {
    const tflite::Model* model = nullptr;
    tflite::MicroInterpreter* interpreter = nullptr;
    TfLiteTensor* input = nullptr;
    TfLiteTensor* output = nullptr;
    uint8_t tensor_arena[12 * 1024];
}

extern "C" bool ai_init(void) {
    model = tflite::GetModel(model_data);
    
    static tflite::MicroMutableOpResolver<4> resolver;
    resolver.AddFullyConnected();
    resolver.AddRelu();
    resolver.AddSoftmax();
    resolver.AddQuantize();
    
    static tflite::MicroInterpreter static_interpreter(
        model, resolver, tensor_arena, sizeof(tensor_arena));
    interpreter = &static_interpreter;
    
    if (interpreter->AllocateTensors() != kTfLiteOk) return false;
    
    input = interpreter->input(0);
    output = interpreter->output(0);
    return true;
}

extern "C" const char* ai_run_inference(float *features, float *confidence_out) {
    //Normalizar e Quantizar
    int8_t* in_data = input->data.int8;
    for (int i = 0; i < 14; i++) {
        float norm = (features[i] - SCALER_MEAN[i]) / SCALER_SCALE[i];
        int32_t q = (int32_t)(norm / input->params.scale) + input->params.zero_point;
        if(q < -128) q = -128; if(q > 127) q = 127;
        in_data[i] = (int8_t)q;
    }

    //Rodar Modelo
    if (interpreter->Invoke() != kTfLiteOk) return "Erro";

    //Processar Saída
    int8_t* out_data = output->data.int8;
    float max_score = -1000.0f;
    int max_idx = 0;
    float probs[4];
    float sum = 0;

    // Dequantizar
    for(int i=0; i<4; i++) {
        float val = (out_data[i] - output->params.zero_point) * output->params.scale;
        if(val > max_score) max_score = val;
        probs[i] = val;
    }
    
    // Softmax simplificado
    for(int i=0; i<4; i++) {
        probs[i] = expf(probs[i] - max_score);
        sum += probs[i];
    }
    
    float best_conf = 0;
    for(int i=0; i<4; i++) {
        probs[i] /= sum;
        if(probs[i] > best_conf) {
            best_conf = probs[i];
            max_idx = i;
        }
    }

    *confidence_out = best_conf * 100.0f;
    return CLASSES[max_idx];
}