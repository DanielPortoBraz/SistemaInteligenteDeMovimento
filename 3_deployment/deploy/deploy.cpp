#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <math.h>


#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/system_setup.h"
#include "tensorflow/lite/schema/schema_generated.h"

// Modelo
#include "model.h"

// ==================== CONFIGURAÇÕES ====================
#define I2C_PORT i2c0
#define I2C_SDA 0
#define I2C_SCL 1
#define MPU6500_ADDR 0x68

#define MPU6500_PWR_MGMT_1 0x6B
#define MPU6500_ACCEL_CONFIG 0x1C
#define MPU6500_GYRO_CONFIG 0x1B
#define MPU6500_ACCEL_XOUT_H 0x3B

#define WINDOW_SIZE 20
#define SAMPLE_INTERVAL_MS 50

//VALORES DO COLAB
const float scaler_mean[14] = {
    1934.8329f,
    1674.6427f,
    3704.0657f,
    3830.9411f,
    4841.4041f,
    3924.0823f,
    16553.6141f,
    8511.7087f,
    7386.9779f,
    6556.6814f,
    11602.0662f,
    4.7035f,
    4.7634f,
    4.0962f
};

const float scaler_scale[14] = {
    1901.2161f,
    1595.6699f,
    4633.7235f,
    3558.0585f,
    4998.3214f,
    3844.8550f,
    2659.0029f,
    7030.7041f,
    7495.9663f,
    6728.4963f,
    12850.8414f,
    2.2013f,
    1.9772f,
    2.0447f
};

const char* class_names[4] = {
    "caminhando",
    "correndo",
    "parado",
    "pulando"
};

// ==================== JANELA DE DADOS ====================
struct SensorSample {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
};

class SensorWindow {
private:
    SensorSample buffer[WINDOW_SIZE];
    int write_index;
    bool is_full;
    
public:
    SensorWindow() : write_index(0), is_full(false) {}
    
    void add_sample(int16_t ax, int16_t ay, int16_t az, 
                    int16_t gx, int16_t gy, int16_t gz) {
        buffer[write_index].ax = ax;
        buffer[write_index].ay = ay;
        buffer[write_index].az = az;
        buffer[write_index].gx = gx;
        buffer[write_index].gy = gy;
        buffer[write_index].gz = gz;
        
        write_index++;
        if (write_index >= WINDOW_SIZE) {
            write_index = 0;
            is_full = true;
        }
    }
    
    bool ready() const {
        return is_full;
    }
    
    //EXTRAÇÃO DE FEATURES
    void extract_movement_features(float features[14]) {
        // Arrays temporários para cálculos
        float ax[WINDOW_SIZE], ay[WINDOW_SIZE], az[WINDOW_SIZE];
        float gx[WINDOW_SIZE], gy[WINDOW_SIZE], gz[WINDOW_SIZE];
        
        // Copiar dados
        for (int i = 0; i < WINDOW_SIZE; i++) {
            ax[i] = buffer[i].ax;
            ay[i] = buffer[i].ay;
            az[i] = buffer[i].az;
            gx[i] = buffer[i].gx;
            gy[i] = buffer[i].gy;
            gz[i] = buffer[i].gz;
        }
        
        // DESVIO PADRÃO (features 0-5)
        features[0] = compute_std(ax, WINDOW_SIZE);
        features[1] = compute_std(ay, WINDOW_SIZE);
        features[2] = compute_std(az, WINDOW_SIZE);
        features[3] = compute_std(gx, WINDOW_SIZE);
        features[4] = compute_std(gy, WINDOW_SIZE);
        features[5] = compute_std(gz, WINDOW_SIZE);
        
        //MAGNITUDE MÉDIA
        float mag_accel_sum = 0;
        float mag_gyro_sum = 0;
        for (int i = 0; i < WINDOW_SIZE; i++) {
            mag_accel_sum += sqrtf(ax[i]*ax[i] + ay[i]*ay[i] + az[i]*az[i]);
            mag_gyro_sum += sqrtf(gx[i]*gx[i] + gy[i]*gy[i] + gz[i]*gz[i]);
        }
        features[6] = mag_accel_sum / WINDOW_SIZE;
        features[7] = mag_gyro_sum / WINDOW_SIZE;
        
        //RANGE
        features[8] = compute_range(ax, WINDOW_SIZE);
        features[9] = compute_range(ay, WINDOW_SIZE);
        features[10] = compute_range(az, WINDOW_SIZE);
        
        // 4. ZERO-CROSSING RATE (features 11-13)
        features[11] = compute_zcr(ax, WINDOW_SIZE);
        features[12] = compute_zcr(ay, WINDOW_SIZE);
        features[13] = compute_zcr(az, WINDOW_SIZE);
    }
    
private:
    float compute_mean(float data[], int len) {
        float sum = 0;
        for (int i = 0; i < len; i++) {
            sum += data[i];
        }
        return sum / len;
    }
    
    float compute_std(float data[], int len) {
        float mean = compute_mean(data, len);
        float sum_sq = 0;
        for (int i = 0; i < len; i++) {
            float diff = data[i] - mean;
            sum_sq += diff * diff;
        }
        return sqrtf(sum_sq / len);
    }
    
    float compute_range(float data[], int len) {
        float min_val = data[0];
        float max_val = data[0];
        for (int i = 1; i < len; i++) {
            if (data[i] < min_val) min_val = data[i];
            if (data[i] > max_val) max_val = data[i];
        }
        return max_val - min_val;
    }
    
    float compute_zcr(float data[], int len) {
        float mean = compute_mean(data, len);
        int crossings = 0;
        
        for (int i = 1; i < len; i++) {
            bool prev_above = (data[i-1] - mean) > 0;
            bool curr_above = (data[i] - mean) > 0;
            if (prev_above != curr_above) {
                crossings++;
            }
        }
        
        return crossings / 2.0f;
    }
};

// ==================== MPU-6500 ====================
void mpu6500_write(uint8_t reg, uint8_t data) {
    uint8_t buf[2] = {reg, data};
    i2c_write_blocking(I2C_PORT, MPU6500_ADDR, buf, 2, false);
}

void mpu6500_read_registers(uint8_t reg, uint8_t *buffer, uint8_t len) {
    i2c_write_blocking(I2C_PORT, MPU6500_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU6500_ADDR, buffer, len, false);
}

void mpu6500_init() {
    mpu6500_write(MPU6500_PWR_MGMT_1, 0x00);
    sleep_ms(100);
    mpu6500_write(MPU6500_ACCEL_CONFIG, 0x00);
    mpu6500_write(MPU6500_GYRO_CONFIG, 0x00);
    sleep_ms(100);
}

void mpu6500_read_data(int16_t *accel, int16_t *gyro) {
    uint8_t buffer[14];
    mpu6500_read_registers(MPU6500_ACCEL_XOUT_H, buffer, 14);
    
    accel[0] = (buffer[0] << 8) | buffer[1];
    accel[1] = (buffer[2] << 8) | buffer[3];
    accel[2] = (buffer[4] << 8) | buffer[5];
    
    gyro[0] = (buffer[8] << 8) | buffer[9];
    gyro[1] = (buffer[10] << 8) | buffer[11];
    gyro[2] = (buffer[12] << 8) | buffer[13];
}

// ==================== TENSORFLOW LITE ====================
namespace {
    const tflite::Model* model = nullptr;
    tflite::MicroInterpreter* interpreter = nullptr;
    TfLiteTensor* input = nullptr;
    TfLiteTensor* output = nullptr;
    
    constexpr int kTensorArenaSize = 12 * 1024;
    uint8_t tensor_arena[kTensorArenaSize];
}

void setup_tflite() {
    model = tflite::GetModel(model_data);
    if (model->version() != TFLITE_SCHEMA_VERSION) {
        printf("Model schema mismatch!\n");
        return;
    }
    
    static tflite::MicroMutableOpResolver<4> resolver;
    resolver.AddFullyConnected();
    resolver.AddRelu();
    resolver.AddSoftmax();
    resolver.AddQuantize();
    
    static tflite::MicroInterpreter static_interpreter(
        model, resolver, tensor_arena, kTensorArenaSize);
    interpreter = &static_interpreter;
    
    TfLiteStatus allocate_status = interpreter->AllocateTensors();
    if (allocate_status != kTfLiteOk) {
        printf("AllocateTensors() failed!\n");
        return;
    }
    
    input = interpreter->input(0);
    output = interpreter->output(0);
    
    printf("✓ TensorFlow Lite Micro initialized!\n");
    printf("  Input shape: [%d, %d]\n", input->dims->data[0], input->dims->data[1]);
    printf("  Output shape: [%d, %d]\n", output->dims->data[0], output->dims->data[1]);
}

void normalize_and_quantize(float *features, int8_t *quantized_data) {
    float input_scale = input->params.scale;
    int input_zero_point = input->params.zero_point;
    
    for (int i = 0; i < 14; i++) {
        // Normalizar
        float normalized = (features[i] - scaler_mean[i]) / scaler_scale[i];
        
        // Quantizar
        int32_t quantized = (int32_t)(normalized / input_scale) + input_zero_point;
        quantized = (quantized < -128) ? -128 : (quantized > 127) ? 127 : quantized;
        quantized_data[i] = (int8_t)quantized;
    }
}

struct PredictionResult {
    const char* activity;
    float confidence;
    float probabilities[4];
};

PredictionResult predict_activity(SensorWindow &window) {
    PredictionResult result;
    
    // Extrair features de movimento
    float features[14];
    window.extract_movement_features(features);
    
    // Normalizar e quantizar
    int8_t* input_data = input->data.int8;
    normalize_and_quantize(features, input_data);
    
    // Inferência
    TfLiteStatus invoke_status = interpreter->Invoke();
    if (invoke_status != kTfLiteOk) {
        result.activity = "ERROR";
        result.confidence = 0.0f;
        return result;
    }
    
    // Processar saída
    int8_t* output_data = output->data.int8;
    float output_scale = output->params.scale;
    int output_zero_point = output->params.zero_point;
    
    // Dequantizar e softmax
    float raw_outputs[4];
    float max_val = -1000.0f;
    
    for (int i = 0; i < 4; i++) {
        raw_outputs[i] = (output_data[i] - output_zero_point) * output_scale;
        if (raw_outputs[i] > max_val) {
            max_val = raw_outputs[i];
        }
    }
    
    float sum_exp = 0.0f;
    for (int i = 0; i < 4; i++) {
        result.probabilities[i] = expf(raw_outputs[i] - max_val);
        sum_exp += result.probabilities[i];
    }
    
    int max_idx = 0;
    for (int i = 0; i < 4; i++) {
        result.probabilities[i] /= sum_exp;
        if (result.probabilities[i] > result.probabilities[max_idx]) {
            max_idx = i;
        }
    }
    
    result.activity = class_names[max_idx];
    result.confidence = result.probabilities[max_idx] * 100.0f;
    
    return result;
}

// ==================== MAIN ====================
int main() {
    stdio_init_all();
    
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    
    sleep_ms(3000);
    
    printf("\n==========================================\n");
    printf("  TinyML - Features de MOVIMENTO\n");
    printf("  (não posição!)\n");
    printf("==========================================\n\n");
    
    printf("Initializing MPU6500...\n");
    mpu6500_init();
    printf("✓ MPU6500 ready!\n\n");
    
    printf("Initializing TensorFlow Lite Micro...\n");
    setup_tflite();
    
    printf("\n==========================================\n");
    printf("  Features Extraídas:\n");
    printf("  - Desvio padrão (6)\n");
    printf("  - Magnitude (2)\n");
    printf("  - Range (3)\n");
    printf("  - Zero-crossing (3)\n");
    printf("  Total: 14 features\n");
    printf("==========================================\n\n");
    
    SensorWindow window;
    
    printf("Coletando amostras...\n\n");
    
    int16_t accel[3];
    int16_t gyro[3];
    uint32_t last_sample_time = 0;
    
    while (true) {
        uint32_t current_time = to_ms_since_boot(get_absolute_time());
        
        if (current_time - last_sample_time >= SAMPLE_INTERVAL_MS) {
            last_sample_time = current_time;
            
            mpu6500_read_data(accel, gyro);
            window.add_sample(accel[0], accel[1], accel[2],
                            gyro[0], gyro[1], gyro[2]);
            
            if (window.ready()) {
                PredictionResult prediction = predict_activity(window);
                
                printf("\n┌─────────────────────────────────────────┐\n");
                printf("│ Atividade: %-28s │\n", prediction.activity);
                printf("│ Confiança: %5.1f%%                       │\n", prediction.confidence);
                printf("└─────────────────────────────────────────┘\n");
                
                sleep_ms(500);
            }
        }
        
        sleep_ms(1);
    }
    
    return 0;
}