/* * SirenSense - Driver Safety System
 * Author: [Ibrahim Abughazala ]
 * Purpose: Real-time emergency vehicle detection and localization
 */

#include <Arduino.h>
#include <driver/i2s.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SirenSense_inferencing.h> // Edge Impulse model header (library)

// Pin Mapping for ESP32-S3 N16R8
#define OLED_SDA 8
#define OLED_SCL 9
#define LED_ALERT 10

// Microphone Array Setup (4 channels)
#define MIC_FRONT_WS 5
#define MIC_FRONT_DATA 6
#define MIC_FRONT_CLK 4
#define MIC_REAR_WS 16
#define MIC_REAR_DATA 7
#define MIC_REAR_CLK 15

// Audio Specs
#define MIC_FREQ 16000
#define LCD_WIDTH 128
#define LCD_HEIGHT 64

// Tuning Params - Adjusted for reliable detection
#define MIN_CONFIDENCE 0.88f   // Adjust if needed
#define PERSIST_TIME_MS 2500   
#define DRIFT_FILTER 0.2f      
#define WARMUP_TIME 10000      

Adafruit_SSD1306 oled(LCD_WIDTH, LCD_HEIGHT, &Wire, -1);

// Audio Buffers - Mapped to PSRAM because they are huge
int16_t *audio_FL, *audio_FR, *audio_RL, *audio_RR; 

// Global tracking variables
unsigned long siren_timestamp = 0;
bool is_siren_on = false;
float vector_x = 0;
float vector_y = 0;
int last_valid_angle = 0;

// Local helpers
void setup_audio_hardware();
void stream_mic_data();
int map_audio_to_ai(size_t offset, size_t length, float *out_ptr);
int get_siren_heading();
void render_compass(int degree);
void perform_startup_check();

void setup() {
    Serial.begin(115200);
    pinMode(LED_ALERT, OUTPUT);
    
    // Fire up the screen
    Wire.begin(OLED_SDA, OLED_SCL);
    if(!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println("OLED init failed");
        for(;;);
    }
    
    // Quick splash screen for the user
    oled.clearDisplay();
    oled.setTextColor(WHITE);
    oled.setTextSize(2);
    oled.setCursor(5, 15);
    oled.println("SirenSense");
    oled.setTextSize(1);
    oled.setCursor(25, 40);
    oled.println("Drive safer");
    oled.display();
    delay(3000);

    // Grab memory from the 8MB Octal PSRAM
    if(psramFound()){
        size_t samples = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
        audio_FL = (int16_t*)ps_malloc(samples * sizeof(int16_t));
        audio_FR = (int16_t*)ps_malloc(samples * sizeof(int16_t));
        audio_RL = (int16_t*)ps_malloc(samples * sizeof(int16_t));
        audio_RR = (int16_t*)ps_malloc(samples * sizeof(int16_t));
    } else {
        oled.clearDisplay();
        oled.setCursor(0,0);
        oled.println("CRITICAL: NO PSRAM");
        oled.display();
        while(1); 
    }

    setup_audio_hardware();
    perform_startup_check();
}

void loop() {
    // 1. Grab fresh audio frames
    stream_mic_data();

    // 2. Prepare data for the ei classifier
    signal_t ai_signal;
    ai_signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
    ai_signal.get_data = &map_audio_to_ai;

    ei_impulse_result_t ai_result = { 0 };
    run_classifier(&ai_signal, &ai_result, false);

    // 3. Extract siren label score
    float probability = 0;
    for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        if (strcmp(ai_result.classification[i].label, "siren") == 0) {
            probability = ai_result.classification[i].value;
        }
    }

    oled.clearDisplay();

    // 4. Decision Logic
    if (probability > MIN_CONFIDENCE) {
        siren_timestamp = millis();
        is_siren_on = true;
        last_valid_angle = get_siren_heading();
        
        digitalWrite(LED_ALERT, HIGH);
        render_compass(last_valid_angle);
    } 
    else if (millis() - siren_timestamp < PERSIST_TIME_MS) {
        // Keep showing the last known direction for a bit
        digitalWrite(LED_ALERT, HIGH);
        render_compass(last_valid_angle);
    } 
    else {
        // Clear status
        is_siren_on = false;
        digitalWrite(LED_ALERT, LOW);
        oled.setTextSize(2);
        oled.setCursor(40, 25);
        oled.print("SAFE");
    }

    oled.display();
    yield(); // Keep ESP32 system tasks happy
}

// 10 seconds test for 4 microphones are actually working
void perform_startup_check() {
    unsigned long timer = millis();
    while (millis() - timer < WARMUP_TIME) {
        size_t bytes;
        int32_t raw_f[256], raw_r[256];
        long fl_max=0, fr_max=0, rl_max=0, rr_max=0;

        i2s_read(I2S_NUM_0, raw_f, sizeof(raw_f), &bytes, 10);
        i2s_read(I2S_NUM_1, raw_r, sizeof(raw_r), &bytes, 10);

        for (int i=0; i<64; i+=2) {
            fl_max = max(fl_max, (long)abs(raw_f[i] >> 14));
            fr_max = max(fr_max, (long)abs(raw_f[i+1] >> 14));
            rl_max = max(rl_max, (long)abs(raw_r[i] >> 14));
            rr_max = max(rr_max, (long)abs(raw_r[i+1] >> 14));
        }

        oled.clearDisplay();
        oled.setTextSize(1);
        oled.setCursor(0, 0);
        oled.println("Mics Test (10s)");
        
        // Dynamic visual feedback bars
        oled.fillRect(0, 15, map(fl_max, 0, 8000, 0, 128), 6, WHITE); oled.setCursor(0, 15); oled.print("FL");
        oled.fillRect(0, 25, map(fr_max, 0, 8000, 0, 128), 6, WHITE); oled.setCursor(0, 25); oled.print("FR");
        oled.fillRect(0, 35, map(rl_max, 0, 8000, 0, 128), 6, WHITE); oled.setCursor(0, 35); oled.print("RL");
        oled.fillRect(0, 45, map(rr_max, 0, 8000, 0, 128), 6, WHITE); oled.setCursor(0, 45); oled.print("RR");

        int remaining = (WARMUP_TIME - (millis() - timer)) / 1000;
        oled.setCursor(30, 56);
        oled.print("Starting in: "); oled.print(remaining);
        oled.display();
        yield();
    }
}

// Draw compass arrow for the driver
void render_compass(int degree) {
    int x_off = 64, y_off = 32, radius = 26;
    oled.drawCircle(x_off, y_off, radius, WHITE);
    
    // Adjust 0 deg to point Up
    float rad = (degree - 90) * 0.0174533f;
    int tip_x = x_off + (cos(rad) * (radius - 4));
    int tip_y = y_off + (sin(rad) * (radius - 4));
    
    oled.drawLine(x_off, y_off, tip_x, tip_y, WHITE);
    oled.fillCircle(tip_x, tip_y, 4, WHITE);

    oled.setTextSize(1);
    oled.setCursor(0, 0);
    oled.print("SIREN");
}

// Math for direction the sound is pulling from
int get_siren_heading() {
    long fl_e=0, fr_e=0, rl_e=0, rr_e=0;
    size_t samples = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;

    for(int i=0; i < samples; i++) {
        fl_e += abs(audio_FL[i]); fr_e += abs(audio_FR[i]);
        rl_e += abs(audio_RL[i]); rr_e += abs(audio_RR[i]);
    }

    // Convert chanel energy into spatial pull
    float raw_x = (float)(-fl_e + fr_e - rl_e + rr_e);
    float raw_y = (float)(fl_e + fr_e - rl_e - rr_e);

    // Drift filter prevents the arrow from jittring
    vector_x = (vector_x * (1.0f - DRIFT_FILTER)) + (raw_x * DRIFT_FILTER);
    vector_y = (vector_y * (1.0f - DRIFT_FILTER)) + (raw_y * DRIFT_FILTER);

    float final_rad = atan2(vector_x, vector_y);
    int final_deg = (int)(final_rad * 57.2958f);
    
    if (final_deg < 0) final_deg += 360;
    return final_deg;
}

void setup_audio_hardware() {
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = MIC_FREQ,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_I2S,

        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = 256,
        .use_apll = false
    };

    // Front pair
    i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);

    i2s_pin_config_t pins_f = {.bck_io_num=MIC_FRONT_CLK, .ws_io_num=MIC_FRONT_WS, .data_out_num=-1, .data_in_num=MIC_FRONT_DATA};
    i2s_set_pin(I2S_NUM_0, &pins_f);

    // Rear pair
    i2s_driver_install(I2S_NUM_1, &i2s_config, 0, NULL);

    i2s_pin_config_t pins_r = {.bck_io_num=MIC_REAR_CLK, .ws_io_num=MIC_REAR_WS, .data_out_num=-1, .data_in_num=MIC_REAR_DATA};
    i2s_set_pin(I2S_NUM_1, &pins_r);
}

void stream_mic_data() {
    size_t bytes_in;
    int32_t front_block[256], rear_block[256];
    int captured = 0;
    size_t target = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;

    while (captured < target) {
        i2s_read(I2S_NUM_0, front_block, sizeof(front_block), &bytes_in, 20);
        i2s_read(I2S_NUM_1, rear_block, sizeof(rear_block), &bytes_in, 20);
        
        int block_samples = bytes_in / 4; 
        for (int i=0; i < block_samples; i+=2) {
            if (captured < target) {
                // Downsample 32-bit to 16-bit for AI processing
                audio_FL[captured] = front_block[i] >> 14;
                audio_FR[captured] = front_block[i+1] >> 14;

            audio_RL[captured] = rear_block[i] >> 14;
            audio_RR[captured] = rear_block[i+1] >> 14;
            captured++;
            }
        }
    }
}

// Wrapper for the edge impulse library to read the front-left buffer
int map_audio_to_ai(size_t offset, size_t length, float *out_ptr) {
    for (size_t i = 0; i < length; i++) {
        out_ptr[i] = (float)audio_FL[offset + i];
    }
    return 0;
}