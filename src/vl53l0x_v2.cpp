#include "vl53l0x_v2.h"

// Three ToF sensors using VL53L0X library
VL53L0X tofSensor[NUM_TOF_SENSORS];
float tof_distance[NUM_TOF_SENSORS] = {0, 0, 0};
float tof_filtered[NUM_TOF_SENSORS] = {0, 0, 0};  // Filtered values
bool tof_initialized[NUM_TOF_SENSORS] = {false, false, false};  // Track if filter is initialized
bool tof_ready[NUM_TOF_SENSORS] = {false, false, false};

// Robust filtering configuration
static constexpr int MEDIAN_WINDOW = 5;           // small window for embedded
static constexpr float OUTLIER_JUMP_MM = 150.0f;  // reject sudden spikes beyond this
// Ring buffers for median
static float raw_buf[NUM_TOF_SENSORS][MEDIAN_WINDOW] = {};
static uint8_t raw_count[NUM_TOF_SENSORS] = {0, 0, 0};
static uint8_t raw_idx[NUM_TOF_SENSORS] = {0, 0, 0};

// XSHUT pins for each sensor
const int XSHUT_PINS[NUM_TOF_SENSORS] = {17, 16, 5};  // Front, Right, Left
const uint8_t I2C_ADDRESSES[NUM_TOF_SENSORS] = {0x30, 0x31, 0x32};  // Custom addresses (not 0x29)
const float MAX_RANGE = 1200.0f;
const float CALIBRATION_OFFSET[NUM_TOF_SENSORS] = {16.0f, 7.0f, 44.0f};  // Front, Right, Left
const float FILTER_ALPHA = 0.25f;  // EMA on median (lower = smoother)

static float median_of(float* arr, int n)
{
    // insertion sort into local buffer, n <= MEDIAN_WINDOW
    float tmp[MEDIAN_WINDOW];
    for (int i = 0; i < n; ++i) tmp[i] = arr[i];
    for (int i = 1; i < n; ++i) {
        float key = tmp[i];
        int j = i - 1;
        while (j >= 0 && tmp[j] > key) {
            tmp[j + 1] = tmp[j];
            --j;
        }
        tmp[j + 1] = key;
    }
    if (n % 2 == 1) return tmp[n/2];
    return 0.5f * (tmp[n/2 - 1] + tmp[n/2]);
}


bool setupToF() {
    Serial.println("Initializing ToF sensors...");
    
    // Pull all XSHUT pins low first
    for (int i = 0; i < NUM_TOF_SENSORS; i++) {
        pinMode(XSHUT_PINS[i], OUTPUT);
        digitalWrite(XSHUT_PINS[i], LOW);
    }
    delay(20);
    
    // Initialize each sensor one by one
    for (int i = 0; i < NUM_TOF_SENSORS; i++) {
        // Enable this sensor
        digitalWrite(XSHUT_PINS[i], HIGH);
        delay(20);
        
        // Initialize sensor
        if (!tofSensor[i].init()) {
            Serial.print("ToF Error ");
            Serial.println(i + 1);
            tof_ready[i] = false;
            continue;
        } else {
            Serial.print("ToF Sensor initialized ");
            Serial.println(i + 1);
            tof_ready[i] = true;
        }
        
        // Set custom address immediately
        tofSensor[i].setAddress(I2C_ADDRESSES[i]);
        // Longer budget -> better accuracy (we read every ~200ms in loop)
        tofSensor[i].setMeasurementTimingBudget(50000);
        tofSensor[i].startContinuous();
    }
    
    Serial.println("ToF init complete");
    bool anyReady = false;
    for (int i = 0; i < NUM_TOF_SENSORS; i++) {
        anyReady |= tof_ready[i];
    }
    return anyReady;
}


bool readToF() {
    bool anyValid = false;
    for (int i = 0; i < NUM_TOF_SENSORS; i++) {
        if (!tof_ready[i]) {
            continue;
        }
        float distance = tofSensor[i].readRangeContinuousMillimeters();
        
        // Check for timeout or out of range
        if (tofSensor[i].timeoutOccurred() || distance == 0) {
            // Skip this reading, keep previous filtered value
            continue;
        }
        else if (distance >= 1200.0f) {
            distance = MAX_RANGE;
        }
        else {
            distance = distance - CALIBRATION_OFFSET[i];
            if (distance < 0) distance = 0;
        }
        
        // Update ring buffer
        raw_buf[i][raw_idx[i]] = distance;
        raw_idx[i] = (raw_idx[i] + 1) % MEDIAN_WINDOW;
        if (raw_count[i] < MEDIAN_WINDOW) raw_count[i]++;

        // Build linear window for median
        float window[MEDIAN_WINDOW];
        int n = raw_count[i];
        for (int k = 0; k < n; ++k) {
            int pos = (raw_idx[i] + MEDIAN_WINDOW - n + k) % MEDIAN_WINDOW;
            window[k] = raw_buf[i][pos];
        }
        float med = median_of(window, n);

        // Outlier rejection relative to filtered value
        if (tof_initialized[i]) {
            float delta = med - tof_filtered[i];
            if (delta > OUTLIER_JUMP_MM) med = tof_filtered[i] + OUTLIER_JUMP_MM;
            else if (delta < -OUTLIER_JUMP_MM) med = tof_filtered[i] - OUTLIER_JUMP_MM;
        }

        // EMA on median
        if (!tof_initialized[i]) {
            tof_filtered[i] = med;
            tof_initialized[i] = true;
        } else {
            tof_filtered[i] = FILTER_ALPHA * med + (1.0f - FILTER_ALPHA) * tof_filtered[i];
        }
        
        // Update output with filtered value
        tof_distance[i] = tof_filtered[i];
        anyValid = true;
    }

    return anyValid;
}


void flushToF() {
    // Clear the sensor buffer by reading multiple times
    for (int i = 0; i < 10; i++) {
        readToF();
        delay(5);
    }
}
