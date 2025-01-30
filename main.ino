#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <Adafruit_AHTX0.h>

// Define the LED pin
#define LED_PIN PA7

/** Flag if transmit is active */
volatile bool tx_active = false;

// Sensors
VL53L1X vl53;
Adafruit_AHTX0 aht;

// Data collection
uint8_t collected_data[16] = {0};

#define OTAA_DEVEUI {0x60, 0x7A, 0x55, 0x41, 0x4E, 0x41, 0x6C, 0x7B}
#define OTAA_APPEUI {0x59, 0x8C, 0x6D, 0x58, 0xCC, 0x04, 0xB2, 0xC1}
#define OTAA_APPKEY {0x61, 0x06, 0x1B, 0x09, 0x81, 0x25, 0xFF, 0xC5, 0x76, 0xD2, 0x39, 0xE5, 0x84, 0x52, 0x00, 0x03}

// Debug flag
const bool ENABLE_DEBUG = true;  // Set to false to disable debug output

void recvCallback(SERVICE_LORA_RECEIVE_T *data)
{
    if (data->BufferSize > 0) {
        Serial.println("Something received!");

        // Convert received bytes to string for easier comparison
        char hexStr[32] = {0};
        for (int i = 0; i < data->BufferSize; i++) {
            sprintf(hexStr + (i * 2), "%02x", data->Buffer[i]);
        }
        Serial.println(hexStr);

        // Check if it starts with "sleep"
        if (strncmp(hexStr, "736c656570", 10) == 0) { // "sleep" in hex
            // Extract the ASCII numbers after "sleep"
            char numStr[4] = {0};
            int numLen = (data->BufferSize - 5); // 5 is length of "sleep"
            if (numLen > 0 && numLen <= 3) {
                memcpy(numStr, (char *)&data->Buffer[5], numLen);
                long minutes = atol(numStr); // Convert ASCII to long

                // Update timer interval (convert minutes to milliseconds)
                uint32_t interval = minutes * 60 * 1000;
                api.system.timer.stop(RAK_TIMER_0);
                api.system.timer.start(RAK_TIMER_0, interval, NULL);

                Serial.printf("Sleep time updated to %ld minutes\n", minutes);
            }
        }
    }
    tx_active = false;
}

void joinCallback(int32_t status)
{
    Serial.printf("Join status: %d\r\n", status);
    if (status == 0) {
        Serial.println("Successfully joined network");
        digitalWrite(LED_PIN, LOW);

        // Start the periodic timer after successful join
        api.system.timer.create(RAK_TIMER_0, sensor_handler, RAK_TIMER_PERIODIC);
        api.system.timer.start(RAK_TIMER_0, 30000, NULL); // 30000 = 30 seconds
    } else {
        // If join failed, try again
        api.lorawan.join();
    }
}

void sendCallback(int32_t status)
{
    Serial.printf("Send status: %d\r\n", status);
    digitalWrite(LED_PIN, LOW);
    tx_active = false;
}

void sensor_handler(void *)
{
    if (ENABLE_DEBUG) {
        Serial.println("\n============================================================");
        Serial.println("🔄 NEW MEASUREMENT CYCLE STARTED");
        Serial.println("============================================================");
    }
    digitalWrite(LED_PIN, HIGH);

    // Check if we're joined to the network
    if (!api.lorawan.njs.get()) {
        if (ENABLE_DEBUG)
            Serial.println("❌ Not joined, skip sending");
        return;
    }

    uint8_t data_len = 0;

    // VL53L1X operations using Pololu library
    if (ENABLE_DEBUG) {
        Serial.println("\n📡 TOF SENSOR OPERATIONS");
        Serial.println("------------------------------------------------------------");
    }
    vl53.stopContinuous();
    delay(50);

    vl53.startContinuous(50);

    uint32_t start = millis();
    bool validReading = false;
    int16_t distance = 0; // Default to 0 for invalid readings

    while ((millis() - start) < 1000) {
        if (vl53.dataReady()) {
            distance = vl53.read();
            if (distance >= 0) {
                if (ENABLE_DEBUG) {
                    Serial.printf("Distance: %d mm (read time: %lu ms)\n",
                                  distance, millis() - start);
                }
                validReading = true;
            } else if (ENABLE_DEBUG) {
                Serial.println("❌ Invalid distance reading, setting to 0 mm");
            }
            break;
        }
        delay(10);
    }
    vl53.stopContinuous();

    if (!validReading) {
        if (ENABLE_DEBUG) {
            Serial.println("❌ Failed to get valid TOF reading - Setting to 0 mm");
        }
        distance = 0;
    }

    // Store the distance value regardless of validity
    collected_data[data_len++] = (distance >> 8) & 0xFF;
    collected_data[data_len++] = distance & 0xFF;

    if (ENABLE_DEBUG) {
        Serial.println("\n🌡️ AHT20 SENSOR OPERATIONS");
        Serial.println("------------------------------------------------------------");
    }

    // AHT20 operations
    sensors_event_t humidity, temp;
    if (aht.getEvent(&humidity, &temp)) {
        int16_t temp_scaled = (int16_t)(temp.temperature * 10);
        collected_data[data_len++] = (temp_scaled >> 8) & 0xFF;
        collected_data[data_len++] = temp_scaled & 0xFF;

        int16_t hum_scaled = (int16_t)(humidity.relative_humidity * 10);
        collected_data[data_len++] = (hum_scaled >> 8) & 0xFF;
        collected_data[data_len++] = hum_scaled & 0xFF;

        if (ENABLE_DEBUG) {
            Serial.printf("Temperature: %.1f °C, Humidity: %.1f %%\n",
                          temp.temperature, humidity.relative_humidity);
        }
    } else if (ENABLE_DEBUG) {
        Serial.println("❌ Failed to read AHT20 sensor");
    }

    // Send the packet
    if (ENABLE_DEBUG) {
        Serial.println("\n📤 LORAWAN OPERATIONS");
        Serial.println("------------------------------------------------------------");
    }
    if (api.lorawan.send(data_len, collected_data, 2, false, 1)) {
        if (ENABLE_DEBUG) {
            Serial.printf("✅ Packet sent successfully (Payload: %d bytes)\n", data_len);
        }
        tx_active = true;
    } else {
        if (ENABLE_DEBUG)
            Serial.println("❌ Failed to send message");
        tx_active = false;
    }

    if (ENABLE_DEBUG) {
        Serial.println("\n============================================================");
        Serial.println("✨ MEASUREMENT CYCLE COMPLETED");
        Serial.println("============================================================\n");
    }
}

void setup()
{
    Serial.begin(115200);
    delay(1000);

    // Configure the LED pin
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // Configure PB4 pin
    pinMode(PB4, OUTPUT);
    digitalWrite(PB4, LOW);

    // Initialize I2C and sensors
    Wire.begin();
    if (!vl53.init()) {
        Serial.println("Failed to initialize VL53L1X");
        while (1);
    }
    vl53.setDistanceMode(VL53L1X::Long);
    vl53.setMeasurementTimingBudget(50000); // 50 ms
    vl53.startContinuous(50); // Start continuous ranging

    if (!aht.begin()) {
        Serial.println("Failed to initialize AHT20");
        while (1);
    }

    // OTAA Device EUI MSB first
    uint8_t node_device_eui[8] = OTAA_DEVEUI;
    // OTAA Application EUI MSB first
    uint8_t node_app_eui[8] = OTAA_APPEUI;
    // OTAA Application Key MSB first
    uint8_t node_app_key[16] = OTAA_APPKEY;

    // Configure LoRaWAN
    api.lorawan.appeui.set(node_app_eui, 8);
    api.lorawan.appkey.set(node_app_key, 16);
    api.lorawan.deui.set(node_device_eui, 8);
    api.lorawan.band.set(RAK_REGION_EU868);
    api.lorawan.deviceClass.set(RAK_LORA_CLASS_A);
    api.lorawan.njm.set(RAK_LORA_OTAA);

    // Register callbacks
    api.lorawan.registerRecvCallback(recvCallback);
    api.lorawan.registerJoinCallback(joinCallback);
    api.lorawan.registerSendCallback(sendCallback);

    // Enable low power mode
    api.system.lpm.set(1);

    // Attempt to join - callback will handle the result
    api.lorawan.join();
}

void loop()
{
    // Let the system sleep, everything is timer/event driven
    api.system.sleep.all();
}
