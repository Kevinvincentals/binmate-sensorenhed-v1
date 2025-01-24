#include <Arduino.h>
#include "Adafruit_VL53L1X.h"
#include <Adafruit_AHTX0.h>

// Define the LED pin
#define LED_PIN PA7

/** Flag if transmit is active */
volatile bool tx_active = false;

// Sensors
Adafruit_VL53L1X vl53;
Adafruit_AHTX0 aht;

// Data collection
uint8_t collected_data[16] = {0};

#define OTAA_DEVEUI {0x21, 0x56, 0xD0, 0x66, 0x4F, 0x99, 0x4E, 0xF8}
#define OTAA_APPEUI {0x06, 0x8C, 0xFA, 0x32, 0x0E, 0x78, 0xE6, 0xBE}
#define OTAA_APPKEY {0x42, 0x44, 0x29, 0x89, 0x59, 0x3B, 0x50, 0xBA, 0x0B, 0x72, 0xFB, 0x50, 0x86, 0x2A, 0x10, 0x63}

// Debug flag
const bool DEBUG = true;  // Set to false to disable debug output

void recvCallback(SERVICE_LORA_RECEIVE_T * data)
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
                memcpy(numStr, (char*)&data->Buffer[5], numLen);
                long minutes = atol(numStr);  // Convert ASCII to long
                
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
        api.system.timer.start(RAK_TIMER_0, 10000, NULL);  // 10000 = 10 seconds
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
    if (DEBUG) Serial.println("\n--- Wake up - Starting new measurement cycle ---");
    digitalWrite(LED_PIN, HIGH);

    // Check if we're joined to the network
    if (!api.lorawan.njs.get()) {
        if (DEBUG) Serial.println("Not joined, skip sending");
        return;
    }

    uint8_t data_len = 0;

    // VL53L1X operations
    if (DEBUG) {
        Serial.println("\nTOF Sensor Operations:");
        Serial.println("1. Stopping previous ranging...");
    }
    vl53.stopRanging();  // Stop any ongoing ranging
    delay(50);           // Give sensor time to stop
    
    if (DEBUG) Serial.println("2. Clearing interrupts and starting new ranging...");
    vl53.clearInterrupt();  // Clear any pending interrupts
    vl53.startRanging();    // Start fresh ranging
    
    // Wait for fresh data (with timeout)
    if (DEBUG) Serial.println("3. Waiting for fresh data...");
    uint32_t start = millis();
    bool validReading = false;
    while ((millis() - start) < 1000) { // Increased timeout to 1000ms
        if (vl53.dataReady()) {
            int16_t distance = vl53.distance();
            if (distance != -1) {
                if (DEBUG) {
                    Serial.println("4. Fresh data received:");
                    Serial.printf("   - Distance: %d mm\n", distance);
                    Serial.printf("   - Time taken: %lu ms\n", millis() - start);
                }
                collected_data[data_len++] = (distance >> 8) & 0xFF;
                collected_data[data_len++] = distance & 0xFF;
                validReading = true;
            } else if (DEBUG) {
                Serial.println("   - Invalid distance reading (-1)");
            }
            vl53.clearInterrupt();
            break;
        }
        delay(10);
    }
    vl53.stopRanging();
    
    if (!validReading && DEBUG) {
        Serial.println("Failed to get valid TOF reading - Timeout occurred");
    }

    if (DEBUG) Serial.println("\nAHT20 Sensor Operations:");

    // AHT20 operations - Already gets fresh readings by design
    sensors_event_t humidity, temp;
    if (aht.getEvent(&humidity, &temp)) {
        int16_t temp_scaled = (int16_t)(temp.temperature * 10);
        collected_data[data_len++] = (temp_scaled >> 8) & 0xFF;
        collected_data[data_len++] = temp_scaled & 0xFF;

        int16_t hum_scaled = (int16_t)(humidity.relative_humidity * 10);
        collected_data[data_len++] = (hum_scaled >> 8) & 0xFF;
        collected_data[data_len++] = hum_scaled & 0xFF;

        if (DEBUG) {
            Serial.println("Temperature and Humidity readings:");
            Serial.printf("   - Temperature: %.1f °C\n", temp.temperature);
            Serial.printf("   - Humidity: %.1f %%\n", humidity.relative_humidity);
        }
    } else if (DEBUG) {
        Serial.println("Failed to read AHT20 sensor");
    }

    // Send the packet
    if (DEBUG) Serial.println("\nLoRaWAN Operations:");
    if (api.lorawan.send(data_len, collected_data, 2, false, 1)) {
        if (DEBUG) {
            Serial.println("Packet sent successfully");
            Serial.printf("Payload size: %d bytes\n", data_len);
        }
        tx_active = true;
    } else {
        if (DEBUG) Serial.println("Failed to send message");
        tx_active = false;
    }
    
    if (DEBUG) Serial.println("--- End of measurement cycle ---\n");
}

void setup()
{
    Serial.begin(115200);
    delay(1000);

    // Configure the LED pin
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // Initialize I2C and sensors
    Wire.begin();
    if (!vl53.begin(0x29, &Wire)) {
        Serial.println("Failed to initialize VL53L1X");
        while(1);
    }
    if (!aht.begin()) {
        Serial.println("Failed to initialize AHT20");
        while(1);
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