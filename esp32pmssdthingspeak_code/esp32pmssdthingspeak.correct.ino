#include <WiFi.h>
#include "time.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <HTTPClient.h>
#include <HardwareSerial.h>

// Wi-Fi credentials
const char* ssid = "CANALBOX-6098-2G";
const char* password = "w38C68ugkg";

// NTP server and time zone settings
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 7200;  // Rwanda is UTC+2
const int daylightOffset_sec = 0; // No daylight saving in Rwanda

// Define the pins for PMS5003 communication
const int PMS5003_RX_PIN = 16;  // Connect PMS5003 TX to ESP32 GPIO 16 (RX2)
const int PMS5003_TX_PIN = 17;  // Connect PMS5003 RX to ESP32 GPIO 17 (TX2)

// ThingSpeak API information
String serverName = "http://api.thingspeak.com/update?api_key=D436HWEOYM9W9UZR";

// Define the pins for the LEDs
const int SUCCESS_LED_PIN = 2;  // Connect the LED to GPIO 2 for success indication
const int FAILURE_LED_PIN = 4;  // Connect the LED to GPIO 4 for failure indication

HardwareSerial pmsSerial(2); // Use Serial2 for communication with PMS5003

struct pms5003data {
  uint16_t framelen;
  uint16_t pm10_standard, pm25_standard, pm100_standard;
  uint16_t pm10_env, pm25_env, pm100_env;
  uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
  uint16_t unused;
  uint16_t checksum;
};

struct pms5003data data;
unsigned long previousMillis = 0;
const long interval = 60000;  // 1 minute interval for reconnection attempts

// Function to reconnect to Wi-Fi if disconnected
void checkWiFiConnection() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Reconnecting to WiFi...");
        WiFi.disconnect();
        WiFi.begin(ssid, password);
        while (WiFi.status() != WL_CONNECTED) {
            delay(500);
            Serial.print(".");
        }
        Serial.println("\nWiFi reconnected.");
    }
}

// Function to reconnect PMS5003 if not responding
void checkPMS5003Connection() {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        if (!readPMSdata(&pmsSerial)) {
            Serial.println("PMS5003 sensor not responding. Reinitializing...");
            pmsSerial.end();
            delay(1000);
            pmsSerial.begin(9600, SERIAL_8N1, PMS5003_RX_PIN, PMS5003_TX_PIN);
            delay(6000);  // Sensor warm-up time
        }
    }
}

// Function to write data to SD card
void writeFile(fs::FS &fs, const char *path, const char *message) {
    Serial.printf("Appending to file: %s\n", path);
    File file = fs.open(path, FILE_APPEND);
    if (!file) {
        Serial.println("Failed to open file for appending");
        digitalWrite(FAILURE_LED_PIN, HIGH); // Turn on failure LED
        return;
    }
    if (file.print(message)) {
        Serial.println("Message appended");
        digitalWrite(SUCCESS_LED_PIN, HIGH); // Turn on success LED
        digitalWrite(FAILURE_LED_PIN, LOW);  // Turn off failure LED
    } else {
        Serial.println("Append failed");
        digitalWrite(FAILURE_LED_PIN, HIGH); // Turn on failure LED
        digitalWrite(SUCCESS_LED_PIN, LOW);  // Turn off success LED
    }
    file.close();
}

// Function to publish data to ThingSpeak
void publishToThingSpeak(float pm10Env, float pm25Env, float pm100Env) {
    if (WiFi.status() == WL_CONNECTED) {
        WiFiClient client;
        HTTPClient http;
        
        String serverPath = serverName + "&field1=" + String(pm10Env) + "&field2=" + String(pm25Env) + "&field3=" + String(pm100Env);
        
        http.begin(client, serverPath.c_str());
        int httpResponseCode = http.GET();
        
        if (httpResponseCode > 0) {
            Serial.print("HTTP Response code: ");
            Serial.println(httpResponseCode);
            String payload = http.getString();
            Serial.println(payload);
        } else {
            Serial.print("Error code: ");
            Serial.println(httpResponseCode);
        }
        http.end();
    } else {
        Serial.println("WiFi Disconnected");
    }
}

void setup() {
    Serial.begin(115200);
    
    if (!SD.begin(5)) {
        Serial.println("Card Mount Failed");
        return;
    }

    // Connect to Wi-Fi
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected.");
  
    // Initialize and sync time with the NTP server
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  
    // Wait for a few seconds to sync time
    delay(5000);
    Serial.println("Time synced.");

    // Initialize PMS5003 serial communication
    pmsSerial.begin(9600, SERIAL_8N1, PMS5003_RX_PIN, PMS5003_TX_PIN);
  
    // Allow sensor to stabilize
    delay(6000);  // 60 seconds delay for warm-up
  
    // Initialize the LED pins
    pinMode(SUCCESS_LED_PIN, OUTPUT);
    pinMode(FAILURE_LED_PIN, OUTPUT);
    digitalWrite(SUCCESS_LED_PIN, LOW);  // Ensure the success LED is off initially
    digitalWrite(FAILURE_LED_PIN, LOW);  // Ensure the failure LED is off initially
}

void loop() {
    static int count = 0; // Counter to determine when to store data
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
        Serial.println("Failed to obtain time");
        return;
    }

    // File path for storing data
    String filePath = "/data_day.txt";

    checkWiFiConnection();
    checkPMS5003Connection();

    if (readPMSdata(&pmsSerial)) {
        // Convert PMS5003 data to float values
        float pm10Env = data.pm10_env;
        float pm25Env = data.pm25_env;
        float pm100Env = data.pm100_env;

        // Create the data message with the current time
        String dataMessage = String(timeinfo.tm_mday) + "/" + String(timeinfo.tm_mon + 1) + "/" + String(timeinfo.tm_year + 1900) + " " +
                             String(timeinfo.tm_hour) + ":" + String(timeinfo.tm_min) + ":" + String(timeinfo.tm_sec) + ", " +
                             String(pm10Env, 2) + ", " + String(pm25Env, 2) + ", " + String(pm100Env, 2) + "\n";

        // Print data to Serial Monitor every second
        Serial.print(dataMessage);

        // Increment counter
        count++;

        // Store data on SD card every 5 seconds
        if (count >= 5) {
            writeFile(SD, filePath.c_str(), dataMessage.c_str());
            count = 0; // Reset counter

            // Publish data to ThingSpeak every 5 seconds
            publishToThingSpeak(pm10Env, pm25Env, pm100Env);
        }

        // Turn the success LED on to indicate success
        digitalWrite(SUCCESS_LED_PIN, HIGH);
    } else {
        Serial.println("Failed to read data from PMS5003");
        digitalWrite(FAILURE_LED_PIN, HIGH);  // Turn the failure LED on to indicate failure
        digitalWrite(SUCCESS_LED_PIN, LOW);   // Turn off the success LED
    }

    delay(1000);  // Delay between reads (1 second)
}

boolean readPMSdata(Stream *s) {
    if (!s->available()) {
        return false;
    }

    // Wait until we get the start byte '0x42'
    while (s->available() > 0 && s->peek() != 0x42) {
        s->read();  // Discard bytes until the start byte is found
    }

    if (s->available() < 32) {
        return false;
    }

    uint8_t buffer[32];
    s->readBytes(buffer, 32);  // Read all 32 bytes

    uint16_t sum = 0;
    for (uint8_t i = 0; i < 30; i++) {
        sum += buffer[i];
    }

    uint16_t buffer_u16[15];
    for (uint8_t i = 0; i < 15; i++) {
        buffer_u16[i] = (buffer[2 + i * 2] << 8) | buffer[2 + i * 2 + 1];
    }

    if (sum != buffer_u16[14]) {
        Serial.println("Checksum mismatch");
        return false;
    }

    // Populate the struct with the correct data
    data.pm10_standard = buffer_u16[0];
    data.pm25_standard = buffer_u16[1];
    data.pm100_standard = buffer_u16[2];

    data.pm10_env = buffer_u16[4];   // Corrected index for pm10_env
    data.pm25_env = buffer_u16[5];   // Corrected index for pm25_env
    data.pm100_env = buffer_u16[3];  // Corrected index for pm100_env

    data.particles_03um = buffer_u16[6];
    data.particles_05um = buffer_u16[7];
    data.particles_10um = buffer_u16[8];
    data.particles_25um = buffer_u16[9];
    data.particles_50um = buffer_u16[10];
    data.particles_100um = buffer_u16[11];

    return true;
}
