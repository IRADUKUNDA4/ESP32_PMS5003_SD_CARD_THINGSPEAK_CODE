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
const long gmtOffset_sec = 7200; // Rwanda is UTC+2
const int daylightOffset_sec = 0; // No daylight saving in Rwanda

// Define the pins for PMS5003 communication
const int PMS5003_RX_PIN = 16;  // Connect PMS5003 TX to ESP32 GPIO 16 (RX2)
const int PMS5003_TX_PIN = 17;  // Connect PMS5003 RX to ESP32 GPIO 17 (TX2)

// Define the analog pin connected to the NO2 sensor output
const int no2SensorPin = 34;  // GPIO34, adjust if using a different pin

// ThingSpeak API information
String serverName = "http://api.thingspeak.com/update?api_key=D436HWEOYM9W9UZR";

// Define the pin for the LED
const int LED_PIN = 2;  // Connect the LED to GPIO 2
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

void createDir(fs::FS &fs, const char *path) {
    Serial.printf("Creating Dir: %s\n", path);
    if(fs.mkdir(path)){
        Serial.println("Dir created");
    } else {
        Serial.println("mkdir failed or directory already exists");
    }
}

void writeFile(fs::FS &fs, const char *path, const char *message) {
    Serial.printf("Appending to file: %s\n", path);
    File file = fs.open(path, FILE_APPEND);
    if(!file){
        Serial.println("Failed to open file for appending");
        return;
    }
    if(file.print(message)){
        Serial.println("Message appended");
    } else {
        Serial.println("Append failed");
    }
    file.close();
}

// Function to publish data to ThingSpeak
void publishToThingSpeak(float pm10Env, float pm25Env, float pm100Env, float no2ppm) {
    if (WiFi.status() == WL_CONNECTED) {
        WiFiClient client;
        HTTPClient http;
        
        String serverPath = serverName + "&field1=" + String(pm10Env) + "&field2=" + String(pm25Env) + "&field3=" + String(pm100Env) + "&field4=" + String(no2ppm);
        
        http.begin(client, serverPath.c_str());
        int httpResponseCode = http.GET();
        
        if (httpResponseCode > 0) {
            Serial.print("HTTP Response code: ");
            Serial.println(httpResponseCode);
            String payload = http.getString();
            Serial.println(payload);
            // Turn the LED on to indicate success
            digitalWrite(LED_PIN, HIGH);
            digitalWrite(FAILURE_LED_PIN, LOW);  // Ensure failure LED is off
        } else {
            Serial.print("Error code: ");
            Serial.println(httpResponseCode);
            // Turn the failure LED on to indicate failure
            digitalWrite(FAILURE_LED_PIN, HIGH);
            digitalWrite(LED_PIN, LOW);  // Ensure success LED is off
        }
        http.end();
    } else {
        Serial.println("WiFi Disconnected");
    }
}

void setup() {
    Serial.begin(115200);
    
    if(!SD.begin(5)){
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
    delay(60000);  // 60 seconds delay for warm-up
  
    // Initialize the LED pins
    pinMode(LED_PIN, OUTPUT);
    pinMode(FAILURE_LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);  // Ensure the LED is off initially
    digitalWrite(FAILURE_LED_PIN, LOW);  // Ensure the failure LED is off initially

    // Set ADC resolution for NO2 sensor
    analogReadResolution(12);  // Set the ADC resolution to 12 bits (0-4095)
}

void loop() {
    static int count = 0; // Counter to determine when to store data
    struct tm timeinfo;
    if(!getLocalTime(&timeinfo)){
        Serial.println("Failed to obtain time");
        return;
    }

    // Get the current hour, minute, second, year, and date
    int hours = timeinfo.tm_hour;
    int minutes = timeinfo.tm_min;
    int seconds = timeinfo.tm_sec;
    int year = timeinfo.tm_year + 1900;  // tm_year is years since 1900
    int month = timeinfo.tm_mon + 1;     // tm_mon is months since January (0-11)
    int day = timeinfo.tm_mday;

    // Create directory paths based on the current year
    String backupDir = "/backup/" + String(year);
    String backupFilePath = backupDir + "/" + String(year) + String(month) + String(day) + ".txt";

    // Live folder
    String liveDir = "/live_folder";
    String liveFilePath = liveDir + "/live_data.txt";

    // Ensure the backup and live directories are created
    createDir(SD, "/backup");
    createDir(SD, backupDir.c_str());
    createDir(SD, liveDir.c_str());

    if (readPMSdata(&pmsSerial)) {
        // Convert PMS5003 data to float values
        float pm10Env = data.pm10_env;
        float pm25Env = data.pm25_env;
        float pm100Env = data.pm100_env;

        // Read NO2 sensor value
        int no2SensorValue = analogRead(no2SensorPin);  // Read the sensor value from the ADC
        float no2Voltage = no2SensorValue * (3.3 / 4095.0);  // Convert ADC value to voltage

        // Calibration formula for NO2 concentration (adjust based on your sensor's datasheet)
        float no2ppm = (no2Voltage - 0.1) * (100.0 / (3.3 - 0.1));  // Example calibration formula

        // Ensure that no2ppm is not negative
        if (no2ppm < 0) {
            no2ppm = 0;
        }

        // Create the data message with the current time
        String dataMessage = String(day) + "/" + String(month) + "/" + String(year) + " " + 
                             String(hours) + ":" + String(minutes) + ":" + String(seconds) + ", " +
                             String(pm10Env, 2) + ", " + String(pm25Env, 2) + ", " + 
                             String(pm100Env, 2) + ", " + String(no2ppm, 2) + "\n";
                             
        Serial.print(dataMessage);
                             
        count++;

        if (count >= 5) {
            // Write the data message to both the backup and live files
            writeFile(SD, backupFilePath.c_str(), dataMessage.c_str());
            writeFile(SD, liveFilePath.c_str(), dataMessage.c_str());
            count = 0; // Reset counter
            // Publish data to ThingSpeak every 5 readings
            publishToThingSpeak(pm10Env, pm25Env, pm100Env, no2ppm);
        }
    } else {
        Serial.println("Failed to read data from PMS5003");
        digitalWrite(FAILURE_LED_PIN, HIGH);  // Turn the failure LED on to indicate failure
        digitalWrite(LED_PIN, LOW);  // Ensure success LED is off
    }

    delay(1000);  // Delay between reads
}

boolean readPMSdata(Stream *s) {
    if (!s->available()) {
        return false;
    }

    // Wait until we get the start byte '0x42'
    while (s->available() > 0 && s->peek() != 0x42) {
        s->read();
    }
    if (s->available() < 32) {
        return false;
    }
  
    if (s->read() != 0x42 || s->read() != 0x4d) {
        return false;
    }

    uint16_t length = (s->read() << 8) | s->read();
    if (length != 28) {
        return false;
    }

    data.framelen = length;
    data.pm10_standard = (s->read() << 8) | s->read();
    data.pm25_standard = (s->read() << 8) | s->read();
    data.pm100_standard = (s->read() << 8) | s->read();
    data.pm10_env = (s->read() << 8) | s->read();
    data.pm25_env = (s->read() << 8) | s->read();
    data.pm100_env = (s->read() << 8) | s->read();
    data.particles_03um = (s->read() << 8) | s->read();
    data.particles_05um = (s->read() << 8) | s->read();
    data.particles_10um = (s->read() << 8) | s->read();
    data.particles_25um = (s->read() << 8) | s->read();
    data.particles_50um = (s->read() << 8) | s->read();
    data.particles_100um = (s->read() << 8) | s->read();
    data.unused = (s->read() << 8) | s->read();
    data.checksum = (s->read() << 8) | s->read();

    return true;
}

