#include <Arduino.h>
#include <Wire.h>
#include <NimBLEDevice.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <DFRobot_OxygenSensor.h>

#define MAX_TIMEOUT_COUNT 3
#define DFR_OXY_ADDR 0x73

#ifndef DEVICE_ID
  #define DEVICE_ID "1"
#endif
#define DEVICE_NAME "EnvSensor"

#ifdef DEBUG
  #define DEBUG_PRINT(x)  Serial.print(x)
  #define DEBUG_PRINTLN(x)  Serial.println(x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif

// TYPE DEFINITIONS ---------------------
typedef enum {
  LISTEN,
  SEND_DATA,
  SLEEP,
  BROKEN_LINK
} SlaveState;

// SENSOR OBJECTS -----------------------
Adafruit_BME280 bme; // I2C
DFRobot_OxygenSensor o2sensor; // I2C

// BLE SERVER OBJECT --------------------
static NimBLEServer* pServer = nullptr;
static NimBLECharacteristic* pTempCharacteristic = nullptr;
static NimBLECharacteristic* pHumidCharacteristic = nullptr;
static NimBLECharacteristic* pOxyCharacteristic = nullptr;

RTC_DATA_ATTR int TIMEOUT_COUNTER = 0;
RTC_DATA_ATTR int PROVIDED_SLEEP_DURATION = 1800000; // default is 30 minutes in milliseconds

bool has_oxy_sensor = false; // Assume false; set true if O2 sensor is present

// FUNCTION PROTOTYPES -------------------
void init_BLE();
bool temp_accessed();
bool humid_accessed();
bool oxy_accessed();
bool sleep_time_accessed();
bool BLE_connection_established();


void setup() {
  // Initialize sensors and variables

  // init Serial for debugging only
  # ifdef DEBUG
    Serial.begin(9600);
    while (!Serial) ;
  # endif

  DEBUG_PRINTLN("Starting up...");
  DEBUG_PRINTLN("WIP");
  Serial.println("after debug");

  // init I2C as master
  DEBUG_PRINTLN("Initializing I2C...");
  // TODO: Set correct SDA and SCL pins
  if (!Wire.begin()) {
    DEBUG_PRINTLN("Failed to initialize I2C bus!");
  }

  // init BME280 sensor
  DEBUG_PRINTLN("[I2C] Connecting to temperature and humidity sensor...");
  if (!bme.begin(0x76)) {  
    DEBUG_PRINTLN("Could not find a valid BME280 sensor.");
  } else {
    DEBUG_PRINTLN("BME280 sensor connected.");
  }
  
  // init check all 4 possible addresses
  DEBUG_PRINTLN("[I2C] Connecting to oxygen sensor...");
  for (int i = 0; i < 4; i++) {
    if (o2sensor.begin(ADDRESS_0 + i)) {
      # ifdef DEBUG
        DEBUG_PRINT("Oxygen sensor found at ADDRESS_");
        DEBUG_PRINTLN(i);
        DEBUG_PRINTLN("Oxygen sensor connected.");
        o2sensor.calibrate(20.9); // Calibrate at 20.9% O2
        float cx = o2sensor.getOxygenData(10); // Preload some data
        DEBUG_PRINT("Oxygen concentration: ");
        DEBUG_PRINTLN(cx);
      # endif
      has_oxy_sensor = true;
    }
  }
  if (!has_oxy_sensor) {
    DEBUG_PRINTLN("Oxygen sensor not found.");
  }

  // init BLE server
  DEBUG_PRINTLN("Initializing BLE...");

  // create BLE environmental sensing service
    // create BLE temperature characteristic
    // create BLE humidity characteristic
    // create BLE oxygen characteristic

  // start BLE service
  // start advertising
  // TODO: remember to set device name with the same value as local name
}

void loop() {
  delay(1000);
  DEBUG_PRINTLN("LOOP");
  SlaveState currentState = LISTEN;
  int32_t timer_start_time = millis();
  /*
  while (1) {
    switch(currentState) {
      case LISTEN:
        // Check for timeout
        if (millis() - timer_start_time > PROVIDED_SLEEP_DURATION) { // Example timeout
          TIMEOUT_COUNTER++;
          timer_start_time = millis();
        }
        // Check for incoming BLE requests
        if(BLE_connection_established()) {
          currentState = SEND_DATA;
          TIMEOUT_COUNTER = 0; // Reset on successful access
        } else if (TIMEOUT_COUNTER >= MAX_TIMEOUT_COUNT) {
          currentState = BROKEN_LINK;
        }
        break;
      
      case SEND_DATA:
        if (millis() - timer_start_time > PROVIDED_SLEEP_DURATION) { // Example timeout
          TIMEOUT_COUNTER++;
          timer_start_time = millis();
        }
        if (temp_accessed() && humid_accessed() && (!has_oxy_sensor || oxy_accessed())) {
          TIMEOUT_COUNTER = 0; // Reset on successful access
          currentState = SLEEP;
        } else if (TIMEOUT_COUNTER >= MAX_TIMEOUT_COUNT) {
          currentState = BROKEN_LINK;
        }
        break;

      case SLEEP:
        if (millis() - timer_start_time > PROVIDED_SLEEP_DURATION) { // Example timeout
          TIMEOUT_COUNTER++;
          timer_start_time = millis();
        }

        if (sleep_time_accessed()) {
          TIMEOUT_COUNTER = 0; // Reset on successful access
          // Prepare for deep sleep
          // set sleep duration
          // enter deep sleep
        } else if (TIMEOUT_COUNTER >= MAX_TIMEOUT_COUNT) {
          currentState = BROKEN_LINK;
        }
        break;

      case BROKEN_LINK:
        // Handle broken link scenario
        // Reset TIMEOUT_COUNTER
        TIMEOUT_COUNTER = 0;
        // shutdown ESP32
        break;

      default:
        currentState = LISTEN;
        break;
    }
  }
  */
}

void init_BLE() {
  // Initialize BLE server and characteristics
  NimBLEDevice::init(DEVICE_NAME "_" DEVICE_ID);
  pServer = NimBLEDevice::createServer();

  BLEService *pEnvSensorService = pServer->createService("181A"); // Environmental Sensing Service UUID
}