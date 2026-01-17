#include <Arduino.h>
#include <esp_sleep.h>
#include <Wire.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <DFRobot_OxygenSensor.h>

#define MAX_TIMEOUT_COUNT 3
#define DFR_OXY_ADDR 0x76
#define BME_280_ADDR 0x77

// SERVICE AND CHARACTERISTIC UUIDs -----
#define SENSOR_SERVICE_UUID "A870DC1B-0265-4D5F-9A21-8AC5BD2BACD7"

#define TEMP_CHARACTERISTIC_UUID  "A07038DF-7C8E-4914-87B3-131B91DAAB73"
#define PRES_CHARACTERISTIC_UUID  "594BF212-A4FC-4130-ACB1-8FD4FD28EFD3"
#define HUMID_CHARACTERISTIC_UUID "72A7B435-989D-4369-8F58-D6E98B4AB262"
#define OXY_CHARACTERISTIC_UUID   "759E38A8-BB58-4F70-96EB-A4BDCEC3977A"

#define SLEEP_TIME_SERVICE_UUID   "9D818D7B-A445-46F5-8A3F-B9F86EA5DE2F"
#define SLEEP_TIME_CHARACTERISTIC_UUID "CEF11275-083B-4027-AD0E-0DDB904278A5"

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
  PREPARE_SLEEP,
  BROKEN_LINK
} SlaveState;

// SENSOR OBJECTS -----------------------
Adafruit_BME280 bme; // I2C
DFRobot_OxygenSensor o2sensor; // I2C

// GLOBALS ------------------------------
float humidity = NAN;
float temperature = NAN;
float pressure = NAN;
float oxygen = NAN;
bool has_oxy_sensor = false; // Assume false; set true if O2 sensor is present

bool temp_accessed = false;
bool humid_accessed = false;
bool oxy_accessed = false;
bool sleep_time_accessed = false;
bool BLE_device_connected = false;

// PERSISTENT STATE ---------------------
RTC_DATA_ATTR int TIMEOUT_COUNTER = 0;
RTC_DATA_ATTR uint64_t PROVIDED_SLEEP_DURATION = 30 * 60 * 1000 * 1000; // default is 30 minutes in µ-seconds

// FUNCTION PROTOTYPES ------------------
void init_BLE();

// BLUETOOTH OBJECTS --------------------
BLEServer *pServer = nullptr;

BLEService *pSensingService = nullptr;
BLECharacteristic *pTempCharacteristic = nullptr;
BLECharacteristic *pHumidCharacteristic = nullptr;
BLECharacteristic *pPressCharacteristic = nullptr;
BLECharacteristic *pOxyCharacteristic = nullptr;

BLEService *pSleepTimeService = nullptr;
BLECharacteristic *pSleepTimeCharacteristic = nullptr;

BLEAdvertising *pAdvert = nullptr;
BLEAdvertisementData *pAdvData = nullptr;

// BLUETOOTH CALLBACKS ------------------
class ServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer *serverPointer) {
    BLE_device_connected = true;
    DEBUG_PRINTLN("[BLE] device connected!");
  }

  void onDisconnect(BLEServer *serverPointer) {
    BLE_device_connected = false;
    DEBUG_PRINTLN("[BLE] device disconnected!");
  }
};

class SensorCallbacks: public BLECharacteristicCallbacks {
  void onRead(BLECharacteristic *characteristicPointer) {
    if (characteristicPointer == pTempCharacteristic) {
      temp_accessed = true;
      DEBUG_PRINTLN("[BLE] Temperature Accessed");
    }
    else if (characteristicPointer == pHumidCharacteristic) {
      humid_accessed = true;
      DEBUG_PRINTLN("[BLE] Humidity Accessed");
    }
    else if (characteristicPointer == pOxyCharacteristic) {
      oxy_accessed = true;
      DEBUG_PRINTLN("[BLE] Cx Oxygen Accessed");
    } else {
      DEBUG_PRINT("[BLE] Accessed unidentified characteristic with UUID: ");
      const char *uuid = characteristicPointer->getUUID().toString().c_str();
      DEBUG_PRINTLN(uuid);
    }
  }
};

class SleepTimeCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *characteristicPointer) {
    sleep_time_accessed = true;
  }
};


void setup() {
  // Initialize sensors and variables

  // init Serial for debugging only
  # ifdef DEBUG
    Serial.begin(9600);
    while (!Serial) ;
  # endif

  DEBUG_PRINTLN("Starting up...");
  // init I2C as master
  DEBUG_PRINTLN("Initializing I2C...");
  if (!Wire.begin()) {
    DEBUG_PRINTLN("[I2C] Failed to initialize I2C bus!");
  }

  // init BME280 sensor
  DEBUG_PRINTLN("[I2C] Connecting to temperature and humidity sensor...");
  if (!bme.begin(BME_280_ADDR)) {  
    DEBUG_PRINTLN("[I2C] Could not find a valid BME280 sensor.");
  } else {
    DEBUG_PRINTLN("[I2C] BME280 sensor connected.");
    humidity = bme.readHumidity();  // % of humidity
    temperature = bme.readTemperature();  // Celcius
    pressure = bme.readPressure(); // Pascals
  }
  
  // init
  DEBUG_PRINTLN("[I2C] Connecting to oxygen sensor...");
  if (o2sensor.begin(ADDRESS_3)) {
    oxygen = o2sensor.getOxygenData(10); // Preload some data
    has_oxy_sensor = true;
  }
  if (!has_oxy_sensor) {
    DEBUG_PRINTLN("[I2C] Oxygen sensor not found.");
  }

  // init BLE server
  DEBUG_PRINTLN("Setting up BLE...");
  init_BLE();

  DEBUG_PRINTLN("[BLE] Loading characteristic values...");
  pTempCharacteristic->setValue(temperature);
  pHumidCharacteristic->setValue(humidity);
  pPressCharacteristic->setValue(pressure);
  pOxyCharacteristic->setValue(oxygen);

  DEBUG_PRINTLN("[BLE] Starting Server...");
  pSensingService->start();
  pSleepTimeService->start();

  DEBUG_PRINTLN("[BLE] Starting advertising...");
  pAdvert = BLEDevice::getAdvertising();
  pAdvData = new BLEAdvertisementData();
  pAdvData->setName(DEVICE_NAME "_" DEVICE_ID);
  pAdvData->setShortName("S_" DEVICE_ID);
  // pAdvData->addServiceUUID(BLEUUID(SENSOR_SERVICE_UUID));
  pAdvData->setCompleteServices(BLEUUID(SENSOR_SERVICE_UUID));
  //cc,rc
  pAdvert->setAdvertisementData(*pAdvData);
  pAdvert->addServiceUUID(SENSOR_SERVICE_UUID);

  pAdvert->setScanResponse(true);
  pAdvert->setMinPreferred(0x06);
  pAdvert->setMinPreferred(0x12);

  BLEDevice::startAdvertising();

  DEBUG_PRINTLN("--- Finished setup !!! ---");
}

void loop() {
  SlaveState currentState = LISTEN;
  int32_t timer_start_time = millis();
  while (1) {
    switch(currentState) {
      case LISTEN:
        // Check for timeout
        if (millis() - timer_start_time > PROVIDED_SLEEP_DURATION) { // Example timeout
          TIMEOUT_COUNTER++;
          DEBUG_PRINT("[LISTEN] TIMEOUT_COUNTER = ");
          DEBUG_PRINTLN(TIMEOUT_COUNTER);
          timer_start_time = millis();
        }
        // Check for incoming BLE requests
        if(BLE_device_connected) {
          DEBUG_PRINTLN("[LISTEN] Device detected, awaiting data retreival...");
          currentState = SEND_DATA;
          TIMEOUT_COUNTER = 0; // Reset on successful access
          BLEDevice::stopAdvertising();
        } else if (TIMEOUT_COUNTER >= MAX_TIMEOUT_COUNT) {
          DEBUG_PRINTLN("[LISTEN] Too many timeouts, shutting down indefinitely...");
          currentState = BROKEN_LINK;
        }
        break;
      
      case SEND_DATA:
        if (millis() - timer_start_time > PROVIDED_SLEEP_DURATION) { // Example timeout
          TIMEOUT_COUNTER++;
          DEBUG_PRINT("[SEND_DATA] TIMEOUT_COUNTER = ");
          DEBUG_PRINTLN(TIMEOUT_COUNTER);
          timer_start_time = millis();
        }
        if (temp_accessed && humid_accessed && (!has_oxy_sensor || oxy_accessed)) {
          DEBUG_PRINTLN("[SEND_DATA] All the data was read.");
          TIMEOUT_COUNTER = 0; // Reset on successful access
          currentState = PREPARE_SLEEP;
        } else if (TIMEOUT_COUNTER >= MAX_TIMEOUT_COUNT) {
          DEBUG_PRINTLN("[SEND_DATA] Too many timeouts, shutting down indefinitely...");
          currentState = BROKEN_LINK;
        }
        break;

      case PREPARE_SLEEP:
        if (millis() - timer_start_time > PROVIDED_SLEEP_DURATION) {
          TIMEOUT_COUNTER++;
          DEBUG_PRINT("[PREPARE_SLEEP] TIMEOUT_COUNTER = ");
          DEBUG_PRINTLN(TIMEOUT_COUNTER);
          timer_start_time = millis();
        }

        if (sleep_time_accessed) {
          DEBUG_PRINTLN("[PREPARE_SLEEP] Sleep duration written");
          TIMEOUT_COUNTER = 0; // Reset on successful access
          const char *string_value = pSleepTimeCharacteristic->getValue().c_str();
          uint64_t sleep_duration = (uint64_t)strtoull(string_value, NULL, 16);
          DEBUG_PRINT("sleep time value ");
          DEBUG_PRINTLN(sleep_duration);

          pSensingService->stop();
          pSleepTimeService->stop();
          
          BLEDevice::deinit();

          esp_sleep_enable_timer_wakeup(sleep_duration);
          esp_deep_sleep_start();
        } else if (TIMEOUT_COUNTER >= MAX_TIMEOUT_COUNT) {
          DEBUG_PRINTLN("[PREPARE_SLEEP] Too many timeouts, shutting down indefinitely...");
          currentState = BROKEN_LINK;
        }
        break;

      case BROKEN_LINK:
        TIMEOUT_COUNTER = 0;
        esp_deep_sleep_start();
        break;

      default:
        currentState = LISTEN;
        break;
    }
  }
}


void init_BLE() {
  // Initialize BLE server and characteristics
  DEBUG_PRINTLN("[BLE] Initializing BLE Device");
  BLEDevice::init(DEVICE_NAME "_" DEVICE_ID);
  DEBUG_PRINTLN("[BLE] Creating Server...");
  pServer = BLEDevice::createServer();
  DEBUG_PRINTLN("[BLE] Adding server callbacks...");
  pServer->setCallbacks(new ServerCallbacks());
  DEBUG_PRINTLN("[BLE] Creating sensor service...");
  pSensingService = pServer->createService(SENSOR_SERVICE_UUID);

  DEBUG_PRINTLN("[BLE] Creating sensor characteristics...");
  pTempCharacteristic = pSensingService->createCharacteristic(TEMP_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ);
  pHumidCharacteristic = pSensingService->createCharacteristic(HUMID_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ);
  pOxyCharacteristic = pSensingService->createCharacteristic(OXY_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ);
  pPressCharacteristic = pSensingService->createCharacteristic(PRES_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ);
  
  DEBUG_PRINTLN("[BLE] Adding sensor callbacks...");
  pTempCharacteristic->setCallbacks(new SensorCallbacks());
  pHumidCharacteristic->setCallbacks(new SensorCallbacks());
  pOxyCharacteristic->setCallbacks(new SensorCallbacks());
  // No Pressure characteristic Callback because not relevant

  DEBUG_PRINTLN("[BLE] Creating sleep time service...");
  pSleepTimeService = pServer->createService(SLEEP_TIME_SERVICE_UUID);
  DEBUG_PRINTLN("[BLE] Creating sleep time characteristic...");
  pSleepTimeCharacteristic = pSleepTimeService->createCharacteristic(SLEEP_TIME_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_WRITE);
  pSleepTimeCharacteristic->setCallbacks(new SleepTimeCallbacks());

  DEBUG_PRINTLN("[BLE] Finished initializing.");
}