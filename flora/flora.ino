#include "BLEDevice.h"
#include <EEPROM.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <PubSubClient.h>
#include <Syslog.h>
#include "config.h"


// device count
static int deviceCount = sizeof FLORA_DEVICES / sizeof FLORA_DEVICES[0];

// the remote service we wish to connect to
static BLEUUID serviceUUID("00001204-0000-1000-8000-00805f9b34fb");

// the characteristic of the remote service we are interested in
static BLEUUID uuid_version_battery("00001a02-0000-1000-8000-00805f9b34fb");
static BLEUUID uuid_sensor_data("00001a01-0000-1000-8000-00805f9b34fb");
static BLEUUID uuid_write_mode("00001a00-0000-1000-8000-00805f9b34fb");

TaskHandle_t hibernateTaskHandle = NULL;

int bootCount;
int deviceSeq;

WiFiClient espClient;
PubSubClient client(espClient);
WiFiUDP udpClient;

Syslog syslog(udpClient, SYSLOG_SERVER, SYSLOG_PORT, DEVICE_HOSTNAME, NULL, LOG_KERN);

void connectWifi() {
  Serial.println("Connecting to WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("WiFi connected with IP address: ");
  Serial.println(WiFi.localIP());
}

void disconnectWifi() {
  WiFi.disconnect(true);
  Serial.println("WiFi disconnected");
}

void connectMqtt() {
  while (!client.connected()) {
    if (!client.connect(MQTT_CLIENTID, MQTT_USERNAME, MQTT_PASSWORD)) {
      Serial.print("MQTT connection failed:");
      Serial.print(client.state());
      Serial.println("Retrying...");
      delay(MQTT_RETRY_WAIT);
    } else {
      Serial.println("MQTT connected");
    }
  }
  client.loop();
}

void disconnectMqtt() {
  client.disconnect();
  Serial.println("MQTT disconnected");
}

BLEClient* getFloraClient(BLEAddress floraAddress, char* addr, int cnt, int retry) {
  BLEClient* floraClient = BLEDevice::createClient();

  Serial.print("- Connecting to the device ");
  Serial.println(addr);
  #ifdef DEBUG
  syslog.logf(LOG_DEBUG, "%s (C:%d - R:%d) -> Connecting to the device", addr, cnt, retry);
  #endif

  if (!floraClient->connect(floraAddress)) {
    Serial.println("- Connection failed, skipping");
    syslog.logf(LOG_WARNING, "%s (C:%d - R:%d) -> Connection to the device failed", addr, cnt, retry);
    return nullptr;
  }

  Serial.println("- Connection successful");
  syslog.logf(LOG_DEBUG, "%s (C:%d - R:%d) -> Connection to the device successful", addr, cnt, retry);
  return floraClient;
}

BLERemoteService* getFloraService(BLEClient* floraClient, char* addr, int cnt, int retry) {
  BLERemoteService* floraService = nullptr;

  try {
    floraService = floraClient->getService(serviceUUID);
  }
  catch (...) {
    Serial.println("- Exception getting data service");
    syslog.logf(LOG_WARNING, "%s (C:%d - R:%d) -> Exception getting data service", addr, cnt, retry);
  }
  if (floraService == nullptr) {
    Serial.println("- No data service found");
    syslog.logf(LOG_WARNING, "%s (C:%d - R:%d) -> No data service found", addr, cnt, retry);
  }
  else {
    Serial.println("- Found data service");
    #ifdef DEBUG
    syslog.logf(LOG_DEBUG, "%s (C:%d - R:%d) -> Found data service", addr, cnt, retry);
    #endif
  }

  return floraService;
}

bool forceFloraServiceDataMode(BLERemoteService* floraService, char* addr, int cnt, int retry) {
  BLERemoteCharacteristic* floraCharacteristic;
  
  Serial.println("- Force device in data mode");
  #ifdef DEBUG
  syslog.logf(LOG_DEBUG, "%s (C:%d - R:%d) -> Force device in data mode", addr, cnt, retry);
  #endif
  floraCharacteristic = nullptr;
  try {
    floraCharacteristic = floraService->getCharacteristic(uuid_write_mode);
  }
  catch (...) {
    Serial.println("- Exception forcing device in data mode");
    syslog.logf(LOG_WARNING, "%s (C:%d - R:%d) -> Exception forcing device in data mode", addr, cnt, retry);
  }
  if (floraCharacteristic == nullptr) {
    Serial.println("- Failed to force the device in data mode");
    syslog.logf(LOG_WARNING, "%s (C:%d - R:%d) -> Failed to force the device in data mode", addr, cnt, retry);
    return false;
  }

  // write the magic data
  uint8_t buf[2] = {0xA0, 0x1F};
  floraCharacteristic->writeValue(buf, 2, true);

  delay(1000);
  return true;
}

bool readFloraDataCharacteristic(BLERemoteService* floraService, String baseTopic, char* addr, int cnt, int retry) {
  BLERemoteCharacteristic* floraCharacteristic = nullptr;

  // get the main device data characteristic
  Serial.println("- Access characteristic from device (sensor data)");
  #ifdef DEBUG
  syslog.logf(LOG_DEBUG, "%s (C:%d - R:%d) -> Access characteristic from device (sensor data)", addr, cnt, retry);
  #endif
  try {
    floraCharacteristic = floraService->getCharacteristic(uuid_sensor_data);
  }
  catch (...) {
    Serial.println("- Exception getting data characteristic (sensor data)");
    syslog.logf(LOG_WARNING, "%s (C:%d - R:%d) -> Exception getting data characteristic (sensor data)", addr, cnt, retry);
  }
  if (floraCharacteristic == nullptr) {
    Serial.println("-- Failed to get data characteristic (sensor data)");
    syslog.logf(LOG_WARNING, "%s (C:%d - R:%d) -> Failed to get data characteristic (sensor data)", addr, cnt, retry);
    return false;
  }

  // read characteristic value
  Serial.println("- Read value from characteristic (sensor data)");
  syslog.logf(LOG_DEBUG, "%s (C:%d - R:%d) -> Read value from characteristic (sensor data)", addr, cnt, retry);
  std::string value;
  try{
    value = floraCharacteristic->readValue();
  }
  catch (...) {
    // something went wrong
    Serial.println("-- Failed, skipping device");
    syslog.logf(LOG_WARNING, "%s (C:%d - R:%d) -> Exception reading value from characteristic (sensor data)", addr, cnt, retry);
    return false;
  }
  const char *val = value.c_str();

  Serial.print("Hex: ");
  for (int i = 0; i < 16; i++) {
    Serial.print((int)val[i], HEX);
    Serial.print(" ");
  }
  Serial.println(" ");

  int16_t* temp_raw = (int16_t*)val;
  float temperature = (*temp_raw) / ((float)10.0);
  int moisture = val[7];
  int light = val[3] + val[4] * 256;
  int conductivity = val[8] + val[9] * 256;

  Serial.print("Temperature ");
  Serial.print(temperature);
  Serial.print(" - Moisture ");
  Serial.print(moisture);
  Serial.print(" - Light ");
  Serial.print(light);
  Serial.print(" - Conductivity ");
  Serial.println(conductivity);
  syslog.logf(LOG_INFO, "%s (C:%d - R:%d) -> Temperature %.1f - Moisture %d - Light %d - Conductivity %d", addr, cnt, retry, temperature, moisture, light, conductivity);

  if (temperature > 80 || temperature < -20 || conductivity > 20000 ) {
    Serial.println("-- Unreasonable values fetched, skip publish");
    syslog.logf(LOG_WARNING, "%s (C:%d - R:%d) -> Unreasonable values fetched, skip publish", addr, cnt, retry);
    return false;
  }

  // Check MQTT connection
  connectMqtt();
  
  char buffer[64];
  snprintf(buffer, 64, "%.1f", temperature);
  client.publish((baseTopic + "temperature").c_str(), buffer, true); 
  snprintf(buffer, 64, "%d", moisture); 
  client.publish((baseTopic + "moisture").c_str(), buffer, true);
  snprintf(buffer, 64, "%d", light);
  client.publish((baseTopic + "light").c_str(), buffer, true);
  snprintf(buffer, 64, "%d", conductivity);
  client.publish((baseTopic + "conductivity").c_str(), buffer, true);

  return true;
}

bool readFloraBatteryCharacteristic(BLERemoteService* floraService, String baseTopic, char* addr, int cnt, int retry) {
  BLERemoteCharacteristic* floraCharacteristic = nullptr;

  // get the device battery characteristic
  Serial.println("- Access battery characteristic from device");
  #ifdef DEBUG
  syslog.logf(LOG_DEBUG, "%s (C:%d - R:%d) -> Access battery characteristic from device", addr, cnt, retry);
  #endif
  try {
    floraCharacteristic = floraService->getCharacteristic(uuid_version_battery);
  }
  catch (...) {
    Serial.println("- Exception getting battery characteristic from device");
    syslog.logf(LOG_WARNING, "%s (C:%d - R:%d) -> Exception getting battery characteristic from device", addr, cnt, retry);
  }
  if (floraCharacteristic == nullptr) {
    Serial.println("- Failed getting battery characteristic from device");
    syslog.logf(LOG_WARNING, "%s (C:%d - R:%d) -> Failed getting battery characteristic from device", addr, cnt, retry);
    return false;
  }

  // read characteristic value
  Serial.println("- Read value from battery characteristic");
  syslog.logf(LOG_DEBUG, "%s (C:%d - R:%d) -> Read value from battery characteristic", addr, cnt, retry);
  std::string value;
  try{
    value = floraCharacteristic->readValue();
  }
  catch (...) {
    // something went wrong
    Serial.println("-- Exception getting battery characteristic from device");
    syslog.logf(LOG_WARNING, "%s (C:%d - R:%d) -> Exception getting battery characteristic from device", addr, cnt, retry);
    return false;
  }
  const char *val2 = value.c_str();
  int battery = val2[0];

  if (battery == 0 || battery > 100) {
    Serial.println("-- Unreasonable battery level fetched, skip publish");
    syslog.logf(LOG_WARNING, "%s (C:%d - R:%d) -> Unreasonable battery level fetched, skip publish", addr, cnt, retry);
    return false;
  }

  // Check MQTT connection
  connectMqtt();

  char buffer[64];
  Serial.print("- Battery: ");
  Serial.println(battery);
  syslog.logf(LOG_INFO, "%s (C:%d - R:%d) -> Battery %d", addr, cnt, retry, battery);
  snprintf(buffer, 64, "%d", battery);
  client.publish((baseTopic + "battery").c_str(), buffer, true);

  return true;
}

bool processFloraService(BLERemoteService* floraService, char* deviceMacAddress, bool readBattery, char* addr, int cnt, int retry) {
  // set device in data mode
  if (!forceFloraServiceDataMode(floraService, addr, cnt, retry)) {
    return false;
  }

  String baseTopic = MQTT_BASE_TOPIC + "/" + deviceMacAddress + "/";
  bool dataSuccess = readFloraDataCharacteristic(floraService, baseTopic, addr, cnt, retry);

  bool batterySuccess = true;
  if (readBattery) {
    batterySuccess = readFloraBatteryCharacteristic(floraService, baseTopic, addr, cnt, retry);
  }

  return dataSuccess && batterySuccess;
}

bool processFloraDevice(BLEAddress floraAddress, char* deviceMacAddress, bool getBattery, int tryCount, int i) {
  Serial.print("Processing Flora device #");
  Serial.print(i);
  Serial.print(" at ");
  Serial.print(floraAddress.toString().c_str());
  Serial.print(" (try ");
  Serial.print(tryCount);
  Serial.println(")");

  // connect to flora ble server
  BLEClient* floraClient = getFloraClient(floraAddress, deviceMacAddress, i, tryCount);
  if (floraClient == nullptr) {
    return false;
  }

  // connect data service
  BLERemoteService* floraService = getFloraService(floraClient, deviceMacAddress, i, tryCount);
  if (floraService == nullptr) {
    floraClient->disconnect();
    return false;
  }

  // process devices data
  bool success = processFloraService(floraService, deviceMacAddress, getBattery, deviceMacAddress, i, tryCount);

  // disconnect from device
  floraClient->disconnect();

  return success;
}

void hibernate() {
  esp_sleep_enable_timer_wakeup(SLEEP_DURATION * 1000000ll);
  delay(100);
  esp_deep_sleep_start();
}

void delayedHibernate(void *parameter) {
  delay(EMERGENCY_HIBERNATE*1000); // delay for five minutes
  Serial.println("- Something got stuck, restarting the device...");
  syslog.logf(LOG_WARNING, "Something got stuck, restarting the device...");
  delay(5000);
  disconnectMqtt();
  disconnectWifi();
  ESP.restart();
}

void setup() {
  client.setServer(MQTT_HOST, MQTT_PORT);
  
  // all action is done when device is woken up
  Serial.begin(115200);
 
  delay(1000);

  if (!EEPROM.begin(EEPROM_SIZE)) {
    Serial.println("- Failed to initialise EEPROM");
  }

  deviceSeq = EEPROM.read(1);
  if (deviceSeq < 0 || deviceSeq > 48) {
    deviceSeq = 0;
    EEPROM.write(1, deviceSeq);
    EEPROM.commit();
  }

  Serial.print("Starting from device sequence #");
  Serial.println(deviceSeq);

  bootCount = EEPROM.read(0);
  if (bootCount < 1 || bootCount > 9) {
    bootCount = 1;
    EEPROM.write(0, bootCount);
    EEPROM.commit();
  }

  // create a hibernate task in case something gets stuck
  xTaskCreate(delayedHibernate, "hibernate", 4096, NULL, 1, &hibernateTaskHandle);

  Serial.println("Initialize BLE client...");

  BLEDevice::init("");
  BLEDevice::setPower(ESP_PWR_LVL_P7);

  // connecting wifi
  connectWifi();

  // delete emergency hibernate task
  vTaskDelete(hibernateTaskHandle);

  // check if battery status should be read - based on boot count
  bool readBattery = ((bootCount % BATTERY_INTERVAL) == 0);

  Serial.print("Boot loop #");
  Serial.println(bootCount);
  syslog.logf(LOG_DEBUG, "Welcome back, we are at boot loop #%d", bootCount);

  // process devices
  for (int i=deviceSeq; i<deviceCount; i++) {
    int tryCount = EEPROM.read(2);
    if (tryCount < 0 || tryCount >= RETRY) {
      tryCount = 0;
      EEPROM.write(2, tryCount);
      EEPROM.commit();
    }
    
    char* deviceMacAddress = FLORA_DEVICES[i];
    BLEAddress floraAddress(deviceMacAddress);

    while (tryCount < RETRY) {
      xTaskCreate(delayedHibernate, "hibernate", 4096, NULL, 1, &hibernateTaskHandle);
      
      tryCount++;
      EEPROM.write(2, tryCount);
      EEPROM.commit();

      // Check MQTT connection
      connectMqtt();

      if (processFloraDevice(floraAddress, deviceMacAddress, readBattery, tryCount, i)) {
          vTaskDelete(hibernateTaskHandle);
          break;
      } else {
          vTaskDelete(hibernateTaskHandle);
          if ((tryCount % 4) == 0) {
            Serial.println("- We have to flush the memory, restarting the device...");
            syslog.logf(LOG_DEBUG, "We have to flush the memory, restarting the device...");
            disconnectMqtt();
            delay(5000);
            disconnectWifi();
            ESP.restart();
          } else {
            delay(10000);
          }
      }
    }
    deviceSeq++;
    tryCount = 0;
    EEPROM.write(1, deviceSeq);
    EEPROM.write(2, tryCount);
    EEPROM.commit();
    delay(1500);

    if ((deviceSeq % 6) == 0) {
      Serial.println("- We have to flush the memory, restarting the device...");
      syslog.logf(LOG_DEBUG, "We have to flush the memory, restarting the device...");
      disconnectMqtt();
      delay(5000);
      disconnectWifi();
      ESP.restart();
    }
  }

  bootCount++;
  deviceSeq = 0;
  EEPROM.write(0, bootCount);
  EEPROM.write(1, deviceSeq);
  EEPROM.commit();

  Serial.println("Loop completed, going to sleep for ");
  Serial.print(SLEEP_DURATION);
  Serial.println(" seconds now.");
  syslog.logf(LOG_DEBUG, "Loop completed, going to sleep for %d seconds now.", SLEEP_DURATION);
  delay(5000);

  // disconnect wifi and mqtt
  disconnectMqtt();
  delay(5000);
  disconnectWifi();

  // go to sleep now
  hibernate();
}

void loop() {
  /// we're not doing anything in the loop, only on device wakeup
  delay(10000);
}
