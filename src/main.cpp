// Configure variables
#define US_KEYBOARD 1
#define DEVICE_NAME "ESP32 Keyboard"
#define KBD_PIN 4
#define BATT_PIN 35
#define BATT_THOLD 0
#define WIFI_BTN 32
#define DP 18
#define DN 19

#include <Arduino.h>
#include <ESP32-USBSoftHost.hpp>

#include <BLEDevice.h>
#include <BLEHIDDevice.h>
#include <HIDTypes.h>
#include <HIDKeyboardTypes.h>

#include <WifiManager.h>
#include <ArduinoOTA.h>


bool isBleConnected = false;
bool wifiMode = false;
uint8_t percentage = 0;

usb_pins_config_t USB_Pins_Config = { DP, DN, -1, -1, -1, -1, -1, -1 };


// Message (report) received when an LED's state changed
struct OutputReport {
    uint8_t leds; // bitmask: num lock = 1, caps lock = 2, scroll lock = 4, compose = 8, kana = 16
};


// The report map describes the HID device (a keyboard in this case) and
// the messages (reports in HID terms) sent and received.
static const uint8_t REPORT_MAP[] = {
    USAGE_PAGE(1),      0x01,       // Generic Desktop Controls
    USAGE(1),           0x06,       // Keyboard
    COLLECTION(1),      0x01,       // Application
    REPORT_ID(1),       0x01,       //   Report ID (1)
    USAGE_PAGE(1),      0x07,       //   Keyboard/Keypad
    USAGE_MINIMUM(1),   0xE0,       //   Keyboard Left Control
    USAGE_MAXIMUM(1),   0xE7,       //   Keyboard Right Control
    LOGICAL_MINIMUM(1), 0x00,       //   Each bit is either 0 or 1
    LOGICAL_MAXIMUM(1), 0x01,
    REPORT_COUNT(1),    0x08,       //   8 bits for the modifier keys
    REPORT_SIZE(1),     0x01,       
    HIDINPUT(1),        0x02,       //   Data, Var, Abs
    REPORT_COUNT(1),    0x01,       //   1 byte (unused)
    REPORT_SIZE(1),     0x08,
    HIDINPUT(1),        0x01,       //   Const, Array, Abs
    REPORT_COUNT(1),    0x06,       //   6 bytes (for up to 6 concurrently pressed keys)
    REPORT_SIZE(1),     0x08,
    LOGICAL_MINIMUM(1), 0x00,
    LOGICAL_MAXIMUM(1), 0x65,       //   101 keys
    USAGE_MINIMUM(1),   0x00,
    USAGE_MAXIMUM(1),   0x65,
    HIDINPUT(1),        0x00,       //   Data, Array, Abs
    REPORT_COUNT(1),    0x05,       //   5 bits (Num lock, Caps lock, Scroll lock, Compose, Kana)
    REPORT_SIZE(1),     0x01,
    USAGE_PAGE(1),      0x08,       //   LEDs
    USAGE_MINIMUM(1),   0x01,       //   Num Lock
    USAGE_MAXIMUM(1),   0x05,       //   Kana
    LOGICAL_MINIMUM(1), 0x00,
    LOGICAL_MAXIMUM(1), 0x01,
    HIDOUTPUT(1),       0x02,       //   Data, Var, Abs
    REPORT_COUNT(1),    0x01,       //   3 bits (Padding)
    REPORT_SIZE(1),     0x03,
    HIDOUTPUT(1),       0x01,       //   Const, Array, Abs
    END_COLLECTION(0)               // End application collection
};


BLEHIDDevice* hid;
BLECharacteristic* input;
BLECharacteristic* output;


static void sendKeypress(uint8_t usbNum, uint8_t byte_depth, uint8_t* data, uint8_t data_len) {
  if (isBleConnected) {
    input->setValue(data, data_len);
    input->notify();
    delay(1);
  }
  printf("in: ");
  for(int k = 0; k < data_len; k++) {
    printf("0x%02x ", data[k] );
  }
  printf("\n");
}


static void my_USB_DetectCB( uint8_t usbNum, void * dev ) {
  sDevDesc *device = (sDevDesc*)dev;
  printf("New device detected on USB#%d\n", usbNum);
  printf("desc.bcdUSB             = 0x%04x\n", device->bcdUSB);
  printf("desc.bDeviceClass       = 0x%02x\n", device->bDeviceClass);
  printf("desc.bDeviceSubClass    = 0x%02x\n", device->bDeviceSubClass);
  printf("desc.bDeviceProtocol    = 0x%02x\n", device->bDeviceProtocol);
  printf("desc.bMaxPacketSize0    = 0x%02x\n", device->bMaxPacketSize0);
  printf("desc.idVendor           = 0x%04x\n", device->idVendor);
  printf("desc.idProduct          = 0x%04x\n", device->idProduct);
  printf("desc.bcdDevice          = 0x%04x\n", device->bcdDevice);
  printf("desc.iManufacturer      = 0x%02x\n", device->iManufacturer);
  printf("desc.iProduct           = 0x%02x\n", device->iProduct);
  printf("desc.iSerialNumber      = 0x%02x\n", device->iSerialNumber);
  printf("desc.bNumConfigurations = 0x%02x\n", device->bNumConfigurations);
}


/*
 * Callbacks related to BLE connection
 */
class BleKeyboardCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* server) {
        // Allow notifications for characteristics
        BLE2902* cccDesc = (BLE2902*)input->getDescriptorByUUID(BLEUUID((uint16_t)0x2902));
        cccDesc->setNotifications(true);

        Serial.println("Client has connected");
        isBleConnected = true;
    }

    void onDisconnect(BLEServer* server) {
        isBleConnected = false;
        USH.TimerPause();

        // Disallow notifications for characteristics
        BLE2902* cccDesc = (BLE2902*)input->getDescriptorByUUID(BLEUUID((uint16_t)0x2902));
        cccDesc->setNotifications(false);

        Serial.println("Client has disconnected");
    }
};


/*
 * Called when the client (computer, smart phone) wants to turn on or off
 * the LEDs in the keyboard.
 * 
 * bit 0 - NUM LOCK
 * bit 1 - CAPS LOCK
 * bit 2 - SCROLL LOCK
 */
class OutputCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* characteristic) {
    OutputReport* report = (OutputReport*) characteristic->getData();
    Serial.print("LED state: ");
    Serial.print((int) report->leds);
    Serial.println();
    USH.TimerResume();
    digitalWrite(KBD_PIN, HIGH);
  }
};


void readBattery() {
  float voltage;
  voltage = 100 * (analogRead(BATT_PIN) / 4096.0) * 6.81;
  percentage = constrain(map(voltage, 350, 420, 0, 100), 0, 100);
  printf("Percentage: %d\n", percentage);
}


void batteryUpdateTask(void*){
  const TickType_t xDelay = 60000 / portTICK_PERIOD_MS;
  while (1) {
    readBattery();
    if (isBleConnected && percentage > BATT_THOLD) {
      hid->setBatteryLevel(percentage);
    }
    else if (percentage <= BATT_THOLD) {
      esp_deep_sleep_start();
    }
    vTaskDelay(xDelay);
  }
}


void bluetoothTask(void*) {
  // initialize the device
  BLEDevice::init(DEVICE_NAME);
  BLEServer* server = BLEDevice::createServer();
  server->setCallbacks(new BleKeyboardCallbacks());

  // create an HID device
  hid = new BLEHIDDevice(server);
  input = hid->inputReport(1); // report ID
  output = hid->outputReport(1); // report ID
  output->setCallbacks(new OutputCallbacks());

  // set manufacturer name
  hid->manufacturer()->setValue("Maker Community");
  // set USB vendor and product ID
  hid->pnp(0x02, 0xe502, 0xa111, 0x0210);
  // information about HID device: device is not localized, device can be connected
  hid->hidInfo(0x00, 0x02);

  // Security: device requires bonding
  BLESecurity* security = new BLESecurity();
  security->setAuthenticationMode(ESP_LE_AUTH_BOND);

  // set report map
  hid->reportMap((uint8_t*)REPORT_MAP, sizeof(REPORT_MAP));
  hid->startServices();

  // set battery level to 100%
  hid->setBatteryLevel(percentage);

  // advertise the services
  BLEAdvertising* advertising = server->getAdvertising();
  advertising->setAppearance(HID_KEYBOARD);
  advertising->addServiceUUID(hid->hidService()->getUUID());
  advertising->addServiceUUID(hid->deviceInfo()->getUUID());
  advertising->addServiceUUID(hid->batteryService()->getUUID());
  advertising->start();

  printf("BLE ready\n");
  delay(portMAX_DELAY);
}


void setupWifi() {
  pinMode(2, OUTPUT);
  printf("Wifi mode enabled\n");
  WiFiManager wifiManager;
  wifiManager.autoConnect(DEVICE_NAME);
  printf("Wifi Connected!\n");
  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
          type = "sketch";
      else
          type = "filesystem";
      Serial.println("Start updating " + type);
    })
    .onEnd([]() { Serial.println("\nEnd"); })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR)
        Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR)
        Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR)
        Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR)
        Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR)
        Serial.println("End Failed");
    });
  ArduinoOTA.begin();
}


void handleWifi() {
  ArduinoOTA.handle();
  digitalWrite(2, !digitalRead(2));
  delay(250);
}


void setup() {
  Serial.begin(115200);
  pinMode(KBD_PIN, OUTPUT);
  pinMode(BATT_PIN, INPUT);
  digitalWrite(KBD_PIN, LOW);
  pinMode(WIFI_BTN, INPUT_PULLUP);
  wifiMode = !digitalRead(WIFI_BTN);
  delay(100);
  if (wifiMode) {
    setupWifi();
  }
  else {
    readBattery();
    delay(100);
    USH.init(USB_Pins_Config, my_USB_DetectCB, sendKeypress);
    USH.TimerPause();
    xTaskCreate(bluetoothTask, "bluetooth", 20000, NULL, 5, NULL);
    xTaskCreate(batteryUpdateTask, "battery", 4096, NULL, 5, NULL);
  }
}


void loop() {
  if (wifiMode) {
    handleWifi();
  }
  else {
    vTaskDelete(NULL);
  }
}
