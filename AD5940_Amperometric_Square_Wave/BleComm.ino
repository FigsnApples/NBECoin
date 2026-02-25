// BleComm.ino — implementation for the header you shared
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include "BleComm.h"  // declares ble_init / ble_push_current / ble_push_swv

// Use your own UUIDs if you already have them
static const char* SERVICE_UUID        = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"; // UART-like service
static const char* CHAR_NOTIFY_UUID    = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"; // notify characteristic

static BLEServer*        g_server        = nullptr;
static BLEService*       g_service       = nullptr;
static BLECharacteristic* g_notifyChar   = nullptr;

void ble_init(void) {
  // Init BLE and set a friendly name
  BLEDevice::init("ESP32_AMPEROMETRIC");
  // (Optional) request a larger MTU for longer lines; 185 is typical on iOS/Android
  BLEDevice::setMTU(185);

  g_server  = BLEDevice::createServer();
  g_service = g_server->createService(SERVICE_UUID);

  // Create a characteristic that supports NOTIFY (and READ so nRF Connect can inspect it)
  g_notifyChar = g_service->createCharacteristic(
      CHAR_NOTIFY_UUID,
      BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ
  );

  // *** CRITICAL: add CCCD so clients can subscribe ***
  g_notifyChar->addDescriptor(new BLE2902()); // Without this, phones can’t “Subscribe”

  // Some apps like seeing an initial value when you tap the characteristic
  g_notifyChar->setValue("current=ready\n");

  // Start service and advertise it
  g_service->start();

  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setScanResponse(true);
  adv->setMinPreferred(0x06); // helps with iOS
  adv->setMinPreferred(0x12); // helps with iOS
  BLEDevice::startAdvertising();
}

// Send one value as a text line: "index:<idx>, <current_uA>\n"
void ble_push_current(uint32_t index, float current_uA) {
  if (!g_notifyChar) return;
  char line[48];
  // Keep it short; \n makes logs line-friendly
  snprintf(line, sizeof(line), "index:%lu, %.9f\n", (unsigned long)index, (double)current_uA);
  g_notifyChar->setValue((uint8_t*)line, strlen(line));
  g_notifyChar->notify(); // triggers to all subscribed centrals
}

// SWV-aware line: "idx:<idx>, step:<step>, phase:<F/R>, I_uA:<current>\n"
void ble_push_swv(uint32_t index, uint32_t step, char phase, float current_uA) {
  if (!g_notifyChar) return;
  char line[96];
  snprintf(line, sizeof(line),
           "idx:%lu, step:%lu, phase:%c, I_uA:%.9f\n",
           (unsigned long)index,
           (unsigned long)step,
           phase,
           (double)current_uA);
  g_notifyChar->setValue((uint8_t*)line, strlen(line));
  g_notifyChar->notify();
}

