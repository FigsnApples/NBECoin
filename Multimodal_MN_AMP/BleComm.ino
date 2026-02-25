#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "BleComm.h"

// Nordic UART style UUIDs
static const char* SERVICE_UUID     = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
static const char* CHAR_NOTIFY_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";

static BLEServer*         g_server     = nullptr;
static BLEService*        g_service    = nullptr;
static BLECharacteristic* g_notifyChar = nullptr;

static inline void ble_notify_line(const char* line) {
  if (!g_notifyChar || !line) return;
  g_notifyChar->setValue((uint8_t*)line, strlen(line));
  g_notifyChar->notify();
}

extern "C" {

void ble_init(void) {
  BLEDevice::init("ESP32_AMPEROMETRIC");
  BLEDevice::setMTU(185);

  g_server  = BLEDevice::createServer();
  g_service = g_server->createService(SERVICE_UUID);

  g_notifyChar = g_service->createCharacteristic(
      CHAR_NOTIFY_UUID,
      BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ
  );
  g_notifyChar->addDescriptor(new BLE2902());
  g_notifyChar->setValue("READY\n");

  g_service->start();

  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setScanResponse(true);
  adv->setMinPreferred(0x06);
  adv->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
}

// CA sample (Option A) -> "CA,index,current_uA"
void ble_push_current(uint32_t index, float current_uA) {
  char line[64];
  snprintf(line, sizeof(line), "CA,%lu,%.9f\n",
           (unsigned long)index, (double)current_uA);
  ble_notify_line(line);
}

// Biomarkers -> "BIO,sys,dia,hr,spo2,tmpC,imuTempC,ax,ay,az,gx,gy,gz"
void ble_push_bio(float sys, float dia, float hr, float spo2,
                  float tmpC, float imuTempC,
                  float ax, float ay, float az,
                  float gx, float gy, float gz) {
  char line[200];
  snprintf(line, sizeof(line),
           "BIO,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
           (double)sys, (double)dia, (double)hr, (double)spo2,
           (double)tmpC, (double)imuTempC,
           (double)ax, (double)ay, (double)az,
           (double)gx, (double)gy, (double)gz);
  ble_notify_line(line);
}

} // extern "C"