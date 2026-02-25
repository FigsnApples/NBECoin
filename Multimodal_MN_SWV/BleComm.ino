// BleComm.ino — BLE notifier with 2 notify characteristics (SWV + BIO)

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include "BleComm.h"

// Nordic UART-like service (your existing one)
static const char* SERVICE_UUID      = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";

// Existing SWV notify characteristic (UNCHANGED)
static const char* CHAR_SWV_UUID     = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";

// NEW biomarker notify characteristic (same base UUID, different short ID)
static const char* CHAR_BIO_UUID     = "6E400004-B5A3-F393-E0A9-E50E24DCCA9E";

static BLEServer*         g_server   = nullptr;
static BLEService*        g_service  = nullptr;
static BLECharacteristic* g_swvChar  = nullptr;
static BLECharacteristic* g_bioChar  = nullptr;

void ble_init(void) {
  BLEDevice::init("ESP32_AMPEROMETRIC");

  // Request a larger MTU so our CSV lines fit comfortably
  BLEDevice::setMTU(185);

  g_server  = BLEDevice::createServer();
  g_service = g_server->createService(SERVICE_UUID);

  // --- SWV characteristic (existing) ---
  g_swvChar = g_service->createCharacteristic(
    CHAR_SWV_UUID,
    BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ
  );
  g_swvChar->addDescriptor(new BLE2902());
  g_swvChar->setValue("swv=ready\n");

  // --- BIO characteristic (new) ---
  g_bioChar = g_service->createCharacteristic(
    CHAR_BIO_UUID,
    BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ
  );
  g_bioChar->addDescriptor(new BLE2902());
  g_bioChar->setValue("bio=ready\n");

  g_service->start();

  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setScanResponse(true);
  adv->setMinPreferred(0x06);  // helps iOS
  adv->setMinPreferred(0x12);  // helps iOS
  BLEDevice::startAdvertising();
}

// Backward-compatible single-current line
void ble_push_current(uint32_t index, float current_uA) {
  if (!g_swvChar) return;
  char line[48];
  snprintf(line, sizeof(line), "index:%lu, %.9f\n",
           (unsigned long)index, (double)current_uA);
  g_swvChar->setValue((uint8_t*)line, strlen(line));
  g_swvChar->notify();
}

// SWV line (unchanged)
void ble_push_swv(uint32_t index, uint32_t step, char phase, float current_uA) {
  if (!g_swvChar) return;
  char line[96];
  snprintf(line, sizeof(line),
           "idx:%lu, step:%lu, phase:%c, I_uA:%.9f\n",
           (unsigned long)index, (unsigned long)step, phase, (double)current_uA);
  g_swvChar->setValue((uint8_t*)line, strlen(line));
  g_swvChar->notify();
}

// NEW: biomarker CSV
void ble_push_bio(float sys, float dia, float hr, float spo2,
                  float tmp117_C, float icmTemp_C,
                  float ax, float ay, float az,
                  float gx, float gy, float gz) {
  if (!g_bioChar) return;

  // Keep this <= ~180 bytes to fit MTU=185 comfortably.
  char line[180];
  snprintf(line, sizeof(line),
    "%.0f,%.0f,%.2f,%.2f,%.3f,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f\n",
    (double)sys, (double)dia, (double)hr, (double)spo2,
    (double)tmp117_C, (double)icmTemp_C,
    (double)ax, (double)ay, (double)az,
    (double)gx, (double)gy, (double)gz
  );

  g_bioChar->setValue((uint8_t*)line, strlen(line));
  g_bioChar->notify();
}

// NEW: send already formatted biomarker line
void ble_push_bio_line(const char* line) {
  if (!g_bioChar || !line) return;
  g_bioChar->setValue((uint8_t*)line, strlen(line));
  g_bioChar->notify();
}
