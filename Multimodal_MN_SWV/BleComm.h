#pragma once
// BleComm.h - Minimal BLE notifier interface for amperometric / SWV streaming

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Initialize BLE (advertises a UART-like service with TWO notify characteristics)
void ble_init(void);

// Backward-compatible: send one current sample only.
// Format: "index:<idx>, <I_uA>\n"
void ble_push_current(uint32_t index, float current_uA);

// SWV-aware: send current with step + phase labels.
// phase should be 'F' (forward/high) or 'R' (reverse/low)
// Format: "idx:<idx>, step:<step>, phase:<phase>, I_uA:<val>\n"
void ble_push_swv(uint32_t index, uint32_t step, char phase, float current_uA);

// NEW: Biomarker CSV notify (separate characteristic)
// Format: "sys,dia,hr,spo2,tmp117_C,icmTemp_C,ax,ay,az,gx,gy,gz\n"
void ble_push_bio(float sys, float dia, float hr, float spo2,
                  float tmp117_C, float icmTemp_C,
                  float ax, float ay, float az,
                  float gx, float gy, float gz);

// NEW: optional helper if you want to send preformatted lines
void ble_push_bio_line(const char* line);

#ifdef __cplusplus
}
#endif
