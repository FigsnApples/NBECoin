#pragma once
// BleComm.h - Minimal BLE notifier interface for amperometric / SWV streaming

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Initialize BLE (advertises a UART-like service that notifies text samples)
void ble_init(void);

// Backward-compatible: send one current sample only.
// Format: "index:<idx>, <current_uA>\n"
void ble_push_current(uint32_t index, float current_uA);

// SWV-aware: send current with step + phase labels.
// phase should be 'F' (forward/high) or 'R' (reverse/low)
// Format: "idx:<idx>, step:<step>, phase:<F/R>, I_uA:<current>\n"
void ble_push_swv(uint32_t index, uint32_t step, char phase, float current_uA);

#ifdef __cplusplus
}
#endif
