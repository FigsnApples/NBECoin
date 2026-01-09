#pragma once
// BleComm.h - Minimal Bluedroid BLE interface for amperometric streaming

#ifdef __cplusplus
extern "C" {
#endif

// Initialize BLE (advertises a UART-like service that notifies text samples)
void ble_init(void);

// Push one current value (in microamps) as text notification: "current=<value>"
void ble_push_current(float amps_uA);

// Push an array of current values (sends multiple notifications, simple loop)
void ble_push_buffer(const float* vals, unsigned int count);

#ifdef __cplusplus
}
#endif

