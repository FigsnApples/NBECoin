#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void ble_init(void);

// Chronoamperometry sample (Option A)
// Sends: "CA,<index>,<current_uA>\n"
void ble_push_current(uint32_t index, float current_uA);

// Biomarker snapshot
// Sends: "BIO,<sys>,<dia>,<hr>,<spo2>,<tmpC>,<imuTempC>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>\n"
void ble_push_bio(float sys, float dia, float hr, float spo2,
                  float tmpC, float imuTempC,
                  float ax, float ay, float az,
                  float gx, float gy, float gz);

#ifdef __cplusplus
}
#endif