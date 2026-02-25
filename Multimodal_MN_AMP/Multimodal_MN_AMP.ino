#include <Arduino.h>
#include <Wire.h>

// ===== MAX32664 + TMP117 + ICM20948 =====
#include "max32664.h"
#include <Adafruit_TMP117.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>

// ===== AD5940 AMP + BLE =====
extern "C" {
  #include "ad5940.h"
  #include "Amperometric.h"
  #include "BleComm.h"
}

// ----------------- MAX32664 pins (CHANGE to your actual free GPIOs) -----------------
#define MAX_RESET_PIN   2
#define MAX_MFIO_PIN    3
#define RAWDATA_BUFFLEN 250

max32664 MAX32664(MAX_RESET_PIN, MAX_MFIO_PIN, RAWDATA_BUFFLEN);

// TMP117 / ICM20948 objects
Adafruit_TMP117   tmp117;
Adafruit_ICM20948 icm;

// ----------------- Shared APP buffer for AD5940 -----------------
#define APPBUFF_SIZE 1024
uint32_t AppBuff[APPBUFF_SIZE];
float LFOSCFreq = 0.0f;

// ----------------- Biomarker snapshot -----------------
typedef struct {
  volatile bool  estimation_ready;

  // MAX32664
  volatile float sys;
  volatile float dia;
  volatile float hr;
  volatile float spo2;

  // TMP117
  volatile float tmpC;

  // ICM20948
  volatile float imuTempC;
  volatile float ax, ay, az;
  volatile float gx, gy, gz;
} BiomarkerSnapshot;

static BiomarkerSnapshot gBio = {
  .estimation_ready = false,
  .sys = NAN, .dia = NAN, .hr = NAN, .spo2 = NAN,
  .tmpC = NAN,
  .imuTempC = NAN,
  .ax = NAN, .ay = NAN, .az = NAN,
  .gx = NAN, .gy = NAN, .gz = NAN
};

// ----------------- Optional MFIO interrupt handler -----------------
void mfioInterruptHndlr() {
  // optional (not required for polling mode)
}

static void enableInterruptPin() {
  attachInterrupt(digitalPinToInterrupt(MAX32664.mfioPin), mfioInterruptHndlr, FALLING);
}

// ----------------- Your calibration loader (EDIT values to your calibration) -----------------
static void loadAlgomodeParameters() {
  algomodeInitialiser algoParameters;

  // Replace with your calibration values from reference device
  algoParameters.calibValSys[0] = 110;
  algoParameters.calibValSys[1] = 116;
  algoParameters.calibValSys[2] = 111;

  algoParameters.calibValDia[0] = 70;
  algoParameters.calibValDia[1] = 76;
  algoParameters.calibValDia[2] = 71;

  algoParameters.spo2CalibCoefA = 1.5958422;
  algoParameters.spo2CalibCoefB = -34.659664;
  algoParameters.spo2CalibCoefC = 112.68987;

  MAX32664.loadAlgorithmParameters(&algoParameters);
}

// ----------------- Biomarker Task (init + calibration + continuous updates) -----------------
static void Biomarker_Task(void *pv) {
  (void)pv;

  // Init MAX32664
  loadAlgomodeParameters();
  Serial.println("[BIO] Initializing MAX32664 sensor hub...");
  int result = MAX32664.hubBegin();
  if (result == CMD_SUCCESS) {
    Serial.println("[BIO] MAX32664 hub begin OK!");
  } else {
    Serial.println("[BIO] ERROR: Could not communicate with MAX32664. Check connections.");
    while (1) vTaskDelay(pdMS_TO_TICKS(5000));
  }

  // Init TMP117 once
  Serial.println("[BIO] Initializing TMP117...");
  bool tmp_ok = tmp117.begin();
  if (!tmp_ok) Serial.println("[BIO] TMP117 not found -> temp will be N/A");
  else         Serial.println("[BIO] TMP117 found!");

  // Init ICM20948 once
  Serial.println("[BIO] Initializing ICM20948...");
  bool icm_ok = icm.begin_I2C();
  if (!icm_ok) Serial.println("[BIO] ICM20948 not found -> IMU will be N/A");
  else         Serial.println("[BIO] ICM20948 found!");

  // BP calibration
  Serial.println("[BIO] Starting BP calibration...");
  bool ret = MAX32664.startBPTcalibration();
  while (!ret) {
    Serial.println("[BIO] BP calibration failed - retrying...");
    vTaskDelay(pdMS_TO_TICKS(10000));
    ret = MAX32664.startBPTcalibration();
  }
  Serial.println("[BIO] BP calibration succeeded!");
  vTaskDelay(pdMS_TO_TICKS(1000));

  // Estimation mode
  Serial.println("[BIO] Entering estimation mode...");
  ret = MAX32664.configAlgoInEstimationMode();
  while (!ret) {
    Serial.println("[BIO] Failed to enter estimation mode - retrying...");
    vTaskDelay(pdMS_TO_TICKS(10000));
    ret = MAX32664.configAlgoInEstimationMode();
  }
  Serial.println("[BIO] Estimation mode active!");

  // Optional: enable MFIO interrupt (polling is fine, leave off if unstable)
  // enableInterruptPin();

  gBio.estimation_ready = true;

  // Push BIO over BLE at a slower rate than sensor polling
  uint32_t lastBioPushMs = 0;

  // Continuous updates
  while (1) {
    // MAX32664 samples (updates when available)
    uint8_t num_samples = MAX32664.readSamples();
    if (num_samples) {
      gBio.sys  = MAX32664.max32664Output.sys;
      gBio.dia  = MAX32664.max32664Output.dia;
      gBio.hr   = MAX32664.max32664Output.hr;
      gBio.spo2 = MAX32664.max32664Output.spo2;
    }

    // TMP117
    if (tmp_ok) {
      sensors_event_t tmp_event;
      tmp117.getEvent(&tmp_event);
      gBio.tmpC = tmp_event.temperature;
    } else {
      gBio.tmpC = NAN;
    }

    // ICM20948
    if (icm_ok) {
      sensors_event_t accel, gyro, icm_temp, mag;
      icm.getEvent(&accel, &gyro, &icm_temp, &mag);

      gBio.imuTempC = icm_temp.temperature;

      gBio.ax = accel.acceleration.x;
      gBio.ay = accel.acceleration.y;
      gBio.az = accel.acceleration.z;

      gBio.gx = gyro.gyro.x;
      gBio.gy = gyro.gyro.y;
      gBio.gz = gyro.gyro.z;
    } else {
      gBio.imuTempC = NAN;
      gBio.ax = gBio.ay = gBio.az = NAN;
      gBio.gx = gBio.gy = gBio.gz = NAN;
    }

    // BLE push BIO @ 2 Hz (adjust as needed)
    uint32_t now = millis();
    if (now - lastBioPushMs >= 500) {
      lastBioPushMs = now;
      ble_push_bio(
        gBio.sys, gBio.dia, gBio.hr, gBio.spo2,
        gBio.tmpC, gBio.imuTempC,
        gBio.ax, gBio.ay, gBio.az,
        gBio.gx, gBio.gy, gBio.gz
      );
    }

    vTaskDelay(pdMS_TO_TICKS(100)); // ~10 Hz update loop
  }
}

// ----------------- Serial formatting helpers -----------------
static void printFloatOrNA(float v, int decimals) {
  if (isnan(v)) Serial.print("N/A");
  else Serial.print(v, decimals);
}

static void printVec3OrNA(float x, float y, float z, int decimals,
                          const char* prefix, const char* suffix) {
  if (isnan(x) || isnan(y) || isnan(z)) {
    Serial.print("N/A");
    return;
  }
  Serial.print(prefix);
  Serial.print(x, decimals); Serial.print(",");
  Serial.print(y, decimals); Serial.print(",");
  Serial.print(z, decimals);
  Serial.print(suffix);
}

// ----------------- Per-AMP-sample: BLE + clean labeled Serial -----------------
static int32_t AMPShowResult(float *pData, uint32_t DataCount) {
  static uint32_t index = 0;

  for (uint32_t i = 0; i < DataCount; i++) {
    float current_uA = pData[i];

    // BLE (Option A with CA prefix inside ble_push_current implementation)
    ble_push_current(index, current_uA);

    // ---- Clean labeled Serial line ----
    Serial.print("[CA] idx=");
    Serial.print(index);

    Serial.print(" | I=");
    Serial.print(current_uA, 9);
    Serial.print(" uA");

    // BP combined + SYS + DIA
    Serial.print(" | BP=");
    if (isnan((float)gBio.sys) || isnan((float)gBio.dia)) {
      Serial.print("N/A");
    } else {
      Serial.print((float)gBio.sys, 0);
      Serial.print("/");
      Serial.print((float)gBio.dia, 0);
      Serial.print(" mmHg");
    }

    Serial.print(" | SYS=");
    printFloatOrNA((float)gBio.sys, 0);
    if (!isnan((float)gBio.sys)) Serial.print(" mmHg");

    Serial.print(" | DIA=");
    printFloatOrNA((float)gBio.dia, 0);
    if (!isnan((float)gBio.dia)) Serial.print(" mmHg");

    // HR
    Serial.print(" | HR=");
    printFloatOrNA((float)gBio.hr, 2);
    if (!isnan((float)gBio.hr)) Serial.print(" bpm");

    // SpO2
    Serial.print(" | SpO2=");
    printFloatOrNA((float)gBio.spo2, 2);
    if (!isnan((float)gBio.spo2)) Serial.print(" %");

    // TMP117
    Serial.print(" | TMP=");
    printFloatOrNA((float)gBio.tmpC, 3);
    if (!isnan((float)gBio.tmpC)) Serial.print(" C");

    // IMU temp
    Serial.print(" | IMU_T=");
    printFloatOrNA((float)gBio.imuTempC, 2);
    if (!isnan((float)gBio.imuTempC)) Serial.print(" C");

    // Accel
    Serial.print(" | Accel=");
    printVec3OrNA((float)gBio.ax, (float)gBio.ay, (float)gBio.az, 2, "(", ") m/s^2");

    // Gyro
    Serial.print(" | Gyro=");
    printVec3OrNA((float)gBio.gx, (float)gBio.gy, (float)gBio.gz, 3, "(", ") rad/s");

    Serial.println();
    index++;
  }
  return 0;
}

// ----------------- AD5940 platform config -----------------
static int32_t AD5940PlatformCfg(void) {
  CLKCfg_Type         clk_cfg;
  FIFOCfg_Type        fifo_cfg;
  SEQCfg_Type         seq_cfg;
  AGPIOCfg_Type       gpio_cfg;
  LFOSCMeasure_Type   LfoscMeasure;

  AD5940_HWReset();
  AD5940_Initialize();

  // Clock
  clk_cfg.HFOSCEn         = bTRUE;
  clk_cfg.HFXTALEn        = bFALSE;
  clk_cfg.LFOSCEn         = bTRUE;
  clk_cfg.HfOSC32MHzMode  = bFALSE;
  clk_cfg.SysClkSrc       = SYSCLKSRC_HFOSC;
  clk_cfg.SysClkDiv       = SYSCLKDIV_1;
  clk_cfg.ADCCLkSrc       = ADCCLKSRC_HFOSC;
  clk_cfg.ADCClkDiv       = ADCCLKDIV_1;
  AD5940_CLKCfg(&clk_cfg);

  // FIFO + Sequencer
  fifo_cfg.FIFOEn      = bFALSE;
  fifo_cfg.FIFOMode    = FIFOMODE_FIFO;
  fifo_cfg.FIFOSize    = FIFOSIZE_4KB;
  fifo_cfg.FIFOSrc     = FIFOSRC_DFT;     // keep as in your existing repo (don’t change unless you know)
  fifo_cfg.FIFOThresh  = 1;
  AD5940_FIFOCfg(&fifo_cfg);

  fifo_cfg.FIFOEn = bTRUE;
  AD5940_FIFOCfg(&fifo_cfg);

  seq_cfg.SeqMemSize   = SEQMEMSIZE_2KB;
  seq_cfg.SeqBreakEn   = bFALSE;
  seq_cfg.SeqIgnoreEn  = bFALSE;
  seq_cfg.SeqCntCRCClr = bTRUE;
  seq_cfg.SeqEnable    = bFALSE;
  seq_cfg.SeqWrTimer   = 0;
  AD5940_SEQCfg(&seq_cfg);

  // Interrupts
  AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ALLINT, bTRUE);
  AD5940_INTCCfg(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH, bTRUE);
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);

  // GPIO
  gpio_cfg.FuncSet      = GP6_SYNC | GP5_SYNC | GP4_SYNC | GP2_SYNC | GP1_SLEEP | GP0_INT;
  gpio_cfg.InputEnSet   = AGPIO_Pin2;
  gpio_cfg.OutputEnSet  = AGPIO_Pin0 | AGPIO_Pin1 | AGPIO_Pin4 | AGPIO_Pin5 | AGPIO_Pin6;
  gpio_cfg.OutVal       = AGPIO_Pin1; // set high to turn off LED (per your note)
  gpio_cfg.PullEnSet    = 0;
  AD5940_AGPIOCfg(&gpio_cfg);

  AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);

  // LFOSC measure
  LfoscMeasure.CalDuration   = 1000.0f;
  LfoscMeasure.CalSeqAddr    = 0;
  LfoscMeasure.SystemClkFreq = 16000000.0f; // 16 MHz
  AD5940_LFOSCMeasure(&LfoscMeasure, &LFOSCFreq);

  return 0;
}

// ----------------- AMP parameter init (chronoamperometry / amperometry) -----------------
static void AD5940AMPStructInit(void) {
  AppAMPCfg_Type *pAMPCfg;
  AppAMPGetCfg(&pAMPCfg);

  pAMPCfg->WuptClkFreq = LFOSCFreq;

  // General
  pAMPCfg->SeqStartAddr = 0;
  pAMPCfg->MaxSeqLen    = 512;
  pAMPCfg->RcalVal      = 10000.0f;
  pAMPCfg->NumOfData    = -1;     // run continuously

  // Sampling
  pAMPCfg->AmpODR      = 2.0f;    // seconds between samples (adjust!)
  pAMPCfg->FifoThresh  = 1;

  // Electrochem settings (keep your known-good values)
  pAMPCfg->SensorBias   = -350;          // mV
  pAMPCfg->LptiaRtiaSel = LPTIARTIA_256K;
  pAMPCfg->LpTiaRl      = LPTIARLOAD_3K1;
  pAMPCfg->Vzero        = 1100;          // mV
  pAMPCfg->ADCRefVolt   = 1.82f;
}

// ----------------- AD5940 main loop -----------------
static void AD5940_Main(void) {
  uint32_t temp;

  AD5940PlatformCfg();
  AD5940AMPStructInit();

  AppAMPInit(AppBuff, APPBUFF_SIZE);
  AppAMPCtrl(AMPCTRL_START, 0);

  while (1) {
    if (AD5940_GetMCUIntFlag()) {
      AD5940_ClrMCUIntFlag();
      temp = APPBUFF_SIZE;

      AppAMPISR(AppBuff, &temp);

      // NOTE: This assumes AppBuff contains float current values (uA).
      // If your AppAMPISR returns raw codes, we’ll need to convert before printing.
      AMPShowResult((float*)AppBuff, temp);
    }
  }
}

// ----------------- FreeRTOS wrapper -----------------
static void AD5940_Task(void *pv) {
  (void)pv;
  AD5940_MCUResourceInit(NULL);
  AD5940_Main();
  vTaskDelete(NULL);
}

// ----------------- Arduino entry points -----------------
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("start");

  // Start I2C (your pins)
  Wire.begin();
  Wire.setClock(400000);
  Wire.setTimeOut(50);


  // Start BLE
  ble_init();
  delay(200);

  // Start biomarker init/calibration task
  Serial.println("[BIO] Initializing MAX32664 sensor hub...");
  int result = MAX32664.hubBegin();

  Serial.printf("[BIO] hubBegin result=%d\n", result);

  if (result != CMD_SUCCESS) {
    Serial.println("[BIO] ERROR: Could not communicate with MAX32664. Check wiring/pins.");
    vTaskDelete(NULL);   // don’t hard-lock the whole program
  }

  loadAlgomodeParameters();

  // Gate AD5940 start until estimation ready
  Serial.println("[SYSTEM] Waiting for BP calibration + estimation mode before starting AMP...");
  uint32_t t0 = millis();
  while (!gBio.estimation_ready && millis() - t0 < 15000) delay(50);

  if (!gBio.estimation_ready)
    Serial.println("[SYSTEM] BIO not ready after 15s -> starting AMP anyway");

  Serial.println("[SYSTEM] Streaming labeled CA lines: idx | I(uA) | BP | SYS | DIA | HR | SpO2 | TMP | IMU_T | Accel | Gyro");

  // Start AD5940 task
  xTaskCreatePinnedToCore(
    AD5940_Task,
    "AD5940",
    12288,
    NULL,
    2,
    NULL,
    1  // core 1
  );
}

void loop() {
  // tasks do the work
}