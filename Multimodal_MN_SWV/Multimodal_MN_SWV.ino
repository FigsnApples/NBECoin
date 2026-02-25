/*
  AD5940_Amperometric_Square_Wave.ino
  - SWV scan with BLE streaming (unchanged BLE format)
  - Adds biomarkers: MAX32664 (BP/HR/SpO2), TMP117, ICM20948
  - SWV does NOT start until BP calibration succeeds + estimation mode is active
  - Serial prints combined CSV per SWV sample:
    idx,step,phase,I_uA,sys,dia,hr,spo2,tmp117_temp_C,icm_temp_C,ax,ay,az,gx,gy,gz
*/

#include <Arduino.h>
#include <Wire.h>

// ===== MAX32664 + TMP117 + ICM20948 =====
#include "max32664.h"
#include <Adafruit_TMP117.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>

// !!! IMPORTANT: CHANGE THESE TWO PINS TO FREE GPIOs ON YOUR BOARD !!!
// AD5940 port in your repo uses GPIO2/3/4/1/7/8/9, so don't reuse those.
#define MAX_RESET_PIN 2//10
#define MAX_MFIO_PIN  3//11
#define RAWDATA_BUFFLEN 250

max32664 MAX32664(MAX_RESET_PIN, MAX_MFIO_PIN, RAWDATA_BUFFLEN);
Adafruit_TMP117 tmp117;            // default 0x4B
Adafruit_ICM20948 icm;             // default 0x69

// ===== AD5940/SWV + BLE (your repo) =====
extern "C" {
  #include "ad5940.h"
  #include "SqrWaveVoltammetry.h"
  #include "BleComm.h"
}

// ===== Shared SWV buffer =====
#define APPBUFF_SIZE 1024
uint32_t AppBuff[APPBUFF_SIZE];
float LFOSCFreq;

// ===== Biomarker snapshot shared with SWV printing =====
typedef struct {
  volatile bool estimation_ready;

  // MAX32664
  volatile float sys;
  volatile float dia;
  volatile float hr;
  volatile float spo2;

  // TMP117
  volatile float tmp117_C;

  // ICM20948
  volatile float icm_temp_C;
  volatile float ax, ay, az;
  volatile float gx, gy, gz;
} BiomarkerSnapshot;

static BiomarkerSnapshot gBio = {
  .estimation_ready = false,
  .sys = NAN, .dia = NAN, .hr = NAN, .spo2 = NAN,
  .tmp117_C = NAN,
  .icm_temp_C = NAN,
  .ax = NAN, .ay = NAN, .az = NAN,
  .gx = NAN, .gy = NAN, .gz = NAN
};

// ---------- MAX32664 optional interrupt handler ----------
void mfioInterruptHndlr() {
  // optional
}
static void enableInterruptPin() {
  attachInterrupt(digitalPinToInterrupt(MAX32664.mfioPin), mfioInterruptHndlr, FALLING);
}

// ---------- Your calibration loader ----------
static void loadAlgomodeParameters() {
  algomodeInitialiser algoParameters;

  // Replace with your own calibration values from a reference device
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

// ---------- Biomarker task: init + calibration + continuous updates ----------
static void Biomarker_Task(void *pv) {
  (void)pv;

  // I2C already started in setup()
  loadAlgomodeParameters();

  Serial.println("[BIO] Initializing MAX32664 sensor hub...");
  int result = MAX32664.hubBegin();
  if (result == CMD_SUCCESS) {
    Serial.println("[BIO] MAX32664 hub begin OK!");
  } else {
    Serial.println("[BIO] ERROR: Could not communicate with MAX32664. Check connections.");
    // Don't start SWV if biomarkers are required
    while (1) vTaskDelay(pdMS_TO_TICKS(5000));
  }

  Serial.println("[BIO] Initializing TMP117...");
  if (!tmp117.begin()) {
    Serial.println("[BIO] TMP117 not found -> temp will be N/A");
  } else {
    Serial.println("[BIO] TMP117 found!");
  }

  Serial.println("[BIO] Initializing ICM20948...");
  if (!icm.begin_I2C()) {
    Serial.println("[BIO] ICM20948 not found -> IMU will be N/A");
  } else {
    Serial.println("[BIO] ICM20948 found!");
  }

  Serial.println("[BIO] Starting BP calibration...");
  bool ret = MAX32664.startBPTcalibration();
  while (!ret) {
    Serial.println("[BIO] BP calibration failed - retrying...");
    vTaskDelay(pdMS_TO_TICKS(10000));
    ret = MAX32664.startBPTcalibration();
  }
  Serial.println("[BIO] BP calibration succeeded!");

  vTaskDelay(pdMS_TO_TICKS(1000));

  Serial.println("[BIO] Entering estimation mode...");
  ret = MAX32664.configAlgoInEstimationMode();
  while (!ret) {
    Serial.println("[BIO] Failed to enter estimation mode - retrying...");
    vTaskDelay(pdMS_TO_TICKS(10000));
    ret = MAX32664.configAlgoInEstimationMode();
  }
  Serial.println("[BIO] Estimation mode active!");

  // If you want MFIO interrupt-driven, uncomment:
  // enableInterruptPin();

  gBio.estimation_ready = true;

  // Continuous updates at ~10 Hz
  while (1) {
    // MAX32664 samples (updates when available)
    uint8_t num_samples = MAX32664.readSamples();
    if (num_samples) {
      gBio.sys  = MAX32664.max32664Output.sys;
      gBio.dia  = MAX32664.max32664Output.dia;
      gBio.hr   = MAX32664.max32664Output.hr;
      gBio.spo2 = MAX32664.max32664Output.spo2;
    }

    // TMP117 (always attempt)
    sensors_event_t tmp_event;
    if (tmp117.begin()) { // safe if already began; keeps behavior robust across boards
      tmp117.getEvent(&tmp_event);
      gBio.tmp117_C = tmp_event.temperature;
    } else {
      gBio.tmp117_C = NAN;
    }

    // ICM20948 (always attempt)
    sensors_event_t accel, gyro, icm_temp, mag;
    if (icm.begin_I2C()) {
      icm.getEvent(&accel, &gyro, &icm_temp, &mag);
      gBio.icm_temp_C = icm_temp.temperature;

      gBio.ax = accel.acceleration.x;
      gBio.ay = accel.acceleration.y;
      gBio.az = accel.acceleration.z;

      gBio.gx = gyro.gyro.x;
      gBio.gy = gyro.gyro.y;
      gBio.gz = gyro.gyro.z;
    } else {
      gBio.icm_temp_C = NAN;
      gBio.ax = gBio.ay = gBio.az = NAN;
      gBio.gx = gBio.gy = gBio.gz = NAN;
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// ---------- SWV result printer (Serial combined CSV) + BLE unchanged ----------
static int32_t RampShowResult(float *pData, uint32_t DataCount) {
  static uint32_t index = 0;

  for (uint32_t i = 0; i < DataCount; i++) {
    uint32_t step = AppSWV_StreamNextStepIndex();
    char phase = AppSWV_StreamNextPhaseChar(); // 'F' or 'R'

    // BLE stays the same as your current pipeline expects:
    ble_push_swv(index, step, phase, pData[i]);

    // Serial combined CSV (one line per SWV sample)
    Serial.print(index); Serial.print(",");
    Serial.print(step);  Serial.print(",");
    Serial.print(phase); Serial.print(",");
    Serial.print(pData[i], 9); Serial.print(",");

    // biomarkers (cached)
    Serial.print(gBio.sys, 0);  Serial.print(",");
    Serial.print(gBio.dia, 0);  Serial.print(",");
    Serial.print(gBio.hr,  2);  Serial.print(",");
    Serial.print(gBio.spo2,2);  Serial.print(",");

    if (!isnan((float)gBio.tmp117_C)) Serial.print((float)gBio.tmp117_C, 3);
    else Serial.print("N/A");
    Serial.print(",");

    if (!isnan((float)gBio.icm_temp_C)) Serial.print((float)gBio.icm_temp_C, 2);
    else Serial.print("N/A");
    Serial.print(",");

    Serial.print((float)gBio.ax, 2); Serial.print(",");
    Serial.print((float)gBio.ay, 2); Serial.print(",");
    Serial.print((float)gBio.az, 2); Serial.print(",");

    Serial.print((float)gBio.gx, 3); Serial.print(",");
    Serial.print((float)gBio.gy, 3); Serial.print(",");
    Serial.print((float)gBio.gz, 3);

    Serial.println();
    index++;
  }
  return 0;
}

// ---------- AD5940 platform config ----------
static int32_t AD5940PlatformCfg(void) {
  CLKCfg_Type clk_cfg;
  SEQCfg_Type seq_cfg;
  FIFOCfg_Type fifo_cfg;
  AGPIOCfg_Type gpio_cfg;
  LFOSCMeasure_Type LfoscMeasure;

  AD5940_HWReset();
  AD5940_Initialize();

  clk_cfg.HFOSCEn = bTRUE;
  clk_cfg.HFXTALEn = bFALSE;
  clk_cfg.LFOSCEn = bTRUE;
  clk_cfg.HfOSC32MHzMode = bFALSE;
  clk_cfg.SysClkSrc = SYSCLKSRC_HFOSC;
  clk_cfg.SysClkDiv = SYSCLKDIV_1;
  clk_cfg.ADCCLkSrc = ADCCLKSRC_HFOSC;
  clk_cfg.ADCClkDiv = ADCCLKDIV_1;
  AD5940_CLKCfg(&clk_cfg);

  fifo_cfg.FIFOEn = bTRUE;
  fifo_cfg.FIFOMode = FIFOMODE_FIFO;
  fifo_cfg.FIFOSize = FIFOSIZE_4KB;
  fifo_cfg.FIFOSrc = FIFOSRC_SINC3;
  fifo_cfg.FIFOThresh = 4;
  AD5940_FIFOCfg(&fifo_cfg);

  seq_cfg.SeqMemSize = SEQMEMSIZE_2KB;
  seq_cfg.SeqBreakEn = bFALSE;
  seq_cfg.SeqIgnoreEn = bTRUE;
  seq_cfg.SeqCntCRCClr = bTRUE;
  seq_cfg.SeqEnable = bFALSE;
  seq_cfg.SeqWrTimer = 0;
  AD5940_SEQCfg(&seq_cfg);

  AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ALLINT, bTRUE);
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  AD5940_INTCCfg(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH | AFEINTSRC_ENDSEQ | AFEINTSRC_CUSTOMINT0, bTRUE);
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);

  gpio_cfg.FuncSet = GP0_INT | GP1_SLEEP | GP2_SYNC;
  gpio_cfg.InputEnSet = 0;
  gpio_cfg.OutputEnSet = AGPIO_Pin0 | AGPIO_Pin1 | AGPIO_Pin2;
  gpio_cfg.OutVal = 0;
  gpio_cfg.PullEnSet = 0;
  AD5940_AGPIOCfg(&gpio_cfg);

  LfoscMeasure.CalDuration = 1000.0;
  LfoscMeasure.CalSeqAddr = 0;
  LfoscMeasure.SystemClkFreq = 16000000.0f;
  AD5940_LFOSCMeasure(&LfoscMeasure, &LFOSCFreq);

  printf("LFOSC Freq:%f\n", LFOSCFreq);
  return 0;
}

// ---------- SWV parameter init (your existing settings) ----------
void AD5940RampStructInit(void) {
  AppSWVCfg_Type *pRampCfg;
  AppSWVGetCfg(&pRampCfg);

  pRampCfg->SeqStartAddr = 0x10;
  pRampCfg->MaxSeqLen = 512 - 0x10;

  pRampCfg->RcalVal = 10000.0;
  pRampCfg->ADCRefVolt = 1.820f;

  pRampCfg->FifoThresh = 1023;
  pRampCfg->SysClkFreq = 16000000.0f;
  pRampCfg->LFOSCClkFreq = LFOSCFreq;

  pRampCfg->AdcPgaGain = ADCPGA_1P5;
  pRampCfg->ADCSinc3Osr = ADCSINC3OSR_4;

  // SWV settings
  pRampCfg->RampStartVolt = 100.0f;
  pRampCfg->RampPeakVolt = 500.0f;
  pRampCfg->VzeroStart = 1300.0f;
  pRampCfg->VzeroPeak  = 1300.0f;

  pRampCfg->Frequency = 400;
  pRampCfg->SqrWvAmplitude = 50;
  pRampCfg->SqrWvRampIncrement = 2;
  pRampCfg->SampleDelay = 0.2f;

  pRampCfg->LPTIARtiaSel = LPTIARTIA_1K;
  pRampCfg->bRampOneDir = bTRUE;
}

// ---------- Main SWV loop ----------
void AD5940_Main(void) {
  uint32_t temp;

  const uint32_t RESTART_DELAY_MS = 60000; // 1 minute park after sweep
  bool waiting_for_restart = false;
  uint32_t run_complete_ms = 0;

  AD5940PlatformCfg();
  AD5940RampStructInit();
  AppSWVInit(AppBuff, APPBUFF_SIZE);

  AD5940_Delay10us(100000); // equilibrium
  AppSWVCtrl(APPCTRL_START, 0);

  while (1) {
    if (waiting_for_restart) {
      if ((uint32_t)(millis() - run_complete_ms) >= RESTART_DELAY_MS) {
        AD5940RampStructInit();
        AppSWVInit(AppBuff, APPBUFF_SIZE);
        AppSWVCtrl(APPCTRL_START, 0);
        waiting_for_restart = false;
        printf("[SWV] Restarting sweep after 60s park delay\n");
      }
    }

    if (AD5940_GetMCUIntFlag()) {
      AD5940_ClrMCUIntFlag();
      temp = APPBUFF_SIZE;
      AppSWVISR(AppBuff, &temp);
      RampShowResult((float *)AppBuff, temp);

      if (AppSWV_ConsumeRunCompleteFlag()) {
        waiting_for_restart = true;
        run_complete_ms = millis();
        printf("[SWV] Sweep complete. Parked at ~0 mV. Waiting 60s...\n");
      }
    }

    vTaskDelay(1); // keep BLE + system healthy
  }
}

// ---------- FreeRTOS wrapper for AD5940 ----------
void AD5940_Task(void *pv) {
  (void)pv;
  printf("Attempting to initialise MCU...\n");
  AD5940_MCUResourceInit(NULL);
  printf("initialized\n");
  AD5940_Main();
  vTaskDelete(NULL);
}

// ---------- Arduino entry points ----------
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("start");

  // Start I2C (TMP117 + ICM20948 + MAX32664)
  Wire.begin();

  // Start BLE now (doesn't change your BLE stream format)
  ble_init();
  delay(200);

  // Start biomarker init/calibration task
  xTaskCreatePinnedToCore(
    Biomarker_Task,
    "BIO",
    8192,
    NULL,
    2,
    NULL,
    0  // core 0
  );

  // Wait here until MAX32664 is calibrated and estimation mode is active
  Serial.println("[SYSTEM] Waiting for BP calibration + estimation mode before starting SWV...");
  while (!gBio.estimation_ready) {
    delay(50);
  }

  // Print combined CSV header for Serial logging
  Serial.println("idx,step,phase,I_uA,sys,dia,hr,spo2,tmp117_temp_C,icm_temp_C,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z");

  // Now start AD5940 on its own core/task
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
  // nothing (tasks do the work)
}
