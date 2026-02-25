/*
  AD5940_Amperometric_Square_Wave.ino
  - SWV scan with BLE streaming
  - Parks at ~0V after each sweep for 60s, then restarts
  - Runs AD5940_Main() on a separate FreeRTOS task so BLE notifications stay reliable (ESP32)
*/

extern "C" {
  #include "ad5940.h"
  #include "SqrWaveVoltammetry.h"
  #include "BleComm.h"
}

#include <Arduino.h>

/** User could configure following parameters **/
#define APPBUFF_SIZE 1024
uint32_t AppBuff[APPBUFF_SIZE];
float LFOSCFreq; /* Measured LFOSC frequency */

/*
  Print/stream SWV results.
  NOTE: pData is already in microamps (uA) inside AppSWVDataProcess().
*/
static int32_t RampShowResult(float *pData, uint32_t DataCount)
{
  static uint32_t index = 0;

  for (uint32_t i = 0; i < DataCount; i++)
  {
    uint32_t step = AppSWV_StreamNextStepIndex();
    char phase = AppSWV_StreamNextPhaseChar(); // 'F' (high) / 'R' (low) per your SWV engine

    // BLE: send SWV-aware packet
    ble_push_swv(index, step, phase, pData[i]);

    // UART: mirror
    printf("idx:%lu, step:%lu, phase:%c, I_uA:%.9f\n",
           (unsigned long)index,
           (unsigned long)step,
           phase,
           (double)pData[i]);

    index++;
  }
  return 0;
}

/*
  Platform configuration (FIFO/Sequencer/Clock/GPIO/LFOSC measure)
*/
static int32_t AD5940PlatformCfg(void)
{
  CLKCfg_Type clk_cfg;
  SEQCfg_Type seq_cfg;
  FIFOCfg_Type fifo_cfg;
  AGPIOCfg_Type gpio_cfg;
  LFOSCMeasure_Type LfoscMeasure;

  AD5940_HWReset();
  AD5940_Initialize();

  /* Step1: Configure clock */
  clk_cfg.HFOSCEn = bTRUE;
  clk_cfg.HFXTALEn = bFALSE;
  clk_cfg.LFOSCEn = bTRUE;
  clk_cfg.HfOSC32MHzMode = bFALSE;
  clk_cfg.SysClkSrc = SYSCLKSRC_HFOSC;
  clk_cfg.SysClkDiv = SYSCLKDIV_1;
  clk_cfg.ADCCLkSrc = ADCCLKSRC_HFOSC;
  clk_cfg.ADCClkDiv = ADCCLKDIV_1;
  AD5940_CLKCfg(&clk_cfg);

  /* Step2: Configure FIFO and Sequencer */
  fifo_cfg.FIFOEn = bTRUE; /* enable later by App if desired */
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

  /* Step3: Interrupt controller */
  AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ALLINT, bTRUE);
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  AD5940_INTCCfg(AFEINTC_0,
                 AFEINTSRC_DATAFIFOTHRESH | AFEINTSRC_ENDSEQ | AFEINTSRC_CUSTOMINT0,
                 bTRUE);
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);

  /* Step4: GPIOs */
  gpio_cfg.FuncSet = GP0_INT | GP1_SLEEP | GP2_SYNC;
  gpio_cfg.InputEnSet = 0;
  gpio_cfg.OutputEnSet = AGPIO_Pin0 | AGPIO_Pin1 | AGPIO_Pin2;
  gpio_cfg.OutVal = 0;
  gpio_cfg.PullEnSet = 0;
  AD5940_AGPIOCfg(&gpio_cfg);

  /* Measure LFOSC frequency */
  LfoscMeasure.CalDuration = 1000.0;
  LfoscMeasure.CalSeqAddr = 0;
  LfoscMeasure.SystemClkFreq = 16000000.0f;
  AD5940_LFOSCMeasure(&LfoscMeasure, &LFOSCFreq);
  printf("LFOSC Freq:%f\n", LFOSCFreq);

  return 0;
}

/*
  SWV parameter initialization
*/
void AD5940RampStructInit(void)
{
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

  // SWV settings (same as your repo)
  pRampCfg->RampStartVolt = 100.0f;
  pRampCfg->RampPeakVolt = 500.0f;
  pRampCfg->VzeroStart = 1300.0f;
  pRampCfg->VzeroPeak = 1300.0f;
  pRampCfg->Frequency = 400;
  pRampCfg->SqrWvAmplitude = 50;
  pRampCfg->SqrWvRampIncrement = 2;
  pRampCfg->SampleDelay = 0.2f;
  pRampCfg->LPTIARtiaSel = LPTIARTIA_1K;
  pRampCfg->bRampOneDir = bTRUE;
}

/*
  Main SWV loop: runs sweeps, then parks at ~0V for 60s, then restarts.
*/
void AD5940_Main(void)
{
  uint32_t temp;

  const uint32_t RESTART_DELAY_MS = 60000; // 1 minute
  bool waiting_for_restart = false;
  uint32_t run_complete_ms = 0;

  AD5940PlatformCfg();
  AD5940RampStructInit();
  AppSWVInit(AppBuff, APPBUFF_SIZE);

  AD5940_Delay10us(100000); // allow equilibrium
  AppSWVCtrl(APPCTRL_START, 0);

  while (1)
  {
    // If a sweep finished, wait 60s, then restart
    if (waiting_for_restart)
    {
      if ((uint32_t)(millis() - run_complete_ms) >= RESTART_DELAY_MS)
      {
        AD5940RampStructInit();
        AppSWVInit(AppBuff, APPBUFF_SIZE);
        AppSWVCtrl(APPCTRL_START, 0);
        waiting_for_restart = false;
        printf("[SWV] Restarting sweep after 60s park delay\n");
      }
    }

    if (AD5940_GetMCUIntFlag())
    {
      AD5940_ClrMCUIntFlag();
      temp = APPBUFF_SIZE;
      AppSWVISR(AppBuff, &temp);
      RampShowResult((float *)AppBuff, temp);

      if (AppSWV_ConsumeRunCompleteFlag())
      {
        waiting_for_restart = true;
        run_complete_ms = millis();
        printf("[SWV] Sweep complete. Parked at ~0 mV. Waiting 60s...\n");
      }
    }

    // Yield so BLE + system tasks keep running smoothly
    vTaskDelay(1);
  }
}

/* =======================
   FreeRTOS task wrapper
   ======================= */
void AD5940_Task(void *pv)
{
  // Your MCU init prints from serial monitor:
  printf("Attempting to initialise MCU...\n");
  AD5940_MCUResourceInit(NULL);
  printf("initialized\n");

  AD5940_Main(); // infinite loop
  vTaskDelete(NULL);
}

/* =======================
   Arduino entry points
   ======================= */
void setup()
{
  Serial.begin(115200);
  Serial.println("start");

  ble_init();     // start BLE advertising
  delay(200);     // small delay (no need for 20s)

  // Run AD5940 on its own task/core to avoid starving BLE
  xTaskCreatePinnedToCore(
      AD5940_Task,
      "AD5940",
      12288,       // stack (safer than 8192)
      NULL,
      2,           // priority
      NULL,
      1            // core
  );
}

void loop()
{

}

