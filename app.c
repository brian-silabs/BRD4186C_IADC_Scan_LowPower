/***************************************************************************//**
 * @file
 * @brief Top level application functions
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/


#include "em_letimer.h"
#include "em_iadc.h"
#include "em_ldma.h"
#include "em_prs.h"
#include "em_gpio.h"
#include "em_emu.h"
#include "em_cmu.h"

#define SAMPLING_TIME_TICK                14u // top value for timer @ 32768Hz = CMU_ClockFreqGet(cmuClock_LETIMER0) / SAMPLING_FREQ_HZ
#define LETIMER_IRQ_PRIORITY              0 //Makes use of NVIC_IPRn. See : https://developer.arm.com/documentation/ka001285/latest/

// How many samples to capture
#define NUM_SAMPLES                       128

// Set CLK_ADC to 20MHz
#define CLK_SRC_ADC_FREQ                  20000000 // Targeted CLK_SRC_ADC after prescaling applied set to 20MHz

#if (CLK_SRC_ADC_FREQ == 20000000)
#define HFRCOEM23_CAL_INDEX               9
#elif(CLK_SRC_ADC_FREQ == 38000000)
#define HFRCOEM23_CAL_INDEX               12
#endif

// Takes Errata IADC_E306 into account
#define CLK_ADC_FREQ_GAIN_4X              2500000 // CLK_ADC - 2.5MHz max in gain 4x
#define CLK_ADC_FREQ_GAIN_0P5X            10000000 // CLK_ADC - 10MHz max in 0.5x gain

#define SINGLE_ENDED_CONFIG               0
#define DIFFERENTIAL_CONFIG               1

#define LDMA_IRQ_PRIORITY                 1 //Makes use of NVIC_IPRn. See : https://developer.arm.com/documentation/ka001285/latest/

//Pin config and Analog Buss allocation for IADC
/// ////////////////////////////////////////////////////////////////////////////

#define IADC_POS_INPUT_0_PORT_PIN         iadcPosInputPortBPin3//EXP #3
#define IADC_NEG_INPUT_0_PORT_PIN         iadcNegInputGnd;

/// ////////////////////////////////////////////////////////////////////////////

#define IADC_POS_INPUT_1_PORT_PIN         iadcPosInputPortBPin3//BUTTON 0
#define IADC_NEG_INPUT_1_PORT_PIN         iadcNegInputGnd;

/// ////////////////////////////////////////////////////////////////////////////

#define IADC_POS_INPUT_2_PORT_PIN         iadcPosInputPortBPin3//LED 0
#define IADC_NEG_INPUT_2_PORT_PIN         iadcNegInputGnd;

/// ////////////////////////////////////////////////////////////////////////////

#define IADC_POS_INPUT_3_PORT_PIN         iadcPosInputPortAPin0//EXP #5
#define IADC_NEG_INPUT_3_PORT_PIN         iadcNegInputPortAPin5//EXP #7

/// ////////////////////////////////////////////////////////////////////////////

#define IADC_POS_INPUT_4_PORT_PIN         iadcPosInputPortAPin0//EXP #12 - Do not use with default VCOM
#define IADC_NEG_INPUT_4_PORT_PIN         iadcNegInputPortAPin9//EXP #14 - Do not use with default VCOM

/// ////////////////////////////////////////////////////////////////////////////

#define IADC_POS_INPUT_5_PORT_PIN         iadcPosInputPortAPin0//EXP #11
#define IADC_NEG_INPUT_5_PORT_PIN         iadcNegInputPortAPin7;//EXP #13

/// ////////////////////////////////////////////////////////////////////////////

#define IADC_POS_INPUT_6_PORT_PIN         iadcPosInputPortBPin3//EXP #11
#define IADC_NEG_INPUT_6_PORT_PIN         iadcNegInputGnd//iadcNegInputPortAPin7;//EXP #13

// Number of scan channels
#define IADC_LDMA_CHANNEL                 0 // Highest priority LDMA channel


// Globally declared LDMA link descriptor
LDMA_Descriptor_t descriptor;
// Buffer to store IADC samples
uint32_t scanBuffer[NUM_SAMPLES] = {0xFF};
LDMA_TransferCfg_t transferCfg = LDMA_TRANSFER_CFG_PERIPHERAL(ldmaPeripheralSignal_IADC0_IADC_SCAN);

uint32_t iadcTriggerPrsChannel = 0;

#if DEBUG_IADC_SCAN_DONE_GPIO
uint32_t iadcScanDonePrsChannel = 0;
#endif

#if DEBUG_IADC_SCAN_ENTRY_DONE_GPIO
uint32_t iadcScanEntryDonePrsChannel = 0;
#endif

static bool startedSampling = false;


static void letimerInit(void)
{
  // Declare init struct
  LETIMER_Init_TypeDef init = LETIMER_INIT_DEFAULT;

  // Select LETimer0 clock to run off LFXO
  // Reference: EFR32xG24 RM, Figure 8.3
  CMU_ClockSelectSet(cmuClock_EM23GRPACLK, cmuSelect_LFXO);
  // Enable LETimer0 clock
  CMU_ClockEnable(cmuClock_LETIMER0, true);

  // Initialize letimer to run in free running mode
  // Reference: EFR32xG24 RM, Section 18.3.2
  init.repMode = letimerRepeatFree;
  // Pulse output for PRS
  init.ufoa0 = letimerUFOAPulse;
  // Set desired frequency
  // Warning : Trig each Topvalue+1, so minus 1 is needed, but not done in LETIMER_Init
  init.topValue = SAMPLING_TIME_TICK-1; //CMU_ClockFreqGet(cmuClock_LETIMER0) / SAMPLING_FREQ_HZ;

  // Enable letimer
  //init.enable = true; //We chose to enable the LETIMER once all the other acquisition chain members are
  init.debugRun = false;

  LETIMER_IntClear(LETIMER0, LETIMER_IEN_UF);
  LETIMER_IntEnable(LETIMER0, LETIMER_IEN_UF);

  //NVIC_ClearPendingIRQ(LETIMER0_IRQn);
  //NVIC_EnableIRQ(LETIMER0_IRQn);

  NVIC_ClearPendingIRQ(LETIMER0_IRQn);
  NVIC_DisableIRQ(LETIMER0_IRQn);
  NVIC_SetPriority(LETIMER0_IRQn, LETIMER_IRQ_PRIORITY);

  // Initialize free-running letimer
  LETIMER_Init(LETIMER0, &init);
}

static void initIADC (void)
{
  // Declare init structs
  IADC_Init_t init = IADC_INIT_DEFAULT;
  IADC_AllConfigs_t initAllConfigs = IADC_ALLCONFIGS_DEFAULT;

  IADC_InitScan_t initScan = IADC_INITSCAN_DEFAULT;
  IADC_ScanTable_t initScanTable = IADC_SCANTABLE_DEFAULT; // Scan Table

  // Enable IADC clock
  CMU_ClockEnable(cmuClock_IADC0, true);
  CMU_ClockEnable(cmuClock_GPIO, true);

  // Reset IADC to reset configuration in case it has been modified
  IADC_reset(IADC0);

  // Configure IADC clock source for use while in EM2
  // Set HFRCOEM23 (used by IADC) to specified frequency band
#if defined(CMU_CLKEN0_HFRCOEM23)
  CMU->CLKEN0_SET = CMU_CLKEN0_HFRCOEM23;
#endif
  HFRCOEM23->CAL = DEVINFO->HFRCOEM23CAL[HFRCOEM23_CAL_INDEX].HFRCOEM23CAL;
  HFRCOEM23->CTRL |= HFRCO_CTRL_EM23ONDEMAND;//Constant jitter if WU from EM2 -> 1.4us
  //HFRCOEM23->CTRL |= HFRCO_CTRL_FORCEEN;//REDUCES JITTER, BUT NOT COMLETELY
  CMU_ClockSelectSet(cmuClock_IADCCLK, cmuSelect_HFRCOEM23);

  init.iadcClkSuspend0 = false;//Turn off clocks between scan acquisitions, allows for better power consumption figures

  // Modify init structs and initialize
  init.warmup = iadcWarmupNormal;// This shows 9us between TIMER_TRIGGER and 1st SCAN_ENTRY_DONE


  // Set the HFSCLK prescale value here
  init.srcClkPrescale = IADC_calcSrcClkPrescale(IADC0, CLK_SRC_ADC_FREQ, 0);//0 Will fetch the values of the frequency from the IADC_ClkSel
  init.timebase = IADC_calcTimebase(IADC0, CLK_SRC_ADC_FREQ);//0 Will fetch the values of the frequency from the IADC_ClkSel

  /*
   * Configuration 0 is used by both scan and single conversions by
   * default.  Use internal bandgap as the reference and specify the
   * reference voltage in mV.
   *
   * Resolution is not configurable directly but is based on the
   * selected oversampling ratio (osrHighSpeed), which defaults to
   * 2x and generates 12-bit results.
   */
  //Diff measurements
  initAllConfigs.configs[DIFFERENTIAL_CONFIG].reference = iadcCfgReferenceInt1V2;
  initAllConfigs.configs[DIFFERENTIAL_CONFIG].vRef = 1210;

  initAllConfigs.configs[DIFFERENTIAL_CONFIG].osrHighSpeed = iadcCfgOsrHighSpeed32x;
  initAllConfigs.configs[DIFFERENTIAL_CONFIG].analogGain = iadcCfgAnalogGain4x;

  // Divides CLK_SRC_ADC to set the CLK_ADC frequency
  // Default oversampling (OSR) is 2x, and Conversion Time = ((4 * OSR) + 2) / fCLK_ADC
  // Combined with the 2 cycle delay when switching input channels, total sample rate is 833ksps
  initAllConfigs.configs[DIFFERENTIAL_CONFIG].adcClkPrescale = IADC_calcAdcClkPrescale(IADC0,
                                                                            CLK_ADC_FREQ_GAIN_4X,
                                                                            CLK_SRC_ADC_FREQ,
                                                                     iadcCfgModeNormal,
                                                                     init.srcClkPrescale);

  //SE measurements
  initAllConfigs.configs[SINGLE_ENDED_CONFIG].reference = iadcCfgReferenceInt1V2;
  initAllConfigs.configs[SINGLE_ENDED_CONFIG].vRef = 1210;

  initAllConfigs.configs[SINGLE_ENDED_CONFIG].osrHighSpeed = iadcCfgOsrHighSpeed2x;
  initAllConfigs.configs[SINGLE_ENDED_CONFIG].analogGain = iadcCfgAnalogGain0P5x;

  // Divides CLK_SRC_ADC to set the CLK_ADC frequency
  // Default oversampling (OSR) is 2x, and Conversion Time = ((4 * OSR) + 2) / fCLK_ADC
  // Combined with the 2 cycle delay when switching input channels, total sample rate is 833ksps
  initAllConfigs.configs[SINGLE_ENDED_CONFIG].adcClkPrescale = IADC_calcAdcClkPrescale(IADC0,
                                                                     CLK_ADC_FREQ_GAIN_0P5X,
                                                                     CLK_SRC_ADC_FREQ,
                                                                     iadcCfgModeNormal,
                                                                     init.srcClkPrescale);

  // Scan initialization
  // On every trigger, start conversion
  initScan.triggerAction = iadcTriggerActionOnce;

  // Set conversions to trigger from PRS
  initScan.triggerSelect = iadcTriggerSelPrs0PosEdge;

  initScan.dataValidLevel = iadcFifoCfgDvl8;

  // Set alignment to the left 16 bits
  initScan.alignment = iadcAlignRight16;

  // Enable triggering of scan conversion
  //initScan.start = true;

  // Set to run in EM2
  initScan.fifoDmaWakeup = true;

  initScan.showId = true;

  // Configure entries in scan table
  // 0, 1 & 2 -> Single ended inputs
  // 3, 4 & 5 -> Differential inputs
  // Takes Errata IADC_E306 into account
  initScanTable.entries[0].posInput = IADC_POS_INPUT_0_PORT_PIN;//SE1
  initScanTable.entries[0].negInput = IADC_NEG_INPUT_0_PORT_PIN;
  initScanTable.entries[0].includeInScan = true;
  initScanTable.entries[0].configId = SINGLE_ENDED_CONFIG;

  initScanTable.entries[1].posInput = IADC_POS_INPUT_1_PORT_PIN;//SE2
  initScanTable.entries[1].negInput = IADC_NEG_INPUT_1_PORT_PIN;
  initScanTable.entries[1].includeInScan = true;
  initScanTable.entries[1].configId = SINGLE_ENDED_CONFIG;

  initScanTable.entries[2].posInput = IADC_POS_INPUT_2_PORT_PIN;//SE3
  initScanTable.entries[2].negInput = IADC_NEG_INPUT_2_PORT_PIN;
  initScanTable.entries[2].includeInScan = true;
  initScanTable.entries[2].configId = SINGLE_ENDED_CONFIG;

  initScanTable.entries[3].posInput = IADC_POS_INPUT_3_PORT_PIN;//D1
  initScanTable.entries[3].negInput = IADC_NEG_INPUT_3_PORT_PIN;
  initScanTable.entries[3].includeInScan = true;
  initScanTable.entries[3].configId = DIFFERENTIAL_CONFIG;

  initScanTable.entries[4].posInput = IADC_POS_INPUT_4_PORT_PIN;//D2
  initScanTable.entries[4].negInput = IADC_NEG_INPUT_4_PORT_PIN;
  initScanTable.entries[4].includeInScan = true;
  initScanTable.entries[4].configId = DIFFERENTIAL_CONFIG;

  initScanTable.entries[5].posInput = IADC_POS_INPUT_5_PORT_PIN;//D3
  initScanTable.entries[5].negInput = IADC_NEG_INPUT_5_PORT_PIN;
  initScanTable.entries[5].includeInScan = true;
  initScanTable.entries[5].configId = DIFFERENTIAL_CONFIG;

  initScanTable.entries[6].posInput = IADC_POS_INPUT_6_PORT_PIN;//SE4
  initScanTable.entries[6].negInput = IADC_NEG_INPUT_6_PORT_PIN;
  initScanTable.entries[6].includeInScan = true;
  initScanTable.entries[6].configId = SINGLE_ENDED_CONFIG;

  initScanTable.entries[7].posInput = iadcPosInputAvdd;//SE5, connected to AVDD
  initScanTable.entries[7].negInput = iadcNegInputGnd;
  initScanTable.entries[7].includeInScan = true;
  initScanTable.entries[7].configId = SINGLE_ENDED_CONFIG;

  //Reserve all ABUS ports for IADC
  GPIO->BBUSALLOC |= GPIO_BBUSALLOC_BODD0_ADC0;
  GPIO->BBUSALLOC |= GPIO_BBUSALLOC_BODD1_ADC0;

  GPIO->BBUSALLOC |= GPIO_BBUSALLOC_BEVEN0_ADC0;
  GPIO->BBUSALLOC |= GPIO_BBUSALLOC_BEVEN1_ADC0;


  GPIO->ABUSALLOC |= GPIO_ABUSALLOC_AODD0_ADC0;
  GPIO->ABUSALLOC |= GPIO_ABUSALLOC_AODD1_ADC0;

  GPIO->ABUSALLOC |= GPIO_ABUSALLOC_AEVEN0_ADC0;
  GPIO->ABUSALLOC |= GPIO_ABUSALLOC_AEVEN1_ADC0;

  // Initialize IADC
  IADC_init(IADC0, &init, &initAllConfigs);

  // Initialize Scan
  IADC_initScan(IADC0, &initScan, &initScanTable);

}

static void initLDMA(uint32_t *buffer, uint32_t size)
{
  // Declare LDMA init structs
  LDMA_Init_t init = LDMA_INIT_DEFAULT;

  // Enable LDMA clock branch
  CMU_ClockEnable(cmuClock_LDMA, true);

  init.ldmaInitIrqPriority = LDMA_IRQ_PRIORITY;

  // Initialize LDMA with default configuration
  LDMA_Init(&init);

  // Configure LDMA for transfer from IADC to memory
  // Set up descriptors for buffer transfer
  descriptor = (LDMA_Descriptor_t)LDMA_DESCRIPTOR_LINKREL_P2M_WORD(&IADC0->SCANFIFODATA, buffer, size, 0);

  // Set descriptor to loop NUM_SAMPLES times and run continuously
  descriptor.xfer.decLoopCnt = 0;
  descriptor.xfer.xferCnt = (NUM_SAMPLES - 1) ;
  descriptor.xfer.blockSize = ldmaCtrlBlockSizeUnit8;

  // Interrupt after transfer complete
  descriptor.xfer.doneIfs = 1;
  descriptor.xfer.ignoreSrec = 1;

  // Start transfer, LDMA will sample the IADC NUM_SAMPLES time, and then interrupt
  //LDMA_StartTransfer(IADC_LDMA_CHANNEL, (void*)&transferCfg, (void*)&descriptor);
}


static void initPrs(void)
{
  CMU_ClockEnable(cmuClock_GPIO, true);
  CMU_ClockEnable(cmuClock_PRS, true);

  iadcTriggerPrsChannel = PRS_GetFreeChannel(prsTypeAsync);

  if(iadcTriggerPrsChannel > 5)
  {
    while(1);//wont work on port A B
  }

  // LETIMER --------- PRS CHx --------> IADC0
  PRS_SourceAsyncSignalSet(iadcTriggerPrsChannel,
                           PRS_ASYNC_CH_CTRL_SOURCESEL_LETIMER0,
                           PRS_ASYNC_CH_CTRL_SIGSEL_LETIMER0CH0);

  // Select PRS channel 1 as trigger for IADC Scan trigger
  PRS_ConnectConsumer(iadcTriggerPrsChannel,
                      prsTypeAsync,
                      prsConsumerIADC0_SCANTRIGGER);

#if DEBUG_IADC_SCAN_DONE_GPIO

  iadcScanDonePrsChannel = PRS_GetFreeChannel(prsTypeAsync);
  if(iadcScanDonePrsChannel > 5)
      {
        while(1);//wont work on port A B
      }

  PRS_SourceAsyncSignalSet(iadcScanDonePrsChannel,
                             PRS_ASYNC_CH_CTRL_SOURCESEL_IADC0,
                             PRS_ASYNC_CH_CTRL_SIGSEL_IADC0SCANDONE);

  GPIO_PinModeSet(IADC_SCAN_DONE_GPIO_PORT, IADC_SCAN_DONE_GPIO_PIN, gpioModePushPull,1);
  PRS_PinOutput(iadcScanDonePrsChannel, prsTypeAsync, IADC_SCAN_DONE_GPIO_PORT, IADC_SCAN_DONE_GPIO_PIN);
#endif //#if DEBUG_IADC_SCAN_DONE_GPIO

#if DEBUG_IADC_SCAN_ENTRY_DONE_GPIO
  GPIO_PinModeSet(IADC_SCAN_ENTRY_DONE_GPIO_PORT, IADC_SCAN_ENTRY_DONE_GPIO_PIN, gpioModePushPull,1);
  iadcScanEntryDonePrsChannel = PRS_GetFreeChannel(prsTypeAsync);
  if(iadcScanEntryDonePrsChannel > 5)
  {
    while(1);//wont work on port A B
  }

  PRS_SourceAsyncSignalSet(iadcScanEntryDonePrsChannel,
                             PRS_ASYNC_CH_CTRL_SOURCESEL_IADC0,
                             PRS_ASYNC_CH_CTRL_SIGSEL_IADC0SCANENTRYDONE);

  PRS_PinOutput(iadcScanEntryDonePrsChannel, prsTypeAsync, IADC_SCAN_ENTRY_DONE_GPIO_PORT, IADC_SCAN_ENTRY_DONE_GPIO_PIN);

#endif //#if DEBUG_IADC_SCAN_ENTRY_DONE_GPIO

#if DEBUG_TIMER_TRIGGER_GPIO
  GPIO_PinModeSet(TIMER_TRIGGER_GPIO_PORT, TIMER_TRIGGER_GPIO_PIN, gpioModePushPull,1);
  PRS_PinOutput(iadcTriggerPrsChannel, prsTypeAsync, TIMER_TRIGGER_GPIO_PORT, TIMER_TRIGGER_GPIO_PIN);
  //CMU_ClkOutPinConfig(HFRCO_CLKOUT_NO, cmuSelect_HFRCOEM23, 1, HFRCO_GPIO_PORT, HFRCO_GPIO_PIN);
#endif

}

static void startSampling(void)
{
  if( !startedSampling )
  {
    //Load Scan sequence (untriggered)
    IADC_command(IADC0, iadcCmdStartScan);
    // Start LDMA (untriggered)
    LDMA_StartTransfer(IADC_LDMA_CHANNEL, (void*)&transferCfg, (void*)&descriptor);
    // One of the desc has doneIfs set, so interrupt should be activated on the channel :
    LDMA->IEN |= 1UL << (uint8_t)IADC_LDMA_CHANNEL; // Allow  interrupt to be fired by any desc
    // Set flag to indicate sampling is occuring.
    startedSampling = true;
    //Enable LETIMER (will trigger after 14 ticks)
    LETIMER_Enable(LETIMER0, true);
  }
}

void LDMA_IRQHandler(void)
{
  // Clear interrupt flags
  LDMA_IntClear(LDMA_IF_DONE0);
}

/***************************************************************************//**
 * Initialize application.
 ******************************************************************************/
void app_init(void)
{
  letimerInit();
  initIADC();
  initLDMA(scanBuffer, NUM_SAMPLES);
  initPrs();

  startSampling();
}

/***************************************************************************//**
 * App ticking function.
 ******************************************************************************/
void app_process_action(void)
{
  EMU_EnterEM2(true);
}
