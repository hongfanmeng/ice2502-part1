#include <stdbool.h>
#include <stdint.h>

#include "driverlib/adc.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "drivers/pinout.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "tm1638.h"

/**
 * defines
 */
#define SYSTICK_FREQUENCY 50  // SysTick freq 50 Hz
#define SYSTICK_PERIOD_MS 20  // SysTick period 20ms
#define QUEUE_SIZE 128        // Queue size

/**
 * functions
 */
void GPIOInit(void);
void ADCInit(void);
void SysTickInit(void);
void DevicesInit(void);
void SysTickHandler(void);
uint32_t ADCSampleCurrent(void);
uint32_t ADCSampleVoltage(void);

/**
 * variables
 */

// counters
volatile uint8_t clock100ms = 0;
volatile uint8_t clock100msFlag = 0;

/**
 * digit: 7seg display numbers
 * pnt:   7seg dot display with bit flag, 1=on, 0=off
 * 7seg on board with order 4,5,6,7, 0,1,2,3
 */
uint8_t digit[8] = {' ', ' ', ' ', ' ', ' ', ' ', ' ', ' '};
uint8_t pnt = 0x11;

/**
 * LED status on baord
 * 0=off, 1=on
 * LED1 to LED8
 */
uint8_t led[] = {0, 0, 0, 0, 0, 0, 0, 0};

// system clock freq
uint32_t ui32SysClock;

// voltage and current queue avg
uint32_t voltQue[QUEUE_SIZE], voltSum = 0, vpos = 0;
uint32_t currQue[QUEUE_SIZE], currSum = 0, cpos = 0;

int main(void) {
  DevicesInit();

  // delay wait for TM1638 ready, wait for 60ms
  while (clock100ms < 3)
    ;

  TM1638_Init();

  while (1) {
    // calc voltage avg of past QUEUE_SIZE samples
    voltQue[vpos++] = ADCSampleVoltage();
    voltSum += voltQue[vpos - 1];
    voltSum -= voltQue[vpos % QUEUE_SIZE];
    vpos %= QUEUE_SIZE;
    uint32_t volt = voltSum / QUEUE_SIZE;

    // convert ADC to voltage
    volt = volt * 1629 / 1000;

    digit[4] = volt / 1000;
    digit[5] = volt / 100 % 10;
    digit[6] = volt / 10 % 10;
    digit[7] = volt % 10;

    // calc current avg of past QUEUE_SIZE samples
    currQue[cpos++] = ADCSampleCurrent();
    currSum += currQue[cpos - 1];
    currSum -= currQue[cpos % QUEUE_SIZE];
    cpos %= QUEUE_SIZE;
    uint32_t curr = currSum / QUEUE_SIZE;

    // convert ADC output to current
    curr = curr * 803 / 1000;
    // avoid overflow
    curr -= curr <= 22 ? curr : 22;

    digit[0] = (curr / 1000) % 10;
    digit[1] = (curr / 100) % 10;
    digit[2] = (curr / 10) % 10;
    digit[3] = curr % 10;
  }
}

/**
 * @brief initialize GPIO
 *
 */
void GPIOInit(void) {
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);  // enable port K
  while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOK)) {
  };  // wait for port K to be ready

  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
  while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOM)) {
  };  // wait for port M to be ready

  // set PK4, PK5 as output port
  GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_4 | GPIO_PIN_5);
  // set PM0 as output port
  GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_0);
}
/**
 * @brief Initialize ADC moudule
 *
 */
void ADCInit(void) {
  // enable ADC0
  SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

  // set PE0, PE1 as ADC input
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
  GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1);

  // set 64x hardware over sampling for ADC0
  ADCHardwareOversampleConfigure(ADC0_BASE, 64);

  // config ADC0, single sampling for sequencer 0, 1
  ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);
  ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 1);

  // set 8x software over sampling for ADC0, sequencer 0
  ADCSoftwareOversampleConfigure(ADC0_BASE, 0, 8);
  // set 4x software over sampling for ADC0, sequencer 1
  ADCSoftwareOversampleConfigure(ADC0_BASE, 1, 4);

  // config ADC0
  // sequencer 0, channel 3, PE0, current
  // sequencer 1, channel 2, PE1, voltage
  ADCSoftwareOversampleStepConfigure(ADC0_BASE, 0, 0,
                                     ADC_CTL_CH3 | ADC_CTL_IE | ADC_CTL_END);
  ADCSoftwareOversampleStepConfigure(ADC0_BASE, 1, 0,
                                     ADC_CTL_CH2 | ADC_CTL_IE | ADC_CTL_END);

  // enable ADC0 sample sequence 0, 1
  ADCSequenceEnable(ADC0_BASE, 0);
  ADCSequenceEnable(ADC0_BASE, 1);

  // clear interrupt flag before sampling
  ADCIntClear(ADC0_BASE, 0);
  ADCIntClear(ADC0_BASE, 1);
}

/**
 * @brief ADC sampling for current
 *
 * @return uint32_t, the value of ADC sampling of current [0-4095]
 */
uint32_t ADCSampleCurrent(void) {
  uint32_t pui32ADC0Value[8];

  // trigger adc sampling
  ADCProcessorTrigger(ADC0_BASE, 0);

  // wait for sampling
  while (!ADCIntStatus(ADC0_BASE, 0, false)) {
  }

  // clear interrupt flag
  ADCIntClear(ADC0_BASE, 0);

  // read adc sampling value
  ADCSoftwareOversampleDataGet(ADC0_BASE, 0, pui32ADC0Value, 8);

  // get the avg value
  uint32_t ADC0Sum = 0;
  for (int i = 0; i < 8; i++) {
    ADC0Sum += pui32ADC0Value[i];
  }

  // return adc sampling value
  return ADC0Sum >> 3;
}

/**
 * @brief ADC sampling for voltage
 *
 * @return uint32_t, the value of ADC sampling of voltage [0-4095]
 */
uint32_t ADCSampleVoltage(void) {
  uint32_t pui32ADC0Value[4];

  // trigger adc sampling
  ADCProcessorTrigger(ADC0_BASE, 1);

  // wait for sampling
  while (!ADCIntStatus(ADC0_BASE, 1, false)) {
  }

  // clear interrupt flag
  ADCIntClear(ADC0_BASE, 1);

  // read adc sampling value
  ADCSoftwareOversampleDataGet(ADC0_BASE, 1, pui32ADC0Value, 4);

  // get the avg value
  uint32_t ADC0Sum = 0;
  for (int i = 0; i < 4; i++) {
    ADC0Sum += pui32ADC0Value[i];
  }

  // return adc sampling value
  // return (ADC0Sum * 6615) >> 14;
  return ADC0Sum >> 2;
}

/**
 * @brief initialize SysTick
 *
 */
void SysTickInit(void) {
  SysTickIntRegister(SysTickHandler);
  SysTickPeriodSet(ui32SysClock /
                   SYSTICK_FREQUENCY);  // set SysTick interrupt period 20ms
  SysTickEnable();                      // enable systick
  SysTickIntEnable();                   // enable systick interrupt
}

/**
 * @brief Devices initialization, clock setting, gpio initialization
 * and systick initialization
 *
 */
void DevicesInit(void) {
  // set system clock to 20MHz
  ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
                                     SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480),
                                    20000000);
  PinoutSet(false, false);
  GPIOInit();         // GPIO initialization
  ADCInit();          // ADC initialization
  SysTickInit();      // SysTick initialization
  IntMasterEnable();  // Enable interrupts to the processor.
}

/**
 * @brief SysTick interrupt handler
 *
 */
void SysTickHandler(void) {
  // counters
  if (++clock100ms >= 5) {
    clock100msFlag = 1;
    clock100ms = 0;
  }

  // refresh 7seg and led
  TM1638_RefreshDIGIandLED(digit, pnt, led);
}
