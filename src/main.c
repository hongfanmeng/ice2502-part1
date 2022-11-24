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
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "tm1638.h"

/**
 * defines
 */
#define SYSTICK_FREQUENCY 50  // SysTick freq 50 Hz, 20ms

#define V_T40ms 2   // 40ms = 2 * 20ms
#define V_T100ms 5  // 0.1s = 5 * 20ms
/**
 * functions
 */
void GPIOInit(void);
void ADCInit(void);
void SysTickInit(void);
void DevicesInit(void);
void SysTickHandler(void);
uint32_t ADCSample(void);

/**
 * variables
 */

// software counter
uint8_t clock40ms = 0;
uint8_t clock100ms = 0;

// software counter overflow flag
uint8_t clock40ms_flag = 0;
uint8_t clock100ms_flag = 0;

/**
 * digit: 7seg display numbers
 * pnt:   7seg dot display with bit flag, 1=on, 0=off
 * 7seg on board with order 4,5,6,7 0,1,2,3
 */
uint8_t digit[8] = {' ', ' ', ' ', ' ', ' ', ' ', ' ', ' '};
uint8_t pnt = 0x2;

/**
 * LED status on baord
 * 0=off, 1=on
 * LED8 to LED1
 */
uint8_t led[] = {0, 0, 0, 0, 0, 0, 0, 0};

// system clock freq
uint32_t ui32SysClock;

// ADC0 sampling value (0 - 4095)
uint32_t ui32ADC0Value;

// ADC0 voltage, 0000 - 3300 (0.000, 3.300)
uint32_t ui32ADC0Voltage;

int main(void) {
  uint8_t temp, i;

  DevicesInit();

  // delay wait for TM1638 ready
  while (clock100ms < 3)
    ;

  TM1638_Init();

  while (1) {
    // check 40ms counter
    if (clock40ms_flag == 1) {
      clock40ms = 0;

      ui32ADC0Value = ADCSample();

      // convert sampling value to 7seg
      digit[4] = ui32ADC0Value / 1000;      // 显示ADC采样值千位数
      digit[5] = ui32ADC0Value / 100 % 10;  // 显示ADC采样值百位数
      digit[6] = ui32ADC0Value / 10 % 10;   // 显示ADC采样值十位数
      digit[7] = ui32ADC0Value % 10;        // 显示ADC采样值个位数

      ui32ADC0Voltage = ui32ADC0Value * 330 / 4095;

      digit[1] = (ui32ADC0Voltage / 100) % 10;  // 显示电压值个位数
      digit[2] = (ui32ADC0Voltage / 10) % 10;   // 显示电压值十分位数
      digit[3] = ui32ADC0Voltage % 10;          // 显示电压值百分位数
    }
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

  // use PE1 as ADC input
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
  GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_1);

  // single sampling(sample sequence 3 mode)
  ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);

  ADCSequenceStepConfigure(ADC0_BASE, 3, 0,
                           ADC_CTL_CH2 | ADC_CTL_IE | ADC_CTL_END);

  // enable ADC0 sample sequence 3
  ADCSequenceEnable(ADC0_BASE, 3);

  // clear interrupt flag before sampling
  ADCIntClear(ADC0_BASE, 3);
}

/**
 * @brief ADC sampling
 *
 * @return uint32_t the value of ADC sampling [0-4095]
 */
uint32_t ADCSample(void) {
  //
  // This array is used for storing the data read from the ADC FIFO. It
  // must be as large as the FIFO for the sequencer in use.  This example
  // uses sequence 3 which has a FIFO depth of 1.  If another sequence
  // was used with a deeper FIFO, then the array size must be changed.
  //
  uint32_t pui32ADC0Value[1];

  // trigger adc sampling
  ADCProcessorTrigger(ADC0_BASE, 3);

  // wait for sampling
  while (!ADCIntStatus(ADC0_BASE, 3, false)) {
  }

  // clear interrupt flag
  ADCIntClear(ADC0_BASE, 3);

  // read adc sampling value
  ADCSequenceDataGet(ADC0_BASE, 3, pui32ADC0Value);

  // return adc sampling value
  return pui32ADC0Value[0];
}

/**
 * @brief initialize SysTick
 *
 */
void SysTickInit(void) {
  SysTickIntRegister(SystickHandler);
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

  GPIOInit();         // GPIO initialization
  ADCInit();          // ADC initialization
  SysTickInit();      // SysTick initialization
  IntMasterEnable();  // Enable interrupts to the processor.
}

/**
 * @brief SysTick interrupt handler
 *
 */
void SystickHandler(void) {
  // refresh 7seg and led
  TM1638_RefreshDIGIandLED(digit, pnt, led);
}
