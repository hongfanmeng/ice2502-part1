// //*****************************************************************************
// //
// // Copyright: 2020-2021, 上海交通大学工程实践与科技创新III-A教学组
// // File name: adc_demo.c
// // Description:
// //    1.该示例展示如何利用AIN2/PE1端口实现单端输入单次ADC采样,采样频率25Hz；
// //    2.左侧四个数码管显示ADC采样值[0-4095]；
// //    3.右侧三个数码管显示电压值[0.00-3.30V]；
// //    4.注意：输入电压值范围必须为[0-3.3V]，否则会烧坏端口。
// // Author:	上海交通大学工程实践与科技创新III-A教学组
// // Version: 1.0.0.20200924
// // Date：2020-09-24
// // History：
// //
// //*****************************************************************************

// //*****************************************************************************
// //
// // 头文件
// //
// //*****************************************************************************
// #include <stdint.h>
// #include <stdbool.h>
// #include "inc/hw_memmap.h"       // 基址宏定义
// #include "inc/hw_types.h"        // 数据类型宏定义，寄存器访问函数
// #include "driverlib/debug.h"     // 调试用
// #include "driverlib/gpio.h"      // 通用IO口宏定义
// #include "driverlib/pin_map.h"   // TM4C系列MCU外围设备管脚宏定义
// #include "driverlib/sysctl.h"    // 系统控制定义
// #include "driverlib/systick.h"   // SysTick Driver 原型
// #include "driverlib/interrupt.h" // NVIC Interrupt Controller Driver 原型
// #include "driverlib/adc.h"       // 与ADC有关的定义

// #include "tm1638.h" // 与控制TM1638芯片有关的函数

// //*****************************************************************************
// //
// // 宏定义
// //
// //*****************************************************************************
// #define SYSTICK_FREQUENCY 50 // SysTick频率为50Hz，即循环定时周期20ms

// #define V_T40ms 2  // 40ms软件定时器溢出值，2个20ms
// #define V_T100ms 5 // 0.1s软件定时器溢出值，5个20ms
// //*****************************************************************************
// //
// // 函数原型声明
// //
// //*****************************************************************************
// void GPIOInit(void);       // GPIO初始化
// void ADCInit(void);        // ADC初始化
// void SysTickInit(void);    // 设置SysTick中断
// void DevicesInit(void);    // MCU器件初始化，注：会调用上述函数
// uint32_t ADC_Sample(void); // 获取ADC采样值
// //*****************************************************************************
// //
// // 变量定义
// //
// //*****************************************************************************

// // 软件定时器计数
// uint8_t clock40ms = 0;
// uint8_t clock100ms = 0;

// // 软件定时器溢出标志
// uint8_t clock40ms_flag = 0;
// uint8_t clock100ms_flag = 0;

// // 8位数码管显示的数字或字母符号
// // 注：板上数码位从左到右序号排列为4、5、6、7、0、1、2、3
// uint8_t digit[8] = {' ', ' ', ' ', ' ', ' ', ' ', ' ', ' '};

// // 8位小数点 1亮  0灭
// // 注：板上数码位小数点从左到右序号排列为4、5、6、7、0、1、2、3
// uint8_t pnt = 0x2;

// // 8个LED指示灯状态，0灭，1亮
// // 注：板上指示灯从左到右序号排列为7、6、5、4、3、2、1、0
// //     对应元件LED8、LED7、LED6、LED5、LED4、LED3、LED2、LED1
// uint8_t led[] = {0, 0, 0, 0, 0, 0, 0, 0};

// // 系统时钟频率
// uint32_t ui32SysClock;

// // AIN2(PE1)  ADC采样值[0-4095]
// uint32_t ui32ADC0Value;

// // AIN2电压值(单位为0.01V) [0.00-3.30]
// uint32_t ui32ADC0Voltage;

// //*****************************************************************************
// //
// // 主程序
// //
// //*****************************************************************************
// int main(void)
// {
//     uint8_t temp, i;

//     DevicesInit(); //  MCU器件初始化

//     while (clock100ms < 3)
//         ;          // 延时>60ms,等待TM1638上电完成
//     TM1638_Init(); // 初始化TM1638

//     while (1)
//     {

//         if (clock40ms_flag == 1) // 检查40ms秒定时是否到
//         {
//             clock40ms = 0;

//             ui32ADC0Value = ADC_Sample();

//             digit[4] = ui32ADC0Value / 1000;     // 显示ADC采样值千位数
//             digit[5] = ui32ADC0Value / 100 % 10; // 显示ADC采样值百位数
//             digit[6] = ui32ADC0Value / 10 % 10;  // 显示ADC采样值十位数
//             digit[7] = ui32ADC0Value % 10;       // 显示ADC采样值个位数

//             ui32ADC0Voltage = ui32ADC0Value * 330 / 4095;

//             digit[1] = (ui32ADC0Voltage / 100) % 10; // 显示电压值个位数
//             digit[2] = (ui32ADC0Voltage / 10) % 10;  // 显示电压值十分位数
//             digit[3] = ui32ADC0Voltage % 10;         // 显示电压值百分位数
//         }
//     }
// }

// //*****************************************************************************
// //
// // 函数原型：void GPIOInit(void)
// // 函数功能：GPIO初始化。使能PortK，设置PK4,PK5为输出；使能PortM，设置PM0为输出。
// //          （PK4连接TM1638的STB，PK5连接TM1638的DIO，PM0连接TM1638的CLK）
// // 函数参数：无
// // 函数返回值：无
// //
// //*****************************************************************************
// void GPIOInit(void)
// {
//     //配置TM1638芯片管脚
//     SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK); // 使能端口 K
//     while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOK))
//     {
//     }; // 等待端口 K准备完毕

//     SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM); // 使能端口 M
//     while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOM))
//     {
//     }; // 等待端口 M准备完毕

//     // 设置端口 K的第4,5位（PK4,PK5）为输出引脚		PK4-STB  PK5-DIO
//     GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_4 | GPIO_PIN_5);
//     // 设置端口 M的第0位（PM0）为输出引脚   PM0-CLK
//     GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_0);
// }

// //*****************************************************************************
// //
// // 函数原型：void ADCInit(void)
// // 函数功能：ADC0初始化。
// //           选择AIN2/PE1作为ADC采样输入端口，采用单端口输入单次采样方式
// // 函数参数：无
// // 函数返回值：无
// //
// //*****************************************************************************
// void ADCInit(void)
// {
//     // 使能ADC0模块
//     SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

//     // 使用AIN2/PE1端口作为ADC输入，使能端口E1
//     SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

//     GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_1);

//     // 单次采样(sample sequence 3方式)
//     ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);

//     //
//     // Configure step 0 on sequence 3.  Sample channel 0 (ADC_CTL_CH0) in
//     // single-ended mode (default) and configure the interrupt flag
//     // (ADC_CTL_IE) to be set when the sample is done.  Tell the ADC logic
//     // that this is the last conversion on sequence 3 (ADC_CTL_END).  Sequence
//     // 3 has only one programmable step.  Sequence 1 and 2 have 4 steps, and
//     // sequence 0 has 8 programmable steps.  Since we are only doing a single
//     // conversion using sequence 3 we will only configure step 0.  For more
//     // information on the ADC sequences and steps, reference the datasheet.
//     //
//     ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH2 | ADC_CTL_IE | ADC_CTL_END);

//     // 使能单次采样方式(sample sequence 3)
//     ADCSequenceEnable(ADC0_BASE, 3);

//     // 在采样前，必须清除中断状态标志
//     ADCIntClear(ADC0_BASE, 3);
// }

// //*****************************************************************************
// //
// // 函数原型：uint32_t ADC_Sample(void)
// // 函数功能：获取ADC采样值。
// // 函数参数：无
// // 函数返回值：ADC采样值[0-4095]
// //
// //*****************************************************************************
// uint32_t ADC_Sample(void)
// {

//     //
//     // This array is used for storing the data read from the ADC FIFO. It
//     // must be as large as the FIFO for the sequencer in use.  This example
//     // uses sequence 3 which has a FIFO depth of 1.  If another sequence
//     // was used with a deeper FIFO, then the array size must be changed.
//     //
//     uint32_t pui32ADC0Value[1];

//     // 触发ADC采样
//     ADCProcessorTrigger(ADC0_BASE, 3);

//     // 等待采样转换完成
//     while (!ADCIntStatus(ADC0_BASE, 3, false))
//     {
//     }

//     // 清除ADC中断标志
//     ADCIntClear(ADC0_BASE, 3);

//     // 读取ADC采样值
//     ADCSequenceDataGet(ADC0_BASE, 3, pui32ADC0Value);

//     return pui32ADC0Value[0];
// }

// //*****************************************************************************
// //
// // 函数原型：SysTickInit(void)
// // 函数功能：设置SysTick中断
// // 函数参数：无
// // 函数返回值：无
// //
// //*****************************************************************************
// void SysTickInit(void)
// {
//     SysTickPeriodSet(ui32SysClock / SYSTICK_FREQUENCY); // 设置心跳节拍,定时周期20ms
//     SysTickEnable();                                    // SysTick使能
//     SysTickIntEnable();                                 // SysTick中断允许
// }

// //*****************************************************************************
// //
// // 函数原型：DevicesInit(void)
// // 函数功能：CU器件初始化，包括系统时钟设置、GPIO初始化和SysTick中断设置
// // 函数参数：无
// // 函数返回值：无
// //
// //*****************************************************************************
// void DevicesInit(void)
// {
//     // 使用外部25MHz主时钟源，经过PLL，然后分频为20MHz
//     ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
//                                        SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480),
//                                       20000000);

//     GPIOInit();        // GPIO初始化
//     ADCInit();         // ADC初始化
//     SysTickInit();     // 设置SysTick中断
//     IntMasterEnable(); // 总中断允许
// }

// //*****************************************************************************
// //
// // 函数原型：void SysTick_Handler(void)
// // 函数功能：SysTick中断服务程序
// // 函数参数：无
// // 函数返回值：无
// //
// //*****************************************************************************
// void SysTick_Handler(void) // 定时周期为20ms
// {
//     //
//     clock40ms_flag = 1;

//     // 0.1秒钟软定时器计数
//     if (++clock100ms >= V_T40ms)
//     {
//         clock40ms_flag = 1; // 当0.1秒到时，溢出标志置1
//         clock40ms = 0;
//     }

//     // 0.1秒钟软定时器计数
//     if (++clock100ms >= V_T100ms)
//     {
//         clock100ms_flag = 1; // 当0.1秒到时，溢出标志置1
//         clock100ms = 0;
//     }

//     // 刷新全部数码管和LED
//     TM1638_RefreshDIGIandLED(digit, pnt, led);
// }
