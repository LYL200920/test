
/*

GD32F303RCT6A external interrupts
  - CORTEX-M4
  - Non-connectivity devices
  - HD (High-Density) device

Note:
    1. Timer8 and Timer13 (IRQ24、IRQ25、IRQ26、IRQ43、IRQ44、IRQ45)
       are not available
    2. USB and CAN (IRQ19, IRQ20) function
       cann't be used at the same time

*/

// Window watchdog interrupt
wwdg = 0,

// LVD from EXTI interrupt
lvd = 1,

// Tamper interrupt
tamper = 2,

// RTC global interrupt
rtc = 3,

// FMC global interrupt
fmc = 4,

// RCU and CTC interrupt
rcu_ctc = 5,

// EXTI line 0 interrupt
exti0 = 6,

// EXTI line 1 interrupt
exti1 = 7,

// EXTI line 2 interrupt
exti2 = 8,

// EXTI line 3 interrupt
exti3 = 9,

// EXTI line 4 interrupt
exti4 = 10,

// DMA 0 channel 0 interrupt
dma0_ch0 = 11,

// DMA 0 channel 1 interrupt
dma0_ch1 = 12,

// DMA 0 channel 2 interrupt
dma0_ch2 = 13,

// DMA 0 channel 3 interrupt
dma0_ch3 = 14,

// DMA 0 channel 4 interrupt
dma0_ch4 = 15,

// DMA 0 channel 5 interrupt
dma0_ch5 = 16,

// DMA 0 channel 6 interrupt
dma0_ch6 = 17,

// ADC 0 and ADC 1 global interrupts
adc0_adc1 = 18,

// USBD high priority
// OR
// CAN0 TX interrupts
usbd_high_pri_can0_tx = 19,

// USBD low priority
// OR
// CAN0 RX0 interrupts
usbd_low_pri_can0_rx0 = 20,

// CAN0 RX1 interrupts
can0_rx1 = 21,

// CAN0 EWMC interrupts
can0_swmc = 22,

// EXTI line[9:5] interrupts
exti9_5 = 23,

// Timer 0 break interrupt
// and
// Timer 8 global interrupt
tim0_brk_tim8 = 24,

// Timer 0 update interrupt
// and
// Timer 9 global interrupt
tim0_updt_tim9 = 25,

// Timer 0 trigger and channel commutation interrupt
// Timer 10 global interrupt
tim0_trg_ch_comm_tim10 = 26,

// Timer 0 channel capture compare interrupt
tim0_ch_capt_comp = 27,

// Timer 1 global interrupt
tim1 = 28,

// Timer 2 global interrupt
tim2 = 29,

// Timer 3 global interrupt
tim3 = 30,

// I2C 0 event interrupt
i2c0_event = 31,

// I2C 0 error interrupt
i2c0_error = 32,

// I2C 1 event interrupt
i2c1_event = 33,

// I2C 1 error interrupt
i2c1_error = 34,

// SPI 0 global interrupt
spi0 = 35,

// SPI 1 global interrupt
spi1 = 36,

// USART 0 global interrupt
usart0 = 37,

// USART 1 global interrupt
usart1 = 38,

// USART 2 global interrupt
usart2 = 39,

// EXTI line[15:10] interrupts
exti15_10 = 40,

// RTC alarm from EXTI interrupt
rtc_alarm = 41,

// USBFS wakeup interrupt
usbfs_wakeup = 42,

// Timer 7 break
// Timer 11 global interrupt
tim7_brk_tim11 = 43,

// Timer 7 update
// Timer 12 global interrupt
tim7_up_tim12 = 44,

// Timer 7 trigger and channel commutation
// Timer 13 global interrupt
tim7_trg_ch_comm_tim13 = 45,

// Timer 7 channel capture compare interrupt
tim7_ch_capt_comp = 46,

// ADC 2 global interrupt
adc2 = 47,

// EXMC global interrupt
exmc = 48,

// SDIO global interrupt
sdio = 49,

// Timer 4 global interrupt
tim4 = 50,

// SPI2 global interrupt
spi2 = 51,

// UART 3 global interrupt
uart3 = 52,

// UART 4 global interrupt
uart4 = 53,

// Timer 5 global interrupt
tim5 = 54,

// Timer 6 global interrupt
tim6 = 55,

// DMA 1 channel 0 interrupt
dma1_ch0 = 56,

// DMA 1 channel 1 interrupt
dma1_ch1 = 57,

// DMA 1 channel 2 interrupt
dma1_ch2 = 58,

// DMA 1 channel 3
// DMA 1 channel 4 interrupt
dma1_ch3_4 = 59,

// -----------------------------------------------------
// IRQ 60-67:
//   - "Non-connectivity devices": reserved.
//   - "Connectivity devices"    : special definitions

// DMA 1 channel 4 global interrupt
dma1_ch4 = 60,

// ENET global interrupt
enet = 61,

// ENET wakeup from EXTI interrupt
enet_wake = 62,

// CAN1 TX interrupt
can1_tx = 63,

// CAN1 RX0 interrupt
can1_rx0 = 64,

// CAN1 RX1 interrupt
can1_rx1 = 65,

// CAN1 EWMC interrupt
can1_ewmc = 66,

// USBFS global interrupt
usbfs = 67,

max_interrupts = 68





