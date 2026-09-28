
/*

STM32F051 external interrupts

*/

// windows watchdog interrupt
wwdg = 0,

// PVD and Vddio2 supply comparator interrupt
pvd_vddio2 = 1,

// RTC interrupts
rtc = 2,

// flash global interrupt
flash = 3,

// RCC and CRS global interrupts
rrc_crs = 4,

// EXTI line[1:0] interrupts
exti0_1 = 5,

// EXTI line[3:2] interrupts
exti2_3 = 6,

// EXIT line[15:4] interrupts
exti4_15 = 7,

// TSC touch sensing interrupts
tsc = 8,

// DMA channel 1 interrupt
dma_ch1 = 9,

// DMA channel 2 and 3 interrupts
// DMA2 channel 1 and 2 interrupts
dma_ch2_3 = 10,
dma2_ch1_2 = 10,

// DMA channel 4, 5, 6 and 7 interrupts
// DMA2 channel 3, 4 and 5 interrupts
dma_ch4_5_6_7 = 11,
dma2_ch3_4_5 = 11,

// ADC and COMP interrupts (ADC interrupt combined with EXTI lines 21 and 22)
adc_comp = 12,

// TIM1 break, update, trigger and commutation interrupt
tim_brk_up_trg_com = 13,

// TIM1 capture compare interrupt
tim1_cc = 14,

// TIM2 global interrupt
tim2 = 15,

// TIM3 global interrupt
tim3 = 16,

// TIM6 global interrupt and DAC underrun interrupt
tim6_dac = 17,

// TIM7 global interrupt
tim7 = 18,

// TIM14 global interrupt
tim14 = 19,

// TIM15 global interrupt
tim15 = 20,

// TIM16 global interrupt
tim16 = 21,

// TIM17 global interrupt
tim17 = 22,

// I2C1 global interrupt (combined with EXTI line 23)
i2c1 = 23,

// I2C2 global interrupt
i2c2 = 24,

// SPI1 global interrupt
spi1 = 25,

// SPI2 global interrupt
spi2 = 26,

// USART1 global interrupt (combined with EXTI line 25)
usart1 = 27,

// USART2 global interrupt (combined with EXTI line 26)
usart2 = 28,

// CEC and CAN global interrupts (combined with EXTI line 27)
cec_can = 30,

// USB global interrupt (combined with EXTI line 18)
usb = 31,

// maximum number of (external) interrupts in the system
max_interrupts = 32