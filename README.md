# cmsis_stm32f103c6t6_mylibs
Writing my own HAL for stm32f103c6t6 bluepill microcontroller. This project is focused on learning M3 Cortex architecture and STM32 microcontroller line. Probably will not be finished due to amount of options for every peripheral and the topic being already well developed by vendors and other develeopers.

Every driver is written from scratch by using only CMSIS library and datasheet. 

Basically a testzone for bluepill and my C programming.

What i learned from it so far :  
- CAN bus basics - configuration/filter banks/modes  
- ADC operation  
- RCC clocks - configuration of HSE/HSI/PLL to achive desired frequency  
- GPIO - how different it is from STM32F401 line on M4 cortex  
- SPI - configuration and send  
- USART - configuration/irq handling  
