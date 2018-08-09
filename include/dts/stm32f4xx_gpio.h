#pragma once

#define GPIO_Pin_0                 0x0001  /* Pin 0 selected */
#define GPIO_Pin_1                 0x0002  /* Pin 1 selected */
#define GPIO_Pin_2                 0x0004  /* Pin 2 selected */
#define GPIO_Pin_3                 0x0008  /* Pin 3 selected */
#define GPIO_Pin_4                 0x0010  /* Pin 4 selected */
#define GPIO_Pin_5                 0x0020  /* Pin 5 selected */
#define GPIO_Pin_6                 0x0040  /* Pin 6 selected */
#define GPIO_Pin_7                 0x0080  /* Pin 7 selected */
#define GPIO_Pin_8                 0x0100  /* Pin 8 selected */
#define GPIO_Pin_9                 0x0200  /* Pin 9 selected */
#define GPIO_Pin_10                0x0400  /* Pin 10 selected */
#define GPIO_Pin_11                0x0800  /* Pin 11 selected */
#define GPIO_Pin_12                0x1000  /* Pin 12 selected */
#define GPIO_Pin_13                0x2000  /* Pin 13 selected */
#define GPIO_Pin_14                0x4000  /* Pin 14 selected */
#define GPIO_Pin_15                0x8000  /* Pin 15 selected */
#define GPIO_Pin_All               0xFFFF  /* All pins selected */

/**
 * Below is mode specified for all stm32f4 gpio drivers.
 * This is then decoded by the driver and converted into CMSIS values
 */
#define GPIO_Mode_IN	0
#define GPIO_Mode_OUT   (0x01 << 4)
#define GPIO_Mode_AF    (0x02 << 4)
#define GPIO_Mode_AN    (0x03 << 4)

#define GPIO_OType_PP	(0x00 << 6)
#define GPIO_OType_OD	(0x01 << 6)

#define GPIO_Speed_2MHz    (0x00 << 7)
#define GPIO_Speed_25MHz   (0x01 << 7)
#define GPIO_Speed_50MHz   (0x02 << 7)
#define GPIO_Speed_100MHz  (0x03 << 7)

#define GPIO_PuPd_NOPULL  (0x00 << 9)
#define GPIO_PuPd_UP      (0x01 << 9)
#define GPIO_PuPd_DOWN    (0x02 << 9)

/** 
  * @brief   AF 0 selection  
  */ 
#define GPIO_AF_RTC_50Hz      (GPIO_Mode_AF | 0x00)  /* RTC_50Hz Alternate Function mapping */
#define GPIO_AF_MCO           (GPIO_Mode_AF | 0x00)  /* MCO (MCO1 and MCO2) Alternate Function mapping */
#define GPIO_AF_TAMPER        (GPIO_Mode_AF | 0x00)  /* TAMPER (TAMPER_1 and TAMPER_2) Alternate Function mapping */
#define GPIO_AF_SWJ           (GPIO_Mode_AF | 0x00)  /* SWJ (SWD and JTAG) Alternate Function mapping */
#define GPIO_AF_TRACE         (GPIO_Mode_AF | 0x00)  /* TRACE Alternate Function mapping */

/** 
  * @brief   AF 1 selection  
  */ 
#define GPIO_AF_TIM1          (GPIO_Mode_AF | 0x01)  /* TIM1 Alternate Function mapping */
#define GPIO_AF_TIM2          (GPIO_Mode_AF | 0x01)  /* TIM2 Alternate Function mapping */

/** 
  * @brief   AF 2 selection  
  */ 
#define GPIO_AF_TIM3          (GPIO_Mode_AF | 0x02)  /* TIM3 Alternate Function mapping */
#define GPIO_AF_TIM4          (GPIO_Mode_AF | 0x02)  /* TIM4 Alternate Function mapping */
#define GPIO_AF_TIM5          (GPIO_Mode_AF | 0x02)  /* TIM5 Alternate Function mapping */

/** 
  * @brief   AF 3 selection  
  */ 
#define GPIO_AF_TIM8          (GPIO_Mode_AF | 0x03)  /* TIM8 Alternate Function mapping */
#define GPIO_AF_TIM9          (GPIO_Mode_AF | 0x03)  /* TIM9 Alternate Function mapping */
#define GPIO_AF_TIM10         (GPIO_Mode_AF | 0x03)  /* TIM10 Alternate Function mapping */
#define GPIO_AF_TIM11         (GPIO_Mode_AF | 0x03)  /* TIM11 Alternate Function mapping */

/** 
  * @brief   AF 4 selection  
  */ 
#define GPIO_AF_I2C1          (GPIO_Mode_AF | 0x04)  /* I2C1 Alternate Function mapping */
#define GPIO_AF_I2C2          (GPIO_Mode_AF | 0x04)  /* I2C2 Alternate Function mapping */
#define GPIO_AF_I2C3          (GPIO_Mode_AF | 0x04)  /* I2C3 Alternate Function mapping */

/** 
  * @brief   AF 5 selection  
  */ 
#define GPIO_AF_SPI1          (GPIO_Mode_AF | 0x05)  /* SPI1 Alternate Function mapping      */
#define GPIO_AF_SPI2          (GPIO_Mode_AF | 0x05)  /* SPI2/I2S2 Alternate Function mapping */
#define GPIO_AF_SPI4          (GPIO_Mode_AF | 0x05)  /* SPI4 Alternate Function mapping      */
#define GPIO_AF_SPI5          (GPIO_Mode_AF | 0x05)  /* SPI5 Alternate Function mapping      */
#define GPIO_AF_SPI6          (GPIO_Mode_AF | 0x05)  /* SPI6 Alternate Function mapping      */

/** 
  * @brief   AF 6 selection  
  */ 
#define GPIO_AF_SPI3          (GPIO_Mode_AF | 0x06)  /* SPI3/I2S3 Alternate Function mapping */

#define GPIO_AF_SAI1          (GPIO_Mode_AF | 0x06)  /* SAI1 Alternate Function mapping      */

/** 
  * @brief   AF 7 selection  
  */ 
#define GPIO_AF_USART1        (GPIO_Mode_AF | 0x0007)  /* USART1 Alternate Function mapping  */
#define GPIO_AF_USART2        (GPIO_Mode_AF | 0x0007)  /* USART2 Alternate Function mapping  */
#define GPIO_AF_USART3        (GPIO_Mode_AF | 0x0007)  /* USART3 Alternate Function mapping  */
#define GPIO_AF_I2S3ext       (GPIO_Mode_AF | 0x0007)  /* I2S3ext Alternate Function mapping */

/** 
  * @brief   AF 8 selection  
  */ 
#define GPIO_AF_UART4         (GPIO_Mode_AF | 0x08)  /* UART4 Alternate Function mapping  */
#define GPIO_AF_UART5         (GPIO_Mode_AF | 0x08)  /* UART5 Alternate Function mapping  */
#define GPIO_AF_USART6        (GPIO_Mode_AF | 0x08)  /* USART6 Alternate Function mapping */
#define GPIO_AF_UART7         (GPIO_Mode_AF | 0x08)  /* UART7 Alternate Function mapping  */
#define GPIO_AF_UART8         (GPIO_Mode_AF | 0x08)  /* UART8 Alternate Function mapping  */

/** 
  * @brief   AF 9 selection 
  */ 
#define GPIO_AF_CAN1          (GPIO_Mode_AF | 0x09)  /* CAN1 Alternate Function mapping  */
#define GPIO_AF_CAN2          (GPIO_Mode_AF | 0x09)  /* CAN2 Alternate Function mapping  */
#define GPIO_AF_TIM12         (GPIO_Mode_AF | 0x09)  /* TIM12 Alternate Function mapping */
#define GPIO_AF_TIM13         (GPIO_Mode_AF | 0x09)  /* TIM13 Alternate Function mapping */
#define GPIO_AF_TIM14         (GPIO_Mode_AF | 0x09)  /* TIM14 Alternate Function mapping */

#define GPIO_AF9_I2C2          (GPIO_Mode_AF | 0x09)  /* I2C2 Alternate Function mapping (Only for STM32F401xx Devices) */
#define GPIO_AF9_I2C3          (GPIO_Mode_AF | 0x09)  /* I2C3 Alternate Function mapping (Only for STM32F401xx Devices) */

/** 
  * @brief   AF 10 selection  
  */ 
#define GPIO_AF_OTG_FS         (GPIO_Mode_AF | 0xA)  /* OTG_FS Alternate Function mapping */
#define GPIO_AF_OTG_HS         (GPIO_Mode_AF | 0xA)  /* OTG_HS Alternate Function mapping */

/** 
  * @brief   AF 11 selection  
  */ 
#define GPIO_AF_ETH             (GPIO_Mode_AF | 0x0B)  /* ETHERNET Alternate Function mapping */

/** 
  * @brief   AF 12 selection  
  */ 
#if defined (STM32F40_41xxx)
#define GPIO_AF_FSMC             (GPIO_Mode_AF | 0xC)  /* FSMC Alternate Function mapping                     */
#endif /* STM32F40_41xxx */

#if defined (STM32F427_437xx) || defined (STM32F429_439xx)
#define GPIO_AF_FMC              (GPIO_Mode_AF | 0xC)  /* FMC Alternate Function mapping                      */
#endif /* STM32F427_437xx ||  STM32F429_439xx */

#define GPIO_AF_OTG_HS_FS        (GPIO_Mode_AF | 0xC)  /* OTG HS configured in FS, Alternate Function mapping */
#define GPIO_AF_SDIO             (GPIO_Mode_AF | 0xC)  /* SDIO Alternate Function mapping                     */

/** 
  * @brief   AF 13 selection  
  */ 
#define GPIO_AF_DCMI          (GPIO_Mode_AF | 0x0D)  /* DCMI Alternate Function mapping */

/** 
  * @brief   AF 14 selection  
  */

#define GPIO_AF_LTDC          (GPIO_Mode_AF | 0x0E)  /* LCD-TFT Alternate Function mapping */

/** 
  * @brief   AF 15 selection  
  */ 
#define GPIO_AF_EVENTOUT      (GPIO_Mode_AF | 0x0F)  /* EVENTOUT Alternate Function mapping */

