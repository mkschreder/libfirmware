#pragma once

/** @addtogroup Peripheral_memory_map
  * @{
  */
#define FLASH            (0x08000000) /*!< FLASH(up to 1 MB) base address in the alias region                         */
#define CCMDATARAM       (0x10000000) /*!< CCM(core coupled memory) data RAM(64 KB) base address in the alias region  */
#define SRAM1            (0x20000000) /*!< SRAM1(112 KB) base address in the alias region                             */
#define SRAM2            (0x2001C000) /*!< SRAM2(16 KB) base address in the alias region                              */
#define SRAM3            (0x20020000) /*!< SRAM3(64 KB) base address in the alias region                              */
#define PERIPH           (0x40000000) /*!< Peripheral base address in the alias region                                */
#define BKPSRAM          (0x40024000) /*!< Backup SRAM(4 KB) base address in the alias region                         */

#if defined (STM32F40_41xxx)
#define FSMC_R           (0xA0000000) /*!< FSMC registers base address                                                */
#endif /* STM32F40_41xxx */

#if defined (STM32F427_437xx) || defined (STM32F429_439xx)
#define FMC_R            (0xA0000000) /*!< FMC registers base address                                                 */
#endif /* STM32F427_437xx ||  STM32F429_439xx */

#define CCMDATARAM_BB    (0x12000000) /*!< CCM(core coupled memory) data RAM(64 KB) base address in the bit-band region  */
#define SRAM1_BB         (0x22000000) /*!< SRAM1(112 KB) base address in the bit-band region                             */
#define SRAM2_BB         (0x2201C000) /*!< SRAM2(16 KB) base address in the bit-band region                              */
#define SRAM3_BB         (0x22400000) /*!< SRAM3(64 KB) base address in the bit-band region                              */
#define PERIPH_BB        (0x42000000) /*!< Peripheral base address in the bit-band region                                */
#define BKPSRAM_BB       (0x42024000) /*!< Backup SRAM(4 KB) base address in the bit-band region                         */

/* Legacy defines */
#define SRAM             SRAM1
#define SRAM_BB          SRAM1_BB


/*!< Peripheral memory map */
#define APB1PERIPH       PERIPH
#define APB2PERIPH       (PERIPH + 0x00010000)
#define AHB1PERIPH       (PERIPH + 0x00020000)
#define AHB2PERIPH       (PERIPH + 0x10000000)

/*!< APB1 peripherals */
#define TIM2             (APB1PERIPH + 0x0000)
#define TIM3             (APB1PERIPH + 0x0400)
#define TIM4             (APB1PERIPH + 0x0800)
#define TIM5             (APB1PERIPH + 0x0C00)
#define TIM6             (APB1PERIPH + 0x1000)
#define TIM7             (APB1PERIPH + 0x1400)
#define TIM12            (APB1PERIPH + 0x1800)
#define TIM13            (APB1PERIPH + 0x1C00)
#define TIM14            (APB1PERIPH + 0x2000)
#define RTC              (APB1PERIPH + 0x2800)
#define WWDG             (APB1PERIPH + 0x2C00)
#define IWDG             (APB1PERIPH + 0x3000)
#define I2S2ext          (APB1PERIPH + 0x3400)
#define SPI2             (APB1PERIPH + 0x3800)
#define SPI3             (APB1PERIPH + 0x3C00)
#define I2S3ext          (APB1PERIPH + 0x4000)
#define USART2           (APB1PERIPH + 0x4400)
#define USART3           (APB1PERIPH + 0x4800)
#define UART4            (APB1PERIPH + 0x4C00)
#define UART5            (APB1PERIPH + 0x5000)
#define I2C1             (APB1PERIPH + 0x5400)
#define I2C2             (APB1PERIPH + 0x5800)
#define I2C3             (APB1PERIPH + 0x5C00)
#define CAN1             (APB1PERIPH + 0x6400)
#define CAN2             (APB1PERIPH + 0x6800)
#define PWR              (APB1PERIPH + 0x7000)
#define DAC              (APB1PERIPH + 0x7400)
#define UART7            (APB1PERIPH + 0x7800)
#define UART8            (APB1PERIPH + 0x7C00)

/*!< APB2 peripherals */
#define TIM1             (APB2PERIPH + 0x0000)
#define TIM8             (APB2PERIPH + 0x0400)
#define USART1           (APB2PERIPH + 0x1000)
#define USART6           (APB2PERIPH + 0x1400)
#define ADC1             (APB2PERIPH + 0x2000)
#define ADC2             (APB2PERIPH + 0x2100)
#define ADC3             (APB2PERIPH + 0x2200)
#define ADC              (APB2PERIPH + 0x2300)
#define SDIO             (APB2PERIPH + 0x2C00)
#define SPI1             (APB2PERIPH + 0x3000)
#define SPI4             (APB2PERIPH + 0x3400)
#define SYSCFG           (APB2PERIPH + 0x3800)
#define EXTI             (APB2PERIPH + 0x3C00)
#define TIM9             (APB2PERIPH + 0x4000)
#define TIM10            (APB2PERIPH + 0x4400)
#define TIM11            (APB2PERIPH + 0x4800)
#define SPI5             (APB2PERIPH + 0x5000)
#define SPI6             (APB2PERIPH + 0x5400)
#define SAI1             (APB2PERIPH + 0x5800)
#define SAI1_Block_A     (SAI1 + 0x004)
#define SAI1_Block_B     (SAI1 + 0x024)
#define LTDC             (APB2PERIPH + 0x6800)
#define LTDC_Layer1      (LTDC + 0x84)
#define LTDC_Layer2      (LTDC + 0x104) 

/*!< AHB1 peripherals */
#define GPIOA            (AHB1PERIPH + 0x0000)
#define GPIOB            (AHB1PERIPH + 0x0400)
#define GPIOC            (AHB1PERIPH + 0x0800)
#define GPIOD            (AHB1PERIPH + 0x0C00)
#define GPIOE            (AHB1PERIPH + 0x1000)
#define GPIOF            (AHB1PERIPH + 0x1400)
#define GPIOG            (AHB1PERIPH + 0x1800)
#define GPIOH            (AHB1PERIPH + 0x1C00)
#define GPIOI            (AHB1PERIPH + 0x2000)
#define GPIOJ            (AHB1PERIPH + 0x2400)
#define GPIOK            (AHB1PERIPH + 0x2800)
#define CRC              (AHB1PERIPH + 0x3000)
#define RCC              (AHB1PERIPH + 0x3800)
#define FLASH_R          (AHB1PERIPH + 0x3C00)
#define DMA1             (AHB1PERIPH + 0x6000)
#define DMA1_Stream0     (DMA1 + 0x010)
#define DMA1_Stream1     (DMA1 + 0x028)
#define DMA1_Stream2     (DMA1 + 0x040)
#define DMA1_Stream3     (DMA1 + 0x058)
#define DMA1_Stream4     (DMA1 + 0x070)
#define DMA1_Stream5     (DMA1 + 0x088)
#define DMA1_Stream6     (DMA1 + 0x0A0)
#define DMA1_Stream7     (DMA1 + 0x0B8)
#define DMA2             (AHB1PERIPH + 0x6400)
#define DMA2_Stream0     (DMA2 + 0x010)
#define DMA2_Stream1     (DMA2 + 0x028)
#define DMA2_Stream2     (DMA2 + 0x040)
#define DMA2_Stream3     (DMA2 + 0x058)
#define DMA2_Stream4     (DMA2 + 0x070)
#define DMA2_Stream5     (DMA2 + 0x088)
#define DMA2_Stream6     (DMA2 + 0x0A0)
#define DMA2_Stream7     (DMA2 + 0x0B8)
#define ETH              (AHB1PERIPH + 0x8000)
#define ETH_MAC          (ETH)
#define ETH_MMC          (ETH + 0x0100)
#define ETH_PTP          (ETH + 0x0700)
#define ETH_DMA          (ETH + 0x1000)
#define DMA2D            (AHB1PERIPH + 0xB000)

/*!< AHB2 peripherals */
#define DCMI             (AHB2PERIPH + 0x50000)
#define CRYP             (AHB2PERIPH + 0x60000)
#define HASH             (AHB2PERIPH + 0x60400)
#define HASH_DIGEST      (AHB2PERIPH + 0x60710)
#define RNG              (AHB2PERIPH + 0x60800)

#if defined (STM32F40_41xxx)
/*!< FSMC Bankx registers base address */
#define FSMC_Bank1_R     (FSMC_R + 0x0000)
#define FSMC_Bank1E_R    (FSMC_R + 0x0104)
#define FSMC_Bank2_R     (FSMC_R + 0x0060)
#define FSMC_Bank3_R     (FSMC_R + 0x0080)
#define FSMC_Bank4_R     (FSMC_R + 0x00A0)
#endif /* STM32F40_41xxx */

#if defined (STM32F427_437xx) || defined (STM32F429_439xx)
/*!< FMC Bankx registers base address */
#define FMC_Bank1_R      (FMC_R + 0x0000)
#define FMC_Bank1E_R     (FMC_R + 0x0104)
#define FMC_Bank2_R      (FMC_R + 0x0060)
#define FMC_Bank3_R      (FMC_R + 0x0080)
#define FMC_Bank4_R      (FMC_R + 0x00A0)
#define FMC_Bank5_6_R    (FMC_R + 0x0140)
#endif /* STM32F427_437xx ||  STM32F429_439xx */

/* Debug MCU registers base address */
#define DBGMCU           (0xE0042000)

/******  Cortex-M4 Processor Exceptions Numbers ****************************************************************/
#define  NonMaskableInt_IRQn          -14    /*!< 2 Non Maskable Interrupt                                          */
#define  MemoryManagement_IRQn        -12    /*!< 4 Cortex-M4 Memory Management Interrupt                           */
#define  BusFault_IRQn                -11    /*!< 5 Cortex-M4 Bus Fault Interrupt                                   */
#define  UsageFault_IRQn              -10    /*!< 6 Cortex-M4 Usage Fault Interrupt                                 */
#define  SVCall_IRQn                  -5     /*!< 11 Cortex-M4 SV Call Interrupt                                    */
#define  DebugMonitor_IRQn            -4     /*!< 12 Cortex-M4 Debug Monitor Interrupt                              */
#define  PendSV_IRQn                  -2     /*!< 14 Cortex-M4 Pend SV Interrupt                                    */
#define  SysTick_IRQn                 -1     /*!< 15 Cortex-M4 System Tick Interrupt                                */
/******  STM32 specific Interrupt Numbers **********************************************************************/
#define  WWDG_IRQn                    0      /*!< Window WatchDog Interrupt                                         */
#define  PVD_IRQn                     1      /*!< PVD through EXTI Line detection Interrupt                         */
#define  TAMP_STAMP_IRQn              2      /*!< Tamper and TimeStamp interrupts through the EXTI line             */
#define  RTC_WKUP_IRQn                3      /*!< RTC Wakeup interrupt through the EXTI line                        */
#define  FLASH_IRQn                   4      /*!< FLASH global Interrupt                                            */
#define  RCC_IRQn                     5      /*!< RCC global Interrupt                                              */
#define  EXTI0_IRQn                   6      /*!< EXTI Line0 Interrupt                                              */
#define  EXTI1_IRQn                   7      /*!< EXTI Line1 Interrupt                                              */
#define  EXTI2_IRQn                   8      /*!< EXTI Line2 Interrupt                                              */
#define  EXTI3_IRQn                   9      /*!< EXTI Line3 Interrupt                                              */
#define  EXTI4_IRQn                   10     /*!< EXTI Line4 Interrupt                                              */
#define  DMA1_Stream0_IRQn            11     /*!< DMA1 Stream 0 global Interrupt                                    */
#define  DMA1_Stream1_IRQn            12     /*!< DMA1 Stream 1 global Interrupt                                    */
#define  DMA1_Stream2_IRQn            13     /*!< DMA1 Stream 2 global Interrupt                                    */
#define  DMA1_Stream3_IRQn            14     /*!< DMA1 Stream 3 global Interrupt                                    */
#define  DMA1_Stream4_IRQn            15     /*!< DMA1 Stream 4 global Interrupt                                    */
#define  DMA1_Stream5_IRQn            16     /*!< DMA1 Stream 5 global Interrupt                                    */
#define  DMA1_Stream6_IRQn            17     /*!< DMA1 Stream 6 global Interrupt                                    */
#define  ADC_IRQn                     18     /*!< ADC1, ADC2 and ADC3 global Interrupts                             */

#if defined (STM32F40_41xxx)
#define  CAN1_TX_IRQn                 19     /*!< CAN1 TX Interrupt                                                 */
#define  CAN1_RX0_IRQn                20     /*!< CAN1 RX0 Interrupt                                                */
#define  CAN1_RX1_IRQn                21     /*!< CAN1 RX1 Interrupt                                                */
#define  CAN1_SCE_IRQn                22     /*!< CAN1 SCE Interrupt                                                */
#define  EXTI9_5_IRQn                 23     /*!< External Line[9:5] Interrupts                                     */
#define  TIM1_BRK_TIM9_IRQn           24     /*!< TIM1 Break interrupt and TIM9 global interrupt                    */
#define  TIM1_UP_TIM10_IRQn           25     /*!< TIM1 Update Interrupt and TIM10 global interrupt                  */
#define  TIM1_TRG_COM_TIM11_IRQn      26     /*!< TIM1 Trigger and Commutation Interrupt and TIM11 global interrupt */
#define  TIM1_CC_IRQn                 27     /*!< TIM1 Capture Compare Interrupt                                    */
#define  TIM2_IRQn                    28     /*!< TIM2 global Interrupt                                             */
#define  TIM3_IRQn                    29     /*!< TIM3 global Interrupt                                             */
#define  TIM4_IRQn                    30     /*!< TIM4 global Interrupt                                             */
#define  I2C1_EV_IRQn                 31     /*!< I2C1 Event Interrupt                                              */
#define  I2C1_ER_IRQn                 32     /*!< I2C1 Error Interrupt                                              */
#define  I2C2_EV_IRQn                 33     /*!< I2C2 Event Interrupt                                              */
#define  I2C2_ER_IRQn                 34     /*!< I2C2 Error Interrupt                                              */  
#define  SPI1_IRQn                    35     /*!< SPI1 global Interrupt                                             */
#define  SPI2_IRQn                    36     /*!< SPI2 global Interrupt                                             */
#define  USART1_IRQn                  37     /*!< USART1 global Interrupt                                           */
#define  USART2_IRQn                  38     /*!< USART2 global Interrupt                                           */
#define  USART3_IRQn                  39     /*!< USART3 global Interrupt                                           */
#define  EXTI15_10_IRQn               40     /*!< External Line[15:10] Interrupts                                   */
#define  RTC_Alarm_IRQn               41     /*!< RTC Alarm (A and B) through EXTI Line Interrupt                   */
#define  OTG_FS_WKUP_IRQn             42     /*!< USB OTG FS Wakeup through EXTI line interrupt                     */    
#define  TIM8_BRK_TIM12_IRQn          43     /*!< TIM8 Break Interrupt and TIM12 global interrupt                   */
#define  TIM8_UP_TIM13_IRQn           44     /*!< TIM8 Update Interrupt and TIM13 global interrupt                  */
#define  TIM8_TRG_COM_TIM14_IRQn      45     /*!< TIM8 Trigger and Commutation Interrupt and TIM14 global interrupt */
#define  TIM8_CC_IRQn                 46     /*!< TIM8 Capture Compare Interrupt                                    */
#define  DMA1_Stream7_IRQn            47     /*!< DMA1 Stream7 Interrupt                                            */
#define  FSMC_IRQn                    48     /*!< FSMC global Interrupt                                             */
#define  SDIO_IRQn                    49     /*!< SDIO global Interrupt                                             */
#define  TIM5_IRQn                    50     /*!< TIM5 global Interrupt                                             */
#define  SPI3_IRQn                    51     /*!< SPI3 global Interrupt                                             */
#define  UART4_IRQn                   52     /*!< UART4 global Interrupt                                            */
#define  UART5_IRQn                   53     /*!< UART5 global Interrupt                                            */
#define  TIM6_DAC_IRQn                54     /*!< TIM6 global and DAC1&2 underrun error  interrupts                 */
#define  TIM7_IRQn                    55     /*!< TIM7 global interrupt                                             */
#define  DMA2_Stream0_IRQn            56     /*!< DMA2 Stream 0 global Interrupt                                    */
#define  DMA2_Stream1_IRQn            57     /*!< DMA2 Stream 1 global Interrupt                                    */
#define  DMA2_Stream2_IRQn            58     /*!< DMA2 Stream 2 global Interrupt                                    */
#define  DMA2_Stream3_IRQn            59     /*!< DMA2 Stream 3 global Interrupt                                    */
#define  DMA2_Stream4_IRQn            60     /*!< DMA2 Stream 4 global Interrupt                                    */
#define  ETH_IRQn                     61     /*!< Ethernet global Interrupt                                         */
#define  ETH_WKUP_IRQn                62     /*!< Ethernet Wakeup through EXTI line Interrupt                       */
#define  CAN2_TX_IRQn                 63     /*!< CAN2 TX Interrupt                                                 */
#define  CAN2_RX0_IRQn                64     /*!< CAN2 RX0 Interrupt                                                */
#define  CAN2_RX1_IRQn                65     /*!< CAN2 RX1 Interrupt                                                */
#define  CAN2_SCE_IRQn                66     /*!< CAN2 SCE Interrupt                                                */
#define  OTG_FS_IRQn                  67     /*!< USB OTG FS global Interrupt                                       */
#define  DMA2_Stream5_IRQn            68     /*!< DMA2 Stream 5 global interrupt                                    */
#define  DMA2_Stream6_IRQn            69     /*!< DMA2 Stream 6 global interrupt                                    */
#define  DMA2_Stream7_IRQn            70     /*!< DMA2 Stream 7 global interrupt                                    */
#define  USART6_IRQn                  71     /*!< USART6 global interrupt                                           */
#define  I2C3_EV_IRQn                 72     /*!< I2C3 event interrupt                                              */
#define  I2C3_ER_IRQn                 73     /*!< I2C3 error interrupt                                              */
#define  OTG_HS_EP1_OUT_IRQn          74     /*!< USB OTG HS End Point 1 Out global interrupt                       */
#define  OTG_HS_EP1_IN_IRQn           75     /*!< USB OTG HS End Point 1 In global interrupt                        */
#define  OTG_HS_WKUP_IRQn             76     /*!< USB OTG HS Wakeup through EXTI interrupt                          */
#define  OTG_HS_IRQn                  77     /*!< USB OTG HS global interrupt                                       */
#define  DCMI_IRQn                    78     /*!< DCMI global interrupt                                             */
#define  CRYP_IRQn                    79     /*!< CRYP crypto global interrupt                                      */
#define  HASH_RNG_IRQn                80     /*!< Hash and Rng global interrupt                                     */
#define  FPU_IRQn                     81      /*!< FPU global interrupt                                              */
#endif /* STM32F40_41xxx */

#if defined (STM32F427_437xx)
#define  CAN1_TX_IRQn                 19     /*!< CAN1 TX Interrupt                                                 */
#define  CAN1_RX0_IRQn                20     /*!< CAN1 RX0 Interrupt                                                */
#define  CAN1_RX1_IRQn                21     /*!< CAN1 RX1 Interrupt                                                */
#define  CAN1_SCE_IRQn                22     /*!< CAN1 SCE Interrupt                                                */
#define  EXTI9_5_IRQn                 23     /*!< External Line[9:5] Interrupts                                     */
#define  TIM1_BRK_TIM9_IRQn           24     /*!< TIM1 Break interrupt and TIM9 global interrupt                    */
#define  TIM1_UP_TIM10_IRQn           25     /*!< TIM1 Update Interrupt and TIM10 global interrupt                  */
#define  TIM1_TRG_COM_TIM11_IRQn      26     /*!< TIM1 Trigger and Commutation Interrupt and TIM11 global interrupt */
#define  TIM1_CC_IRQn                 27     /*!< TIM1 Capture Compare Interrupt                                    */
#define  TIM2_IRQn                    28     /*!< TIM2 global Interrupt                                             */
#define  TIM3_IRQn                    29     /*!< TIM3 global Interrupt                                             */
#define  TIM4_IRQn                    30     /*!< TIM4 global Interrupt                                             */
#define  I2C1_EV_IRQn                 31     /*!< I2C1 Event Interrupt                                              */
#define  I2C1_ER_IRQn                 32     /*!< I2C1 Error Interrupt                                              */
#define  I2C2_EV_IRQn                 33     /*!< I2C2 Event Interrupt                                              */
#define  I2C2_ER_IRQn                 34     /*!< I2C2 Error Interrupt                                              */  
#define  SPI1_IRQn                    35     /*!< SPI1 global Interrupt                                             */
#define  SPI2_IRQn                    36     /*!< SPI2 global Interrupt                                             */
#define  USART1_IRQn                  37     /*!< USART1 global Interrupt                                           */
#define  USART2_IRQn                  38     /*!< USART2 global Interrupt                                           */
#define  USART3_IRQn                  39     /*!< USART3 global Interrupt                                           */
#define  EXTI15_10_IRQn               40     /*!< External Line[15:10] Interrupts                                   */
#define  RTC_Alarm_IRQn               41     /*!< RTC Alarm (A and B) through EXTI Line Interrupt                   */
#define  OTG_FS_WKUP_IRQn             42     /*!< USB OTG FS Wakeup through EXTI line interrupt                     */    
#define  TIM8_BRK_TIM12_IRQn          43     /*!< TIM8 Break Interrupt and TIM12 global interrupt                   */
#define  TIM8_UP_TIM13_IRQn           44     /*!< TIM8 Update Interrupt and TIM13 global interrupt                  */
#define  TIM8_TRG_COM_TIM14_IRQn      45     /*!< TIM8 Trigger and Commutation Interrupt and TIM14 global interrupt */
#define  TIM8_CC_IRQn                 46     /*!< TIM8 Capture Compare Interrupt                                    */
#define  DMA1_Stream7_IRQn            47     /*!< DMA1 Stream7 Interrupt                                            */
#define  FMC_IRQn                     48     /*!< FMC global Interrupt                                              */
#define  SDIO_IRQn                    49     /*!< SDIO global Interrupt                                             */
#define  TIM5_IRQn                    50     /*!< TIM5 global Interrupt                                             */
#define  SPI3_IRQn                    51     /*!< SPI3 global Interrupt                                             */
#define  UART4_IRQn                   52     /*!< UART4 global Interrupt                                            */
#define  UART5_IRQn                   53     /*!< UART5 global Interrupt                                            */
#define  TIM6_DAC_IRQn                54     /*!< TIM6 global and DAC1&2 underrun error  interrupts                 */
#define  TIM7_IRQn                    55     /*!< TIM7 global interrupt                                             */
#define  DMA2_Stream0_IRQn            56     /*!< DMA2 Stream 0 global Interrupt                                    */
#define  DMA2_Stream1_IRQn            57     /*!< DMA2 Stream 1 global Interrupt                                    */
#define  DMA2_Stream2_IRQn            58     /*!< DMA2 Stream 2 global Interrupt                                    */
#define  DMA2_Stream3_IRQn            59     /*!< DMA2 Stream 3 global Interrupt                                    */
#define  DMA2_Stream4_IRQn            60     /*!< DMA2 Stream 4 global Interrupt                                    */
#define  ETH_IRQn                     61     /*!< Ethernet global Interrupt                                         */
#define  ETH_WKUP_IRQn                62     /*!< Ethernet Wakeup through EXTI line Interrupt                       */
#define  CAN2_TX_IRQn                 63     /*!< CAN2 TX Interrupt                                                 */
#define  CAN2_RX0_IRQn                64     /*!< CAN2 RX0 Interrupt                                                */
#define  CAN2_RX1_IRQn                65     /*!< CAN2 RX1 Interrupt                                                */
#define  CAN2_SCE_IRQn                66     /*!< CAN2 SCE Interrupt                                                */
#define  OTG_FS_IRQn                  67     /*!< USB OTG FS global Interrupt                                       */
#define  DMA2_Stream5_IRQn            68     /*!< DMA2 Stream 5 global interrupt                                    */
#define  DMA2_Stream6_IRQn            69     /*!< DMA2 Stream 6 global interrupt                                    */
#define  DMA2_Stream7_IRQn            70     /*!< DMA2 Stream 7 global interrupt                                    */
#define  USART6_IRQn                  71     /*!< USART6 global interrupt                                           */
#define  I2C3_EV_IRQn                 72     /*!< I2C3 event interrupt                                              */
#define  I2C3_ER_IRQn                 73     /*!< I2C3 error interrupt                                              */
#define  OTG_HS_EP1_OUT_IRQn          74     /*!< USB OTG HS End Point 1 Out global interrupt                       */
#define  OTG_HS_EP1_IN_IRQn           75     /*!< USB OTG HS End Point 1 In global interrupt                        */
#define  OTG_HS_WKUP_IRQn             76     /*!< USB OTG HS Wakeup through EXTI interrupt                          */
#define  OTG_HS_IRQn                  77     /*!< USB OTG HS global interrupt                                       */
#define  DCMI_IRQn                    78     /*!< DCMI global interrupt                                             */
#define  CRYP_IRQn                    79     /*!< CRYP crypto global interrupt                                      */
#define  HASH_RNG_IRQn                80     /*!< Hash and Rng global interrupt                                     */
#define  FPU_IRQn                     81     /*!< FPU global interrupt                                              */
#define  UART7_IRQn                   82     /*!< UART7 global interrupt                                            */
#define  UART8_IRQn                   83     /*!< UART8 global interrupt                                            */
#define  SPI4_IRQn                    84     /*!< SPI4 global Interrupt                                             */
#define  SPI5_IRQn                    85     /*!< SPI5 global Interrupt                                             */
#define  SPI6_IRQn                    86     /*!< SPI6 global Interrupt                                             */
#define  SAI1_IRQn                    87     /*!< SAI1 global Interrupt                                             */
#define  DMA2D_IRQn                   90      /*!< DMA2D global Interrupt                                            */   
#endif /* STM32F427_437xx */
    
#if defined (STM32F429_439xx)
#define  CAN1_TX_IRQn                 19     /*!< CAN1 TX Interrupt                                                 */
#define  CAN1_RX0_IRQn                20     /*!< CAN1 RX0 Interrupt                                                */
#define  CAN1_RX1_IRQn                21     /*!< CAN1 RX1 Interrupt                                                */
#define  CAN1_SCE_IRQn                22     /*!< CAN1 SCE Interrupt                                                */
#define  EXTI9_5_IRQn                 23     /*!< External Line[9:5] Interrupts                                     */
#define  TIM1_BRK_TIM9_IRQn           24     /*!< TIM1 Break interrupt and TIM9 global interrupt                    */
#define  TIM1_UP_TIM10_IRQn           25     /*!< TIM1 Update Interrupt and TIM10 global interrupt                  */
#define  TIM1_TRG_COM_TIM11_IRQn      26     /*!< TIM1 Trigger and Commutation Interrupt and TIM11 global interrupt */
#define  TIM1_CC_IRQn                 27     /*!< TIM1 Capture Compare Interrupt                                    */
#define  TIM2_IRQn                    28     /*!< TIM2 global Interrupt                                             */
#define  TIM3_IRQn                    29     /*!< TIM3 global Interrupt                                             */
#define  TIM4_IRQn                    30     /*!< TIM4 global Interrupt                                             */
#define  I2C1_EV_IRQn                 31     /*!< I2C1 Event Interrupt                                              */
#define  I2C1_ER_IRQn                 32     /*!< I2C1 Error Interrupt                                              */
#define  I2C2_EV_IRQn                 33     /*!< I2C2 Event Interrupt                                              */
#define  I2C2_ER_IRQn                 34     /*!< I2C2 Error Interrupt                                              */  
#define  SPI1_IRQn                    35     /*!< SPI1 global Interrupt                                             */
#define  SPI2_IRQn                    36     /*!< SPI2 global Interrupt                                             */
#define  USART1_IRQn                  37     /*!< USART1 global Interrupt                                           */
#define  USART2_IRQn                  38     /*!< USART2 global Interrupt                                           */
#define  USART3_IRQn                  39     /*!< USART3 global Interrupt                                           */
#define  EXTI15_10_IRQn               40     /*!< External Line[15:10] Interrupts                                   */
#define  RTC_Alarm_IRQn               41     /*!< RTC Alarm (A and B) through EXTI Line Interrupt                   */
#define  OTG_FS_WKUP_IRQn             42     /*!< USB OTG FS Wakeup through EXTI line interrupt                     */    
#define  TIM8_BRK_TIM12_IRQn          43     /*!< TIM8 Break Interrupt and TIM12 global interrupt                   */
#define  TIM8_UP_TIM13_IRQn           44     /*!< TIM8 Update Interrupt and TIM13 global interrupt                  */
#define  TIM8_TRG_COM_TIM14_IRQn      45     /*!< TIM8 Trigger and Commutation Interrupt and TIM14 global interrupt */
#define  TIM8_CC_IRQn                 46     /*!< TIM8 Capture Compare Interrupt                                    */
#define  DMA1_Stream7_IRQn            47     /*!< DMA1 Stream7 Interrupt                                            */
#define  FMC_IRQn                     48     /*!< FMC global Interrupt                                              */
#define  SDIO_IRQn                    49     /*!< SDIO global Interrupt                                             */
#define  TIM5_IRQn                    50     /*!< TIM5 global Interrupt                                             */
#define  SPI3_IRQn                    51     /*!< SPI3 global Interrupt                                             */
#define  UART4_IRQn                   52     /*!< UART4 global Interrupt                                            */
#define  UART5_IRQn                   53     /*!< UART5 global Interrupt                                            */
#define  TIM6_DAC_IRQn                54     /*!< TIM6 global and DAC1&2 underrun error  interrupts                 */
#define  TIM7_IRQn                    55     /*!< TIM7 global interrupt                                             */
#define  DMA2_Stream0_IRQn            56     /*!< DMA2 Stream 0 global Interrupt                                    */
#define  DMA2_Stream1_IRQn            57     /*!< DMA2 Stream 1 global Interrupt                                    */
#define  DMA2_Stream2_IRQn            58     /*!< DMA2 Stream 2 global Interrupt                                    */
#define  DMA2_Stream3_IRQn            59     /*!< DMA2 Stream 3 global Interrupt                                    */
#define  DMA2_Stream4_IRQn            60     /*!< DMA2 Stream 4 global Interrupt                                    */
#define  ETH_IRQn                     61     /*!< Ethernet global Interrupt                                         */
#define  ETH_WKUP_IRQn                62     /*!< Ethernet Wakeup through EXTI line Interrupt                       */
#define  CAN2_TX_IRQn                 63     /*!< CAN2 TX Interrupt                                                 */
#define  CAN2_RX0_IRQn                64     /*!< CAN2 RX0 Interrupt                                                */
#define  CAN2_RX1_IRQn                65     /*!< CAN2 RX1 Interrupt                                                */
#define  CAN2_SCE_IRQn                66     /*!< CAN2 SCE Interrupt                                                */
#define  OTG_FS_IRQn                  67     /*!< USB OTG FS global Interrupt                                       */
#define  DMA2_Stream5_IRQn            68     /*!< DMA2 Stream 5 global interrupt                                    */
#define  DMA2_Stream6_IRQn            69     /*!< DMA2 Stream 6 global interrupt                                    */
#define  DMA2_Stream7_IRQn            70     /*!< DMA2 Stream 7 global interrupt                                    */
#define  USART6_IRQn                  71     /*!< USART6 global interrupt                                           */
#define  I2C3_EV_IRQn                 72     /*!< I2C3 event interrupt                                              */
#define  I2C3_ER_IRQn                 73     /*!< I2C3 error interrupt                                              */
#define  OTG_HS_EP1_OUT_IRQn          74     /*!< USB OTG HS End Point 1 Out global interrupt                       */
#define  OTG_HS_EP1_IN_IRQn           75     /*!< USB OTG HS End Point 1 In global interrupt                        */
#define  OTG_HS_WKUP_IRQn             76     /*!< USB OTG HS Wakeup through EXTI interrupt                          */
#define  OTG_HS_IRQn                  77     /*!< USB OTG HS global interrupt                                       */
#define  DCMI_IRQn                    78     /*!< DCMI global interrupt                                             */
#define  CRYP_IRQn                    79     /*!< CRYP crypto global interrupt                                      */
#define  HASH_RNG_IRQn                80     /*!< Hash and Rng global interrupt                                     */
#define  FPU_IRQn                     81     /*!< FPU global interrupt                                              */
#define  UART7_IRQn                   82     /*!< UART7 global interrupt                                            */
#define  UART8_IRQn                   83     /*!< UART8 global interrupt                                            */
#define  SPI4_IRQn                    84     /*!< SPI4 global Interrupt                                             */
#define  SPI5_IRQn                    85     /*!< SPI5 global Interrupt                                             */
#define  SPI6_IRQn                    86     /*!< SPI6 global Interrupt                                             */
#define  SAI1_IRQn                    87     /*!< SAI1 global Interrupt                                             */
#define  LTDC_IRQn                    88     /*!< LTDC global Interrupt                                             */
#define  LTDC_ER_IRQn                 89     /*!< LTDC Error global Interrupt                                       */
#define  DMA2D_IRQn                   90      /*!< DMA2D global Interrupt                                            */   
#endif /* STM32F429_439xx */
   
#if defined (STM32F401xx)
#define  EXTI9_5_IRQn                 23     /*!< External Line[9:5] Interrupts                                     */
#define  TIM1_BRK_TIM9_IRQn           24     /*!< TIM1 Break interrupt and TIM9 global interrupt                    */
#define  TIM1_UP_TIM10_IRQn           25     /*!< TIM1 Update Interrupt and TIM10 global interrupt                  */
#define  TIM1_TRG_COM_TIM11_IRQn      26     /*!< TIM1 Trigger and Commutation Interrupt and TIM11 global interrupt */
#define  TIM1_CC_IRQn                 27     /*!< TIM1 Capture Compare Interrupt                                    */
#define  TIM2_IRQn                    28     /*!< TIM2 global Interrupt                                             */
#define  TIM3_IRQn                    29     /*!< TIM3 global Interrupt                                             */
#define  TIM4_IRQn                    30     /*!< TIM4 global Interrupt                                             */
#define  I2C1_EV_IRQn                 31     /*!< I2C1 Event Interrupt                                              */
#define  I2C1_ER_IRQn                 32     /*!< I2C1 Error Interrupt                                              */
#define  I2C2_EV_IRQn                 33     /*!< I2C2 Event Interrupt                                              */
#define  I2C2_ER_IRQn                 34     /*!< I2C2 Error Interrupt                                              */  
#define  SPI1_IRQn                    35     /*!< SPI1 global Interrupt                                             */
#define  SPI2_IRQn                    36     /*!< SPI2 global Interrupt                                             */
#define  USART1_IRQn                  37     /*!< USART1 global Interrupt                                           */
#define  USART2_IRQn                  38     /*!< USART2 global Interrupt                                           */
#define  EXTI15_10_IRQn               40     /*!< External Line[15:10] Interrupts                                   */
#define  RTC_Alarm_IRQn               41     /*!< RTC Alarm (A and B) through EXTI Line Interrupt                   */
#define  OTG_FS_WKUP_IRQn             42     /*!< USB OTG FS Wakeup through EXTI line interrupt                     */  
#define  DMA1_Stream7_IRQn            47     /*!< DMA1 Stream7 Interrupt                                            */
#define  SDIO_IRQn                    49     /*!< SDIO global Interrupt                                             */
#define  TIM5_IRQn                    50     /*!< TIM5 global Interrupt                                             */
#define  SPI3_IRQn                    51     /*!< SPI3 global Interrupt                                             */
#define  DMA2_Stream0_IRQn            56     /*!< DMA2 Stream 0 global Interrupt                                    */
#define  DMA2_Stream1_IRQn            57     /*!< DMA2 Stream 1 global Interrupt                                    */
#define  DMA2_Stream2_IRQn            58     /*!< DMA2 Stream 2 global Interrupt                                    */
#define  DMA2_Stream3_IRQn            59     /*!< DMA2 Stream 3 global Interrupt                                    */
#define  DMA2_Stream4_IRQn            60     /*!< DMA2 Stream 4 global Interrupt                                    */
#define  OTG_FS_IRQn                  67     /*!< USB OTG FS global Interrupt                                       */
#define  DMA2_Stream5_IRQn            68     /*!< DMA2 Stream 5 global interrupt                                    */
#define  DMA2_Stream6_IRQn            69     /*!< DMA2 Stream 6 global interrupt                                    */
#define  DMA2_Stream7_IRQn            70     /*!< DMA2 Stream 7 global interrupt                                    */
#define  USART6_IRQn                  71     /*!< USART6 global interrupt                                           */
#define  I2C3_EV_IRQn                 72     /*!< I2C3 event interrupt                                              */
#define  I2C3_ER_IRQn                 73     /*!< I2C3 error interrupt                                              */
#define  FPU_IRQn                     81      /*!< FPU global interrupt                                             */
#define  SPI4_IRQn                    84       /*!< SPI4 global Interrupt                                            */
#endif /* STM32F401xx */


