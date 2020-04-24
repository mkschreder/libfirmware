#pragma once

#define TIM_OCMode_Timing                  (0x0000)
#define TIM_OCMode_Active                  (0x0010)
#define TIM_OCMode_Inactive                (0x0020)
#define TIM_OCMode_Toggle                  (0x0030)
#define TIM_OCMode_PWM1                    (0x0060)
#define TIM_OCMode_PWM2                    (0x0070)

#define TIM_OPMode_Single                  (0x0008)
#define TIM_OPMode_Repetitive              (0x0000)

#define TIM_Channel_1                      (0x0000)
#define TIM_Channel_2                      (0x0004)
#define TIM_Channel_3                      (0x0008)
#define TIM_Channel_4                      (0x000C)

#define TIM_CKD_DIV1                       (0x0000)
#define TIM_CKD_DIV2                       (0x0100)
#define TIM_CKD_DIV4                       (0x0200)

#define TIM_CounterMode_Up                 (0x0000)
#define TIM_CounterMode_Down               (0x0010)
#define TIM_CounterMode_CenterAligned1     (0x0020)
#define TIM_CounterMode_CenterAligned2     (0x0040)
#define TIM_CounterMode_CenterAligned3     (0x0060)

#define TIM_OCPolarity_High                (0x0000)
#define TIM_OCPolarity_Low                 (0x0002)

#define TIM_OCNPolarity_High               (0x0000)
#define TIM_OCNPolarity_Low                (0x0008)

#define TIM_OutputState_Disable            (0x0000)
#define TIM_OutputState_Enable             (0x0001)

#define TIM_OutputNState_Disable           (0x0000)
#define TIM_OutputNState_Enable            (0x0004)

#define TIM_CCx_Enable                      (0x0001)
#define TIM_CCx_Disable                     (0x0000)

#define TIM_CCxN_Enable                     (0x0004)
#define TIM_CCxN_Disable                    (0x0000)

#define TIM_Break_Enable                   (0x1000)
#define TIM_Break_Disable                  (0x0000)

#define TIM_BreakPolarity_Low              (0x0000)
#define TIM_BreakPolarity_High             (0x2000)

#define TIM_AutomaticOutput_Enable         (0x4000)
#define TIM_AutomaticOutput_Disable        (0x0000)

#define TIM_LOCKLevel_OFF                  (0x0000)
#define TIM_LOCKLevel_1                    (0x0100)
#define TIM_LOCKLevel_2                    (0x0200)
#define TIM_LOCKLevel_3                    (0x0300)

#define TIM_OSSIState_Enable               (0x0400)
#define TIM_OSSIState_Disable              (0x0000)

#define TIM_OSSRState_Enable               (0x0800)
#define TIM_OSSRState_Disable              (0x0000)

#define TIM_OCIdleState_Set                (0x0100)
#define TIM_OCIdleState_Reset              (0x0000)

#define TIM_OCNIdleState_Set               (0x0200)
#define TIM_OCNIdleState_Reset             (0x0000)

#define  TIM_ICPolarity_Rising             (0x0000)
#define  TIM_ICPolarity_Falling            (0x0002)
#define  TIM_ICPolarity_BothEdge           (0x000A)

#define TIM_ICSelection_DirectTI           (0x0001) /*!< TIM Input 1, 2, 3 or 4 is selected to be
                                                                   connected to IC1, IC2, IC3 or IC4, respectively */
#define TIM_ICSelection_IndirectTI         (0x0002) /*!< TIM Input 1, 2, 3 or 4 is selected to be
                                                                   connected to IC2, IC1, IC4 or IC3, respectively. */
#define TIM_ICSelection_TRC                (0x0003) /*!< TIM Input 1, 2, 3 or 4 is selected to be connected to TRC. */

#define TIM_ICPSC_DIV1                     (0x0000) /*!< Capture performed each time an edge is detected on the capture input. */
#define TIM_ICPSC_DIV2                     (0x0004) /*!< Capture performed once every 2 events. */
#define TIM_ICPSC_DIV4                     (0x0008) /*!< Capture performed once every 4 events. */
#define TIM_ICPSC_DIV8                     (0x000C) /*!< Capture performed once every 8 events. */

#define TIM_IT_Update                      (0x0001)
#define TIM_IT_CC1                         (0x0002)
#define TIM_IT_CC2                         (0x0004)
#define TIM_IT_CC3                         (0x0008)
#define TIM_IT_CC4                         (0x0010)
#define TIM_IT_COM                         (0x0020)
#define TIM_IT_Trigger                     (0x0040)
#define TIM_IT_Break                       (0x0080)

#define TIM_DMABase_CR1                    (0x0000)
#define TIM_DMABase_CR2                    (0x0001)
#define TIM_DMABase_SMCR                   (0x0002)
#define TIM_DMABase_DIER                   (0x0003)
#define TIM_DMABase_SR                     (0x0004)
#define TIM_DMABase_EGR                    (0x0005)
#define TIM_DMABase_CCMR1                  (0x0006)
#define TIM_DMABase_CCMR2                  (0x0007)
#define TIM_DMABase_CCER                   (0x0008)
#define TIM_DMABase_CNT                    (0x0009)
#define TIM_DMABase_PSC                    (0x000A)
#define TIM_DMABase_ARR                    (0x000B)
#define TIM_DMABase_RCR                    (0x000C)
#define TIM_DMABase_CCR1                   (0x000D)
#define TIM_DMABase_CCR2                   (0x000E)
#define TIM_DMABase_CCR3                   (0x000F)
#define TIM_DMABase_CCR4                   (0x0010)
#define TIM_DMABase_BDTR                   (0x0011)
#define TIM_DMABase_DCR                    (0x0012)

#define TIM_DMABurstLength_1Transfer           (0x0000)
#define TIM_DMABurstLength_2Transfers          (0x0100)
#define TIM_DMABurstLength_3Transfers          (0x0200)
#define TIM_DMABurstLength_4Transfers          (0x0300)
#define TIM_DMABurstLength_5Transfers          (0x0400)
#define TIM_DMABurstLength_6Transfers          (0x0500)
#define TIM_DMABurstLength_7Transfers          (0x0600)
#define TIM_DMABurstLength_8Transfers          (0x0700)
#define TIM_DMABurstLength_9Transfers          (0x0800)
#define TIM_DMABurstLength_10Transfers         (0x0900)
#define TIM_DMABurstLength_11Transfers         (0x0A00)
#define TIM_DMABurstLength_12Transfers         (0x0B00)
#define TIM_DMABurstLength_13Transfers         (0x0C00)
#define TIM_DMABurstLength_14Transfers         (0x0D00)
#define TIM_DMABurstLength_15Transfers         (0x0E00)
#define TIM_DMABurstLength_16Transfers         (0x0F00)
#define TIM_DMABurstLength_17Transfers         (0x1000)
#define TIM_DMABurstLength_18Transfers         (0x1100)

#define TIM_DMA_Update                     (0x0100)
#define TIM_DMA_CC1                        (0x0200)
#define TIM_DMA_CC2                        (0x0400)
#define TIM_DMA_CC3                        (0x0800)
#define TIM_DMA_CC4                        (0x1000)
#define TIM_DMA_COM                        (0x2000)
#define TIM_DMA_Trigger                    (0x4000)

#define TIM_ExtTRGPSC_OFF                  (0x0000)
#define TIM_ExtTRGPSC_DIV2                 (0x1000)
#define TIM_ExtTRGPSC_DIV4                 (0x2000)
#define TIM_ExtTRGPSC_DIV8                 (0x3000)

#define TIM_TS_ITR0                        (0x0000)
#define TIM_TS_ITR1                        (0x0010)
#define TIM_TS_ITR2                        (0x0020)
#define TIM_TS_ITR3                        (0x0030)
#define TIM_TS_TI1F_ED                     (0x0040)
#define TIM_TS_TI1FP1                      (0x0050)
#define TIM_TS_TI2FP2                      (0x0060)
#define TIM_TS_ETRF                        (0x0070)

#define TIM_TIxExternalCLK1Source_TI1      (0x0050)
#define TIM_TIxExternalCLK1Source_TI2      (0x0060)
#define TIM_TIxExternalCLK1Source_TI1ED    (0x0040)

#define TIM_ExtTRGPolarity_Inverted        (0x8000)
#define TIM_ExtTRGPolarity_NonInverted     (0x0000)

#define TIM_PSCReloadMode_Update           (0x0000)
#define TIM_PSCReloadMode_Immediate        (0x0001)

#define TIM_ForcedAction_Active            (0x0050)
#define TIM_ForcedAction_InActive          (0x0040)

#define TIM_EncoderMode_TI1                (0x0001)
#define TIM_EncoderMode_TI2                (0x0002)
#define TIM_EncoderMode_TI12               (0x0003)

#define TIM_EventSource_Update             (0x0001)
#define TIM_EventSource_CC1                (0x0002)
#define TIM_EventSource_CC2                (0x0004)
#define TIM_EventSource_CC3                (0x0008)
#define TIM_EventSource_CC4                (0x0010)
#define TIM_EventSource_COM                (0x0020)
#define TIM_EventSource_Trigger            (0x0040)
#define TIM_EventSource_Break              (0x0080)

#define TIM_UpdateSource_Global            (0x0000) /*!< Source of update is the counter overflow/underflow
                                                                   or the setting of UG bit, or an update generation
                                                                   through the slave mode controller. */
#define TIM_UpdateSource_Regular           (0x0001) /*!< Source of update is counter overflow/underflow. */

#define TIM_OCPreload_Enable               (0x0008)
#define TIM_OCPreload_Disable              (0x0000)

#define TIM_OCFast_Enable                  (0x0004)
#define TIM_OCFast_Disable                 (0x0000)

#define TIM_OCClear_Enable                 (0x0080)
#define TIM_OCClear_Disable                (0x0000)

#define TIM_TRGOSource_Reset               (0x0000)
#define TIM_TRGOSource_Enable              (0x0010)
#define TIM_TRGOSource_Update              (0x0020)
#define TIM_TRGOSource_OC1                 (0x0030)
#define TIM_TRGOSource_OC1Ref              (0x0040)
#define TIM_TRGOSource_OC2Ref              (0x0050)
#define TIM_TRGOSource_OC3Ref              (0x0060)
#define TIM_TRGOSource_OC4Ref              (0x0070)

#define TIM_SlaveMode_Reset                (0x0004)
#define TIM_SlaveMode_Gated                (0x0005)
#define TIM_SlaveMode_Trigger              (0x0006)
#define TIM_SlaveMode_External1            (0x0007)

#define TIM_MasterSlaveMode_Enable         (0x0080)
#define TIM_MasterSlaveMode_Disable        (0x0000)

#define TIM_FLAG_Update                    (0x0001)
#define TIM_FLAG_CC1                       (0x0002)
#define TIM_FLAG_CC2                       (0x0004)
#define TIM_FLAG_CC3                       (0x0008)
#define TIM_FLAG_CC4                       (0x0010)
#define TIM_FLAG_COM                       (0x0020)
#define TIM_FLAG_Trigger                   (0x0040)
#define TIM_FLAG_Break                     (0x0080)
#define TIM_FLAG_CC1OF                     (0x0200)
#define TIM_FLAG_CC2OF                     (0x0400)
#define TIM_FLAG_CC3OF                     (0x0800)
#define TIM_FLAG_CC4OF                     (0x1000)

#define TIM_DMABurstLength_1Byte           TIM_DMABurstLength_1Transfer
#define TIM_DMABurstLength_2Bytes          TIM_DMABurstLength_2Transfers
#define TIM_DMABurstLength_3Bytes          TIM_DMABurstLength_3Transfers
#define TIM_DMABurstLength_4Bytes          TIM_DMABurstLength_4Transfers
#define TIM_DMABurstLength_5Bytes          TIM_DMABurstLength_5Transfers
#define TIM_DMABurstLength_6Bytes          TIM_DMABurstLength_6Transfers
#define TIM_DMABurstLength_7Bytes          TIM_DMABurstLength_7Transfers
#define TIM_DMABurstLength_8Bytes          TIM_DMABurstLength_8Transfers
#define TIM_DMABurstLength_9Bytes          TIM_DMABurstLength_9Transfers
#define TIM_DMABurstLength_10Bytes         TIM_DMABurstLength_10Transfers
#define TIM_DMABurstLength_11Bytes         TIM_DMABurstLength_11Transfers
#define TIM_DMABurstLength_12Bytes         TIM_DMABurstLength_12Transfers
#define TIM_DMABurstLength_13Bytes         TIM_DMABurstLength_13Transfers
#define TIM_DMABurstLength_14Bytes         TIM_DMABurstLength_14Transfers
#define TIM_DMABurstLength_15Bytes         TIM_DMABurstLength_15Transfers
#define TIM_DMABurstLength_16Bytes         TIM_DMABurstLength_16Transfers
#define TIM_DMABurstLength_17Bytes         TIM_DMABurstLength_17Transfers
#define TIM_DMABurstLength_18Bytes         TIM_DMABurstLength_18Transfers
