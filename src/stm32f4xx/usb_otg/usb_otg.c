/**
  ******************************************************************************
  * @file    usb_otg.c
  * @author  MCD Application Team
  * @version V2.0.0
  * @date    22-July-2011
  * @brief   OTG Core Layer
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#include "stm32f4xx/usb_otg/usb_defines.h"
#include "stm32f4xx/usb_otg/usb_regs.h"
#include "stm32f4xx/usb_otg/usb_core.h"
#include "stm32f4xx/usb_otg/usb_otg.h"

/** @addtogroup USB_OTG_DRIVER
  * @{
  */
  
/** @defgroup USB_OTG 
  * @brief This file is the interface between EFSL ans Host mass-storage class
  * @{
  */


/** @defgroup USB_OTG_Private_Defines
  * @{
  */ 
/**
  * @}
  */ 
 

/** @defgroup USB_OTG_Private_TypesDefinitions
  * @{
  */ 
/**
  * @}
  */ 



/** @defgroup USB_OTG_Private_Macros
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup USB_OTG_Private_Variables
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup USB_OTG_Private_FunctionPrototypes
  * @{
  */ 

static uint32_t USB_OTG_Read_itr(USB_OTG_CORE_HANDLE *pdev);

/**
  * @}
  */ 


/** @defgroup USB_OTG_Private_Functions
  * @{
  */ 


/*                           OTG Interrupt Handler                         */


/**
  * @brief  STM32_USBO_OTG_ISR_Handler
  *         
  * @param  None
  * @retval : None
  */
uint32_t STM32_USBO_OTG_ISR_Handler(USB_OTG_CORE_HANDLE *pdev)
{
  uint32_t retval = 0;
  USB_OTG_GINTSTS_TypeDef  gintsts ;
  gintsts.d32 = 0;

  gintsts.d32 = USB_OTG_Read_itr(pdev);
  if (gintsts.d32 == 0)
  {
    return 0;
  }
  if (gintsts.b.otgintr)
  {
	  // TODO: figure out where this is defined
    //retval |= USB_OTG_HandleOTG_ISR(pdev);
  }
  if (gintsts.b.conidstschng)
  {
    //retval |= USB_OTG_HandleConnectorIDStatusChange_ISR(pdev);
  }
  if (gintsts.b.sessreqintr)
  {
    //retval |= USB_OTG_HandleSessionRequest_ISR(pdev);
  }
  return retval;
}


/**
  * @brief  USB_OTG_Read_itr
  *         returns the Core Interrupt register
  * @param  None
  * @retval : status
  */
static uint32_t USB_OTG_Read_itr(USB_OTG_CORE_HANDLE *pdev)
{
  USB_OTG_GINTSTS_TypeDef  gintsts;
  USB_OTG_GINTMSK_TypeDef  gintmsk;
  USB_OTG_GINTMSK_TypeDef  gintmsk_common;
  
  
  gintsts.d32 = 0;
  gintmsk.d32 = 0;
  gintmsk_common.d32 = 0;
  
  /* OTG interrupts */
  gintmsk_common.b.sessreqintr = 1;
  gintmsk_common.b.conidstschng = 1;
  gintmsk_common.b.otgintr = 1;
  
  gintsts.d32 = USB_OTG_READ_REG32(&pdev->regs.GREGS->GINTSTS);
  gintmsk.d32 = USB_OTG_READ_REG32(&pdev->regs.GREGS->GINTMSK);
  return ((gintsts.d32 & gintmsk.d32 ) & gintmsk_common.d32);
}


/**
  * @brief  USB_OTG_GetCurrentState
  *         Return current OTG State
  * @param  None
  * @retval : None
  */
uint32_t USB_OTG_GetCurrentState (USB_OTG_CORE_HANDLE *pdev)
{
	#ifdef USE_OTG_MODE
		return pdev->otg.OTG_State;
	#endif
	return 0;
}

/**
* @}
*/ 

/**
* @}
*/ 

/**
* @}
*/

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
