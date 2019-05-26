/** :ms-top-comment
* +-------------------------------------------------------------------------------+
* |                      ____                  _ _     _                          |
* |                     / ___|_      _____  __| (_)___| |__                       |
* |                     \___ \ \ /\ / / _ \/ _` | / __| '_ \                      |
* |                      ___) \ V  V /  __/ (_| | \__ \ | | |                     |
* |                     |____/ \_/\_/ \___|\__,_|_|___/_| |_|                     |
* |                                                                               |
* |               _____           _              _     _          _               |
* |              | ____|_ __ ___ | |__   ___  __| | __| | ___  __| |              |
* |              |  _| | '_ ` _ \| '_ \ / _ \/ _` |/ _` |/ _ \/ _` |              |
* |              | |___| | | | | | |_) |  __/ (_| | (_| |  __/ (_| |              |
* |              |_____|_| |_| |_|_.__/ \___|\__,_|\__,_|\___|\__,_|              |
* |                                                                               |
* |                       We design hardware and program it                       |
* |                                                                               |
* |               If you have software that needs to be developed or              |
* |                      hardware that needs to be designed,                      |
* |                               then get in touch!                              |
* |                                                                               |
* |                            info@swedishembedded.com                           |
* +-------------------------------------------------------------------------------+
*
*                       This file is part of TheBoss Project
*
* FILE ............... src/stm32f4xx/usb_serial.c
* AUTHOR ............. Martin K. Schröder
* VERSION ............ Not tagged
* DATE ............... 2019-05-26
* GIT ................ https://github.com/mkschreder/libfirmware
* WEBSITE ............ http://swedishembedded.com
* LICENSE ............ Swedish Embedded Open Source License
*
*          Copyright (C) 2014-2019 Martin Schröder. All rights reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a copy of
* this software and associated documentation files (the "Software"), to deal in
* the Software without restriction, including without limitation the rights to
* use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
* the Software, and to permit persons to whom the Software is furnished to do so,
* subject to the following conditions:
*
* The above copyright notice and this text, in full, shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
* FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
* COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
* IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
* CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
**/
#include "driver.h"

#include "console.h"

#include <usb_vcp/usbd_usr.h>
#include <usb_vcp/usbd_desc.h>
#include <usb_vcp/usbd_cdc_vcp.h>

#include "serial.h"

struct usb_serial {
	struct serial_device_ops dev;
};

void usb_serial_init(struct usb_serial *self);
int usb_serial_read(struct usb_serial *self, void *buf, size_t size);
int usb_serial_write(struct usb_serial *self, const void *buf, size_t size);
USB_OTG_CORE_HANDLE    USB_OTG_dev;

int usb_serial_read(struct usb_serial *self, void *buf, size_t size){
	(void)self;

	uint32_t cnt = VCP_DataRx((uint8_t*)buf, size);
	if(cnt <= 0) return -1;
	return (int)cnt;
}

int usb_serial_write(struct usb_serial *self, const void *buf, size_t size){
	(void)self;
	uint32_t cnt = 0;
	while(cnt < size){
		size_t c = VCP_DataTx(buf, (uint8_t)size);
		cnt += c;
	}

	return (int)cnt;
}

static int _stm32_usb_cmd(console_device_t con, void *ptr, int argc, char **argv){
	if(argc == 2 && strcmp(argv[1], "status") == 0){
		static const struct {
			int bit; 
			const char *name;
		} bits[] = {
			{ .bit = 0, "SRQSCS" },
			{ .bit = 1, "SRQ" },
			{ .bit = 8, "HNGSCS" },
			{ .bit = 9, "HNPRQ" },
			{ .bit = 10, "HSHNPEN" },
			{ .bit = 11, "DHNPEN" },
			{ .bit = 16, "CIDSTS" },
			{ .bit = 17, "DBCT" },
			{ .bit = 18, "ASVLD" },
			{ .bit = 19, "BSVLD" }
		};
		uint32_t status = USB_OTG_dev.regs.GREGS->GOTGCTL;
		for(size_t c = 0; c < sizeof(bits) / sizeof(bits[0]); c++){
			if(status & (uint32_t)(1 << bits[c].bit)){
				console_printf(con, "%s: %d\n", bits[c].name, 1);
			} else {
				console_printf(con, "%s: %d\n", bits[c].name, 0);
			}
		}
	}
	return 0;
}

static int _stm32_usb_serial_probe(void *fdt, int fdt_node){
	console_device_t console = console_find_by_ref(fdt, fdt_node, "console");
	int core = fdt_get_int_or_default(fdt, fdt_node, "core", USB_OTG_FS_CORE_ID);

	if(console){
		console_add_command(console, NULL, fdt_get_name(fdt, fdt_node, NULL), "", "", _stm32_usb_cmd);
	}

	USB_OTG_CORE_HANDLE *hw = &USB_OTG_dev;

#ifdef VBUS_SENSING_ENABLED
    //IOConfigGPIO(IOGetByTag(IO_TAG(VBUS_SENSING_PIN)), IOCFG_IN_FLOATING);
#endif

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_OTG_FS, ENABLE) ;
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_PWR, ENABLE);

    EXTI_ClearITPendingBit(EXTI_Line0);

	hw->dev.usr_device = &USR_desc;
	hw->dev.class_cb = &USBD_CDC_cb;
	hw->dev.usr_cb = &USR_cb;

	DCD_Init(hw, core);

	hw->dev.usr_cb->Init();

    NVIC_InitTypeDef nvic;

	if(core == USB_OTG_HS_CORE_ID){
    	nvic.NVIC_IRQChannel = OTG_HS_IRQn;
		printk("USB: high speed mode (480Mbit)\n");
	} else if(core == USB_OTG_FS_CORE_ID){
    	nvic.NVIC_IRQChannel = OTG_FS_IRQn;
		printk("USB: full speed mode (12Mbit)\n");
	} else {
		printk("usb: unrecognized core option %d\n", core);
		return -EINVAL;
	}

    nvic.NVIC_IRQChannelPreemptionPriority = 2;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

	printk(PRINT_SUCCESS "USB: ready\n");
	return 0;
}

static int _stm32_usb_serial_remove(void *fdt, int fdt_node){
	return -1;
}

DEVICE_DRIVER(stm32_usb, "st,usb-serial", _stm32_usb_serial_probe, _stm32_usb_serial_remove)
