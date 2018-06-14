#include <libfirmware/driver.h>

#include "usb_serial.h"

#include <usb_vcp/usbd_usr.h>
#include <usb_vcp/usbd_desc.h>
#include <usb_vcp/usbd_cdc_vcp.h>

__ALIGN_BEGIN USB_OTG_CORE_HANDLE    USB_OTG_dev __ALIGN_END ;

void usb_serial_init(struct usb_serial *self){
	(void)self;
	USBD_Init(&USB_OTG_dev,
             USB_OTG_FS_CORE_ID,
             &USR_desc,
             &USBD_CDC_cb,
             &USR_cb);
}

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

static int _stm32_usb_serial_probe(void *fdt, int fdt_node){
	USBD_Init(&USB_OTG_dev,
             USB_OTG_FS_CORE_ID,
             &USR_desc,
             &USBD_CDC_cb,
             &USR_cb);
	return 0;
}

static int _stm32_usb_serial_remove(void *fdt, int fdt_node){

}

DEVICE_DRIVER("stm32_usb", "st,usb-serial", _stm32_usb_serial_probe, _stm32_usb_serial_remove);
