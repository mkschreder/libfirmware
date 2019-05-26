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
* FILE ............... src/usb.c
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
#include <errno.h>
#include <stdarg.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

#include <libfdt/libfdt.h>

#include "usb.h"
#include "list.h"
#include "thread.h"
#include "driver.h"
#include "chip.h"

static LIST_HEAD(_usbd_ports);

#define USB_DEFAULT_VENDOR_ID 0xffff
#define USB_DEFAULT_PRODUCT_ID 0xffff

static struct usb_endpoint_descriptor _ep0_desc = {
    0x07, /* bLength: Endpoint Descriptor size */
    0x05, /* bDescriptorType: Endpoint */
    0x00, /* bEndpointAddress IN1 */
    USB_EP_TYPE_CONTROL, /* bmAttributes: Control */
    0x08, /* wMaxPacketSize LO: */
    0x00, /* wMaxPacketSize HI: */
    0x00, /* bInterval: */
};

static const uint8_t _default_config_desc[] = {
    /*Configuration Descriptor*/
    0x09, /* bLength: Configuration Descriptor size */
    0x02, /* bDescriptorType: Configuration */
    0,   /* wTotalLength:no of returned bytes */
    0x00,
    0x04, /* bNumInterfaces: 2 interface */
    0x01, /* bConfigurationValue: Configuration value */
    0x00, /* iConfiguration: Index of string descriptor describing the configuration */
    0x80, /* bmAttributes - Bus powered */
    0x32, /* MaxPower 100 mA */

    /*---------------------------------------------------------------------------*/

    /*Interface Descriptor */
    0x09, /* bLength: Interface Descriptor size */
    0x04, /* bDescriptorType: Interface */
    0x00, /* bInterfaceNumber: Number of Interface */
    0x00, /* bAlternateSetting: Alternate setting */
    0x01, /* bNumEndpoints: One endpoints used */
    0x02, /* bInterfaceClass: Communication Interface Class */
    0x02, /* bInterfaceSubClass: Abstract Control Model */
    0x01, /* bInterfaceProtocol: Common AT commands */
    0x00, /* iInterface: */

    /*Header Functional Descriptor*/
    0x05, /* bLength: Endpoint Descriptor size */
    0x24, /* bDescriptorType: CS_INTERFACE */
    0x00, /* bDescriptorSubtype: Header Func Desc */
    0x10, /* bcdCDC: spec release number */
    0x01,

    /*Call Management Functional Descriptor*/
    0x05, /* bFunctionLength */
    0x24, /* bDescriptorType: CS_INTERFACE */
    0x01, /* bDescriptorSubtype: Call Management Func Desc */
    0x00, /* bmCapabilities: D0+D1 */
    0x01, /* bDataInterface: 1 */

    /*ACM Functional Descriptor*/
    0x04, /* bFunctionLength */
    0x24, /* bDescriptorType: CS_INTERFACE */
    0x02, /* bDescriptorSubtype: Abstract Control Management desc */
    0x02, /* bmCapabilities */

    /*Union Functional Descriptor*/
    0x05, /* bFunctionLength */
    0x24, /* bDescriptorType: CS_INTERFACE */
    0x06, /* bDescriptorSubtype: Union func desc */
    0x00, /* bMasterInterface: Communication class interface */
    0x01, /* bSlaveInterface0: Data Class Interface */

    /*Endpoint 2 Descriptor*/
    0x07, /* bLength: Endpoint Descriptor size */
    0x05, /* bDescriptorType: Endpoint */
    0x81, /* bEndpointAddress IN1 */
    USB_EP_TYPE_INTERRUPT, /* bmAttributes: Interrupt */
    0x08, /* wMaxPacketSize LO: */
    0x00, /* wMaxPacketSize HI: */
    0x10, /* bInterval: */
    /*---------------------------------------------------------------------------*/

    /*Data class interface descriptor*/
    0x09, /* bLength: Endpoint Descriptor size */
    0x04, /* bDescriptorType: */
    0x01, /* bInterfaceNumber: Number of Interface */
    0x00, /* bAlternateSetting: Alternate setting */
    0x02, /* bNumEndpoints: Two endpoints used */
    0x0A, /* bInterfaceClass: CDC */
    0x02, /* bInterfaceSubClass: */
    0x00, /* bInterfaceProtocol: */
    0x00, /* iInterface: */

    /*Endpoint IN2 Descriptor*/
    0x07, /* bLength: Endpoint Descriptor size */
    0x05, /* bDescriptorType: Endpoint */
    0x82, /* bEndpointAddress IN2 */
    USB_EP_TYPE_BULK, /* bmAttributes: Bulk */
    64,   /* wMaxPacketSize: */
    0x00,
    0x00, /* bInterval: ignore for Bulk transfer */

    /*Endpoint OUT3 Descriptor*/
    0x07, /* bLength: Endpoint Descriptor size */
    0x05, /* bDescriptorType: Endpoint */
    0x03, /* bEndpointAddress */
    USB_EP_TYPE_BULK, /* bmAttributes: Bulk */
    64,   /* wMaxPacketSize: */
    0,
    0x00, /* bInterval: ignore for Bulk transfer */

    /*Interface Descriptor */
    0x09, /* bLength: Interface Descriptor size */
    0x04, /* bDescriptorType: Interface */
    0x02, /* bInterfaceNumber: Number of Interface */
    0x00, /* bAlternateSetting: Alternate setting */
    0x02, /* bNumEndpoints: One endpoints used */
    0xff, /* bInterfaceClass: Communication Interface Class */
    0x00, /* bInterfaceSubClass: Abstract Control Model */
    0x00, /* bInterfaceProtocol: Common AT commands */
    0x00, /* iInterface: */

    /*Endpoint IN2 Descriptor*/
    0x07, /* bLength: Endpoint Descriptor size */
    0x05, /* bDescriptorType: Endpoint */
    0x84, /* bEndpointAddress IN2 */
    USB_EP_TYPE_ISOCHRONOUS, /* bmAttributes: Bulk */
    64,   /* wMaxPacketSize: */
    0x00,
    0x00, /* bInterval: ignore for Bulk transfer */

    /*Endpoint OUT3 Descriptor*/
    0x07, /* bLength: Endpoint Descriptor size */
    0x05, /* bDescriptorType: Endpoint */
    0x05, /* bEndpointAddress */
    USB_EP_TYPE_ISOCHRONOUS | USB_EP_ISO_ASYNC, /* bmAttributes: Bulk */
    64,   /* wMaxPacketSize: */
    0,
    0x00 /* bInterval: ignore for Bulk transfer */

#if 0
    /*Interface Descriptor */
    0x09, /* bLength: Interface Descriptor size */
    0x04, /* bDescriptorType: Interface */
    0x02, /* bInterfaceNumber: Number of Interface */
    0x00, /* bAlternateSetting: Alternate setting */
    0x01, /* bNumEndpoints: One endpoints used */
    0x02, /* bInterfaceClass: Communication Interface Class */
    0x02, /* bInterfaceSubClass: Abstract Control Model */
    0x01, /* bInterfaceProtocol: Common AT commands */
    0x00, /* iInterface: */

    /*Header Functional Descriptor*/
    0x05, /* bLength: Endpoint Descriptor size */
    0x24, /* bDescriptorType: CS_INTERFACE */
    0x00, /* bDescriptorSubtype: Header Func Desc */
    0x10, /* bcdCDC: spec release number */
    0x01,

    /*Call Management Functional Descriptor*/
    0x05, /* bFunctionLength */
    0x24, /* bDescriptorType: CS_INTERFACE */
    0x01, /* bDescriptorSubtype: Call Management Func Desc */
    0x00, /* bmCapabilities: D0+D1 */
    0x03, /* bDataInterface: 1 */

    /*ACM Functional Descriptor*/
    0x04, /* bFunctionLength */
    0x24, /* bDescriptorType: CS_INTERFACE */
    0x02, /* bDescriptorSubtype: Abstract Control Management desc */
    0x02, /* bmCapabilities */

    /*Union Functional Descriptor*/
    0x05, /* bFunctionLength */
    0x24, /* bDescriptorType: CS_INTERFACE */
    0x06, /* bDescriptorSubtype: Union func desc */
    0x02, /* bMasterInterface: Communication class interface */
    0x03, /* bSlaveInterface0: Data Class Interface */

    /*Endpoint 2 Descriptor*/
    0x07, /* bLength: Endpoint Descriptor size */
    0x05, /* bDescriptorType: Endpoint */
    0x84, /* bEndpointAddress IN1 */
    0x03, /* bmAttributes: Interrupt */
    0x08, /* wMaxPacketSize LO: */
    0x00, /* wMaxPacketSize HI: */
    0x10, /* bInterval: */

    /*Data class interface descriptor*/
    0x09, /* bLength: Endpoint Descriptor size */
    0x04, /* bDescriptorType: */
    0x03, /* bInterfaceNumber: Number of Interface */
    0x00, /* bAlternateSetting: Alternate setting */
    0x02, /* bNumEndpoints: Two endpoints used */
    0x0A, /* bInterfaceClass: CDC */
    0x02, /* bInterfaceSubClass: */
    0x00, /* bInterfaceProtocol: */
    0x00, /* iInterface: */

    /*Endpoint IN2 Descriptor*/
    0x07, /* bLength: Endpoint Descriptor size */
    0x05, /* bDescriptorType: Endpoint */
    0x85, /* bEndpointAddress IN2 */
    0x02, /* bmAttributes: Bulk */
    64,   /* wMaxPacketSize: */
    0x00,
    0x00, /* bInterval: ignore for Bulk transfer */

    /*Endpoint OUT3 Descriptor*/
    0x07, /* bLength: Endpoint Descriptor size */
    0x05, /* bDescriptorType: Endpoint */
    0x06, /* bEndpointAddress */
    0x02, /* bmAttributes: Bulk */
    64,   /* wMaxPacketSize: */
    0,
    0x00 /* bInterval: ignore for Bulk transfer */
#endif
};

static const char *_default_vendor = "Default Vendor";
static const char *_default_product = "Default Product";

static struct usb_string_descriptor *_alloc_string_desc(const char *str){
    size_t str_len = strlen(str);
    // limit string size so our descriptor does not become too large
    if(str_len > 126) str_len = 126;

    size_t dat_len = str_len * 2;
    struct usb_string_descriptor *s = kzmalloc(sizeof(struct usb_string_descriptor) + dat_len);
    s->bLength = (uint8_t)(sizeof(struct usb_string_descriptor) + dat_len);
    s->bDescriptorType = USB_DESC_TYPE_STR;
    uint8_t *data = (uint8_t*)s + sizeof(struct usb_string_descriptor);
    for(size_t c = 0; c < str_len; c++){
        data[c * 2] = (uint8_t)str[c];
        data[c * 2 + 1] = 0;
    }
    return s;
}

#define foreach_config_desc(d, cd) \
    for(struct usb_descriptor_header *d = (struct usb_descriptor_header*)cd;\
        ((uint16_t)((char*)d - (char*)cd)) < USB_WORD_HL(cd->wTotalLengthH, cd->wTotalLengthL);\
        d = (struct usb_descriptor_header*)((char*)d + d->bLength))

static void _alloc_endpoints(struct usbd_device *self){
    // first count number of endpoints
    self->endpoint_count = 1; // count control endpoint as well
    foreach_config_desc(d, self->config_desc){
        if(d->bDescriptorType == USB_DESC_TYPE_EP){
            self->endpoint_count++;
        }
    }
    self->endpoints = kzmalloc(sizeof(struct usb_endpoint*) * self->endpoint_count);
    // go over endpoints again and now allocate the buffers
    foreach_config_desc(d, self->config_desc){
        if(d->bDescriptorType == USB_DESC_TYPE_EP){
            struct usb_endpoint_descriptor *epd = (struct usb_endpoint_descriptor*)d;
            uint8_t idx = epd->bEndpointAddress & 0xf;
            if(idx >= self->endpoint_count || self->endpoints[idx])  continue;

            struct usb_endpoint *eb =(struct usb_endpoint*)kzmalloc(sizeof(struct usb_endpoint));
            usb_endpoint_init(eb, epd);
            self->endpoints[idx] = eb;

	        dbg_printk("USB: new %d, %d, %d\n", idx, (((uint8_t*)d - (uint8_t*)self->config_desc)), d->bLength);
        } else {
	        dbg_printk("USB: sk %d\n", d->bLength);
        }
    }
}

void usbd_device_init(struct usbd_device *self, void *fdt, int fdt_node, const struct usbd_device_ops *ops){
	memset(self, 0, sizeof(*self));
	INIT_LIST_HEAD(&self->list);
	self->fdt_node = fdt_node;
	self->ops = ops;

    // allocate descriptors and fill them in from fdt
    struct usb_device_descriptor *d = kzmalloc(sizeof(struct usb_device_descriptor));
    d->bLength = sizeof(struct usb_device_descriptor);
    d->bDescriptorType = USB_DESC_TYPE_DEVICE;
    d->bcdUSBL = 0x10;
    d->bcdUSBH = 0x01;
    d->bDeviceClass = 0;
    d->bDeviceSubClass = 0;
    d->bDeviceProtocol = 0;
    d->bMaxPacketSize0 = 8;
    d->idVendorL = USB_LOBYTE(USB_DEFAULT_VENDOR_ID);
    d->idVendorH = USB_HIBYTE(USB_DEFAULT_VENDOR_ID);
    d->idProductL = USB_LOBYTE(USB_DEFAULT_PRODUCT_ID);
    d->idProductH = USB_HIBYTE(USB_DEFAULT_PRODUCT_ID);
    d->bcdDeviceL = 0x00;
    d->bcdDeviceH = 0x01;
    d->iManufacturer = 1;
    d->iProduct = 2;
    d->iSerialNumber = 3;
    d->bNumConfigurations = 1;
    self->device_desc = d;

    uint16_t conf_len = (uint16_t)sizeof(_default_config_desc);
    struct usb_config_descriptor *cd = kzmalloc(sizeof(_default_config_desc));
    memcpy(cd, _default_config_desc, conf_len);
    cd->wTotalLengthL = USB_LOBYTE(conf_len);
    cd->wTotalLengthH = USB_HIBYTE(conf_len);
    self->config_desc = cd;

    self->string_desc_count = 4;
    const struct usb_string_descriptor **sd = kzmalloc(sizeof(struct usb_string_descriptor*) * self->string_desc_count);

    // language
    struct usb_string_descriptor *s = kzmalloc(sizeof(struct usb_string_descriptor) + 2);
    s->bLength = 4;
    s->bDescriptorType = USB_DESC_TYPE_STR;
    uint8_t *data = (uint8_t*)s + sizeof(struct usb_string_descriptor);
    data[0] = 0x09;
    data[1] = 0x04;
    sd[0] = s;

    // vendor / product
    sd[1] = _alloc_string_desc(_default_vendor);
    sd[2] = _alloc_string_desc(_default_product);

    // serial
    uint32_t uuid[3];
    chip_get_uuid(uuid);
    #define _max_len (3 * sizeof(uint32_t) * 2 + 3)
    char str_serial[_max_len];
    snprintf(str_serial, _max_len, "%04x-%04x-%04x", (unsigned int)uuid[0], (unsigned int)uuid[1], (unsigned int)uuid[2]);
    #undef _max_len
    sd[3] = _alloc_string_desc(str_serial);

    self->string_desc = sd;

    // allocate endpoint buffers
    _alloc_endpoints(self);

    // allocate control endpoint buffer
    struct usb_endpoint *eb = self->endpoints[0] = kzmalloc(sizeof(struct usb_endpoint));
    usb_endpoint_init(eb, &_ep0_desc);
}

int usbd_device_register(struct usbd_device *self){
	BUG_ON(!self);
	BUG_ON(!self->ops);
	BUG_ON(!self->ops->write);
	BUG_ON(!self->ops->read);
	list_add_tail(&self->list, &_usbd_ports);
	return 0;
}

usbd_device_t usbd_find(const char *dtb_path){
	struct usbd_device *usbd;
	int node = fdt_path_offset(_devicetree, dtb_path);
	if(node < 0) return NULL;
	list_for_each_entry(usbd, &_usbd_ports, list){
		if(usbd->fdt_node == node) return &usbd->ops;
	}
	return NULL;
}

void usb_endpoint_init(struct usb_endpoint *self, struct usb_endpoint_descriptor *epd){
    uint16_t buffer_size = USB_WORD_HL(epd->wMaxPacketSizeH, epd->wMaxPacketSizeL);

    thread_sem_init(&self->tx_ready);
    thread_sem_init(&self->rx_ready);

    self->buffer_size = buffer_size;
    self->desc = epd;
}
/*
int usbd_endpoint_write(struct usbd_device *self, unsigned int endp, const uint8_t *data, size_t size, uint32_t timeout){
    if(endp >= self->endpoint_count) return -EINVAL;
    for(size_t c = 0; c < size; c++){
        if(thread_queue_send(&self->endpoints[endp]->queue, &data[c], timeout) < 0) return -1;
    }
    return 0;
}

int usbd_endpoint_read(struct usbd_device *self, unsigned int endp, uint8_t *data, size_t size, uint32_t timeout){
    if(endp >= self->endpoint_count) return -EINVAL;
    for(size_t c = 0; c < size; c++){
        if(thread_queue_recv(&self->endpoints[endp]->queue, &data[c], timeout) < 0) return -1;
    }
    return 0;
}

int usbd_endpoint_write_from_isr(struct usbd_device *self, unsigned int endp, const uint8_t *data, size_t size, int32_t *wake){
    if(endp >= self->endpoint_count) return -EINVAL;
    return 0;
}

int usbd_endpoint_read_from_isr(struct usbd_device *self, unsigned int endp, uint8_t *data, size_t size, int32_t *wake){
    if(endp >= self->endpoint_count) return -EINVAL;
    return (int)size;
}
size_t usbd_endpoint_space_available(struct usbd_device *self, unsigned int endp){
    if(endp >= self->endpoint_count) return 0;
    return (size_t)((long)self->endpoints[endp]->queue_size - thread_queue_length(&self->endpoints[endp]->queue));
}

size_t usbd_endpoint_data_available(struct usbd_device *self, unsigned int endp){
    if(endp >= self->endpoint_count) return 0;
    return (size_t)thread_queue_length(&self->endpoints[endp]->queue);
}

size_t usbd_endpoint_size(struct usbd_device *self, unsigned int endp){
    if(endp >= self->endpoint_count) return 0;
    struct usb_endpoint_descriptor *epd = self->endpoints[endp]->desc;
    return USB_WORD_HL(epd->wMaxPacketSizeH, epd->wMaxPacketSizeL);
}
*/
