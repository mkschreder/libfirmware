/*
 * Copyright (C) 2017 Martin K. Schröder <mkschreder.uk@gmail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/

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

static const uint8_t _default_config_desc[] = {
    /*Configuration Descriptor*/
    0x09, /* bLength: Configuration Descriptor size */
    0x02, /* bDescriptorType: Configuration */
    67,   /* wTotalLength:no of returned bytes */
    0x00,
    0x02, /* bNumInterfaces: 2 interface */
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
    0x03, /* bmAttributes: Interrupt */
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
    0x02, /* bmAttributes: Bulk */
    64,   /* wMaxPacketSize: */
    0x00,
    0x00, /* bInterval: ignore for Bulk transfer */

    /*Endpoint OUT3 Descriptor*/
    0x07, /* bLength: Endpoint Descriptor size */
    0x05, /* bDescriptorType: Endpoint */
    0x03, /* bEndpointAddress */
    0x02, /* bmAttributes: Bulk */
    64,   /* wMaxPacketSize: */
    0,
    0x00 /* bInterval: ignore for Bulk transfer */
};

static const char *_default_vendor = "Default Vendor";
static const char *_default_product = "Default Product";

static struct usb_string_descriptor *_alloc_string_desc(const char *str){
    size_t dat_len = strlen(str) * 2;
    // limit string size so our descriptor does not become too large
    if(dat_len > 127 - 2) dat_len = 125;
    struct usb_string_descriptor *s = kzmalloc(sizeof(struct usb_string_descriptor) + dat_len);
    s->bLength = (uint8_t)(2 + dat_len);
    s->bDescriptorType = USB_DESC_TYPE_STR;
    uint8_t *data = (uint8_t*)s + sizeof(struct usb_string_descriptor);
    for(size_t c = 0; c < dat_len; c++){
        data[c * 2] = str[c];
        data[c * 2 + 1] = 0;
    }
    return s;
}

void usbd_device_init(struct usbd_device *self, void *fdt, int fdt_node, const struct usbd_device_ops *ops){
	memset(self, 0, sizeof(*self));
	INIT_LIST_HEAD(&self->list);
	self->fdt_node = fdt_node;
	self->ops = ops;

    // allocate descriptors and fill them in from fdt
    struct usb_device_descriptor *d = kzmalloc(sizeof(struct usb_device_descriptor));
    d->bLength = 18;
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

    struct usb_config_descriptor *cd = kzmalloc(sizeof(_default_config_desc));
    memcpy(cd, &_default_config_desc, sizeof(_default_config_desc));
    self->config_desc = cd;

    self->string_desc_count = 4;
    const struct usb_string_descriptor **sd = kzmalloc(sizeof(struct usb_string_descriptor*) * self->string_desc_count);

    // language
    struct usb_string_descriptor *s = kzmalloc(sizeof(struct usb_string_descriptor) + 2);
    s->bLength = 4;
    s->bDescriptorType = USB_DESC_TYPE_STR;
    uint8_t *data = (uint8_t*)s + sizeof(struct usb_string_descriptor);
    data[0] = 0x04;
    data[1] = 0x09;
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

