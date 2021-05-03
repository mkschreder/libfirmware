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
* FILE ............... include/usb.h
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
#pragma once

#include "driver.h"
#include "thread/queue.h"
#include "thread/sem.h"

/* USB Descriptor Types */
#define USB_DESC_TYPE_DEVICE 1
#define USB_DESC_TYPE_CFG 2
#define USB_DESC_TYPE_STR 3
#define USB_DESC_TYPE_IFACE 4
#define USB_DESC_TYPE_EP 5
#define USB_DESC_TYPE_DEVICE_QR 6
#define USB_DESC_TYPE_OSPEED_CFG 7
#define USB_DESC_TYPE_IFACE_PWR 8
/* USB Device Classes */
#define USB_DEV_CLASS_RESERVED 0x00
#define USB_DEV_CLASS_AUDIO 0x01
#define USB_DEV_CLASS_COMM 0x02
#define USB_DEV_CLASS_HID 0x03
#define USB_DEV_CLASS_MONITOR 0x04
#define USB_DEV_CLASS_PHYSIC 0x05
#define USB_DEV_CLASS_POWER 0x06
#define USB_DEV_CLASS_PRINTER 0x07
#define USB_DEV_CLASS_STORAGE 0x08
#define USB_DEV_CLASS_HUB 0x09
#define USB_DEV_CLASS_VENDOR_SPEC 0xFF
/* Interface Class SubClass Codes */
#define USB_IF_SUBCLASS_ACM_COMM 0x02
#define USB_IF_SUBCLASS_CDC_DATA_IFACE 0x0A
#define USB_IF_SUBCLASS_CS_INTERFACE 0x24
#define USB_IF_SUBCLASS_CS_ENDPOINT 0x25

#define USB_LOBYTE(x) ((uint8_t)((x) & 0x00FF))
#define USB_HIBYTE(x) ((uint8_t)(((x) & 0xFF00) >> 8))
#define USB_WORD_HL(h, l) ((uint16_t)(((uint16_t)(h) << 8) | ((l) & 0xff)))

/* EP_TYPE: EndPoint Types */
#define USB_EP_TYPE_CONTROL 0x00        /* BULK EndPoint */
#define USB_EP_TYPE_ISOCHRONOUS 0x01     /* CONTROL EndPoint */
#define USB_EP_TYPE_BULK 0x02 /* ISOCHRONOUS EndPoint */
#define USB_EP_TYPE_INTERRUPT 0x03   /* INTERRUPT EndPoint */
#define USB_EP_ISO_NO_SYNC (0x0 << 2)
#define USB_EP_ISO_ASYNC (0x1 << 2)
#define USB_EP_ISO_ADAPTIVE (0x2 << 2)
#define USB_EP_ISO_SYNC (0x3 << 2)

struct __packed usb_device_descriptor {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bcdUSBL;
    uint8_t bcdUSBH;
    uint8_t bDeviceClass;
    uint8_t bDeviceSubClass;
    uint8_t bDeviceProtocol;
    uint8_t bMaxPacketSize0;
    uint8_t idVendorL;
    uint8_t idVendorH;
    uint8_t idProductL;
    uint8_t idProductH;
    uint8_t bcdDeviceL;
    uint8_t bcdDeviceH;
    uint8_t iManufacturer;
    uint8_t iProduct;
    uint8_t iSerialNumber;
    uint8_t bNumConfigurations;
};

struct __packed usb_descriptor_header {
    uint8_t bLength;
    uint8_t bDescriptorType;
};

struct __packed usb_config_descriptor {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t wTotalLengthL;
    uint8_t wTotalLengthH;
    uint8_t bNumInterfaces;
    uint8_t bConfigurationValue;
    uint8_t bConfigurationString;
    uint8_t bmAttributes;
    uint8_t bMaxPower;
};

struct __packed usb_interface_descriptor {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bInterfaceNumber;
    uint8_t bAlternateSetting;
    uint8_t bNumEndpoints;
    uint8_t bInterfaceClass;
    uint8_t bInterfaceSubClass;
    uint8_t bInterfaceProtocol;
    uint8_t iInterface;
};

struct __packed usb_endpoint_descriptor {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bEndpointAddress;
    uint8_t bmAttributes;
    uint8_t wMaxPacketSizeL;
    uint8_t wMaxPacketSizeH;
    uint8_t bInterval;
};

struct __packed usb_string_descriptor {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t data[];
};

struct __packed usb_setup_packet {
    union {
        uint16_t wRequestAndType;
        struct {
            uint8_t bmRequestType;
            uint8_t bRequest;
        };
    };
    union {
        uint16_t wValue;
        struct {
            uint8_t       wValueL;
            uint8_t       wValueH;
        };
    };
    union {
        uint16_t wIndex;
        struct {
            uint8_t       wIndexL;
            uint8_t       wIndexH;
        };
    };
    union {
        uint16_t wLength;
        struct {
            uint8_t            wLengthL;
            uint8_t            wLengthH;
        };
    };
};

typedef const struct usbd_device_ops ** usbd_device_t;

struct usbd_device_ops {
    int (*write)(usbd_device_t port, uint8_t ep, const void *ptr, size_t size, uint32_t timeout_ms);
	int (*read)(usbd_device_t port, uint8_t ep, void *ptr, size_t size, uint32_t timeout_ms);
};

#define usbd_recv(s, ep, b, sz, t) (*(s))->read(s, ep, b, sz, t)
#define usbd_send(s, ep, b, sz, t) (*(s))->write(s, ep, b, sz, t)

struct __packed usb_endpoint_message {
    // total size is determined by buffer_size in endpoint class
    uint16_t size; //! stored data size
    uint16_t cursor; //! cursor
    uint8_t data[]; //! data follows in memory
};

struct usb_endpoint {
    struct semaphore tx_ready;
    struct semaphore rx_ready;
    uint16_t buffer_size;
    struct usb_endpoint_descriptor *desc;
};

struct usbd_device {
    struct list_head list;
    const struct usbd_device_ops *ops;
    int fdt_node;

    const struct usb_device_descriptor *device_desc;
    const struct usb_config_descriptor *config_desc;
    const struct usb_string_descriptor **string_desc;
    uint8_t string_desc_count;

    struct usb_endpoint **endpoints;
    uint8_t endpoint_count;
};

void usbd_device_init(struct usbd_device *self, void *fdt, int fdt_node, const struct usbd_device_ops *ops);
int usbd_device_register(struct usbd_device *self);
usbd_device_t usbd_find(const char *dtb_path);
usbd_device_t usbd_find_by_node(int fdt_node);

void usb_endpoint_init(struct usb_endpoint *self, struct usb_endpoint_descriptor *desc);

