#pragma once

#include "driver.h"

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

#define USB_LOBYTE(x) ((uint8_t)(x & 0x00FF))
#define USB_HIBYTE(x) ((uint8_t)((x & 0xFF00) >> 8))

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

#define usbd_read(s, ep, b, sz, t) (*(s))->read(s, ep, b, sz, t)
#define usbd_write(s, ep, b, sz, t) (*(s))->write(s, ep, b, sz, t)

struct usbd_device {
    struct list_head list;
    const struct usbd_device_ops *ops;
    int fdt_node;

    const struct usb_device_descriptor *device_desc;
    const struct usb_config_descriptor *config_desc;
    const struct usb_string_descriptor **string_desc;
    uint8_t string_desc_count;
};

void usbd_device_init(struct usbd_device *self, void *fdt, int fdt_node, const struct usbd_device_ops *ops);
int usbd_device_register(struct usbd_device *self);
usbd_device_t usbd_find(const char *dtb_path);
usbd_device_t usbd_find_by_node(int fdt_node);

