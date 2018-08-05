#include <stm32f10x.h>
#include <stm32f10x_exti.h>
#include <stm32f10x_rcc.h>
//#include "usb-fs-core/usb_lib.h"
//#include "usb-fs-core/usb_istr.h"
#include "driver.h"

#define EPCOUNT 4
#define USB_BASE ((uint32_t)0x40005C00)
#define USB_PBUFFER ((uint32_t)0x40006000)
#define STRX 12
#define STTX 4
#define CTR_RX 0x8000
#define CTR_TX 0x80
#define CDC_CMD_PACKET_SIZE 8          /* Control Endpoint Packet size */
#define CDC_DATA_FS_CMD_PACKET_SIZE 16 /* Endpoint IN & OUT Packet size */
#define CDC_DATA_FS_MAX_PACKET_SIZE 64 /* Endpoint IN & OUT Packet size */
#define LANG_US (uint16_t)0x0409

#define DEVICE_VENDOR_ID 0x25AE
#define DEVICE_PRODUCT_ID 0x24AB
#define STRING_DT 3

/* EPxREG: EndPoint Registers Bit Definitions */
#define EP_CTR_RX 0x8000  /* Correct RX Transfer */
#define EP_DTOG_RX 0x4000 /* RX Data Toggle */
#define EP_STAT_RX 0x3000 /* RX Status */
#define EP_SETUP 0x0800   /* EndPoint Setup */
#define EP_TYPE 0x0600    /* EndPoint Type */
#define EP_KIND 0x0100    /* EndPoint Kind */
#define EP_CTR_TX 0x0080  /* Correct TX Transfer */
#define EP_DTOG_TX 0x0040 /* TX Data Toggle */
#define EP_STAT_TX 0x0030 /* TX Status */
#define EP_EA 0x000F      /* EndPoint Address */
#define EP_MASK (EP_CTR_RX | EP_SETUP | EP_TYPE | EP_KIND | EP_CTR_TX | EP_EA)

/* EP_STAT_TX: TX Status */
#define TX_DISABLE 0x0000 /* Disabled */
#define TX_STALL 0x0010   /* Stalled */
#define TX_NAK 0x0020     /* NAKed */
#define TX_VALID 0x0030   /* Valid */

/* EP_STAT_RX: RX Status */
#define RX_DISABLE 0x0000 /* Disabled */
#define RX_STALL 0x1000   /* Stalled */
#define RX_NAK 0x2000     /* NAKed */
#define RX_VALID 0x3000   /* Valid */

/* EP_TYPE: EndPoint Types */
#define EP_BULK 0x0000        /* BULK EndPoint */
#define EP_CONTROL 0x0200     /* CONTROL EndPoint */
#define EP_ISOCHRONOUS 0x0400 /* ISOCHRONOUS EndPoint */
#define EP_INTERRUPT 0x0600   /* INTERRUPT EndPoint */

/* bmRequestType.Type */
#define REQUEST_STANDARD 0
#define REQUEST_CLASS 1
#define REQUEST_VENDOR 2
#define REQUEST_RESERVED 3

/* USB Standard Request Codes */
#define USB_REQUEST_GET_STATUS 0
#define USB_REQUEST_CLEAR_FEATURE 1
#define USB_REQUEST_SET_FEATURE 3
#define USB_REQUEST_SET_ADDRESS 5
#define USB_REQUEST_GET_DESCRIPTOR 6
#define USB_REQUEST_SET_DESCRIPTOR 7
#define USB_REQUEST_GET_CONFIGURATION 8
#define USB_REQUEST_SET_CONFIGURATION 9
#define USB_REQUEST_GET_INTERFACE 10
#define USB_REQUEST_SET_INTERFACE 11
#define USB_REQUEST_SYNC_FRAME 12

/* USB Descriptor Types */
#define USB_DEVICE_DESC_TYPE 1
#define USB_CFG_DESC_TYPE 2
#define USB_STR_DESC_TYPE 3
#define USB_IFACE_DESC_TYPE 4
#define USB_EP_DESC_TYPE 5
#define USB_DEVICE_QR_DESC_TYPE 6
#define USB_OSPEED_CFG_DESC_TYPE 7
#define USB_IFACE_PWR_DESC_TYPE 8
/* USB Device Classes */
#define USB_RESERVED 0x00
#define USB_AUDIO 0x01
#define USB_COMM 0x02
#define USB_HID 0x03
#define USB_MONITOR 0x04
#define USB_PHYSIC 0x05
#define USB_POWER 0x06
#define USB_PRINTER 0x07
#define USB_STORAGE 0x08
#define USB_HUB 0x09
#define USB_VENDOR_SPEC 0xFF
/* Interface Class SubClass Codes */
#define USB_ACM_COMM 0x02

#define CDC_DATA_IFACE 0x0A
#define CS_INTERFACE 0x24
#define CS_ENDPOINT 0x25

/* CDC */
#define USB_CDC_CONFIG_DESC_SIZ 67
#define CDC_CMD_EP 0x81 /* EP2 for CDC commands */
#define CDC_IN_EP 0x82  /* EP1 for data IN */
#define CDC_OUT_EP 0x02 /* EP1 for data OUT */

#define USB_DEVICE_CDC_REQUEST_SEND_ENCAPSULATED_COMMAND \
    (0x00) /*!< The CDC class request code for SEND_ENCAPSULATED_COMMAND. */
#define USB_DEVICE_CDC_REQUEST_GET_ENCAPSULATED_RESPONSE \
    (0x01)                                               /*!< The CDC class request code for GET_ENCAPSULATED_RESPONSE. */
#define USB_DEVICE_CDC_REQUEST_SET_COMM_FEATURE (0x02)   /*!< The CDC class request code for SET_COMM_FEATURE. */
#define USB_DEVICE_CDC_REQUEST_GET_COMM_FEATURE (0x03)   /*!< The CDC class request code for GET_COMM_FEATURE. */
#define USB_DEVICE_CDC_REQUEST_CLEAR_COMM_FEATURE (0x04) /*!< The CDC class request code for CLEAR_COMM_FEATURE. */
#define USB_DEVICE_CDC_REQUEST_SET_AUX_LINE_STATE (0x10) /*!< The CDC class request code for SET_AUX_LINE_STATE. */
#define USB_DEVICE_CDC_REQUEST_SET_HOOK_STATE (0x11)     /*!< The CDC class request code for SET_HOOK_STATE. */
#define USB_DEVICE_CDC_REQUEST_PULSE_SETUP (0x12)        /*!< The CDC class request code for PULSE_SETUP. */
#define USB_DEVICE_CDC_REQUEST_SEND_PULSE (0x13)         /*!< The CDC class request code for SEND_PULSE. */
#define USB_DEVICE_CDC_REQUEST_SET_PULSE_TIME (0x14)     /*!< The CDC class request code for SET_PULSE_TIME. */
#define USB_DEVICE_CDC_REQUEST_RING_AUX_JACK (0x15)      /*!< The CDC class request code for RING_AUX_JACK. */
#define USB_DEVICE_CDC_REQUEST_SET_LINE_CODING (0x20)    /*!< The CDC class request code for SET_LINE_CODING. */
#define USB_DEVICE_CDC_REQUEST_GET_LINE_CODING (0x21)    /*!< The CDC class request code for GET_LINE_CODING. */
#define USB_DEVICE_CDC_REQUEST_SET_CONTROL_LINE_STATE \
    (0x22)                                                /*!< The CDC class request code for SET_CONTROL_LINE_STATE. */
#define USB_DEVICE_CDC_REQUEST_SEND_BREAK (0x23)          /*!< The CDC class request code for SEND_BREAK. */
#define USB_DEVICE_CDC_REQUEST_SET_RINGER_PARAMS (0x30)   /*!< The CDC class request code for SET_RINGER_PARAMS. */
#define USB_DEVICE_CDC_REQUEST_GET_RINGER_PARAMS (0x31)   /*!< The CDC class request code for GET_RINGER_PARAMS. */
#define USB_DEVICE_CDC_REQUEST_SET_OPERATION_PARAM (0x32) /*!< The CDC class request code for SET_OPERATION_PARAM. */
#define USB_DEVICE_CDC_REQUEST_GET_OPERATION_PARAM (0x33) /*!< The CDC class request code for GET_OPERATION_PARAM. */
#define USB_DEVICE_CDC_REQUEST_SET_LINE_PARAMS (0x34)     /*!< The CDC class request code for SET_LINE_PARAMS. */
#define USB_DEVICE_CDC_REQUEST_GET_LINE_PARAMS (0x35)     /*!< The CDC class request code for GET_LINE_PARAMS. */
#define USB_DEVICE_CDC_REQUEST_DIAL_DIGITS (0x36)         /*!< The CDC class request code for DIAL_DIGITS. */
#define USB_DEVICE_CDC_REQUEST_SET_UNIT_PARAMETER (0x37)  /*!< The CDC class request code for SET_UNIT_PARAMETER. */
#define USB_DEVICE_CDC_REQUEST_GET_UNIT_PARAMETER (0x38)  /*!< The CDC class request code for GET_UNIT_PARAMETER. */
#define USB_DEVICE_CDC_REQUEST_CLEAR_UNIT_PARAMETER \
    (0x39) /*!< The CDC class request code for CLEAR_UNIT_PARAMETER. */
#define USB_DEVICE_CDC_REQUEST_SET_ETHERNET_MULTICAST_FILTERS \
    (0x40) /*!< The CDC class request code for SET_ETHERNET_MULTICAST_FILTERS. */
#define USB_DEVICE_CDC_REQUEST_SET_ETHERNET_POW_PATTER_FILTER \
    (0x41) /*!< The CDC class request code for SET_ETHERNET_POW_PATTER_FILTER. */
#define USB_DEVICE_CDC_REQUEST_GET_ETHERNET_POW_PATTER_FILTER \
    (0x42) /*!< The CDC class request code for GET_ETHERNET_POW_PATTER_FILTER. */
#define USB_DEVICE_CDC_REQUEST_SET_ETHERNET_PACKET_FILTER \
    (0x43) /*!< The CDC class request code for SET_ETHERNET_PACKET_FILTER. */
#define USB_DEVICE_CDC_REQUEST_GET_ETHERNET_STATISTIC \
    (0x44)                                                /*!< The CDC class request code for GET_ETHERNET_STATISTIC. */
#define USB_DEVICE_CDC_REQUEST_SET_ATM_DATA_FORMAT (0x50) /*!< The CDC class request code for SET_ATM_DATA_FORMAT. */
#define USB_DEVICE_CDC_REQUEST_GET_ATM_DEVICE_STATISTICS \
    (0x51)                                               /*!< The CDC class request code for GET_ATM_DEVICE_STATISTICS. */
#define USB_DEVICE_CDC_REQUEST_SET_ATM_DEFAULT_VC (0x52) /*!< The CDC class request code for SET_ATM_DEFAULT_VC. */
#define USB_DEVICE_CDC_REQUEST_GET_ATM_VC_STATISTICS \
    (0x53) /*!< The CDC class request code for GET_ATM_VC_STATISTICS. */
#define USB_DEVICE_CDC_REQUEST_MDLM_SPECIFIC_REQUESTS_MASK \
    (0x7F) /*!< The CDC class request code for MDLM_SPECIFIC_REQUESTS_MASK. */

//#define RXCNT(bsize, nblock) (uint16_t)(((bsize & 1) << 15) | ((nblock / 2 & 0x1F) << 10))
#define LOBYTE(x) ((uint8_t)(x & 0x00FF))
#define HIBYTE(x) ((uint8_t)((x & 0xFF00) >> 8))

#define LOG_LENGTH 50
#define LOG_DATA_LENGTH 10
#define LOG_OP_RESET 1
#define LOG_OP_GET_DESC_RX 2
#define LOG_OP_GET_DESC_TX 3
#define LOG_OP_GET_STATUS_TX 4
#define LOG_OP_SET_ADDRESS_RX 5
#define LOG_OP_GET_CLASS_DATA 6

typedef struct __packed _USB_STRING_DESCRIPTOR_ {
    uint8_t bLength;
    uint8_t bDescriptorType;
} USB_STR_DESCRIPTOR;

typedef struct __packed _USB_DEVICE_DESCRIPTOR_ {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint16_t bcdUSB;
    uint8_t  bDeviceClass;
    uint8_t  bDeviceSubClass;
    uint8_t  bDeviceProtocol;
    uint8_t  bMaxPacketSize0;
    uint16_t idVendor;
    uint16_t idProduct;
    uint16_t bcdDevice;
    uint8_t  iManufacturer;
    uint8_t  iProduct;
    uint8_t  iSerialNumber;
    uint8_t  bNumConfigurations;
} USB_DEVICE_DESCRIPTOR;

typedef struct __packed {
    uint32_t EPR[8];
    uint32_t RESERVED[8];
    uint32_t CNTR;
    uint32_t ISTR;
    uint32_t FNR;
    uint32_t DADDR;
    uint32_t BTABLE;
} USB_TypeDef;

typedef struct __packed {
    uint16_t Value;
    uint16_t _res;
} USBLIB_PBElement;

typedef struct __packed {
    USBLIB_PBElement TX_Address;
    USBLIB_PBElement TX_Count;
    USBLIB_PBElement RX_Address;
    USBLIB_PBElement RX_Count;
} USBLIB_EPBuf;

typedef struct __packed {
    uint16_t  Number;       // EP number
    uint16_t  Type;         // EP Type
    uint8_t   TX_Max;       // Max TX EP Buffer
    uint8_t   RX_Max;       // Max RT EP Buffer
    volatile uint16_t *pTX_BUFF;     // TX Buffer pointer
    uint32_t  lTX;          // TX Data length
    volatile uint16_t *pRX_BUFF;     // RX Buffer pointer
    uint32_t  lRX;          // RX Data length
    bool send_zlp;
} USBLIB_EPData;

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

typedef struct __packed
{
    uint8_t epidx;
    uint8_t Operation;
    uint8_t Data[LOG_DATA_LENGTH];
    uint8_t Length;
} USBLIB_Log;

typedef struct __packed {
    uint8_t   Size;
    uint8_t   DescriptorType;
    uint16_t *String;
} USBLIB_StringDesc;

typedef struct __packed {
    uint32_t baudRate;
    uint8_t  charFormat;
    uint8_t  parityType;
    uint8_t  dataBits;
} USBLIB_LineCoding;


#define USB_REQ_DIR_IN   (1 << 7)
#define USB_REQ_DIR_OUT  (0 << 7)
#define USB_REQ_TYPE_STD (0 << 5)
#define USB_REQ_TYPE_CLS (1 << 5)
#define USB_REQ_TYPE_VND (2 << 5)
#define USB_REQ_RCP_DEV  (0)
#define USB_REQ_RCP_IFACE (1)
#define USB_REQ_RCP_ENDP  (2)
#define USB_REQ_RCP_OTHER (3)

#define USB_REQ(REQUEST, TYPE) (uint16_t)(((REQUEST) << 8) | ((TYPE) & 0xFF))

#define USB_TR_DIR_OUT 0
#define USB_TR_DIR_IN 1

enum {
    USB_FLAG_ATTACHED = (1 << 0),
    USB_FLAG_POWERED = (1 << 1),
    USB_FLAG_RESET = (1 << 2),
    USB_FLAG_ADDRESSED = (1 << 3),
    USB_FLAG_CONFIGURED = (1 << 4),
    USB_FLAG_SUSPENDED = (1 << 5)
};

#define USB_MASK_ATTACHED (USB_FLAG_ATTACHED)
#define USB_MASK_POWERED (USB_FLAG_ATTACHED | USB_FLAG_POWERED)
#define USB_MASK_IDLE (USB_FLAG_ATTACHED | USB_FLAG_POWERED | USB_FLAG_RESET)
#define USB_MASK_ADDRESSED (USB_FLAG_ATTACHED | USB_FLAG_POWERED | USB_FLAG_RESET | USB_FLAG_ADDRESSED)
#define USB_MASK_CONFIGURED (USB_FLAG_ATTACHED | USB_FLAG_POWERED | USB_FLAG_RESET | USB_FLAG_ADDRESSED | USB_FLAG_CONFIGURED)
#define USB_MASK_SUSPENDED (USB_FLAG_ATTACHED | USB_FLAG_POWERED | USB_FLAG_SUSPENDED)

#define USB_IS_ATTACHED(dev) ((dev->flags & USB_MASK_ATTACHED) == USB_MASK_ATTACHED)

static volatile USB_TypeDef *USB = (USB_TypeDef *)USB_BASE;
static volatile USBLIB_EPBuf *_endp_buffers = (USBLIB_EPBuf*)USB_PBUFFER;
static USBLIB_EPData _endp_config[EPCOUNT] =
{
	{0, EP_CONTROL, 8, 8, 0, 0, 0, 0, 0},
	{1, EP_INTERRUPT, 64, 64, 0, 0, 0, 0, 0},
	{2, EP_BULK, 64, 64, 0, 0, 0, 0, 0},  //IN  (Device -> Host)
	{3, EP_BULK, 64, 64, 0, 0, 0, 0, 0} //OUT (Host   -> Device)
};

struct stm32_usb {
	uint8_t addr;
	uint8_t config;
	uint16_t status;
};

static struct stm32_usb *_devices[1] = {0};
//usb_setup_packet *  SetupPacket;

USBLIB_LineCoding lineCoding = {115200, 0, 0, 8};

static const uint8_t USB_DEVICE_DESC[] =
    {
        (uint8_t)18,                        //    bLength
        (uint8_t)USB_DEVICE_DESC_TYPE,      //    bDescriptorType
        (uint8_t)0x10,                      //    bcdUSB
        (uint8_t)0x01,                      //    bcdUSB
        (uint8_t)0,                  //    bDeviceClass
        (uint8_t)0,                         //    bDeviceSubClass
        (uint8_t)0,                         //    bDeviceProtocol
        (uint8_t)8,                         //    bMaxPacketSize0
        (uint8_t)LOBYTE(DEVICE_VENDOR_ID),  //    idVendor
        (uint8_t)HIBYTE(DEVICE_VENDOR_ID),  //    idVendor
        (uint8_t)LOBYTE(DEVICE_PRODUCT_ID), //    idProduct
        (uint8_t)HIBYTE(DEVICE_PRODUCT_ID), //    idProduct
        (uint8_t)0x00,                      //    bcdDevice
        (uint8_t)0x01,                      //    bcdDevice
        (uint8_t)1,                         //    iManufacturer
        (uint8_t)2,                         //    iProduct
        (uint8_t)3,                         //    iSerialNumbert
        (uint8_t)1                          //    bNumConfigurations
};

const uint8_t USB_DEVICE_QR_DESC[] = {
        (uint8_t)10,                        //    bLength
        (uint8_t)0x06,      //    bDescriptorType
        (uint8_t)0x00,                      //    bcdUSB
        (uint8_t)0x02,                      //    bcdUSB
        (uint8_t)USB_COMM,                  //    bDeviceClass
        (uint8_t)0,                         //    bDeviceSubClass
        (uint8_t)0,                         //    bDeviceProtocol
        (uint8_t)8,                         //    bMaxPacketSize0
        (uint8_t)1,                         //    bNumConfigurations
        (uint8_t)0                          // reserved
};

static const uint8_t USBD_CDC_CFG_DESCRIPTOR[] =
    {
        /*Configuration Descriptor*/
        (uint8_t)0x09, /* bLength: Configuration Descriptor size */
        (uint8_t)0x02, /* bDescriptorType: Configuration */
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

void _stm32_usb_set_tx_status(uint8_t epidx, uint16_t Stat)
{
	if(epidx >= EPCOUNT) return;
    register uint16_t val = (uint16_t)USB->EPR[epidx];
    USB->EPR[epidx]         = (val ^ (Stat & EP_STAT_TX)) & (EP_MASK | EP_STAT_TX);
}

void _stm32_usb_set_rx_status(uint8_t epidx, uint16_t Stat)
{
	if(epidx >= EPCOUNT) return;
    register uint16_t val = (uint16_t)USB->EPR[epidx];
    USB->EPR[epidx]         = (val ^ (Stat & EP_STAT_RX)) & (EP_MASK | EP_STAT_RX);
}


void _stm32_usb_reset(void){
    uint16_t Addr = sizeof(USBLIB_EPBuf) * EPCOUNT;
    for (uint8_t i = 0; i < EPCOUNT; i++) {
        _endp_buffers[i].TX_Address.Value = Addr;
        _endp_buffers[i].TX_Count.Value   = 0;
        Addr = (uint16_t)(Addr + _endp_config[i].TX_Max);
        _endp_buffers[i].RX_Address.Value = Addr;

        // compute number of rx blocks
        if (_endp_config[i].RX_Max > 62)
            _endp_buffers[i].RX_Count.Value = 0x8000 | (uint16_t)((_endp_config[i].RX_Max / 64) << 10);
        else
            _endp_buffers[i].RX_Count.Value = (uint16_t)((_endp_config[i].RX_Max / 2) << 10);

        Addr = (uint16_t)(Addr + _endp_config[i].RX_Max);

        USB->EPR[i] = (_endp_config[i].Number | _endp_config[i].Type);

    }

    // enable all other interrupts and enable the usb peripheral
    USB->CNTR   = USB_CNTR_CTRM | USB_CNTR_RESETM | USB_CNTR_SUSPM | USB_CNTR_ERRM;
    USB->ISTR   = 0x00;
    USB->BTABLE = 0x00;
    USB->DADDR  = USB_DADDR_EF;

    // start receiving on ep 0
    _stm32_usb_set_rx_status(0, RX_VALID);
}

void _user_to_pma(uint16_t wPMABufAddr, uint8_t *pbUsrBuf, uint16_t wNBytes)
{
  uint32_t n = (uint32_t)((wNBytes + 1) >> 1);   /* n = (wNBytes + 1) / 2 */
  uint32_t i, temp1, temp2;
  uint16_t *pdwVal;
  pdwVal = (uint16_t *)((uint32_t)(wPMABufAddr * 2) + USB_PBUFFER);
  for (i = n; i != 0; i--)
  {
    temp1 = (uint16_t) * pbUsrBuf;
    pbUsrBuf++;
    temp2 = temp1 | (uint16_t)((uint16_t) * pbUsrBuf << 8);
    *pdwVal++ = (uint16_t)temp2;
    pdwVal++;
    pbUsrBuf++;
  }
}

void _pma_to_user(uint8_t *pbUsrBuf, uint16_t wPMABufAddr, uint16_t wNBytes)
{
  uint32_t n = (uint32_t)((wNBytes + 1) >> 1);/* /2*/
  uint32_t i;
  uint32_t *pdwVal;
  pdwVal = (uint32_t *)((uint32_t)(wPMABufAddr * 2) + USB_PBUFFER);
  for (i = n; i != 0; i--)
  {
    *(uint16_t*)pbUsrBuf++ = (uint16_t)(*pdwVal);
    *pdwVal = 0;
    pdwVal++;
    pbUsrBuf++;
  }
}


static size_t _stm32_usb_pma_read(uint8_t epidx){
    uint16_t count = (uint16_t)(_endp_buffers[epidx].RX_Count.Value & 0x3FF);
    if(count > _endp_config[epidx].RX_Max) count = _endp_config[epidx].RX_Max;
    _endp_config[epidx].lRX = count;
    _pma_to_user((uint8_t*)_endp_config[epidx].pRX_BUFF, _endp_buffers[epidx].RX_Address.Value, count);
    /*
    uint32_t *pma = (uint32_t *)(USB_PBUFFER + (uint32_t)(_endp_buffers[epidx].RX_Address.Value * 2));
    uint16_t *buf = (uint16_t *)_endp_config[epidx].pRX_BUFF;
    for (uint8_t i = 0; i < count; i++) {
        *(volatile uint16_t *)buf = *(volatile uint16_t *)pma;
        buf++;
        pma++;
    }
    */
	return count;
}

static void _stm32_usb_pma_write(uint8_t epidx){
    uint16_t count = (uint16_t)(_endp_config[epidx].lTX <= _endp_config[epidx].TX_Max) ? (uint16_t)_endp_config[epidx].lTX : (uint16_t)_endp_config[epidx].TX_Max;
    _user_to_pma(_endp_buffers[epidx].TX_Address.Value, (uint8_t*)_endp_config[epidx].pTX_BUFF, count);
    /*
    uint32_t *pma = (uint32_t *)(USB_PBUFFER + (uint32_t)(_endp_buffers[epidx].TX_Address.Value * 2));
    uint16_t *buf = _endp_config[epidx].pTX_BUFF;
    for (uint8_t i = 0; i < (count + 1) / 2; i++) {
        *(volatile uint32_t *)pma = *(volatile uint16_t*)buf;
        pma++;
        buf++;
    }
    */
    volatile const char *b = (volatile const char*)_endp_config[epidx].pTX_BUFF;
    printk_isr("TX %d %02x%02x%02x%02x%02x%02x%02x%02x\n", count, b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7]);

    _endp_config[epidx].lTX -= count;
    _endp_config[epidx].pTX_BUFF += (count + 1) / 2;

    _endp_buffers[epidx].TX_Count.Value = count;
}

void _stm32_usb_send(uint8_t epidx, uint16_t *buf, uint16_t count){
    // setup buffers which will then be handled inside interrupt
    _endp_config[epidx].lTX      = count;
    _endp_config[epidx].pTX_BUFF = buf;
    _endp_config[epidx].send_zlp = count && (count % _endp_config[epidx].TX_Max) == 0;
    if (count > 0) {
        _stm32_usb_pma_write(epidx);
    } else {
        _endp_buffers[epidx].TX_Count.Value = 0;
    }
    //_stm32_usb_set_rx_status(0, RX_NAK);
    _stm32_usb_set_tx_status(epidx, TX_VALID);
}

static void _stm32_usb_send_descriptor(int type, uint8_t index, uint16_t length){
    //printk_isr("GD %d %d %d\n", type, index, length);
    switch (type) {
    case USB_DEVICE_DESC_TYPE:
        _stm32_usb_send(0, (uint16_t *)&USB_DEVICE_DESC[0], (length < sizeof(USB_DEVICE_DESC))?length:sizeof(USB_DEVICE_DESC));
        break;
    case USB_CFG_DESC_TYPE: {
        //static uint16_t t[2] = {0xdead, 0xbeef};
        //_stm32_usb_send(0, (uint16_t *)t, 4);
        _stm32_usb_send(0, (uint16_t *)USBD_CDC_CFG_DESCRIPTOR, (length < sizeof(USBD_CDC_CFG_DESCRIPTOR))?length:sizeof(USBD_CDC_CFG_DESCRIPTOR));
    } break;
    case USB_DEVICE_QR_DESC_TYPE:
        _stm32_usb_send(0, (uint16_t *)&USB_DEVICE_QR_DESC[0], sizeof(USB_DEVICE_QR_DESC));
        break;
    case USB_STR_DESC_TYPE: {
        static const struct descriptor {
                uint8_t  bLength;
                uint8_t  bDescriptorType;
                uint8_t  bString[6];
        } desc[] = {
            {0x04, 0x03, {0x4, 0x09}},
            // vendor
            {0x06, 0x03, {'M', 0, 'S', 0}},
            // product
            {0x06, 0x03, {'M', 0, 'S', 0}},
            // serial
            {0x06, 0x03, {'M', 0, 'S', 0}},
            // cdc
            {0x06, 0x03, {'M', 0, 'S', 0}},
            // cdcdata
            {0x06, 0x03, {'M', 0, 'S', 0}}
        };

        if(index >= (sizeof(desc) / sizeof(desc[0])))  {
            _stm32_usb_send(0, (uint16_t*)&desc[1], 6);
        } else {
            const struct descriptor *pSTR = &desc[index];
            _stm32_usb_send(0, (uint16_t*)pSTR, pSTR->bLength);
        }
    } break;
    default:
        _stm32_usb_send(0, 0, 0);
        break;
    }
}

static int _stm32_usb_handle_ep0_setup(struct stm32_usb *self, volatile struct usb_setup_packet *packet, uint16_t epr){
    switch (packet->wRequestAndType) {
    case USB_REQ(0x00, USB_REQ_DIR_IN | USB_REQ_TYPE_STD | USB_REQ_RCP_DEV):
    case USB_REQ(0x00, USB_REQ_DIR_IN | USB_REQ_TYPE_STD | USB_REQ_RCP_IFACE):
        _stm32_usb_send(0, &self->status, 2);
        break;
    case USB_REQ(0x06, USB_REQ_DIR_IN | USB_REQ_TYPE_STD | USB_REQ_RCP_DEV):
    case USB_REQ(0x06, USB_REQ_DIR_IN | USB_REQ_TYPE_STD | USB_REQ_RCP_IFACE):
        _stm32_usb_send_descriptor(packet->wValueH, packet->wValueL, packet->wLength);
        break;
    case USB_REQ(0x05, USB_REQ_DIR_OUT | USB_REQ_TYPE_STD | USB_REQ_RCP_DEV):
        self->addr = (uint8_t)(packet->wValueL & 0x7F);
        _stm32_usb_send(0, 0, 0);
        break;
    case USB_REQ(USB_REQUEST_GET_CONFIGURATION, USB_REQ_DIR_IN | USB_REQ_TYPE_STD | USB_REQ_RCP_DEV): {
        _stm32_usb_send(0, (uint16_t*)&self->config, 1);
    } break;
    case USB_REQ(USB_REQUEST_SET_CONFIGURATION, USB_REQ_DIR_OUT | USB_REQ_TYPE_STD | USB_REQ_RCP_DEV):
        self->config = packet->wValueL;
        _stm32_usb_send(0, 0, 0);
        break;
    default:
        //_stm32_usb_set_rx_status(0, RX_VALID);
        //_stm32_usb_set_rx_status(epidx, RX_VALID);
        //_stm32_usb_send(0, 0, 0);
        //_stm32_usb_send(0, (uint16_t*)packet, 4);
        //_stm32_usb_send(0, (uint16_t*)&count, 2);
        //_stm32_usb_set_rx_status(0, RX_STALL);
        //_stm32_usb_set_tx_status(0, TX_STALL);
        break;
    }
    return 0;
}

void _stm32_usb_handle_ep_request(struct stm32_usb *self, int dir, uint8_t epidx){
    if(!self) return;
    uint16_t epr  = (uint16_t)USB->EPR[epidx];
    if (epidx == 0) {
        //_stm32_usb_set_tx_status(epidx, TX_NAK);
        //_stm32_usb_set_rx_status(epidx, RX_NAK);

        if(dir == 0){
            if (epr & EP_CTR_TX) {
                USB->EPR[epidx] = epr & EP_MASK & ~EP_CTR_TX;
                // transmission has completed
                if (self->addr) {
                    USB->DADDR = (USB->DADDR & 0x0080) | self->addr;
                    self->addr = 0;
                }

                // send more data if more data is still to be sent
                if (_endp_config[epidx].lTX) {
                    _stm32_usb_pma_write(epidx);
                    _stm32_usb_set_tx_status(epidx, TX_VALID);
                } else if(_endp_config[epidx].send_zlp){
                    _stm32_usb_send(0, 0, 0);
                    _endp_config[epidx].send_zlp = false;
                } else {
                    _stm32_usb_set_tx_status(epidx, TX_NAK);
                    //_stm32_usb_set_tx_status(epidx, TX_STALL);
                    //_stm32_usb_set_rx_status(0, RX_VALID);
                }
            } else if(epr & EP_CTR_RX){
            }
        } else {
            int len = (int)_stm32_usb_pma_read(epidx);
            if (epr & USB_EP0R_SETUP) {
                USB->EPR[epidx] = epr & EP_MASK & ~EP_CTR_RX;
                volatile struct usb_setup_packet *packet = (volatile struct usb_setup_packet *)_endp_config[0].pRX_BUFF;
                //volatile const char *b = (volatile const char*)packet;
                //printk_isr("S %d %02x%02x\n", len, b[6], b[7]);
                //printk_isr("S %d %02x%02x%02x%02x%02x%02x%02x%02x\n", len, b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7]);
                //printk_isr("SUP %d %02x %02x %04x %04x %04x\n", len, packet->bmRequestType, packet->bRequest, packet->wValue, packet->wIndex, packet->wLength);
                _stm32_usb_handle_ep0_setup(self, packet, epr);
                //_stm32_usb_set_rx_status(0, RX_VALID);
            } else if (epr & EP_CTR_RX) {
                USB->EPR[epidx] = epr & EP_MASK & ~EP_CTR_RX;
                printk_isr("DAT %d\n", len);
                //_stm32_usb_set_tx_status(epidx, TX_STALL);
                //_stm32_usb_send(0, 0, 0);
                //_stm32_usb_send(0, &self->status, 1);
            }
        }
    } else {
        // handle application endpoint
    }
    _stm32_usb_set_rx_status(epidx, RX_VALID);
}

void USB_LP_CAN1_RX0_IRQHandler(){
	struct stm32_usb *self = _devices[0];

    uint16_t stat = (uint16_t)USB->ISTR;
    if (stat & USB_ISTR_RESET) { // Reset
        printk_isr("URST\n");
        _stm32_usb_reset();
    }
    if (stat & USB_ISTR_PMAOVR) {
        USB->ISTR &= (uint32_t)~USB_ISTR_PMAOVR;
    }
    if (stat & USB_ISTR_SUSP) {
        USB->ISTR &= (uint16_t)~USB_ISTR_SUSP;
    }
    if (stat & USB_ISTR_ERR) {
        USB->ISTR &= (uint32_t)~USB_ISTR_ERR;
    }
    if (stat & USB_ISTR_WKUP) {
        USB->ISTR &= (uint32_t)~USB_ISTR_WKUP;
    }
    if (stat & USB_ISTR_SOF) {
        USB->ISTR &= (uint32_t)~USB_ISTR_SOF;
    }
    if (stat & USB_ISTR_ESOF) {
        USB->ISTR &= (uint32_t)~USB_ISTR_ESOF;
    }

    //while((stat = (uint16_t)USB->ISTR) & USB_ISTR_CTR) { //Handle data on EP
    while((stat = (uint16_t)USB->ISTR) & USB_ISTR_CTR){
        // flags can only be written as zero so we can clear all flags like this
        USB->ISTR = 0;
		uint8_t epidx = stat & USB_ISTR_EP_ID;
        uint8_t dir = !!(stat & USB_ISTR_DIR);
        _stm32_usb_handle_ep_request(self, dir, epidx);
    }
}

void USBLIB_Transmit(uint16_t *Data, uint16_t Length, uint16_t Timeout)
{
    //    while (USBEP[2] & )
    //_stm32_usb_send(2, Data, Length);
}

void uUSBLIB_DataReceivedHandler(uint16_t *Data, uint16_t Length)
{
    /* NOTE: This function Should not be modified, when the callback is needed,
       the uUSBLIB_DataReceivedHandler could be implemented in the user file
    */
}

void uUSBLIB_LineStateHandler(uint16_t LineState)
{
    /* NOTE: This function Should not be modified, when the callback is needed,
       the uUSBLIB_LineStateHandler could be implemented in the user file
    */
}
void USBWakeUp_IRQHandler(void){
	EXTI_ClearITPendingBit(EXTI_Line18);
}

static int _stm32_usb_probe(void *fdt, int fdt_node){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);

	RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);

	for(int i = 0; i < EPCOUNT; i++){
		_endp_config[i].pRX_BUFF = (uint16_t *)kzmalloc((size_t)(_endp_config[i].RX_Max * 2));
	}

    _devices[0] = kzmalloc(sizeof(struct stm32_usb));

    _stm32_usb_reset();

    USB->CNTR   = USB_CNTR_FRES; /* Force USB Reset */
    USB->BTABLE = 0;
    USB->DADDR  = 0;
    USB->ISTR   = 0;

	NVIC_InitTypeDef nvic;
	nvic.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 2;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);

    // clear reset bit and enable reset interrupt
    USB->CNTR   = USB_CNTR_RESETM;

	printk("USB: ok\n");
	return 0;
}

static int _stm32_usb_remove(void *fdt, int fdt_node){
	return 0;
}

DEVICE_DRIVER(stm32_usb, "st,stm32_usb", _stm32_usb_probe, _stm32_usb_remove)
