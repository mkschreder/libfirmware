/**
 * This is a usb interface driver for the HS and FS device side interface. In
 * unconfigured state the interface is started with NAK on both TX and RX lines
 * on all endpoints. The interface needs to be configured with callbacks and
 * descriptors using a specific interface device driver which will handle
 * communication with the host.
 */

#include <errno.h>

#include <stm32f10x.h>
#include <stm32f10x_exti.h>
#include <stm32f10x_rcc.h>

#include <libfdt/libfdt.h>

#include "driver.h"
#include "serial.h"
#include "queue.h"
#include "usb.h"

#define USB_BASE ((uint32_t)0x40005C00)
#define USB_PBUFFER ((uint32_t)0x40006000)
#define STRX 12
#define STTX 4
#define CTR_RX 0x8000
#define CTR_TX 0x80
#define LANG_US (uint16_t)0x0409
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
} USB_BufferDescElem;

typedef struct __packed {
    USB_BufferDescElem TX_Address;
    USB_BufferDescElem TX_Count;
    USB_BufferDescElem RX_Address;
    USB_BufferDescElem RX_Count;
} USB_BufferDesc;

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

static USBLIB_EPData _endp_config[] =
{
	{0, EP_CONTROL, 8, 8, 0, 0, 0, 0, 0},
	{1, EP_INTERRUPT, 64, 64, 0, 0, 0, 0, 0},
	{2, EP_BULK, 64, 64, 0, 0, 0, 0, 0},  //IN  (Device -> Host)
	{3, EP_BULK, 64, 64, 0, 0, 0, 0, 0} //OUT (Host   -> Device)
};

#define EPCOUNT (sizeof(_endp_config) / sizeof(_endp_config[0]))

struct stm32_usb {
    struct usbd_device dev;

	uint8_t addr;
	uint8_t current_config;
	uint16_t status;

    volatile USB_TypeDef *hw;
    volatile USB_BufferDesc *ep_desc;
};

static struct stm32_usb *_devices[1] = {0};

void _stm32_usb_set_tx_status(struct stm32_usb *self, uint8_t epidx, uint16_t Stat)
{
	if(epidx >= EPCOUNT) return;
    register uint16_t val = (uint16_t)self->hw->EPR[epidx];
    self->hw->EPR[epidx]         = (val ^ (Stat & EP_STAT_TX)) & (EP_MASK | EP_STAT_TX);
}

void _stm32_usb_set_rx_status(struct stm32_usb *self, uint8_t epidx, uint16_t Stat)
{
	if(epidx >= EPCOUNT) return;
    register uint16_t val = (uint16_t)self->hw->EPR[epidx];
    self->hw->EPR[epidx]         = (val ^ (Stat & EP_STAT_RX)) & (EP_MASK | EP_STAT_RX);
}


void _stm32_usb_reset(struct stm32_usb *self){
    uint16_t Addr = sizeof(USB_BufferDesc) * EPCOUNT;
    for (uint8_t i = 0; i < EPCOUNT; i++) {
        self->ep_desc[i].TX_Address.Value = Addr;
        self->ep_desc[i].TX_Count.Value   = 0;
        Addr = (uint16_t)(Addr + _endp_config[i].TX_Max);
        self->ep_desc[i].RX_Address.Value = Addr;

        // compute number of rx blocks
        if (_endp_config[i].RX_Max > 62)
            self->ep_desc[i].RX_Count.Value = 0x8000 | (uint16_t)((_endp_config[i].RX_Max / 64) << 10);
        else
            self->ep_desc[i].RX_Count.Value = (uint16_t)((_endp_config[i].RX_Max / 2) << 10);

        Addr = (uint16_t)(Addr + _endp_config[i].RX_Max);

        self->hw->EPR[i] = (_endp_config[i].Number | _endp_config[i].Type);

        _stm32_usb_set_rx_status(self, i, RX_VALID);
        _stm32_usb_set_tx_status(self, i, TX_NAK);
    }

    // enable all other interrupts and enable the usb peripheral
    self->hw->CNTR   = USB_CNTR_CTRM | USB_CNTR_RESETM | USB_CNTR_SUSPM | USB_CNTR_ERRM;
    self->hw->ISTR   = 0x00;
    self->hw->BTABLE = 0x00;
    self->hw->DADDR  = USB_DADDR_EF;

    // start receiving on ep 0
    _stm32_usb_set_rx_status(self, 0, RX_VALID);
}

static void _user_to_pma(uint16_t wPMABufAddr, uint8_t *pbUsrBuf, uint16_t wNBytes) {
    uint32_t n = (uint32_t)((wNBytes + 1) >> 1);   /* n = (wNBytes + 1) / 2 */
    uint32_t i, temp1, temp2;
    uint16_t *pdwVal;
    pdwVal = (uint16_t *)((uint32_t)(wPMABufAddr * 2) + USB_PBUFFER);
    for (i = n; i != 0; i--) {
        temp1 = (uint16_t) * pbUsrBuf;
        pbUsrBuf++;
        temp2 = temp1 | (uint16_t)((uint16_t) * pbUsrBuf << 8);
        *pdwVal++ = (uint16_t)temp2;
        pdwVal++;
        pbUsrBuf++;
    }
}

static void _pma_to_user(uint8_t *pbUsrBuf, uint16_t wPMABufAddr, uint16_t wNBytes){
    uint32_t n = (uint32_t)((wNBytes + 1) >> 1);/* /2*/
    uint32_t i;
    uint32_t *pdwVal;
    pdwVal = (uint32_t *)((uint32_t)(wPMABufAddr * 2) + USB_PBUFFER);
    for (i = n; i != 0; i--){
        *(uint16_t*)pbUsrBuf++ = (uint16_t)(*pdwVal);
        *pdwVal = 0;
        pdwVal++;
        pbUsrBuf++;
    }
}

static size_t _stm32_usb_pma_read(struct stm32_usb *self, uint8_t epidx){
    uint16_t count = (uint16_t)(self->ep_desc[epidx].RX_Count.Value & 0x3FF);
    if(count > _endp_config[epidx].RX_Max) count = _endp_config[epidx].RX_Max;
    _endp_config[epidx].lRX = count;
    _pma_to_user((uint8_t*)_endp_config[epidx].pRX_BUFF, self->ep_desc[epidx].RX_Address.Value, count);
	return count;
}

static void _stm32_usb_pma_write(struct stm32_usb *self, uint8_t epidx){
    uint16_t count = (uint16_t)(_endp_config[epidx].lTX <= _endp_config[epidx].TX_Max) ? (uint16_t)_endp_config[epidx].lTX : (uint16_t)_endp_config[epidx].TX_Max;
    _user_to_pma(self->ep_desc[epidx].TX_Address.Value, (uint8_t*)_endp_config[epidx].pTX_BUFF, count);

    volatile const char *b = (volatile const char*)_endp_config[epidx].pTX_BUFF;
    printk_isr("TX %d %02x%02x%02x%02x%02x%02x%02x%02x\n", count, b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7]);

    _endp_config[epidx].lTX -= count;
    _endp_config[epidx].pTX_BUFF += (count + 1) / 2;

    self->ep_desc[epidx].TX_Count.Value = count;
}

void _stm32_usb_send(struct stm32_usb *self, uint8_t epidx, uint16_t *buf, uint16_t count){
    // setup buffers which will then be handled inside interrupt
    _endp_config[epidx].lTX      = count;
    _endp_config[epidx].pTX_BUFF = buf;
    _endp_config[epidx].send_zlp = count && (count % _endp_config[epidx].TX_Max) == 0;
    if (count > 0) {
        _stm32_usb_pma_write(self, epidx);
    } else {
        self->ep_desc[epidx].TX_Count.Value = 0;
    }
    //_stm32_usb_set_rx_status(0, RX_NAK);
    _stm32_usb_set_tx_status(self, epidx, TX_VALID);
}

static int _stm32_usb_send_descriptor(struct stm32_usb *self, int type, uint8_t index, uint16_t length){
    //printk_isr("GD %d %d %d\n", type, index, length);
    switch (type) {
    case USB_DESC_TYPE_DEVICE:
        if(!self->dev.device_desc) goto error;
        _stm32_usb_send(self, 0, (uint16_t *)self->dev.device_desc, (length < sizeof(struct usb_device_descriptor))?length:sizeof(struct usb_device_descriptor));
        break;
    case USB_DESC_TYPE_CFG: {
        if(!self->dev.config_desc) goto error;
        uint16_t len = (uint16_t)(((uint16_t)self->dev.config_desc->wTotalLengthH << 8) | (uint16_t)self->dev.config_desc->wTotalLengthL);
        _stm32_usb_send(self, 0, (uint16_t *)self->dev.config_desc, (length < len)?length:len);
    } break;
    case USB_DESC_TYPE_DEVICE_QR:
        _stm32_usb_send(self, 0, 0, 0);
        //_stm32_usb_send(self, 0, (uint16_t *)self->config, sizeof(USB_DEVICE_QR_DESC));
        break;
    case USB_DESC_TYPE_STR: {
        if(index >= self->dev.string_desc_count) goto error;
        const struct usb_string_descriptor *str = self->dev.string_desc[index];
        if(!str) goto error;
        _stm32_usb_send(self, 0, (uint16_t*)str, str->bLength);
    } break;
    default:
        goto error;
        break;
    }
    return 0;
error:
    return -1;
}

static int _stm32_usb_handle_ep0_setup(struct stm32_usb *self, volatile struct usb_setup_packet *packet, uint16_t epr){
    switch (packet->wRequestAndType) {
    case USB_REQ(0x00, USB_REQ_DIR_IN | USB_REQ_TYPE_STD | USB_REQ_RCP_DEV):
    case USB_REQ(0x00, USB_REQ_DIR_IN | USB_REQ_TYPE_STD | USB_REQ_RCP_IFACE):
        _stm32_usb_send(self, 0, &self->status, 2);
        break;
    case USB_REQ(0x06, USB_REQ_DIR_IN | USB_REQ_TYPE_STD | USB_REQ_RCP_DEV):
    case USB_REQ(0x06, USB_REQ_DIR_IN | USB_REQ_TYPE_STD | USB_REQ_RCP_IFACE):
        if(_stm32_usb_send_descriptor(self, packet->wValueH, packet->wValueL, packet->wLength) < 0) goto error;
        break;
    case USB_REQ(0x05, USB_REQ_DIR_OUT | USB_REQ_TYPE_STD | USB_REQ_RCP_DEV):
        self->addr = (uint8_t)(packet->wValueL & 0x7F);
        _stm32_usb_send(self, 0, 0, 0);
        break;
    case USB_REQ(USB_REQUEST_GET_CONFIGURATION, USB_REQ_DIR_IN | USB_REQ_TYPE_STD | USB_REQ_RCP_DEV): {
        _stm32_usb_send(self, 0, (uint16_t*)&self->current_config, 1);
    } break;
    case USB_REQ(USB_REQUEST_SET_CONFIGURATION, USB_REQ_DIR_OUT | USB_REQ_TYPE_STD | USB_REQ_RCP_DEV):
        self->current_config = packet->wValueL;
        _stm32_usb_send(self, 0, 0, 0);
        break;
    default:
        goto error;
        break;
    }
    return 0;
error:
    _stm32_usb_set_rx_status(self, 0, RX_STALL);
    _stm32_usb_set_tx_status(self, 0, TX_STALL);
    return -1;
}

void _stm32_usb_handle_ep_request(struct stm32_usb *self, int dir, uint8_t epidx){
    if(!self) return;
    uint16_t epr  = (uint16_t)self->hw->EPR[epidx];
    if (epidx == 0) {
        //_stm32_usb_set_tx_status(epidx, TX_NAK);
        //_stm32_usb_set_rx_status(epidx, RX_NAK);

        if(dir == 0){
            if (epr & EP_CTR_TX) {
                self->hw->EPR[epidx] = epr & EP_MASK & ~EP_CTR_TX;
                // transmission has completed
                if (self->addr) {
                    self->hw->DADDR = (self->hw->DADDR & 0x0080) | self->addr;
                    self->addr = 0;
                }

                // send more data if more data is still to be sent
                if (_endp_config[epidx].lTX) {
                    _stm32_usb_pma_write(self, epidx);
                    _stm32_usb_set_tx_status(self, epidx, TX_VALID);
                } else if(_endp_config[epidx].send_zlp){
                    _stm32_usb_send(self, 0, 0, 0);
                    _endp_config[epidx].send_zlp = false;
                } else {
                    _stm32_usb_set_tx_status(self, epidx, TX_NAK);
                    //_stm32_usb_set_tx_status(epidx, TX_STALL);
                    //_stm32_usb_set_rx_status(0, RX_VALID);
                }
            } else if(epr & EP_CTR_RX){
            }
        } else {
            int len = (int)_stm32_usb_pma_read(self, epidx);
            if (epr & USB_EP0R_SETUP) {
                self->hw->EPR[epidx] = epr & EP_MASK & ~EP_CTR_RX;
                volatile struct usb_setup_packet *packet = (volatile struct usb_setup_packet *)_endp_config[0].pRX_BUFF;
                //volatile const char *b = (volatile const char*)packet;
                //printk_isr("S %d %02x%02x\n", len, b[6], b[7]);
                //printk_isr("S %d %02x%02x%02x%02x%02x%02x%02x%02x\n", len, b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7]);
                //printk_isr("SUP %d %02x %02x %04x %04x %04x\n", len, packet->bmRequestType, packet->bRequest, packet->wValue, packet->wIndex, packet->wLength);
                _stm32_usb_handle_ep0_setup(self, packet, epr);
                //_stm32_usb_set_rx_status(0, RX_VALID);
            } else if (epr & EP_CTR_RX) {
                self->hw->EPR[epidx] = epr & EP_MASK & ~EP_CTR_RX;
                printk_isr("DAT %d\n", len);
                //_stm32_usb_set_tx_status(epidx, TX_STALL);
                //_stm32_usb_send(0, 0, 0);
                //_stm32_usb_send(0, &self->status, 1);
            }
        }
        _stm32_usb_set_rx_status(self, epidx, RX_VALID);
    } else if(epidx == 2 && epr & EP_CTR_TX){
        self->hw->EPR[epidx] = epr & EP_MASK & ~EP_CTR_TX;
        if (_endp_config[epidx].lTX) {
            _stm32_usb_pma_write(self, epidx);
            _stm32_usb_set_tx_status(self, epidx, TX_VALID);
        }
    } else if(epidx == 3 && epr & EP_CTR_RX){
        self->hw->EPR[epidx] = epr & EP_MASK & ~EP_CTR_RX;
        static uint8_t str[] = {'H', 'e', 'l', 'l', 'o'};
        _stm32_usb_send(self, 2, (uint16_t*)str, 5);
        _stm32_usb_set_rx_status(self, epidx, RX_VALID);
    }
}

void USB_LP_CAN1_RX0_IRQHandler(){
	struct stm32_usb *self = _devices[0];

    uint16_t stat = (uint16_t)self->hw->ISTR;
    if (stat & USB_ISTR_RESET) { // Reset
        printk_isr("URST\n");
        _stm32_usb_reset(self);
    }
    if (stat & USB_ISTR_PMAOVR) {
        self->hw->ISTR &= (uint32_t)~USB_ISTR_PMAOVR;
    }
    if (stat & USB_ISTR_SUSP) {
        self->hw->ISTR &= (uint16_t)~USB_ISTR_SUSP;
    }
    if (stat & USB_ISTR_ERR) {
        self->hw->ISTR &= (uint32_t)~USB_ISTR_ERR;
    }
    if (stat & USB_ISTR_WKUP) {
        self->hw->ISTR &= (uint32_t)~USB_ISTR_WKUP;
    }
    if (stat & USB_ISTR_SOF) {
        self->hw->ISTR &= (uint32_t)~USB_ISTR_SOF;
    }
    if (stat & USB_ISTR_ESOF) {
        self->hw->ISTR &= (uint32_t)~USB_ISTR_ESOF;
    }

    while((stat = (uint16_t)self->hw->ISTR) & USB_ISTR_CTR){
        // flags can only be written as zero so we can clear all flags like this
        self->hw->ISTR = 0;
		uint8_t epidx = stat & USB_ISTR_EP_ID;
        uint8_t dir = !!(stat & USB_ISTR_DIR);
        _stm32_usb_handle_ep_request(self, dir, epidx);
    }
}

void USBWakeUp_IRQHandler(void){
	EXTI_ClearITPendingBit(EXTI_Line18);
}

static int _usb_device_write(usbd_device_t dev, uint8_t ep,  const void *ptr, size_t size, uint32_t timeout_ms){
    //struct stm32_usb *self = container_of(dev, struct stm32_usb, dev.ops);
    //_stm32_usb_send(self, ep, ptr, (uint16_t)size);
    return 0;
}

static int _usb_device_read(usbd_device_t dev, uint8_t ep, void *ptr, size_t size, uint32_t timeout_ms){
    //struct stm32_usb *self = container_of(dev, struct stm32_usb, dev.ops);
    //_stm32_usb_send(self, ep, ptr, (uint16_t)size);
    return 0;
}

static const struct usbd_device_ops _usb_device_ops = {
    .write = _usb_device_write,
    .read = _usb_device_read
};

static int _stm32_usb_probe(void *fdt, int fdt_node){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);

	RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);

	for(unsigned int i = 0; i < EPCOUNT; i++){
		_endp_config[i].pRX_BUFF = (uint16_t *)kzmalloc((size_t)(_endp_config[i].RX_Max * 2));
	}

    struct stm32_usb *self = _devices[0] = kzmalloc(sizeof(struct stm32_usb));
    self->hw = (volatile USB_TypeDef *)USB_BASE;
    self->ep_desc = (volatile USB_BufferDesc*)USB_PBUFFER;

    usbd_device_init(&self->dev, fdt, fdt_node, &_usb_device_ops);

    _stm32_usb_reset(self);

    self->hw->CNTR   = USB_CNTR_FRES; /* Force USB Reset */
    self->hw->BTABLE = 0;
    self->hw->DADDR  = 0;
    self->hw->ISTR   = 0;

	NVIC_InitTypeDef nvic;
	nvic.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 2;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);

    // clear reset bit and enable reset interrupt
    self->hw->CNTR   = USB_CNTR_RESETM;

    usbd_device_register(&self->dev);

	printk("USB: ok\n");
	return 0;
}

static int _stm32_usb_remove(void *fdt, int fdt_node){
	return 0;
}

DEVICE_DRIVER(stm32_usb, "st,stm32_usb", _stm32_usb_probe, _stm32_usb_remove)
