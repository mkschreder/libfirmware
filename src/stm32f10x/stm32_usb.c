/**
 * This is a usb interface driver for the HS and FS device side interface. In
 * unconfigured state the interface is started with NAK on both TX and RX lines
 * on all endpoints. The interface needs to be configured with callbacks and
 * descriptors using a specific interface device driver which will handle
 * communication with the host.
 */

/**
 * USB operation
 *
 * When data is sent from the host to the device: 
 * - host sends an OUT transaction to an endpoint
 * - ISTR is set in status and CTR_RX flag is set in EPR
 * - Interrupt clears rx flag and sets rx_ready semaphore
 * - Application that sleeps on rx_ready wakes up
 * - Application disables usb interrupt, reads data from pma, sets RX_VALID and enables the interrupt
 *
 * When data is sent from device to host:
 * - Application waits for tx_ready semaphore
 * - Application disables usb interrupt, copies data into pma, sets TX_VALID and enables the interrupt
 * - USB transmits data to host
 * - Interrupt sets tx_ready once done
 **/

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

// maximum number of supported eps by stm32
#define MAX_ENDPOINTS 8

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

#define USB_CONTROL_WRITE_TIMEOUT 1000
#define USB_CONTROL_READ_TIMEOUT 1000

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

struct stm32_usb {
    struct usbd_device dev;

	uint8_t addr;
	uint8_t current_config;
	uint16_t status;

    volatile USB_TypeDef *hw;
    volatile USB_BufferDesc *ep_desc;

    int32_t wake;

    struct semaphore isr_ready;
};

static struct stm32_usb *_devices[1] = {0};

void _stm32_usb_set_tx_status(struct stm32_usb *self, uint8_t epidx, uint16_t Stat)
{
	if(epidx >= MAX_ENDPOINTS) return;
    register uint16_t val = (uint16_t)self->hw->EPR[epidx];
    self->hw->EPR[epidx]         = (val ^ (Stat & EP_STAT_TX)) & (EP_MASK | EP_STAT_TX);
}

void _stm32_usb_set_rx_status(struct stm32_usb *self, uint8_t epidx, uint16_t Stat)
{
	if(epidx >= MAX_ENDPOINTS) return;
    register uint16_t val = (uint16_t)self->hw->EPR[epidx];
    self->hw->EPR[epidx]         = (val ^ (Stat & EP_STAT_RX)) & (EP_MASK | EP_STAT_RX);
}
static int _stm32_usb_pma_read(struct stm32_usb *self, uint8_t epidx){
    struct usb_endpoint *ep = self->dev.endpoints[epidx];
    uint16_t count = (uint16_t)(self->ep_desc[epidx].RX_Count.Value & 0x3FF);
    uint8_t *data = ep->rx_buffer;
    uint16_t size = ep->rx_count;

    if(!data || !size) return 0;

    if(count > size) count = (uint16_t)size;

    uint32_t n = (uint32_t)((count + 1) >> 1);/* /2*/
    uint32_t i;
    uint32_t *pdwVal = (uint32_t *)((uint32_t)(self->ep_desc[epidx].RX_Address.Value * 2) + USB_PBUFFER);
    for (i = n; i != 0; i--){
        *(uint16_t*)data = (uint16_t)(*pdwVal);
        data+=2;

        *pdwVal = 0;
        pdwVal++;
    }

    // update pointers for next read
    ep->rx_buffer = data;
    ep->rx_count = (uint16_t)(ep->rx_count - count);

	return count;
}

/* write a single buffer out to pma. The buffer must not be longer than maximum packet size */
static size_t _stm32_usb_pma_write(struct stm32_usb *self, uint8_t epidx){
    struct usb_endpoint *ep = self->dev.endpoints[epidx];
    uint8_t *data = ep->tx_buffer;
    size_t size = ep->tx_count;

    if(size && !data) return 0;

    uint16_t max_size = self->dev.endpoints[epidx]->buffer_size;
    if(size > max_size) size = max_size;

    uint32_t n = (uint32_t)((size + 1) >> 1);   /* n = (wNBytes + 1) / 2 */
    uint32_t i, temp1, temp2;
    uint16_t *pdwVal;
    pdwVal = (uint16_t *)((uint32_t)(self->ep_desc[epidx].TX_Address.Value * 2) + USB_PBUFFER);
    for (i = n; i != 0; i--) {
        temp1 = (uint16_t) *data;
        data++;
        temp2 = temp1 | (uint16_t)((uint16_t) * data << 8);
        *pdwVal++ = (uint16_t)temp2;
        pdwVal++;
        data++;
    }

    self->ep_desc[epidx].TX_Count.Value = (uint16_t)size;

    ep->tx_buffer = data;
    ep->tx_count = (uint16_t)(ep->tx_count - size);

    printk("TX %d\n", size);

    return size;
}

void _stm32_usb_handle_ep_request(struct stm32_usb *self, int dir, uint8_t epidx){
    if(!self) return;
    uint16_t epr  = (uint16_t)self->hw->EPR[epidx];
    struct usb_endpoint *ep = self->dev.endpoints[epidx];

    if(epr & EP_CTR_RX) {
        self->hw->EPR[epidx] = epr & EP_MASK & ~EP_CTR_RX;

        // load data into buffer
        _stm32_usb_pma_read(self, epidx);

        // if there is space left in buffer enable reception and keep waiting for more data
        if(ep->rx_count > 0){
            // continue reception of more packets
            _stm32_usb_set_rx_status(self, epidx, RX_VALID);
        } else {
            // we are ready
            thread_sem_give(&ep->rx_ready);
        }
    }

    if(epr & EP_CTR_TX){
        self->hw->EPR[epidx] = epr & EP_MASK & ~EP_CTR_TX;

        if (self->addr) {
            self->hw->DADDR = (self->hw->DADDR & 0x0080) | self->addr;
            self->addr = 0;
        }

        // if we get here and the packet was an empty packet then it has already been sent
        if(_stm32_usb_pma_write(self, epidx) > 0){
            // if there was more data loaded then we mark transmit buffer for transmission
            _stm32_usb_set_tx_status(self, epidx, TX_VALID);
        } else {
            // transmission has completed, load more data into pma and enable transmission again or bail
            printk("TXDONE\n");
            thread_sem_give(&ep->tx_ready);
        }
    }
}

void _stm32_usb_reset(struct stm32_usb *self){
    // reconfigure the pma descriptors
    uint8_t ep_count = self->dev.endpoint_count;
    uint32_t Addr = sizeof(USB_BufferDesc) * ep_count;
    for (uint8_t i = 0; i < ep_count; i++) {
        struct usb_endpoint *epb = self->dev.endpoints[i];
        if(!epb) continue;
        uint16_t buf_size = USB_WORD_HL(epb->desc->wMaxPacketSizeH, epb->desc->wMaxPacketSizeL);
        uint16_t endp_type = (uint16_t)((uint16_t)(epb->desc->bmAttributes & 0x6) << 8);

        self->ep_desc[i].TX_Address.Value = (uint16_t)Addr;
        self->ep_desc[i].TX_Count.Value   = 0;
        Addr += buf_size;

        self->ep_desc[i].RX_Address.Value = (uint16_t)Addr;

        // compute number of rx blocks
        if (buf_size > 62)
            self->ep_desc[i].RX_Count.Value = 0x8000 | (uint16_t)((buf_size / 64) << 10);
        else
            self->ep_desc[i].RX_Count.Value = (uint16_t)((buf_size / 2) << 10);

        Addr += buf_size;

        self->hw->EPR[i] = (uint16_t)((uint16_t)i | endp_type);

        // start reception but do not respond to transmission requests
        _stm32_usb_set_rx_status(self, i, RX_NAK);
        _stm32_usb_set_tx_status(self, i, TX_NAK);
    }

    // enable all other interrupts and enable the usb peripheral
    self->hw->CNTR   = USB_CNTR_CTRM | USB_CNTR_RESETM | USB_CNTR_SUSPM | USB_CNTR_ERRM;
    self->hw->ISTR   = 0x00;
    self->hw->BTABLE = 0x00;
    self->hw->DADDR  = USB_DADDR_EF;
}

void _stm32_usb_handle_isr(struct stm32_usb *self){
    uint16_t stat = (uint16_t)self->hw->ISTR;
    if (stat & USB_ISTR_RESET) { // Reset
        //self->usb_is_ready = false;
        printk("URST\n");
        /*
         * Set when the USB peripheral detects an active USB RESET signal at its inputs. The USB
         * peripheral, in response to a RESET, just resets its internal protocol state machine, generating
         * an interrupt if RESETM enable bit in the USB_CNTR register is set. Reception and
         * transmission are disabled until the RESET bit is cleared. All configuration registers do not
         * reset: the microcontroller must explicitly clear these registers (this is to ensure that the
         * RESET interrupt can be safely delivered, and any transaction immediately followed by a
         * RESET can be completed). The function address and endpoint registers are reset by an USB
         * reset event.
         */
        _stm32_usb_reset(self);
    }
    if (stat & USB_ISTR_PMAOVR) {
        self->hw->ISTR &= (uint32_t)~USB_ISTR_PMAOVR;
        /*
         * This bit is set if the microcontroller has not been able to respond in time to an USB memory
         * request. The USB peripheral handles this event in the following way: During reception an
         * ACK handshake packet is not sent, during transmission a bit-stuff error is forced on the
         * transmitted stream; in both cases the host will retry the transaction. The PMAOVR interrupt
         * should never occur during normal operations. Since the failed transaction is retried by the
         * host, the application software has the chance to speed-up device operations during this
         * interrupt handling, to be ready for the next transaction retry; however this does not happen
         * during Isochronous transfers (no isochronous transaction is anyway retried) leading to a loss
         * of data in this case. 
         */
        printk("PMAOVR\n");
    }
    if (stat & USB_ISTR_SUSP) {
        self->hw->ISTR &= (uint16_t)~USB_ISTR_SUSP;
        /*
         * This bit is set by the hardware when no traffic has been received for 3mS, indicating a
         * suspend mode request from the USB bus. The suspend condition check is enabled
         * immediately after any USB reset and it is disabled by the hardware when the suspend mode
         * is active (FSUSP=1) until the end of resume sequence.
         */
        //printk("SUSP\n");
    }
    if (stat & USB_ISTR_ERR) {
        self->hw->ISTR &= (uint32_t)~USB_ISTR_ERR;
        /*
         * This flag is set whenever one of the errors listed below has occurred:
         * NANS: No ANSwer. The timeout for a host response has expired.
         * CRC: Cyclic Redundancy Check error. One of the received CRCs, either in the token or in
         * the data, was wrong.
         * BST: Bit Stuffing error. A bit stuffing error was detected anywhere in the PID, data, and/or
         * CRC.
         * FVIO: Framing format Violation. A non-standard frame was received (EOP not in the right
         * place, wrong token sequence, etc.).
         * The USB software can usually ignore errors, since the USB peripheral and the PC host
         * manage retransmission in case of errors in a fully transparent way. This interrupt can be
         * useful during the software development phase, or to monitor the quality of transmission over
         * the USB bus, to flag possible problems to the user (e.g. loose connector, too noisy
         * environment, broken conductor in the USB cable and so on). 
         */
        printk("ERR\n");
    }
    if (stat & USB_ISTR_WKUP) {
        self->hw->ISTR &= (uint32_t)~USB_ISTR_WKUP;
        /*
         * This bit is set to 1 by the hardware when, during suspend mode, activity is detected that
         * wakes up the USB peripheral. This event asynchronously clears the LP_MODE bit in the
         * CTLR register and activates the USB_WAKEUP line, which can be used to notify the rest of
         * the device (e.g. wakeup unit) about the start of the resume process.
         */
        printk("WKUP\n");
    }
    if (stat & USB_ISTR_SOF) {
        self->hw->ISTR &= (uint32_t)~USB_ISTR_SOF;
        /*
         * This bit signals the beginning of a new USB frame and it is set when a SOF packet arrives
         * through the USB bus. The interrupt service routine may monitor the SOF events to have a
         * 1 mS synchronization event to the USB host and to safely read the USB_FNR register which
         * is updated at the SOF packet reception (this could be useful for isochronous applications).
         */
        //printk("SOF\n");
    }
    if (stat & USB_ISTR_ESOF) {
        self->hw->ISTR &= (uint32_t)~USB_ISTR_ESOF;
        /*
         * This bit is set by the hardware when an SOF packet is expected but not received. The host
         * sends an SOF packet each mS, but if the hub does not receive it properly, the Suspend
         * Timer issues this interrupt. If three consecutive ESOF interrupts are generated (i.e. three
         * SOF packets are lost) without any traffic occurring in between, a SUSP interrupt is
         * generated. This bit is set even when the missing SOF packets occur while the Suspend
         * Timer is not yet locked.
         */
        //printk("ESOF\n");
    }

    while((stat = (uint16_t)self->hw->ISTR) & USB_ISTR_CTR){
        /*
         * This bit is set by the hardware to indicate that an endpoint has successfully completed a
         * transaction; using DIR and EP_ID bits software can determine which endpoint requested the
         * interrupt. 
         */
        // flags can only be written as zero so we can clear all flags like this
        self->hw->ISTR = 0;
		uint8_t epidx = stat & USB_ISTR_EP_ID;
        uint8_t dir = !!(stat & USB_ISTR_DIR);
        _stm32_usb_handle_ep_request(self, dir, epidx);
    }
}

void USB_LP_CAN1_RX0_IRQHandler(){
	struct stm32_usb *self = _devices[0];

    thread_sem_give_from_isr(&self->isr_ready, &self->wake);

	NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);

    thread_yield_from_isr(self->wake);
    self->wake = 0;
}

void USBWakeUp_IRQHandler(void){
	EXTI_ClearITPendingBit(EXTI_Line18);
}

static int _usb_device_write(usbd_device_t dev, uint8_t epidx,  const void *ptr, size_t size, uint32_t timeout_ms){
    struct stm32_usb *self = container_of(dev, struct stm32_usb, dev.ops);
    if(epidx >= self->dev.endpoint_count || !self->dev.endpoints[epidx]) return -1;
    struct usb_endpoint *ep = self->dev.endpoints[epidx];
    uint8_t *data = (uint8_t*)ptr;

    ep->tx_buffer = data;
    ep->tx_count = (uint16_t)size;

    // load the first packet into memory and enable transmission
    _stm32_usb_pma_write(self, epidx);

    // start transmission
    _stm32_usb_set_tx_status(self, epidx, TX_VALID);

    // wait until current transmission is done
    int ret;
    if((ret = thread_sem_take_wait(&ep->tx_ready, timeout_ms)) < 0) {
        goto err;
    }

    // this is necessary if the transfer is multiple of buffer size (send empty packet)
    if(size && size % ep->buffer_size == 0) {
        _stm32_usb_pma_write(self, epidx);
        _stm32_usb_set_tx_status(self, epidx, TX_VALID);
        if((ret = thread_sem_take_wait(&ep->tx_ready, timeout_ms)) < 0) {
            goto err;
        }
    }

    return (int)(size - ep->tx_count);
err:
    dbg_printk("TX timeout\n");
    return (ep->tx_count != size)?(int)(size - ep->tx_count):-ETIMEDOUT;
}

static int _usb_device_read(usbd_device_t dev, uint8_t epidx, void *ptr, size_t size, uint32_t timeout_ms){
    struct stm32_usb *self = container_of(dev, struct stm32_usb, dev.ops);
    if(epidx >= self->dev.endpoint_count || !self->dev.endpoints[epidx]) return -1;
    struct usb_endpoint *ep = self->dev.endpoints[epidx];
    uint8_t *data = (uint8_t*)ptr;

    // wait until usb interrupt is ready to accept reception requests
    ep->rx_buffer = data;
    ep->rx_count = (uint16_t)size;

    // start reception
    _stm32_usb_set_rx_status(self, epidx, RX_VALID);

    // wait for reception to be completed
    int received;
    if(thread_sem_take_wait(&ep->rx_ready, timeout_ms) < 0){
        _stm32_usb_set_rx_status(self, epidx, RX_NAK);
        printk("RXTOUT\n");
        received = -ETIMEDOUT;
    } else {
        received = (int)(size - ep->rx_count);
    }

    ep->rx_buffer = 0;
    ep->rx_count = 0;

    // epcount now contains number of bytes left to receive
    // return number of bytes received
    return received;
}

static const struct usbd_device_ops _usb_device_ops = {
    .write = _usb_device_write,
    .read = _usb_device_read
};

static int _stm32_usb_send_descriptor(struct stm32_usb *self, int type, uint8_t index, uint16_t length){
    printk("GD %d %d %d\n", type, index, length);
    switch (type) {
    case USB_DESC_TYPE_DEVICE:{
        if(!self->dev.device_desc) goto error;
        uint16_t len = (length < sizeof(struct usb_device_descriptor))?length:sizeof(struct usb_device_descriptor);
        _usb_device_write(&self->dev.ops, 0, (uint8_t*)self->dev.device_desc, len, USB_CONTROL_WRITE_TIMEOUT);
    } break;
    case USB_DESC_TYPE_CFG: {
        if(!self->dev.config_desc) goto error;
        uint16_t len = USB_WORD_HL(self->dev.config_desc->wTotalLengthH, self->dev.config_desc->wTotalLengthL);
        _usb_device_write(&self->dev.ops, 0, (uint8_t*)self->dev.config_desc, (length < len)?length:len, USB_CONTROL_WRITE_TIMEOUT);
    } break;
    case USB_DESC_TYPE_DEVICE_QR:
        _usb_device_write(&self->dev.ops, 0, NULL, 0, USB_CONTROL_WRITE_TIMEOUT);
        break;
    case USB_DESC_TYPE_STR: {
        if(index >= self->dev.string_desc_count) goto error;
        const struct usb_string_descriptor *str = self->dev.string_desc[index];
        if(!str) goto error;
        _usb_device_write(&self->dev.ops, 0, (uint8_t*)str, (length < str->bLength)?length:str->bLength, USB_CONTROL_WRITE_TIMEOUT);
    } break;
    default:
        goto error;
        break;
    }
    return 0;
error:
    printk("STALL\n");
    //_stm32_usb_set_tx_status(self, 0, TX_STALL);
    //_stm32_usb_set_rx_status(self, 0, RX_STALL);
    _usb_device_write(&self->dev.ops, 0, NULL, 0, USB_CONTROL_WRITE_TIMEOUT);
    return -1;
}

static int _stm32_usb_control_setup(struct stm32_usb *self, volatile struct usb_setup_packet *packet){
    printk("CTRL %04x\n", packet->wRequestAndType);
    switch (packet->wRequestAndType) {
    case USB_REQ(0x00, USB_REQ_DIR_IN | USB_REQ_TYPE_STD | USB_REQ_RCP_DEV):
    case USB_REQ(0x00, USB_REQ_DIR_IN | USB_REQ_TYPE_STD | USB_REQ_RCP_IFACE):
        _usb_device_write(&self->dev.ops, 0, (uint8_t*)&self->status, 2, USB_CONTROL_WRITE_TIMEOUT);
        break;
    case USB_REQ(0x06, USB_REQ_DIR_IN | USB_REQ_TYPE_STD | USB_REQ_RCP_DEV):
    case USB_REQ(0x06, USB_REQ_DIR_IN | USB_REQ_TYPE_STD | USB_REQ_RCP_IFACE):
        if(_stm32_usb_send_descriptor(self, packet->wValueH, packet->wValueL, packet->wLength) < 0) goto error;
        break;
    case USB_REQ(0x05, USB_REQ_DIR_OUT | USB_REQ_TYPE_STD | USB_REQ_RCP_DEV):
        self->addr = (uint8_t)(packet->wValueL & 0x7F);
        _usb_device_write(&self->dev.ops, 0, (uint8_t*)&self->status, 0, USB_CONTROL_WRITE_TIMEOUT);
        break;
    case USB_REQ(USB_REQUEST_GET_CONFIGURATION, USB_REQ_DIR_IN | USB_REQ_TYPE_STD | USB_REQ_RCP_DEV): {
        _usb_device_write(&self->dev.ops, 0, (uint8_t*)&self->current_config, 1, USB_CONTROL_WRITE_TIMEOUT);
    } break;
    case USB_REQ(USB_REQUEST_SET_CONFIGURATION, USB_REQ_DIR_OUT | USB_REQ_TYPE_STD | USB_REQ_RCP_DEV):
    case USB_REQ(USB_REQUEST_SET_CONFIGURATION, USB_REQ_DIR_OUT | USB_REQ_TYPE_STD | USB_REQ_RCP_IFACE):
        self->current_config = packet->wValueL;
        _usb_device_write(&self->dev.ops, 0, NULL, 0, USB_CONTROL_WRITE_TIMEOUT);
        break;
    default:
        goto error;
        break;
    }
    return 0;
error:
    printk("STALL2\n");
    _usb_device_write(&self->dev.ops, 0, NULL, 0, USB_CONTROL_WRITE_TIMEOUT);
    //_stm32_usb_set_rx_status(self, 0, RX_STALL);
    //_stm32_usb_set_tx_status(self, 0, TX_STALL);
    return -1;
}


// control handler task for a usb device. This task takes care of control transactions.
static void _usb_interrupt_task(void *ptr){
    struct stm32_usb *self = (struct stm32_usb*)ptr;
    while(1){
        thread_sem_take(&self->isr_ready);

        _stm32_usb_handle_isr(self);

	    NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
    }
}

static void _usb_control_task(void *ptr){
    struct stm32_usb *self = (struct stm32_usb*)ptr;
    while(1){
        struct usb_setup_packet packet;

        int ret = _usb_device_read(&self->dev.ops, 0, &packet, sizeof(packet), USB_CONTROL_READ_TIMEOUT);
        if(ret < 0) continue;

        //volatile const char *b = (volatile const char*)&packet;
        //printk("S %d %02x%02x%02x%02x%02x%02x%02x%02x\n", ret, b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7]);

        _stm32_usb_control_setup(self, &packet);
    }
}

static int _stm32_usb_probe(void *fdt, int fdt_node){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);

	RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);
/*
	for(unsigned int i = 0; i < EPCOUNT; i++){
		_endp_config[i].pRX_BUFF = (uint16_t *)kzmalloc((size_t)(_endp_config[i].RX_Max * 2));
	}
*/
    struct stm32_usb *self = _devices[0] = kzmalloc(sizeof(struct stm32_usb));
    self->hw = (volatile USB_TypeDef *)USB_BASE;
    self->ep_desc = (volatile USB_BufferDesc*)USB_PBUFFER;
    thread_sem_init(&self->isr_ready);

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

	thread_create(
		  _usb_interrupt_task,
		  "usb",
		  250,
		  self,
		  4,
		  NULL);

	thread_create(
		  _usb_control_task,
		  "usb",
		  250,
		  self,
		  4,
		  NULL);

	printk("USB: %d endpoints\n", self->dev.endpoint_count);
	return 0;
}

static int _stm32_usb_remove(void *fdt, int fdt_node){
	return 0;
}

DEVICE_DRIVER(stm32_usb, "st,stm32_usb", _stm32_usb_probe, _stm32_usb_remove)
