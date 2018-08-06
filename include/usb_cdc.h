#pragma once

#include "usb.h"

#define USB_CDC_CONFIG_DESC_SIZ 67
#define USB_CDC_CMD_EP 0x81 /* EP2 for CDC commands */
#define USB_CDC_IN_EP 0x82  /* EP1 for data IN */
#define USB_CDC_OUT_EP 0x02 /* EP1 for data OUT */

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


struct __packed usb_cdc_line_coding_packet {
    uint32_t baudRate;
    uint8_t  charFormat;
    uint8_t  parityType;
    uint8_t  dataBits;
};


