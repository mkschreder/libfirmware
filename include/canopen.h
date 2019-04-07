/*
* Copyright 2017-2019 Martin K. Schröder
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
* may be used to endorse or promote products derived from this software without
* specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#pragma once

#include "can.h"
#include "timestamp.h"
#include "mutex.h"
#include "sem.h"
#include "thread.h"
#include "atomic.h"
#include "work.h"
#include "vardir.h"

enum {
	CANOPEN_COB_NMT = 0,
	CANOPEN_COB_SYNC_OR_EMCY = 0x080,
	CANOPEN_COB_TIME	= 0x100,
	CANOPEN_COB_TXPDO_0 = 0x180,
	CANOPEN_COB_RXPDO_0 = 0x200,
	CANOPEN_COB_TXPDO_1 = 0x280,
	CANOPEN_COB_RXPDO_1 = 0x300,
	CANOPEN_COB_TXPDO_2 = 0x380,
	CANOPEN_COB_RXPDO_2 = 0x400,
	CANOPEN_COB_TXPDO_3 = 0x480,
	CANOPEN_COB_RXPDO_3 = 0x500,
	CANOPEN_COB_TXSDO	= 0x580,
	CANOPEN_COB_RXSDO	= 0x600,
	CANOPEN_COB_LSS		= 0x780,
};

enum {
	CANOPEN_LSS_RX = 0x64,
	CANOPEN_LSS_TX = 0x65
};

enum {
	CANOPEN_NMT_CMD_ENABLE = 0x01,
	CANOPEN_NMT_CMD_DISABLE = 0x02,
	CANOPEN_NMT_CMD_PREOP = 0x80,
	CANOPEN_NMT_CMD_RESET = 0x81,
	CANOPEN_NMT_CMD_RESET_COM = 0x82
};

enum {
	CANOPEN_SDO_CMD_WRITE1		= 0x2F,
	CANOPEN_SDO_CMD_WRITE2		= 0x2B,
	CANOPEN_SDO_CMD_WRITE3		= 0x27,
	CANOPEN_SDO_CMD_WRITE4		= 0x23,
	CANOPEN_SDO_CMD_WRITE		= 0x60,
	CANOPEN_SDO_CMD_READ		= 0x40,
	CANOPEN_SDO_CMD_READ1		= 0x4F,
	CANOPEN_SDO_CMD_READ2		= 0x4B,
	CANOPEN_SDO_CMD_READ3		= 0x47,
	CANOPEN_SDO_CMD_READ4		= 0x43,
	CANOPEN_SDO_CMD_ABORT		= 0x80
};

#define CANOPEN_SDO_ERR_TOGGLE_BIT			0x05030000
#define CANOPEN_SDO_ERR_CLIENT_SERVER		0x05040001
#define CANOPEN_SDO_ERR_UNSUPPORTED_ACCESS	0x06010000
#define CANOPEN_SDO_ERR_NO_EXIST			0x06020000
#define CANOPEN_SDO_ERR_PDO_NO_MAP			0x06040041
#define CANOPEN_SDO_ERR_PDO_MAP_TOO_BIG		0x06040042
#define CANOPEN_SDO_ERR_INCOMPATIBLE_PARAMS	0x06040043
#define CANOPEN_SDO_ERR_GENERAL_INTERNAL	0x06040047
#define CANOPEN_SDO_ERR_LEN_INVAL			0x06070010
#define CANOPEN_SDO_ERR_LEN_HIGH			0x06070012
#define CANOPEN_SDO_ERR_LEN_LOW				0x06070013
#define CANOPEN_SDO_ERR_SUBINDEX_INVAL		0x06090011
#define CANOPEN_SDO_ERR_VALUE_OUT_OF_RANGE	0x06090030
#define CANOPEN_SDO_ERR_VALUE_TOO_HIGH		0x06090031
#define CANOPEN_SDO_ERR_VALUE_TOO_LOW		0x06090032
#define CANOPEN_SDO_ERR_GENERAL				0x08000000
#define CANOPEN_SDO_ERR_NOSTORE				0x08000020
#define CANOPEN_SDO_ERR_NOSTORE_LOCAL		0x08000021
#define CANOPEN_SDO_ERR_NOSTORE_STATE		0x08000022

#define CANOPEN_COMM_RANGE_START					0x100000
#define CANOPEN_COMM_RANGE_END						0x1fffff
#define CANOPEN_MFR_RANGE_START						0x200000
#define CANOPEN_MFR_RANGE_END						0x2fffff

#define CANOPEN_REG_DEVICE_TYPE						0x100000
#define CANOPEN_REG_DEVICE_ERROR					0x100100
#define CANOPEN_REG_DEVICE_ERROR_GENERIC_BIT		(1 << 0)
#define CANOPEN_REG_DEVICE_ERROR_CURRENT_BIT		(1 << 1)
#define CANOPEN_REG_DEVICE_ERROR_VOLTAGE_BIT		(1 << 2)
#define CANOPEN_REG_DEVICE_ERROR_TEMP_BIT			(1 << 3)
#define CANOPEN_REG_DEVICE_ERROR_COMM_BIT			(1 << 4)

#define CANOPEN_REG_DEVICE_MFR_STATUS				0x100200
#define CANOPEN_REG_DEVICE_MFR_STATUS_INIT_DONE_BIT		(1 << 16)
#define CANOPEN_REG_DEVICE_MFR_STATUS_TRIG1_REACHED_BIT	(1 << 17)
#define CANOPEN_REG_DEVICE_MFR_STATUS_TRIG2_REACHED_BIT	(1 << 18)
#define CANOPEN_REG_DEVICE_MFR_STATUS_TRIG3_REACHED_BIT	(1 << 19)
#define CANOPEN_REG_DEVICE_MFR_STATUS_TRIG4_REACHED_BIT	(1 << 20)
#define CANOPEN_REG_DEVICE_MFR_STATUS_AUTORUN_BIT		(1 << 21)
#define CANOPEN_REG_DEVICE_MFR_STATUS_LIM_SW_POS_BIT		(1 << 22)
#define CANOPEN_REG_DEVICE_MFR_STATUS_LIM_SW_NEG_BIT		(1 << 23)
#define CANOPEN_REG_DEVICE_MFR_STATUS_CAPTURE_BIT			(1 << 24)
#define CANOPEN_REG_DEVICE_MFR_STATUS_CMD_REACHED_BIT		(1 << 25)
#define CANOPEN_REG_DEVICE_MFR_STATUS_MOTOR_I2T_BIT		(1 << 26)
#define CANOPEN_REG_DEVICE_MFR_STATUS_DRIVE_I2T_BIT		(1 << 27)
#define CANOPEN_REG_DEVICE_MFR_STATUS_FAULT_BIT			(uint32_t)(1 << 31)


#define CANOPEN_REG_DEVICE_NAME						0x100800
#define CANOPEN_REG_DEVICE_HW_VER					0x100900
#define CANOPEN_REG_DEVICE_SW_VER					0x100A00
#define CANOPEN_REG_DEVICE_SERIAL					0x101800
#define CANOPEN_REG_DEVICE_SYNC_COB_ID				0x100500
#define CANOPEN_REG_DEVICE_CYCLE_PERIOD				0x100600
#define CANOPEN_REG_RXPDO_BASE						0x140000
#define CANOPEN_REG_TXPDO_BASE						0x180000

#define CANOPEN_REG_MOTION_ERROR					0x200000
#define CANOPEN_REG_MOTION_ERROR_CAN_ERROR_BIT		(1 << 0)
#define CANOPEN_REG_MOTION_ERROR_SHORT_BIT			(1 << 1)
#define CANOPEN_REG_MOTION_ERROR_INVALID_SETUP_BIT	(1 << 2)
#define CANOPEN_REG_MOTION_ERROR_CONTROL_ERR_BIT	(1 << 3)
#define CANOPEN_REG_MOTION_ERROR_COM_ERR_BIT		(1 << 4)
#define CANOPEN_REG_MOTION_ERROR_POS_WRAP_BIT		(1 << 5)
#define CANOPEN_REG_MOTION_ERROR_LIM_SW_POS_BIT		(1 << 6)
#define CANOPEN_REG_MOTION_ERROR_LIM_SW_NEG_BIT		(1 << 7)
#define CANOPEN_REG_MOTION_ERROR_OVCT_FAULT_BIT		(1 << 8)
#define CANOPEN_REG_MOTION_ERROR_I2T_FAULT_BIT		(1 << 9)
#define CANOPEN_REG_MOTION_ERROR_MOT_OVT_BIT		(1 << 10)
#define CANOPEN_REG_MOTION_ERROR_DRV_OVT_BIT		(1 << 11)
#define CANOPEN_REG_MOTION_ERROR_VHI_BIT			(1 << 12)
#define CANOPEN_REG_MOTION_ERROR_VLO_BIT			(1 << 13)
#define CANOPEN_REG_MOTION_ERROR_CMD_ERR_BIT		(1 << 14)
#define CANOPEN_REG_MOTION_ERROR_NO_EN_BIT			(1 << 15)

#define CANOPEN_REG_MOTION_ERROR_MASK				0x200100
#define CANOPEN_REG_DETAILED_ERROR					0x200200
#define CANOPEN_REG_DETAILED_ERROR2					0x200900
#define CANOPEN_REG_EXT_REFERENCE					0x201C00
#define CANOPEN_REG_EXT_REFERENCE_TYPE				0x201D00
#define CANOPEN_REG_EXT_REFERENCE_TYPE_ONLINE		1
#define CANOPEN_REG_EXT_REFERENCE_TYPE_ANALOG		2
#define CANOPEN_REG_TML_RUN							0x207700
#define CANOPEN_REG_TS_CURRENT_ACTUAL				0x207E00

#define CANOPEN_REG_DRIVE_CONN_FAULT_ACTION			0x600700
#define CANOPEN_REG_DRIVE_ERROR_CODE				0x603F00

#define CANOPEN_REG_DRIVE_CONTROL					0x604000
#define CANOPEN_REG_DRIVE_CONTROL_SW_ON_BIT			(1 << 0)
#define CANOPEN_REG_DRIVE_CONTROL_V_EN_BIT			(1 << 1)
#define CANOPEN_REG_DRIVE_CONTROL_RUN_BIT			(1 << 2)
#define CANOPEN_REG_DRIVE_CONTROL_OP_EN_BIT			(1 << 3)
#define CANOPEN_REG_DRIVE_CONTROL_MODE1_BIT			(1 << 4)
#define CANOPEN_REG_DRIVE_CONTROL_MODE2_BIT			(1 << 5)
#define CANOPEN_REG_DRIVE_CONTROL_FAULT_RST_BIT		(1 << 7)
#define CANOPEN_REG_DRIVE_CONTROL_HALT_BIT			(1 << 8)

#define CANOPEN_REG_DRIVE_STATUS					0x604100
#define CANOPEN_REG_DRIVE_STATUS_RDY_BIT			(1 << 0)
#define CANOPEN_REG_DRIVE_STATUS_SW_ON_BIT			(1 << 1)
#define CANOPEN_REG_DRIVE_STATUS_OP_EN_BIT			(1 << 2)
#define CANOPEN_REG_DRIVE_STATUS_FAULT_BIT			(1 << 3)
#define CANOPEN_REG_DRIVE_STATUS_VMOT_BIT			(1 << 4)
#define CANOPEN_REG_DRIVE_STATUS_QSTOP_BIT			(1 << 5)
#define CANOPEN_REG_DRIVE_STATUS_SW_ON_DIS_BIT		(1 << 6)
#define CANOPEN_REG_DRIVE_STATUS_WARNING_BIT		(1 << 7)
#define CANOPEN_REG_DRIVE_STATUS_HOMING_BIT			(1 << 8)
#define CANOPEN_REG_DRIVE_STATUS_REMOTE_BIT			(1 << 9)
#define CANOPEN_REG_DRIVE_STATUS_TARGET_REACHED_BIT	(1 << 10)
#define CANOPEN_REG_DRIVE_STATUS_INTERNAL_LIMIT_BIT	(1 << 11)
#define CANOPEN_REG_DRIVE_STATUS_EVENT_BIT			(1 << 14)
#define CANOPEN_REG_DRIVE_STATUS_AXIS_ON_BIT		(1 << 15)

#define CANOPEN_REG_DRIVE_REQUESTED_MODE			0x606000
#define CANOPEN_REG_DRIVE_CURRENT_MODE				0x606100
#define CANOPEN_REG_DRIVE_MODE_EXT_REF_TORQUE		(-5)
#define CANOPEN_REG_DRIVE_MODE_EXT_REF_SPEED		(-4)
#define CANOPEN_REG_DRIVE_MODE_EXT_REF_POS			(-3)
#define CANOPEN_REG_DRIVE_MODE_CAM_POS				(-2)
#define CANOPEN_REG_DRIVE_MODE_GEAR_POS				(-1)
#define CANOPEN_REG_DRIVE_MODE_PROFILE_POS			(1)
#define CANOPEN_REG_DRIVE_MODE_PROFILE_VEL			(3)
#define CANOPEN_REG_DRIVE_MODE_HOMING				(6)
#define CANOPEN_REG_DRIVE_MODE_INTERP_POS			(7)

#define CANOPEN_REG_DRIVE_FAULT_ACTION				0x605e00
#define CANOPEN_REG_DRIVE_POSITION_DEMAND			0x606200
#define CANOPEN_REG_DRIVE_POSITION_ACTUAL			0x606300
#define CANOPEN_REG_DRIVE_POSITION_MEASURED			0x606400
#define CANOPEN_REG_DRIVE_FOLLOWING_ERROR_WINDOW	0x606500
#define CANOPEN_REG_DRIVE_POSITION_WINDOW			0x606700
#define CANOPEN_REG_DRIVE_VELOCITY_DEMAND			0x606B00
#define CANOPEN_REG_DRIVE_VELOCITY_ACTUAL			0x606C00
#define CANOPEN_REG_DRIVE_VELOCITY_WINDOW			0x606D00
#define CANOPEN_REG_DRIVE_VELOCITY_THRESHOLD		0x606F00
#define CANOPEN_REG_DRIVE_VELOCITY_THRESHOLD_TIME	0x607000
#define CANOPEN_REG_DRIVE_TARGET_TORQUE				0x607100
#define CANOPEN_REG_DRIVE_MAX_CURRENT				0x607300
#define CANOPEN_REG_DRIVE_TORQUE_DEMAND				0x607400
#define CANOPEN_REG_DRIVE_CURRENT_RATED				0x607500
#define CANOPEN_REG_DRIVE_CURRENT_ACTUAL			0x607800
#define CANOPEN_REG_DRIVE_DC_VOLTAGE				0x607900
#define CANOPEN_REG_DRIVE_TARGET_POSITION			0x607A00
#define CANOPEN_REG_DRIVE_POLARITY					0x607E00
#define CANOPEN_REG_DRIVE_PROFILE_VELOCITY			0x608100
#define CANOPEN_REG_DRIVE_PROFILE_ACCELERATION		0x608300
#define CANOPEN_REG_DRIVE_PROFILE_DECELERATION		0x608400
#define CANOPEN_REG_DRIVE_FOLLOWING_ERROR			0x60F400
#define CANOPEN_REG_DRIVE_TARGET_VELOCITY			0x60FF00
#define CANOPEN_REG_DRIVE_SUPPORTED_MODES			0x650200

#define CANOPEN_PDO_TYPE_ACYCLIC	0
#define CANOPEN_PDO_TYPE_CYCLIC(n)	(((n) < 1)?1:(((n) > 240)?240:(n)))
#define CANOPEN_PDO_TYPE_SYNC_RTR	252
#define CANOPEN_PDO_TYPE_ASYNC_RTR	253
#define CANOPEN_PDO_TYPE_ASYNC		254
#define CANOPEN_PDO_TYPE_ASYNC_255	255

#define CANOPEN_DEFAULT_TXPDO_COUNT 8
#define CANOPEN_DEFAULT_RXPDO_COUNT 8
#define CANOPEN_DEFAULT_PDO_MAP_COUNT 8

#define CANOPEN_TXPDO_BASE (uint16_t)0x1800
#define CANOPEN_RXPDO_BASE (uint16_t)0x1400

enum {
	CANOPEN_DEVICE_TYPE_DRIVE = 402
};

enum {
	CANOPEN_DEVICE_ERR_GENERIC	= 1,
	CANOPEN_DEVICE_ERR_CURRENT	= 1 << 1,
	CANOPEN_DEVICE_ERR_VOLTAGE	= 1 << 2,
	CANOPEN_DEVICE_ERR_TEMP		= 1 << 3,
	CANOPEN_DEVICE_ERR_COMM		= 1 << 4,
	CANOPEN_DEVICE_ERR_PROFILE	= 1 << 5,
};

enum {
	CANOPEN_LSS_CMD_SWITCH_MODE = 0x04,
	CANOPEN_LSS_CMD_SET_ID		= 0x11,
	CANOPEN_LSS_CMD_SET_BAUD	= 0x13,
	CANOPEN_LSS_CMD_SAVE		= 0x17,
	CANOPEN_LSS_CMD_GET_ID		= 0x5e,
	CANOPEN_LSS_CMD_RESET		= 0x80,
	CANOPEN_LSS_CMD_FASTSCAN	= 0x81
};

typedef enum {
	CANOPEN_LSS_STATE_OFF,
	CANOPEN_LSS_STATE_SCAN_WAIT_CONFIRM,
	CANOPEN_LSS_STATE_ENABLE_WAIT_CONFIRM,
	CANOPEN_LSS_STATE_SET_ID_WAIT_CONFIRM,
} canopen_lss_state_t;

typedef enum {
	CANOPEN_MASTER,
	CANOPEN_SLAVE
} canopen_mode_t;

enum {
	CANOPEN_WRITE,
	CANOPEN_READ,
	CANOPEN_ERROR
};
// this is used when user configure the pdo
struct canopen_pdo_config {
	uint32_t cob_id;
	uint8_t index;
	uint8_t type;
	uint16_t inhibit_time;
	uint16_t event_time;
	uint32_t map[8];
};

#define CANOPEN_PDO_MAP_ENTRY(id, size) (uint32_t)((uint32_t)(id) << 8 | (uint32_t)(size))
#define CANOPEN_PDO_SIZE_32 0x20
#define CANOPEN_PDO_SIZE_16 0x10
#define CANOPEN_PDO_SIZE_8 0x08
#define CANOPEN_PDO_DISABLED 0x80000000
#define CANOPEN_COB_DISABLED 0x80000000

typedef const struct canopen_device_ops ** canopen_device_t;

struct canopen_device_ops {
	int (*sdo_read)(canopen_device_t dev, uint8_t node_id, uint32_t dict, void *data, size_t size);
	int (*sdo_write)(canopen_device_t dev, uint8_t node_id, uint32_t dict, const uint8_t *data, size_t size);
};

struct canopen_counters {
	atomic_t sync_in;
};

struct canopen_listener {
	struct list_head list;
	void (*callback)(struct canopen_listener *self, uint8_t node_id, struct can_message *msg);
};
/*
void canopen_set_identity(struct canopen *self, uint32_t uuid[4]);
void canopen_set_node_id(struct canopen *self, uint8_t id);
void canopen_set_sync_period(struct canopen *self, uint32_t period_us);

int canopen_lss_reset(struct canopen *self);
int canopen_lss_find_node(struct canopen *self, struct canopen_serial_number *serial);
int canopen_lss_mode(struct canopen *self, uint8_t mode);
int canopen_lss_set_node_id(struct canopen *self, struct canopen_serial_number *serial, uint8_t node_id);
int canopen_lss_enable(struct canopen *self, struct canopen_serial_number *node_serial);

int canopen_sdo_read(struct canopen *self, uint8_t node_id, uint32_t dict, void *data, size_t size);
int canopen_sdo_read_u32(struct canopen *self, uint8_t node_id, uint32_t dic, uint32_t *value);
int canopen_sdo_read_i32(struct canopen *self, uint8_t node_id, uint32_t dic, int32_t *value);
int canopen_sdo_read_u16(struct canopen *self, uint8_t node_id, uint32_t dic, uint16_t *value);
int canopen_sdo_read_i16(struct canopen *self, uint8_t node_id, uint32_t dic, int16_t *value);
int canopen_sdo_read_u8(struct canopen *self, uint8_t node_id, uint32_t dic, uint8_t *value);
int canopen_sdo_read_i8(struct canopen *self, uint8_t node_id, uint32_t dic, int8_t *value);

int canopen_sdo_write(struct canopen *self, uint8_t node_id, uint32_t dict, const uint8_t *data, size_t size);
int canopen_sdo_write_u32(struct canopen *self, uint8_t node_id, uint32_t dic, uint32_t value);
int canopen_sdo_write_i32(struct canopen *self, uint8_t node_id, uint32_t dic, int32_t value);
int canopen_sdo_write_u16(struct canopen *self, uint8_t node_id, uint32_t dic, uint16_t value);
int canopen_sdo_write_i16(struct canopen *self, uint8_t node_id, uint32_t dic, int16_t value);
int canopen_sdo_write_u8(struct canopen *self, uint8_t node_id, uint32_t dic, uint8_t value);
int canopen_sdo_write_i8(struct canopen *self, uint8_t node_id, uint32_t dic, int8_t value);

int canopen_sdo_read_device_type(struct canopen *self, uint8_t node_id, struct canopen_device_type *type);
int canopen_sdo_read_serial(struct canopen *self, uint8_t node_id, struct canopen_serial_number *serial);

int canopen_pdo_rx(struct canopen *self, uint8_t node_id, const struct canopen_pdo_config *conf);
int canopen_pdo_tx(struct canopen *self, uint8_t node_id, const struct canopen_pdo_config *conf);

// used for locally sending or receiving pdos
int canopen_pdo_tx_local(struct canopen *self, const struct canopen_pdo_config *conf);
int canopen_pdo_rx_local(struct canopen *self, const struct canopen_pdo_config *conf);

int canopen_pdo_transmit(struct canopen *self, uint16_t cob_id, const uint8_t data[8]);
int canopen_send_sync(struct canopen *self);

int canopen_nmt_enable(struct canopen *self, uint8_t node_id);
int canopen_nmt_reset(struct canopen *self, uint8_t node_id);

void canopen_listener_init(struct canopen_listener *self, void (*callback)(struct canopen_listener *self, uint8_t node_id, struct can_message *msg));

void canopen_register_listener(struct canopen *self, struct canopen_listener *l);
*/
const char *canopen_strerror(int32_t err);
