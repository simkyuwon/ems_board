#ifndef EMS_MESSAGES_H__
#define EMS_MESSAGES_H__

#include <stdint.h>

#define EMS_SET_COMMAND_MESSAGE_LEN   (sizeof(ems_set_msg_pkt_t))
#define EMS_SET_NOTICE_MESSAGE_LEN    (sizeof(ems_set_msg_pkt_t) - 5)

#define EMS_STATUS_MESSAGE_LEN        (sizeof(ems_status_msg_pkt_t))

#define NOTICE_MESSAGE(msg_type)      ((msg_type) & 0x8000)

typedef enum
{
    CMD_CONTROL_BLE = 0,
    CMD_CONTROL_BUTTON,

    CMD_WAVEFORM_START,
    CMD_WAVEFORM_STOP,
    CMD_WAVEFORM_SINGLESHOT,

    CMD_WAVEFORM_PULSE_COUNT_SET,
    CMD_WAVEFORM_PULSE_PERIOD_SET,
    CMD_WAVEFORM_PULSE_WIDTH_SET,

    CMD_VOLTAGE_SEQ_PERIOD_SET,
    CMD_VOLTAGE_LEVEL_SET,
    CMD_VOLTAGE_SEQUENCE_SET,

    CMD_PELTIER_STOP,
    CMD_PELTIER_HEATING,
    CMD_PELTIER_COOLING,
    
    //test command
    CMD_VOLTAGE_PERIOD_SET = 100,
    CMD_VOLTAGE_DUTY_SET,

    NOTICE_PORT_OPEN = 0x8000,
    NOTICE_PORT_CLOSE,
} ems_msg_type_t;

typedef enum
{
  EMS_OPCODE_GET                = 0x8205,
  EMS_OPCODE_SET                = 0x8206,
  EMS_OPCODE_SET_UNACKNOWLEDGED = 0x8207,
  EMS_OPCODE_STATUS             = 0x8208,
} ems_opcode_t;

typedef struct __attribute((packed))
{
    uint32_t  board_position;
}ems_status_msg_pkt_t;

typedef struct __attribute((packed))
{
    uint8_t         tid;
    ems_msg_type_t  message_type;
    uint8_t         position;
    int32_t         data;
}ems_set_msg_pkt_t;

#endif