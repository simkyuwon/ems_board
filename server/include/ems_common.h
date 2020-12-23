#ifndef EMS_COMMON_H__
#define EMS_COMMON_H__

#include <stdint.h>

#include "model_common.h"
#include "ems_messages.h"

#define EMS_PWM_COMPANY_ID 0xFFFF

typedef struct
{
    uint8_t  board_position;
}ems_status_params_t;

typedef struct
{
    uint8_t         tid;
    ems_msg_type_t  message_type;
    uint8_t         position;
    int32_t         data;
}ems_set_params_t;

typedef struct
{
    ems_msg_type_t  message_type;
    int32_t         data;
}ems_response_params_t;

typedef struct
{
    uint8_t         tid;
    ems_msg_type_t  message_type;
}ems_get_params_t;
#endif