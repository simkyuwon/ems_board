#ifndef APP_EMS_BOARD_H__
#define APP_ENS_BOARD_H__

#include <stdint.h>

#include "ems_server.h"
#include "app_transition.h"
#include "app_timer.h"

#define APP_EMS_PWM_SERVER_DEF(_name, _force_segmented, _mic_size, _set_cb, _get_cb)  \
    APP_TIMER_DEF(_name ## _timer);                                                   \
    static app_ems_server_t _name =                                                   \
    {                                                                                 \
        .server.settings.force_segmented = _force_segmented,                          \
        .server.settings.transmic_size = _mic_size,                                   \
        .ems_set_cb = _set_cb,                                                        \
        .ems_get_cb = _get_cb                                                         \
    };

typedef struct{
    uint8_t   position;
    uint16_t  command;
    int32_t   data;
}app_ems_state_t;

typedef struct __app_ems_server_t app_ems_server_t;

typedef void (*app_ems_set_cb_t)(ems_msg_type_t commamd, uint8_t position, int32_t data, uint8_t * const p_board_position);

typedef void (*app_ems_get_cb_t)(const ems_msg_type_t * p_command, int32_t * const p_data);

struct __app_ems_server_t{
    ems_server_t            server;
    app_timer_id_t const *  p_timer_id;
    app_ems_set_cb_t        ems_set_cb;
    app_ems_get_cb_t        ems_get_cb;

    app_ems_state_t         state;
    uint32_t                last_rtc_counter;
    bool                    value_updated;
};

void app_ems_status_publish(app_ems_server_t * p_app);

uint32_t app_ems_init(app_ems_server_t * p_app, uint8_t element_index);

#endif