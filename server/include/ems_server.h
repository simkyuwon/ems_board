#ifndef EMS_SERVER_H__
#define EMS_SERVER_H__

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "access.h"
#include "ems_common.h"
#include "ems_messages.h"

#define EMS_SERVER_MODEL_ID (0x1234)

typedef struct __ems_server_t ems_server_t;

typedef void (*ems_state_set_cb_t)(const ems_server_t * p_self,
                                   const access_message_rx_meta_t * p_meta,
                                   const ems_set_params_t * p_in,
                                   ems_status_params_t * p_out);

typedef void (*ems_state_get_cb_t)(const ems_server_t * p_self,
                                   const access_message_rx_meta_t * p_meta,
                                   const ems_get_params_t * pin,
                                   ems_response_params_t * p_out);

typedef struct
{
    ems_state_set_cb_t set_cb;
    ems_state_get_cb_t get_cb;
}ems_server_state_cbs_t;

typedef struct
{
    ems_server_state_cbs_t ems_cbs;
}ems_server_callbacks_t;

typedef struct
{
    bool                            force_segmented;
    nrf_mesh_transmic_size_t        transmic_size;
    const ems_server_callbacks_t *  p_callbacks;
}ems_server_settings_t;

struct __ems_server_t
{
    access_model_handle_t     model_handle;
    tid_tracker_t             tid_tracker;
    ems_server_settings_t     settings;
};

uint32_t ems_server_init(ems_server_t * p_server, uint8_t element_index);
uint32_t ems_server_status_publish(ems_server_t * p_server, const ems_status_params_t * p_params);
 
#endif