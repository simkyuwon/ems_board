#ifndef EMS_CLIENT_H__
#define EMS_CLIENT_H__

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "access.h"
#include "access_reliable.h"

#include "ems_common.h"
#include "ems_messages.h"

#define EMS_CLIENT_MODEL_ID 0x1235
#define EMS_COMPANY_ID 0xFFFF

typedef struct __ems_client_t ems_client_t;

typedef void (*ems_state_status_cb_t)(const ems_client_t * p_self,
                                      const access_message_rx_meta_t * p_meta,
                                      const ems_status_params_t * p_in);

typedef void (*ems_response_cb_t)(const ems_client_t * p_self,
                                  const access_message_rx_meta_t * p_meta,
                                  const ems_response_params_t * p_in);

typedef struct{
    ems_state_status_cb_t       ems_status_cb;
    ems_response_cb_t           ems_response_cb;
    access_reliable_cb_t        ack_transaction_status_cb;
    access_publish_timeout_cb_t periodic_publish_cb;
}ems_client_callbacks_t;

typedef struct{
    uint32_t                        timeout;
    bool                            force_segmented;
    nrf_mesh_transmic_size_t        transmic_size;
    const ems_client_callbacks_t *  p_callbacks;
}ems_client_settings_t;

typedef union{
    ems_set_msg_pkt_t set;
    ems_get_msg_pkt_t get;
}ems_client_msg_data_t;

struct __ems_client_t{
    access_model_handle_t   model_handle;
    ems_client_msg_data_t   msg_pkt;
    access_reliable_t       access_message;
    ems_client_settings_t   settings;
};

uint32_t ems_client_init(ems_client_t * p_client, uint8_t element_index);

uint32_t ems_client_set(ems_client_t * p_client,
                        const ems_set_params_t * p_params);

uint32_t ems_client_set_unack(ems_client_t * p_client,
                              const ems_set_params_t * p_params,
                              uint8_t repeats);

uint32_t ems_client_get(ems_client_t * p_client,
                        const ems_get_params_t * p_params);

#endif