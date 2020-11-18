#include "ems_server.h"

#include <stddef.h>
#include <string.h>
#include "access_config.h"
#include "nrf_mesh_assert.h"
#include "nrf_mesh_utils.h"
#include "nordic_common.h"

#include "log.h"

static uint32_t status_send(ems_server_t * p_server,
                            const access_message_rx_t * p_message,
                            const ems_status_params_t * p_params)
                            {
    ems_status_msg_pkt_t msg_pkt;

    msg_pkt.board_position = p_params->board_position;

    access_message_tx_t reply = {
        .opcode = ACCESS_OPCODE_SIG(EMS_OPCODE_STATUS),
        .p_buffer = (const uint8_t *) &msg_pkt,
        .length = EMS_STATUS_MESSAGE_LEN,
        .force_segmented = p_server->settings.force_segmented,
        .transmic_size = p_server->settings.transmic_size
    };

    if(p_message == NULL)
    {
        return access_model_publish(p_server->model_handle, &reply);
    }
    else
    {
        return access_model_reply(p_server->model_handle, p_message, &reply);
    }
}

static void periodic_publish_cb(access_model_handle_t handle, void * p_args)
{
    ems_server_t * p_server = (ems_server_t *)p_args;
    ems_status_params_t out_data = {0};

    p_server->settings.p_callbacks->ems_cbs.get_cb(p_server, NULL, &out_data);
    (void) status_send(p_server, NULL, &out_data);
}

static void handle_set(access_model_handle_t model_handle, const access_message_rx_t * p_rx_msg, void * p_args)
{
    ems_server_t * p_server = (ems_server_t *) p_args;
    ems_set_params_t in_data = {0};
    ems_status_params_t out_data = {0};

    if(p_rx_msg->length == EMS_SET_COMMAND_MESSAGE_LEN || p_rx_msg->length == EMS_SET_NOTICE_MESSAGE_LEN)
    {
        ems_set_msg_pkt_t * p_msg_params_packed = (ems_set_msg_pkt_t *) p_rx_msg->p_data;
        in_data.tid = p_msg_params_packed->tid;
        in_data.message_type = p_msg_params_packed->message_type;

        if(p_rx_msg->length == EMS_SET_COMMAND_MESSAGE_LEN)
        {
            in_data.position = p_msg_params_packed->position;
            in_data.data = p_msg_params_packed->data;
        }

        if(model_tid_validate(&p_server->tid_tracker, &p_rx_msg->meta_data, EMS_OPCODE_SET, in_data.tid))
        {
            p_server->settings.p_callbacks->ems_cbs.set_cb(p_server,
                                                           &p_rx_msg->meta_data,
                                                           &in_data,
                                                           &out_data);
            if(p_rx_msg->opcode.opcode == EMS_OPCODE_SET)
            {
                (void) status_send(p_server, p_rx_msg, &out_data);
            }
        }
    }
}

static void handle_get(access_model_handle_t model_handle, const access_message_rx_t * p_rx_msg, void * p_args)
{
    ems_server_t * p_server = (ems_server_t *) p_args;
    ems_status_params_t out_data = {0};

    if(p_rx_msg->length == 0)
    {
        p_server->settings.p_callbacks->ems_cbs.get_cb(p_server, &p_rx_msg->meta_data, &out_data);
        (void)status_send(p_server, p_rx_msg, &out_data);
    }
}

static const access_opcode_handler_t m_opcode_handlers[] = {
    {ACCESS_OPCODE_SIG(EMS_OPCODE_GET), handle_get},
    {ACCESS_OPCODE_SIG(EMS_OPCODE_SET), handle_set},
    {ACCESS_OPCODE_SIG(EMS_OPCODE_SET_UNACKNOWLEDGED), handle_set},
};

uint32_t ems_server_init(ems_server_t * p_server, uint8_t element_index)
{
    if(p_server == NULL ||
       p_server->settings.p_callbacks == NULL ||
       p_server->settings.p_callbacks->ems_cbs.set_cb == NULL ||
       p_server->settings.p_callbacks->ems_cbs.get_cb == NULL)
    {
        return NRF_ERROR_NULL;
    }

     access_model_add_params_t init_params = {
         .model_id = ACCESS_MODEL_SIG(EMS_SERVER_MODEL_ID),
         .element_index = element_index,
         .p_opcode_handlers = m_opcode_handlers,
         .opcode_count = ARRAY_SIZE(m_opcode_handlers),
         .p_args = p_server,
         .publish_timeout_cb = periodic_publish_cb
     };

     uint32_t status = access_model_add(&init_params, &p_server->model_handle);

     if(status == NRF_SUCCESS)
     {
        status = access_model_subscription_list_alloc(p_server->model_handle);
     }

     return status;
}

uint32_t ems_server_status_publish(ems_server_t * p_server, const ems_status_params_t * p_params)
{
    if(p_server == NULL ||
       p_params == NULL)
    {
         return NRF_ERROR_NULL;
    }

    return status_send(p_server, NULL, p_params);
}