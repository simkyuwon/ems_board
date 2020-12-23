#include "ems_client.h"
#include "model_common.h"

#include "access_config.h"
#include "nrf_mesh_assert.h"
#include "nrf_mesh_utils.h"
#include "nordic_common.h"

static void status_handle(access_model_handle_t handle,
                          const access_message_rx_t * p_rx_msg,
                          void * p_args)
{
    ems_client_t * p_client     = (ems_client_t *)p_args;
    ems_status_params_t in_data = {0};

    if(p_rx_msg->length == EMS_STATUS_MESSAGE_LEN)
    {
        ems_status_msg_pkt_t * p_msg_params_packed = (ems_status_msg_pkt_t *) p_rx_msg->p_data;

        in_data.board_position = p_msg_params_packed->board_position;

        p_client->settings.p_callbacks->ems_status_cb(p_client,
                                                      &p_rx_msg->meta_data,
                                                      &in_data);
    }
}

static void response_handle(access_model_handle_t handle,
                            const access_message_rx_t * p_rx_msg,
                            void * p_args)
{
    ems_client_t * p_client   = (ems_client_t *)p_args;
    ems_response_params_t in_data  = {0};

    if(p_rx_msg->length == EMS_RESPONSE_MESSAGE_LEN)
    {
        ems_response_msg_pkt_t * p_msg_params_packed = (ems_response_msg_pkt_t *) p_rx_msg->p_data;

        in_data.data          = p_msg_params_packed->data;
        in_data.message_type  = p_msg_params_packed->message_type;

        p_client->settings.p_callbacks->ems_response_cb(p_client,
                                                        &p_rx_msg->meta_data,
                                                        &in_data);
    }
}

static const access_opcode_handler_t m_opcode_handlers[] = {
    {ACCESS_OPCODE_SIG(EMS_OPCODE_STATUS), status_handle},
    {ACCESS_OPCODE_SIG(EMS_OPCODE_RESPONSE), response_handle},
};

static uint8_t message_set_packet_create(ems_set_msg_pkt_t * p_set,
                                         const ems_set_params_t * p_params)
{
    p_set->tid = p_params->tid;
    p_set->message_type = p_params->message_type;
    if(NOTICE_MESSAGE(p_set->message_type))
    {
        return EMS_SET_NOTICE_MESSAGE_LEN;
    }
    else
    {
        p_set->position = p_params->position;
        p_set->data     = p_params->data;
        return EMS_SET_COMMAND_MESSAGE_LEN;
    }
}

static uint8_t message_get_packet_create(ems_get_msg_pkt_t * p_get,
                                         const ems_get_params_t * p_params)
{
    p_get->tid = p_params->tid;
    p_get->message_type = p_params->message_type;
    return EMS_GET_COMMAND_MESSAGE_LEN;
}

static void message_create(ems_client_t * p_client,
                           uint16_t tx_opcode,
                           const uint8_t * p_buffer,
                           uint16_t length,
                           access_message_tx_t * p_message)
{
    p_message->opcode.opcode      = tx_opcode;
    p_message->opcode.company_id  = ACCESS_COMPANY_ID_NONE;
    p_message->p_buffer           = p_buffer;
    p_message->length             = length;
    p_message->force_segmented    = p_client->settings.force_segmented;
    p_message->transmic_size      = p_client->settings.transmic_size;
    p_message->access_token       = nrf_mesh_unique_token_get();
}

static void reliable_context_create(ems_client_t * p_client,
                                    uint16_t reply_opcode,
                                    access_reliable_t * p_reliable)
{
    p_reliable->model_handle            = p_client->model_handle;
    p_reliable->reply_opcode.opcode     = reply_opcode;
    p_reliable->reply_opcode.company_id = ACCESS_COMPANY_ID_NONE;
    p_reliable->timeout                 = p_client->settings.timeout;
    p_reliable->status_cb               = p_client->settings.p_callbacks->ack_transaction_status_cb;
}

uint32_t ems_client_init(ems_client_t * p_client,
                         uint8_t element_index)
{
    if(p_client == NULL ||
       p_client->settings.p_callbacks == NULL ||
       p_client->settings.p_callbacks->ems_status_cb == NULL ||
       p_client->settings.p_callbacks->periodic_publish_cb == NULL)
    {
         return NRF_ERROR_NULL;
    }

    if(p_client->settings.timeout == 0)
    {
        p_client->settings.timeout = MODEL_ACKNOWLEDGED_TRANSACTION_TIMEOUT;
    }

    access_model_add_params_t add_params =
    {
        .model_id           = ACCESS_MODEL_SIG(EMS_CLIENT_MODEL_ID),
        .element_index      = element_index,
        .p_opcode_handlers  = &m_opcode_handlers[0],
        .opcode_count       = ARRAY_SIZE(m_opcode_handlers),
        .p_args             = p_client,
        .publish_timeout_cb = p_client->settings.p_callbacks->periodic_publish_cb
    };

    uint32_t status = access_model_add(&add_params, &p_client->model_handle);

    if(status == NRF_SUCCESS)
    {
        status = access_model_subscription_list_alloc(p_client->model_handle);
    }

    return status;
}

uint32_t ems_client_set(ems_client_t * p_client,
                        const ems_set_params_t * p_params)
{
    if(p_client == NULL || p_params == NULL)
    {
        return NRF_ERROR_NULL;
    }
  
    if(access_reliable_model_is_free(p_client->model_handle))
    {
        uint8_t server_msg_length = message_set_packet_create(&p_client->msg_pkt.set, p_params);
        message_create(p_client,
                       EMS_OPCODE_SET,
                       (const uint8_t *) &p_client->msg_pkt.set,
                       server_msg_length,
                       &p_client->access_message.message);
        reliable_context_create(p_client, EMS_OPCODE_STATUS, &p_client->access_message);
        return access_model_reliable_publish(&p_client->access_message);
    }
    else
    {
        return NRF_ERROR_BUSY;
    }
}

uint32_t ems_client_set_unack(ems_client_t * p_client,
                              const ems_set_params_t * p_params,
                              uint8_t repeats)
{
    if(p_client == NULL || p_params == NULL)
    {
        return NRF_ERROR_NULL;
    }

    ems_set_msg_pkt_t msg;
    uint8_t server_msg_length = message_set_packet_create(&msg, p_params);

    message_create(p_client,
                  EMS_OPCODE_SET_UNACKNOWLEDGED,
                  (const uint8_t *) &msg,
                  server_msg_length,
                  &p_client->access_message.message);

    uint32_t status = NRF_SUCCESS;

    ++repeats;
    while(repeats-- > 0 && status == NRF_SUCCESS)
    {
        status = access_model_publish(p_client->model_handle, &p_client->access_message.message);
    }
    return status;
}

uint32_t ems_client_get(ems_client_t * p_client,
                        const ems_get_params_t * p_params)
{
    if(p_client == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if(access_reliable_model_is_free(p_client->model_handle))
    {
        uint8_t server_msg_length = message_get_packet_create(&p_client->msg_pkt.get, p_params);
        message_create(p_client,
                       EMS_OPCODE_GET,
                       (const uint8_t *) &p_client->msg_pkt.get,
                       server_msg_length,
                       &p_client->access_message.message);
        reliable_context_create(p_client, EMS_OPCODE_RESPONSE, &p_client->access_message);
        return access_model_reliable_publish(&p_client->access_message);
    }
    else
    {
        return NRF_ERROR_BUSY;
    }
}