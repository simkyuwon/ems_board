#include "app_ems_board.h"

static void ems_state_get_cb(const ems_server_t * p_self,
                             const access_message_rx_meta_t * p_meta,
                             const ems_get_params_t * p_in,
                             ems_response_params_t * p_out);

static void ems_state_set_cb(const ems_server_t * p_self,
                             const access_message_rx_meta_t * p_meta,
                             const ems_set_params_t * p_in,
                             ems_status_params_t * p_out);

const ems_server_callbacks_t ems_srv_cbs =
{
    .ems_cbs.set_cb = ems_state_set_cb,
    .ems_cbs.get_cb = ems_state_get_cb,
};

static void ems_state_get_cb(const ems_server_t * p_self,
                             const access_message_rx_meta_t * p_meta,
                             const ems_get_params_t * p_in,
                             ems_response_params_t * p_out)
{
    app_ems_server_t * p_app = PARENT_BY_FIELD_GET(app_ems_server_t, server, p_self);

    p_out->message_type = p_in->message_type;
    p_app->ems_get_cb(p_in->message_type, &p_out->data);
}

static void ems_state_set_cb(const ems_server_t * p_self,
                             const access_message_rx_meta_t * p_meta,
                             const ems_set_params_t * p_in,
                             ems_status_params_t * p_out)
{
    app_ems_server_t * p_app = PARENT_BY_FIELD_GET(app_ems_server_t, server, p_self);

    p_app->ems_set_cb(p_in->message_type, p_in->position, p_in->data, &(p_out->board_position));
}

void app_ems_status_publish(app_ems_server_t * p_app)
{

    ems_status_params_t status;

    (void) ems_server_status_publish(&p_app->server, &status);
}

uint32_t app_ems_init(app_ems_server_t * p_app, uint8_t element_index)
{
    uint32_t status = NRF_ERROR_INTERNAL;

    if(p_app == NULL)
    {
        return NRF_ERROR_NULL;
    }

    p_app->server.settings.p_callbacks = &ems_srv_cbs;
    if((p_app->ems_get_cb == NULL) ||
       (p_app->ems_set_cb == NULL))
    {
        return NRF_ERROR_NULL;
    }

    status = ems_server_init(&p_app->server, element_index);

    if(status != NRF_SUCCESS)
    {
        return status;
    }

    return NRF_SUCCESS;
}