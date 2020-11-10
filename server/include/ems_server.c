#include "ems_server.h"

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "access.h"
#include "access_config.h"
#include "nrf_mesh_assert.h"
#include "nrf_mesh_utils.h"
#include "nordic_common.h"

#include "log.h"

static uint32_t status_send(ems_pwm_server_t * p_server,
                            const access_message_rx_t * p_message,
                            const ems_pwm_status_params_t * p_params){
  ems_pwm_status_msg_pkt_t msg_pkt;
  
  msg_pkt.present_pwm_duty = p_params->present_pwm_duty;
  if(p_params->remaining_time_ms > 0){
    msg_pkt.target_pwm_duty = p_params->target_pwm_duty;
    msg_pkt.remaining_time = model_transition_time_encode(p_params->remaining_time_ms);
  }

  access_message_tx_t reply = {
    .opcode = ACCESS_OPCODE_SIG(EMS_PWM_OPCODE_STATUS),
    .p_buffer = (const uint8_t *) &msg_pkt,
    .length = p_params->remaining_time_ms > 0 ? EMS_PWM_STATUS_MAXLEN : EMS_PWM_STATUS_MINLEN,
    .force_segmented = p_server->settings.force_segmented,
    .transmic_size = p_server->settings.transmic_size
  };

  if(p_message == NULL){
    return access_model_publish(p_server->model_handle, &reply);
  }
  else{
    return access_model_reply(p_server->model_handle, p_message, &reply);
  }
}

static void periodic_publish_cb(access_model_handle_t handle, void * p_args){
  ems_pwm_server_t * p_server = (ems_pwm_server_t *)p_args;
  ems_pwm_status_params_t out_data = {0};

  p_server->settings.p_callbacks->ems_pwm_cbs.get_cb(p_server, NULL, &out_data);
  (void) status_send(p_server, NULL, &out_data);
}

static void handle_set(access_model_handle_t model_handle, const access_message_rx_t * p_rx_msg, void * p_args){
  ems_pwm_server_t * p_server = (ems_pwm_server_t *) p_args;
  ems_pwm_set_params_t in_data = {0};
  model_transition_t in_data_tr = {0};
  ems_pwm_status_params_t out_data = {0};

  if(p_rx_msg->length == EMS_PWM_SET_MINLEN || p_rx_msg->length == EMS_PWM_SET_MAXLEN){
    ems_pwm_set_msg_pkt_t * p_msg_params_packed = (ems_pwm_set_msg_pkt_t *) p_rx_msg->p_data;
    in_data.pwm_duty = p_msg_params_packed->pwm_duty;
    in_data.tid = p_msg_params_packed->tid;
    if(model_tid_validate(&p_server->tid_tracker, &p_rx_msg->meta_data, EMS_PWM_OPCODE_SET, in_data.tid)){
      if(p_rx_msg->length == EMS_PWM_SET_MAXLEN){
        if(!model_transition_time_is_valid(p_msg_params_packed->transition_time)){
          return;
        }
        in_data_tr.transition_time_ms = model_transition_time_decode(p_msg_params_packed->transition_time);
        in_data_tr.delay_ms = model_delay_decode(p_msg_params_packed->delay);
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Server: transition_time_ms = %X\n", in_data_tr.transition_time_ms);
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Server: delay_ms = %X\n", in_data_tr.delay_ms);
      }
      p_server->settings.p_callbacks->ems_pwm_cbs.set_cb(p_server,
                                                     &p_rx_msg->meta_data, &in_data,
                                                     (p_rx_msg->length == EMS_PWM_SET_MINLEN) ? NULL : &in_data_tr,
                                                     (p_rx_msg->opcode.opcode == EMS_PWM_OPCODE_SET) ? &out_data : NULL);
      if(p_rx_msg->opcode.opcode == EMS_PWM_OPCODE_SET){
        (void) status_send(p_server, p_rx_msg, &out_data);
      }
    }
  }
}

static void handle_get(access_model_handle_t model_handle, const access_message_rx_t * p_rx_msg, void * p_args){
  ems_pwm_server_t * p_server = (ems_pwm_server_t *) p_args;
  ems_pwm_status_params_t out_data = {0};

  if(p_rx_msg->length == 0){
    p_server->settings.p_callbacks->ems_pwm_cbs.get_cb(p_server, &p_rx_msg->meta_data, &out_data);
    (void)status_send(p_server, p_rx_msg, &out_data);
  }
}

static const access_opcode_handler_t m_opcode_handlers[] = {
    {ACCESS_OPCODE_SIG(EMS_PWM_OPCODE_GET), handle_get},
    {ACCESS_OPCODE_SIG(EMS_PWM_OPCODE_SET), handle_set},
    {ACCESS_OPCODE_SIG(EMS_PWM_OPCODE_SET_UNACKNOWLEDGED), handle_set},
};

uint32_t ems_pwm_server_init(ems_pwm_server_t * p_server, uint8_t element_index){
  if(p_server == NULL ||
        p_server->settings.p_callbacks == NULL ||
        p_server->settings.p_callbacks->ems_pwm_cbs.set_cb == NULL ||
        p_server->settings.p_callbacks->ems_pwm_cbs.get_cb == NULL){
     return NRF_ERROR_NULL;
   }

   access_model_add_params_t init_params = {
     .model_id = ACCESS_MODEL_SIG(EMS_PWM_SERVER_MODEL_ID),
     .element_index = element_index,
     .p_opcode_handlers = m_opcode_handlers,
     .opcode_count = ARRAY_SIZE(m_opcode_handlers),
     .p_args = p_server,
     .publish_timeout_cb = periodic_publish_cb
   };

   uint32_t status = access_model_add(&init_params, &p_server->model_handle);

   if(status == NRF_SUCCESS){
    status = access_model_subscription_list_alloc(p_server->model_handle);
   }

   return status;
}

uint32_t ems_pwm_server_status_publish(ems_pwm_server_t * p_server, const ems_pwm_status_params_t * p_params){
  if(p_server == NULL ||
     p_params == NULL){
     return NRF_ERROR_NULL;
  }

  return status_send(p_server, NULL, p_params);
}