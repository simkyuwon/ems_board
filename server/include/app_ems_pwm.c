#include "app_ems_pwm.h"

#include <stdint.h>

#include "utils.h"
#include "sdk_config.h"
#include "example_common.h"
#include "ems_server.h"
#include "app_transition.h"

#include "log.h"
#include "app_timer.h"

static void ems_pwm_state_get_cb(const ems_pwm_server_t * p_self,
                                 const access_message_rx_meta_t * p_meta,
                                 ems_pwm_status_params_t * p_out);

static void ems_pwm_state_set_cb(const ems_pwm_server_t * p_self,
                                 const access_message_rx_meta_t * p_meta,
                                 const ems_pwm_set_params_t * p_in,
                                 const model_transition_t * p_in_transition,
                                 ems_pwm_status_params_t * p_out);

static void ems_pwm_state_delta_set_cb(const ems_pwm_server_t * p_self,
                                       const access_message_rx_meta_t * p_meta,
                                       const ems_pwm_delta_set_params_t * p_in,
                                       ems_pwm_status_params_t * p_out);

static void ems_pwm_state_move_set_cb(const ems_pwm_server_t * p_self,
                                      const access_message_rx_meta_t * p_meta,
                                      const ems_pwm_move_set_params_t * p_in,
                                      const model_transition_t * p_in_transition,
                                      ems_pwm_status_params_t * p_out);

const ems_pwm_server_callbacks_t ems_pwm_srv_cbs = {
  .ems_pwm_cbs.set_cb = ems_pwm_state_set_cb,
  .ems_pwm_cbs.get_cb = ems_pwm_state_get_cb,
};

static void transition_parameters_set(app_ems_pwm_server_t * p_app,
                                      const model_transition_t * p_in_transition){
  app_transition_params_t * p_params = app_transition_requested_get(&p_app->state.transition);

  p_params->transition_type = APP_TRANSITION_TYPE_SET;
  p_params->minimum_step_ms = TRANSITION_STEP_MIN_MS;

  if(p_in_transition == NULL){
    p_app->state.transition.delay_ms = 0;
    p_params->transition_time_ms = 0;
  }
  else{
    p_app->state.transition.delay_ms = p_in_transition->delay_ms;
    p_params->transition_time_ms = p_in_transition->transition_time_ms;
  }
}

static void ems_pwm_current_value_update(app_ems_pwm_server_t * p_app){
  if(p_app->ems_pwm_set_cb != NULL){
    p_app->ems_pwm_set_cb(p_app, p_app->state.present_duty);
  }
}

static void ems_pwm_state_get_cb(const ems_pwm_server_t * p_self,
                                 const access_message_rx_meta_t * p_metak,
                                 ems_pwm_status_params_t * p_out){
  __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "msg: GET\n");

  app_ems_pwm_server_t * p_app = PARENT_BY_FIELD_GET(app_ems_pwm_server_t, server, p_self);

  p_app->ems_pwm_get_cb(p_app, &p_app->state.present_duty);
  p_out->present_pwm_duty = p_app->state.present_duty;
  p_out->target_pwm_duty = p_app->state.target_duty;

  p_out->remaining_time_ms = app_transition_remaining_time_get(&p_app->state.transition);
}

static void ems_pwm_state_set_cb(const ems_pwm_server_t * p_self,
                                 const access_message_rx_meta_t * p_meta,
                                 const ems_pwm_set_params_t * p_in,
                                 const model_transition_t * p_in_transition,
                                 ems_pwm_status_params_t * p_out){
  __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "msg: SET : %d\n", p_in->pwm_duty);

  app_ems_pwm_server_t * p_app = PARENT_BY_FIELD_GET(app_ems_pwm_server_t, server, p_self);

  int16_t present_duty;
  p_app->ems_pwm_get_cb(p_app, &present_duty);
  p_app->value_updated = false;
  p_app->state.target_duty = p_in->pwm_duty;

  uint32_t transition_time_ms = 0;
  if(present_duty != p_in->pwm_duty){
    transition_parameters_set(p_app, p_in_transition);
    transition_time_ms = app_transition_requested_get(&p_app->state.transition)->transition_time_ms;
    app_transition_trigger(&p_app->state.transition);
  }

  if(p_out != NULL){
    p_out->present_pwm_duty = p_app->state.present_duty;
    p_out->target_pwm_duty = p_app->state.target_duty;
    p_out->remaining_time_ms = transition_time_ms;
  }
}




static void ems_pwm_state_delta_set_cb(const ems_pwm_server_t * p_self,
                                       const access_message_rx_meta_t * p_meta,
                                       const ems_pwm_delta_set_params_t * p_in,
                                       ems_pwm_status_params_t * p_out){
  
}

static void ems_pwm_state_move_set_cb(const ems_pwm_server_t * p_self,
                                      const access_message_rx_meta_t * p_meta,
                                      const ems_pwm_move_set_params_t * p_in,
                                      const model_transition_t * p_in_transition,
                                      ems_pwm_status_params_t * p_out){
}


static inline app_ems_pwm_server_t * transition_to_app(const app_transition_t * p_transition){
  app_ems_pwm_state_t * p_state = PARENT_BY_FIELD_GET(app_ems_pwm_state_t, transition, p_transition);
  return PARENT_BY_FIELD_GET(app_ems_pwm_server_t, state, p_state);
}

static void transition_start_cb(const app_transition_t * p_transition){
  app_ems_pwm_server_t * p_app = transition_to_app(p_transition);
  app_transition_params_t * p_params = app_transition_ongoing_get(&p_app->state.transition);
  
  if(p_app->ems_pwm_transition_cb != NULL){
    p_app->ems_pwm_transition_cb(p_app, p_params->transition_time_ms, p_app->state.target_duty);
  }
}

static void transition_complete_cb(const app_transition_t * p_transition){
  app_ems_pwm_server_t * p_app = transition_to_app(p_transition);
  app_transition_params_t * p_params = app_transition_ongoing_get(&p_app->state.transition);

  if(p_app->state.present_duty != p_app->state.target_duty){
    p_app->state.present_duty = p_app->state.target_duty;

    ems_pwm_current_value_update(p_app);

    ems_pwm_status_params_t status = {
              .present_pwm_duty = p_app->state.present_duty,
              .target_pwm_duty = p_app->state.target_duty,
              .remaining_time_ms = 0
              };
    (void) ems_pwm_server_status_publish(&p_app->server, &status);

    if(p_app->ems_pwm_transition_cb != NULL){
      p_app->ems_pwm_transition_cb(p_app, p_params->transition_time_ms, p_app->state.target_duty);
    }
  }
}


void app_ems_pwm_status_publish(app_ems_pwm_server_t * p_app){
  app_transition_abort(&p_app->state.transition);
  p_app->ems_pwm_get_cb(p_app, &p_app->state.present_duty);
  p_app->state.target_duty = p_app->state.present_duty;

  ems_pwm_status_params_t status = {
        .present_pwm_duty = p_app->state.present_duty,
        .target_pwm_duty = p_app->state.target_duty,
        .remaining_time_ms = 0
        };
   
   (void) ems_pwm_server_status_publish(&p_app->server, &status);
}

uint32_t app_ems_pwm_init(app_ems_pwm_server_t * p_app, uint8_t element_index){
  uint32_t status = NRF_ERROR_INTERNAL;

  if(p_app == NULL){
    return NRF_ERROR_NULL;
  }

  p_app->server.settings.p_callbacks = &ems_pwm_srv_cbs;
  if((p_app->ems_pwm_get_cb == NULL) ||
      ((p_app->ems_pwm_set_cb == NULL) &&
        p_app->ems_pwm_transition_cb == NULL)){
    return NRF_ERROR_NULL;
  }

  status = ems_pwm_server_init(&p_app->server, element_index);
  if(status != NRF_SUCCESS){
    return status;
  }

  p_app->state.transition.delay_start_cb = NULL;
  p_app->state.transition.transition_start_cb = transition_start_cb;
  p_app->state.transition.transition_tick_cb = NULL;
  p_app->state.transition.transition_complete_cb = transition_complete_cb;
  p_app->state.transition.p_context = &p_app->state.transition;

  app_transition_requested_get(&p_app->state.transition)->required_delta = 1;
  return app_transition_init(&p_app->state.transition);
}