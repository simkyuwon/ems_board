#include <stdint.h>

#include "ems_server.h"
#include "app_transition.h"
#include "app_timer.h"

#define APP_EMS_PWM_SERVER_DEF(_name, _force_segmented, _mic_size, _set_cb, _get_cb, _transition_cb)  \
    APP_TIMER_DEF(_name ## _timer); \
    static app_ems_pwm_server_t _name =  \
    {  \
        .server.settings.force_segmented = _force_segmented,  \
        .server.settings.transmic_size = _mic_size,  \
        .state.transition.timer.p_timer_id = &_name ## _timer,  \
        .ems_pwm_set_cb = _set_cb,  \
        .ems_pwm_get_cb = _get_cb,  \
        .ems_pwm_transition_cb = _transition_cb  \
    };

typedef struct{
  int16_t present_duty;
  int16_t initial_present_duty;
  int16_t target_duty;
  app_transition_t transition;
}app_ems_pwm_state_t;

typedef struct __app_ems_pwm_server_t app_ems_pwm_server_t;

typedef void (*app_ems_pwm_set_cb_t)(const app_ems_pwm_server_t * p_app, int16_t duty);

typedef void (*app_ems_pwm_get_cb_t)(const app_ems_pwm_server_t * p_app, int16_t * p_present_duty);

typedef void (*app_ems_pwm_transition_cb_t)(const app_ems_pwm_server_t * p_app,
                                              uint32_t transition_time_ms,
                                              int16_t target_duty);

struct __app_ems_pwm_server_t{
  ems_pwm_server_t server;
  app_timer_id_t const * p_timer_id;
  app_ems_pwm_set_cb_t ems_pwm_set_cb;
  app_ems_pwm_get_cb_t ems_pwm_get_cb;
  app_ems_pwm_transition_cb_t ems_pwm_transition_cb;

  app_ems_pwm_state_t state;
  uint32_t last_rtc_counter;
  bool value_updated;
};

void app_ems_pwm_status_publish(app_ems_pwm_server_t * p_app);

uint32_t app_ems_pwm_init(app_ems_pwm_server_t * p_app, uint8_t element_index);