#ifndef EMS_COMMON_H__
#define EMS_COMMON_H__

#include <stdint.h>

#define EMS_PWM_COMPANY_ID 0xFFFF

typedef struct{
  int16_t pwm_duty;
  uint8_t tid;
}ems_pwm_state_t;

typedef struct{
  int16_t pwm_duty;
  uint8_t tid;
}ems_pwm_set_params_t;

typedef struct{
  int32_t delta_pwm_duty;
  uint8_t tid;
}ems_pwm_delta_set_params_t;

typedef struct{
  int16_t move_pwm_duty;
  uint8_t tid;
}ems_pwm_move_set_params_t;

typedef struct{
  int16_t present_pwm_duty;
  int16_t target_pwm_duty;
  int16_t remaining_time_ms;
}ems_pwm_status_params_t;
#endif