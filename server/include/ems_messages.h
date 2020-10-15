#include <stdint.h>
#include "generic_level_messages.h"

typedef struct __attribute((packed)){
  int16_t pwm_duty;
  uint8_t tid;
  uint8_t transition_time;
  uint8_t delay;
}ems_pwm_set_msg_pkt_t;

typedef struct __attribute((packed)){
  int32_t delta_pwm_duty;
  uint8_t tid;
  uint8_t transition_time;
  uint8_t delay;
}ems_pwm_delta_set_msg_pkt_t;

typedef struct __attribute((packed)){
  int16_t move_pwm_duty;
  uint8_t tid;
  uint8_t transition_time;
  uint8_t delay;
}ems_pwm_move_set_msg_pkt_t;

typedef struct __attribute((packed)){
  int16_t present_pwm_duty;
  int16_t target_pwm_duty;
  uint8_t remaining_time;
}ems_pwm_status_msg_pkt_t;