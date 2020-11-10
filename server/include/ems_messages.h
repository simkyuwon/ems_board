#ifndef EMS_MESSAGES_H__
#define EMS_MESSAGES_H__

#include <stdint.h>

#define EMS_PWM_SET_MINLEN 3
#define EMS_PWM_SET_MAXLEN 5
#define EMS_PWM_STATUS_MINLEN 2
#define EMS_PWM_STATUS_MAXLEN 5

typedef enum
{
  EMS_PWM_OPCODE_GET = 0x8205,
  EMS_PWM_OPCODE_SET = 0x8206,
  EMS_PWM_OPCODE_SET_UNACKNOWLEDGED = 0x8207,
  EMS_PWM_OPCODE_STATUS = 0x8208,
} ems_pwm_opcode_t;

typedef struct __attribute((packed)){
  int16_t pwm_duty;
  uint8_t tid;
  uint8_t transition_time;
  uint8_t delay;
}ems_pwm_set_msg_pkt_t;

typedef struct __attribute((packed)){
  int16_t present_pwm_duty;
  int16_t target_pwm_duty;
  uint8_t remaining_time;
}ems_pwm_status_msg_pkt_t;

#endif