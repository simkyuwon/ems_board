#ifndef EMS_PWM_H_
#define EMS_PWM_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "nrf_assert.h"
#include "nrf_drv_pwm.h"
#include "log.h"

#define PWM_NOPIN         0xFFFFFFFF

#define WAVEFORM_PWM_CHANNELS_PER_INSTANCE 2

#define WAVEFORM_PWM_CONFIG(period_in_us, width_in_us, count_in, pin0, pin1) \
    {                                                                        \
        .pins               = {pin0, pin1},                                  \
        .pulse_period_us    = period_in_us,                                  \
        .pulse_width_us     = width_in_us,                                   \
        .pulse_count        = count_in                                       \
    }                                                                        \

#define VOLTAGE_PWM_CONFIG(voltage_in, pin_in)  \
    {                                           \
        .pin  = pin_in,                         \
        .voltage  = voltage_in                  \
    }                                           \


#define WAVEFORM0_PWM_NUMBER  (0)
#define WAVEFORM1_PWM_NUMBER  (1)
#define VOLTAGE_PWM_NUMBER    (2)

#define PWM_POLARITY_Msk 0x8000UL
#define PWM_COMPARE_Msk 0x7FFFUL

typedef enum
{
    PWM_POLARITY_ACTIVE_LOW = 0,
    PWM_POLARITY_ACTIVE_HIGH = 1 << 15,
}pwm_polarity_t;

typedef struct
{
    uint32_t        pins[WAVEFORM_PWM_CHANNELS_PER_INSTANCE];
    uint32_t        pulse_period_us;
    uint32_t        pulse_width_us;
    uint32_t        pulse_count;
}waveform_pwm_config_t;

static NRF_PWM_Type* nrf_pwm_base(const uint32_t pwm_number);

bool waveform_pwm_init(const uint32_t pwm_number, const waveform_pwm_config_t * const p_config);

bool waveform_pulse_count_change(const uint32_t pwm_number, waveform_pwm_config_t * const p_config, uint16_t count);

bool waveform_pulse_period_change(const uint32_t pwm_number, waveform_pwm_config_t * const p_config, uint16_t period_us);

typedef struct
{
    uint32_t    pin;
    uint32_t    voltage;
}voltage_pwm_config_t;

bool voltage_pwm_init(const uint32_t pwm_number, const voltage_pwm_config_t * const p_config);

bool change_voltage(int16_t duty);

bool pwm_start(const uint32_t pwm_number);

bool pwm_stop(const uint32_t pwm_number);

#endif