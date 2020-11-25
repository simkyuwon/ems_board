#ifndef EMS_PWM_H__
#define EMS_PWM_H__

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "nrf_drv_pwm.h"
#include "nrf_drv_ppi.h"
#include "log.h"

#define PWM_NOPIN         (0xFFFFFFFF)

#define WAVEFORM_PWM_CHANNELS_PER_INSTANCE  (2)
#define PELTIER_PWM_CHANNELS_PER_INSTANCE   (2)

#define WAVEFORM_PWM_CONFIG(period_in_us, width_in_us, count_in, pin0, pin1) \
    {                                                                        \
        .pins               = {pin0, pin1},                                  \
        .pulse_period_us    = period_in_us,                                  \
        .pulse_width_us     = width_in_us,                                   \
        .pulse_count        = count_in                                       \
    }

#define PAD_VOLTAGE_PWM_CONFIG(pin_in, p_seq_in)  \
    {                                             \
        .pin                = pin_in,             \
        .p_seq              = p_seq_in            \
    }

#define PWM_SEQUENCE_CONFIG(p_origin_dma_in, period_ms_in)                \
    {                                                                     \
        .p_origin_dma       = p_origin_dma_in,                            \
        .count              = sizeof(p_origin_dma_in) / sizeof(uint16_t), \
        .period_ms          = period_ms_in                                \
    }

#define PELTIER_PWM_CONFIG(period_in_us, heating_pin, cooling_pin)  \
    {                                                               \
        .pins               = {heating_pin, cooling_pin},           \
        .period_us          = period_in_us,                         \
    }

#define WAVEFORM_PWM_NUMBER     (0)
#define PAD_VOLTAGE_PWM_NUMBER  (1)
#define PELTIER_PWM_NUMBER      (2)

#define PWM_POLARITY_Msk        (0x8000UL)
#define PWM_COMPARE_Msk         (0x7FFFUL)

#define PWM_PIN_NOT_CONNECTED   (0x1FUL)

#define PAD_VOLTAGE_COUNTER_TOP (1000)
#define PAD_VOLTAGE_PERIOD_KHZ  (8)

#define PAD_VOLTAGE_LEVEL_MAX   (10)
#define PAD_VOLTAGE_LEVEL_MIN   (0)

typedef enum
{
    PWM_POLARITY_ACTIVE_LOW = 0,
    PWM_POLARITY_ACTIVE_HIGH = 1 << 15,
}pwm_polarity_t;

typedef struct
{
    const uint16_t *      p_origin_dma;
    uint16_t *            p_dma;
    uint32_t              count;
    uint32_t              period_ms;
}pwm_sequence_config_t;

typedef struct
{
    uint32_t        pins[WAVEFORM_PWM_CHANNELS_PER_INSTANCE];
    uint32_t        pulse_period_us;
    uint32_t        pulse_width_us;
    uint32_t        pulse_count;
}waveform_pwm_config_t;

typedef struct
{
    uint32_t                    pin;
    pwm_sequence_config_t *     p_seq;
}pad_voltage_pwm_config_t;

typedef struct
{
    uint32_t      pins[PELTIER_PWM_CHANNELS_PER_INSTANCE];
    uint32_t      period_us;
}peltier_pwm_config_t;

static NRF_PWM_Type* nrf_pwm_base(const uint32_t pwm_number);


bool waveform_pwm_init(const uint32_t pwm_number, const waveform_pwm_config_t * const p_config);

bool waveform_pulse_count_change(const uint32_t pwm_number, waveform_pwm_config_t * const p_config, uint16_t count);

bool waveform_pulse_period_change(const uint32_t pwm_number, waveform_pwm_config_t * const p_config, uint16_t period_us);


bool pad_voltage_pwm_init(const uint32_t pwm_number, const pad_voltage_pwm_config_t * const p_config);

bool pad_voltage_sequence_init(pwm_sequence_config_t * const p_config);

bool pad_voltage_sequence_mode_change(pad_voltage_pwm_config_t * const p_config, pwm_sequence_config_t * const p_seq_config);

bool pad_voltage_level_up(pwm_sequence_config_t * const p_config);

bool pad_voltage_level_down(pwm_sequence_config_t * const p_config);

bool pad_voltage_level_set(pwm_sequence_config_t * const p_config, const uint8_t level);

bool pad_voltage_period_set(pad_voltage_pwm_config_t * const p_config, const uint32_t period_ms);


bool peltier_pwm_init(const uint32_t pwm_number, const peltier_pwm_config_t * const p_config);

bool peltier_heating(uint32_t duty);

bool peltier_cooling(uint32_t duty);

bool peltier_stop(void);


bool pwm_single_shot(const uint32_t pwm_number);

bool pwm_start(const uint32_t pwm_number);

bool pwm_stop(const uint32_t pwm_number);

#endif