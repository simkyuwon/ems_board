#ifndef EMS_PWM_H__
#define EMS_PWM_H__

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "ems_board.h"
#include "nrf_drv_ppi.h"

#define WAVEFORM_PWM_CHANNELS_PER_INSTANCE  (2)
#define PELTIER_PWM_CHANNELS_PER_INSTANCE   (2)

#define WAVEFORM_PWM_CONFIG(period_in_us, width_in_us, count_in, pin0, pin1) \
    {                                                                        \
        .pins               = {pin0, pin1},                                  \
        .pulse_period_us    = period_in_us,                                  \
        .pulse_width_us     = width_in_us,                                   \
        .pulse_count        = count_in                                       \
    }

#define PAD_VOLTAGE_PWM_CONFIG(pin_in, p_seq_in, counter_in)  \
    {                                                         \
        .pin                = pin_in,                         \
        .p_seq              = p_seq_in,                       \
        .counter            = counter_in,                     \
        .dma                = 0 | PWM_POLARITY_ACTIVE_HIGH    \
    }

#define PWM_SEQUENCE_CONFIG(p_sequence_in, period_ms_in)                  \
    {                                                                     \
        .p_sequence         = p_sequence_in,                              \
        .seq_size           = sizeof(p_sequence_in) / sizeof(uint16_t),   \
        .period_ms          = period_ms_in                                \
    }

#define PELTIER_PWM_CONFIG(period_in_us, heating_pin, cooling_pin)  \
    {                                                               \
        .pins               = {heating_pin, cooling_pin},           \
        .period_us          = period_in_us,                         \
    }

#define WAVEFORM_PWM_NUMBER     (0UL)
#define PAD_VOLTAGE_PWM_NUMBER  (1UL)
#define PELTIER_PWM_NUMBER      (2UL)

#define PWM_POLARITY_Msk        (0x8000UL)
#define PWM_COMPARE_Msk         (0x7FFFUL)

#define PWM_NO_OUTPUT_COMPARE   (0x7FFFUL)

#define PWM_PIN_NOT_CONNECTED   (0x1FUL)

#define WAVEFORM_WIDTH_US_MAX   (1000UL)
#define WAVEFORM_WIDTH_US_MIN   (100UL)

#define PAD_VOLTAGE_MAX   (50.0F)
#define PAD_VOLTAGE_MIN   (3.3F)

#define PAD_VOLTAGE_PRESCALER       (PWM_PRESCALER_PRESCALER_DIV_1) //16MHz(source clock) / 1(prescaler) = 16MHz(clock)
#define PAD_VOLTAGE_COUNTER_TOP     (320UL)                         //16MHz(clock) / 320(counter top) = 50KHz
#define PAD_VOLTAGE_COMP_MAX        (192L)                          //duty 60%(60~65V)
#define PAD_VOLTAGE_COMP_MIN        (0L)

#define PAD_VOLTAGE_HZ              (16000000UL / (1UL << PAD_VOLTAGE_PRESCALER) / PAD_VOLTAGE_COUNTER_TOP)

#define PAD_VOLTAGE_PERIOD_MS_MIN   (10UL)

#define PAD_VOLTAGE_CONTROL_TERM    (PAD_VOLTAGE_HZ / 10000 + (PAD_VOLTAGE_HZ < 10000 ? 1 : 0))//10KHz

#define PAD_VOLTAGE_SEQ_COUNTER     (NRF_TIMER4)

typedef enum
{
    PWM_POLARITY_ACTIVE_LOW   = 0U,        //first edge within the PWM period is rising edge
    PWM_POLARITY_ACTIVE_HIGH  = 1U << 15,  //first edge within the PWM period is falling edge
}pwm_polarity_t;

typedef enum
{
    OFF = 0U,
    ON,
    SINGLE_SHOT,
}pwm_state_t;

typedef struct
{
    const uint16_t *      p_sequence;
    uint32_t              period_ms;
    uint32_t              seq_size;
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
    uint16_t                    dma;
    NRF_TIMER_Type*             counter;
}pad_voltage_pwm_config_t;

typedef struct
{
    uint32_t      pins[PELTIER_PWM_CHANNELS_PER_INSTANCE];
    uint32_t      period_us;
}peltier_pwm_config_t;

static NRF_PWM_Type* nrf_pwm_base(const uint32_t pwm_number);


bool waveform_pwm_init(const uint32_t pwm_number, const waveform_pwm_config_t * const p_config);

bool waveform_pulse_count_set(const uint32_t pwm_number, waveform_pwm_config_t * const p_config, uint16_t count);

bool waveform_pulse_period_set(const uint32_t pwm_number, waveform_pwm_config_t * const p_config, uint32_t period_us);

bool waveform_pulse_width_set(const uint32_t pwm_number, waveform_pwm_config_t * const p_config, uint16_t width_us);


bool pad_voltage_pwm_init(const uint32_t pwm_number, const pad_voltage_pwm_config_t * const p_config);

void pad_voltage_up(board_state * const p_board);

void pad_voltage_down(board_state * const p_board);

bool pad_voltage_set(board_state * const p_board, const double voltage);

bool pad_voltage_sequence_period_set(pad_voltage_pwm_config_t * const p_config, const uint32_t period_ms);

bool pad_voltage_sequence_mode_set(pad_voltage_pwm_config_t * const p_config, const pwm_sequence_config_t * const p_seq_config);

bool pad_voltage_comp_set(pad_voltage_pwm_config_t * const p_config, const uint16_t comp);

bool pad_voltage_period_set(const uint32_t clock);

bool pad_voltage_duty_set(pad_voltage_pwm_config_t * const p_config, const double duty);


bool peltier_pwm_init(const uint32_t pwm_number, const peltier_pwm_config_t * const p_config);

bool peltier_heating(uint32_t duty);

bool peltier_cooling(uint32_t duty);

bool peltier_stop(void);


bool waveform_single_shot(const uint32_t count);

bool pwm_start(const uint32_t pwm_number);

bool pwm_stop(const uint32_t pwm_number);

pwm_state_t pwm_state_get(const uint32_t pwm_number);

#endif