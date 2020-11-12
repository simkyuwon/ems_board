#ifndef EMS_GPIOTE_H__
#define EMS_GPIOTE_H__

#include "nrf_drv_gpiote.h"
#include "nrf_drv_ppi.h"
#include "ems_gpio.h"
#include "ems_board.h"

#define BUTTON_GPIOTE_CONFIG(pin_in, polarity_in)  \
    {                                              \
        .mode = GPIOTE_CONFIG_MODE_Event,          \
        .psel = pin_in,                            \
        .polarity = polarity_in                    \
    }                                              \

#define PULSE_GPIOTE_CONFIG(pin_in, polarity_in, outinit_in)  \
    {                                                         \
        .mode = GPIOTE_CONFIG_MODE_Task,                      \
        .psel = pin_in,                                       \
        .polarity = polarity_in,                              \
        .outinit = outinit_in                                 \
    }                                                         \

typedef struct
{
    uint8_t     mode;
    uint32_t    psel;
    uint8_t     polarity;
    uint8_t     outinit;
}gpiote_config_t;

//typedef bool (*button_cb)(const pwm_sequence_config_t * const p_config);
typedef void (*button_cb)(void);

typedef struct
{
    button_cb cb;
}gpiote_button_event_config_t;

int gpiote_config_init(const gpiote_config_t * const p_config, button_cb callback);
bool button_event_init(uint32_t pin_number, button_cb callback);

#endif