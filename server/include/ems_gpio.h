#ifndef EMS_GPIO_H__
#define EMS_GPIO_H__

#include "nrf_gpio.h"
#include "ems_board.h"

#define DIP_SWITCH_GPIO_CONFIG(pin_num, pull_in)  \
    {                                             \
        .pin_number = pin_num,                    \
        .dir = NRF_GPIO_PIN_DIR_INPUT,            \
        .input = NRF_GPIO_PIN_INPUT_CONNECT,      \
        .pull = pull_in,                          \
        .drive = NRF_GPIO_PIN_S0S1,               \
        .sense = NRF_GPIO_PIN_NOSENSE             \
    }                                             \

typedef struct
{
    uint32_t pin_number;
    nrf_gpio_pin_dir_t dir;
    nrf_gpio_pin_input_t input;
    nrf_gpio_pin_pull_t pull;
    nrf_gpio_pin_drive_t drive;
    nrf_gpio_pin_sense_t sense;
}gpio_config_t;

bool dip_switch_gpio_init(void);
void read_dip_switch(uint32_t * const pin_input);
bool gpio_pin_init(const gpio_config_t * const p_config);

#endif