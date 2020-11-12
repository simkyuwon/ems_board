#ifndef EMS_GPIO_H__
#define EMS_GPIO_H__

#include "nrf_drv_timer.h"
#include "ems_gpiote.h"
#include "ems_board.h"

#define DIP_SWITCH_GPIO_CONFIG(pin_num, pull_in)  \
    {                                             \
        .pin_number = pin_num,                    \
        .dir = GPIO_PIN_CNF_DIR_Input,            \
        .input = GPIO_PIN_CNF_INPUT_Connect,      \
        .pull = pull_in,                          \
        .drive = GPIO_PIN_CNF_DRIVE_S0S1,         \
        .sense = GPIO_PIN_CNF_SENSE_Disabled      \
    }                                             \

#define BUTTON_GPIO_CONFIG(pin_num, pull_in)      \
    {                                             \
        .pin_number = pin_num,                    \
        .dir = GPIO_PIN_CNF_DIR_Input,            \
        .input = GPIO_PIN_CNF_INPUT_Connect,      \
        .pull = pull_in,                          \
        .drive = GPIO_PIN_CNF_DRIVE_S0S1,         \
        .sense = GPIO_PIN_CNF_SENSE_Disabled      \
    }                                             \

#define PULSE_GPIO_CONFIG(pin_num)                \
    {                                             \
        .pin_number = pin_num,                    \
        .dir = GPIO_PIN_CNF_DIR_Output,           \
        .input = GPIO_PIN_CNF_INPUT_Disconnect,   \
        .pull = GPIO_PIN_CNF_PULL_Disabled,       \
        .sense = GPIO_PIN_CNF_SENSE_Disabled      \
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

bool dip_switch_gpio_init(uint32_t pin_number);

void read_dip_switch(uint32_t * const pin_input);

bool pulse_generator_init(const uint32_t pin_number);

bool gpio_pin_init(const gpio_config_t * const p_config);

#endif