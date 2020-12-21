#ifndef EMS_GPIO_H__
#define EMS_GPIO_H__

#include "nrf_drv_timer.h"
#include "ems_gpiote.h"
#include "ems_board.h"
#include "ems_rtc.h"

#define DIP_SWITCH_GPIO_CONFIG(pin_num, pull_in)      \
    {                                                 \
        .pin_number = pin_num,                        \
        .dir        = GPIO_PIN_CNF_DIR_Input,         \
        .input      = GPIO_PIN_CNF_INPUT_Connect,     \
        .pull       = pull_in,                        \
        .drive      = GPIO_PIN_CNF_DRIVE_S0S1,        \
        .sense      = GPIO_PIN_CNF_SENSE_Disabled     \
    }

#define BUTTON_GPIO_CONFIG(pin_num, pull_in)          \
    {                                                 \
        .pin_number = pin_num,                        \
        .dir        = GPIO_PIN_CNF_DIR_Input,         \
        .input      = GPIO_PIN_CNF_INPUT_Connect,     \
        .pull       = pull_in,                        \
        .drive      = GPIO_PIN_CNF_DRIVE_S0S1,        \
        .sense      = GPIO_PIN_CNF_SENSE_Disabled     \
    }

#define LED_GPIO_CONFIG(pin_num, pull_in)             \
    {                                                 \
        .pin_number = pin_num,                        \
        .dir        = GPIO_PIN_CNF_DIR_Output,        \
        .input      = GPIO_PIN_CNF_INPUT_Disconnect,  \
        .pull       = pull_in,                        \
        .drive      = GPIO_PIN_CNF_DRIVE_S0S1,        \
        .sense      = GPIO_PIN_CNF_SENSE_Disabled     \
    }

#define BOARD_POWER_GPIO_CONFIG(pin_num)              \
    {                                                 \
        .pin_number = pin_num,                        \
        .dir        = GPIO_PIN_CNF_DIR_Output,        \
        .input      = GPIO_PIN_CNF_INPUT_Disconnect,  \
        .pull       = GPIO_PIN_CNF_PULL_Pullup,       \
        .drive      = GPIO_PIN_CNF_DRIVE_S0S1,        \
        .sense      = GPIO_PIN_CNF_SENSE_Disabled     \
    }

    
#define LED_STATE_CONFIG(pin_num, type)               \
    {                                                 \
        .pin_number = pin_num,                        \
        .period_ms  = 500,                            \
        .state      = LED_OFF,                        \
        .pin_type   = type                            \
    }

typedef struct
{
    uint32_t              pin_number;
    nrf_gpio_pin_dir_t    dir;
    nrf_gpio_pin_input_t  input;
    nrf_gpio_pin_pull_t   pull;
    nrf_gpio_pin_drive_t  drive;
    nrf_gpio_pin_sense_t  sense;
}gpio_config_t;

typedef enum
{
    GPIO_LOW  = 0UL,
    GPIO_HIGH = 1UL
}gpio_pin_state;

typedef enum
{
    LED_OFF,
    LED_ON,
    LED_BLINK,
}led_state;

typedef enum
{
    CATHODE,    //VDD --LED-> MCU
    ANODE       //GND <-LED-- MCU
}led_type;

typedef struct
{
    uint32_t    pin_number;
    uint32_t    period_ms;
    led_state   state;
    led_type    pin_type;
}led_config_t;


bool gpio_pin_init(const gpio_config_t * const p_config);

bool dip_switch_gpio_init(uint32_t pin_number);

void read_dip_switch(uint32_t * const pin_input);

gpio_pin_state gpio_pin_read(const uint32_t pin_number);

bool gpio_pin_write(const uint32_t pin_number, gpio_pin_state state);

bool gpio_pin_toggle(const uint32_t pin_number);

bool led_state_set(led_config_t * const p_config, led_state state);

#endif