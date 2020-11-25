#include "ems_gpio.h"

static uint32_t gpio_pin_used = 0;
static nrf_ppi_channel_t pulse_generator_ppi_channel[2];

bool dip_switch_gpio_init(uint32_t pin_number)
{
    gpio_config_t dip_switch = DIP_SWITCH_GPIO_CONFIG(pin_number,                 //input pin
                                                      GPIO_PIN_CNF_PULL_Pullup);  //resistor pullup

    if(!gpio_pin_init(&dip_switch))
    {
        return false;
    }

    return true;
}

void read_dip_switch(uint32_t * const pin_input)
{
    *pin_input = ~(((NRF_P0->IN >> DIP_SWITCH_0) & 0x1) |
                 ((NRF_P0->IN >> (DIP_SWITCH_1 - 1)) & 0x2) |
                 ((NRF_P0->IN >> (DIP_SWITCH_2 - 2)) & 0x4)) & 0x7;
}

gpio_pin_state gpio_pin_read(const uint32_t pin_number)
{
    return (NRF_P0->IN >> pin_number) & 0x1;
}

bool gpio_pin_write(const uint32_t pin_number, gpio_pin_state state)
{
    if(pin_number >= P0_PIN_NUM || !(gpio_pin_used & (1UL << pin_number)))
    {
        return false;
    }

    NRF_P0->OUT = state << pin_number;

    return true;
}

bool gpio_pin_init(const gpio_config_t * const p_config)
{
    if(p_config->pin_number >= P0_PIN_NUM || (gpio_pin_used & (1UL << p_config->pin_number)))
    {
        return false;
    }
    gpio_pin_used |= 1UL << p_config->pin_number;

    NRF_P0->PIN_CNF[p_config->pin_number] = (p_config->dir << GPIO_PIN_CNF_DIR_Pos) |
                                            (p_config->input << GPIO_PIN_CNF_INPUT_Pos) |
                                            (p_config->pull << GPIO_PIN_CNF_PULL_Pos) |
                                            (p_config->drive << GPIO_PIN_CNF_DRIVE_Pos) |
                                            (p_config->sense << GPIO_PIN_CNF_SENSE_Pos);

    if(p_config->dir == GPIO_PIN_CNF_DIR_Output)
    {
        NRF_P0->OUTCLR = (GPIO_OUTCLR_PIN0_Clear << p_config->pin_number);
    }

    return true;
}