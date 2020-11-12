#include "ems_gpio.h"

static uint32_t gpio_pin_used = 0;
static nrf_ppi_channel_t pulse_generator_ppi_channel;

bool dip_switch_gpio_init(uint32_t pin_number)
{
    gpio_config_t dip_switch = DIP_SWITCH_GPIO_CONFIG(pin_number, GPIO_PIN_CNF_PULL_Pullup); 

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

bool pulse_generator_init(const uint32_t pin_number)
{
    if(pin_number >= P0_PIN_NUM)
    {
        return false;
    }
    
    gpio_config_t m_gpio_config = PULSE_GPIO_CONFIG(pin_number);
    if(!gpio_pin_init(&m_gpio_config))
    {
        return false;
    }

    gpiote_config_t m_gpiote_config = PULSE_GPIOTE_CONFIG(pin_number, GPIOTE_CONFIG_POLARITY_Toggle, GPIOTE_CONFIG_OUTINIT_Low);
    uint32_t gpiote_number = gpiote_config_init(&m_gpiote_config, NULL);
    
    if(gpiote_number < 0)
    {
        return false;
    }

    PULSE_GENERATOR_TIMER->MODE = NRF_TIMER_MODE_TIMER;
    PULSE_GENERATOR_TIMER->BITMODE = NRF_TIMER_BIT_WIDTH_8;
    PULSE_GENERATOR_TIMER->PRESCALER = NRF_TIMER_FREQ_31250Hz;

    PULSE_GENERATOR_TIMER->INTENSET = (TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos) |
                                      (TIMER_INTENSET_COMPARE1_Enabled << TIMER_INTENSET_COMPARE1_Pos);
    PULSE_GENERATOR_TIMER->CC[0] = 0;
    PULSE_GENERATOR_TIMER->CC[1] = 1;
    PULSE_GENERATOR_TIMER->SHORTS = (TIMER_SHORTS_COMPARE1_CLEAR_Enabled << TIMER_SHORTS_COMPARE1_CLEAR_Pos);

    nrf_drv_ppi_channel_alloc(&pulse_generator_ppi_channel);
    nrf_drv_ppi_channel_assign(pulse_generator_ppi_channel, (uint32_t)&(PULSE_GENERATOR_TIMER->EVENTS_COMPARE[0]), (uint32_t)&(NRF_GPIOTE->TASKS_OUT[gpiote_number]));
    nrf_drv_ppi_channel_enable(pulse_generator_ppi_channel);

    PULSE_GENERATOR_TIMER->TASKS_START = 1;

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