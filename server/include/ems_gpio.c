#include "ems_gpio.h"

bool dip_switch_gpio_init(void)
{
    gpio_config_t dip_switch_0 = DIP_SWITCH_GPIO_CONFIG(DIP_SWITCH_0, GPIO_PIN_CNF_PULL_Pullup); 
    gpio_config_t dip_switch_1 = DIP_SWITCH_GPIO_CONFIG(DIP_SWITCH_1, GPIO_PIN_CNF_PULL_Pullup); 
    gpio_config_t dip_switch_2 = DIP_SWITCH_GPIO_CONFIG(DIP_SWITCH_2, GPIO_PIN_CNF_PULL_Pullup); 

    if(!gpio_pin_init(&dip_switch_0) ||
       !gpio_pin_init(&dip_switch_1) ||
       !gpio_pin_init(&dip_switch_2))
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

bool gpio_pin_init(const gpio_config_t * const p_config)
{
    if(p_config->pin_number >= P0_PIN_NUM)
    {
        return false;
    }

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