#include "ems_gpiote.h"

static uint16_t gpiote_count = 0;
static gpiote_button_event_config_t gpiote_event_config_array[GPIOTE_CH_NUM];

int gpiote_config_init(const gpiote_config_t * const p_config, button_cb callback)
{
    if(gpiote_count >= GPIOTE_CH_NUM)
    {
        return -1;
    }

    NRF_GPIOTE->CONFIG[gpiote_count] = (p_config->mode << GPIOTE_CONFIG_MODE_Pos) |
                                       (p_config->psel << GPIOTE_CONFIG_PSEL_Pos) |
                                       (p_config->polarity << GPIOTE_CONFIG_POLARITY_Pos) |
                                       (p_config->outinit << GPIOTE_CONFIG_OUTINIT_Pos);

    if(p_config->mode == GPIOTE_CONFIG_MODE_Event)
    {
        NRF_GPIOTE->INTENSET = (GPIOTE_INTENCLR_IN0_Enabled << gpiote_count);
        NRF_GPIOTE->EVENTS_IN[gpiote_count] = 0;
    }

    gpiote_event_config_array[gpiote_count].cb = callback;

    return gpiote_count++;
}

bool button_event_init(uint32_t pin_number, button_cb callback)
{
    if(pin_number >= P0_PIN_NUM)
    {
        return false;
    }

    gpio_config_t m_gpio_config = BUTTON_GPIO_CONFIG(pin_number, GPIO_PIN_CNF_PULL_Pullup);
    gpio_pin_init(&m_gpio_config);

    gpiote_config_t m_gpiote_config = BUTTON_GPIOTE_CONFIG(pin_number, GPIOTE_CONFIG_POLARITY_HiToLo);
    if(gpiote_config_init(&m_gpiote_config, callback) < 0)
    {
        return false;
    }

    sd_nvic_SetPriority(GPIOTE_IRQn, 6);
    sd_nvic_EnableIRQ(GPIOTE_IRQn);

    return true;
}

void GPIOTE_IRQHandler(void)
{
    for(uint32_t ch_num = 0; ch_num < GPIOTE_CH_NUM; ch_num++)
    {
        if(NRF_GPIOTE->EVENTS_IN[ch_num])
        {
            if(gpiote_event_config_array[ch_num].cb != NULL)
            {
                gpiote_event_config_array[ch_num].cb();
            }
            NRF_GPIOTE->EVENTS_IN[ch_num] = 0;
        }
    }
}