#include "ems_button.h"

static uint16_t gpiote_count = 0;

bool button_event_init(nrfx_gpiote_pin_t pin_number)
{
    if(pin_number >= P0_PIN_NUM || gpiote_count >= GPIOTE_CH_NUM)
    {
        return false;
    }

    NRF_P0->PIN_CNF[pin_number] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
                                  (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
                                  (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos);

    NRF_GPIOTE->CONFIG[gpiote_count] = (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos) |
                                         (pin_number << GPIOTE_CONFIG_PSEL_Pos) |
                                         (GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos);

    NRF_GPIOTE->INTENSET = (GPIOTE_INTENSET_IN0_Enabled << gpiote_count);
    gpiote_count++;

    NVIC_ClearPendingIRQ(GPIOTE_IRQn);
    NVIC_EnableIRQ(GPIOTE_IRQn);
}
