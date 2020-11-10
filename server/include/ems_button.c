#include "ems_button.h"

bool button_event_init(nrfx_gpiote_pin_t pin_number)
{
    if(pin_number >= P0_PIN_NUM)
    {
        return false;
    }

    NRF_P0->PIN_CNF[pin_number] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
                                  (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
                                  (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos);

    NRF_GPIOTE->CONFIG[0] = (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos) |
                            (pin_number << GPIOTE_CONFIG_PSEL_Pos) |
                            (GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos);

    NRF_GPIOTE->INTENSET = (GPIOTE_INTENSET_IN0_Enabled << GPIOTE_INTENSET_IN0_Pos);

    NVIC_ClearPendingIRQ(GPIOTE_IRQn);
    NVIC_EnableIRQ(GPIOTE_IRQn);
}

void GPIOTE_IRQHandler()
{
    NRF_GPIOTE->EVENTS_IN[0] = 0;
    printf("GPIOTE_Handler\n");
}
