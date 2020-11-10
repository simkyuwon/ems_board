#include "nrf_drv_gpiote.h"
#include "nrf_assert.h"

#define VOLTAGE_UP_PIN 13
#define VOLTAGE_DOWN_PIN 14

bool button_event_init(nrfx_gpiote_pin_t pin_number);