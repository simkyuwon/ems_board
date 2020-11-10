#ifndef EMS_BUTTON_H__
#define EMS_BUTTON_H__

#include "nrf_drv_gpiote.h"
#include "nrf_assert.h"
#include "ems_board.h"

bool button_event_init(nrfx_gpiote_pin_t pin_number);

#endif