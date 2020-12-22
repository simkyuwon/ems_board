#include "ems_board.h"
#include "ems_gpio.h"

void button_control_mode(board_state * p_board)
{
    if(p_board->control_mode == BLE_CONTROL)
    {
        p_board->control_mode = BUTTON_CONTROL;
    }
}

void ble_control_mode(board_state * p_board)
{
    if(p_board->control_mode == BUTTON_CONTROL)
    {
        p_board->control_mode = BLE_CONTROL;
    }
}

void board_turn_off(void)
{
    gpio_pin_write(POWER_CONTROL_PIN, GPIO_LOW);
}