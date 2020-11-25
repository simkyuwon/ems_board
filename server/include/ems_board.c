#include "ems_board.h"
#include "ems_gpio.h"

void button_control_mode(board_state * p_board)
{
    if(p_board->control_mode == BLE_CONTROL)
    {
        p_board->control_mode = BUTTON_CONTROL;
        gpio_pin_write(BLUE_LED, GPIO_LOW);
    }
}

void ble_control_mode(board_state * p_board)
{
    if(p_board->control_mode == BUTTON_CONTROL)
    {
        p_board->control_mode = BLE_CONTROL;
        gpio_pin_write(BLUE_LED, GPIO_HIGH);
    }
}

void board_turn_off(void)
{
    gpio_pin_write(POWER_CONTROL_PIN, GPIO_LOW);
}