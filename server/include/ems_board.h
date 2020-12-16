#ifndef EMS_BOARD_H__
#define EMS_BOARD_H__

#include <stdint.h>

/* pin setting */

#define DIP_SWITCH_0    (29UL)
#define DIP_SWITCH_1    (30UL)
#define DIP_SWITCH_2    (31UL)

#define PELTIER_HEATING_PWM_PIN   (6UL)
#define PELTIER_COOLING_PWM_PIN   (7UL)
#define PAD_VOLTAGE_PWM_PIN       (17UL)
#define PAD_RIGHT_PWM_PIN         (19UL)
#define PAD_LEFT_PWM_PIN          (20UL)

#define TEMPERATURE_ANALOG_P_PIN    (NRF_SAADC_INPUT_AIN0)//P0.02
#define TEMPERATURE_ANALOG_N_PIN    (NRF_SAADC_INPUT_AIN1)//P0.03
#define TEMPERATURE_ANALOG_VIN_PIN  (NRF_SAADC_INPUT_AIN2)//P0.04
#define PELTIER_VOLTAGE_ANALOG_PIN  (NRF_SAADC_INPUT_AIN3)//P0.05
#define PAD_VOLTAGE_ANALOG_PIN      (NRF_SAADC_INPUT_AIN4)//P0.28

#define POWER_CONTROL_PIN (13UL)

#define UP_BUTTON         (25UL)
#define DOWN_BUTTON       (26UL)
#define POWER_BUTTON      (11UL)
#define MODE_BUTTON       (POWER_BUTTON)

#define BLUE_LED          (23UL)
#define WHITE_LED         (24UL)

typedef enum
{
    BUTTON_CONTROL  = 0UL,
    BLE_CONTROL     = 1UL,
}board_control;

typedef struct
{
    board_control   control_mode;
    uint8_t         position;
    double          pad_target_voltage;
    double          peltier_target_temperature;
}board_state;

void button_control_mode(board_state * p_board);
void ble_control_mode(board_state * p_board);

void board_turn_off(void);


#endif