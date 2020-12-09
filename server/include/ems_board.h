#ifndef EMS_BOARD_H__
#define EMS_BOARD_H__

#include <stdint.h>

/* pin setting */

#define DIP_SWITCH_0    (29)
#define DIP_SWITCH_1    (30)
#define DIP_SWITCH_2    (31)

#define PELTIER_HEATING_PWM_PIN   (6)
#define PELTIER_COOLING_PWM_PIN   (7)
#define PAD_VOLTAGE_PWM_PIN       (17)
#define PAD_RIGHT_PWM_PIN         (21)//(19)
#define PAD_LEFT_PWM_PIN          (20)

#define TEMPERATURE_ANALOG_P_PIN    (NRF_SAADC_INPUT_AIN0)//P0.02
#define TEMPERATURE_ANALOG_N_PIN    (NRF_SAADC_INPUT_AIN1)//P0.03
#define TEMPERATURE_ANALOG_VIN_PIN  (NRF_SAADC_INPUT_AIN2)//P0.04
#define PELTIER_VOLTAGE_ANALOG_PIN  (NRF_SAADC_INPUT_AIN3)//P0.05
#define PAD_VOLTAGE_ANALOG_PIN      (NRF_SAADC_INPUT_AIN4)//P0.28

#define POWER_CONTROL_PIN (13)

#define UP_BUTTON         (15)//(25)
#define DOWN_BUTTON       (16)//(26)
#define POWER_BUTTON      (11)
#define MODE_BUTTON       (POWER_BUTTON)

#define BLUE_LED          (19)//(23)
#define WHITE_LED         (18)//(24)

/* */

#define BUTTON_CONTROL    (0)
#define BLE_CONTROL       (1)

typedef struct
{
    uint8_t   control_mode;
    uint8_t   position;

    double    pad_target_voltage;
    double    peltier_target_voltage;
}board_state;

void button_control_mode(board_state * p_board);
void ble_control_mode(board_state * p_board);

void board_turn_off(void);


#endif