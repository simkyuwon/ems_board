#ifndef EMS_BOARD_H__
#define EMS_BOARD_H__

#define DIP_SWITCH_0 (6)
#define DIP_SWITCH_1 (7)
#define DIP_SWITCH_2 (8)

#define PULSE_GENERATOR_PIN (11)

#define PWM_0_L_PIN (17)
#define PWM_0_R_PIN (18)
#define PWM_1_L_PIN (19)
#define PWM_1_R_PIN (21)

#define PWM_VOLTAGE_PIN (12)

#define VOLTAGE_ANALOG_PIN          (NRF_SAADC_INPUT_AIN0)
#define TEMPERATURE_ANALOG_P_PIN    (NRF_SAADC_INPUT_AIN1)
#define TEMPERATURE_ANALOG_N_PIN    (NRF_SAADC_INPUT_AIN2)
#define TEMPERATURE_ANALOG_VSS_PIN  (NRF_SAADC_INPUT_AIN3)

#define BUTTON_1          (13)
#define BUTTON_2          (14)
#define BUTTON_3          (15)
#define BUTTON_4          (16)

#define VOLTAGE_SAADC_CHANNEL             (0)
#define TEMPERATURE_SENSOR_SAADC_CHANNEL  (1)
#define TEMPERATURE_VSS_SAADC_CHANNEL     (2)

#define BOARD_LED_0   (20)

#define PULSE_GENERATOR_TIMER (NRF_TIMER4)

#define BUTTON_CONTROL  (0)
#define BLE_CONTROL     (1)

typedef struct
{
    uint8_t   control_mode;
    uint8_t  position;
}board_state;

#endif