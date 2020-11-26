#ifndef EMS_SAADC_H__
#define EMS_SAADC_H__

#include <stdbool.h>

#include "nrf_drv_saadc.h"
#include "ems_board.h"
#include "ems_rtc.h"
#include "log.h"

#define SAADC_CHANNEL_MAX_COUNT (8)
#define SAMPLES_IN_BUFFER (32)

#define SE_SAADC_CONFIG(resistor_in, gain_in, reference_in, acq_time_in, pin_in) \
    {                                                                            \
        .resistor_p   = resistor_in,                                             \
        .gain         = gain_in,                                                 \
        .reference    = reference_in,                                            \
        .acq_time     = acq_time_in,                                             \
        .pin_p        = pin_in                                                   \
    }

#define PAD_VOLTAGE_SAADC_CONFIG(resistor_in, gain_in, reference_in, acq_time_in, pin_in) \
        SE_SAADC_CONFIG(resistor_in, gain_in, reference_in, acq_time_in, pin_in)

#define PELTIER_VOLTAGE_SAADC_CONFIG(resistor_in, gain_in, reference_in, acq_time_in, pin_in) \
        SE_SAADC_CONFIG(resistor_in, gain_in, reference_in, acq_time_in, pin_in)

#define TEMPERATURE_VIN_SAADC_CONFIG(resistor_in, gain_in, reference_in, acq_time_in, pin_in) \
        SE_SAADC_CONFIG(resistor_in, gain_in, reference_in, acq_time_in, pin_in)

#define TEMPERATURE_DIFF_SAADC_CONFIG(resistor_p_in, resistor_n_in, gain_in, reference_in, acq_time_in, pin_p_in, pin_n_in) \
    {                                                                                                                       \
        .resistor_p   = resistor_p_in,                                                                                      \
        .resistor_n   = resistor_n_in,                                                                                      \
        .gain         = gain_in,                                                                                            \
        .reference    = reference_in,                                                                                       \
        .acq_time     = acq_time_in,                                                                                        \
        .pin_p        = pin_p_in,                                                                                           \
        .pin_n        = pin_n_in                                                                                            \
    }

#define MIN_TEMPERATURE   (0)
#define MAX_TEMPERATURE   (150)

#define SAADC_UPDATE_TERM  (100)

typedef struct
{
    nrf_saadc_resistor_t  resistor_p;
    nrf_saadc_resistor_t  resistor_n;
    nrf_saadc_gain_t      gain;
    nrf_saadc_reference_t reference;
    nrf_saadc_acqtime_t   acq_time;
    nrf_saadc_input_t     pin_p;
    nrf_saadc_input_t     pin_n;
    uint32_t              channel_num;
} saadc_config_t;

void nrf_saadc_init(void);

bool voltage_saadc_init(uint32_t * p_saadc_ch_number,
                        const saadc_config_t * const p_config);
bool temperature_saadc_init(uint32_t * p_sensor_saadc_ch_number,
                            const saadc_config_t * const p_sensor_config,
                            uint32_t * p_vin_saadc_ch_number,
                            const saadc_config_t * const p_vin_config);

double pt100_res2them(const double resistance);

void saadc_buffer_update(void);

double pad_voltage_get(uint32_t channel_num);
double peltier_voltage_get(uint32_t channel_num);
double themperature_get(void);

#endif