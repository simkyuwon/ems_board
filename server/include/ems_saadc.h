#ifndef EMS_SAADC_H__
#define EMS_SAADC_H__

#include <stdbool.h>

#include "nrf_drv_saadc.h"
#include "log.h"

#define SAADC_CHANNEL_COUNT (3)
#define SAMPLES_IN_BUFFER (1)

#define VOLTAGE_SAADC_CHANNEL (0)
#define TEMPERATURE_SAADC_CHANNEL (1)

#define VOLTAGE_SAADC_CONFIG(resistor_in, gain_in, reference_in, acq_time_in, pin_in) \
    {                                                                                 \
        .resistor_p   = resistor_in,                                                  \
        .gain         = gain_in,                                                      \
        .reference   = reference_in,                                                  \
        .acq_time     = acq_time_in,                                                  \
        .pin_p        = pin_in                                                        \
    }                                                                                 \

#define TEMPERATURE_SAADC_CONFIG(resistor_p_in, resistor_n_in, gain_in, reference_in, acq_time_in, pin_p_in, pin_n_in)  \
    {                                                                                                                   \
        .resistor_p   = resistor_p_in,                                                                                  \
        .resistor_n   = resistor_n_in,                                                                                  \
        .gain         = gain_in,                                                                                        \
        .reference   = reference_in,                                                                                    \
        .acq_time     = acq_time_in,                                                                                    \
        .pin_p        = pin_p_in,                                                                                       \
        .pin_n        = pin_n_in                                                                                        \
    }                                                                                                                   \
    
#define MIN_TEMPERATURE 0
#define MAX_TEMPERATURE 150

typedef struct
{
    nrf_saadc_resistor_t  resistor_p;
    nrf_saadc_resistor_t  resistor_n;
    nrf_saadc_gain_t      gain;
    nrf_saadc_reference_t reference;
    nrf_saadc_acqtime_t   acq_time;
    nrf_saadc_input_t     pin_p;
    nrf_saadc_input_t     pin_n;
} saadc_config_t;

static volatile SAADC_CH_Type* nrf_saadc_base(const uint32_t saadc_ch_number);

bool nrf_saadc_init(const uint16_t * p_saadc_result,
                    const uint16_t buffer_size);

bool voltage_saadc_init(const uint32_t saadc_ch_number,
                        const saadc_config_t * const p_config);

bool temperature_saadc_init(const uint32_t saadc_ch_number,
                            const saadc_config_t * const p_config);

void saadc_buffer_update(void);

int search_res_table_index(const int min_index, const int max_index, const double resistance);

double pt100_res2them(const double resistance);

#endif