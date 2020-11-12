#include "ems_saadc.h"

const double pt100_resistance_table[] = {
100.00, 100.39, 100.78, 101.17, 101.56, 101.95, 102.34, 102.73, 103.12, 103.51,
103.90, 104.29, 104.68, 105.07, 105.46, 105.85, 106.24, 106.63, 107.02, 107.40,
107.79, 108.18, 108.57, 108.96, 109.35, 109.73, 110.12, 110.51, 110.90, 111.29,
111.67, 112.06, 112.45, 112.83, 113.22, 113.61, 114.00, 114.38, 114.77, 115.15,
115.54, 115.93, 116.31, 116.70, 117.08, 117.47, 117.86, 118.24, 118.63, 119.01,
119.40, 119.78, 120.17, 120.55, 120.94, 121.32, 121.71, 122.09, 122.47, 122.86,
123.24, 123.63, 124.01, 124.39, 124.78, 125.16, 125.54, 125.93, 126.31, 126.69,
127.08, 127.46, 127.84, 128.22, 128.61, 128.99, 129.37, 129.75, 130.13, 130.52,
130.90, 131.28, 131.66, 132.04, 132.42, 132.80, 133.18, 133.57, 133.95, 134.33,
134.71, 135.09, 135.47, 135.85, 136.23, 136.61, 136.99, 137.37, 137.75, 138.13,
138.51, 138.88, 139.26, 139.64, 140.02, 140.40, 140.78, 141.16, 141.54, 141.91,
142.29, 142.67, 143.05, 143.43, 143.80, 144.18, 144.56, 144.94, 145.31, 145.69,
146.07, 146.44, 146.82, 147.20, 147.57, 147.95, 148.33, 148.70, 149.08, 149.46,
149.83, 150.21, 150.58, 150.96, 151.33, 151.71, 152.08, 152.46, 152.83, 153.21,
153.58, 153.96, 154.33, 154.71, 155.08, 155.46, 155.83, 156.20, 156.58, 156.95,
157.33
};

static volatile SAADC_CH_Type* nrf_saadc_base(const uint32_t saadc_ch_number)
{
    if(saadc_ch_number >= sizeof(NRF_SAADC->CH)/sizeof(SAADC_CH_Type))
    {
        return NULL;
    }
    return &(NRF_SAADC->CH[saadc_ch_number]);
}

bool nrf_saadc_init(const uint16_t * p_saadc_result,
                    const uint16_t buffer_size)
{
    if(p_saadc_result == NULL)
    {
        return false;
    }
    NRF_SAADC->RESOLUTION = (SAADC_RESOLUTION_VAL_14bit << SAADC_RESOLUTION_VAL_Pos);
    NRF_SAADC->OVERSAMPLE = (SAADC_OVERSAMPLE_OVERSAMPLE_Bypass << SAADC_OVERSAMPLE_OVERSAMPLE_Pos);
    NRF_SAADC->SAMPLERATE = (SAADC_SAMPLERATE_MODE_Timers << SAADC_SAMPLERATE_MODE_Pos) |
                            (1000UL << SAADC_SAMPLERATE_CC_Pos);
    NRF_SAADC->RESULT.PTR = (((uint32_t)p_saadc_result) << SAADC_RESULT_PTR_PTR_Pos);
    NRF_SAADC->RESULT.MAXCNT = (buffer_size << SAADC_RESULT_MAXCNT_MAXCNT_Pos) & SAADC_RESULT_MAXCNT_MAXCNT_Msk;
    NRF_SAADC->ENABLE = (SAADC_ENABLE_ENABLE_Enabled << SAADC_ENABLE_ENABLE_Pos);
}

bool voltage_saadc_init(const uint32_t saadc_ch_number, 
                        const saadc_config_t * const p_config)
{
    volatile SAADC_CH_Type* p_nrf_saadc_ch = nrf_saadc_base(saadc_ch_number);

    if(p_nrf_saadc_ch == NULL)
    {
        return false;
    }
    
    p_nrf_saadc_ch->PSELP = (p_config->pin_p << SAADC_CH_PSELP_PSELP_Pos);
    p_nrf_saadc_ch->CONFIG = (p_config->resistor_p << SAADC_CH_CONFIG_RESP_Pos) |
                             (p_config->gain << SAADC_CH_CONFIG_GAIN_Pos) |
                             (p_config->reference << SAADC_CH_CONFIG_REFSEL_Pos) |
                             (p_config->acq_time << SAADC_CH_CONFIG_TACQ_Pos) |
                             (SAADC_CH_CONFIG_MODE_SE << SAADC_CH_CONFIG_MODE_Pos) |
                             (SAADC_CH_CONFIG_BURST_Disabled << SAADC_CH_CONFIG_BURST_Pos);
    return true;
}

bool temperature_saadc_init(const uint32_t saadc_ch_number,
                            const saadc_config_t * const p_config)
{
    volatile SAADC_CH_Type* p_nrf_th_saadc_ch = nrf_saadc_base(saadc_ch_number);

    if(p_nrf_th_saadc_ch == NULL)
    {
        return false;
    }

    p_nrf_th_saadc_ch->PSELP = (p_config->pin_p << SAADC_CH_PSELP_PSELP_Pos);
    p_nrf_th_saadc_ch->PSELN = (p_config->pin_n << SAADC_CH_PSELP_PSELP_Pos);
    p_nrf_th_saadc_ch->CONFIG = (p_config->resistor_p << SAADC_CH_CONFIG_RESP_Pos) |
                                (p_config->resistor_n << SAADC_CH_CONFIG_RESN_Pos) |
                                (p_config->gain << SAADC_CH_CONFIG_GAIN_Pos) |
                                (p_config->reference << SAADC_CH_CONFIG_REFSEL_Pos) |
                                (p_config->acq_time << SAADC_CH_CONFIG_TACQ_Pos) |
                                (SAADC_CH_CONFIG_MODE_Diff << SAADC_CH_CONFIG_MODE_Pos) |
                                (SAADC_CH_CONFIG_BURST_Disabled << SAADC_CH_CONFIG_BURST_Pos);

    volatile SAADC_CH_Type* p_nrf_ref_saadc_ch = nrf_saadc_base(saadc_ch_number + 1);
   
    if(p_nrf_ref_saadc_ch == NULL)
    {
        return false;
    }

    p_nrf_ref_saadc_ch->PSELP = (NRF_SAADC_INPUT_AIN3 << SAADC_CH_PSELP_PSELP_Pos);
    p_nrf_ref_saadc_ch->CONFIG = (p_config->resistor_p << SAADC_CH_CONFIG_RESP_Pos) |
                                 (SAADC_CH_CONFIG_GAIN_Gain1 << SAADC_CH_CONFIG_GAIN_Pos) |
                                 (SAADC_CH_CONFIG_REFSEL_Internal << SAADC_CH_CONFIG_REFSEL_Pos) |
                                 (p_config->acq_time << SAADC_CH_CONFIG_TACQ_Pos) |
                                 (SAADC_CH_CONFIG_MODE_SE << SAADC_CH_CONFIG_MODE_Pos) |
                                 (SAADC_CH_CONFIG_BURST_Disabled << SAADC_CH_CONFIG_BURST_Pos);
    return true;
}

void saadc_buffer_update(void)
{
    while(NRF_SAADC->STATUS == (SAADC_STATUS_STATUS_Busy << SAADC_STATUS_STATUS_Pos));
    NRF_SAADC->EVENTS_END = 0;
    NRF_SAADC->TASKS_START = 1;
    NRF_SAADC->TASKS_SAMPLE = 1;
    while(!NRF_SAADC->EVENTS_END);
    NRF_SAADC->TASKS_STOP = 1;
}

int search_res_table_index(const int min_index, const int max_index, const double resistance)
{
    if(min_index >= max_index)
    {
        return min_index;
    }
    int mid_index = (min_index + max_index + 1) / 2;
    if(pt100_resistance_table[mid_index] <= resistance)
    {
        return search_res_table_index(mid_index, max_index, resistance);
    }
    else
    {
        return search_res_table_index(min_index, mid_index - 1, resistance);
    }
}

double pt100_res2them(const double resistance)
{
    if(resistance > pt100_resistance_table[MAX_TEMPERATURE] || resistance < pt100_resistance_table[MIN_TEMPERATURE])
    {
        return -1;
    }
    int table_index = search_res_table_index(MIN_TEMPERATURE, MAX_TEMPERATURE, resistance);
    return (double)table_index
          + (resistance - pt100_resistance_table[table_index]) / (pt100_resistance_table[table_index + 1] - pt100_resistance_table[table_index]);
}