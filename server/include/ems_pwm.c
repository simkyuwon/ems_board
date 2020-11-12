#include "ems_pwm.h"

static nrf_ppi_channel_t pwm_ppi_channel[3];

static uint16_t pwm_seq0[3][4];
static uint16_t pwm_seq1[3][4];

static uint8_t pwm_running[3] = {0, 0, 0};

static uint8_t voltage_level = VOLTAGE_LEVEL_MAX;

static NRF_PWM_Type* nrf_pwm_base(const uint32_t pwm_number)
{
    switch(pwm_number)
    {
      case 0:
        return NRF_PWM0;
      case 1:
        return NRF_PWM1;
      case 2:
        return NRF_PWM2;
      default:
        return NULL;
    }
    return NULL;
}

bool waveform_pwm_init(const uint32_t pwm_number, waveform_pwm_config_t const * const p_config)
{
    NRF_PWM_Type* p_nrf_pwm = nrf_pwm_base(pwm_number);
    if(p_nrf_pwm == NULL || (p_nrf_pwm->ENABLE & PWM_ENABLE_ENABLE_Msk) == true)
    {
        return false;
    }

    for(uint32_t idx = 0; idx < WAVEFORM_PWM_CHANNELS_PER_INSTANCE; idx++)
    {
        p_nrf_pwm->PSEL.OUT[idx] = (p_config->pins[idx] << PWM_PSEL_OUT_PIN_Pos) |
                                   (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);
        NRF_P0->PIN_CNF[p_config->pins[idx]] = (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) |
                                               (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) |
                                               (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
                                               (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
                                               (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
    }
    pwm_seq0[pwm_number][0] = (p_config->pulse_width_us / 2) | PWM_POLARITY_ACTIVE_LOW;
    pwm_seq0[pwm_number][1] = (p_config->pulse_width_us / 2) | PWM_POLARITY_ACTIVE_HIGH;
    pwm_seq0[pwm_number][3] = p_config->pulse_width_us;
  
    pwm_seq1[pwm_number][0] = p_config->pulse_width_us | PWM_POLARITY_ACTIVE_LOW;
    pwm_seq1[pwm_number][1] = p_config->pulse_width_us | PWM_POLARITY_ACTIVE_LOW;
    pwm_seq1[pwm_number][3] = p_config->pulse_width_us;

    p_nrf_pwm->MODE = (PWM_MODE_UPDOWN_Up << PWM_MODE_UPDOWN_Pos);
    p_nrf_pwm->ENABLE = (PWM_ENABLE_ENABLE_Enabled << PWM_ENABLE_ENABLE_Pos);
    p_nrf_pwm->PRESCALER = (PWM_PRESCALER_PRESCALER_DIV_16 << PWM_PRESCALER_PRESCALER_Pos);
    p_nrf_pwm->DECODER = (PWM_DECODER_LOAD_WaveForm << PWM_DECODER_LOAD_Pos) |
                         (PWM_DECODER_MODE_RefreshCount << PWM_DECODER_MODE_Pos);
    p_nrf_pwm->LOOP = (1 << PWM_LOOP_CNT_Pos);

    p_nrf_pwm->SEQ[0].PTR = ((uint32_t)(pwm_seq0[pwm_number]) << PWM_SEQ_PTR_PTR_Pos);
    p_nrf_pwm->SEQ[0].CNT = ((sizeof(pwm_seq0[pwm_number])/sizeof(uint16_t)) << PWM_SEQ_CNT_CNT_Pos);
    p_nrf_pwm->SEQ[0].REFRESH = ((p_config->pulse_count - 1) << PWM_SEQ_REFRESH_CNT_Pos);
    p_nrf_pwm->SEQ[0].ENDDELAY = (0 << PWM_SEQ_ENDDELAY_CNT_Pos);
 
    p_nrf_pwm->SEQ[1].PTR = ((uint32_t)(pwm_seq1[pwm_number]) << PWM_SEQ_PTR_PTR_Pos);
    p_nrf_pwm->SEQ[1].CNT = ((sizeof(pwm_seq1[pwm_number])/sizeof(uint16_t)) << PWM_SEQ_CNT_CNT_Pos);
    p_nrf_pwm->SEQ[1].REFRESH = (((p_config->pulse_period_us / p_config->pulse_width_us) - (p_nrf_pwm->SEQ[0].REFRESH + 1) - 1) << PWM_SEQ_REFRESH_CNT_Pos);
    p_nrf_pwm->SEQ[1].ENDDELAY = (0 << PWM_SEQ_ENDDELAY_CNT_Pos);

    nrf_drv_ppi_channel_alloc(&pwm_ppi_channel[pwm_number]);
    nrf_drv_ppi_channel_assign(pwm_ppi_channel[pwm_number], (uint32_t)&(p_nrf_pwm->EVENTS_SEQEND[1]), (uint32_t)&(p_nrf_pwm->TASKS_SEQSTART[0]));
    return true;
}

bool waveform_pulse_count_change(const uint32_t pwm_number, waveform_pwm_config_t * const p_config, uint16_t count)
{
    NRF_PWM_Type* p_nrf_pwm = nrf_pwm_base(pwm_number);
    if(p_nrf_pwm == NULL || !(p_nrf_pwm->ENABLE & PWM_ENABLE_ENABLE_Msk))
    {
        return false;
    }

    if(count >= (p_config->pulse_period_us / p_config->pulse_width_us))
    {
        return false;
    }

    pwm_stop(pwm_number);

    if(count <= 0)
    {
        count = 1;
    }

    p_config->pulse_count = count;

    while(!p_nrf_pwm->EVENTS_LOOPSDONE);

    p_nrf_pwm->SEQ[0].REFRESH = ((p_config->pulse_count - 1) << PWM_SEQ_REFRESH_CNT_Pos);
    p_nrf_pwm->SEQ[1].REFRESH = (((p_config->pulse_period_us / p_config->pulse_width_us) - (p_nrf_pwm->SEQ[0].REFRESH + 1) - 1) << PWM_SEQ_REFRESH_CNT_Pos);

    pwm_start(pwm_number);

    return true;
}

bool waveform_pulse_period_change(const uint32_t pwm_number, waveform_pwm_config_t * const p_config, uint16_t period_us)
{
    NRF_PWM_Type* p_nrf_pwm = nrf_pwm_base(pwm_number);
    if(p_nrf_pwm == NULL || !(p_nrf_pwm->ENABLE & PWM_ENABLE_ENABLE_Msk))
    {
        return false;
    }

    if(period_us <= p_config->pulse_width_us * p_config->pulse_count)
    {
        return false;
    }

    pwm_stop(pwm_number);

    p_config->pulse_period_us = period_us;
    while(!p_nrf_pwm->EVENTS_LOOPSDONE);

    p_nrf_pwm->SEQ[1].REFRESH = (((p_config->pulse_period_us / p_config->pulse_width_us) - (p_nrf_pwm->SEQ[0].REFRESH + 1) - 1) << PWM_SEQ_REFRESH_CNT_Pos);

    pwm_start(pwm_number);

    return true;
}

bool voltage_pwm_init(const uint32_t pwm_number, const voltage_pwm_config_t * const p_config)
{
    NRF_PWM_Type* p_nrf_pwm = nrf_pwm_base(pwm_number);
    if(p_nrf_pwm == NULL || (p_nrf_pwm->ENABLE & PWM_ENABLE_ENABLE_Msk))
    {
        return false;
    }

     p_nrf_pwm->PSEL.OUT[0] = (p_config->pin << PWM_PSEL_OUT_PIN_Pos) |
                              (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);
     NRF_P0->PIN_CNF[p_config->pin] = (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) |
                                      (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) |
                                      (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
                                      (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
                                      (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
    
    p_nrf_pwm->MODE = (PWM_MODE_UPDOWN_UpAndDown << PWM_MODE_UPDOWN_Pos);
    p_nrf_pwm->ENABLE = (PWM_ENABLE_ENABLE_Enabled << PWM_ENABLE_ENABLE_Pos);
    p_nrf_pwm->PRESCALER = (PWM_PRESCALER_PRESCALER_DIV_1 << PWM_PRESCALER_PRESCALER_Pos);
    p_nrf_pwm->DECODER = (PWM_DECODER_LOAD_Common << PWM_DECODER_LOAD_Pos) |
                         (PWM_DECODER_MODE_RefreshCount << PWM_DECODER_MODE_Pos);
    p_nrf_pwm->COUNTERTOP = VOLTAGE_COUNTER_TOP;

    p_nrf_pwm->SEQ[0].PTR = ((uint32_t)(p_config->p_seq->p_dma) << PWM_SEQ_PTR_PTR_Pos);
    p_nrf_pwm->SEQ[0].CNT = (p_config->p_seq->count << PWM_SEQ_CNT_CNT_Pos);
    p_nrf_pwm->SEQ[0].REFRESH = ((p_config->p_seq->period_ms * 8 - 1) << PWM_SEQ_REFRESH_CNT_Pos);
    p_nrf_pwm->SEQ[0].ENDDELAY = (0 << PWM_SEQ_ENDDELAY_CNT_Pos);

    nrf_drv_ppi_channel_alloc(&pwm_ppi_channel[pwm_number]);
    nrf_drv_ppi_channel_assign(pwm_ppi_channel[pwm_number], (uint32_t)&(p_nrf_pwm->EVENTS_SEQEND[0]), (uint32_t)&(p_nrf_pwm->TASKS_SEQSTART[0]));
}

bool voltage_sequence_init(pwm_sequence_config_t * const p_config)
{
    if(p_config == NULL)
    {
        return false;
    }

    p_config->p_dma = malloc(p_config->count * sizeof(uint16_t));

    for(uint32_t idx = 0; idx < p_config->count; idx++)
    {
        p_config->p_dma[idx] = (VOLTAGE_COUNTER_TOP - p_config->p_origin_dma[idx] * voltage_level) | PWM_POLARITY_ACTIVE_LOW;
    }

    return true;
}

static bool voltage_sequence_update(pwm_sequence_config_t * const p_config)
{
    if(p_config == NULL)
    {
        return false;
    }

    pwm_stop(VOLTAGE_PWM_NUMBER);

    for(uint32_t idx = 0; idx < p_config->count; idx++)
    {
        p_config->p_dma[idx] = (VOLTAGE_COUNTER_TOP - p_config->p_origin_dma[idx] * voltage_level) | PWM_POLARITY_ACTIVE_LOW;
    }

    pwm_stop(VOLTAGE_PWM_NUMBER);

    return true;
}

bool voltage_sequence_mode_change(voltage_pwm_config_t * const p_config, pwm_sequence_config_t * const p_seq_config)
{
    if(p_config == NULL)
    {
        return false;
    }
    
    pwm_stop(VOLTAGE_PWM_NUMBER);

    if(!voltage_sequence_update(p_seq_config))
    {
        return false;
    }

    NRF_PWM_Type* p_nrf_pwm = nrf_pwm_base(VOLTAGE_PWM_NUMBER);

    p_nrf_pwm->SEQ[0].PTR = ((uint32_t)(p_seq_config->p_dma) << PWM_SEQ_PTR_PTR_Pos);
    p_nrf_pwm->SEQ[0].CNT = (p_seq_config->count << PWM_SEQ_CNT_CNT_Pos);
    p_nrf_pwm->SEQ[0].REFRESH = ((p_seq_config->period_ms * 8 - 1) << PWM_SEQ_REFRESH_CNT_Pos);
    p_nrf_pwm->SEQ[0].ENDDELAY = (0 << PWM_SEQ_ENDDELAY_CNT_Pos);

    p_config->p_seq = p_seq_config;

    pwm_start(VOLTAGE_PWM_NUMBER);
}

bool voltage_level_up(pwm_sequence_config_t * const p_config)
{
    if(voltage_level == VOLTAGE_LEVEL_MAX)
    {
        return false;
    }

    voltage_level += 1;

    return voltage_sequence_update(p_config);
}

bool voltage_level_down(pwm_sequence_config_t * const p_config)
{
    if(voltage_level == VOLTAGE_LEVEL_MIN)
    {
        return false;
    }

    voltage_level -= 1;

    return voltage_sequence_update(p_config);
}

bool voltage_level_set(pwm_sequence_config_t * const p_config, const uint8_t level)
{
    if(voltage_level < VOLTAGE_LEVEL_MIN || voltage_level > VOLTAGE_LEVEL_MAX)
    {
        return false;
    }

    voltage_level = level;

    return voltage_sequence_update(p_config);
}

bool voltage_period_set(voltage_pwm_config_t * const p_config, const uint32_t period_ms)
{
    if(p_config == NULL || period_ms == 0)
    {
        return false;
    }

    pwm_stop(VOLTAGE_PWM_NUMBER);
    
    p_config->p_seq->period_ms = period_ms;

    NRF_PWM_Type* p_nrf_pwm = nrf_pwm_base(VOLTAGE_PWM_NUMBER);

    p_nrf_pwm->SEQ[0].REFRESH = ((p_config->p_seq->period_ms * 8 - 1) << PWM_SEQ_REFRESH_CNT_Pos);

    pwm_start(VOLTAGE_PWM_NUMBER);
    
    return true;
}

bool pwm_single_shot(const uint32_t pwm_number)
{
    NRF_PWM_Type* p_nrf_pwm = nrf_pwm_base(pwm_number);
    if(p_nrf_pwm == NULL || !(p_nrf_pwm->ENABLE & PWM_ENABLE_ENABLE_Msk))
    {
        return false;
    }

    if(!pwm_running[pwm_number])
    {
        pwm_stop(pwm_number);
    }

    p_nrf_pwm->TASKS_SEQSTART[0] = 1;
    while(!p_nrf_pwm->EVENTS_SEQSTARTED[0]);
    p_nrf_pwm->TASKS_SEQSTART[0] = 0;

    return true;
}

bool pwm_start(const uint32_t pwm_number)
{
    NRF_PWM_Type* p_nrf_pwm = nrf_pwm_base(pwm_number);
    if(p_nrf_pwm == NULL ||
      !(p_nrf_pwm->ENABLE & PWM_ENABLE_ENABLE_Msk) ||
      pwm_running[pwm_number])
    {
        return false;
    }

    nrf_drv_ppi_channel_enable(pwm_ppi_channel[pwm_number]);

    p_nrf_pwm->EVENTS_STOPPED = 0;
    p_nrf_pwm->TASKS_SEQSTART[0] = 1;
    pwm_running[pwm_number] = 1;

    return true;
}

bool pwm_stop(const uint32_t pwm_number)
{
    NRF_PWM_Type* p_nrf_pwm = nrf_pwm_base(pwm_number);
    if(p_nrf_pwm == NULL ||
       !(p_nrf_pwm->ENABLE & PWM_ENABLE_ENABLE_Msk) ||
       !pwm_running[pwm_number])
    {
        return false;
    }

    p_nrf_pwm->TASKS_SEQSTART[0] = 0;
    p_nrf_pwm->EVENTS_STOPPED = 0;
    p_nrf_pwm->TASKS_STOP = 1;
    pwm_running[pwm_number] = 0;

    while(!p_nrf_pwm->EVENTS_STOPPED);

    nrf_drv_ppi_channel_disable(pwm_ppi_channel[pwm_number]);

    return true;
}