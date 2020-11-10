#include "ems_pwm.h"

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

static uint16_t pwm_seq0[3][4];
static uint16_t pwm_seq1[3][2][4];

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

    for(uint32_t i = 0; i < 4; i++)
    {
        pwm_seq0[pwm_number][i] = p_config->pulse_width_us;
    }

    for(uint32_t j = 0; j < 2; j++)
    {
        for(uint32_t i = 0; i < 4; i++)
        {
            pwm_seq1[pwm_number][0][i] = pwm_seq1[pwm_number][1][i] = p_config->pulse_width_us;
        }
    }

    pwm_seq0[pwm_number][0] = (p_config->pulse_width_us / 2) | PWM_POLARITY_ACTIVE_LOW;
    pwm_seq0[pwm_number][1] = (p_config->pulse_width_us / 2) | PWM_POLARITY_ACTIVE_HIGH;
    pwm_seq1[pwm_number][1][3] = 1;

    p_nrf_pwm->MODE = (PWM_MODE_UPDOWN_Up << PWM_MODE_UPDOWN_Pos);
    p_nrf_pwm->PRESCALER = (PWM_PRESCALER_PRESCALER_DIV_16 << PWM_PRESCALER_PRESCALER_Pos);
    p_nrf_pwm->DECODER = (PWM_DECODER_LOAD_WaveForm << PWM_DECODER_LOAD_Pos) |
                         (PWM_DECODER_MODE_RefreshCount << PWM_DECODER_MODE_Pos);
    p_nrf_pwm->LOOP = (1 << PWM_LOOP_CNT_Pos);
    p_nrf_pwm->ENABLE = (PWM_ENABLE_ENABLE_Enabled << PWM_ENABLE_ENABLE_Pos);
    p_nrf_pwm->SEQ[0].PTR = ((uint32_t)(pwm_seq0[pwm_number]) << PWM_SEQ_PTR_PTR_Pos);
    p_nrf_pwm->SEQ[0].CNT = ((sizeof(pwm_seq0[pwm_number])/sizeof(uint16_t)) << PWM_SEQ_CNT_CNT_Pos);
    p_nrf_pwm->SEQ[0].REFRESH = ((p_config->pulse_count - 1) << PWM_SEQ_REFRESH_CNT_Pos);
    p_nrf_pwm->SEQ[0].ENDDELAY = (0 << PWM_SEQ_ENDDELAY_CNT_Pos);
    p_nrf_pwm->TASKS_SEQSTART[0] = 0;

    p_nrf_pwm->SEQ[1].PTR = ((uint32_t)(pwm_seq1[pwm_number]) << PWM_SEQ_PTR_PTR_Pos);
    p_nrf_pwm->SEQ[1].CNT = ((sizeof(pwm_seq1[pwm_number])/sizeof(uint16_t)) << PWM_SEQ_CNT_CNT_Pos);
    p_nrf_pwm->SEQ[1].REFRESH = (((p_config->pulse_period_us / p_config->pulse_width_us) - (p_nrf_pwm->SEQ[0].REFRESH + 1) - 1) << PWM_SEQ_REFRESH_CNT_Pos);
    p_nrf_pwm->SEQ[1].ENDDELAY = (0 << PWM_SEQ_ENDDELAY_CNT_Pos);

    p_nrf_pwm->SHORTS = (PWM_SHORTS_LOOPSDONE_SEQSTART0_Enabled << PWM_SHORTS_LOOPSDONE_SEQSTART0_Pos);
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

static uint16_t voltage_seq[1];

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
    
    voltage_seq[0] = p_config->voltage | PWM_POLARITY_ACTIVE_HIGH;

    p_nrf_pwm->MODE = (PWM_MODE_UPDOWN_Up << PWM_MODE_UPDOWN_Pos);
    p_nrf_pwm->PRESCALER = (PWM_PRESCALER_PRESCALER_DIV_8 << PWM_PRESCALER_PRESCALER_Pos);
    p_nrf_pwm->DECODER = (PWM_DECODER_LOAD_Common << PWM_DECODER_LOAD_Pos) |
                         (PWM_DECODER_MODE_RefreshCount << PWM_DECODER_MODE_Pos);
    p_nrf_pwm->COUNTERTOP = 100;
    p_nrf_pwm->ENABLE = (PWM_ENABLE_ENABLE_Enabled << PWM_ENABLE_ENABLE_Pos);
    p_nrf_pwm->SEQ[0].PTR = ((uint32_t)(voltage_seq) << PWM_SEQ_PTR_PTR_Pos);
    p_nrf_pwm->SEQ[0].CNT = ((sizeof(voltage_seq)/sizeof(uint16_t)) << PWM_SEQ_CNT_CNT_Pos);
    p_nrf_pwm->SEQ[0].REFRESH = (PWM_SEQ_REFRESH_CNT_Continuous << PWM_SEQ_REFRESH_CNT_Pos);
    p_nrf_pwm->SEQ[0].ENDDELAY = (0 << PWM_SEQ_ENDDELAY_CNT_Pos);
    p_nrf_pwm->TASKS_SEQSTART[0] = 0;
}

bool change_voltage(int16_t duty)
{
    if(duty < 0)
      duty = 0;
    if(duty > 100)
      duty = 100;
    voltage_seq[0] = (voltage_seq[0] & ~PWM_COMPARE_Msk) | (duty & PWM_COMPARE_Msk);
    pwm_start(VOLTAGE_PWM_NUMBER);
}

bool pwm_start(const uint32_t pwm_number)
{
    NRF_PWM_Type* p_nrf_pwm = nrf_pwm_base(pwm_number);
    if(p_nrf_pwm == NULL || !(p_nrf_pwm->ENABLE & PWM_ENABLE_ENABLE_Msk))
    {
        return false;
    }

    p_nrf_pwm->TASKS_SEQSTART[0] = 1;
    return true;
}

bool pwm_stop(const uint32_t pwm_number)
{
    NRF_PWM_Type* p_nrf_pwm = nrf_pwm_base(pwm_number);
    if(p_nrf_pwm == NULL || !(p_nrf_pwm->ENABLE & PWM_ENABLE_ENABLE_Msk))
    {
        return false;
    }

    p_nrf_pwm->TASKS_STOP = 0;
    return true;
}