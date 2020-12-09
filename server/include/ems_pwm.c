#include "ems_pwm.h"

static nrf_ppi_channel_t pwm_ppi_channel[3];
static nrf_ppi_channel_t pad_seq_cnt_ppi_channel;

static uint16_t pwm_seq0[3][4];
static uint16_t pwm_seq1[3][4];

static bool pwm_is_running[3] = {false, false, false};

static uint8_t target_voltage = PAD_VOLTAGE_MIN;

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
    if(p_nrf_pwm == NULL || p_nrf_pwm->ENABLE & PWM_ENABLE_ENABLE_Msk)
    {
        return false;
    }

    for(uint32_t idx = 0; idx < WAVEFORM_PWM_CHANNELS_PER_INSTANCE; idx++)
    {
        p_nrf_pwm->PSEL.OUT[idx] = (p_config->pins[idx] << PWM_PSEL_OUT_PIN_Pos) |                          //pwm output pin
                                   (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);            //pwm output connect
    }

    //pulse output part
    pwm_seq0[pwm_number][0] = (p_config->pulse_width_us / 2) | PWM_POLARITY_ACTIVE_LOW;
    pwm_seq0[pwm_number][1] = (p_config->pulse_width_us / 2) | PWM_POLARITY_ACTIVE_HIGH;
    pwm_seq0[pwm_number][3] = p_config->pulse_width_us;
   
    //pulse waiting part
    pwm_seq1[pwm_number][0] = p_config->pulse_width_us | PWM_POLARITY_ACTIVE_LOW;
    pwm_seq1[pwm_number][1] = p_config->pulse_width_us | PWM_POLARITY_ACTIVE_LOW;
    pwm_seq1[pwm_number][3] = p_config->pulse_width_us;

    p_nrf_pwm->MODE = (PWM_MODE_UPDOWN_Up << PWM_MODE_UPDOWN_Pos);
    p_nrf_pwm->ENABLE = (PWM_ENABLE_ENABLE_Enabled << PWM_ENABLE_ENABLE_Pos);
    p_nrf_pwm->PRESCALER = (PWM_PRESCALER_PRESCALER_DIV_16 << PWM_PRESCALER_PRESCALER_Pos);           //clock = 16MHz / 16 = 1MHz
    p_nrf_pwm->DECODER = (PWM_DECODER_LOAD_WaveForm << PWM_DECODER_LOAD_Pos) |
                         (PWM_DECODER_MODE_RefreshCount << PWM_DECODER_MODE_Pos);
    p_nrf_pwm->LOOP = (1 << PWM_LOOP_CNT_Pos);                                                        //LOOP(SEQ[0] -> SEQ[1]) -> ppi -> (SEQ[0] ...

    p_nrf_pwm->SEQ[0].PTR = ((uint32_t)(pwm_seq0[pwm_number]) << PWM_SEQ_PTR_PTR_Pos);
    p_nrf_pwm->SEQ[0].CNT = (ARRAY_SIZE(pwm_seq0[pwm_number]) << PWM_SEQ_CNT_CNT_Pos);
    p_nrf_pwm->SEQ[0].REFRESH = ((p_config->pulse_count - 1) << PWM_SEQ_REFRESH_CNT_Pos);             //pwm period count = REFRESH.CNT + 1 
    p_nrf_pwm->SEQ[0].ENDDELAY = (0 << PWM_SEQ_ENDDELAY_CNT_Pos);
 
    p_nrf_pwm->SEQ[1].PTR = ((uint32_t)(pwm_seq1[pwm_number]) << PWM_SEQ_PTR_PTR_Pos);
    p_nrf_pwm->SEQ[1].CNT = (ARRAY_SIZE(pwm_seq1[pwm_number]) << PWM_SEQ_CNT_CNT_Pos);
    //wating part count = (pwm period count = pwm period / pwm 1 pulse width) - output part count
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
        return false;
    }

    p_config->pulse_count = count;

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

    p_nrf_pwm->SEQ[1].REFRESH = (((p_config->pulse_period_us / p_config->pulse_width_us) - (p_nrf_pwm->SEQ[0].REFRESH + 1) - 1) << PWM_SEQ_REFRESH_CNT_Pos);

    pwm_start(pwm_number);

    return true;
}

bool waveform_pulse_width_change(const uint32_t pwm_number, waveform_pwm_config_t * const p_config, uint16_t width_us)
{
    NRF_PWM_Type* p_nrf_pwm = nrf_pwm_base(pwm_number);
    if(p_nrf_pwm == NULL || !(p_nrf_pwm->ENABLE & PWM_ENABLE_ENABLE_Msk))
    {
        return false;
    }

    if(width_us < WAVEFORM_WIDTH_US_MIN || width_us > WAVEFORM_WIDTH_US_MAX)
    {
        return false;
    }

    pwm_stop(pwm_number);
    
    p_config->pulse_width_us = width_us;

    pwm_seq0[pwm_number][0] = (p_config->pulse_width_us / 2) | PWM_POLARITY_ACTIVE_LOW;
    pwm_seq0[pwm_number][1] = (p_config->pulse_width_us / 2) | PWM_POLARITY_ACTIVE_HIGH;
    pwm_seq0[pwm_number][3] = p_config->pulse_width_us;

    pwm_start(pwm_number);

    return true;
}

bool pad_voltage_pwm_init(const uint32_t pwm_number, const pad_voltage_pwm_config_t * const p_config)
{
    NRF_PWM_Type* p_nrf_pwm = nrf_pwm_base(pwm_number);
    if(p_nrf_pwm == NULL || (p_nrf_pwm->ENABLE & PWM_ENABLE_ENABLE_Msk))
    {
        return false;
    }

     p_nrf_pwm->PSEL.OUT[0] = (p_config->pin << PWM_PSEL_OUT_PIN_Pos) |                             //pwm output pin
                              (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);         //pwm output connect
    
    p_nrf_pwm->MODE = (PWM_MODE_UPDOWN_Up << PWM_MODE_UPDOWN_Pos);
    p_nrf_pwm->ENABLE = (PWM_ENABLE_ENABLE_Enabled << PWM_ENABLE_ENABLE_Pos);
    p_nrf_pwm->PRESCALER = (PWM_PRESCALER_PRESCALER_DIV_1 << PWM_PRESCALER_PRESCALER_Pos);          //clock = 16MHz / 1 = 16MHz
    p_nrf_pwm->DECODER = (PWM_DECODER_LOAD_Common << PWM_DECODER_LOAD_Pos) |
                         (PWM_DECODER_MODE_RefreshCount << PWM_DECODER_MODE_Pos);
    p_nrf_pwm->COUNTERTOP = PAD_VOLTAGE_COUNTER_TOP;                                                //1KHz(period) = 16MHz(clock) / 16K(COUNTER_TOP)

    p_nrf_pwm->SEQ[0].PTR = ((uint32_t)(&p_config->dma) << PWM_SEQ_PTR_PTR_Pos);
    p_nrf_pwm->SEQ[0].CNT = (1 << PWM_SEQ_CNT_CNT_Pos);

    //period_ms = (REFRESH.CNT + 1) / period_Hz
    //REFRESH.CNT = period_ms * period_Hz - 1
    uint32_t refersh = (p_config->p_seq->period_ms > PAD_VOLTAGE_PERIOD_MS_MIN) ?
                       (p_config->p_seq->period_ms - 1) : PAD_VOLTAGE_PERIOD_MS_MIN;

    p_nrf_pwm->SEQ[0].REFRESH = (refersh << PWM_SEQ_REFRESH_CNT_Pos);
    p_nrf_pwm->SEQ[0].ENDDELAY = (0 << PWM_SEQ_ENDDELAY_CNT_Pos);


    NRF_TIMER_Type* p_nrf_timer = p_config->counter;
    p_nrf_timer->MODE = (TIMER_MODE_MODE_Counter << TIMER_MODE_MODE_Pos);
    p_nrf_timer->BITMODE = (TIMER_BITMODE_BITMODE_32Bit << TIMER_BITMODE_BITMODE_Pos);
    p_nrf_timer->PRESCALER = (0 << TIMER_PRESCALER_PRESCALER_Pos);
    p_nrf_timer->TASKS_START = 1;

    nrf_drv_ppi_channel_alloc(&pwm_ppi_channel[pwm_number]);
    nrf_drv_ppi_channel_assign(pwm_ppi_channel[pwm_number], (uint32_t)&(p_nrf_pwm->EVENTS_SEQEND[0]), (uint32_t)&(p_nrf_pwm->TASKS_SEQSTART[0]));

    nrf_drv_ppi_channel_alloc(&pad_seq_cnt_ppi_channel);
    nrf_drv_ppi_channel_assign(pad_seq_cnt_ppi_channel, (uint32_t)&(p_nrf_pwm->EVENTS_PWMPERIODEND), (uint32_t)&(p_nrf_timer->TASKS_COUNT));
    nrf_drv_ppi_channel_enable(pad_seq_cnt_ppi_channel);
}

bool pad_voltage_sequence_mode_change(pad_voltage_pwm_config_t * const p_config, const pwm_sequence_config_t * const p_seq_config)
{
    if(p_config == NULL || p_seq_config == NULL)
    {
        return false;
    }
    
    pwm_stop(PAD_VOLTAGE_PWM_NUMBER);

    NRF_PWM_Type* p_nrf_pwm = nrf_pwm_base(PAD_VOLTAGE_PWM_NUMBER);

    uint32_t refersh = (p_config->p_seq->period_ms > PAD_VOLTAGE_PERIOD_MS_MIN) ?
                       (p_config->p_seq->period_ms - 1) : PAD_VOLTAGE_PERIOD_MS_MIN;

    p_nrf_pwm->SEQ[0].REFRESH = (refersh << PWM_SEQ_REFRESH_CNT_Pos);
    p_nrf_pwm->SEQ[0].ENDDELAY = (0 << PWM_SEQ_ENDDELAY_CNT_Pos);

    p_config->p_seq =  (pwm_sequence_config_t *)p_seq_config;

    pwm_start(PAD_VOLTAGE_PWM_NUMBER);
}

void pad_voltage_up(void)
{
    target_voltage += 1;
    if(target_voltage > PAD_VOLTAGE_MAX)
    {
        target_voltage = PAD_VOLTAGE_MAX;
    }
}

void pad_voltage_down(void)
{
    target_voltage -= 1;
    if(target_voltage < PAD_VOLTAGE_MIN)
    {
        target_voltage = PAD_VOLTAGE_MIN;
    }
}

bool pad_voltage_set(const double voltage)
{
    if(voltage < PAD_VOLTAGE_MIN || voltage > PAD_VOLTAGE_MAX)
    {
        return false;
    }

    target_voltage = voltage;

    return true;
}

double pad_target_voltage_get(void)
{
    return target_voltage;
}

bool pad_voltage_period_set(pad_voltage_pwm_config_t * const p_config, const uint32_t period_ms)
{
    if(p_config == NULL || period_ms < PAD_VOLTAGE_PERIOD_MS_MIN)
    {
        return false;
    }

    pwm_stop(PAD_VOLTAGE_PWM_NUMBER);
    
    p_config->p_seq->period_ms = period_ms;

    NRF_PWM_Type* p_nrf_pwm = nrf_pwm_base(PAD_VOLTAGE_PWM_NUMBER);

    p_nrf_pwm->SEQ[0].REFRESH = ((period_ms - 1) << PWM_SEQ_REFRESH_CNT_Pos);

    pwm_start(PAD_VOLTAGE_PWM_NUMBER);
    
    return true;
}

bool peltier_pwm_init(const uint32_t pwm_number, const peltier_pwm_config_t * const p_config)
{
    NRF_PWM_Type* p_nrf_pwm = nrf_pwm_base(pwm_number);
    if(p_nrf_pwm == NULL || p_nrf_pwm->ENABLE & PWM_ENABLE_ENABLE_Msk)
    {
        return false;
    }

    for(uint32_t idx = 0; idx < PELTIER_PWM_CHANNELS_PER_INSTANCE; idx++)
    {
        p_nrf_pwm->PSEL.OUT[idx] = (p_config->pins[idx] << PWM_PSEL_OUT_PIN_Pos) |                          //pwm output pin
                                   (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);            //pwm output connect
    }

    //pulse output part
    pwm_seq0[pwm_number][0] = 0 | PWM_POLARITY_ACTIVE_HIGH;
    pwm_seq0[pwm_number][1] = 0 | PWM_POLARITY_ACTIVE_HIGH;
    pwm_seq0[pwm_number][3] = p_config->period_us;

    p_nrf_pwm->MODE = (PWM_MODE_UPDOWN_Up << PWM_MODE_UPDOWN_Pos);
    p_nrf_pwm->ENABLE = (PWM_ENABLE_ENABLE_Enabled << PWM_ENABLE_ENABLE_Pos);
    p_nrf_pwm->PRESCALER = (PWM_PRESCALER_PRESCALER_DIV_16 << PWM_PRESCALER_PRESCALER_Pos);           //clock = 16MHz / 16 = 1MHz
    p_nrf_pwm->DECODER = (PWM_DECODER_LOAD_WaveForm << PWM_DECODER_LOAD_Pos) |
                         (PWM_DECODER_MODE_RefreshCount << PWM_DECODER_MODE_Pos);

    p_nrf_pwm->SEQ[0].PTR = ((uint32_t)(pwm_seq0[pwm_number]) << PWM_SEQ_PTR_PTR_Pos);
    p_nrf_pwm->SEQ[0].CNT = (ARRAY_SIZE(pwm_seq0[pwm_number]) << PWM_SEQ_CNT_CNT_Pos);
    p_nrf_pwm->SEQ[0].REFRESH = (PWM_SEQ_REFRESH_CNT_Continuous << PWM_SEQ_REFRESH_CNT_Pos);
    p_nrf_pwm->SEQ[0].ENDDELAY = (0 << PWM_SEQ_ENDDELAY_CNT_Pos);

    nrf_drv_ppi_channel_alloc(&pwm_ppi_channel[pwm_number]);
    nrf_drv_ppi_channel_assign(pwm_ppi_channel[pwm_number], (uint32_t)&(p_nrf_pwm->EVENTS_SEQEND[0]), (uint32_t)&(p_nrf_pwm->TASKS_SEQSTART[0]));
    return true;
}

bool peltier_heating(uint32_t duty)
{
    NRF_PWM_Type* p_nrf_pwm = nrf_pwm_base(PELTIER_PWM_NUMBER);

    if(!(p_nrf_pwm->ENABLE & PWM_ENABLE_ENABLE_Msk) || duty > 100)
    {
        return false;
    }

    pwm_stop(PELTIER_PWM_NUMBER);

    uint32_t period_us = pwm_seq0[PELTIER_PWM_NUMBER][3];
    pwm_seq0[PELTIER_PWM_NUMBER][0] = (duty * period_us / 100) | PWM_POLARITY_ACTIVE_LOW;
    pwm_seq0[PELTIER_PWM_NUMBER][1] = 0 | PWM_POLARITY_ACTIVE_LOW;
    
    pwm_start(PELTIER_PWM_NUMBER);

    return true;
}

bool peltier_cooling(uint32_t duty)
{
    NRF_PWM_Type* p_nrf_pwm = nrf_pwm_base(PELTIER_PWM_NUMBER);

    if(!(p_nrf_pwm->ENABLE & PWM_ENABLE_ENABLE_Msk) || duty > 100)
    {
        return false;
    }

    pwm_stop(PELTIER_PWM_NUMBER);

    uint32_t period_us = pwm_seq0[PELTIER_PWM_NUMBER][3];
    pwm_seq0[PELTIER_PWM_NUMBER][0] = 0 | PWM_POLARITY_ACTIVE_LOW;
    pwm_seq0[PELTIER_PWM_NUMBER][1] = (duty * period_us / 100) | PWM_POLARITY_ACTIVE_LOW;
    
    pwm_start(PELTIER_PWM_NUMBER);

    return true;
}

bool peltier_stop(void)
{
    NRF_PWM_Type* p_nrf_pwm = nrf_pwm_base(PELTIER_PWM_NUMBER);

    if(p_nrf_pwm->ENABLE & PWM_ENABLE_ENABLE_Msk)
    {
        return false;
    }

    pwm_stop(PELTIER_PWM_NUMBER);

    uint32_t period_us = pwm_seq0[PELTIER_PWM_NUMBER][3];
    pwm_seq0[PELTIER_PWM_NUMBER][0] = 0 | PWM_POLARITY_ACTIVE_LOW;
    pwm_seq0[PELTIER_PWM_NUMBER][1] = 0 | PWM_POLARITY_ACTIVE_LOW;
    
    pwm_start(PELTIER_PWM_NUMBER);

    return true;
}

bool pwm_single_shot(const uint32_t pwm_number)
{
    NRF_PWM_Type* p_nrf_pwm = nrf_pwm_base(pwm_number);
    if(p_nrf_pwm == NULL || !(p_nrf_pwm->ENABLE & PWM_ENABLE_ENABLE_Msk))
    {
        return false;
    }

    if(pwm_is_running[pwm_number])
    {
        pwm_stop(pwm_number);
    }

    p_nrf_pwm->EVENTS_SEQEND[0] = 0;
    p_nrf_pwm->TASKS_SEQSTART[0] = 1;

    while(!p_nrf_pwm->EVENTS_SEQEND[0]);

    p_nrf_pwm->EVENTS_STOPPED = 0;
    p_nrf_pwm->TASKS_STOP = 1;

    while(!p_nrf_pwm->EVENTS_STOPPED);

    return true;
}

bool pwm_start(const uint32_t pwm_number)
{
    NRF_PWM_Type* p_nrf_pwm = nrf_pwm_base(pwm_number);
    if(p_nrf_pwm == NULL ||
      !(p_nrf_pwm->ENABLE & PWM_ENABLE_ENABLE_Msk) ||
      pwm_is_running[pwm_number])
    {
        return false;
    }

    nrf_drv_ppi_channel_enable(pwm_ppi_channel[pwm_number]);

    pwm_is_running[pwm_number] = true;
    p_nrf_pwm->TASKS_SEQSTART[0] = 1;

    return true;
}

bool pwm_stop(const uint32_t pwm_number)
{
    NRF_PWM_Type* p_nrf_pwm = nrf_pwm_base(pwm_number);
    if(p_nrf_pwm == NULL ||
       !(p_nrf_pwm->ENABLE & PWM_ENABLE_ENABLE_Msk) ||
       !pwm_is_running[pwm_number])
    {
        return false;
    }

    nrf_drv_ppi_channel_disable(pwm_ppi_channel[pwm_number]);

    p_nrf_pwm->EVENTS_STOPPED = 0;
    p_nrf_pwm->TASKS_STOP = 1;
    pwm_is_running[pwm_number] = false;

    while(!p_nrf_pwm->EVENTS_STOPPED);

    return true;
}