#include "ems_rtc.h"

static uint32_t rtc2_using_count;
static rtc_cb rtc2_compare_cb[RTC2_COMPARE_COUNT];
static uint32_t rtc2_overflow_count;

void rtc2_init(void)
{
    NRF_RTC2->PRESCALER     = RTC2_CLOCK_PRESCALER - 1;
    NRF_RTC2->EVENTS_OVRFLW = false;
    NRF_RTC2->INTENSET      = (RTC_INTENSET_OVRFLW_Set << RTC_INTENSET_OVRFLW_Pos);
    rtc2_using_count        = 0;
    rtc2_overflow_count     = 0;
    for(int idx = 0; idx < RTC2_COMPARE_COUNT; idx++)
    {
        rtc2_compare_cb[idx] = NULL;
    }

    sd_nvic_SetPriority(RTC2_IRQn, 7);
    sd_nvic_EnableIRQ(RTC2_IRQn);
}

uint32_t rtc2_counter_get(void)
{
    if(NRF_RTC2->EVENTS_OVRFLW)
    {
        NRF_RTC2->EVENTS_OVRFLW = false;
        ++rtc2_overflow_count;
    }
    return NRF_RTC2->COUNTER + rtc2_overflow_count * (RTC2_COUNTER_MAX + 1);
}

static uint32_t rtc2_start(void)
{
    if(rtc2_using_count == 0)//first event
    {
        rtc2_overflow_count = 0;
        NRF_RTC2->TASKS_CLEAR = true;
        NRF_RTC2->TASKS_START = true;
    }
    ++rtc2_using_count;
    return rtc2_counter_get();
}

static bool rtc2_stop(void)
{
    if(rtc2_using_count > 0)
    {
        --rtc2_using_count;
        if(rtc2_using_count == 0)//last event
        {
            NRF_RTC2->TASKS_STOP = true;
        }
        return true;
    }
    return false;
}

bool rtc2_delay(const uint32_t delay_ms, check_func p_func)
{
    uint32_t start_counter = rtc2_start();
    while(RTC2_CLOCK_TO_MS(rtc2_counter_get() - start_counter) < delay_ms)
    {
        if(p_func != NULL)
        {
            if(!p_func())
            {
                rtc2_stop();
                return false;
            }
        }
    }
    rtc2_stop();
    return true;
}

static int32_t alloc_compare_channel(rtc_cb p_cb)
{
    for(int idx = 0; idx < RTC2_COMPARE_COUNT; idx++)
    {
        if(rtc2_compare_cb[idx] == NULL)
        {
            rtc2_compare_cb[idx] = p_cb;
            rtc2_start();
            return idx;
        }
    }

    return -1;
}

bool rtc2_interrupt(uint32_t delay_ms, rtc_cb p_cb)
{
    int32_t channel_num = alloc_compare_channel(p_cb);
    if(channel_num < 0)
    {
        return false;
    }

    uint32_t now_counter = rtc2_counter_get();
    delay_ms = (delay_ms < RTC2_CC_DELAY_MIN) ? RTC2_CC_DELAY_MIN : delay_ms;

    NRF_RTC2->EVENTS_COMPARE[channel_num] = 0;
    NRF_RTC2->INTENSET                    = (RTC_INTENSET_COMPARE0_Set << (RTC_INTENSET_COMPARE0_Pos + channel_num));

    NRF_RTC2->CC[channel_num] = now_counter + RTC2_MS_TO_CLOCK(delay_ms);
    return true;
}

void RTC2_IRQHandler(void)
{
    for(uint32_t ch_num = 0; ch_num < RTC2_COMPARE_COUNT; ch_num++)
    {
        if(NRF_RTC2->EVENTS_COMPARE[ch_num])
        {
            NRF_RTC2->INTENCLR                = (RTC_INTENCLR_COMPARE0_Clear << (RTC_INTENCLR_COMPARE0_Pos + ch_num));
            NRF_RTC2->EVENTS_COMPARE[ch_num]  = false;
            rtc2_compare_cb[ch_num]();
            rtc2_compare_cb[ch_num]           = NULL;
            rtc2_stop();
        }
    }
}