#include "ems_rtc.h"

static int32_t rtc2_event_count;
static int32_t rtc2_overflow_count;

void rtc2_init(void)
{
    NRF_RTC2->PRESCALER = RTC2_CLOCK_PRESCALER - 1;
    NRF_RTC2->EVENTS_OVRFLW = 0;
    NRF_RTC2->INTENSET = RTC_INTENSET_OVRFLW_Msk;
    rtc2_event_count = 0;
    rtc2_overflow_count = 0;
}

inline uint32_t rtc2_counter_get(void)
{
    if(NRF_RTC2->EVENTS_OVRFLW)
    {
        NRF_RTC2->EVENTS_OVRFLW = 0;
        ++rtc2_overflow_count;
    }
    return NRF_RTC2->COUNTER + rtc2_overflow_count * (RTC2_COUNTER_MAX + 1);
}

uint32_t rtc2_start(void)
{
    if(rtc2_event_count == 0)
    {
        rtc2_overflow_count = 0;
        NRF_RTC2->TASKS_START = 1;
    }
    rtc2_event_count++;
    return rtc2_counter_get();
}

bool rtc2_stop(void)
{
    if(rtc2_event_count > 0)
    {
        --rtc2_event_count;
        return true;
    }
    return false;
}

bool rtc2_delay(uint32_t delay_ms, bool(*p_check)(void))
{
    uint32_t start_counter = rtc2_start();
    while(RTC2_CLOCK_TO_MS(rtc2_counter_get() - start_counter) < delay_ms)
    {
        if(p_check)
        {
            if(!p_check())
            {
                return false;
            }
        }
    }
    rtc2_stop();
    return true;
}