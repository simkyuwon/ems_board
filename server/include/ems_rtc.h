#ifndef EMS_RTC_H__
#define EMS_RTC_H__

#include "log.h"
#include "nrf52.h"
#include "nrf52_bitfields.h"
#include <stdbool.h>

#define RTC2_CLOCK_FREQ           (32768UL) //32.768KHz
#define RTC2_CLOCK_PRESCALER      (33UL) //fRTC = 32768 / (prescaler + 1) = 992.9Hz
#define RTC2_COUNTER_MAX          (0xFFFFFFUL)
#define RTC2_COMPARE_COUNT        (4)
#define RTC2_CC_DELAY_MIN         (5)

#define RTC2_CLOCK_TO_MS(counter) ((counter) * RTC2_CLOCK_PRESCALER * 1000UL / RTC2_CLOCK_FREQ)
#define RTC2_MS_TO_CLOCK(ms)      ((ms) * RTC2_CLOCK_FREQ / RTC2_CLOCK_PRESCALER / 1000UL)

void rtc2_init(void);
uint32_t rtc2_counter_get(void);
bool rtc2_delay(const uint32_t delay_ms, bool(*p_check)(void));
bool rtc2_interrupt(uint32_t delay_ms);

#endif