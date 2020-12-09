#ifndef EMS_RTC_H__
#define EMS_RTC_H__

#include "nrf52.h"
#include "nrf52_bitfields.h"
#include "nrf_nvic.h"
#include "log.h"
#include <stdbool.h>

#define RTC2_CLOCK_FREQ           (32768UL) //32.768KHz
#define RTC2_CLOCK_PRESCALER      (33UL) //fRTC = 32768 / (prescaler + 1) = 992.9Hz
#define RTC2_COUNTER_MAX          (0xFFFFFFUL)
#define RTC2_COMPARE_COUNT        (4UL)
#define RTC2_CC_DELAY_MIN         (5UL)

#define RTC2_CLOCK_TO_MS(counter) ((counter) * RTC2_CLOCK_PRESCALER * 1000UL / RTC2_CLOCK_FREQ)
#define RTC2_MS_TO_CLOCK(ms)      ((ms) * RTC2_CLOCK_FREQ / RTC2_CLOCK_PRESCALER / 1000UL)

#ifndef NULL
#define NULL 0
#endif

typedef void (* rtc_cb)(void);
typedef bool(* check_func)(void);

void rtc2_init(void);
uint32_t rtc2_counter_get(void);
bool rtc2_delay(const uint32_t delay_ms, check_func);
bool rtc2_interrupt(uint32_t delay_ms, rtc_cb p_cb);

#endif