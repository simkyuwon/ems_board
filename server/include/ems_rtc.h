#ifndef EMS_RTC_H__
#define EMS_RTC_H__

#include "nrf52.h"
#include "nrf52_bitfields.h"
#include <stdbool.h>

#define RTC2_CLOCK_FREQ           (32768UL) //32.768KHz
#define RTC2_CLOCK_PRESCALER      (33UL) //fRTC = 32768 / (prescaler + 1) = 992.9Hz
#define RTC2_COUNTER_MAX          (0xFFFFFFUL)

#define RTC2_CLOCK_TO_MS(counter) ((counter) * RTC2_CLOCK_PRESCALER * 1000 / RTC2_CLOCK_FREQ)

void rtc2_init(void);
uint32_t rtc2_counter_get(void);
uint32_t rtc2_start(void);
bool rtc2_stop(void);
bool rtc2_delay(uint32_t delay_ms, bool(*p_check)(void));

#endif