/* buzzer.h */
#ifndef __BUZZER_H
#define __BUZZER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include "tim.h"

/* º¯ÊýÉùÃ÷ */
void Buzzer_Init(uint32_t frequency);
void Buzzer_On(void);
void Buzzer_Off(void);
void Buzzer_SetFrequency(uint32_t frequency);
void Buzzer_Beep(uint32_t duration_ms);
#ifdef __cplusplus
}
#endif

#endif /* __BUZZER_H */