#ifndef __STEPPER_H
#define __STEPPER_H

void Stepper_ClearLines (void);
void Stepper_AddLine (int32_t x, int32_t y, int32_t z);
void Stepper_Start (void);
void Stepper_Stop (void);
void Stepper_Enable (uint8_t state);
void Stepper_Init (void);



#endif
