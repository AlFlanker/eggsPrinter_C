#include "stm32f10x.h"


int main (void) {
  char c;
  int32_t x, y, z;
  
  Stepper_Init ();
  UART_Init ();
  
  while (1) {
    if (UART_IsRx ()) {
      c = UART_Rx ();
      if (c == '1')
        Stepper_Enable (1);
      if (c == '0')
        Stepper_Enable (0);

      if (c == 'c')
        Stepper_ClearLines ();
      if (c == 'a') {
        x = (UART_Rx () << 0);
        x |= (UART_Rx () << 8);
        x |= (UART_Rx () << 16);
        x |= (UART_Rx () << 24);
        y = (UART_Rx () << 0);
        y |= (UART_Rx () << 8);
        y |= (UART_Rx () << 16);
        y |= (UART_Rx () << 24);
        z = (UART_Rx () << 0);
        z |= (UART_Rx () << 8);
        z |= (UART_Rx () << 16);
        z |= (UART_Rx () << 24);
        Stepper_AddLine (x, y, z);
      }
      if (c == 'b')
        Stepper_Start();
      if (c == 'e')
        Stepper_Stop();
      
      

    }
    
  }

}


