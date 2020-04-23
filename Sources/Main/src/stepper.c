//  Управление шаговыми двигателями
//
//  Использованные аппаратные ресурсы:
//  GPIOB.11 - Stepper_ENABLE
//  GPIOB.12 - Stepper_X_Step
//  GPIOB.13 - Stepper_X_Dir
//  GPIOB.14 - Stepper_Y_Step
//  GPIOB.15 - Stepper_Y_Dir

#include "stm32f10x.h"
#include "math.h"


#define Stepper_GPIO          (GPIOB)
#define Stepper_Pin_Enable    (GPIO_Pin_11)
#define Stepper_Pin_X_Step    (GPIO_Pin_12)
#define Stepper_Pin_X_Dir     (GPIO_Pin_13)
#define Stepper_Pin_Y_Step    (GPIO_Pin_14)
#define Stepper_Pin_Y_Dir     (GPIO_Pin_15)

#define Stepper_Tim_TIM       (TIM2)
#define Stepper_Tim_Prescaler (72)
#define Stepper_Tim_Freq      (30000.0)

#define Stepper_Servo_GPIO    (GPIOA)
#define Stepper_Pin_Servo     (GPIO_Pin_8)

#define Stepper_Servo_TIM     (TIM1)

int32_t Stepper_x1, Stepper_y1, Stepper_z1;                                     //Координаты начала движения
int32_t Stepper_x2, Stepper_y2, Stepper_z2;                                     //Координаты конца движения
int32_t Stepper_dx, Stepper_dy, Stepper_dz;                                     //Дистанции по осям
int32_t Stepper_cur_x, Stepper_cur_y, Stepper_cur_z;                            //Координаты текущие
int32_t Stepper_st_x, Stepper_st_y, Stepper_st_z;                               //Координаты двигателей
double Stepper_d, Stepper_cur_d;                                                //Дистанция движения и пройденная
double Stepper_s1, Stepper_s2, Stepper_cur_s;                                   //Скорость начала и конца движения, текущая
double Stepper_acc_d, Stepper_brk_d;                                            //Дистанция до конца и разгона, до начала торможения 

#define Stepper_MaxLines    3000

#define Stepper_Max_A     2500.0
#define Stepper_Max_S     1000.0
#define Stepper_Min_S     10.0


uint8_t Stepper_IsDraw;
uint16_t Stepper_NLines, Stepper_CurLine;
int16_t Stepper_Lines_x[Stepper_MaxLines];
int16_t Stepper_Lines_y[Stepper_MaxLines];
int16_t Stepper_Lines_z[Stepper_MaxLines];


void Stepper_NextPoint (void) {

    int32_t Stepper_dx_ = Stepper_dx, Stepper_dy_ = Stepper_dy, Stepper_dz_ = Stepper_dz;
    double Stepper_d_ = Stepper_d;
    double MaxS;
  
    if (Stepper_CurLine >= Stepper_NLines) {
      Stepper_IsDraw = 0;
    } else {
      
      Stepper_x1 = Stepper_x2;
      Stepper_y1 = Stepper_y2;
      Stepper_z1 = Stepper_z2;

      Stepper_s1 = Stepper_s2;
      
      
      Stepper_x2 = Stepper_Lines_x[Stepper_CurLine];
      Stepper_y2 = Stepper_Lines_y[Stepper_CurLine];
      Stepper_z2 = Stepper_Lines_z[Stepper_CurLine];

      Stepper_dx = Stepper_x2 - Stepper_x1;
      Stepper_dy = Stepper_y2 - Stepper_y1;
      Stepper_dz = Stepper_z2 - Stepper_z1;
      
      Stepper_d = 
        sqrt(Stepper_dx * Stepper_dx + Stepper_dy * Stepper_dy + Stepper_dz * Stepper_dz);
      
      if (Stepper_CurLine < Stepper_NLines - 1)
        Stepper_s2 = Stepper_Max_S;
      else
        Stepper_s2 = Stepper_Min_S;


      MaxS = sqrt (Stepper_Max_A * Stepper_d +                                  //Максимальная скорость на отрезке
        (Stepper_s1 * Stepper_s1 + Stepper_s2 * Stepper_s2) * 0.5);
      if (MaxS > Stepper_Max_S) MaxS = Stepper_Max_S;                           //Предел
      
      Stepper_acc_d =
        (MaxS * MaxS - Stepper_s1 * Stepper_s1) / (2.0 * Stepper_Max_A);        //Дистанция до набора максимальной скорости
      Stepper_brk_d = Stepper_d - 
        (MaxS * MaxS - Stepper_s2 * Stepper_s2) / (2.0 * Stepper_Max_A);        //Дистанция до начала торможения

      
      Stepper_cur_d = 0.0;
      Stepper_cur_s = Stepper_Min_S;

      
      Stepper_CurLine++;
    }
}


void Stepper_Update (void) {
  int32_t Stepper_d_int = Stepper_d;
  int32_t Stepper_cur_d_int = Stepper_cur_d;
  
  if ((Stepper_cur_d_int < Stepper_d) && (Stepper_d > 0)) {
    Stepper_cur_x = Stepper_x1 + Stepper_dx * Stepper_cur_d_int / Stepper_d_int;
    Stepper_cur_y = Stepper_y1 + Stepper_dy * Stepper_cur_d_int / Stepper_d_int;
    Stepper_cur_z = Stepper_z1 + Stepper_dz * Stepper_cur_d_int / Stepper_d_int;
    
    Stepper_cur_d += (Stepper_cur_s / Stepper_Tim_Freq);

    if (Stepper_cur_d < Stepper_acc_d)                                          //Разгоняем
      Stepper_cur_s += (Stepper_Max_A / Stepper_Tim_Freq);
    if (Stepper_cur_d > Stepper_brk_d)                                          //Тормозим
      Stepper_cur_s -= (Stepper_Max_A / Stepper_Tim_Freq);
    if (Stepper_cur_s < Stepper_Min_S) Stepper_cur_s = Stepper_Min_S;           //Если что-то пошло не так и скорость меньше минимальной

  } else if (Stepper_IsDraw) {
    Stepper_NextPoint ();
  }
  
  
  //Обработка шагов двигателей
  
  if (Stepper_st_x > Stepper_cur_x) {
    GPIO_ResetBits (Stepper_GPIO, Stepper_Pin_X_Dir);
    GPIO_SetBits (Stepper_GPIO, Stepper_Pin_X_Step);
    GPIO_ResetBits (Stepper_GPIO, Stepper_Pin_X_Step);
    Stepper_st_x--;
  }
  if (Stepper_st_x < Stepper_cur_x) {
    GPIO_SetBits (Stepper_GPIO, Stepper_Pin_X_Dir);
    GPIO_SetBits (Stepper_GPIO, Stepper_Pin_X_Step);
    GPIO_ResetBits (Stepper_GPIO, Stepper_Pin_X_Step);
    Stepper_st_x++;
  }

  if (Stepper_st_y > Stepper_cur_y) {
    GPIO_ResetBits (Stepper_GPIO, Stepper_Pin_Y_Dir);
    GPIO_SetBits (Stepper_GPIO, Stepper_Pin_Y_Step);
    GPIO_ResetBits (Stepper_GPIO, Stepper_Pin_Y_Step);
    Stepper_st_y--;
  }
  if (Stepper_st_y < Stepper_cur_y) {
    GPIO_SetBits (Stepper_GPIO, Stepper_Pin_Y_Dir);
    GPIO_SetBits (Stepper_GPIO, Stepper_Pin_Y_Step);
    GPIO_ResetBits (Stepper_GPIO, Stepper_Pin_Y_Step);
    Stepper_st_y++;
  }

   Stepper_st_z = Stepper_cur_z;
  if (Stepper_st_z < 0) Stepper_st_z = 0;
  if (Stepper_st_z > 100) Stepper_st_z = 100;
  TIM_SetCompare1 (Stepper_Servo_TIM, 2200 - Stepper_st_z * 10);

}


void Stepper_ClearLines (void) {
  Stepper_NLines = 0;
}

void Stepper_AddLine (int32_t x, int32_t y, int32_t z) {
  if (Stepper_NLines < Stepper_MaxLines) {
    Stepper_Lines_x[Stepper_NLines] = x;
    Stepper_Lines_y[Stepper_NLines] = y;
    Stepper_Lines_z[Stepper_NLines] = z;
    Stepper_NLines++;
  }
}


void Stepper_Start (void) {
  Stepper_IsDraw = 0;

  Stepper_d = 0.0; Stepper_cur_d = 0.0;
  Stepper_s1 = Stepper_Min_S; Stepper_s2 = Stepper_Min_S; Stepper_cur_s = 0.0;

  Stepper_CurLine = 0;
  Stepper_IsDraw = 1;
}

void Stepper_Stop (void) {
  Stepper_IsDraw = 0;
}

void Stepper_Enable (uint8_t state) {
  if (state)
    GPIO_ResetBits (Stepper_GPIO, Stepper_Pin_Enable);
  else
    GPIO_SetBits (Stepper_GPIO, Stepper_Pin_Enable);
}

void Stepper_Init (void) {
	GPIO_InitTypeDef GPIO_InitStruct;
  RCC_ClocksTypeDef RCC_ClocksStatus;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
  NVIC_InitTypeDef NVIC_InitStructure;
  TIM_OCInitTypeDef TIM_OCInitStruct;

  RCC_GetClocksFreq (&RCC_ClocksStatus);

  //Step-Dir
  
	RCC_APB2PeriphClockCmd (RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_InitStruct.GPIO_Pin = (Stepper_Pin_Enable) | (Stepper_Pin_X_Step) |
    (Stepper_Pin_X_Dir) | (Stepper_Pin_Y_Step) | (Stepper_Pin_Y_Dir);
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init (Stepper_GPIO, &GPIO_InitStruct);

  GPIO_SetBits (Stepper_GPIO, Stepper_Pin_Enable);

  //Servo PWM

	RCC_APB2PeriphClockCmd (RCC_APB2Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd (RCC_APB2Periph_AFIO, ENABLE);

	GPIO_InitStruct.GPIO_Pin = (Stepper_Pin_Servo);
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init (Stepper_Servo_GPIO, &GPIO_InitStruct);
  
  RCC_APB2PeriphClockCmd (RCC_APB2Periph_TIM1, ENABLE);
  
  TIM_TimeBaseInitStruct.TIM_Prescaler = 72 - 1;
  TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInitStruct.TIM_Period = 20000 - 1;
  TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;
  TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit (Stepper_Servo_TIM, &TIM_TimeBaseInitStruct);
  
  
  TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStruct.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM_OCInitStruct.TIM_Pulse = 2200;
  TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStruct.TIM_OCNPolarity = TIM_OCNPolarity_Low;
  TIM_OCInitStruct.TIM_OCIdleState = TIM_OCIdleState_Reset;
  TIM_OCInitStruct.TIM_OCNIdleState = TIM_OCNIdleState_Set;
	TIM_OC1Init (Stepper_Servo_TIM, &TIM_OCInitStruct);
  
	TIM_CtrlPWMOutputs (Stepper_Servo_TIM, ENABLE);
  TIM_Cmd (Stepper_Servo_TIM, ENABLE);
  

  //Update timer
  
  RCC_APB1PeriphClockCmd (RCC_APB1Periph_TIM2, ENABLE);
  
  TIM_TimeBaseInitStruct.TIM_Prescaler = Stepper_Tim_Prescaler - 1;
  TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInitStruct.TIM_Period =
    (uint32_t) (RCC_ClocksStatus.PCLK2_Frequency / (Stepper_Tim_Prescaler * Stepper_Tim_Freq)) - 1;
  TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;
  TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit (Stepper_Tim_TIM, &TIM_TimeBaseInitStruct);

  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init (&NVIC_InitStructure);

  TIM_ITConfig (Stepper_Tim_TIM, TIM_IT_Update, ENABLE);

  TIM_Cmd (Stepper_Tim_TIM, ENABLE);


  Stepper_x1 = 0; Stepper_y1 = 0; Stepper_z1 = 0;
  Stepper_x2 = 0; Stepper_y2 = 0; Stepper_z2 = 0;
  Stepper_dx = 0; Stepper_dy = 0; Stepper_dz = 0;
  Stepper_cur_x = 0; Stepper_cur_y = 0; Stepper_cur_z = 0;
  Stepper_st_x = 0; Stepper_st_y = 0; Stepper_st_z = 0;
  Stepper_d = 0.0; Stepper_cur_d = 0.0;
  Stepper_s1 = 0.0; Stepper_s2 = 0.0; Stepper_cur_s = 0.0;

  Stepper_IsDraw = 0;
  Stepper_NLines = 0;
  Stepper_CurLine = 0;

}





void TIM2_IRQHandler (void) {
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
    TIM_ClearITPendingBit (TIM2, TIM_IT_Update);
    
    Stepper_Update ();

  }
}






