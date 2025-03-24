
#include "main.h"
#include <stm32f0xx_hal.h>

int main(void)
{
  #if defined(LAB1)
  
  lab1_main();
  #elif defined(LAB2)
  lab2_main();
  #elif defined(LAB3)
  lab3_main();
  #elif defined(LAB4)
  lab4_main();
  #elif defined(LAB5)
  lab5_main();
  //int lab5_main_part2(void);
  //int lab5_main_part2(void);
  #elif defined(LAB6)
  lab6_main();
  #elif defined(LAB7)
  lab7_main();

  #else
  #error No valid target specified
  #endif

}
//extern void SystemClock_Config(void);  // Prevent multiple definition error
