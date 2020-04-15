#include "Interrupt.h"
#include "gpio.h"
#include "CLOCK_int.h"

void usr_isr_portF(void); //main.c

void main(void)
{
     ConfigureBus();
     GPIOClockSet(PORTF);
     GPIODirModeSet(PORTF, 0b00001110,MODE_OUT);
     GPIODirModeSet(PORTF, 0b00010000,MODE_IN);
     GPIOPadSet(PORTF,0b00010000,Drive_8mA,PAD_PU);
     GPIOPadSet(PORTF,0b00001110,Drive_8mA,PAD_NPU_NPD);

     GPIO_InterruptSet(PORTF,0b0010000,RISING,usr_isr_portF);

    while(1);

}

void usr_isr_portF(void)
{
   GPIOWrite(PORTF,0b00001110,HIGH);
}

