/*
 * gpio.h
 *
 *  Created on: 19 Feb 2020
 *      Author: Eslam
 */

#ifndef GPIO_H_
#define GPIO_H_

#include "regmap.h"
#include "config.h"

typedef enum {MODE_IN = 0x00, MODE_OUT = 0xff, MODE_AF = 0x3} gpio_mode_t;
typedef enum {PORTA = PORT_A, PORTB = PORT_B, PORTC = PORT_C ,PORTD = PORT_D,PORTE = PORT_E ,PORTF=PORT_F} gpio_port_t;

typedef enum {Drive_2mA = 0x500, Drive_4mA = 0x504, Drive_8mA = 0x508, Drive_8mA_Selw = 0x518} gpio_drive_t;
typedef enum {PAD_PU=0x510,PAD_PD=0x514,PAD_NPU_NPD=0,PAD_OD=0x50C} gpio_pad_t;
typedef enum {HIGH =0xff,LOW=0x00} gpio_data_t;
typedef enum {RISING,FALLING,RISING_FALLING,H_LEVEL,L_LEVEL} interrupt_t;


//Functions prototype
void ConfigureBus();
void GPIOClockSet(gpio_port_t port);
void GPIOClockRst(gpio_port_t port);
char GPIOClockGet(gpio_port_t port);

void GPIODirModeSet(gpio_port_t port, unsigned char pins, gpio_mode_t Mode);
unsigned char GPIODirGet(gpio_port_t port, unsigned char pins);
unsigned char GPIOModeGet(gpio_port_t port, unsigned char pins);

void GPIOPadSet(gpio_port_t port, unsigned char pins, gpio_drive_t str, gpio_pad_t pad);

unsigned char GPIOPadDriveStrGet(gpio_port_t port, unsigned char pin);

unsigned char GPIOPadOpenDrainGet(gpio_port_t port, unsigned char pins);

unsigned char GPIOPadPullUpGet(gpio_port_t port, unsigned char pins);
unsigned char GPIOPadPullDownGet(gpio_port_t port, unsigned char pins);

unsigned char GPIORead(gpio_port_t port, unsigned char pins);
void GPIOWrite(gpio_port_t port, unsigned char pins, gpio_data_t gpio_data);
void GPIO_InterruptSet(gpio_port_t port , unsigned char pins , interrupt_t type , void (*registered_interrupt_portF) (void));

#endif /* GPIO_H_ */
