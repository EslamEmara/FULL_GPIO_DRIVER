/*
 * regmap.h
 *
 *  Created on: 14 Feb 2020
 *      Author: AbdulRahman
 */

#ifndef REGMAP_H_
#define REGMAP_H_

#include "config.h"
#include "STD_TYPES.h"

#define AHB                 0
#define APB                 1

#define     PORTA_AHB       (u32) 0x40058000
#define     PORTA_APB       0x40004000

#define     PORTB_APB       0x40005000
#define     PORTB_AHB       0x40059000

#define     PORTC_APB       0x40006000
#define     PORTC_AHB       0x4005A000

#define     PORTD_APB       0x40007000
#define     PORTD_AHB       0x4005B000

#define     PORTE_APB       0x40024000
#define     PORTE_AHB       0x4005C000

#define     PORTF_APB       0x40025000
#define     PORTF_AHB       0x4005D000

#if     PORTA_BUS == AHB
    #define PORT_A   PORTA_AHB
#elif   PORTA_BUS == APB
    #define PORT_A   PORTA_APB
#endif

#if     PORTB_BUS == AHB
    #define PORT_B   PORTB_AHB
#elif   PORTB_BUS == APB
    #define PORT_B   PORTB_APB
#endif

#if     PORTC_BUS == AHB
    #define PORT_C   PORTC_AHB
#elif   PORTC_BUS == APB
    #define PORT_C   PORTC_APB
#endif

#if     PORTD_BUS == AHB
    #define PORT_D   PORTD_AHB
#elif   PORTD_BUS == APB
    #define PORT_D   PORTD_APB
#endif

#if     PORTE_BUS == AHB
    #define PORT_E   PORTE_AHB
#elif   PORTE_BUS == APB
    #define PORT_E   PORTE_APB
#endif

#if     PORTF_BUS == AHB
    #define PORT_F   PORTF_AHB
#elif   PORTF_BUS == APB
    #define PORT_F   PORTF_APB
#endif


#define     GPIODATA        0x3FC
#define     GPIODIR         0x400
#define     GPIOAFSEL       0x420
#define     GPIODR2R        0x500
#define     GPIODR4R        0x504
#define     GPIODR8R        0x508
#define     GPIOODR         0x50C
#define     GPIOPUR         0x510
#define     GPIOPDR         0x514
#define     GPIOSLR         0x518
#define     GPIODEN         0x51C
#define     GPIOIM          0x410
#define     GPIORIS         0x414
#define     GPIOICR         0x41C
#define     GPIOIEV         0x40C
#define     GPIOIBE         0x408
#define     GPIOIS          0x404


#define INT_EN0    (*(volatile u32*)  (0xE000E100))

#define GPIOHBCTL    (*(volatile u32*)  (0x400FE06C))
#define RCC          (*(volatile u32*)  (0x400FE060))
#define RCGCGPIO     (*(volatile u32*)  (0x400FE608))


#endif /* REGMAP_H_ */
