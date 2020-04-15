/*
 * BIT_MODE.h
 *
 *  Created on: Feb 10, 2020
 *      Author: Eslam
 */

#ifndef BIT_MODE_H_
#define BIT_MODE_H_


#define SETBIT(reg,n) (reg |= (1<<n))
#define CLRBIT(reg,n) (reg &= ~(1<<n))
#define GETBIT(reg,n)  ((reg>>n) & 1)

#endif /* BIT_MODE_H_ */
