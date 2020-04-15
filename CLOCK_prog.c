/*
 * CLOCK_prog.c
 *
 *  Created on: Feb 13, 2020
 *      Author: Eslam
 */

#include "CLOCK_private.h"
#include "CLOCK_config.h"
#include "BIT_MODE.h"

void CLOCK_voidInit()
{
    #if (CLOCK_SOURCE == MOSC)
        CLRBIT(RCC , 0);        /* main osc enable */
        CLRBIT(RCC , 4);        /*OSCSRC*/
        CLRBIT(RCC , 5);
#elif (CLOCK_SOURCE == PIOSC)
        SETBIT(RCC , 4);
        CLRBIT(RCC , 5);
#elif (CLOCK_SOURCE == PIOSC_4)
        CLRBIT(RCC , 4);
        SETBIT(RCC , 5);
#elif (CLOCK_SOURCE == LFIOSC)
        SETBIT(RCC , 4);
        SETBIT(RCC , 5);
#endif

#if (PLL_MODE == ENABLE)
        CLRBIT(RCC , 11);               //BYPASS
        CLRBIT(RCC , 13);
#elif (PLL_MODE == DISABLE)
        SETBIT(RCC , 11);
        SETBIT(RCC , 13);       //PLL POWER DOWN

#endif
        SETBIT(RCC,22);          //USE SYS DIV


#if (SYS_DIV == SYSDIV_1 && PLL_MODE == DISABLE)
        CLRBIT(RCC,23);
        CLRBIT(RCC,24);
        CLRBIT(RCC,25);
        CLRBIT(RCC,26);
#elif (SYS_DIV == SYSDIV_2 && PLL_MODE == DISABLE)
        SETBIT(RCC,23);
        CLRBIT(RCC,24);
        CLRBIT(RCC,25);
        CLRBIT(RCC,26);
#elif (SYS_DIV == SYSDIV_3)
        CLRBIT(RCC,23);
        SETBIT(RCC,24);
        CLRBIT(RCC,25);
        CLRBIT(RCC,26);
#elif (SYS_DIV == SYSDIV_4)
        SETBIT(RCC,23);
        SETBIT(RCC,24);
        CLRBIT(RCC,25);
        CLRBIT(RCC,26);
#elif (SYS_DIV == SYSDIV_5)
        CLRBIT(RCC,23);
        CLRBIT(RCC,24);
        SETBIT(RCC,25);
        CLRBIT(RCC,26);
#elif (SYS_DIV == SYSDIV_6)
        SETBIT(RCC,23);
        CLRBIT(RCC,24);
        SETBIT(RCC,25);
        CLRBIT(RCC,26);
#elif (SYS_DIV == SYSDIV_7)
        CLRBIT(RCC,23);
        SETBIT(RCC,24);
        SETBIT(RCC,25);
        CLRBIT(RCC,26);
#elif (SYS_DIV == SYSDIV_8)
        SETBIT(RCC,23);
        SETBIT(RCC,24);
        SETBIT(RCC,25);
        CLRBIT(RCC,26);
#elif (SYS_DIV == SYSDIV_9)
        CLRBIT(RCC,23);
        CLRBIT(RCC,24);
        CLRBIT(RCC,25);
        SETBIT(RCC,26);
#elif (SYS_DIV == SYSDIV_10)
        SETBIT(RCC,23);
        CLRBIT(RCC,24);
        CLRBIT(RCC,25);
        SETBIT(RCC,26);
#elif (SYS_DIV == SYSDIV_11)
        CLRBIT(RCC,23);
        SETBIT(RCC,24);
        CLRBIT(RCC,25);
        SETBIT(RCC,26);
#elif (SYS_DIV == SYSDIV_12)
        SETBIT(RCC,23);
        SETBIT(RCC,24);
        CLRBIT(RCC,25);
        SETBIT(RCC,26);
#elif (SYS_DIV == SYSDIV_13)
        CLRBIT(RCC,23);
        CLRBIT(RCC,24);
        SETBIT(RCC,25);
        SETBIT(RCC,26);
#elif (SYS_DIV == SYSDIV_14)
        SETBIT(RCC,23);
        CLRBIT(RCC,24);
        SETBIT(RCC,25);
        SETBIT(RCC,26);
#elif (SYS_DIV == SYSDIV_15)
        CLRBIT(RCC,23);
        SETBIT(RCC,24);
        SETBIT(RCC,25);
        SETBIT(RCC,26);
#elif (SYS_DIV == SYSDIV_16)
        SETBIT(RCC,23);
        SETBIT(RCC,24);
        SETBIT(RCC,25);
        SETBIT(RCC,26);
#else
        SETBIT(RCC,23);
        SETBIT(RCC,24);
        SETBIT(RCC,25);
        SETBIT(RCC,26);
#endif

}
