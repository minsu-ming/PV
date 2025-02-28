/*********************************************************************
*                     SEGGER Microcontroller GmbH                    *
*                        The Embedded Experts                        *
**********************************************************************
*                                                                    *
*       (c) 1995 - 2022 SEGGER Microcontroller GmbH                  *
*                                                                    *
*       Internet: segger.com  Support: support_embos@segger.com      *
*                                                                    *
**********************************************************************
*                                                                    *
*       embOS * Real time operating system                           *
*                                                                    *
*       Please note:                                                 *
*                                                                    *
*       Knowledge of this file may under no circumstances            *
*       be used to write a similar product or a real-time            *
*       operating system for in-house use.                           *
*                                                                    *
*       Thank you for your fairness !                                *
*                                                                    *
**********************************************************************
*                                                                    *
*       OS version: V5.16.1.0                                        *
*                                                                    *
**********************************************************************

-------------------------- END-OF-HEADER -----------------------------
File    : xmtx.c
Purpose : Thread safe system library functions

Additional information:
  This module enables thread and/or interrupt safety for library functions
  using system locks.
  Per default it ensures thread and interrupt safety by disabling/restoring
  embOS interrupts. Zero latency interrupts are not affected and protected.
  If you need to call e.g. malloc() also from within a zero latency interrupt
  additional handling needs to be added.
  If you don't call such functions from within embOS interrupts you can use
  thread safety instead. This reduces the interrupt latency because a mutex
  is used instead of disabling embOS interrupts.
*/

#include <yvals.h>
#include "RTOS.h"

#if (_MULTI_THREAD != 0)

/*********************************************************************
*
*       Defines
*
**********************************************************************
*/
//
// When set to 1 thread and interrupt safety is guaranteed.
//
// When set to 0 only thread safety is guaranteed. In this case you
// must not call e.g. heap functions from ISRs, software timers or OS_Idle().
//
#ifndef   OS_INTERRUPT_SAFE
  #define OS_INTERRUPT_SAFE  1
#endif

/*********************************************************************
*
*       Global functions
*
**********************************************************************
*/

/*********************************************************************
*
*       __iar_system_Mtxinit()
*/
__ATTRIBUTES void __iar_system_Mtxinit(__iar_Rmtx* m) {
#if (OS_INTERRUPT_SAFE == 1)
  OS_USE_PARA(m);
#else
  OS__iar_system_Mtxinit(m);
#endif
}

/*********************************************************************
*
*       __iar_system_Mtxdst()
*/
__ATTRIBUTES void __iar_system_Mtxdst(__iar_Rmtx* m) {
#if (OS_INTERRUPT_SAFE == 1)
  OS_USE_PARA(m);
#else
  OS__iar_system_Mtxdst(m);
#endif
}

/*********************************************************************
*
*       __iar_system_Mtxlock()
*/
__ATTRIBUTES void __iar_system_Mtxlock(__iar_Rmtx* m) {
#if (OS_INTERRUPT_SAFE == 1)
  OS_USE_PARA(m);
  OS_InterruptSafetyLock();
#else
  OS__iar_system_Mtxlock(m);
#endif
}

/*********************************************************************
*
*       __iar_system_Mtxunlock()
*/
__ATTRIBUTES void __iar_system_Mtxunlock(__iar_Rmtx* m) {
#if (OS_INTERRUPT_SAFE == 1)
  OS_USE_PARA(m);
  OS_InterruptSafetyUnlock();
#else
  OS__iar_system_Mtxunlock(m);
#endif
}

#endif  // (_MULTI_THREAD != 0)

/*************************** End of file ****************************/
