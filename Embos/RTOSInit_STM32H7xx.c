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
Purpose : Initializes and handles the hardware for embOS
*/

#include "RTOS.h"
#include "SEGGER_SYSVIEW.h"
#include "stm32h7xx.h"
#include "PvSafetyPredef.h"

/*********************************************************************
*
*       Defines
*
**********************************************************************
*/

/*********************************************************************
*
*       System tick settings
*/
#ifdef CORE_CM4
  #define OS_TIMER_FREQ  (SystemD2Clock)
#else
  #define OS_TIMER_FREQ  (SystemCoreClock)
#endif
#define OS_TICK_FREQ   (1000u)
#define OS_INT_FREQ    (OS_TICK_FREQ)

/*********************************************************************
*
*       embOSView settings
*/
#ifndef   OS_VIEW_IFSELECT
  #define OS_VIEW_IFSELECT  OS_VIEW_IF_JLINK
#endif

#if (OS_VIEW_IFSELECT == OS_VIEW_IF_JLINK)
  #include "JLINKMEM.h"
#elif (OS_VIEW_IFSELECT == OS_VIEW_IF_UART)
  #include "BSP_UART.h"
  #define OS_UART      (0u)
  #define OS_BAUDRATE  (38400u)
#endif

/*********************************************************************
*
*       Static data
*
**********************************************************************
*/
#if (OS_VIEW_IFSELECT == OS_VIEW_IF_JLINK)
  const OS_U32 OS_JLINKMEM_BufferSize = 32u;  // Size of the communication buffer for JLINKMEM
#else
  const OS_U32 OS_JLINKMEM_BufferSize = 0u;   // Buffer not used
#endif

/*********************************************************************
*
*       Local functions
*
**********************************************************************
*/

#if (OS_VIEW_IFSELECT == OS_VIEW_IF_UART)
/*********************************************************************
*
*       _OS_OnRX()
*
*  Function description
*    Callback wrapper function for BSP UART module.
*/
static void _OS_OnRX(unsigned int Unit, unsigned char c) {
  OS_USE_PARA(Unit);
  OS_COM_OnRx(c);
}

/*********************************************************************
*
*       _OS_OnTX()
*
*  Function description
*    Callback wrapper function for BSP UART module.
*/
static int _OS_OnTX(unsigned int Unit) {
  OS_USE_PARA(Unit);
  return (int)OS_COM_OnTx();
}
#endif

/*********************************************************************
*
*       _OS_GetHWTimerCycles()
*
*  Function description
*    Returns the current hardware timer count value.
*
*  Return value
*    Current timer count value.
*/
static unsigned int _OS_GetHWTimerCycles(void) {
  return SysTick->VAL;
}

/*********************************************************************
*
*       _OS_GetHWTimer_IntPending()
*
*  Function description
*    Returns if the hardware timer interrupt pending flag is set.
*
*  Return value
*    == 0: Interrupt pending flag not set.
*    != 0: Interrupt pending flag set.
*/
static unsigned int _OS_GetHWTimer_IntPending(void) {
  return SCB->ICSR & SCB_ICSR_PENDSTSET_Msk;
}

/*********************************************************************
*
*       Global functions
*
**********************************************************************
*/

/*********************************************************************
*
*       SysTick_Handler()
*
*  Function description
*    This is the hardware timer exception handler.
*/
/* RTOS Systick Handler ISR 사용하지 않음. main() 끝부분에서 "SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;"로 인터럽트 Disable 시킴.
  Rack PMU는 총 3개의 ISR를 사용함.
  - 2개 External ISR (ADC_Busy Clear-7680Hz, GPS PPS-1Hz)
  - 1개 SPI1 RX DMA Transfer Complete ISR-7680Hz
  다수의 고속 ISR이 가동되고 있는 상황에서 1kHz Systick ISR를 사용하는 것은 시스템 Over-Head가 가중되어.
  RTOS Kenerl 부분을 SPI1 RX DMA Transfer Complete ISR(7680Hz)로 옮김. - 960Hz마다 Kenerl처리가 되도록 조정.
*/

static unsigned int m_nTickCount = 0;
extern uint8_t gbTickFlag1;
extern uint8_t gbTickFlag3;
extern uint8_t gbSecFlag; 
extern uint32_t glDtRtc;
extern uint32_t glRtcMod;

uint8_t gbArcTrip   = 0;
uint8_t gbReqArcTrip    = 0;
uint8_t gbReqPvRelayP = 0;
uint8_t gbReqPvRelayN = 0;
uint8_t gbPvRelayP = 0;
uint8_t gbPvRelayN = 0;
uint8_t gbReqLed2 = 0;
uint8_t gbLed2 = 0;

extern OS_TASK TCB_ADC;
extern OS_TASK TCB_LOOP;
extern uint16_t m_nArcTripWaitCnt;

void SysTick_Handler(void) 
{
#if (OS_SUPPORT_PROFILE != 0)
  
    if (SEGGER_SYSVIEW_DWT_IS_ENABLED() == 0u) 
    {
        SEGGER_SYSVIEW_TickCnt++;
    }
    
#endif
  
    HAL_IncTick();
    OS_INT_EnterNestable();
    OS_TICK_Handle();
    
    if (!gbSecFlag)
    {
        m_nTickCount++;
    
        if (m_nTickCount > 999)
        {
            gbSecFlag = 1;
            m_nTickCount = 0;
        }
    }
    
    glRtcMod++;
    
    if (glRtcMod > 999)
    {
        glRtcMod    = 0;
        glDtRtc++;
    }
    
    if (m_nArcTripWaitCnt > 0)
    {
        m_nArcTripWaitCnt--;
    }
    
    gbTickFlag1 = 1;
    gbTickFlag3 = 1;
  
    if (gbArcTrip > 0)
    {
        gbArcTrip--;
        
        if (gbArcTrip == 0)
        {
              ARC_SHORT(0);
              LED2(LED_OFF);
        }
    }
    else
    {
        if (gbReqArcTrip)
        {
            gbReqArcTrip    = 0;
            ARC_SHORT(1);
            LED2(LED_ON);
            gbArcTrip   = 25;
        }
    }
    
    if (gbReqPvRelayP)
    {
        gbReqPvRelayP   = 0;
        
        RY_PVP(gbPvRelayP);
    }
    
    if (gbReqPvRelayN)
    {
        gbReqPvRelayN   = 0;
        
        RY_PVN(gbPvRelayN);
    }
    
    if (gbReqLed2)
    {
        gbReqLed2   = 0;
        
        LED2(gbLed2);
    }

#if (OS_VIEW_IFSELECT == OS_VIEW_IF_JLINK)
  JLINKMEM_Process();
#endif
  OS_INT_LeaveNestable();
}

/*********************************************************************
*
*       OS_InitHW()
*
*  Function description
*    Initialize the hardware required for embOS to run.
*/
void OS_InitHW(void) {
  OS_INT_IncDI();
  //
  // Initialize NVIC vector table offset register if applicable for this device.
  // Might be necessary for RAM targets or application not running from 0x00.
  //
#if (defined(__VTOR_PRESENT) && __VTOR_PRESENT == 1)
  SCB->VTOR = (OS_U32)&__Vectors;
#endif
  //
  // Enable instruction and data cache if applicable for this device
  //
#if (defined(__ICACHE_PRESENT) &&  __ICACHE_PRESENT == 1)
    SCB_EnableICache();
#endif
#if (defined(__DCACHE_PRESENT) &&  __DCACHE_PRESENT == 1)
    //...SCB_EnableDCache();
#endif
  //
  // We assume PLL and core clock were already set by the SystemInit() function,
  // which was called from the startup code. Therefore, we just ensure the system
  // clock variable is updated and then set the periodic system timer tick for embOS.
  //
  SystemCoreClockUpdate();                                        // Update the system clock variable (might not have been set before)
  SysTick_Config(OS_TIMER_FREQ / OS_INT_FREQ);                    // Setup SysTick Timer
  NVIC_SetPriority(SysTick_IRQn, (1u << __NVIC_PRIO_BITS) - 2u);  // Set the priority higher than the PendSV priority
  //
  // Inform embOS about the timer settings
  //
  {
    OS_SYSTIMER_CONFIG SysTimerConfig = {OS_TIMER_FREQ, OS_INT_FREQ, OS_TIMER_DOWNCOUNTING, _OS_GetHWTimerCycles, _OS_GetHWTimer_IntPending};
    OS_TIME_ConfigSysTimer(&SysTimerConfig);
  }
  //
  // Configure and initialize SEGGER SystemView
  //
#if (OS_SUPPORT_PROFILE != 0)
  SEGGER_SYSVIEW_Conf();
#endif
  //
  // Initialize communication for embOSView
  //
#if (OS_VIEW_IFSELECT == OS_VIEW_IF_JLINK)
  JLINKMEM_SetpfOnRx(OS_COM_OnRx);
  JLINKMEM_SetpfOnTx(OS_COM_OnTx);
  JLINKMEM_SetpfGetNextChar(OS_COM_GetNextChar);
#elif (OS_VIEW_IFSELECT == OS_VIEW_IF_UART)
  BSP_UART_Init(OS_UART, OS_BAUDRATE, BSP_UART_DATA_BITS_8, BSP_UART_PARITY_NONE, BSP_UART_STOP_BITS_1);
  BSP_UART_SetReadCallback(OS_UART, _OS_OnRX);
  BSP_UART_SetWriteCallback(OS_UART, _OS_OnTX);
#endif
  OS_INT_DecRI();
}

/*********************************************************************
*
*       OS_Idle()
*
*  Function description
*    This code is executed whenever no task, software timer, or
*    interrupt is ready for execution.
*
*  Additional information
*    The idle loop does not have a stack of its own, therefore no
*    functionality should be implemented that relies on the stack
*    to be preserved.
*/
void OS_Idle(void) {  // Idle loop: No task is ready to execute
  while (1) {         // Nothing to do ... wait for interrupt
    #if ((OS_VIEW_IFSELECT != OS_VIEW_IF_JLINK) && (OS_DEBUG == 0))
      //
      // When uncommenting this line, please be aware device
      // specific issues could occur.
      // Therefore, we do not call __WFI() by default.
      //
      //__WFI();        // Switch CPU into sleep mode
    #endif
  }
}

/*********************************************************************
*
*       Optional communication with embOSView
*
**********************************************************************
*/

/*********************************************************************
*
*       OS_COM_Send1()
*
*  Function description
*    Sends one character.
*/
void OS_COM_Send1(OS_U8 c) {
#if (OS_VIEW_IFSELECT == OS_VIEW_IF_JLINK)
  JLINKMEM_SendChar(c);
#elif (OS_VIEW_IFSELECT == OS_VIEW_IF_UART)
  BSP_UART_Write1(OS_UART, c);
#elif (OS_VIEW_IFSELECT == OS_VIEW_DISABLED)
  OS_USE_PARA(c);          // Avoid compiler warning
  OS_COM_ClearTxActive();  // Let embOS know that Tx is not busy
#endif
}

/*************************** End of file ****************************/
