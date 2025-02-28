/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <math.h>
#include <stdlib.h>

#include "PvSafetyMain.h"
#include "RTOS.h"
#include "PvSafetyComm.h"
#include "PvSafetyInitPeri.h"
#include "PvSafetyRtc.h"
#include "PvSafetyHmi.h"
#include "PvSafetyCalc.h"
#include "PvSafetyAdc1.h"

extern uint16_t gsAdc1Fifo[];
extern uint16_t gsAdc2Fifo[];
extern uint16_t gsAdc3Fifo[];

static uint8_t m_nAdc1Block = 0xFF;
static uint32_t m_nAdc1Cnt = 0;
static uint8_t m_nAdc1Wait = 0;

void runAdc1(void)
{
    uint8_t nBlock = 0;
    int nStart;
    int i, j, k;
    uint32_t nSum[ADC_CH1] = { 0, };
    
    if (m_nAdc1DamIndex < (ADC_CH1 * SAMP_CYC1 * 1))
    {
        nBlock = 3;
    }
    else if (m_nAdc1DamIndex < (ADC_CH1 * SAMP_CYC1 * 2))
    {
        nBlock = 0;
    }
    else if (m_nAdc1DamIndex < (ADC_CH1 * SAMP_CYC1 * 3))
    {
        nBlock = 1;
    }
    else
    {
        nBlock = 2;
    }
    
    if (nBlock != m_nAdc1Block)
    {
        m_nAdc1Block    = nBlock;
        
        m_nAdc1Wait++;
        m_nAdc1Wait %= 9;
        
        if (m_nAdc1Wait == 0)
        {
            m_nAdc1Cnt++;
      
            nStart  = (ADC_CH1 * SAMP_CYC1 * nBlock);
        
            for (i = nStart, j = 0; j < SAMP_CYC1; j++)
            {
                for (k = 0; k < ADC_CH1; k++)
                {
                    nSum[k] += gsAdc1Fifo[i++];
                }
            }
        
            for (i = 0; i < ADC_CH1; i++)
            {
                m_strAdcInfo.nStrAdcAvg[i] = nSum[i] / SAMP_CYC1;
                m_strAdcInfo.fStrCur[i] = ((m_strAdcInfo.nStrAdcAvg[i] -  m_strAdcInfo.nStrAdcOffset[i]) / 32768.0f) * ADC_STRCUR_FACT;
            }
        }
    }
}

