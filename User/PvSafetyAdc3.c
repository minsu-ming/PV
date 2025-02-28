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
#include "PvSafetyAdc3.h"

extern uint16_t gsAdc1Fifo[];
extern uint16_t gsAdc2Fifo[];
extern uint16_t gsAdc3Fifo[];

static uint8_t m_nAdc3Block = 0xFF;
static uint32_t m_nAdc3Cnt = 0;
static uint8_t m_nAdc3Wait = 0;

void runAdc3(void)
{
    uint8_t nBlock = 0;
    int nStart;
    int i, j, k;
    uint16_t temp1, temp2;
    uint32_t nSum[ADC_CH3] = { 0, };
    float fValue;
    
    if (m_nAdc3DamIndex < (ADC_CH3 * SAMP_CYC3 * 1))
    {
        nBlock = 3;
    }
    else if (m_nAdc3DamIndex < (ADC_CH3 * SAMP_CYC3 * 2))
    {
        nBlock = 0;
    }
    else if (m_nAdc3DamIndex < (ADC_CH3 * SAMP_CYC3 * 3))
    {
        nBlock = 1;
    }
    else
    {
        nBlock = 2;
    }
    
    if (nBlock != m_nAdc3Block)
    {
        m_nAdc3Block    = nBlock;

        m_nAdc3Wait++;
        m_nAdc3Wait %= 12;
        
        if (m_nAdc3Wait == 0)
        {
            m_nAdc3Cnt++;
        
            nStart  = (ADC_CH3 * SAMP_CYC3 * nBlock);
        
            for (i = nStart, j = 0; j < SAMP_CYC3; j++)
            {
                for (k = 0; k < ADC_CH3; k++)
                {
                    nSum[k] += (gsAdc3Fifo[i++]);
                }
            }
        
            for (i = 0, k = 16; i < ADC_CH3; i++, k++)
            {
                m_strAdcInfo.nStrAdcAvg[k] = (nSum[i] / SAMP_CYC3);

                //  CH.25 Leakage Current
                if (i == (ADC_CH3 - 4))
                {
                    m_strAdcInfo.fStrCur[k] = ((m_strAdcInfo.nStrAdcAvg[k] - 32100) * 300) / (50400 - 32100);
                }
                //  CH.26 PT1000
                else if (i == (ADC_CH3 - 3))
                {
                    m_strAdcInfo.fStrCur[k] = (65535 - m_strAdcInfo.nStrAdcAvg[k]) * 0.1f;
                }
                //  CH27. Battery
                else if (i == (ADC_CH3 - 2))
                {
                    m_strAdcInfo.fStrCur[k] = (4.0f * (m_strAdcInfo.nStrAdcAvg[k] * 3.3f)) / 65535.0f;
                }
                //  CH28. Temperature
                else if (i == (ADC_CH3 - 1))
                {
                    temp1 = (uint16_t)*TEMPSENSOR_CAL1_ADDR;
                    temp2 = (uint16_t)*TEMPSENSOR_CAL2_ADDR;
                    fValue = (TEMPSENSOR_CAL2_TEMP - TEMPSENSOR_CAL1_TEMP);
                    fValue*= (int32_t)(m_strAdcInfo.nStrAdcAvg[k] - temp1);
                    fValue/= (int32_t)(temp2 - temp1);
                    fValue+= TEMPSENSOR_CAL1_TEMP;
                    m_strAdcInfo.fStrCur[k] = fValue;
                }
                //  String Current
                else
                {
                    m_strAdcInfo.fStrCur[k] = ((m_strAdcInfo.nStrAdcAvg[k] -  m_strAdcInfo.nStrAdcOffset[k]) / 32768.0f) * ADC_STRCUR_FACT;
                }
            }
        }
    }
}

