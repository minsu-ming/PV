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
#include "PvSafetyAdc2.h"

extern uint16_t gsAdc1Fifo[];
extern uint16_t gsAdc2Fifo[];
extern uint16_t gsAdc3Fifo[];

static uint8_t m_nAdc2Block = 0xFF;
static uint32_t m_nAdc2Cnt = 0;

uint16_t m_nAdcRaw[FFT_SAMPLES * 4];
uint32_t m_nArcLightCnt = 0;
uint32_t m_nArcLightLimit = 300;

uint32_t m_nArcLightArray[4096] = { 0, };
uint16_t m_nFlashCntArray[4096] = { 0, };
uint16_t m_nArcLightIndex = 0;
uint16_t m_nLightRaw[FFT_SAMPLES * 4];
uint16_t m_nLightPre = 0xFFFF;
uint16_t m_nLightNow = 0;
uint8_t m_nIsLightSum = 0;
uint16_t m_nLightCnt = 0;
uint32_t m_nLightSum = 0;
uint16_t m_nLightSumArray[FFT_SAMPLES];
int m_nLightIndex = 0;
int m_nHighCnt = 0;
int m_nFlashCnt = 0;
uint8_t m_nArcLightTrip = 0;
uint16_t m_nArcTripWaitCnt = 0;
extern uint8_t gbReqArcTrip;
uint8_t m_nArcType = 1;

void runAdc2(void)
{
    uint8_t nBlock = 0;
    uint16_t uAvg = 0;
    int nStart;
    int i, j, k;
    double fSum = 0;
    uint32_t nSum[ADC_CH2] = { 0, };
    
    if (m_nAdc2DamIndex < (ADC_CH2 * SAMP_CYC2 * 1))
    {
        nBlock = 3;
    }
    else if (m_nAdc2DamIndex < (ADC_CH2 * SAMP_CYC2 * 2))
    {
        nBlock = 0;
    }
    else if (m_nAdc2DamIndex < (ADC_CH2 * SAMP_CYC2 * 3))
    {
        nBlock = 1;
    }
    else
    {
        nBlock = 2;
    }
    
    if (nBlock != m_nAdc2Block)
    {
        m_nAdc2Cnt++;
        
        m_nAdc2Block    = nBlock;
        
        nStart  = (ADC_CH2 * SAMP_CYC2 * nBlock);// + FFT_SAMPLES;

        for (i = nStart, j = 0; j < (FFT_SAMPLES * 4); j++)
        {
            m_nAdcRaw[j]    = gsAdc2Fifo[i];
            nSum[0]        += gsAdc2Fifo[i++];
            m_nLightRaw[j]  = gsAdc2Fifo[i];
            m_nLightNow     = gsAdc2Fifo[i++];
            
            m_nLightIndex++;

            if ((m_nLightIndex % 472) == 0)
            {
                m_strAdcInfo.fStrCur[29]                = m_nLightSum;
                m_nFlashCntArray[m_nArcLightIndex]      = m_nFlashCnt;
                m_nArcLightArray[m_nArcLightIndex++]    = m_nLightSum;
                m_nArcLightIndex                       %= 4096;
                
                if (m_nHighCnt > 0)
                {
                    m_strAdcInfo.nStrAdcAvg[29] = m_nLightSum / m_nHighCnt;
                }
                else
                {
                    m_strAdcInfo.nStrAdcAvg[29] = 0;
                }
                
                if (m_nFlashCnt > 10)
                {
                    m_nLightCnt++;
                }
                else
                {
                    m_nLightCnt = 0;
                }                
        
                if (m_nArcLightTrip)
                {
                    if (m_nArcTripWaitCnt == 0)
                    {
                        m_nArcLightTrip = 0;
                    }
                }
                else
                {
                    if (m_nLightCnt > 3)
                    {
                        for (k = 0; k < 24; k++)
                        {
                            if (m_strAdcInfo.fStrCur[k] >= 2.0f)
                            {
                                m_nArcLightTrip     = 1;
                                gbReqArcTrip        = 1;
                                m_nArcTripWaitCnt   = 5000;
                                m_nModbusMap_04[44]++;
                            }
                        }
                        
                        m_nLightCnt = 0;

                        m_nModbusMap_04[43]++;
                    }
                }
                
                m_nFlashCnt     = 0;
                m_nLightSum     = 0;
                m_nHighCnt      = 0;
                m_nLightIndex   = 0;
            }
            //*--- ARCH  
            if (m_nArcType == 0)
            {
                if (m_nLightNow > 30000)
                {
                    m_nLightSum +=  m_nLightNow;   
                    m_nHighCnt++;
                
                    //  if ((m_nLightNow > 35000) && (m_nLightNow < 40000))
                    if (m_nLightNow > 45000)
                    {
                        m_nFlashCnt++;
                    }
                }
            }
            //*--- Flash Type
            else
            {
                if ((m_nLightNow > 100) && (m_nLightNow < 10000))
                {
                    m_nFlashCnt++;
                }
            
                if (m_nLightNow > 30000)
                {
                    m_nLightSum +=  m_nLightNow;   
                    m_nHighCnt++;
                }
            }
        }
            
        //  Arc Current
        uAvg = (nSum[0] /  (FFT_SAMPLES * 4));
        for (i = 0; i < FFT_SAMPLES; i++)
        {
            m_fftInput[i]   = (m_nAdcRaw[i + FFT_SAMPLES] - uAvg);
            fSum           += (double)(m_fftInput[i] * m_fftInput[i]);
        }
        m_strAdcInfo.nStrAdcAvg[28] = uAvg;
        m_strAdcInfo.fStrCur[28]    = (float)(sqrt(fSum / FFT_SAMPLES));
        
        run_fft();
    }
}

