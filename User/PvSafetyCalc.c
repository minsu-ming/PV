/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <math.h>
#include <stdlib.h>

#define __FPU_PRESENT   1

#include "arm_const_structs.h"
#include "arm_math.h"

#include "PvSafetyMain.h"
#include "RTOS.h"
#include "PvSafetyComm.h"
#include "PvSafetyInitPeri.h"
#include "PvSafetyRtc.h"
#include "PvSafetyHmi.h"
#include "PvSafetyCalc.h"

STRUCT_ADC m_strAdcInfo;

extern uint16_t gsAdc1Fifo[];
extern uint16_t gsAdc2Fifo[];
extern uint16_t gsAdc3Fifo[];
extern uint16_t m_nAdcRaw[];

/* Global variables */
float m_fftInput[FFT_SAMPLES];
float m_fftOutput[FFT_SIZE];
float m_fftMag[FFT_SIZE / 2];
uint32_t m_nFftCnt = 0;
uint32_t m_nFftTick = 0;
float m_fftAvg = 0;
float m_fftAvgArray[2000];
float m_fftMaxArray[2000];
int m_nfftIndex = 0;
float m_fftMax = 0;

void run_fft(void) 
{
	arm_cfft_radix4_instance_f32 S;	/* ARM CFFT module */
	float maxValue;				    /* Max FFT value is stored here */
	uint32_t maxIndex;				/* Index in Output array where max value is */
	uint16_t i, j;
    uint32_t nStartTick;
    
    nStartTick = HAL_GetTick();
    
	/* Initialize the CFFT/CIFFT module, intFlag = 0, doBitReverse = 1 */
	arm_cfft_radix4_init_f32(&S, FFT_SIZE, 0, 1);
		
	/* Process the data through the CFFT/CIFFT module */
	arm_cfft_radix4_f32(&S, m_fftInput);
		
	/* Process the data through the Complex Magniture Module for calculating the magnitude at each bin */
	arm_cmplx_mag_f32(m_fftInput, m_fftOutput, FFT_SIZE);
		
	/* Calculates maxValue and returns corresponding value */
	arm_max_f32(m_fftOutput, FFT_SIZE, &maxValue, &maxIndex);
    
    m_fftAvg = 0;
    m_fftMax = 0;
    
    for (i = 0, j = FFT_SIZE - 1; i < (FFT_SIZE / 2); i++, j--)
    {
        m_fftMag[i] = sqrt(m_fftOutput[i] * m_fftOutput[i] + m_fftOutput[j] * m_fftOutput[j]);

        //  if ((i % 16) && (i % 17) && (i % 18) && (i % 19))
        if ((i > 60) && (i <= 100))
        {
            if (m_fftMag[i] > m_fftMax)
            {
                m_fftMax = m_fftMag[i];
            }
            
            m_fftAvg    += m_fftMag[i];
        }

    }
    
    m_nFftCnt++;
    
    m_fftAvg /= 40;
    m_fftMaxArray[m_nfftIndex]    = m_fftMax;
    m_fftAvgArray[m_nfftIndex++]  = m_fftAvg;
    m_nfftIndex %= 2000;
    
    m_nFftTick  = HAL_GetTick() - nStartTick;
}
