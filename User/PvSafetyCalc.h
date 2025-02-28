#ifndef __PV_SAFETY_CALC__H__
#define __PV_SAFETY_CALC__H__

/* FFT settings */
#define FFT_SAMPLES		512 			    /* 256 real party and 256 imaginary parts */
#define FFT_SIZE	    (FFT_SAMPLES / 2)	/* FFT size is always the same size as we have samples, so 256 in our case */

#define ADC_STRCUR_FACT (1.25f * 20.0f)
#define ADC_STRCUR_MAX  (15)
#define ADC_STRCUR_MIN  (-15)

typedef struct _STRUCT_ADC_ {
    float fStrCur[30];
    uint16_t nStrAdcOffset[30];
    uint16_t nStrAdcAvg[30];
} STRUCT_ADC;

extern  STRUCT_ADC m_strAdcInfo;

extern  float m_fftInput[FFT_SAMPLES];
extern  float m_fftOutput[FFT_SIZE];
extern  float m_fftMag[FFT_SIZE / 2];
extern  float m_fftAvg;

void run_fft(void);

#endif