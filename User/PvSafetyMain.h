
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PVSFT_MAIN_H
#define __PVSFT_MAIN_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "PvSafetyPredef.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void fcRs485Tx1(uint8_t *bStr, uint16_t nCnt);
void fcRs485Tx3(uint8_t *bStr, uint16_t nCnt);
void fcLoadConfig(void);
void fcInitVariable(void);
void fcAiCdCal(void);
void fcDiscreteFourierTransform(void);
void fcSpi1Dma(uint8_t *bTxBuf, uint8_t *bRxBuf, uint16_t wDMACnt);
uint16_t swapWord(uint16_t w);

extern uint32_t m_nAdc1DamIndex;
extern uint32_t m_nAdc2DamIndex;
extern uint32_t m_nAdc3DamIndex;
extern uint16_t m_nModbusMap_04[];

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
