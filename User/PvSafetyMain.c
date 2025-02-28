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
#include "SEGGER_SYSVIEW.h"
#include "PvSafetyCalc.h"
#include "PvSafetyAdc1.h"
#include "PvSafetyAdc2.h"
#include "PvSafetyAdc3.h"

OS_STACKPTR int stackMinute[128];
OS_STACKPTR int stackTaskUart1[128];
OS_STACKPTR int stackTaskUart3[128];
OS_STACKPTR int stackTaskLoop[128];
OS_STACKPTR int stackTaskHmi[128];
OS_STACKPTR int stackTaskAdc[128];
OS_STACKPTR int stackTaskSoc[128];

OS_TASK TCBMinute;
OS_TASK TCB_UART1;
OS_TASK TCB_UART3;
OS_TASK TCB_LOOP;
OS_TASK TCB_HMI;
OS_TASK TCB_ADC;
OS_TASK TCB_SOC;

OS_RSEMA SemaMem;

uint8_t gbTickFlag1, gbTickFlag3, gbCycFlag, gbSecFlag, gbSecCnt, gbGpsRtcSynch, gbFlagPpsForEocGap, gbFlagRstDft;
uint32_t glSysTickValBetweenSampOld, glSysTickValBetweenSampSum, glSysTickValAtPps, glGapPpsToEoc;
int16_t giCosPps2Eoc, giSinPps2Eoc, gwPpsOn;

uint32_t glDtRtc, glRtcMod, glRtcPmu;
uint16_t gwSboCnt1;
uint16_t gwSboCnt3;
ST_RTC gstRtc;

uint8_t gbLcdMem[80];
UN_SOC_CMD gnSocCmd;
ST_METER_OBJ gstMtObj;

ALIGN_32BYTES (__IO uint16_t  gsAdc1Fifo[ADC_FIFO_SIZE1]);
ALIGN_32BYTES (__IO uint16_t  gsAdc2Fifo[ADC_FIFO_SIZE2]);
ALIGN_32BYTES (__IO uint16_t  gsAdc3Fifo[ADC_FIFO_SIZE3]);

//---------------------AI, DI 연산 변수 -------------------
int16_t gsDftSinTable[SAMP_CYC3];
int16_t gsDftCosTable[SAMP_CYC3];

int32_t glDftRe[MAX_AI_CH];	//채널별 Real + jImag
int32_t glDftIm[MAX_AI_CH];

int32_t glDftRePmu[MAX_AI_CH]; //PMU 전송 복소값.
int32_t glDftImPmu[MAX_AI_CH];

int32_t glDftQueRe[SAMP_CYC3 * MAX_AI_CH];	//Recursive Dft 채널 Queue.
int32_t glDftQueIm[SAMP_CYC3 * MAX_AI_CH];

uint16_t gwFreqVal, gwFreqValOld, gwPhaCycCnt;
uint16_t gwTim2Arr;

uint16_t m_nModbusMap_04[INPUT_END_ADDR] = { 0, };

uint8_t gbDftIdx;
uint8_t gbCardIdx;
UN_RAW_ADC gUnRawAdc;
uint16_t gwDiRaw[8];
uint8_t gbRawAdc[18*CARD_AI_NUM];

uint8_t gbModAddr1;
ST_485_CTRL gst485Ctrl1;

uint16_t gbU1RxBufIdx;
uint8_t gbU1RxNdtrOld;
uint16_t gwU1TxCnt;

ALIGN_32BYTES (uint8_t gbU1RxFifo[UART_FIFO_SIZE]);
ALIGN_32BYTES (uint8_t gbU1RxBuf[UART_MAX_PACKET_SIZE]);
ALIGN_32BYTES (uint8_t gbU1TxBuf[UART_MAX_PACKET_SIZE]);

uint8_t gbModAddr3;
ST_485_CTRL gst485Ctrl3;

uint16_t gbU3RxBufIdx;
uint8_t gbU3RxNdtrOld;
uint16_t gwU3TxCnt;

ALIGN_32BYTES (uint8_t gbU3RxFifo[UART_FIFO_SIZE]);
ALIGN_32BYTES (uint8_t gbU3RxBuf[UART_MAX_PACKET_SIZE]);
ALIGN_32BYTES (uint8_t gbU3TxBuf[UART_MAX_PACKET_SIZE]);

//---------------------------------------------------

ST_SYS_SET gstSysSet;	//장치 설정 정보
ST_COM_CFG gstComCfg;
ST_CAL gstCal;
ST_CAL_CMD gstCalCmd;
ST_EVENT_IDX gstSysEvIdx, gstTrgEvIdx;

//----------------------------------------------------

ST_SYS_EVENT gstSysEventFifo[EVENT_FIFO_CNT];
uint8_t gbSysEventFifoIdx, gbSysEventFifoIdxSaved;

uint8_t gbCalFlag;
int16_t gsAccMs;
uint8_t gbAdjustFd;

extern TIM_HandleTypeDef Tim2Handle;

static void MPU_Config(void);
static void SystemClock_Config(void);
static void Error_Handler(void);
static void CPU_CACHE_Enable(void);
//--------------------------------------------------------------------
// taskMinute은 커널(SysTick_Handler)에 의해 1분마다 Resume됨.

static void taskMinute(void)
{
	uint8_t bGpsMinCnt;

	while (1){
		OS_Suspend(NULL);	//1분마다 Resume.
		fcNowRtc(1);	//RTC 동기화.
		
		if(++bGpsMinCnt >= 15){
			bGpsMinCnt = 0;
			gbGpsRtcSynch = 0;	//15마다 GPS RTC를 동기.
		}
	}
}
//--------------------------------------------------------------------

uint8_t m_bReqSkipAdc2 = 0;

static void taskUart1(void)
{
    uint16_t lenPacket;
    UN_W2B unW2B;
    int i;

	while (1) 
    {
		OS_Suspend(NULL);
        
		if(gst485Ctrl1.NewPacket)
        {
			gst485Ctrl1.NewPacket = 0;
            
            if (gstComCfg.wModBusId1 == 0)
            {
                gstComCfg.wModBusId1 = gbU1RxBuf[0];
            }

            gstComCfg.wModBusId1    = 1;
            
            //  주소 일치.
			if(gbU1RxBuf[0] == gstComCfg.wModBusId1)
            { 
                //  Crc 포함.
				if(gbU1RxBuf[1] == WRITE_MULTIPLE_REGISTERS)
                { 
                    lenPacket = 9 + gbU1RxBuf[6]; 
                } 
				else
                { 
                    lenPacket = MODBUS_MIN_PACKET; 
                }
                
                //  Packet 길이 일치.
				if (gst485Ctrl1.RcvdCnt == lenPacket)
                {
					unW2B.wData = fcCrc16Mod(gbU1RxBuf, lenPacket-2);
                    
                    //  Crc16 일치.
					if((unW2B.bData[0] == gbU1RxBuf[lenPacket-2]) && (unW2B.bData[1] == gbU1RxBuf[lenPacket-1]))
                    { 
						fcEtherModbus(gbU1RxBuf, gbU1TxBuf, &gwU1TxCnt);
                        /*
                        for (i = 3; i < gwU1TxCnt; i++)
                        {
                            gbU1TxBuf[i]    = (uint8_t)i;
                        }
                        */
						unW2B.wData = fcCrc16Mod(gbU1TxBuf, gwU1TxCnt);	//fcCrc16 Add.
                        gbU1TxBuf[gwU1TxCnt++] = unW2B.bData[0];
						gbU1TxBuf[gwU1TxCnt++] = unW2B.bData[1];
						fcRs485Tx1(gbU1TxBuf, gwU1TxCnt);
					}
				}
			}
            //  Broad Cast.
			else if (gbU1RxBuf[0] == 0)
            { 
				if(gbU1RxBuf[1] == WRITE_MULTIPLE_REGISTERS)
                { 
                    lenPacket = 9 + gbU1RxBuf[6]; 
                }
				else
                { 
                    lenPacket = MODBUS_MIN_PACKET; 
                }
                
                //  Packet 길이 일치.
				if(gst485Ctrl1.RcvdCnt == lenPacket)
                {
					unW2B.wData = fcCrc16Mod(gbU1RxBuf, lenPacket-2);
					
                    //  Crc16 일치.
                    if(unW2B.bData[0] == gbU1RxBuf[lenPacket-2] && unW2B.bData[1] == gbU1RxBuf[lenPacket-1])
                    { 
						fcBroadCast1(gbU1RxBuf);
					}
				}
			}
		}
	}
}

//--------------------------------------------------------------------
#include <stdio.h>

//OS_Idle 대체
uint16_t gwIcFreq, gwIcFreqOld, gwFreqSumCnt, gwIcFreqAvg;
uint32_t glFreqSum, glEdgeTickValOld, glEdgeClkCnt;
uint8_t gbDftIdxOld;
uint32_t glTcpSndCnt;
uint8_t gbTestBuf[128];
uint32_t m_nAdc1DamIndex = 0;
uint32_t m_nAdc2DamIndex = 0;
uint32_t m_nAdc3DamIndex = 0;

extern UART_HandleTypeDef Uart1Handle;
extern UART_HandleTypeDef Uart3Handle;
extern DMA_HandleTypeDef hUart1DmaTx, hUart3DmaTx, hUart1DmaRx, hUart3DmaRx;
extern TIM_HandleTypeDef Tim2Handle, Tim3Handle;
extern ADC_HandleTypeDef Adc1Handle;
extern ADC_HandleTypeDef Adc2Handle;
extern ADC_HandleTypeDef Adc3Handle;

char m_strDebug[2048 * 10];
extern uint16_t m_nAdcRaw[];

static void taskUart3(void)
{
    uint16_t lenPacket;
    UN_W2B unW2B;
    int nLen;
    int i;

	while (1) 
    {
		OS_Suspend(NULL);
        
		if(gst485Ctrl3.NewPacket)
        {
                      
            /*
            m_bReqSkipAdc2  = 1;
            nLen = 0;

            for (i = 0; i < (FFT_SAMPLES * 4); i++)
            {
                sprintf((char *)&m_strDebug[nLen], "%d\r\n", m_nAdcRaw[i]);
                nLen    = strlen(m_strDebug);
            }
                
            fcRs485Tx3((uint8_t *)m_strDebug, nLen);
            
            m_bReqSkipAdc2  = 0;
            */

            gst485Ctrl3.NewPacket = 0;
            
            if (gstComCfg.wModBusId3 == 0)
            {
                gstComCfg.wModBusId3 = gbU3RxBuf[0];
            }

            //  주소 일치.
			if(gbU3RxBuf[0] == gstComCfg.wModBusId3)
            { 
                //  Crc 포함.
				if(gbU3RxBuf[1] == WRITE_MULTIPLE_REGISTERS)
                { 
                    lenPacket = 9 + gbU3RxBuf[6]; 
                } 
				else
                { 
                    lenPacket = MODBUS_MIN_PACKET; 
                }

                //  Packet 길이 일치.
				if (gst485Ctrl3.RcvdCnt == lenPacket)
                {
					unW2B.wData = fcCrc16Mod(gbU3RxBuf, lenPacket-2);
                    
                    //  Crc16 일치.
					if(unW2B.bData[0] == gbU3RxBuf[lenPacket-2] && unW2B.bData[1] == gbU3RxBuf[lenPacket-1])
                    { 
						fcEtherModbus(gbU3RxBuf, gbU3TxBuf, &gwU3TxCnt);
						unW2B.wData = fcCrc16Mod(gbU3TxBuf, gwU3TxCnt);	//fcCrc16 Add.
						gbU3TxBuf[gwU3TxCnt++] = unW2B.bData[0];
						gbU3TxBuf[gwU3TxCnt++] = unW2B.bData[1];
						fcRs485Tx3(gbU3TxBuf, gwU3TxCnt);
					}
				}
			}
            //  Broad Cast.
			else if (gbU3RxBuf[0] == 0)
            { 
				if(gbU3RxBuf[1] == WRITE_MULTIPLE_REGISTERS)
                { 
                    lenPacket = 9 + gbU3RxBuf[6]; 
                }
				else
                { 
                    lenPacket = MODBUS_MIN_PACKET; 
                }
                
                //  Packet 길이 일치.
				if(gst485Ctrl3.RcvdCnt == lenPacket)
                {
					unW2B.wData = fcCrc16Mod(gbU3RxBuf, lenPacket-2);
					
                    //  Crc16 일치.
                    if(unW2B.bData[0] == gbU3RxBuf[lenPacket-2] && unW2B.bData[1] == gbU3RxBuf[lenPacket-1])
                    { 
						fcBroadCast3(gbU3RxBuf);
					}
				}
			}
		}
	}
}
//--------------------------------------------------------------------
/*
    
     SCB_CleanDCache_by_Addr ((uint32_t *)gbU1RxFifo, UART_FIFO_SIZE);
     SCB_CleanDCache_by_Addr ((uint32_t *)gbU1RxBuf, UART_MAX_PACKET_SIZE);

     SCB_InvalidateDCache_by_Addr ((uint32_t *)gbU1RxFifo, UART_FIFO_SIZE);
     SCB_InvalidateDCache_by_Addr ((uint32_t *)gbU1RxBuf, UART_MAX_PACKET_SIZE);
      
    SCB_InvalidateDCache_by_Addr ((uint32_t *)gbU1TxBuf, UART_MAX_PACKET_SIZE);
  
    SCB_InvalidateDCache_by_Addr ((uint32_t *)gbU3RxFifo, UART_FIFO_SIZE);
    SCB_InvalidateDCache_by_Addr ((uint32_t *)gbU3RxBuf, UART_MAX_PACKET_SIZE);
    SCB_InvalidateDCache_by_Addr ((uint32_t *)gbU3TxBuf, UART_MAX_PACKET_SIZE);
*/
static void taskLoop(void)
{
    int i;

     while (1) 
    { 
        if (((DMA_Stream_TypeDef *)hUart1DmaRx.Instance)->NDTR != gbU1RxNdtrOld)
        {
            //...OS_Delay(5);
            
            while(((DMA_Stream_TypeDef *)hUart1DmaRx.Instance)->NDTR != gbU1RxNdtrOld)
            {
                if(gbU1RxNdtrOld > UART_FIFO_SIZE) 
                {
                    gbU1RxNdtrOld = UART_FIFO_SIZE;
                }
          
                gbU1RxBuf[gbU1RxBufIdx++]  = gbU1RxFifo[UART_FIFO_SIZE - gbU1RxNdtrOld];
          
            
                if(--gbU1RxNdtrOld == 0) 
                {
                    gbU1RxNdtrOld = UART_FIFO_SIZE;
                }
            
                gbU1RxBufIdx &= (UART_MAX_PACKET_SIZE-1);
            }
            
            gst485Ctrl1.TimOut = 100;
        }
        
        if(((DMA_Stream_TypeDef *)hUart3DmaRx.Instance)->NDTR != gbU3RxNdtrOld)
        {
            OS_Delay(5);
            
            while(((DMA_Stream_TypeDef *)hUart3DmaRx.Instance)->NDTR != gbU3RxNdtrOld)
            {
                if(gbU3RxNdtrOld > UART_FIFO_SIZE) 
                {
                    gbU3RxNdtrOld = UART_FIFO_SIZE;
                }
            
                gbU3RxBuf[gbU3RxBufIdx++] = gbU3RxFifo[UART_FIFO_SIZE - gbU3RxNdtrOld];
      
                if(--gbU3RxNdtrOld == 0) 
                {
                    gbU3RxNdtrOld = UART_FIFO_SIZE;
                }
            
                gbU3RxBufIdx &= (UART_MAX_PACKET_SIZE-1);
            }
            
            gst485Ctrl3.TimOut = 4;
        }
        
        if(gst485Ctrl1.TxMode)
        { 
            //Transfer Mode에서 RTS 제어.
            if(((DMA_Stream_TypeDef *)hUart1DmaTx.Instance)->NDTR)
            {	// DMA Transfer Compelete.
                // UART T.C.
                if((Uart1Handle.Instance)->ISR & USART_ISR_TC)
                {
                    gst485Ctrl1.TxMode = 0;
                }
            }
        }

        if(gst485Ctrl3.TxMode)
        { 
            //Transfer Mode에서 RTS 제어.
            if(((DMA_Stream_TypeDef *)hUart3DmaTx.Instance)->NDTR)
            {	// DMA Transfer Compelete.
                // UART T.C.
                if((Uart3Handle.Instance)->ISR & USART_ISR_TC)
                {
                    gst485Ctrl3.TxMode = 0;
                }
            }
        }

		if(gbTickFlag1)
        {
			gbTickFlag1 = 0;


            //  FSR 명령 SBO Discount.
			if(gwSboCnt1)
            {	
                gwSboCnt1--; 
            }	

            if(gst485Ctrl1.TimOut)
            {
                if(--gst485Ctrl1.TimOut == 0)
                {
                    gst485Ctrl1.NewPacket = 1;
                    gst485Ctrl1.RcvdCnt = gbU1RxBufIdx;
                    gbU1RxBufIdx = 0;
                
                    if( OS_GetSuspendCnt(&TCB_UART1) )
                    {
                        OS_Resume(&TCB_UART1);
                    }
                }
            }
		}
		else if(gbTickFlag3)
        {
			gbTickFlag3 = 0;

            //  FSR 명령 SBO Discount.
			if(gwSboCnt3)
            {	
                gwSboCnt3--; 
            }	

            if(gst485Ctrl3.TimOut)
            {
                if(--gst485Ctrl3.TimOut == 0)
                {
                    gst485Ctrl3.NewPacket = 1;
                    gst485Ctrl3.RcvdCnt = gbU3RxBufIdx;
                    gbU3RxBufIdx = 0;
                
                    if( OS_GetSuspendCnt(&TCB_UART3) )
                    {
                        OS_Resume(&TCB_UART3);
                    }
                }
            }
		}
        else if(gbSecFlag)
        {
            //  1초.
            gbSecFlag = 0;
            //  1분에 Minute Task Resume.
            if( ++gbSecCnt >= 60 )
            { 
                gbSecCnt = 0;
            
                if( OS_GetSuspendCnt(&TCBMinute) )
                {
                    OS_Resume(&TCBMinute);
                }
            }
        }

//    //ADC 가변 샘플링 처리 모듈.
//    if(Tim3Handle.Instance->SR & TIM_SR_CC4IF){ //Cature 발생.
//      glEdgeClkCnt = (glEdgeTickValOld - SysTick->VAL)&0xFFFFFF;
//      glEdgeTickValOld = SysTick->VAL;
//  
//      gwIcFreq = (Tim3Handle.Instance->CCR4 - gwIcFreqOld)&0xFFFF;	//Edge to Edge Value.
//      gwIcFreqOld = Tim3Handle.Instance->CCR4;
//    
//      if(glEdgeClkCnt >= VALID_CYC_MIN_CLOCK && glEdgeClkCnt < VALID_CYC_MAX_CLOCK){ //45Hz ~ 65Hz
//        glFreqSum += gwIcFreq;
//        if(++gwFreqSumCnt >= AVERAGE_FREQ_CYC){ //가변샘플링_Table.xlsx 참조.
//          gwFreqValOld = gwFreqVal;
//          // Tim2Handle.Instance->CR1 &= ~TIM_CR1_CEN;	//Tim2 Disable.
//          gwTim2Arr = (glFreqSum+DEN_T2ARR/2)/DEN_T2ARR;
//
//#if RESAMP_SYS
//          if(TIM2_ARR_HIGH >= gwTim2Arr && TIM2_ARR_LOW <= gwTim2Arr){
//            //__HAL_TIM_SET_AUTORELOAD(&Tim2Handle, gwTim2Arr-1);
//            Tim2Handle.Instance->ARR = gwTim2Arr-1;
//          }
//          else{
//            // __HAL_TIM_SET_AUTORELOAD(&Tim2Handle, DEFAULT_TIM2_ARR-1);
//            Tim2Handle.Instance->ARR = DEFAULT_TIM2_ARR-1;
//          }
//#endif
//          // Tim2Handle.Instance->CR1 |= TIM_CR1_CEN;	//Tim2 Enable.
//      	  gwFreqVal = FREQ_CAL_CONST/gwTim2Arr;
//          glFreqSum = 0;
//          gwFreqSumCnt = 0;
//        }
//      }
//      else{
//        gwTim2Arr = DEFAULT_TIM2_ARR-1;
//        glFreqSum = 0;
//        gwFreqSumCnt = 0;
//        // __HAL_TIM_SET_AUTORELOAD(&Tim2Handle, gwTim2Arr-1);
//        Tim2Handle.Instance->ARR = gwTim2Arr;
//      }
//    }
    }
}
//--------------------------------------------------------------------

uint8_t m_bDipSw = 0xFF;

static void taskHmi(void)
{
    uint8_t bDip, bDipOld;
    uint8_t bLed1St = 0;
    uint8_t nIndex = 0;

    LCD_RST(0);
    OS_Delay(2);
    LCD_RST(1);
    OS_Delay(2);

    fcInitLCD();

    while (1) 
    {   
        if (nIndex == 0)
        {
            fcLcdMemUpdate();
        }

        m_bDipSw = (GPIOG->IDR >> 4) & 0x0F ;
//    if(bDip != bDipOld){
//      bDipOld = bDip;
//     fcSetGain(bDip);
//    }

        LED1(bLed1St);
        bLed1St = !bLed1St;
 
        OS_Delay(HMI_MS);
        
        nIndex++;
        nIndex %= 2;
    } 
}
//--------------------------------------------------------------------

extern SPI_HandleTypeDef Spi6Handle;
extern  DMA_HandleTypeDef hSpi6DmaTx, hSpi6DmaRx;

int32_t fcRdSocReg(uint8_t bRegAddr);
void fcWrSocReg(uint8_t bRegAddr, int32_t lRegVal);
int fcReadSocConfig(void);
void fcCmdSocReg(uint8_t *sCmd);
void fcReadSocData(void);

uint8_t gbSpi6RxBuf[32];
uint8_t gbSpi6TxInst[8];
uint8_t gbSpi6TxBuf[32];
uint32_t m_nSocCnt = 0;


int32_t fcRdSocReg(uint8_t bRegAddr)
{
	UN_D2B unD2B;

	SOC_CS(0);
    
    OS_Delay(1);

	gbSpi6TxBuf[0] = ((bRegAddr & 0xC0) / 16) + 0x01;
	gbSpi6TxBuf[1] = bRegAddr * 4;
	gbSpi6TxBuf[2] = 0xFF;
	gbSpi6TxBuf[3] = 0xFF;
	gbSpi6TxBuf[4] = 0xFF;

    HAL_SPI_TransmitReceive(&Spi6Handle, gbSpi6TxBuf, gbSpi6RxBuf, 5, 5000);

	SOC_CS(1);

	unD2B.bData[0] = gbSpi6RxBuf[4];
	unD2B.bData[1] = gbSpi6RxBuf[3];
	unD2B.bData[2] = gbSpi6RxBuf[2];
	unD2B.bData[3] = (gbSpi6RxBuf[2] & 0x80) ? 0xFF : 0;

	return unD2B.lData;
}
//--------------------------------------------

void fcWrSocReg(uint8_t bRegAddr, int32_t lRegVal)
{
	UN_D2B unD2B;

	unD2B.lData = lRegVal;

	SOC_CS(0);
	gbSpi6TxBuf[0] = (bRegAddr&0xC0)/16 + 0x01;
	gbSpi6TxBuf[1] = bRegAddr*4 + 0x02;
	gbSpi6TxBuf[2] = unD2B.bData[2];
	gbSpi6TxBuf[3] = unD2B.bData[1];
	gbSpi6TxBuf[4] = unD2B.bData[0];

    HAL_SPI_Transmit(&Spi6Handle, gbSpi6TxBuf, 5, 5000);

	SOC_CS(1);
}
//-------------------------------------------------------------

void fcCmdSocReg(uint8_t *sCmd)
{
	SOC_CS(0);
	gbSpi6TxBuf[0] = 0x01;
	gbSpi6TxBuf[1] = 0x02;
	gbSpi6TxBuf[2] = sCmd[2];
	gbSpi6TxBuf[3] = sCmd[1];
	gbSpi6TxBuf[4] = sCmd[0];

    HAL_SPI_Transmit(&Spi6Handle, gbSpi6TxBuf, 5, 5000);

	SOC_CS(1);
}
//-------------------------------------------------------------
uint32_t m_bSocVersion;
uint32_t m_bSocConfig;
int32_t m_nVoltageOffset[3];
int32_t m_nVoltageGain[3];
int32_t m_nCurrentOffset[3];
int32_t m_nCurrentGain[3];
int32_t m_nVoltageTarget;
int32_t m_nCurrentTarget;
int32_t m_nTempTarget;
int32_t m_nVoltageIns[3];
int32_t m_nVoltageRms[3];
int32_t m_nCurrentIns[3];
int32_t m_nCurrentRms[3];

int fcReadSocConfig(void)
{
    fcWrSocReg(0x00, 0xBD0000);     //  Reset
    fcWrSocReg(0x00, 0x000000);     //  Normal Operation

    m_bSocVersion   = fcRdSocReg(0x01);

    if ((m_bSocVersion != 0xFFFFFFFF) && (m_bSocVersion != 0x00000000))
    {
        m_bSocConfig    = fcRdSocReg(0x02);

        m_nVoltageOffset[0] = fcRdSocReg(0x75);
        m_nVoltageOffset[1] = fcRdSocReg(0x76);
        m_nVoltageOffset[2] = fcRdSocReg(0x77);

        m_nVoltageGain[0]   = fcRdSocReg(0x78);
        m_nVoltageGain[1]   = fcRdSocReg(0x79);
        m_nVoltageGain[2]   = fcRdSocReg(0x7A);

        m_nCurrentOffset[0] = fcRdSocReg(0x7B);
        m_nCurrentOffset[1] = fcRdSocReg(0x7C);
        m_nCurrentOffset[2] = fcRdSocReg(0x7D);

        m_nCurrentGain[0]   = fcRdSocReg(0x7E);
        m_nCurrentGain[1]   = fcRdSocReg(0x7F);
        m_nCurrentGain[2]   = fcRdSocReg(0x80);

        m_nVoltageTarget    = fcRdSocReg(0x91);
        m_nCurrentTarget    = fcRdSocReg(0x95);
        m_nTempTarget   = fcRdSocReg(0x97);

        fcWrSocReg(0x75, -1800);
        fcWrSocReg(0x76, 0);
        fcWrSocReg(0x77, -1800);

        fcWrSocReg(0x7B, 1000);
        fcWrSocReg(0x7C, 0);
        fcWrSocReg(0x7D, 1000);

        m_nVoltageOffset[0] = fcRdSocReg(0x75);
        m_nVoltageOffset[1] = fcRdSocReg(0x76);
        m_nVoltageOffset[2] = fcRdSocReg(0x77);

        fcWrSocReg(0x6f, 0); //HPF_COEF_I
        fcWrSocReg(0x70, 0); //HPF_COEF_V

        fcWrSocReg(0x91, 0); //Calibration 기준 전압.
        fcWrSocReg(0x95, 0); //Calibration 기준 전류.
        fcWrSocReg(0x94, 0); //정전인식 기준 전압.

        return 0;
    }
    else
    {
        return 1;
    }
}
//-------------------------------------------------------------

void fcReadSocData(void)
{
    //  VA_RMS: RMS Voltage
    OS_Delay(1);    
    m_nVoltageRms[0]    = fcRdSocReg(0x13);

    //  VB_RMS: RMS Voltage
    OS_Delay(1);    
    m_nVoltageRms[1]    = fcRdSocReg(0x14);

    //  VC_RMS: RMS Voltage
    OS_Delay(1);    
    //...m_nVoltageRms[2]    = fcRdSocReg(0x15);

    //  VA_INS: INS Voltage
    OS_Delay(1);    
    m_nVoltageIns[0]    = fcRdSocReg(0x0D);

    //  VB_INS: INS Voltage
    OS_Delay(1);    
    m_nVoltageIns[1]    = fcRdSocReg(0x0E);

    //  VC_INS: INS Voltage
    OS_Delay(1);    
    //...m_nVoltageIns[2]    = fcRdSocReg(0x0F);

    //  IA_RMS: RMS Current
    OS_Delay(1);    
    m_nCurrentRms[0]    = fcRdSocReg(0x1D);

    //  IB_RMS: RMS Current
    OS_Delay(1);    
    m_nCurrentRms[1]    = fcRdSocReg(0x1E);

    //  IC_RMS: RMS Current
    OS_Delay(1);    
    //...m_nCurrentRms[2]    = fcRdSocReg(0x1F);

    //  IA_INS: INS Current
    OS_Delay(1);    
    m_nCurrentIns[0]    = fcRdSocReg(0x10);

    //  IB_INS: INS Current
    OS_Delay(1);    
    m_nCurrentIns[1]    = fcRdSocReg(0x11);

    //  IC_INS: INS Current
    OS_Delay(1);    
    //...m_nCurrentIns[2]    = fcRdSocReg(0x12);
}
//-------------------------------------------------------------
float m_fVoltage[4] = { 0, };
float m_fVoltageInsArray[64] = { 0, };
int m_nInsIndex = 0;

float m_fVoltageOffMin[2] = { 874000, 445000 };
float m_fVoltageOffMax[2] = { 1757000, 887000 };

float m_fVoltageOnMin[4] = { 431000, 226500, 0, 0 };
float m_fVoltageOnMax[4] = { 874000, 445000, 0, 0 };

extern uint8_t gbPvRelayP;
extern uint8_t gbPvRelayN;

static void taskSoc(void)
{ 
    fcReadSocConfig();
    
    while(1)
    {        
        OS_Delay(1);    
        m_bSocVersion       = fcRdSocReg(0x01);
        OS_Delay(1);
        m_nVoltageOffset[0] = fcRdSocReg(0x75);
        OS_Delay(1);
        m_nVoltageOffset[1] = fcRdSocReg(0x76);
        OS_Delay(1);
        //...m_nVoltageOffset[2] = fcRdSocReg(0x77);

        fcReadSocData();

        if (gbPvRelayP == 0)
        {
            m_fVoltage[0]   = (m_nVoltageRms[0] - m_fVoltageOffMin[0]) * (100 - 50) / (m_fVoltageOffMax[0] - m_fVoltageOffMin[0]) + 50;
            m_fVoltage[2]   = 0;
        }
        else
        {
            m_fVoltage[0]   = (m_nVoltageRms[0] - m_fVoltageOnMin[0]) * (50 - 25) / (m_fVoltageOnMax[0] - m_fVoltageOnMin[0]) + 25;
            m_fVoltage[2]   = 0;
        }
        
        if (gbPvRelayN == 0)
        {
            m_fVoltage[1]   = (m_nVoltageRms[1] - m_fVoltageOffMin[1]) * (100 - 50) / (m_fVoltageOffMax[1] - m_fVoltageOffMin[1]) + 50;
            m_fVoltage[3]   = 0;
        }
        else
        {
            m_fVoltage[1]   = (m_nVoltageRms[1] - m_fVoltageOnMin[1]) * (50 - 25) / (m_fVoltageOnMax[1] - m_fVoltageOnMin[1]) + 25;
            m_fVoltage[3]   = 0;
        }

        m_fVoltageInsArray[m_nInsIndex] = m_fVoltage[0];
        m_nInsIndex++;
        m_nInsIndex %= 64;
        
        m_nSocCnt++;
   }
}

uint8_t m_bModbusBusy = 0;

static void taskAdc(void)
{
    int nLoop = 0;
    int i;
    
    OS_Delay(1000);
    
    while(1)
    {        
        OS_Delay(1);
        
        m_nAdc1DamIndex = ((DMA_Stream_TypeDef *)Adc1Handle.DMA_Handle->Instance)->NDTR;
        runAdc1();

        m_nAdc3DamIndex = ((DMA_Stream_TypeDef *)Adc3Handle.DMA_Handle->Instance)->NDTR;
        runAdc3();

        if (!m_bReqSkipAdc2)
        {
            m_nAdc2DamIndex = ((DMA_Stream_TypeDef *)Adc2Handle.DMA_Handle->Instance)->NDTR;
            runAdc2();
        }
        
        nLoop++;
        
        nLoop %= 100;
        
        if (!nLoop && (m_bModbusBusy == 0))
        {
            while(m_bModbusBusy)
            {
                OS_Delay(1);
            }

            m_bModbusBusy = 1;
            
            for (i = 0; i < 24; i++)
            {
                m_nModbusMap_04[i + 45] = swapWord(m_strAdcInfo.nStrAdcAvg[i]);
                m_nModbusMap_04[i + 10] = (int16_t)swapWord((uint16_t)(m_strAdcInfo.fStrCur[i] * 10));
            }
        
            m_nModbusMap_04[38] = swapWord(m_strAdcInfo.nStrAdcAvg[24]);
            m_nModbusMap_04[3]  = (int16_t)swapWord((uint16_t)(m_strAdcInfo.fStrCur[24] * 10));

            m_nModbusMap_04[39] = swapWord(m_strAdcInfo.nStrAdcAvg[25]);
            m_nModbusMap_04[4]  = (int16_t)swapWord((uint16_t)(m_strAdcInfo.fStrCur[25] * 10));
        
            m_nModbusMap_04[42] = swapWord(m_strAdcInfo.nStrAdcAvg[29]);
            
            //...m_nModbusMap_04[7]  = (int16_t)swapWord((uint16_t)(m_strAdcInfo.fStrCur[k] * 10));
            
            //...m_nModbusMap_04[7]  = (int16_t)swapWord((uint16_t)(m_strAdcInfo.fStrCur[k] * 10));

            m_bModbusBusy = 0;
        }
   }
}
//--------------------------------------------------------------------

int main(void)
{
    // Configure the MPU attributes
    MPU_Config();

    // Enable the CPU Cache
    // CPU_CACHE_Enable();  //Cache 처리는 OS_InitHW()에서 수행.
  
    HAL_Init();
    SystemClock_Config();	// Configure the system clock to 215.04 MHz.
    HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSE, RCC_MCODIV_1); //External MCO1 set to 20 MHz

    OS_IncDI();	//Interrupt Disable.
    OS_InitKern();
    OS_InitHW();	//HAL_Init(), SystemClock_Config()에서 처리.
    OS_DecRI();	//Interrupt Enable.
    
    fcInitPeripheral();
    fcLoadConfig();
    fcInitVariable();
    
    MAX_RST(1);

    //...fcEepTest();
    
    OS_CREATERSEMA(&SemaMem);
    OS_CREATETASK(&TCBMinute, "Minute Task", taskMinute, 30, stackMinute);
    OS_CREATETASK(&TCB_UART1, "Uart1", taskUart1,  40, stackTaskUart1);
    OS_CREATETASK(&TCB_UART3, "Uart3", taskUart3,  40, stackTaskUart3);
    OS_CREATETASK(&TCB_ADC, "Adc", taskAdc,  40, stackTaskAdc);
    OS_CREATETASK(&TCB_SOC, "Soc", taskSoc,  40, stackTaskSoc);
    OS_CREATETASK(&TCB_LOOP, "Loop", taskLoop,  1, stackTaskLoop);
    OS_CREATETASK(&TCB_HMI, "4x20 LCD Dispaly", taskHmi,  10, stackTaskHmi);
    
    //  HAL_NVIC_EnableIRQ((IRQn_Type)DMA2_Stream2_IRQn);	//SPI1(ADC Read) RX DMA Transfer Complete ISR.
    //
    //  // Sys_Tick ISR 대신 DMA2_Stream2_IRQHandler 에서 Kernel 처리.
    //  SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
    //  SysTick->LOAD  = (uint32_t)(16777216UL - 1UL);	// 2^24 - 1 , 0xFF FFFF, 12.8 Hz.

    OS_Start();

    return 0;
}
//--------------------------------------------------------------------

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 400000000 (CPU Clock), max: 480Mhz.
  *            HCLK(Hz)                       = 200000000 (AXI and AHBs Clock) max: sysclk/2(240Mhz).
  *            AHB Prescaler                  = 2
  *            D1 APB3 Prescaler              = 2 (APB3 Clock  100MHz) max: AHBs/2 (120Mhz).
  *            D2 APB1 Prescaler              = 2 (APB1 Clock  100MHz)
  *            D2 APB2 Prescaler              = 2 (APB2 Clock  100MHz)
  *            D3 APB4 Prescaler              = 2 (APB4 Clock  100MHz)
  *            HSE Frequency(Hz)              = 25000000
  *            PLL_M                          = 5
  *            PLL_N                          = 160
  *            PLL_P                          = 2
  *            PLL_Q                          = 4
  *            PLL_R                          = 2
  *            VDD(V)                         = 3.3
  *            Flash Latency(WS)              = 4
  * @param  None
  * @retval None

  *            SYSCLK(Hz)                     = 480000000 (CPU Clock), max: 480Mhz.
  *            HCLK(Hz)                       = 240000000 (AXI and AHBs Clock) max: sysclk/2(240Mhz).
  *            AHB Prescaler                  = 2
  *            D1 APB3 Prescaler              = 2 (APB3 Clock  120MHz) max: AHBs/2 (120Mhz).
  *            D2 APB1 Prescaler              = 2 (APB1 Clock  120MHz)
  *            D2 APB2 Prescaler              = 2 (APB2 Clock  120MHz)
  *            D3 APB4 Prescaler              = 2 (APB4 Clock  120MHz)
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 2
  *            PLL_N                          = 240
  *            PLL_P                          = 2
  

  *            SYSCLK(Hz)                     = 480000000 (CPU Clock), max: 480Mhz.
  *            HCLK(Hz)                       = 240000000 (AXI and AHBs Clock) max: sysclk/2(240Mhz).
  *            HSE Frequency(Hz)              = 20000000
  *            PLL_M                          = 5
  *            PLL_N                          = 240
  *            PLL_P                          = 2
  *            PLL_Q                          = 4
  *            PLL_R                          = 2
  *            VDD(V)                         = 3.3
  *            Flash Latency(WS)              = 4
  */
#define HSE_8MHZ  0
#define HSE_20MHZ 1

static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  HAL_StatusTypeDef ret = HAL_OK;
  
  /*!< Supply configuration update enable */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /* The voltage scaling allows optimizing the power consumption when the device is
     clocked below the maximum system frequency, to update the voltage scaling value
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  
  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
  RCC_OscInitStruct.CSIState = RCC_CSI_OFF;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;

#if HSE_8MHZ == 1
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 240;
#elif HSE_20MHZ == 1
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 200;
#else
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 160;
#endif
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;

  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
  if(ret != HAL_OK)
  {
    Error_Handler();
  }
  
/* Select PLL as system clock source and configure  bus clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_D1PCLK1 | RCC_CLOCKTYPE_PCLK1 | \
                                 RCC_CLOCKTYPE_PCLK2  | RCC_CLOCKTYPE_D3PCLK1);

  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;  
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2; 
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2; 
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2; 
  ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);
  if(ret != HAL_OK)
  {
    Error_Handler();
  }
}
//---------------------------------------------------------------------------
#if OS_PROFILE
unsigned int SEGGER_SYSVIEW_TickCnt;
#endif


#ifdef __cplusplus
extern "C" {
#endif
void DMA2_Stream2_IRQHandler(void);
#ifdef __cplusplus
}
#endif

/* SPI1(ADC Read) RX DMA Transfer Complete ISR.
 ADC End of Conversion (EXT15_10_IRQ) 이후 8 Card(AI, DI) 모듈 Read.
 EXT15_10_IRQ 에서 fcRdCard(0) -- CARD0 Read.
 이후 fcRdCard(1) -> fcRdCard(2) -> ... fcRdCard(7) 까지 8회 ISR 발생.
 마지막 카드 읽기-fcRdCard(7) 이후에 CARD_CS(15)로 Serial BUS 선점을 놓고 RTOS의 Keneral 처리를 960Hz로 실시.
*/
void DMA2_Stream2_IRQHandler(void)
{
//	static uint8_t sbTickDiv = 0;
//
//	OS_EnterNestableInterrupt();
//
//	if(++gbCardIdx < SLOT_NUM){
////		fcRdCard(gbCardIdx);
//	}
//	else{
//
//		//-------------------- embOS Sys_Tick Kernel 구간 ---------
//		HAL_IncTick();
//
//		if(++sbTickDiv >= TICK_DIV){
//			sbTickDiv = 0;
//			gbTickFlag = 1;
//			OS_TICK_Handle();
//		}
//		//-------------------- embOS Sys_Tick Kernel 구간 ---------
//		
//#if TEST_SV == 1
//		if(++gbDftIdx == (SAMP_CYC-1)){ // gbDftIdx: 1->2->3.... SAMP_CYC-1 -> 0(1주기 계산 완료).
//			gwPhaCycCnt++;
//			gbCycFlag = 1;
//			gbDftIdx = 0;
//		}
//#else
//		fcDiscreteFourierTransform();
//		if(gbDftIdx == (SAMP_CYC-1)){ // gbDftIdx: 1->2->3.... SAMP_CYC-1 -> 0(1주기 계산 완료).
//			gwPhaCycCnt++;
//			gbCycFlag = 1;
//		}
//#endif
//	}
//
//	OS_LeaveNestableInterrupt();
////	__HAL_DMA_CLEAR_FLAG(&hSpi1DmaRx, DMA_FLAG_TCIF2_6  | DMA_FLAG_HTIF2_6 | DMA_FLAG_TEIF2_6 | DMA_FLAG_DMEIF2_6 | DMA_FLAG_FEIF2_6);
}
//---------------------------------------------------------

/* Hardware Debugging procedure when NVIC.
void HardFault_Handler(void)
{
	static volatile uint32_t _Continue;
	_Continue = 0;

	while(_Continue == 0);
}
//---------------------------------------------------------
*/

void fcRs485Tx1(uint8_t *bStr, uint16_t nCnt)
{
	DMA2->HIFCR |= 0x0f400000;	//DMA2_Stream7 Reset. (for UART1_TX)
	DMA2_Stream7->CR &= 0xFFFFFFFE;
	DMA2_Stream7->NDTR = nCnt;
	DMA2_Stream7->M0AR = (uint32_t)bStr;
	DMA2_Stream7->CR |= 0x00000001;
	
	gst485Ctrl1.TxMode = 1;
}
//-------------------------------------------------------------
 
void fcRs485Tx3(uint8_t *bStr, uint16_t nCnt)
{
	DMA1->LIFCR |= 0x0f400000;	//DMA1_Stream3 Reset. (for UART3_TX)
	DMA1_Stream3->CR &= 0xFFFFFFFE;
	DMA1_Stream3->NDTR = nCnt;
	DMA1_Stream3->M0AR = (uint32_t)bStr;
	DMA1_Stream3->CR |= 0x00000001;
    
	gst485Ctrl3.TxMode = 1;
}
//-------------------------------------------------------------

void fcLoadConfig(void)
{
}
//-------------------------------------------------------------

void fcInitVariable(void)
{
	uint8_t i;
	float fVal, fAng;

	gbModAddr1 = 1;
	
    m_strAdcInfo.nStrAdcOffset[0]   = 32578;
    m_strAdcInfo.nStrAdcOffset[1]   = 32723;
    m_strAdcInfo.nStrAdcOffset[2]   = 32713;
    m_strAdcInfo.nStrAdcOffset[3]   = 32698;
    m_strAdcInfo.nStrAdcOffset[4]   = 32747;
    m_strAdcInfo.nStrAdcOffset[5]   = 32708;

    m_strAdcInfo.nStrAdcOffset[6]   = 32474;
    m_strAdcInfo.nStrAdcOffset[7]   = 32736;
    m_strAdcInfo.nStrAdcOffset[8]   = 32663;
    m_strAdcInfo.nStrAdcOffset[9]   = 32721;
    m_strAdcInfo.nStrAdcOffset[10]   = 32685;
    m_strAdcInfo.nStrAdcOffset[11]   = 32687;
    
    m_strAdcInfo.nStrAdcOffset[12]   = 32595;
    m_strAdcInfo.nStrAdcOffset[13]   = 32646;
    m_strAdcInfo.nStrAdcOffset[14]   = 32714;
    m_strAdcInfo.nStrAdcOffset[15]   = 32758;    
    m_strAdcInfo.nStrAdcOffset[16]   = 32707;
    m_strAdcInfo.nStrAdcOffset[17]   = 32704;
    
    m_strAdcInfo.nStrAdcOffset[18]   = 32580;
    m_strAdcInfo.nStrAdcOffset[19]   = 32745;
    m_strAdcInfo.nStrAdcOffset[20]   = 32611;
    m_strAdcInfo.nStrAdcOffset[21]   = 32800;
    m_strAdcInfo.nStrAdcOffset[22]   = 32713;
    m_strAdcInfo.nStrAdcOffset[23]   = 32661;
    
    m_strAdcInfo.nStrAdcOffset[24]   = 32768;
    m_strAdcInfo.nStrAdcOffset[25]   = 32768;
    m_strAdcInfo.nStrAdcOffset[26]   = 0;
    m_strAdcInfo.nStrAdcOffset[27]   = 0;
    m_strAdcInfo.nStrAdcOffset[28]   = 16175;
    m_strAdcInfo.nStrAdcOffset[29]   = 19;
    
    /*
	//DFT Sine, Cosine Table calculate.
	fAng = MY_PI*2.0/SAMP_CYC;
	for(i=0; i<SAMP_CYC; i++){
		fVal = -1.0*DFT_SINCOS_GAIN*sin(fAng*i);
		if(fVal > 0) fVal += 0.5;
		else fVal -= 0.5;
		gsDftSinTable[i] = (int16_t)fVal;

		fVal = DFT_SINCOS_GAIN*cos(fAng*i);
		if(fVal > 0) fVal += 0.5;
		else fVal -= 0.5;
		gsDftCosTable[i] = (int16_t)fVal;
	}
	
	gwFreqVal = 60000;
	gwFreqValOld = 60000;
	gwTim2Arr = DEFAULT_TIM2_ARR;
	
	giCosPps2Eoc = DEFAULT_GAIN;
	giSinPps2Eoc = 0;
	
	glRtcPmu = 0;
	glRtcMod = 0;
    */
}
//-------------------------------------------------------------

void fcAiCdCal(void)
{
}
//-------------------------------------------------------------
// int16_t gsWav[SAMP_CYC];

void fcDiscreteFourierTransform(void)
{
#if 0
	uint8_t i;
	int32_t lV, lCos, lSin;
	int16_t sSin, sCos;
	int32_t *lpCos, *lpSin;

/* 18bit ADC -> 16Bit 화.
	gUnRawAdc.bData[1] = gbRawAdc[0];	//Big-Endian -> Little Endian.
	gUnRawAdc.bData[0] = gbRawAdc[1];
	gUnRawAdc.bData[3] = (gbRawAdc[2]&0x3F)*4 + (gbRawAdc[3]&0xC0)/64;
	gUnRawAdc.bData[2] = (gbRawAdc[3]&0x3F)*2 + (gbRawAdc[4]&0xC0)/64;
	gUnRawAdc.bData[5] = (gbRawAdc[4]&0x0F)*16 + (gbRawAdc[5]&0xF0)/16;
	gUnRawAdc.bData[4] = (gbRawAdc[5]&0x0F)*16 + (gbRawAdc[6]&0xF0)/16;
	gUnRawAdc.bData[7] = (gbRawAdc[6]&0x03)*64 + (gbRawAdc[7]&0xFC)/4;
	gUnRawAdc.bData[6] = (gbRawAdc[7]&0x03)*64 + (gbRawAdc[8]&0xFC)/4;

	gUnRawAdc.bData[9]  = gbRawAdc[9];
	gUnRawAdc.bData[8]  = gbRawAdc[10];
	gUnRawAdc.bData[11] = (gbRawAdc[11]&0x3F)*4 + (gbRawAdc[12]&0xC0)/64;
	gUnRawAdc.bData[10] = (gbRawAdc[12]&0x3F)*2 + (gbRawAdc[13]&0xC0)/64;
	gUnRawAdc.bData[13] = (gbRawAdc[13]&0x0F)*16 + (gbRawAdc[14]&0xF0)/16;
	gUnRawAdc.bData[12] = (gbRawAdc[14]&0x0F)*16 + (gbRawAdc[15]&0xF0)/16;
	gUnRawAdc.bData[15] = (gbRawAdc[15]&0x03)*64 + (gbRawAdc[16]&0xFC)/4;
	gUnRawAdc.bData[14] = (gbRawAdc[16]&0x03)*64 + (gbRawAdc[17]&0xFC)/4; */

	//Big-Endian -> Little Endian.
	gUnRawAdc.bData[1] = gbRawAdc[0];	//1st Card.
	gUnRawAdc.bData[0] = gbRawAdc[1];
	gUnRawAdc.bData[3] = gbRawAdc[2];
	gUnRawAdc.bData[2] = gbRawAdc[3];
	gUnRawAdc.bData[5] = gbRawAdc[4];
	gUnRawAdc.bData[4] = gbRawAdc[5];
	gUnRawAdc.bData[7] = gbRawAdc[6];
	gUnRawAdc.bData[6] = gbRawAdc[7];

	gUnRawAdc.bData[9]  = gbRawAdc[8];
	gUnRawAdc.bData[8]  = gbRawAdc[9];
	gUnRawAdc.bData[11] = gbRawAdc[10];
	gUnRawAdc.bData[10] = gbRawAdc[11];
	gUnRawAdc.bData[13] = gbRawAdc[12];
	gUnRawAdc.bData[12] = gbRawAdc[13];
	gUnRawAdc.bData[15] = gbRawAdc[14];
	gUnRawAdc.bData[14] = gbRawAdc[15];
	
	gUnRawAdc.bData[16+1] = gbRawAdc[16+0];	//2nd Card.
	gUnRawAdc.bData[16+0] = gbRawAdc[16+1];
	gUnRawAdc.bData[16+3] = gbRawAdc[16+2];
	gUnRawAdc.bData[16+2] = gbRawAdc[16+3];
	gUnRawAdc.bData[16+5] = gbRawAdc[16+4];
	gUnRawAdc.bData[16+4] = gbRawAdc[16+5];
	gUnRawAdc.bData[16+7] = gbRawAdc[16+6];
	gUnRawAdc.bData[16+6] = gbRawAdc[16+7];

	gUnRawAdc.bData[16+9]  = gbRawAdc[16+8];
	gUnRawAdc.bData[16+8]  = gbRawAdc[16+9];
	gUnRawAdc.bData[16+11] = gbRawAdc[16+10];
	gUnRawAdc.bData[16+10] = gbRawAdc[16+11];
	gUnRawAdc.bData[16+13] = gbRawAdc[16+12];
	gUnRawAdc.bData[16+12] = gbRawAdc[16+13];
	gUnRawAdc.bData[16+15] = gbRawAdc[16+14];
	gUnRawAdc.bData[16+14] = gbRawAdc[16+15];

	gUnRawAdc.bData[32+1] = gbRawAdc[32+0];	//3rd Card.
	gUnRawAdc.bData[32+0] = gbRawAdc[32+1];
	gUnRawAdc.bData[32+3] = gbRawAdc[32+2];
	gUnRawAdc.bData[32+2] = gbRawAdc[32+3];
	gUnRawAdc.bData[32+5] = gbRawAdc[32+4];
	gUnRawAdc.bData[32+4] = gbRawAdc[32+5];
	gUnRawAdc.bData[32+7] = gbRawAdc[32+6];
	gUnRawAdc.bData[32+6] = gbRawAdc[32+7];

	gUnRawAdc.bData[32+9]  = gbRawAdc[32+8];
	gUnRawAdc.bData[32+8]  = gbRawAdc[32+9];
	gUnRawAdc.bData[32+11] = gbRawAdc[32+10];
	gUnRawAdc.bData[32+10] = gbRawAdc[32+11];
	gUnRawAdc.bData[32+13] = gbRawAdc[32+12];
	gUnRawAdc.bData[32+12] = gbRawAdc[32+13];
	gUnRawAdc.bData[32+15] = gbRawAdc[32+14];
	gUnRawAdc.bData[32+14] = gbRawAdc[32+15];
	
	gUnRawAdc.bData[48+1] = gbRawAdc[48+0];	//4th Card.
	gUnRawAdc.bData[48+0] = gbRawAdc[48+1];
	gUnRawAdc.bData[48+3] = gbRawAdc[48+2];
	gUnRawAdc.bData[48+2] = gbRawAdc[48+3];
	gUnRawAdc.bData[48+5] = gbRawAdc[48+4];
	gUnRawAdc.bData[48+4] = gbRawAdc[48+5];
	gUnRawAdc.bData[48+7] = gbRawAdc[48+6];
	gUnRawAdc.bData[48+6] = gbRawAdc[48+7];

	gUnRawAdc.bData[48+9]  = gbRawAdc[48+8];
	gUnRawAdc.bData[48+8]  = gbRawAdc[48+9];
	gUnRawAdc.bData[48+11] = gbRawAdc[48+10];
	gUnRawAdc.bData[48+10] = gbRawAdc[48+11];
	gUnRawAdc.bData[48+13] = gbRawAdc[48+12];
	gUnRawAdc.bData[48+12] = gbRawAdc[48+13];
	gUnRawAdc.bData[48+15] = gbRawAdc[48+14];
	gUnRawAdc.bData[48+14] = gbRawAdc[48+15];

//  gsWav[gbDftIdx] = gUnRawAdc.sData[26];
	sCos = gsDftCosTable[gbDftIdx];
	sSin = gsDftSinTable[gbDftIdx];
	lpCos = &glDftQueRe[gbDftIdx*MAX_AI_CH];
	lpSin = &glDftQueIm[gbDftIdx*MAX_AI_CH];

	gbDftIdx++;
	gbDftIdx &= (SAMP_CYC-1);	

	for(i=0; i<MAX_AI_CH; i++){
		lV = gUnRawAdc.sData[i];				// DFT는 Time 값에 sine, cos을 곱하고 한주기 평균(1/샘플수)처리를 하면 Amp가 Peak/2가 됨.
		lCos = (sCos*lV)/(SAMP_CYC/2);	// (sCos*lV : 2^13 * 2^15 = 2^28 ) / (2^6) = 2^22. @SAMP_CYC: 128.
		lSin = (sSin*lV)/(SAMP_CYC/2);	// 누적 과정에서 128샘플 (2^7)까지 최대 누적 발생 가능: 2^22 * 2^7 = 2^29, 연산과정 Overflow 없음.

		// 한주기 평균(1/샘플수) -> Peak/2 인데 (1/(샘플수/2))를 했으므로 복소값의 크기는 Peak에 해당됨.
		glDftRe[i] += lCos;
		glDftIm[i] += lSin;
#if PPS_RST_DFT
    if(!gbFlagRstDft){ //PPS 발생으로 DFT Reset, PPS가 발생하면 gbDftIdx가 틀어지며 DFT를 초기화 해야함.
      glDftRe[i] -= *lpCos;
      glDftIm[i] -= *lpSin;
    }
#else
		glDftRe[i] -= *lpCos;
		glDftIm[i] -= *lpSin;
#endif
		*lpCos++ = lCos;
		*lpSin++ = lSin;
	}

	if(!gbDftIdx){ //PMU로 전송할 복소값, 매주 1주기를 기준으로 복소값을 전송해야 함.
    gbFlagRstDft = 0;
		for(i=0; i<MAX_AI_CH; i++){
			glDftRePmu[i] = glDftRe[i];
			glDftImPmu[i] = glDftIm[i];
		}
	}
#endif    
}
//-------------------------------------------------------------

uint16_t swapWord(uint16_t w)
{
    UN_W2B b2w;
    uint8_t b;
    
    b2w.wData = w;
    b = b2w.bData[0];
    b2w.bData[0] = b2w.bData[1];
    b2w.bData[1] = b;
    
    return b2w.wData;
}


//#define SPI1_TC_WAIT	while( ((SPI1->SR &  SPI_SR_TXC) != SPI_SR_TXC) || DMA2_Stream2->NDTR )
//
//void fcSpi1Dma(uint8_t *bTxBuf, uint8_t *bRxBuf, uint16_t wDMACnt)
//{
//	DMA2->LIFCR |= 0x0f7d0000;	//DMA2_Stream3(SPI1_TX), DMA2_Stream2(SPI1_RX)
//
//	DMA2_Stream2->CR &= 0xFFFFFFFE;
//	DMA2_Stream3->CR &= 0xFFFFFFFE;
//	DMA2_Stream2->NDTR = wDMACnt;
//	DMA2_Stream3->NDTR = wDMACnt;
//
//	if(bRxBuf != NULL){
//#if CACHE_USE == 1
//		SCB_InvalidateDCache_by_Addr((uint32_t*)((uint32_t)bRxBuf & ~(uint32_t)0x1F), wDMACnt+32);
//#endif
//		DMA2_Stream2->CR |= 0x00000400;	// Set MINC (Memory increment mode).
//		DMA2_Stream2->M0AR = (uint32_t)bRxBuf;
//	}
//	else{
//		DMA2_Stream2->CR &= 0xFFFFFBFF;	// Reset MINC
//		DMA2_Stream2->M0AR = (uint32_t)gbSpi1RxBuf;
//	}
//	
//	if(bTxBuf != NULL){
//		DMA2_Stream3->CR |= 0x00000400;	// Set MINC
//		DMA2_Stream3->M0AR = (uint32_t)bTxBuf;
//	}
//	else{
//		DMA2_Stream3->CR &= 0xFFFFFBFF;	// Reset MINC
//		DMA2_Stream3->M0AR = (uint32_t)gbSpi1TxBuf;
//	}
//	DMA2_Stream2->CR |= 0x00000001;
//	DMA2_Stream3->CR |= 0x00000001;
//}
//--------------------------------------------

static void CPU_CACHE_Enable(void)
{
  /* Enable I-Cache */
  SCB_EnableICache();

  /* Enable D-Cache */
  SCB_EnableDCache();
}
//--------------------------------------------

static void Error_Handler(void)
{
	while(1){ }
}
//--------------------------------------------

static void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct;

  /* Disable the MPU */
  HAL_MPU_Disable();

  /* Configure the MPU as Strongly ordered for not defined regions */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = 0x24000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_32KB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Enable the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}
//--------------------------------------------

//void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
//{
//  // Invalidate Data Cache to get the updated content of the SRAM on the first half of the ADC converted data buffer: 32 bytes.
//  SCB_InvalidateDCache_by_Addr((uint32_t *) &gsAdc3Fifo[0], ADC_FIFO_SIZE);
//}
////--------------------------------------------
//
//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
//{
//   // Invalidate Data Cache to get the updated content of the SRAM on the second half of the ADC converted data buffer: 32 bytes.
//  SCB_InvalidateDCache_by_Addr((uint32_t *) &gsAdc3Fifo[ADC_FIFO_SIZE/2], ADC_FIFO_SIZE);
//}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
