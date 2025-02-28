#include <string.h>
#include "PvSafetyComm.h"
#include "PvSafetyMain.h"
#include "PvSafetyRtc.h"
#include "PvSafetyCalc.h"
#include "RTOS.h"

const uint16_t CrcTblMOD[256] = {
	0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
	0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
	0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
	0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
	0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
	0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
	0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
	0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
	0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
	0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
	0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
	0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
	0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
	0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
	0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
	0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
	0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
	0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
	0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
	0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
	0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
	0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
	0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
	0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
	0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
	0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
	0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
	0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
	0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
	0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
	0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
	0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

extern OS_RSEMA SemaMem;

extern uint8_t gbModAddr1;
extern uint8_t gbModAddr3;
extern uint8_t gbHostRx[], gbHostTx[], gbPmuRx[], gbPmuTx[], gbSvRx[], gbSvTx[];

extern int32_t glDftRePmu[];	//채널별 Real + jImag
extern int32_t glDftImPmu[];
extern uint16_t gwDiRaw[];

extern ST_SYS_SET gstSysSet;
extern ST_COM_CFG gstComCfg;
extern ST_EVENT_IDX gstSysEvIdx, gstTrgEvIdx;
extern ST_CAL gstCal;
extern ST_CAL_CMD gstCalCmd;

extern ST_RTC gstRtc;
extern uint32_t glDtRtc, glRtcMod, glRtcPmu;
extern uint8_t gbGpsRtcSynch;
extern uint16_t gwSboCnt1;
extern uint16_t gwSboCnt3;
extern uint8_t gbTurnOnPmuTcp, gbTurnOnSv;
extern uint16_t gwPhaCycCnt;
extern uint16_t gwSockClCnt;
extern uint16_t gwFreqVal, gwFreqValOld;

extern uint32_t glSysTickValBetweenSampSum;
extern int16_t giCosPps2Eoc, giSinPps2Eoc;
extern uint8_t m_bModbusBusy;

void fcEtherModbus(uint8_t *RxBuf, uint8_t *TxBuf, uint16_t *TxCnt)
{
	uint8_t bErrCode;

    while(m_bModbusBusy)
    {
        OS_Delay(1);
    }
    
    m_bModbusBusy = 2;
    
	*TxCnt = 0;
	
    TxBuf[0] = RxBuf[0];
	TxBuf[1] = RxBuf[1];
    
	switch(RxBuf[1]){ //어떤 Tx Packet이든 생성해야 함.
		case READ_INPUT_REGISTERS:{	//30001~..
			bErrCode = fcReadInputReg(RxBuf, TxBuf, TxCnt);
		} break;
		case READ_HOLDING_REGISTERS:{	//40001~..
			bErrCode = fcReadHoldingReg(RxBuf, TxBuf, TxCnt);
		} break;
		case FORCE_SINGLE_COIL:{
			bErrCode = fcSingleCoilReg(RxBuf, TxBuf, TxCnt);
		} break;
		case WRITE_SINGLE_REGISTER:{
			bErrCode = fcWriteSingleReg(RxBuf, TxBuf, TxCnt);
		} break;
		case WRITE_MULTIPLE_REGISTERS:{
			bErrCode = fcWriteMultiReg(RxBuf, TxBuf, TxCnt);
		} break;
		default:{
			bErrCode = ILLEGAL_FUNCTION;
		}
	}
	
	if(bErrCode)
    {
		TxBuf[0] = gbModAddr1;
		TxBuf[1] = RxBuf[1] | 0x80;
		TxBuf[2] = bErrCode;
		*TxCnt = 3;
	}
    
    m_bModbusBusy = 0;
}
//------------------------------------------
	
uint8_t fcReadInputReg(uint8_t *RxBuf, uint8_t *TxBuf, uint16_t *bTxCnt)
{
	uint16_t wAddr, wNum, i, j;
	int32_t lVal;
    
    //	uint8_t *pOr = TxBuf;
	TxBuf += 2;

	//  Address of register.
	wAddr = (uint16_t)(RxBuf[2])*256 + RxBuf[3];

	//  Number of register.
	wNum = (uint16_t)(RxBuf[4])*256 + RxBuf[5];

	//  TxBuf의 사이즈가 256Byte임 따라서 버퍼관리를 위해 250BYTEs 로 제한 함.
	if(wNum >= 126 || wNum == 0) 
    {
        return(ILLEGAL_DATA_VALUE);
    }    
    //    마지막 Register Address.
	else if( (wAddr + wNum) <= INPUT_END_ADDR)
    { 
		wNum *= 2;
		*TxBuf++ = wNum;	//  Byte Count.
        
        memcpy((char *)TxBuf, (char *)&m_nModbusMap_04[wAddr], wNum);   TxBuf += wNum;
	}
	else
    {	
        return(ILLEGAL_DATA_ADDRESS); 
    }

	// *bTxCnt = (TxBuf-pOr);
	*bTxCnt = 3+wNum;
	
    return 0;
}
//--------------------------------------------

uint8_t fcReadHoldingReg(uint8_t *RxBuf, uint8_t *TxBuf, uint16_t *bTxCnt)
{
	uint16_t wAddr, wNum, i;
	float fVal;
	uint8_t *pOr = TxBuf;
	TxBuf += 2;

	//Address of register.
	wAddr = (uint16_t)(RxBuf[2])*256 + RxBuf[3];

	//Number of register.
	wNum = (uint16_t)(RxBuf[4])*256 + RxBuf[5];

	if(wAddr < RHR_PMU_CFG_RCNT){
		if(wNum >= 126 || wNum == 0) return(ILLEGAL_DATA_VALUE);
		else if((wAddr + wNum) > (RHR_PMU_CFG+RHR_PMU_CFG_RCNT)){ return(ILLEAGL_ADU_LENGTH); }
		else{
			wNum *= 2;
			*TxBuf++ = wNum;
			memcpy(TxBuf, ((uint8_t *)&gstSysSet) + wAddr*2, wNum);
			TxBuf += wNum;
		}
	}
	else{
		switch(wAddr){
			case RHR_COM_CFG:{
				if(wNum != RHR_COM_CFG_RCNT){ return(ILLEAGL_ADU_LENGTH); }
				else{
					wNum *= 2;
					*TxBuf++ = wNum;
					memcpy(TxBuf, (uint8_t *)&gstComCfg, wNum);
					TxBuf += wNum;
				}
			} break;
			case RHR_PHA_FACT:{
				if(wNum != RHR_PHA_FACT_RCNT){ return(ILLEAGL_ADU_LENGTH); }
				else{
					wNum *= 2;
					*TxBuf++ = wNum;
  	
					for(i=0; i<MAX_AI_CH; i++){
						if(gstSysSet.stAiCfg[i].Bit.Type == 0){ //전압.
							fVal = (float)gstSysSet.stAiCfg[i].fDisp/gstSysSet.stAiCfg[i].fRate*gfVrmsGain/SQRT_2;
						}
						else{ //전류.
							fVal = (float)gstSysSet.stAiCfg[i].fDisp/gstSysSet.stAiCfg[i].fRate*gfIrmsGain/SQRT_2;
						}
						memcpy(TxBuf, (uint8_t *)&fVal, 4);
						TxBuf += 4;
					}
				}
			} break;
			case RHR_CAL_INFO:{
				if(wNum != RHR_CAL_INFO_RCNT){ return(ILLEAGL_ADU_LENGTH); }
				else{
					wNum *= 2;
					*TxBuf++ = wNum;
					memcpy(TxBuf, (uint8_t *)&gstCal, wNum);
					TxBuf += wNum;
				}
			} break;
			case RHR_RTC_INFO:{
				if(wNum != RHR_RTC_INFO_RCNT){ return(ILLEAGL_ADU_LENGTH); }
				else{
					wNum *= 2;
					*TxBuf++ = wNum;
					fcNowRtc(0);
					fcRtcDecToBcdStr(gstRtc, TxBuf, 0);
					TxBuf += 6;
				}
			} break;
			default:{	return(ILLEGAL_DATA_ADDRESS); }
		}
	}

	*bTxCnt = (TxBuf-pOr);
	return 0;
}
//--------------------------------------------

uint8_t fcSingleCoilReg(uint8_t *RxBuf, uint8_t *TxBuf, uint16_t *bTxCnt)
{
	uint16_t wAddr, wVal;

	//Address of register.
	wAddr = (uint16_t)(RxBuf[2])*256 + RxBuf[3];

	//Value of register.
	wVal = (uint16_t)(RxBuf[4])*256 + RxBuf[5];

    switch(wAddr)
    {
		case FSC_INIT_SYSEV:
        {
			if(gwSboCnt1 == 0)
            { 
                return (SBO_TIME_OUT); 
            }
			else if(wVal != 0xFF00)
            { 
                return(ILLEGAL_DATA_VALUE); 
            }
			else if( OS_Request(&SemaMem) ) 
            {
				memset((uint8_t *)&gstSysEvIdx, 0x00, 4);
				//fcSaveToMem(eSYS_EVIDX);
				OS_Unuse(&SemaMem);
			}
			else
            { 
                return(SLAVE_DEVICE_BUSY); 
            }
		} 
        break;
        
		case FSC_INIT_TRGEV:
        {
			if(gwSboCnt1 == 0)
            { 
                return (SBO_TIME_OUT); 
            }
			else if(wVal != 0xFF00)
            { 
                return(ILLEGAL_DATA_VALUE); 
            }
			else if( OS_Request(&SemaMem) ) 
            {
				memset((uint8_t *)&gstTrgEvIdx, 0x00, 4);
				//fcSaveToMem(eTRG_EVIDX);
				OS_Unuse(&SemaMem);
			}
			else
            { 
                return(SLAVE_DEVICE_BUSY); 
            }
		} 
        break;
        
		case FSC_SBO:
        {
			if(wVal != 0xFF00)
            { 
                return(ILLEGAL_DATA_VALUE); 
            }
			else
            { 
                gwSboCnt1 = SBO_TICK_CNT; 
            }
		} 
        break;
        
		case FSC_BACKLIGHT_ON:
        {
			if (wVal == 0xFF00) 
            { 
                BL_CTRL(1); 
            }
            else if (wVal == 0x0000)
            {
                BL_CTRL(0); 
            }
            else
            {
                return(ILLEGAL_DATA_VALUE); 
            }
		} 
        break;

		case FSC_CLEAR_ARC_CNT:
        {
			if(wVal != 0xFF00)
            { 
                return(ILLEGAL_DATA_VALUE); 
            }
			else
            { 
                m_nModbusMap_04[43] = 0;
                m_nModbusMap_04[44] = 0;
            }
		} 
        break;

        default: return(ILLEGAL_DATA_ADDRESS);
	}

	memcpy(TxBuf+2, RxBuf+2, 4);	//FORCE_SINGLE_COIL 그대로 Rx의 0~5를 전송.
	*bTxCnt = 6;
	return 0;
}
//--------------------------------------------

uint8_t fcWriteSingleReg(uint8_t *RxBuf, uint8_t *TxBuf, uint16_t *bTxCnt)
{
	uint8_t bHi, bLo, bCd, i;
	uint16_t wAddr;

	//Address of register.
	wAddr = (uint16_t)(RxBuf[2])*256 + RxBuf[3];

	//Single register value to write.
	bHi = RxBuf[4];
	bLo = RxBuf[5];
	if(wAddr == PSR_ADDR_CTRL)
    {
		switch(bHi){
			case 0xCA:{ //Input module Calibration.
				bCd = bLo&0x0f;	//AI 카드 번호.
				bLo &= 0xf0;
				if( bLo == 0){ // AI 카드 보정값 초기화.
					if(bCd & 0x01) { for(i=0; i<8; i++){ gstCal.wGain[i] = DEFAULT_GAIN; } }    //AI 카드1 보정값 초기화.
					if(bCd & 0x02) { for(i=0; i<8; i++){ gstCal.wGain[i+8] = DEFAULT_GAIN; } }  //AI 카드2 보정값 초기화.
					if(bCd & 0x04) { for(i=0; i<8; i++){ gstCal.wGain[i+16] = DEFAULT_GAIN; } }  //AI 카드3 보정값 초기화.
					if(bCd & 0x08) { for(i=0; i<8; i++){ gstCal.wGain[i+24] = DEFAULT_GAIN; } }  //AI 카드4 보정값 초기화.
				}
				else if(bLo == 0x10){ // 카드 보정 시작.
					*(uint8_t *)&gstCalCmd = 0;
						gstCalCmd.Cmd = 1;
						gstCalCmd.Card = bCd;
				}
				else if(bLo == 0x20){ // 보정 값 저장.
					if( OS_Request(&SemaMem) ) {
						//fcSaveToMem(eCAL);
						OS_Unuse(&SemaMem);
					}
					else{ return(SLAVE_DEVICE_BUSY); }
				}
				else { return(ILLEGAL_DATA_VALUE); }
			} break;
			case 0xCE:{ //Check Sensor Connection.
				if(bLo == 1){ //Input Sensor.
					*(uint8_t *)&gstCalCmd = 0;
					// gstCalCmd.AdcTest = 1;
				}
				else if(bLo == 2){ //Non-volatile Memory.
				}
				else if(bLo == 3){ //RAM.
				}
				else{ return(ILLEGAL_DATA_VALUE); }
			} break;
			case 0xBD:{ //Reboot.
				if(bLo == 0xDA){
					NVIC_SystemReset();	//Software reset.
					while(1);
				}
				else{ return(ILLEGAL_DATA_VALUE); }
			} break;
			default: return(ILLEGAL_DATA_VALUE);
		}
	}
	else { return(ILLEGAL_DATA_ADDRESS); }
	
	memcpy(TxBuf+2, RxBuf+2, 4);	//WRITE_SINGLE_REGISTERS은 그대로 Rx의 0~5를 전송.
	*bTxCnt = 6;
	return 0;
}
//--------------------------------------------

uint8_t fcWriteMultiReg(uint8_t *RxBuf, uint8_t *TxBuf, uint16_t *bTxCnt)
{
	uint16_t wAddr, wNum;

	//Address of register.
	wAddr = (uint16_t)(RxBuf[2])*256 + RxBuf[3];

	//Number of register.
	wNum = (uint16_t)(RxBuf[4])*256 + RxBuf[5];

	if(wAddr < RHR_PMU_CFG_RCNT){
		if(wNum >= 126 || wNum == 0) return(ILLEGAL_DATA_VALUE);
		else if((wAddr + wNum) > (RHR_PMU_CFG+RHR_PMU_CFG_RCNT)){ return(ILLEAGL_ADU_LENGTH); }
		else{
			memcpy((uint8_t *)&gstSysSet + wAddr*2, RxBuf+7, wNum*2);
			if((wAddr + wNum) == (RHR_PMU_CFG+RHR_PMU_CFG_RCNT)){ //FRAM에 저장 필요.
				if( OS_Request(&SemaMem) ){
					//fcSaveToMem(eSYS_CFG);
					OS_Unuse(&SemaMem);
				}
				else{ return(SLAVE_DEVICE_BUSY); }
			}
		}
	}
	else{
		switch(wAddr){
			case PMR_ADDR_COM_CFG:{
				if(wNum != RHR_COM_CFG_RCNT){ return(ILLEAGL_ADU_LENGTH); }
				if( OS_Request(&SemaMem) ){
					memcpy((uint8_t *)&gstComCfg, RxBuf+7, wNum*2);
					//fcSaveToMem(eCOM_CFG);
					OS_Unuse(&SemaMem);
				}
				else{ return(SLAVE_DEVICE_BUSY); }
			} break;
			case PMR_ADDR_RTC_SET:{
				if(wNum != RHR_RTC_INFO_RCNT){ return(ILLEAGL_ADU_LENGTH); }
				if( fcStrToRtc(RxBuf+7, 1) ) { return(ILLEGAL_DATA_VALUE); }
			} break;
			default:{	return(ILLEGAL_DATA_ADDRESS); }
		}
	}
	memcpy(TxBuf+2, RxBuf+2, 4);	//PRESET_MULTIPLE_REGISTERS은 그대로 Rx의 0~5를 전송.
	*bTxCnt = 6;
	return 0;
}
//--------------------------------------------

void fcBroadCast1(uint8_t *RxBuf)
{
	uint16_t wStartAddr, wNum;
	UN_W2B unW2B;
	UN_D2B unD2B;

	if(RxBuf[1]== WRITE_MULTIPLE_REGISTERS){
		//Start Address.
		unW2B.bData[0] = RxBuf[3];	//BitB :MSB
		unW2B.bData[1] = RxBuf[2];
		wStartAddr = unW2B.wData;

		//Register Count.
		unW2B.bData[0] = RxBuf[5];
		unW2B.bData[1] = RxBuf[4];
		wNum = unW2B.wData;

		if(wStartAddr == PMR_REBOOT && wNum == 2){
			memcpy(unD2B.bData, RxBuf+7, 4);
			if(unD2B.lData == 0x12345678){
				NVIC_SystemReset();	//Software reset.
				while(1);
			}
		}
	}
}
//--------------------------------------------

void fcBroadCast3(uint8_t *RxBuf)
{
	uint16_t wStartAddr, wNum;
	UN_W2B unW2B;
	UN_D2B unD2B;

	if(RxBuf[1]== WRITE_MULTIPLE_REGISTERS){
		//Start Address.
		unW2B.bData[0] = RxBuf[3];	//BitB :MSB
		unW2B.bData[1] = RxBuf[2];
		wStartAddr = unW2B.wData;

		//Register Count.
		unW2B.bData[0] = RxBuf[5];
		unW2B.bData[1] = RxBuf[4];
		wNum = unW2B.wData;

		if(wStartAddr == PMR_REBOOT && wNum == 2){
			memcpy(unD2B.bData, RxBuf+7, 4);
			if(unD2B.lData == 0x12345678){
				NVIC_SystemReset();	//Software reset.
				while(1);
			}
		}
	}
}
//--------------------------------------------

uint16_t fcCrc16Mod(uint8_t *data, uint16_t datacnt)
{
	uint16_t x, crc;
	uint16_t i;

	crc = 0xffff;
    
	for(i=0; i<datacnt; i++) 
    {
		x = crc ^ data[i];
		crc = (crc >> 8) ^ CrcTblMOD[x & 0xff];
	}

	return(crc);
}
//------------------------------------------
