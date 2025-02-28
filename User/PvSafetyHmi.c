#include "RTOS.h"
#include "PvSafetyHmi.h"
#include "PvSafetyRtc.h"
#include "PvSafetyMain.h"
#include "PvSafetyCalc.h"

#include <math.h>
#include <string.h>
#include <stdio.h>

extern I2C_HandleTypeDef I2c2Handle;
extern void fcErrHandler(char *sStr);
extern uint8_t gbLcdMem[];
extern UN_SOC_CMD gnSocCmd;
extern ST_METER_OBJ gstMtObj;

const uint8_t gcbCirFull[]  = {0x0e, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x0e, 0x00};	//Circle-Full.

//------------------------------------------------------------------------------------

// LCD: MCCOG42005A6W-BNMLWI.
// Driver: SSD1803a.

void fcInitLCD(void)
{
  fcLcdSndCmd(COMMAND_CLEAR_DISPLAY);
  //RE = 1  |  IS = 0
  fcLcdSndCmd(COMMAND_8BIT_4LINES_NORMAL_RE1_IS0);  //Function set : 8bit 4line RE = 1, IS = 0, Not reverse.
  fcLcdSndCmd(COMMAND_BS1_1); //Set BS1 = 1 (1/6 bias).
  fcLcdSndCmd(COMMAND_POWER_DOWN_DISABLE);  // Power down disable.
  fcLcdSndCmd(COMMAND_SEGMENT_NORMAL_WAY);  // Segment bidirection : in the right way.
  fcLcdSndCmd(COMMAND_NW);   //NW = 1.

  //RE = 0  |  IS = 1
  fcLcdSndCmd(COMMAND_8BIT_4LINES_RE0_IS1);  //Function set : RE = 0, IS = 1
  fcLcdSndCmd(COMMAND_DISPLAY_ON_CURSOR_OFF_BLINK_OFF);  //Display ON (without cursor...)
  fcLcdSndCmd(COMMAND_BS0_1);  //Set BS0 = 1 (1/6 bias) / Internal frequency.
  fcLcdSndCmd(COMMAND_INTERNAL_DIVIDER);  //Internal divider.
  fcLcdSndCmd(COMMAND_CONTRAST);          //Constrast.
  fcLcdSndCmd(COMMAND_POWER_ICON_CONTRAST);  //Power Icon control contrast.
  fcLcdSndCmd(COMMAND_FOLLOWER_CONTROL); //Follower Control.

  //RE = 0  | IS = 0
  fcLcdSndCmd(COMMAND_8BIT_4LINES_RE0_IS0);  //Function set : 8bit 4line RE = 0, IS = 0.
}
//---------------------------------------------------------

void fcLcdXy(uint8_t x, uint8_t y)
{
  uint8_t Pos;
  Pos = 20*y + x;
  fcLcdSndCmd(Pos);
}
//---------------------------------------------------------

void fcLcdMemStr(uint8_t x, uint8_t y, uint8_t *str)
{
  uint8_t *pLcd;

  pLcd = gbLcdMem + 20*y + x;
  while(*str && x++ < LCD_MAX_X){
    *pLcd++ = *str++;
  }
}
//----------------------------------------------------

void fcLcdMemStrCnt(uint8_t x, uint8_t y, uint8_t *str, uint8_t Cnt)
{
  uint8_t *pLcd;

  pLcd = gbLcdMem + 20*y + x;
  while(*str && x++ < LCD_MAX_X && Cnt--){
    *pLcd++ = *str++;
  }
}
//----------------------------------------------------

void fcLcdMem(uint8_t x, uint8_t y, uint8_t ch)
{
  uint8_t bPos;

  bPos = 20*y + x;
  if(bPos < 80){ gbLcdMem[bPos] = ch; }
}
//----------------------------------------------------

uint16_t m_nDisplayIndex = 0;

void fcLcdMemUpdate(void)
{
  memset(gbLcdMem, 0x20, 80);

  fcDispInfo();
  
  fcLcdXy(0, 0);
  fcLcdSndBuf(gbLcdMem, 80);
}
//----------------------------------------------------

void fcDispInfo(void)
{
    char sBuf[20];

    if(gnSocCmd.St.Cmd == eSOC_VI || gnSocCmd.St.Cmd == eSOC_CALING)
    {
        // Waiting Time.
        if(gstMtObj.WaitCnt)
        { 
            //12345678901234567890
            //Waiting:10, Next:10
            //Previous result
            //V:000.1mV I:000.1mA
            //R:000.1mΩ
            if(gnSocCmd.St.Cmd != eSOC_CALING)
            {
                fcLcdMemStr(0, 0, "Waiting:");
            }
            else
            {
                fcLcdMemStr(0, 0, "Calibra:");
            }
      
            sprintf(sBuf, "%02d, Next:%02d", gstMtObj.WaitCnt, gstMtObj.NowCell+1);
            fcLcdMemStr(8, 0, (uint8_t *)sBuf);
            fcLcdMemStr(0, 1, "Previous result");
            fcDispVIR();
        }
        else
        {
            //12345678901234567890
            //Testing:10 Cell:00
            //
            //V:000.1mV I:000.1mA
            //R:000.1mΩ
            fcLcdMemStr(0, 0, "Testing:");
            sprintf(sBuf, "%02d, Cell:%02d", gstMtObj.SumCnt, gstMtObj.NowCell+1);
            fcLcdMemStr(8, 0, (uint8_t *)sBuf);
            fcDispVIR();
        }
    }
    else if(gnSocCmd.St.Cmd == eSOC_NONE)
    {
        fcDisplayTitle();
        fcDispVIR();
        fcDisplayEvent();
    }
    else if(gnSocCmd.St.Cmd == eCAL_ODD_WAIT)
    {
        fcLcdMemStr(3, 1, "Odd Ch Cal Wait");
    }
    else if(gnSocCmd.St.Cmd == eCAL_EVN_WAIT)
    {
        fcLcdMemStr(3, 1, "Even Ch Cal Wait");
    }
}
//----------------------------------------------------
extern uint32_t glDtRtc;

void fcDisplayTitle(void)
{
    ST_RTC rtc;
    char sBuf[20];
    
    fcLcdMemStr(0, 0, "KETI PVSAFETY-100");
    /*
    rtc = fcLongToRtc(glDtRtc);
    sprintf(sBuf, "%02d:%02d:%02d", rtc.hh, rtc.mm, rtc.ss);
    fcLcdMemStr(12, 0, sBuf);
    */
}

void fcDisplayEvent(void)
{
	char sBuf[20];

    sprintf(sBuf, "%03dmA %05d %05d", (int)m_strAdcInfo.fStrCur[24], (int)m_strAdcInfo.nStrAdcAvg[29], m_nModbusMap_04[43]/*, (int)m_fftAvg*/);

    fcLcdMemStr(0, 3, (uint8_t *)sBuf);

    //...fcLcdMemStr(0, 3, "NO EVENT");
}

extern int32_t m_nVoltageIns[3];
extern int32_t m_nVoltageRms[3];
extern int32_t m_nCurrentIns[3];
extern int32_t m_nCurrentRms[3];
extern float m_fVoltage[4];

uint16_t m_nDisplayChangeIndex = 0;

void fcDispVIR(void)
{
	char sBuf[20];
    int nCh;
    
    m_nDisplayIndex = m_nDisplayChangeIndex;
    
    if ((m_nDisplayIndex >= 0) && (m_nDisplayIndex <= 11))
    {
        nCh = m_nDisplayIndex * 2;
        
        
        if ((m_strAdcInfo.fStrCur[nCh + 0] < 15) && (m_strAdcInfo.fStrCur[nCh + 0] > -15))
        {
            sprintf(sBuf, "> CH.%02d: %4.1f A", nCh + 1, m_strAdcInfo.fStrCur[nCh + 0]);
        }
        else
        {
            sprintf(sBuf, "> CH.%02d: ------ A", nCh + 1);
        }
        
        //sprintf(sBuf, "> CH.%02d: %7.1f", nCh + 1, m_fVoltage[0]);
        fcLcdMemStr(0, 1, (uint8_t *)sBuf);

        
        if ((m_strAdcInfo.fStrCur[nCh + 1] < 15) && (m_strAdcInfo.fStrCur[nCh + 1] > -15))
        {
            sprintf(sBuf, "> CH.%02d: %4.1f A", nCh + 2, m_strAdcInfo.fStrCur[nCh + 1]);
        }
        else
        {
            sprintf(sBuf, "> CH.%02d: ------ A", nCh + 2);
        }
        /*

        sprintf(sBuf, "> CH.%02d: %7.1f", nCh + 2, m_fVoltage[1]);
*/
        fcLcdMemStr(0, 2, (uint8_t *)sBuf);

    }
    else if (m_nDisplayIndex == 12)
    {
        sprintf(sBuf, "> ARC CURR.: %5.1f", m_strAdcInfo.fStrCur[28]);
        fcLcdMemStr(0, 1, (uint8_t *)sBuf);

        sprintf(sBuf, "> ARC LIGHT: %5.1f", m_strAdcInfo.fStrCur[29]);
        fcLcdMemStr(0, 2, (uint8_t *)sBuf);
    }
    else
    {
        sprintf(sBuf, "> MCU TEMP.: %4.1f C", m_strAdcInfo.fStrCur[27]);
        fcLcdMemStr(0, 1, (uint8_t *)sBuf);

        sprintf(sBuf, "> BATTERY  : %4.2f V", m_strAdcInfo.fStrCur[26]);
        fcLcdMemStr(0, 2, (uint8_t *)sBuf);
    }
    
    if (m_nDisplayChangeIndex > 11)
    {
        m_nDisplayIndex++;
        m_nDisplayIndex %= 12;
    }
    
/*
    float fVp, fZp, fIp;
  
    fVp = gstMtObj.vBat;
    fZp = gstMtObj.rBat;
    fIp = gstMtObj.iBat;

    if(fVp >= 1.0)
    {
        sprintf(sBuf, "V:%6.3fV", fVp);
    }
    else if(fVp >= 0.1)
    {
        sprintf(sBuf, "V:%6.1fmV", fVp*1000);
    }
    else if(fVp >= 0.01)
    {
        sprintf(sBuf, "V:%6.2fmV", fVp*1000);
    }
    else
    {
        sprintf(sBuf, "V:%6.3fmV", fVp*1000);
    }
    fcLcdMemStr(0, 2, (uint8_t *)sBuf);

    if(fIp >= 1.0)
    {
        sprintf(sBuf, "I:%5.3fA", fIp);
        fcLcdMemStr(11, 2, (uint8_t *)sBuf);
    }
    else
    {
        sprintf(sBuf, "I:%5.1fmA", fIp*1000);
        fcLcdMemStr(11, 2, (uint8_t *)sBuf);
    }

    if(fZp >= 0.001)
    {
        sprintf(sBuf, "R:%6.3fm ", fZp*1000); sBuf[9] = 0xB5; //Ω.
    }
    else
    {
        sprintf(sBuf, "R:%5.1fu ", fZp*1000000); sBuf[8] = 0xB5; //Ω.
    }

    fcLcdMemStr(0, 3, (uint8_t *)sBuf);
*/
}

// HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)

void fcLcdSndCmd(uint8_t cmd)
{
	while( HAL_I2C_Mem_Write(&I2c2Handle, (uint16_t)LCD_ADDRESS, LCD_CTRL, 1, &cmd, 1, 10000)!= HAL_OK){
		fcErrHandler("fcLcdSndCmd");
	}
}
//----------------------------------------------------

void fcLcdSndByte(uint8_t data)
{
	while( HAL_I2C_Mem_Write(&I2c2Handle, (uint16_t)LCD_ADDRESS, LCD_DATA, 1, &data, 1, 10000)!= HAL_OK){
		fcErrHandler("fcLcdSndByte");
	}
}
//------------------------------------------------------------------------------------

void fcLcdSndBuf(uint8_t *data, uint8_t byte)
{
	while( HAL_I2C_Mem_Write(&I2c2Handle, (uint16_t)LCD_ADDRESS, LCD_DATA, 1, data, byte, 10000)!= HAL_OK){
		fcErrHandler("fcLcdSndBuf");
	}
}
//------------------------------------------------------------------------------------

void fcEepWrByte(uint16_t addr, uint8_t data)
{
    if(addr > EEP_MAX_SIZE-1) return;

	while( HAL_I2C_Mem_Write(&I2c2Handle, (uint16_t)EEP_ADDRESS + (addr/256)*2, addr&0xff, 1, &data, 1, 10000)!= HAL_OK)
    {
		fcErrHandler("fcEepWrByte");
	}
}
//------------------------------------------------------------------------------------

void fcEepWrBuf(uint16_t addr, uint8_t *data, uint16_t byte)
{
    uint8_t bPage, bMod, i;
    uint16_t Pos;
    uint8_t *Tar;

    if(addr+byte > EEP_MAX_SIZE) return;

    bPage   = byte/EEP_PAGE_SIZE;   //  page count.
    bMod    = byte%EEP_PAGE_SIZE;   //  modula byte.

    Pos = addr;
    Tar = data;
    for(i = 0; i < bPage; i++)
    {
        while( HAL_I2C_Mem_Write(&I2c2Handle, (uint16_t)EEP_ADDRESS + (Pos/256)*2, Pos&0xff, 1, Tar, EEP_PAGE_SIZE, 10000)!= HAL_OK)
        {
            fcErrHandler("fcEepWrBuf_Page");
        }
        
        Pos += EEP_PAGE_SIZE;
        Tar += EEP_PAGE_SIZE;
        OS_Delay(5);    // Page write require 5ms Max.
    }
  
    if(bMod)
    { 
        //Modula byte write.
        while( HAL_I2C_Mem_Write(&I2c2Handle, (uint16_t)EEP_ADDRESS + (Pos/256) * 2, Pos & 0xff, 1, Tar, bMod, 10000)!= HAL_OK)
        {
            fcErrHandler("fcEepWrBuf_Mod");
        }
    }
}
//------------------------------------------------------------------------------------

void fcEepRdByte(uint16_t addr, uint8_t data)
{
    if(addr > EEP_MAX_SIZE-1) return;

	while( HAL_I2C_Mem_Read(&I2c2Handle, (uint16_t)EEP_ADDRESS + (addr/256)*2, addr&0xff, 1, &data, 1, 10000)!= HAL_OK)
    {
		fcErrHandler("fcEepRdByte");
	}
}
//------------------------------------------------------------------------------------

void fcEepRdBuf(uint16_t addr, uint8_t *data, uint16_t byte)
{
    if(addr+byte > EEP_MAX_SIZE) return;
 
	while( HAL_I2C_Mem_Read(&I2c2Handle, (uint16_t)EEP_ADDRESS + (addr/256)*2, addr & 0xff, 1, data, byte, 10000)!= HAL_OK)
    {
		fcErrHandler("fcEepRdBuf");
	}
}
//------------------------------------------------------------------------------------
uint8_t m_txEepBuf[512];
uint8_t m_rxEepBuf[512];

void fcEepTest(void)
{
    int i;
    
    for (i = 0; i < 256; i++)
    {
        m_txEepBuf[i]       = (uint8_t)i;
        m_txEepBuf[511 - i] = (uint8_t)i;
    }
    
    fcEepRdBuf(0, m_rxEepBuf, 512);
    fcEepWrBuf(0, m_txEepBuf, 512);
    fcEepRdBuf(0, m_rxEepBuf, 512);
}
/*
uint8_t fcFormatFloat(char *sBuf, float fTmp, ENUM_DIGIT enumDigit, char *sUnit)
{
	uint8_t k, len;
	long lTmp;
	float fAbs;

	if(fTmp < 0){
		fAbs = -fTmp;
	}
	else{
		fAbs = fTmp;
	}

	if(enumDigit == DIGIT_6){
		if(fAbs >= 1000000.0){	//Mega.
			fAbs *= 0.000001;
			fTmp *= 0.000001;
			lTmp = (long)fAbs;

			if(lTmp >= 10000){
				sprintf(sBuf, "%8.1fM", fTmp);
			}
			else if(lTmp >= 10){
				sprintf(sBuf, "%8.2fM", fTmp);
			}
			else{
				sprintf(sBuf, "%8.3fM", fTmp);
			}
			len = 9;
		}
		else if(fAbs >= 1000.0){ //Kilo.
			fAbs *= 0.001;
			fTmp *= 0.001;
			lTmp = (long)fAbs;

			if(lTmp >= 10){
				sprintf(sBuf, "%8.2fk", fTmp);
			}
			else{
				sprintf(sBuf, "%8.3fk", fTmp);
			}
			len = 9;
		}
		else{
			if(fAbs >= 10){
				sprintf(sBuf, "%8.2f ", fTmp);
			}
			else{
				sprintf(sBuf, "%8.3f ", fTmp);
			}
			len = 9;
		}
	}
	else{
		if(fAbs >= 1000.0){ //Kilo.
			fAbs *= 0.001;
			fTmp *= 0.001;
			lTmp = (long)fAbs;

			if(lTmp >= 10){
				sprintf(sBuf, "%7.2fk", fTmp);
			}
			else{
				sprintf(sBuf, "%7.3fk", fTmp);
			}
			len = 8;
		}
		else{
			if(fAbs >= 10){
				sprintf(sBuf, "%7.2f ", fTmp);
			}
			else{
				sprintf(sBuf, "%7.3f ", fTmp);
			}
			len = 8;
		}
	}

	if(sUnit){
		k = strlen(sUnit);
		memcpy(sBuf+len, sUnit, k);
		len += k;
		sBuf[len] = 0x00;
	}
	return len;
}
//----------------------------------------------------
*/

void fcIntToStr(int16_t val, uint8_t *sBuf)
{
	uint16_t wTmp, wIdx;

	wIdx = 0;
	if(val < 0){
		// *sBuf++ = '-';
		sBuf[wIdx] = '-';
		val = -val;
		wIdx++;
	}
	
	if(val >= 10000){
		wTmp = val/10000;
		sBuf[wIdx] = '0'+wTmp;	wIdx++;
		val -= (wTmp*10000);

		wTmp = val/1000;
		sBuf[wIdx] = '0'+wTmp;	wIdx++;
		val -= (wTmp*1000);
		
		wTmp = val/100;
		sBuf[wIdx] = '0'+wTmp;	wIdx++;
		val -= (wTmp*100);
		
		wTmp = val/10;
		sBuf[wIdx] = '0'+wTmp;	wIdx++;
		val -= (wTmp*10);
	}
	else if(val >= 1000){
		wTmp = val/1000;
		sBuf[wIdx] = '0'+wTmp;	wIdx++;
		val -= (wTmp*1000);

		wTmp = val/100;
		sBuf[wIdx] = '0'+wTmp;	wIdx++;
		val -= (wTmp*100);
		
		wTmp = val/10;
		sBuf[wIdx] = '0'+wTmp;	wIdx++;
		val -= (wTmp*10);
	}
	else if(val >= 100){
		wTmp = val/100;
		sBuf[wIdx] = '0'+wTmp;	wIdx++;
		val -= (wTmp*100);
		
		wTmp = val/10;
		sBuf[wIdx] = '0'+wTmp;	wIdx++;
		val -= (wTmp*10);
	}
	else if(val >= 10){
		wTmp = val/10;
		sBuf[wIdx] = '0'+wTmp;	wIdx++;
		val -= (wTmp*10);
	}

	sBuf[wIdx] = '0'+ (val%10);	wIdx++;
	sBuf[wIdx] = 0;
}
//----------------------------------------------------

void MyFloatToStr(float fVal, char *sBuf, int iForm)
{
	uint16_t wIdx;
	long lVal;

	if(iForm == 3){	//소수 3째자리까지
		if(fVal >= 0){
			lVal = (long)(fVal*1000+0.5);
		}
		else{
			lVal = (long)(fVal*1000-0.5);
		}
		wIdx = My_ltoa(lVal, sBuf);
		if(lVal < 0) lVal = -lVal;	//절대값
		
		if(lVal < 1000){
			if(lVal >= 100){//123 --> 0.123(wIdx:3), -123 --> -0.123(wIdx:4)
				sBuf[wIdx+2] = 0;
				sBuf[wIdx+1] = sBuf[wIdx-1];
				sBuf[wIdx] = sBuf[wIdx-2];
				sBuf[wIdx-1] = sBuf[wIdx-3];
				sBuf[wIdx-2] = '.';
				sBuf[wIdx-3] = '0';
			}
			else if(lVal >= 10){//12 --> 0.012(wIdx:2), -12 --> -0.012(wIdx:3)
				sBuf[wIdx+3] = 0;
				sBuf[wIdx+2] = sBuf[wIdx-1];
				sBuf[wIdx+1] = sBuf[wIdx-2];
				sBuf[wIdx  ] = '0';
				sBuf[wIdx-1] = '.';
				sBuf[wIdx-2] = '0';
			}
			else{// 8 --> 0.008(wIdx:1), -8 --> -0.008(wIdx:2)
				sBuf[wIdx+4] = 0;
				sBuf[wIdx+3] = sBuf[wIdx-1];
				sBuf[wIdx+2] = '0';
				sBuf[wIdx+1] = '0';
				sBuf[wIdx  ] = '.';
				sBuf[wIdx-1] = '0';
			}
		}
		else{ // 31012 --> 31.012(wIdx:5), -1012 --> -1.012(wIdx:5)
			sBuf[wIdx+1] = 0;
			sBuf[wIdx] = sBuf[wIdx-1];
			sBuf[wIdx-1] = sBuf[wIdx-2];
			sBuf[wIdx-2] = sBuf[wIdx-3];
			sBuf[wIdx-3] = '.';
		}
	}
	else if(iForm == 2){//
		if(fVal >= 0){
			lVal = (long)(fVal*100 + 0.5);
		}
		else{
			lVal = (long)(fVal*100 - 0.5);
		}
		wIdx = My_ltoa(lVal, sBuf);
		if(lVal < 0) lVal = -lVal;	//절대값

		if(lVal < 100){
			if(lVal >= 10){//89 --> 0.89(wIdx:2), -12 --> -0.12(wIdx:3)
				sBuf[wIdx+2] = 0;
				sBuf[wIdx+1] = sBuf[wIdx-1];
				sBuf[wIdx  ] = sBuf[wIdx-2];
				sBuf[wIdx-1] = '.';
				sBuf[wIdx-2] = '0';
			}
			else{// 8 --> 0.08(wIdx:1), -8 --> -0.08(wIdx:2)
				sBuf[wIdx+3] = 0;
				sBuf[wIdx+2] = sBuf[wIdx-1];
				sBuf[wIdx+1] = '0';
				sBuf[wIdx  ] = '.';
				sBuf[wIdx-1] = '0';
			}
		}
		else{// 1001-->10.01(wIdx:4), -10092 --> -100.92(wIdx:6)
			sBuf[wIdx+1] = 0;
			sBuf[wIdx] = sBuf[wIdx-1];
			sBuf[wIdx-1] = sBuf[wIdx-2];
			sBuf[wIdx-2] = '.';
		}
	}
	else if(iForm == 1){
		if(fVal >= 0){
			lVal = (long)(fVal*10 + 0.5);
		}
		else{
			lVal = (long)(fVal*10 - 0.5);
		}
		wIdx = My_ltoa(lVal, sBuf);
		if(lVal < 0) lVal = -lVal;	//절대값

		if(lVal < 10){// 8 --> 0.8(wIdx:1), -8 --> -0.8(wIdx:2)
			sBuf[wIdx+2] = 0;
			sBuf[wIdx+1] = sBuf[wIdx-1];
			sBuf[wIdx  ] = '.';
			sBuf[wIdx-1] = '0';
		}
		else{//1008 --> 100.8(wIdx:4) , -2004 -> 200.4(wIdx:5)
			sBuf[wIdx+1] = 0;
			sBuf[wIdx] = sBuf[wIdx-1];
			sBuf[wIdx-1] = '.';
		}
	}
}
//------------------------------------------------------------------------------------

int MyFormDec2Str(long lVal, char *sBuf, int iForm)
{
	int i, wTa;

	wTa = (int)pow(10, iForm-1);
	for(i=0; i<iForm; i++){
		sBuf[i] = lVal/wTa;
		lVal -= wTa*sBuf[i];
		wTa /= 10;
	}
	return 0;
}
//------------------------------------------------------------------------------------

void MyFormDecStr2Str(char *Src, char *Tar, int Cnt)
{
	int i, k;
	k = 0;
	for(i=0; i<Cnt; i++){
		if(!k){
			if(Src[i] == 0) Tar[i] = ' ';
			else{
				k=1;
				Tar[i] = Src[i] + '0';
			}
		}
		else{
			Tar[i] = Src[i] + '0';
		}
	}
	
	if(!k){
		Tar[Cnt-1] = '0';
	}
	
	Tar[Cnt] = 0;
}
//------------------------------------------------------------------------------------

#define BUFLEN 16
int My_ltoa(long val, char *buffer)
{
	char tempc[BUFLEN];
	register char *bufptr;
	register int   neg = val < 0;
	register long  uval = neg ? -val : val;

	*(bufptr = &tempc[BUFLEN - 1]) = 0;

	do {*--bufptr = (uval % 10) + '0';}  while(uval /= 10);
	if (neg) *--bufptr = '-';

	memcpy(buffer,bufptr, uval = (tempc + BUFLEN) - bufptr);
	return uval - 1;
}
//------------------------------------------------------------------------------------

uint16_t My_atow(char *st, uint8_t cnt)
{
	uint16_t wrst = 0;

	if(cnt == 5){
		wrst = (st[0] - '0')*10000;
		wrst += (st[1] - '0')*1000;
		wrst += (st[2] - '0')*100;
		wrst += (st[3] - '0')*10;
		wrst += (st[4] - '0');
	}
	else if(cnt == 4){
		wrst = (st[0] - '0')*1000;
		wrst += (st[1] - '0')*100;
		wrst += (st[2] - '0')*10;
		wrst += (st[3] - '0');
	}
	else if(cnt == 3){
		wrst = (st[0] - '0')*100;
		wrst += (st[1] - '0')*10;
		wrst += (st[2] - '0');
	}
	else if(cnt == 2){
		wrst = (st[0] - '0')*10;
		wrst += (st[1] - '0');
	}
	else{
		wrst = (st[0] - '0');
	}
	return wrst;
}
//------------------------------------------------------------------------------------