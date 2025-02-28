#include <stdint.h>
#include "PvSafetyRtc.h"

extern RTC_HandleTypeDef RtcHandle;
extern uint32_t glDtRtc, glRtcMod;
extern ST_RTC gstRtc;

#define RTC_ADDRESS 0x64
extern I2C_HandleTypeDef I2cHandle;

//	1월:31일, 2월(윤년이면 29, 아니면 28), 3월:31일, 4월:30일, 5월:31일, 6월:30일
//	7월:31일, 8월:31일, 9월:30일, 10월:31일, 11월:30일, 12월:31일	
const uint8_t gcbDayOfMonNorm[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
const uint16_t gcbSumDayOfMonNorm[] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334, 365};

// const uint8_t gcbDayOfMonYoon[] = {31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
const uint16_t gcbSumDayOfMonYoon[] = {0, 31, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335, 366};

const uint16_t gcbSumDaysOfYear[] = {0, 366, 366+365, 366+365+365};
extern void fcErrHandler(char *);

#define RTC_VLF	0x02
#define RTC_TE	0x10

uint8_t gbRtcBuf[8];

void fcNowRtc(uint8_t bDoW)
{
    RTC_DateTypeDef stRtcDate;
    RTC_TimeTypeDef stRtcTime;

    // Get the RTC current Time.
    HAL_RTC_GetTime(&RtcHandle, &stRtcTime, RTC_FORMAT_BIN);

    // Get the RTC current Date.
    HAL_RTC_GetDate(&RtcHandle, &stRtcDate, RTC_FORMAT_BIN);

    gstRtc.YY = (uint8_t)stRtcDate.Year;
    gstRtc.MM = (uint8_t)stRtcDate.Month;
    gstRtc.DD = (uint8_t)stRtcDate.Date;

    if(bDoW){ gstRtc.DoW = (uint8_t)stRtcDate.WeekDay; }

    gstRtc.hh = (uint8_t)stRtcTime.Hours;
    gstRtc.mm = (uint8_t)stRtcTime.Minutes;
    gstRtc.ss = (uint8_t)stRtcTime.Seconds;
	
    glDtRtc = fcRtcToLong(gstRtc);
    glRtcMod = 0;
}
//------------------------------------------------------------------------------------

void fcResetTime(void)
{
  RTC_DateTypeDef sdatestructure;
  RTC_TimeTypeDef stimestructure;

  // Set Date: Monday 6 6 2022
  sdatestructure.Year  = 22;
  sdatestructure.Month = 6;
  sdatestructure.Date  = 6;
  sdatestructure.WeekDay = 1;	//Monday.
  if(HAL_RTC_SetDate(&RtcHandle,&sdatestructure, RTC_FORMAT_BIN) != HAL_OK){
    fcErrHandler("HAL_RTC_SetDate");
  }

  // Set Time: 10:00:00
  stimestructure.Hours = 10;
  stimestructure.Minutes = 0x00;
  stimestructure.Seconds = 0x00;
  stimestructure.SubSeconds = 0;
  stimestructure.TimeFormat = RTC_HOURFORMAT12_AM;
  stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE ;
  stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;
  if(HAL_RTC_SetTime(&RtcHandle, &stimestructure, RTC_FORMAT_BIN) != HAL_OK){
    fcErrHandler("HAL_RTC_SetTime");
  }
}
//------------------------------------------------------------------------------------

void fcRtcDecToBcdStr(ST_RTC stRtc, uint8_t *BcdStr, uint8_t bDoW)
{
	uint8_t bRead;
	bRead = stRtc.YY;
	*BcdStr++ = (bRead/10)*16 + (bRead % 10);
	
	bRead = stRtc.MM;
	*BcdStr++ = (bRead/10)*16 + (bRead % 10);

	bRead = stRtc.DD;
	*BcdStr++ = (bRead/10)*16 + (bRead % 10);

	if(bDoW){
		*BcdStr++ = fcGregorianDayofWeek(stRtc.YY, stRtc.MM, stRtc.DD);
	}

	bRead = stRtc.hh;
	*BcdStr++ = (bRead/10)*16 + (bRead % 10);

	bRead = stRtc.mm;
	*BcdStr++ = (bRead/10)*16 + (bRead % 10);
	
	bRead = stRtc.ss;
	*BcdStr++ = (bRead/10)*16 + (bRead % 10);
}
//------------------------------------------------------------------------------------

ST_RTC fcBcdStrToRtcInfo(uint8_t *BcdStr, uint8_t bDoW)
{
	uint8_t i, bRead;
	ST_RTC stRtc;
	
	bRead = *BcdStr++;
	stRtc.ss = (bRead >>4)*10 + (bRead & 0x0f);

	bRead = *BcdStr++;
	stRtc.mm = (bRead >>4)*10 + (bRead & 0x0f);

	bRead = *BcdStr++;
	stRtc.hh = (bRead >>4)*10 + (bRead & 0x0f);

	if(bDoW){
		bRead = *BcdStr++;
		for(i=0; i<7; i++){
			if( (bRead & (0x01 <<i)) ) break;
		}
		stRtc.DoW = i;
	}

	bRead = *BcdStr++;
	stRtc.DD = (bRead >>4)*10 + (bRead & 0x0f);

	bRead = *BcdStr++;
	stRtc.MM = (bRead >>4)*10 + (bRead & 0x0f);

	bRead = *BcdStr++;
	stRtc.YY = (bRead >>4)*10 + (bRead & 0x0f);
	
	return stRtc;
}
//------------------------------------------------------------------------------------

// const uint8_t gcsDow[] = { 7, 1, 2, 3, 4, 5, 6 };

uint8_t fcGregorianDayofWeek(uint16_t dY, uint8_t dM, uint8_t dD)
{
	//Sunday - 0, Monday-1 ... Saturday-6.

	uint16_t dYY, dMM, dMtmp;
	dMtmp = (14 - dM)/12;
	
	dYY = dY - dMtmp;
	dMM = dM + (12*dMtmp) - 2;

	dMtmp = dD + dYY;
	dMtmp += dYY/4;
	dMtmp -= (dYY/100);
	dMtmp += (dYY/400);
	dMtmp += ( 31*dMM / 12 );
	dMtmp = dMtmp%7;

	return (uint8_t)dMtmp;
//	return gcsDow[dMtmp];
}
//-----------------------------------------------------------------------

void fcWriteToPeripheral(ST_RTC stRtc, uint8_t bInit)
{
  RTC_DateTypeDef stRtcDate;
  RTC_TimeTypeDef stRtcTime;
	
  if(bInit){
    fcResetTime();
  }
  else{
    stRtcDate.Year = stRtc.YY;
    stRtcDate.Month = stRtc.MM;
    stRtcDate.Date = stRtc.DD;
    stRtcDate.WeekDay = stRtc.DoW;

    stRtcTime.Hours   = stRtc.hh;
    stRtcTime.Minutes = stRtc.mm;
    stRtcTime.Seconds = stRtc.ss;
    stRtcTime.SubSeconds = 0;
    stRtcTime.TimeFormat = RTC_HOURFORMAT12_AM;
    stRtcTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE ;
    stRtcTime.StoreOperation = RTC_STOREOPERATION_RESET;	

    if(HAL_RTC_SetDate(&RtcHandle, &stRtcDate, RTC_FORMAT_BIN) != HAL_OK){
      fcErrHandler("HAL_RTC_SetDate");
    }

    if(HAL_RTC_SetTime(&RtcHandle, &stRtcTime, RTC_FORMAT_BIN) != HAL_OK){
      fcErrHandler("HAL_RTC_SetTime");
    }

    gstRtc = stRtc;
    glDtRtc = fcRtcToLong(gstRtc);
	
    HAL_RTCEx_BKUPWrite(&RtcHandle, RTC_BKP_DR0, 0x32F2);
	}
}
//-----------------------------------------------------------------------

void fcWordToDate(uint16_t wDate, ST_RTC *stRtc)
{
	uint8_t i;
	stRtc->YY = wDate/YOONBASE;	stRtc->YY *= 4;
	wDate = wDate%YOONBASE;

//    0 --> 0/1/1,  365 --> 0/12/31.
//  366 --> 1/1/1,  730 --> 1/12/31.
//  731 --> 2/1/1, 1095 --> 2/12/31.
// 1096 --> 3/1/1, 1460 --> 3/12/31.

	if(wDate >= gcbSumDaysOfYear[3]){
		stRtc->YY += 3;
		wDate -= gcbSumDaysOfYear[3];
	}
	else if(wDate >= gcbSumDaysOfYear[2]){
		stRtc->YY += 2;
		wDate -= gcbSumDaysOfYear[2];
	}
	else if(wDate >= gcbSumDaysOfYear[1]){
		stRtc->YY += 1;
		wDate -= gcbSumDaysOfYear[1];
	}

	if(stRtc->YY%4){	//평년.
		for(i=1; i<=12; i++){
    	if(wDate < gcbSumDayOfMonNorm[i]) {
    		stRtc->MM = i;
    		stRtc->DD = wDate+1-gcbSumDayOfMonNorm[i-1];
    		break;
    	}
    }
	}
	else{
		for(i=1; i<=12; i++){
    	if(wDate < gcbSumDayOfMonYoon[i]) {
    		stRtc->MM = i;
    		stRtc->DD = wDate+1-gcbSumDayOfMonYoon[i-1];
    		break;
    	}
    }
	}
}
//------------------------------------------

uint16_t fcDateToWord(ST_RTC Date)
{
	uint16_t Days;
	uint8_t bY;

	Days  = (Date.YY/4)*YOONBASE;
	bY = Date.YY&0x03;
	Days += gcbSumDaysOfYear[bY];
	
	if(bY){ Days += (Date.DD + gcbSumDayOfMonNorm[Date.MM-1]); }	//평년.
	else{ Days += (Date.DD + gcbSumDayOfMonYoon[Date.MM-1]); }	//윤년.
	Days--;

	return Days;
}
//------------------------------------------

ST_RTC fcLongToRtc(uint32_t lTime)
{
	uint16_t wDate;
	ST_RTC stRtc;

	wDate = lTime/86400L;	lTime -= wDate*86400L;
	fcWordToDate(wDate, &stRtc);

	stRtc.hh = lTime/3600;
	lTime -= 3600*stRtc.hh;

	stRtc.mm = lTime/60;
	lTime -= 60*stRtc.mm;
	stRtc.ss = lTime;

	stRtc.DoW = fcGregorianDayofWeek(stRtc.YY, stRtc.MM, stRtc.DD);
	
	return stRtc;
}
//------------------------------------------

uint32_t fcRtcToLong(ST_RTC stRtc)
{
	uint32_t lTime;
	uint16_t wDate;

	lTime = stRtc.ss;
	lTime += (stRtc.mm*60);
	lTime += (stRtc.hh*3600);
	
	wDate = fcDateToWord(stRtc);
	lTime += wDate*86400L;
	return lTime;
}
//------------------------------------------

void fcLongToTime(uint32_t lInfo, ST_RTC *stRtc)
{
	lInfo = lInfo % 86400;

	stRtc->hh = lInfo/3600;
	lInfo -= (3600L*stRtc->hh);

	stRtc->mm = lInfo/60;
	lInfo -= 60*stRtc->mm;

	stRtc->ss = lInfo;
}
//------------------------------------------

uint8_t fcStrToRtc(uint8_t *Buf, uint8_t Bcd)
{
	ST_RTC stRtc;

	if(Bcd){
		stRtc.YY = (Buf[0]/16)*10 + (Buf[0]&0x0f);
		stRtc.MM = (Buf[1]/16)*10 + (Buf[1]&0x0f);
		stRtc.DD = (Buf[2]/16)*10 + (Buf[2]&0x0f);
		stRtc.hh = (Buf[3]/16)*10 + (Buf[3]&0x0f);
		stRtc.mm = (Buf[4]/16)*10 + (Buf[4]&0x0f);
		stRtc.ss = (Buf[5]/16)*10 + (Buf[5]&0x0f);
	}
	else{
		stRtc.YY = Buf[0];
		stRtc.MM = Buf[1];
		stRtc.DD = Buf[2];
		stRtc.hh = Buf[3];
		stRtc.mm = Buf[4];
		stRtc.ss = Buf[5];
	}
	
	if( fcChkRtc(stRtc) ) return 1;

	stRtc.DoW = fcGregorianDayofWeek(stRtc.YY, stRtc.MM, stRtc.DD);
	fcWriteToPeripheral(stRtc, 0);
	
	return 0;
}
//------------------------------------------

uint8_t fcChkRtc(ST_RTC stRtc)
{
	uint8_t bC;

	if(stRtc.MM == 0 || stRtc.MM >= 13){ return 2; }
	else{
		if(stRtc.MM != 2){
			if(stRtc.DD > gcbDayOfMonNorm[stRtc.MM-1] || stRtc.DD == 0 ) return 3;
		}
		else{
			if(!(stRtc.YY%4) && (stRtc.YY%100)){ bC = 29; }	//윤년.
			else{ bC = 28;}
			if(stRtc.DD > bC || stRtc.DD == 0 ) return 3;
		}
	}

	if(stRtc.hh >= 24 || stRtc.mm >= 60 || stRtc.ss >= 60)	return 4;
	else return 0;
}
//------------------------------------------

void fcDtToStr(uint32_t lDt, uint8_t *Buf, uint8_t Bcd)
{
	ST_RTC stRtc;

	stRtc = fcLongToRtc(lDt);

	if(Bcd){
		Buf[0] = (stRtc.YY/10)*16 + (stRtc.YY % 10);
		Buf[1] = (stRtc.MM/10)*16 + (stRtc.MM % 10);
		Buf[2] = (stRtc.DD/10)*16 + (stRtc.DD % 10);
		Buf[3] = (stRtc.hh/10)*16 + (stRtc.hh % 10);
		Buf[4] = (stRtc.mm/10)*16 + (stRtc.mm % 10);
		Buf[5] = (stRtc.ss/10)*16 + (stRtc.ss % 10);
	}
	else{
		Buf[0] = stRtc.YY;
		Buf[1] = stRtc.MM;
		Buf[2] = stRtc.DD;
		Buf[3] = stRtc.hh;
		Buf[4] = stRtc.mm;
		Buf[5] = stRtc.ss;
	}
}
//------------------------------------------

void fcUtcToRtc(ST_RTC stRtc)
{
	uint32_t nSec;
	nSec = fcRtcToLong(stRtc);
	nSec += (UTC_TO_LOCAL_DIFF * 3600);		  	
	stRtc = fcLongToRtc(nSec);
	stRtc.DoW = fcGregorianDayofWeek(stRtc.YY, stRtc.MM, stRtc.DD);
	fcWriteToPeripheral(stRtc, 0);
}
//------------------------------------------
