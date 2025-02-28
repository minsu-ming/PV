#ifndef __PVSFT_RTC_H
#define __PVSFT_RTC_H

#include "PvSafetyPredef.h"

#define YOONBASE	(4*365 + 1)	//4년 윤년 포함...
#define BASEYEAR	2000
#define DATETIME_LIMIT	36525
#define UNIX_BASE	946684800		//2000.1.1 0:0:0 의 Unix 타임.

#define SECCNT		1000
#define MINCNT		(60*SECCNT)			// SECCNT * 60
#define HOURCNT		(3600*SECCNT)		// SECCNT * 3600(초)
#define DAYCNT		(86400*SECCNT)	// SECCNT * 86400(초)

#define	UTC_TO_LOCAL_DIFF	9

void fcNowRtc(uint8_t bDoW);
void fcResetTime(void);
uint8_t fcGregorianDayofWeek(uint16_t dY, uint8_t dM, uint8_t dD);
void fcWriteToPeripheral(ST_RTC stRtcInfo, uint8_t bInit);
void fcWordToDate(uint16_t wDate, ST_RTC *stRtc);
uint16_t fcDateToWord(ST_RTC Date);
ST_RTC fcLongToRtc(uint32_t lTime);
uint32_t fcRtcToLong(ST_RTC stRtc);
void fcLongToTime(uint32_t lInfo, ST_RTC *stRtc);
uint8_t fcChkRtc(ST_RTC stRtc);
uint8_t fcStrToRtc(uint8_t *Buf, uint8_t Bcd);
void fcRtcDecToBcdStr(ST_RTC stRtc, uint8_t *BcdStr, uint8_t bDoW);
ST_RTC fcBcdStrToRtcInfo(uint8_t *BcdStr, uint8_t bDoW);
void fcDtToStr(uint32_t lDt, uint8_t *Buf, uint8_t Bcd);
void fcUtcToRtc(ST_RTC stRtc);
#endif
