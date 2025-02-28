#ifndef __BATRS_HMI_H
#define __BATRS_HMI_H

#include "PvSafetyPreDef.h"

#define COMMAND_CLEAR_DISPLAY           0x01
#define COMMAND_RETURN_HOME             0x02
#define COMMAND_ENTRY_MODE_SET          0x04
	#define ENTRY_MODE_LEFT_TO_RIGHT      0x02
	#define ENTRY_MODE_SHIFT_INCREMENT    0x01
#define COMMAND_CURSORSHIFT             0x10
	#define CURSORSHIFT_MOVERIGHT         0x04
	#define CURSORSHIFT_MOVELEFT          0x00

#define COMMAND_ADDRESS_DDRAM           0x80
#define COMMAND_ADDRESS_CGRAM           0x40

#define COMMAND_8BIT_4LINES_NORMAL_RE1_IS0    0x3A  //Extended command access RE = 1
#define COMMAND_8BIT_4LINES_REVERSE_RE1_IS0   0x3B  //Extended command access RE = 1
#define COMMAND_8BIT_4LINES_RE0_IS1           0x39  //Extended command access IS = 1
#define COMMAND_8BIT_4LINES_RE0_IS0           0x38  //Normal mode...

//Command from extended set (RE = 1, IS = 0)
#define COMMAND_BS1_1                 0x1E
#define COMMAND_POWER_DOWN_DISABLE    0x02
#define COMMAND_SEGMENT_NORMAL_WAY    0x05
#define COMMAND_NW                    0x09
#define COMMAND_ROM_SELECTION         0x72

//Command from extended set (RE = 0, IS = 1)
#define COMMAND_DISPLAY_ON_CURSOR_ON_BLINK_ON    0x0F
#define COMMAND_DISPLAY_ON_CURSOR_ON_BLINK_OFF   0x0E
#define COMMAND_DISPLAY_ON_CURSOR_OFF_BLINK_OFF  0x0C
#define COMMAND_DISPLAY_OFF_CURSOR_OFF_BLINK_OFF 0x08
#define COMMAND_BS0_1               0x1C
#define COMMAND_INTERNAL_DIVIDER    0x13
#define COMMAND_CONTRAST            0x77
#define COMMAND_POWER_ICON_CONTRAST 0x5C
#define COMMAND_FOLLOWER_CONTROL    0x6E

#define LCD_MAX_X	20
#define LCD_MAX_Y	4

void fcInitLCD(void);
void fcLcdSndCmd(uint8_t cmd);
void fcLcdSndByte(uint8_t data);
void fcLcdSndBuf(uint8_t *data, uint8_t byte);

void fcEepWrByte(uint16_t addr, uint8_t data);
void fcEepRdByte(uint16_t addr, uint8_t data);
void fcEepWrBuf(uint16_t addr, uint8_t *data, uint16_t byte);
void fcEepRdBuf(uint16_t addr, uint8_t *data, uint16_t byte);
void fcEepTest(void);

void fcLcdXy(uint8_t x, uint8_t y);
void fcLcdMemStr(uint8_t x, uint8_t y, uint8_t *str);
void fcLcdMemStrCnt(uint8_t x, uint8_t y, uint8_t *str, uint8_t Cnt);
void fcLcdMem(uint8_t x, uint8_t y, uint8_t ch);
void fcLcdMemUpdate(void);
void fcDispInfo(void);
void fcDispVIR(void);
void fcIntToStr(int16_t val, uint8_t *);
void fcDisplayTitle(void);
void fcDisplayEvent(void);

void MyFloatToStr(float fVal, char *sBuf, int iForm);
void MyDateToStr(uint16_t wDate, char *sBuf);
void MyTimeToStr(uint32_t lTime, char *sBuf, uint8_t bms);
void DateTimeToStr(uint32_t lDT, char *sBuf);
int MyFormDec2Str(long lVal, char *sBuf, int iForm);
void MyFormDecStr2Str(char *Src, char *Tar, int Cnt);
int My_ltoa(long val, char *buffer);
uint16_t My_atow(char *st, uint8_t cnt);

#endif
