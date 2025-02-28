#ifndef __PREDEF_H
#define __PREDEF_H

#include "stm32h7xx_hal.h"

typedef enum {eKnob1, eKnob2, eKnob3, eKnob4, eKnob5, eBtn, ePlug, eNone } enumRdCs;
typedef enum {eSOC_NONE=0, eSOC_VI=1, eSOC_CAL_INIT, eCAL_ODD_WAIT, eCAL_EVN_WAIT, eSOC_CAL_SAV, eSOC_CALING, eSOC_RESET } ENUM_SOC_CMD;

#define CACHE_USE	1
#define SYSTICK_CLOCK_TEST 0
#define PPS_RST_DFT 0
#define RESAMP_SYS  1

#define HMI_MS		1000

#define LCD_ADDRESS 0x78
#define LCD_CTRL    0x00
#define LCD_DATA    0x40

#define EEP_ADDRESS 0xA0
#define EEP_PAGE_SIZE  16
#define EEP_MAX_SIZE 512

#define EEP_POS_SYSCFG  0
#define EEP_SIZE_SYSCFG 16

#define EEP_POS_CAL    32
//#define EEP_SIZE_CAL   138

#define EEP_POS_EVIDX  176
#define EEP_SIZE_EVIDX 6

#define EEP_POS_EV(d) (184 + EEP_SIZE_EV*d)
#define EEP_SIZE_EV    6
#define EEP_MAX_EV     32

#define LED_ON	0
#define LED_OFF	1

#define STRING_NO 24

#define SBO_SEC		3
#define SBO_TICK_CNT	(SBO_SEC*1000)

#define UART_FIFO_SIZE	128
#define UART_MAX_PACKET_SIZE	1024

#define EVENT_FIFO_CNT 8
#define DEFAULT_GAIN   16384

#define CARD_AI_NUM	4
#define CARD_DI_NUM	4
#define CARD_AI_CH	8
#define CARD_DI_CH	16

#define	MAX_AI_CH	(CARD_AI_CH*CARD_AI_NUM)
#define	MAX_DI_CH	(CARD_DI_CH*CARD_DI_NUM)

#define RTC_ASYNCH_PREDIV       99U
#define RTC_SYNCH_PREDIV        9U

#define LED_ON_ISR	2;

#define SYS_CLK	215040000L			// 2^14 * 13,125.
#define PMU_TIME_BASE	16777215L // 값이 2^24 이하가 되어야 함. PMU_TIME_BASE = 1초, 즉 16777216L -> 1초, 8388608 -> 0.5초.
// #define SYS_CLK_TO_FRACT_NUM	1024		//215040000(Variable) * 1024 / 13125 ≒ 11/141 (99.9938) 
// #define SYS_CLK_TO_FRACT_DEN	13125
// 본래 1024(Numerator), 13125(Denominator)로 처리하기 위해서는 __int64 변수 필요.
#define SYS_CLK_TO_FRACT_NUM	11
#define SYS_CLK_TO_FRACT_DEN	141

// APB1 Clock (53,760,000 Hz) = SYS_CLK/4
// Timer Clock = APB1 x 2 (107,520,000 Hz)
// 7680 Hz --> 14,000 = 107,520,000/7680
// 1920 Hz --> 56,000 = 107,520,000/1920

#define ADC_CH1                     16
#define SAMP_CYC1                128
#define ADC_FIFO_CYC1           4      //  Must be 2의 승수.(2^x)
#define ADC_FIFO_SIZE1           (ADC_CH1 * SAMP_CYC1 * ADC_FIFO_CYC1)
#define ADC_FIFO_IDX_MASK1   (ADC_FIFO_SIZE1 - 1)

#define ADC_CH2                     2
#define SAMP_CYC2                2048
#define ADC_FIFO_CYC2           4      //  Must be 2의 승수.(2^x)
#define ADC_FIFO_SIZE2           (ADC_CH2 * SAMP_CYC2 * ADC_FIFO_CYC2)
#define ADC_FIFO_IDX_MASK2   (ADC_FIFO_SIZE2 - 1)

#define ADC_CH3                     12
#define SAMP_CYC3                128
#define ADC_FIFO_CYC3           4      //  Must be 2의 승수.(2^x)
#define ADC_FIFO_SIZE3           (ADC_CH3 * SAMP_CYC3 * ADC_FIFO_CYC3)
#define ADC_FIFO_IDX_MASK3   (ADC_FIFO_SIZE3 - 1)

#define SYS_FREQ  60
#define SAMP_FREQ	(SYS_FREQ*SAMP_CYC)
#define TICK_DIV	(SAMP_FREQ/960)
#define TIMER_CLK	(SYS_CLK/4*2)
#define DEFAULT_TIM2_ARR	(TIMER_CLK/SAMP_FREQ)
#define TIM2_ARR_HIGH	(TIMER_CLK/45)	//45Hz
#define TIM2_ARR_LOW  (TIMER_CLK/65) //65Hz

//#define AVERAGE_FREQ_CYC	56  // rPMU Firmware Info.xlsx 주파수 측정(Tim3)탭 참조. (7, 14, 21, 28, 35, 42, 49, 56... 에서 선택).
//#define DEN_T2ARR	128         // AVERAGE_FREQ_CYC(7, 14, 21 ... 56...)에 대해서 DEN_T2ARR (16, 32, 48 ... 128...)
#define AVERAGE_FREQ_CYC  7     // rPMU Firmware Info.xlsx 주파수 측정(Tim3)탭 참조. (7, 14, 21, 28, 35, 42, 49, 56... 에서 선택).
#define DEN_T2ARR  16           // AVERAGE_FREQ_CYC(7, 14, 21 ... 56...)에 대해서 DEN_T2ARR (16, 32, 48 ... 128...)
#define T3_CNT_1CYC	32000
#define SUM_T3_CNT_AVGCYC	(T3_CNT_1CYC*AVERAGE_FREQ_CYC)
#define FREQ_CAL_CONST (SUM_T3_CNT_AVGCYC/DEN_T2ARR*SYS_FREQ*1000)	//1000, mHz.
#define VALID_CYC_MIN_CLOCK	(SYS_CLK/65L)  //65Hz
#define VALID_CYC_MAX_CLOCK	(SYS_CLK/45L)  //45Hz
#define VALID_SAMP_MIN_CLOCK	(VALID_CYC_MIN_CLOCK/SAMP_CYC)  //65Hz
#define VALID_SAMP_MAX_CLOCK	(VALID_CYC_MAX_CLOCK/SAMP_CYC)  //45Hz

#define DFT_SINCOS_GAIN	8192	// 1.0 -> 8192(DFT_SINCOS_GAIN).

#define SQRT_2  1.4142135623730950488016887242097
#define MY_PI	  3.1415926535897932384626433832795
#define DIV_2PI 0.1591549430918953357688837633725 	// 1/(2*PI)
#define RAD_2_DEG	(180.0/MY_PI)
#define DEG_2_RAG	(MY_PI/180.0)
#define RAD_2_DEG_100	(RAD_2_DEG*100)

#define PPS_ANG_SHIFT_CONST	(2*MY_PI*0.001/SYS_CLK)

#define PT_RATIO  (2.78/240.0)     // 2.780:240 (GET PT)
#define CT_RATIO  (1.0/2500.0)     // 2,500:1 (태화 CT: TZ77V )
#define PT_RATE	  110.0
#define CT_RATE	  5.0
#define CT_BURDEN	88.4	//44.2옴 2개 (max 100A).

#define ADIN_at_Vn (PT_RATE*PT_RATIO)	           // 정격전압(110V) 입력시 ADC에 입력 전압[mV, RMS], (150/11000*110) -> 1.5V
#define ADIN_at_In (CT_RATE*CT_RATIO*CT_BURDEN)  // 정격전류(5.0)시 ADC에 입력 전압[mV, RMS], 260옴*2개, (540*5.0/2500) -> 1.08V

#define MAX_ADC_VAL	32767 // 2^15.
#define GAIN_ADC (MAX_ADC_VAL/5.0)   // 5.0V가 입력되면 ADC값은 32767로 표시됨.
#define ADC_RMS_at_Vn   (ADIN_at_Vn*GAIN_ADC)
#define ADC_RMS_at_In   (ADIN_at_In*GAIN_ADC)

#define gfVrmsGain (PT_RATE/ADC_RMS_at_Vn)
#define gfIrmsGain (CT_RATE/ADC_RMS_at_In)

#define CAL_CT	5.0
#define CAL_PT	110.0
#define CAL_CYC	8

#define CUTOFF_CT	(CT_RATE*0.02)	// 2% 미만 Cut-Off.
#define CUTOFF_PT	(PT_RATE*0.10)	//10% 미만 Cut-Off.
#define SQ_CUT_ADC 429497L        // (2% 미만 MAX_ADC_VAL)^2 (0.02*32768)^2 : 429,497

#define PMU_NAME_SIZE	16
typedef struct{
	char sPmuName[PMU_NAME_SIZE];
	char sAIName[MAX_AI_CH*PMU_NAME_SIZE];
	char sDIName[MAX_DI_CH*PMU_NAME_SIZE];
} ST_CH_NAME;	//16 + 32*16 + 64*16 = 1552 Byte.

typedef struct{
	float fDisp;	//정격 입력시 표시값.
	float fRate;	//정격 값.
	struct {
		uint8_t Type:  2;	    // 0:PT, 1:CT, 2: TD, 3: DC.
		uint8_t Phase: 2;     // 0:R상, 1:S상, 2: T상. 3: N상.
		uint8_t Bus: 3;       // Bus 번호 (0~7)
		uint8_t Mode: 1;      // Phase-Neutral(0), Phase-Phase(1)
		uint8_t Feeder: 4;    // Feeder 번호(0~15)
		uint8_t TrgHigh:  1;	// Trigger at high amplitude.
		uint8_t TrgLow:   1;	// Trigger at low amplitude.
		uint8_t TrgSlope: 1;	// Trigger at slope change.
		uint8_t PmuPha:   1;	// Send to PMU Phasor.			
	} Bit;
	uint16_t TrgHiPercent;	// x10 Percent, 1234 -> 123.4%
	uint16_t TrgLoPercent;	// x10 Percent, 432  -> 43.2%
	uint8_t TrgSlPercent;   // x10 Percent, 25   -> 2.5%
	uint8_t TrgSlCyc;       // Slope trigger cycle.
	uint8_t res[4];
} ST_AI_CFG;	// 20 Byte.

typedef struct{
	ST_CH_NAME stName;
	ST_AI_CFG	stAiCfg[MAX_AI_CH];
	uint16_t wDiType[CARD_DI_NUM]; // bit 0: NO, 1: NC.
	uint16_t wDiTrg[CARD_DI_NUM];  // bit 0: Disable, 1: Enable.
	uint16_t wPmuId;               // PMU ID.
	uint16_t wPmuDataRate;         // 60주기마다 1회, 1: 1주기마다 1회, 5주기마다 1회...
	struct {
		uint8_t Freq: 1;	    // 0:60Hz, 1:50Hz
		uint8_t PmuDI1: 1;
		uint8_t PmuDI2: 1;
		uint8_t PmuDI3: 1;
		uint8_t PmuDI4: 1;
		uint8_t Res1: 3;
		uint8_t Res2: 4;
		uint8_t Res3: 4;
	} Bit;
	uint16_t wCrc;
} ST_SYS_SET;	//2216 Byte.

typedef struct{
	uint8_t ubIp[4];		// Ip.
	uint8_t ubGw[4];		// Gateway.
	uint8_t ubSm[4];		// Subnet Mask.
	uint16_t wModBusId1; // Modbus Mac 주소.
	uint16_t wBaudRate1;	// Serial Baud Rate.
	uint16_t wModBusId3; // Modbus Mac 주소.
	uint16_t wBaudRate3;	// Serial Baud Rate.
	uint16_t wRes;
	uint16_t wCrc;
} ST_COM_CFG;	//20 Byte.

typedef struct{
	uint8_t Ip[4];		// Ip.
	uint16_t Port;      // Port
} ST_UDP_CLIENT;

typedef struct {
	uint8_t bKind: 2;
	uint8_t bL1: 4;
	uint8_t bOnOff: 2;
	uint8_t bL2: 6;
	uint8_t bLR: 1;
} ST_EVENT_BIT;

typedef struct { //메모리 순서를 빠꾸면 안됨.
	uint32_t lTime;
	ST_EVENT_BIT stEv;
} ST_SYS_EVENT;

typedef struct { //메모리 순서를 빠꾸면 안됨.
	uint8_t Idx;
	uint8_t Cnt;
	uint8_t IdxC;
	uint8_t res;
	uint16_t wCrc;
} ST_EVENT_IDX;

typedef struct {
	uint8_t SysEv:  1;    // 1: System Event Exist.
	uint8_t TrgEv:  1;    // 1: Trigger Event Exist.
	uint8_t FreqSt: 1;    // 0: 45~65 else 1
	uint8_t CalSt:  1;    // 1: Calibrated.
	uint8_t errDMA: 1;    // 1: Error.
	uint8_t errGPS: 1;
	uint8_t errLAN: 1;
	uint8_t err485: 1;
	uint8_t reserve;
	uint16_t CardInf;      // Msb(Card5)-Card6-7-8-Card1-2-3-Lsb(Card4), 0:DI, 1: AI, 2:TD, 3:None.
} ST_PMU_ST;

typedef struct{
	uint16_t wGain[MAX_AI_CH];
	uint16_t wRes;
	uint16_t wCrc;
} ST_CAL;	//68 Byte

typedef struct{
	uint8_t Card: 4;
	uint8_t Cmd:  2;
} ST_CAL_CMD;

typedef union{
	uint32_t lData;		//0x12345678
	int32_t slData;
	float fData;
	uint8_t bData[4];	//[0]:0x78, [1]:0x56, [2]:0x34, [3]:0x12
	uint16_t wData[2];
} UN_D2B;

typedef union{
	int16_t sData;
	uint16_t wData;		//0x1234
	uint8_t bData[2];	//[0]:0x34, [1]:0x12
} UN_W2B;

typedef struct {
  uint8_t SCLL;  // SCL low period (master mode), tSCLL = (SCLL+1) x tPRESC.
  uint8_t SCLH;  // SCL high period (master mode),
  uint8_t SDADEL: 4; // Data hold time, tSDADEL= SDADEL x tPRESC
	uint8_t SCLDEL: 4; // Data setup time, tSCLDEL = (SCLDEL+1) x tPRESC
	uint8_t res:    4;
	uint8_t PRESC:  4; // Timing prescaler, tPRESC = (PRESC+1) x tI2CCLK.
} ST_I2C_TIMING;

typedef union{
	uint32_t lData;		//0x12345678
  ST_I2C_TIMING stTiming;
} UN_I2C_TIMING;

typedef union{
	int16_t sData[MAX_AI_CH];
	uint8_t bData[MAX_AI_CH*2];
} UN_RAW_ADC;

typedef struct {
	uint8_t TimOut;			//ms.
	uint8_t NewPacket;	//'1'-New Frame Received.
	uint16_t RcvdCnt;		//수신 바이트.
	uint8_t TxMode;			//'1'-New Frame Sent.
} ST_485_CTRL;

typedef union {
	struct {
		uint8_t Res1;
		uint8_t Res2: 1;
		uint8_t GainOrOffset: 1;
		uint8_t Temp: 1;
		uint8_t Ch: 7;
		uint8_t Cmd;
	} St;
	uint8_t Byte[4];
} UN_SOC_CMD;

#pragma pack(push, 2)
typedef struct {
	uint32_t cSt;        //Ri Status.  (0:Normal, 1:Abnormal).
	uint32_t cWarn;      //Ri Warning. (0:Normal, 1:Abnormal).
  uint16_t wZc[STRING_NO];
  uint16_t wVc[STRING_NO];
  uint16_t wIc[STRING_NO];
  float rBat;
  float vBat;
  float iBat;
  struct {
    uint8_t Ch:  8;
    uint8_t Op:  2;
    uint8_t res: 6;
  } Inf;
  uint16_t reserved;
  uint32_t RmsV14;
  uint32_t RmsV4;
  uint32_t RmsIsh;
  uint32_t RmsIry;
  uint32_t SumV14;
  uint32_t SumV4;
  uint32_t SumIsh;
  uint32_t SumIry;
  uint32_t PrevSumV14;
  uint32_t PrevSumV4;
  uint32_t PrevSumIsh;
  uint32_t PrevSumIry;
  uint8_t NowCell;
  uint8_t ContCell;
  uint8_t SumCnt;
  uint8_t WaitCnt;
  uint8_t StableSig;
} ST_METER_OBJ;
#pragma pack(pop)

typedef struct {
	uint16_t wEvent;
	uint16_t wDate;
	uint32_t lTime;
} DATETIME;

typedef struct {
	uint8_t YY;
	uint8_t MM;
	uint8_t DD;
	uint8_t hh;
	uint8_t mm;
	uint8_t ss;
	uint8_t DoW;
	uint16_t ms;
} ST_RTC;

typedef union{
	ST_RTC st;
	uint8_t	byte[9];
} UN_RTC;

#define BL_CTRL(d)  ((d == 1) ? ( GPIOF->BSRR = GPIO_PIN_15 ) : ( GPIOF->BSRR = (uint32_t)GPIO_PIN_15 << 16))
#define LCD_RST(d)  ((d == 1) ? ( GPIOF->BSRR = GPIO_PIN_2 ) : ( GPIOF->BSRR = (uint32_t)GPIO_PIN_2 << 16))

#define CB_TRIP(d)  ((d == 1) ? ( GPIOG->BSRR = GPIO_PIN_0  ) : ( GPIOG->BSRR = (uint32_t)GPIO_PIN_0 << 16))
#define ARC_SHORT(d)  ((d == 1) ? ( GPIOG->BSRR = GPIO_PIN_1  ) : ( GPIOG->BSRR = (uint32_t)GPIO_PIN_1 << 16))
#define LED1(d)  ((d == 1) ? ( GPIOG->BSRR = GPIO_PIN_2  ) : ( GPIOG->BSRR = (uint32_t)GPIO_PIN_2 << 16))
#define LED2(d)  ((d == 1) ? ( GPIOG->BSRR = GPIO_PIN_3  ) : ( GPIOG->BSRR = (uint32_t)GPIO_PIN_3 << 16))
#define RY_PVP(d)  ((d == 1) ? ( GPIOG->BSRR = GPIO_PIN_9  ) : ( GPIOG->BSRR = (uint32_t)GPIO_PIN_9 << 16))
#define RY_PVN(d)  ((d == 1) ? ( GPIOG->BSRR = GPIO_PIN_10 ) : ( GPIOG->BSRR = (uint32_t)GPIO_PIN_10 << 16))
#define MAX_RST(d) ((d == 1) ? ( GPIOG->BSRR = GPIO_PIN_11 ) : ( GPIOG->BSRR = (uint32_t)GPIO_PIN_11 << 16))
#define SOC_CS(d)  ((d == 1) ? ( GPIOG->BSRR = GPIO_PIN_15  ) : ( GPIOG->BSRR = (uint32_t)GPIO_PIN_15 << 16))
	
#endif
