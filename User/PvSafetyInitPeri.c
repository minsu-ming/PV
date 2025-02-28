#include "PvSafetyMain.h"
#include "PvSafetyInitPeri.h"
#include "PvSafetyRtc.h"

static void fcInitGpio(void);
static void fcInitAdc(void);
static void fcInitI2C(void);
static void fcInitUART(void);
static void fcInitTim1(void);
static void fcInitTim2(void);
static void fcInitRtc(void);
static void fcInitSPI(void);

ADC_HandleTypeDef Adc1Handle;
ADC_HandleTypeDef Adc2Handle;
ADC_HandleTypeDef Adc3Handle;
ADC_ChannelConfTypeDef AdChConfig;

I2C_HandleTypeDef I2c2Handle;
UART_HandleTypeDef Uart1Handle;
UART_HandleTypeDef Uart3Handle;
TIM_HandleTypeDef Tim1Handle, Tim2Handle;
RTC_HandleTypeDef RtcHandle;

__IO ITStatus UartReady = RESET;

SPI_HandleTypeDef Spi6Handle;
DMA_HandleTypeDef hSpi6DmaTx, hSpi6DmaRx;

DMA_HandleTypeDef hAdc1Dma, hUart1DmaTx, hUart1DmaRx;
DMA_HandleTypeDef hAdc2Dma, hUart2DmaTx, hUart2DmaRx;
DMA_HandleTypeDef hAdc3Dma, hUart3DmaTx, hUart3DmaRx;

extern uint16_t gsAdc1Fifo[];
extern uint16_t gsAdc2Fifo[];
extern uint16_t gsAdc3Fifo[];

extern uint8_t gbU1RxNdtrOld;
extern uint8_t gbU1RxFifo[];

extern uint8_t gbU3RxNdtrOld;
extern uint8_t gbU3RxFifo[];

// APBsCLK: 400Mhz /4 =  100 Hz.

void fcInitPeripheral(void)
{
    fcInitGpio();
    fcInitAdc();
    fcInitUART();
    fcInitI2C();
    fcInitRtc();
    fcInitSPI();

    //  MPU_Config();
    fcInitTim1(); // Chk Pulse.
    fcInitTim2();	// For SOC(Start of Conversion) of ADC. Not use ISR, just generate trigger signal as 7680Hz.
	
    HAL_TIM_PWM_Start(&Tim1Handle, TIM_CHANNEL_2);
    
    //...HAL_DMA_Start(&hAdc3Dma, (uint32_t)(&AdcHandle.Instance->DR), (uint32_t)gsAdc3Fifo, ADC_FIFO_SIZE);
    HAL_ADC_Start_DMA(&Adc1Handle, (uint32_t *)gsAdc1Fifo, ADC_FIFO_SIZE1);
    HAL_ADC_Start_DMA(&Adc2Handle, (uint32_t *)gsAdc2Fifo, ADC_FIFO_SIZE2);
    HAL_ADC_Start_DMA(&Adc3Handle, (uint32_t *)gsAdc3Fifo, ADC_FIFO_SIZE3);

    HAL_Delay(1);

    HAL_TIM_Base_Start(&Tim2Handle);  //Start ADC conversion.    

    //...while(1);
}
//---------------------------------------------------------------------------

void fcInitGpio(void)
{
    GPIO_InitTypeDef  gpio_init_structure;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();

    CB_TRIP(0);
    ARC_SHORT(0);

    LCD_RST(1);
    BL_CTRL(1);
    SOC_CS(1);
    MAX_RST(0);

    // F2(LCD_RST), F15(BL_CTRL)
    gpio_init_structure.Pin = GPIO_PIN_2 | GPIO_PIN_15;
    gpio_init_structure.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init_structure.Pull = GPIO_NOPULL;
    gpio_init_structure.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOF, &gpio_init_structure);

    //  G0(CB_TRIP), G1(ARC_SHORT), G2(LED1), G3(LED2), G9(RY_PVP), G10(RY_PVN), G11(/RST_MAX), G15(/SPI6_CS)
    gpio_init_structure.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_15;
    gpio_init_structure.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init_structure.Pull = GPIO_NOPULL;
    gpio_init_structure.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOG, &gpio_init_structure);

    //  PG4..7(DIP1~4)
    gpio_init_structure.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    gpio_init_structure.Mode = GPIO_MODE_INPUT;
    gpio_init_structure.Pull = GPIO_NOPULL;
    gpio_init_structure.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOG, &gpio_init_structure);

    //  PA8(20MHz)
    gpio_init_structure.Pin     = GPIO_PIN_8;
    gpio_init_structure.Mode    = GPIO_MODE_OUTPUT_PP;
    gpio_init_structure.Pull    = GPIO_PULLDOWN;
    gpio_init_structure.Speed   = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio_init_structure);
    
    HAL_Delay(2);

    LED1(LED_OFF);
    LED2(LED_OFF);
    BL_CTRL(1);
}
//---------------------------------------------------------------------------

#define ADC_OVER_SAMPLING  0

void fcInitAdc(void)
{
    GPIO_InitTypeDef  gpio_init_structure;

//    *((uint32_t *)(ADC3_BASE + 308)) |= (ADC_CCR_VREFEN | ADC_CCR_TSEN | ADC_CCR_VBATEN);

    __HAL_RCC_ADC_CONFIG(RCC_ADCCLKSOURCE_CLKP); //HSI(Default), HSE??

    __HAL_RCC_ADC12_CLK_ENABLE();
    __HAL_RCC_ADC3_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    //  A6(ADC12_P3, StrCur2), A7(ADC12_P7, StrCur6), A2(ADC12_P14, StrCur11), A3(ADC12_P15, StrCur12)
    //  A0(ADC1_P16, StrCur13), A1(ADC1_P17, StrCur14), A4(ADC12_P18, StrCur15), A5(ADC12_P19, StrCur16)
    gpio_init_structure.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5;
    gpio_init_structure.Mode = GPIO_MODE_ANALOG;
    gpio_init_structure.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio_init_structure);

    //  B1(ADC12_P5, StrCur4), B0(ADC12_P9, StrCur8)
    gpio_init_structure.Pin = GPIO_PIN_1 | GPIO_PIN_0;
    gpio_init_structure.Mode = GPIO_MODE_ANALOG;
    gpio_init_structure.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &gpio_init_structure);

    //  C4(ADC1_P4, StrCur3), C5(ADC1_P8, StrCur7), C0(ADC123_P10, StrCur9), C1(ADC123_P11, StrCur10)
    //  C2(ADC3_P0, StrCur17, C3(ADC3_P1, StrCur18)
    gpio_init_structure.Pin = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_0 | GPIO_PIN_1;
    gpio_init_structure.Mode = GPIO_MODE_ANALOG;
    gpio_init_structure.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &gpio_init_structure);

    //  F11(ADC1_P2, StrCur1), F12(ADC1_P6, StrCur6), PF13(ADC2_P2, ArcCur), PF14(ADC2_P6, ArcLight) 
    //  F9(ADC3_P2, StrCur19),  F7(ADC3_P3, StrCur20), F5(ADC3_P4, StrCur21),  F3(ADC3_P5, StrCur22)
    //  F10(ADC3_P6, StrCur23), F8(ADC3_P7, StrCur24), F6(ADC3_P8, LeakCurPV), F4(ADC3_P9, PT1000)
    gpio_init_structure.Pin = GPIO_PIN_9 | GPIO_PIN_7 | GPIO_PIN_5 | GPIO_PIN_3 | GPIO_PIN_10 | GPIO_PIN_8 | GPIO_PIN_6 | GPIO_PIN_4 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14;
    gpio_init_structure.Mode = GPIO_MODE_ANALOG;
    gpio_init_structure.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOF, &gpio_init_structure);

/*
    // Depend on __HAL_RCC_ADC_CONFIG & HAL_RCCEx_PeriphCLKConfig
	// AHB4:200 MHz, ADCCLK = APB2/4 -> 22.5MHz.
	// Tconv = Sampling Time + 12 Cycle.
	// if SamplingTime == 3Cycle, Tconv = 15Cycle (0.67us)
	// if SamplingTime == 15Cycle, Tconv = 27Cycle (1.2us)
	// if SamplingTime == 28Cycle, Tconv = 40Cycle (1.8us)

	Adc1Handle.Instance                   = ADC1;
	Adc1Handle.Init.ClockPrescaler        = ADC_CLOCKPRESCALER_PCLK_DIV4;
	Adc1Handle.Init.Resolution            = ADC_RESOLUTION_12B;
	Adc1Handle.Init.ScanConvMode          = ENABLE;
	Adc1Handle.Init.ContinuousConvMode    = DISABLE;
	Adc1Handle.Init.DiscontinuousConvMode = DISABLE;
	Adc1Handle.Init.NbrOfDiscConversion   = 0;
	Adc1Handle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_RISING;
	Adc1Handle.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T2_TRGO;	//ADC_EXTERNALTRIGCONV_T1_CC1
	Adc1Handle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
	Adc1Handle.Init.NbrOfConversion       = 3;
	Adc1Handle.Init.DMAContinuousRequests = ENABLE;
	Adc1Handle.Init.EOCSelection          = DISABLE;
*/

    // -1- Initialize ADC peripheral -----------------------------------
    Adc1Handle.Instance = ADC1;
    Adc2Handle.Instance = ADC2;
    Adc3Handle.Instance = ADC3;

    // ADC de-initialization Error
    if(HAL_ADC_DeInit(&Adc1Handle) != HAL_OK)
    { 
        fcErrHandler("");
    }

    if(HAL_ADC_DeInit(&Adc2Handle) != HAL_OK)
    { 
        fcErrHandler("");
    }

    if(HAL_ADC_DeInit(&Adc3Handle) != HAL_OK)
    { 
        fcErrHandler("");
    }

    Adc1Handle.Init.ClockPrescaler           = ADC_CLOCK_ASYNC_DIV2;            // Asynchronous clock mode, input ADC clock divided by
    Adc1Handle.Init.Resolution               = ADC_RESOLUTION_16B;              // 16-bit resolution for converted data
    Adc1Handle.Init.ScanConvMode             = ENABLE;                          // Sequencer disabled (ADC conversion on only 1 channel: channel set on rank 1)
    Adc1Handle.Init.EOCSelection             = ADC_EOC_SINGLE_CONV;             // EOC flag picked-up to indicate conversion end
    Adc1Handle.Init.LowPowerAutoWait         = DISABLE;                         // Auto-delayed conversion feature disabled
    Adc1Handle.Init.ContinuousConvMode       = DISABLE;                         // Continuous mode enabled (automatic conversion restart after each conversion)
    Adc1Handle.Init.NbrOfConversion          = 16;                              // Parameter discarded because sequencer is disabled
    Adc1Handle.Init.DiscontinuousConvMode    = DISABLE;                         // Parameter discarded because sequencer is disabled
    Adc1Handle.Init.NbrOfDiscConversion      = 1;                               // Parameter discarded because sequencer is disabled
    Adc1Handle.Init.ExternalTrigConv         = ADC_EXTERNALTRIGCONVEDGE_RISING; // Software start to trig the 1st conversion manually, without external event
    Adc1Handle.Init.ExternalTrigConvEdge     = ADC_EXTERNALTRIG_T2_TRGO;        // Parameter discarded because software trigger chosen
    Adc1Handle.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR; // ADC DMA circular requested
    Adc1Handle.Init.LeftBitShift             = ADC_LEFTBITSHIFT_NONE;           // Left shift of final results
    Adc1Handle.Init.Overrun                  = ADC_OVR_DATA_OVERWRITTEN;        // DR register is overwritten with the last conversion result in case of overrun
    #if ADC_OVER_SAMPLING == 0
    Adc1Handle.Init.OversamplingMode         = DISABLE;                         // Oversampling disable
    #else
    Adc1Handle.Init.OversamplingMode         = ENABLE;                          // Oversampling enabled.
    Adc1Handle.Init.Oversampling.Ratio                 = 1023;
    Adc1Handle.Init.Oversampling.RightBitShift         = ADC_RIGHTBITSHIFT_NONE;
    Adc1Handle.Init.Oversampling.TriggeredMode         = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
    Adc1Handle.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
    #endif

    Adc2Handle.Init.ClockPrescaler           = ADC_CLOCK_ASYNC_DIV2;            // Asynchronous clock mode, input ADC clock divided by
    Adc2Handle.Init.Resolution               = ADC_RESOLUTION_16B;              // 16-bit resolution for converted data
    Adc2Handle.Init.ScanConvMode             = ENABLE;                          // Sequencer disabled (ADC conversion on only 1 channel: channel set on rank 1)
    Adc2Handle.Init.EOCSelection             = ADC_EOC_SINGLE_CONV;             // EOC flag picked-up to indicate conversion end
    Adc2Handle.Init.LowPowerAutoWait         = DISABLE;                         // Auto-delayed conversion feature disabled
    Adc2Handle.Init.ContinuousConvMode       = ENABLE;//DISABLE;                         // Continuous mode enabled (automatic conversion restart after each conversion)
    Adc2Handle.Init.NbrOfConversion          = 2;                               // Parameter discarded because sequencer is disabled
    Adc2Handle.Init.DiscontinuousConvMode    = DISABLE;                         // Parameter discarded because sequencer is disabled
    Adc2Handle.Init.NbrOfDiscConversion      = 1;                               // Parameter discarded because sequencer is disabled
    Adc2Handle.Init.ExternalTrigConv         = ADC_EXTERNALTRIGCONVEDGE_RISING; // Software start to trig the 1st conversion manually, without external event
    Adc2Handle.Init.ExternalTrigConvEdge     = ADC_EXTERNALTRIG_T2_TRGO;        // Parameter discarded because software trigger chosen
    Adc2Handle.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR; // ADC DMA circular requested
    Adc2Handle.Init.LeftBitShift             = ADC_LEFTBITSHIFT_NONE;           // Left shift of final results
    Adc2Handle.Init.Overrun                  = ADC_OVR_DATA_OVERWRITTEN;        // DR register is overwritten with the last conversion result in case of overrun
    #if ADC_OVER_SAMPLING == 0
    Adc2Handle.Init.OversamplingMode         = DISABLE;                         // Oversampling disable
    #else
    Adc2Handle.Init.OversamplingMode         = ENABLE;                          // Oversampling enabled.
    Adc2Handle.Init.Oversampling.Ratio                 = 1023;
    Adc2Handle.Init.Oversampling.RightBitShift         = ADC_RIGHTBITSHIFT_NONE;
    Adc2Handle.Init.Oversampling.TriggeredMode         = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
    Adc2Handle.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
    #endif

    Adc3Handle.Init.ClockPrescaler           = ADC_CLOCK_ASYNC_DIV2;           // Asynchronous clock mode, input ADC clock divided by
    Adc3Handle.Init.Resolution               = ADC_RESOLUTION_16B;              // 16-bit resolution for converted data
    Adc3Handle.Init.ScanConvMode             = ENABLE;                          // Sequencer disabled (ADC conversion on only 1 channel: channel set on rank 1)
    Adc3Handle.Init.EOCSelection             = ADC_EOC_SINGLE_CONV;             // EOC flag picked-up to indicate conversion end
    Adc3Handle.Init.LowPowerAutoWait         = DISABLE;                         // Auto-delayed conversion feature disabled
    Adc3Handle.Init.ContinuousConvMode       = DISABLE;                         // Continuous mode enabled (automatic conversion restart after each conversion)
    Adc3Handle.Init.NbrOfConversion          = 12;                               // Parameter discarded because sequencer is disabled
    Adc3Handle.Init.DiscontinuousConvMode    = DISABLE;                         // Parameter discarded because sequencer is disabled
    Adc3Handle.Init.NbrOfDiscConversion      = 1;                               // Parameter discarded because sequencer is disabled
    Adc3Handle.Init.ExternalTrigConv         = ADC_EXTERNALTRIGCONVEDGE_RISING; // Software start to trig the 1st conversion manually, without external event
    Adc3Handle.Init.ExternalTrigConvEdge     = ADC_EXTERNALTRIG_T2_TRGO;        // Parameter discarded because software trigger chosen
    Adc3Handle.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR; // ADC DMA circular requested
    Adc3Handle.Init.LeftBitShift             = ADC_LEFTBITSHIFT_NONE;           // Left shift of final results
    Adc3Handle.Init.Overrun                  = ADC_OVR_DATA_OVERWRITTEN;        // DR register is overwritten with the last conversion result in case of overrun
    #if ADC_OVER_SAMPLING == 0
    Adc3Handle.Init.OversamplingMode         = DISABLE;                         // Oversampling disable
    #else
    Adc3Handle.Init.OversamplingMode         = ENABLE;                          // Oversampling enabled.
    Adc3Handle.Init.Oversampling.Ratio                 = 1023;
    Adc3Handle.Init.Oversampling.RightBitShift         = ADC_RIGHTBITSHIFT_NONE;
    Adc3Handle.Init.Oversampling.TriggeredMode         = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
    Adc3Handle.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
    #endif

    // Initialize ADC peripheral according to the passed parameters
    if(HAL_ADC_Init(&Adc1Handle) != HAL_OK)
    {
        fcErrHandler("");
    }

    if(HAL_ADC_Init(&Adc2Handle) != HAL_OK)
    {
        fcErrHandler("");
    }

    if(HAL_ADC_Init(&Adc3Handle) != HAL_OK)
    {
        fcErrHandler("");
    }

    // -2- Start calibration -----------------------------------
    if(HAL_ADCEx_Calibration_Start(&Adc1Handle, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK)
    {
        fcErrHandler("");
    }

    if(HAL_ADCEx_Calibration_Start(&Adc2Handle, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK)
    {
        fcErrHandler("");
    }

    if(HAL_ADCEx_Calibration_Start(&Adc3Handle, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK)
    {
        fcErrHandler("");
    }

    // -3- Channel configuration -----------------------------------
    // PF.11 (ADC1_P2) - StrCur1
    AdChConfig.Channel                = ADC_CHANNEL_2;              // Sampled channel number. Ch0-Ch1...
    AdChConfig.Rank                   = ADC_REGULAR_RANK_1;         // Rank of sampled channel number ADCx_CHANNEL. Rank_1, Rand_2...
    AdChConfig.SamplingTime           = ADC_SAMPLETIME_810CYCLES_5;//ADC_SAMPLETIME_8CYCLES_5;   // Sampling time (number of clock cycles unit)
    AdChConfig.SingleDiff             = ADC_SINGLE_ENDED;           // Single input channel
    AdChConfig.OffsetNumber           = ADC_OFFSET_NONE;            // No offset subtraction
    AdChConfig.Offset                 = 0;                          // Parameter discarded because offset correction is disabled
    AdChConfig.OffsetRightShift       = DISABLE;                    // No Right Offset Shift
    AdChConfig.OffsetSignedSaturation = DISABLE;                    // No Signed Saturation

    if(HAL_ADC_ConfigChannel(&Adc1Handle, &AdChConfig) != HAL_OK)
    {
        fcErrHandler("");
    }

    // PA.6 (ADC12_P3) - StrCur2
    AdChConfig.Channel                = ADC_CHANNEL_3;
    AdChConfig.Rank                   = ADC_REGULAR_RANK_2;
    if(HAL_ADC_ConfigChannel(&Adc1Handle, &AdChConfig) != HAL_OK)
    {
        fcErrHandler("");
    }

    // PC.4 (ADC12_P4) - StrCur3
    AdChConfig.Channel                = ADC_CHANNEL_4;
    AdChConfig.Rank                   = ADC_REGULAR_RANK_3;
    if(HAL_ADC_ConfigChannel(&Adc1Handle, &AdChConfig) != HAL_OK)
    {
        fcErrHandler("");
    }

    // PB.1 (ADC12_P5) - StrCur4
    AdChConfig.Channel                = ADC_CHANNEL_5;
    AdChConfig.Rank                   = ADC_REGULAR_RANK_4;
    if(HAL_ADC_ConfigChannel(&Adc1Handle, &AdChConfig) != HAL_OK)
    {
        fcErrHandler("");
    }

    // PF.12 (ADC1_P6) - StrCur5
    AdChConfig.Channel                = ADC_CHANNEL_6;
    AdChConfig.Rank                   = ADC_REGULAR_RANK_5;
    if(HAL_ADC_ConfigChannel(&Adc1Handle, &AdChConfig) != HAL_OK)
    {
        fcErrHandler("");
    }

    // PA.7 (ADC12_P7) - StrCur6
    AdChConfig.Channel                = ADC_CHANNEL_7;
    AdChConfig.Rank                   = ADC_REGULAR_RANK_6;
    if(HAL_ADC_ConfigChannel(&Adc1Handle, &AdChConfig) != HAL_OK)
    {
        fcErrHandler("");
    }

    // PC.5 (ADC12_P8) - StrCur7
    AdChConfig.Channel                = ADC_CHANNEL_8;
    AdChConfig.Rank                   = ADC_REGULAR_RANK_7;
    if(HAL_ADC_ConfigChannel(&Adc1Handle, &AdChConfig) != HAL_OK)
    {
        fcErrHandler("");
    }

    // PB.0 (ADC12_P9) - StrCur8
    AdChConfig.Channel                = ADC_CHANNEL_9;
    AdChConfig.Rank                   = ADC_REGULAR_RANK_8;
    if(HAL_ADC_ConfigChannel(&Adc1Handle, &AdChConfig) != HAL_OK)
    {
        fcErrHandler("");
    }

    // PC.0 (ADC123_P10) - StrCur9
    AdChConfig.Channel                = ADC_CHANNEL_10;
    AdChConfig.Rank                   = ADC_REGULAR_RANK_9;
    if(HAL_ADC_ConfigChannel(&Adc1Handle, &AdChConfig) != HAL_OK)
    {
        fcErrHandler("");
    }

    // PC.1 (ADC123_P11) - StrCur10
    AdChConfig.Channel                = ADC_CHANNEL_11;
    AdChConfig.Rank                   = ADC_REGULAR_RANK_10;
    if(HAL_ADC_ConfigChannel(&Adc1Handle, &AdChConfig) != HAL_OK)
    {
        fcErrHandler("");
    }

    // PA.2 (ADC12_P14) - StrCur11
    AdChConfig.Channel                = ADC_CHANNEL_14;
    AdChConfig.Rank                   = ADC_REGULAR_RANK_11; 
    if(HAL_ADC_ConfigChannel(&Adc1Handle, &AdChConfig) != HAL_OK)
    {
        fcErrHandler("");
    }

    // PA.3 (ADC12_P15) - StrCur12
    AdChConfig.Channel                = ADC_CHANNEL_15;
    AdChConfig.Rank                   = ADC_REGULAR_RANK_12;
    if(HAL_ADC_ConfigChannel(&Adc1Handle, &AdChConfig) != HAL_OK)
    {
        fcErrHandler("");
    }

    // PA.0 (ADC1_P16) - StrCur13
    AdChConfig.Channel                = ADC_CHANNEL_16;
    AdChConfig.Rank                   = ADC_REGULAR_RANK_13;
    if(HAL_ADC_ConfigChannel(&Adc1Handle, &AdChConfig) != HAL_OK)
    {
        fcErrHandler("");
    }

    // PA.1 (ADC1_P17) - StrCur14
    AdChConfig.Channel                = ADC_CHANNEL_17;
    AdChConfig.Rank                   = ADC_REGULAR_RANK_14;
    if(HAL_ADC_ConfigChannel(&Adc1Handle, &AdChConfig) != HAL_OK)
    {
        fcErrHandler("");
    }

    // PA.4 (ADC12_P18) - StrCur15
    AdChConfig.Channel                = ADC_CHANNEL_18;
    AdChConfig.Rank                   = ADC_REGULAR_RANK_15;
    if(HAL_ADC_ConfigChannel(&Adc1Handle, &AdChConfig) != HAL_OK)
    {
        fcErrHandler("");
    }

    // PA.5 (ADC12_P19) - StrCur16
    AdChConfig.Channel                = ADC_CHANNEL_19;
    AdChConfig.Rank                   = ADC_REGULAR_RANK_16;
    if(HAL_ADC_ConfigChannel(&Adc1Handle, &AdChConfig) != HAL_OK)
    {
        fcErrHandler("");
    }
    
    // PC.2 (ADC3_P0) - StrCur17
    AdChConfig.Channel                = ADC_CHANNEL_0;              //  Sampled channel number. Ch0-Ch1...
    AdChConfig.Rank                   = ADC_REGULAR_RANK_1;         //  Rank of sampled channel number ADCx_CHANNEL. Rank_1, Rand_2...
    AdChConfig.SamplingTime           = ADC_SAMPLETIME_810CYCLES_5; //  ADC_SAMPLETIME_8CYCLES_5;   // Sampling time (number of clock cycles unit)
    AdChConfig.SingleDiff             = ADC_SINGLE_ENDED;           //  Single input channel
    AdChConfig.OffsetNumber           = ADC_OFFSET_NONE;            //  No offset subtraction
    AdChConfig.Offset                 = 0;                          //  Parameter discarded because offset correction is disabled
    AdChConfig.OffsetRightShift       = DISABLE;                    //  No Right Offset Shift
    AdChConfig.OffsetSignedSaturation = DISABLE;                    //  No Signed Saturation

    if(HAL_ADC_ConfigChannel(&Adc3Handle, &AdChConfig) != HAL_OK)
    {
        fcErrHandler("");
    }

    // PC.3 (ADC3_P1) - StrCur18
    AdChConfig.Channel                = ADC_CHANNEL_1;
    AdChConfig.Rank                   = ADC_REGULAR_RANK_2;
    if(HAL_ADC_ConfigChannel(&Adc3Handle, &AdChConfig) != HAL_OK)
    {
        fcErrHandler("");
    }

    // PF.9 (ADC3_P2) - StrCur19
    AdChConfig.Channel                = ADC_CHANNEL_2;
    AdChConfig.Rank                   = ADC_REGULAR_RANK_3;
    if(HAL_ADC_ConfigChannel(&Adc3Handle, &AdChConfig) != HAL_OK)
    {
        fcErrHandler("");
    }

    // PF.7 (ADC3_P3) - StrCur20
    AdChConfig.Channel                = ADC_CHANNEL_3;
    AdChConfig.Rank                   = ADC_REGULAR_RANK_4;
    if(HAL_ADC_ConfigChannel(&Adc3Handle, &AdChConfig) != HAL_OK)
    {
        fcErrHandler("");
    }

    // PF.5 (ADC3_P4) - StrCur21
    AdChConfig.Channel                = ADC_CHANNEL_4;
    AdChConfig.Rank                   = ADC_REGULAR_RANK_5;
    if(HAL_ADC_ConfigChannel(&Adc3Handle, &AdChConfig) != HAL_OK)
    {
        fcErrHandler("");
    }

    // PF.3 (ADC3_P5) - StrCur22
    AdChConfig.Channel                = ADC_CHANNEL_5;
    AdChConfig.Rank                   = ADC_REGULAR_RANK_6;
    if(HAL_ADC_ConfigChannel(&Adc3Handle, &AdChConfig) != HAL_OK)
    {
        fcErrHandler("");
    }

    // PF.10 (ADC3_P6) - StrCur23
    AdChConfig.Channel                = ADC_CHANNEL_6;
    AdChConfig.Rank                   = ADC_REGULAR_RANK_7;
    if(HAL_ADC_ConfigChannel(&Adc3Handle, &AdChConfig) != HAL_OK)
    {
        fcErrHandler("");
    }

    // PF.8 (ADC3_P7) - StrCur24
    AdChConfig.Channel                = ADC_CHANNEL_7;
    AdChConfig.Rank                   = ADC_REGULAR_RANK_8;
    if(HAL_ADC_ConfigChannel(&Adc3Handle, &AdChConfig) != HAL_OK)
    {
        fcErrHandler("");
    }

    // PF.6 (ADC3_P8) - LeakCurPV
    AdChConfig.Channel                = ADC_CHANNEL_8;
    AdChConfig.Rank                   = ADC_REGULAR_RANK_9;
    if(HAL_ADC_ConfigChannel(&Adc3Handle, &AdChConfig) != HAL_OK)
    {
        fcErrHandler("");
    }

    // PF.4 (ADC3_P9) - PT1000
    AdChConfig.Channel                = ADC_CHANNEL_9;
    AdChConfig.Rank                   = ADC_REGULAR_RANK_10;
    if(HAL_ADC_ConfigChannel(&Adc3Handle, &AdChConfig) != HAL_OK)
    {
        fcErrHandler("");
    }

    // Internal Channel - Vbat(ADC3_P18)
    AdChConfig.Channel                = ADC_CHANNEL_VBAT;
    AdChConfig.Rank                   = ADC_REGULAR_RANK_11; 
    if(HAL_ADC_ConfigChannel(&Adc3Handle, &AdChConfig) != HAL_OK)
    {
        fcErrHandler("");
    }

    // Internal Channel - Temp/4(ADC3_P17)
    AdChConfig.Channel                = ADC_CHANNEL_TEMPSENSOR;//ADC_CHANNEL_VBAT;
    AdChConfig.Rank                   = ADC_REGULAR_RANK_12;
    if(HAL_ADC_ConfigChannel(&Adc3Handle, &AdChConfig) != HAL_OK)
    {
        fcErrHandler("");
    }

    // PF.13 (ADC2_P2) - ArcCur
    AdChConfig.Channel                = ADC_CHANNEL_2;              // Sampled channel number. Ch0-Ch1...
    AdChConfig.Rank                   = ADC_REGULAR_RANK_1;         // Rank of sampled channel number ADCx_CHANNEL. Rank_1, Rand_2...
    AdChConfig.SamplingTime           = ADC_SAMPLETIME_8CYCLES_5;//ADC_SAMPLETIME_2CYCLES_5;//ADC_SAMPLETIME_8CYCLES_5;//ADC_SAMPLETIME_810CYCLES_5;//ADC_SAMPLETIME_8CYCLES_5;   // Sampling time (number of clock cycles unit)
    AdChConfig.SingleDiff             = ADC_SINGLE_ENDED;           // Single input channel
    AdChConfig.OffsetNumber           = ADC_OFFSET_NONE;            // No offset subtraction
    AdChConfig.Offset                 = 0;                          // Parameter discarded because offset correction is disabled
    AdChConfig.OffsetRightShift       = DISABLE;                    // No Right Offset Shift
    AdChConfig.OffsetSignedSaturation = DISABLE;                    // No Signed Saturation

    if(HAL_ADC_ConfigChannel(&Adc2Handle, &AdChConfig) != HAL_OK)
    {
        fcErrHandler("");
    }

    // PF.14 (ADC2_P6) - ArcLight
    AdChConfig.Channel                = ADC_CHANNEL_6;
    AdChConfig.Rank                   = ADC_REGULAR_RANK_2;
    if(HAL_ADC_ConfigChannel(&Adc2Handle, &AdChConfig) != HAL_OK)
    {
        fcErrHandler("");
    }

    // -4- Start conversion ----------------------------------- 
    //  Replace DMA.
    //  if (HAL_ADC_Start(&AdcHandle) != HAL_OK)
    //  {
    //    fcErrHandler("");
    //  }
 
    hAdc1Dma.Instance                 = DMA2_Stream4;
    hAdc1Dma.Init.Request             = DMA_REQUEST_ADC1;
    hAdc1Dma.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hAdc1Dma.Init.PeriphInc           = DMA_PINC_DISABLE;
    hAdc1Dma.Init.MemInc              = DMA_MINC_ENABLE;
    hAdc1Dma.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hAdc1Dma.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    hAdc1Dma.Init.Mode                = DMA_CIRCULAR;
    hAdc1Dma.Init.Priority            = DMA_PRIORITY_HIGH;
	hAdc1Dma.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
	hAdc1Dma.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
	hAdc1Dma.Init.MemBurst            = DMA_MBURST_INC16;//DMA_MBURST_SINGLE;//DMA_MBURST_INC16;
	hAdc1Dma.Init.PeriphBurst         = DMA_MBURST_INC16;//DMA_MBURST_SINGLE;//DMA_MBURST_INC16;

    hAdc2Dma.Instance                 = DMA2_Stream3;
    hAdc2Dma.Init.Request             = DMA_REQUEST_ADC2;
    hAdc2Dma.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hAdc2Dma.Init.PeriphInc           = DMA_PINC_DISABLE;
    hAdc2Dma.Init.MemInc              = DMA_MINC_ENABLE;
    hAdc2Dma.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hAdc2Dma.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    hAdc2Dma.Init.Mode                = DMA_CIRCULAR;
    hAdc2Dma.Init.Priority            = DMA_PRIORITY_HIGH;
	hAdc2Dma.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
	hAdc2Dma.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
	hAdc2Dma.Init.MemBurst            = DMA_MBURST_INC16;//DMA_MBURST_SINGLE;//DMA_MBURST_INC16;
	hAdc2Dma.Init.PeriphBurst         = DMA_MBURST_INC16;//DMA_MBURST_SINGLE;//DMA_MBURST_INC16;

    hAdc3Dma.Instance                 = DMA2_Stream0;
    hAdc3Dma.Init.Request             = DMA_REQUEST_ADC3;
    hAdc3Dma.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hAdc3Dma.Init.PeriphInc           = DMA_PINC_DISABLE;
    hAdc3Dma.Init.MemInc              = DMA_MINC_ENABLE;
    hAdc3Dma.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hAdc3Dma.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    hAdc3Dma.Init.Mode                = DMA_CIRCULAR;
    hAdc3Dma.Init.Priority            = DMA_PRIORITY_HIGH;
	hAdc3Dma.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
	hAdc3Dma.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
	hAdc3Dma.Init.MemBurst            = DMA_MBURST_INC16;//DMA_MBURST_SINGLE;//DMA_MBURST_INC16;
	hAdc3Dma.Init.PeriphBurst         = DMA_MBURST_INC16;//DMA_MBURST_SINGLE;//DMA_MBURST_INC16;
    
    // Deinitialize  & Initialize the DMA for new transfer.
    HAL_DMA_Init(&hAdc1Dma);
    HAL_DMA_Init(&hAdc2Dma);
    HAL_DMA_Init(&hAdc3Dma);
  
    // Associate the DMA handle.
    __HAL_LINKDMA(&Adc1Handle, DMA_Handle, hAdc1Dma);
    __HAL_LINKDMA(&Adc2Handle, DMA_Handle, hAdc2Dma);
    __HAL_LINKDMA(&Adc3Handle, DMA_Handle, hAdc3Dma);

    ((DMA_Stream_TypeDef *)hAdc1Dma.Instance)->PAR = ADC1_BASE + 0x40;
    ((DMA_Stream_TypeDef *)hAdc1Dma.Instance)->M0AR = (uint32_t)gsAdc1Fifo;
    ((DMA_Stream_TypeDef *)hAdc1Dma.Instance)->NDTR = ADC_FIFO_SIZE1;

    ((DMA_Stream_TypeDef *)hAdc2Dma.Instance)->PAR = ADC2_BASE + 0x40;
    ((DMA_Stream_TypeDef *)hAdc2Dma.Instance)->M0AR = (uint32_t)gsAdc2Fifo;
    ((DMA_Stream_TypeDef *)hAdc2Dma.Instance)->NDTR = ADC_FIFO_SIZE2;

    ((DMA_Stream_TypeDef *)hAdc3Dma.Instance)->PAR = ADC3_BASE + 0x40;
    ((DMA_Stream_TypeDef *)hAdc3Dma.Instance)->M0AR = (uint32_t)gsAdc3Fifo;
    ((DMA_Stream_TypeDef *)hAdc3Dma.Instance)->NDTR = ADC_FIFO_SIZE3;
}
//---------------------------------------------------------------------------

// APB1 Clock (100 MHz)
// Timer Clock = APB1 x 2 (200 MHz)
// 7680 Hz --> 26,042 = 200MHz/7680, 7680: 128x60Hz.

void fcInitTim2(void)
{
    TIM_MasterConfigTypeDef master_timer_config;
    __HAL_RCC_TIM2_CLK_ENABLE();

    Tim2Handle.Instance = TIM2;	//sClock_Timer설정.xlsx 참조.
    Tim2Handle.Init.Period = 26042 - 1;
    Tim2Handle.Init.Prescaler = 0;
    Tim2Handle.Init.ClockDivision = 0;
    Tim2Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    Tim2Handle.Init.RepetitionCounter = 1;

    if(HAL_TIM_Base_Init(&Tim2Handle) != HAL_OK)
    {
        fcErrHandler("HAL_TIM_Base_Init_T2");
    }

    // Timer TRGO selection.
    master_timer_config.MasterOutputTrigger = TIM_TRGO_UPDATE;
    master_timer_config.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    master_timer_config.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

    if(HAL_TIMEx_MasterConfigSynchronization(&Tim2Handle, &master_timer_config) != HAL_OK) 
    {
        fcErrHandler("HAL_TIMEx_MasterConfigSynchronization");
    }
}
//---------------------------------------------------------

/* Initialize TIMx peripheral as follows:
       + Prescaler = (SystemCoreClock / (2*20000000)) - 1
       + Period = (1000 - 1)
       + ClockDivision = 0
       + Counter direction = Up
  */
#define  PERIOD_VALUE       (uint32_t)(1000 - 1)        /* Period Value  */
#define  PULSE1_VALUE       (uint32_t)(PERIOD_VALUE/10)  /* Capture Compare 1 Value  */

uint32_t uhPrescalerValue = 0;

void fcInitTim1(void)
{
    GPIO_InitTypeDef  GPIO_InitStruct;
    /* Timer Output Compare Configuration Structure declaration */
    TIM_OC_InitTypeDef sConfig;

    /* Compute the prescaler value to have TIM1 counter clock equal to 20000000 Hz */
    //uhPrescalerValue = (uint32_t)(SystemCoreClock / (1 * 2000000)) - 1;
    uhPrescalerValue = (uint32_t)(SystemCoreClock / (1 * 2000000)) - 1;

    __HAL_RCC_TIM1_CLK_ENABLE();

    GPIO_InitStruct.Pin       = GPIO_PIN_9;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    Tim1Handle.Instance                 = TIM1;
    Tim1Handle.Init.Period              = PERIOD_VALUE;
    Tim1Handle.Init.Prescaler           = uhPrescalerValue;
    Tim1Handle.Init.ClockDivision       = 0;
    Tim1Handle.Init.CounterMode         = TIM_COUNTERMODE_UP;
    Tim1Handle.Init.RepetitionCounter   = 0;
    if(HAL_TIM_IC_Init(&Tim1Handle) != HAL_OK)
    {
        fcErrHandler("HAL_TIM_IC_Init");
    }

    /* Common configuration for all channels */
    sConfig.OCMode       = TIM_OCMODE_PWM1;
    sConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;
    sConfig.OCFastMode   = TIM_OCFAST_DISABLE;
    sConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    sConfig.OCIdleState  = TIM_OCIDLESTATE_RESET;

    /* Set the pulse value for channel 1 */
    sConfig.Pulse = PULSE1_VALUE;

    HAL_TIM_PWM_ConfigChannel(&Tim1Handle, &sConfig, TIM_CHANNEL_2);
}
////---------------------------------------------------------

/* Timing register value is computed with the STM32CubeMX Tool,
  Fast Mode @400kHz with I2CCLK = 216 MHz,
  rise time = 100ns, fall time = 20ns
  Timing Value = (uint32_t)0x00A01E5D */

/* I2C TIMING Register define when I2C clock source is APB1 (SYSCLK/4) */
/* I2C TIMING is calculated in case of the I2C Clock source is the APB1CLK = 100 MHz */
/* This example use TIMING to 0x00901954 to reach 400 kHz speed (Rise time = 100 ns, Fall time = 10 ns) */
#define I2C_TIMING      0x10905354

static void fcInitI2C(void)
{
    GPIO_InitTypeDef  GPIO_InitStruct;
    //  UN_I2C_TIMING unI2cTiming;

    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_I2C2_CLK_ENABLE();

    GPIO_InitStruct.Pin       = GPIO_PIN_0 | GPIO_PIN_1;	//SDA, SCL.
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    //  unI2cTiming.stTiming.PRESC  = 0;
    //  unI2cTiming.stTiming.SCLDEL = 9;
    //  unI2cTiming.stTiming.SDADEL = 0;
    //  unI2cTiming.stTiming.SCLH   = 0x19;
    //  unI2cTiming.stTiming.SCLL   = 0x54;

    I2c2Handle.Instance             = I2C2;
    //  I2c2Handle.Init.Timing          = unI2cTiming.lData;
    I2c2Handle.Init.Timing          = I2C_TIMING;
    I2c2Handle.Init.OwnAddress1     = 0x78;	//MCCOG43005 Address.
    I2c2Handle.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    I2c2Handle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    I2c2Handle.Init.OwnAddress2     = 0xFF;
    I2c2Handle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    I2c2Handle.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;

    //  I2c2Handle.Init.OwnAddress1     = 0xA6;	//RF EEPROM(M24LR64E) Address.
	
    if(HAL_I2C_Init(&I2c2Handle) != HAL_OK)
    {
        fcErrHandler("HAL_I2C2_Init");
    }
  
    HAL_I2CEx_ConfigAnalogFilter(&I2c2Handle, I2C_ANALOGFILTER_ENABLE);
}
//---------------------------------------------------------------------------

static void fcInitRtc(void)
{
    RCC_OscInitTypeDef        RCC_OscInitStruct;
    RCC_PeriphCLKInitTypeDef  PeriphClkInitStruct;
	
    // -1- Enables access to the backup domain -----------
    // To enable access on RTC registers.
    HAL_PWR_EnableBkUpAccess();
  
    // -2- Configure LSE/LSI as RTC clock source -----------
    RCC_OscInitStruct.OscillatorType =  RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    RCC_OscInitStruct.LSIState = RCC_LSI_OFF;
    if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        fcErrHandler("HAL_RCC_OscConfig");
    }

    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
    if(HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
        fcErrHandler("HAL_RCCEx_PeriphCLKConfig");
    }

    // -3- Enable RTC peripheral Clocks -----------
    // Enable RTC Clock.
    __HAL_RCC_RTC_ENABLE(); 

    // -1- Configure the RTC peripheral -----------
    /* RTC configured as follow:
      - Hour Format    = Format 12
      - Asynch Prediv  = Value according to source clock
      - Synch Prediv   = Value according to source clock
      - OutPut         = Output Disable
      - OutPutPolarity = High Polarity
      - OutPutType     = Open Drain */
    __HAL_RTC_RESET_HANDLE_STATE(&RtcHandle);
    
    RtcHandle.Instance            = RTC;
    RtcHandle.Init.HourFormat     = RTC_HOURFORMAT_24;
    RtcHandle.Init.AsynchPrediv   = RTC_ASYNCH_PREDIV;
    RtcHandle.Init.SynchPrediv    = RTC_SYNCH_PREDIV;
    RtcHandle.Init.OutPut         = RTC_OUTPUT_DISABLE;
    RtcHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    RtcHandle.Init.OutPutType     = RTC_OUTPUT_TYPE_OPENDRAIN;
  
    if(HAL_RTC_Init(&RtcHandle) != HAL_OK)
    {
        fcErrHandler("HAL_RTC_Init");
    }

    // -2- Check RTC backup memory & Set or Get RTC Information -----------
    //  Backup memory disturb.
    if (HAL_RTCEx_BKUPRead(&RtcHandle, RTC_BKP_DR0) != 0x32F2)
    { 
        LED2(LED_ON);
        fcResetTime();
        
        HAL_RTCEx_BKUPWrite(&RtcHandle, RTC_BKP_DR0, 0x32F2);
    }
    else
    {
        LED2(LED_OFF);
        fcNowRtc(1);
    }
}
//---------------------------------------------------------------------------

static void fcInitSPI(void)
{
	GPIO_InitTypeDef  GPIO_InitStruct;

    /* Enable GPIO TX/RX clock */
    __HAL_RCC_GPIOG_CLK_ENABLE();
    /* Enable SPI clock */
	__HAL_RCC_SPI6_CLK_ENABLE();
	__HAL_RCC_DMA2_CLK_ENABLE();

	GPIO_InitStruct.Pin       = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14;
	GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull      = GPIO_PULLUP;
	GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI6;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	//SPI6 -- Max: PCLK/2(24 MHz)
	Spi6Handle.Instance               = SPI6;	//1MHz(MAX78615)
	Spi6Handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;	//48MHz/64 = 750kHz.
	Spi6Handle.Init.Direction         = SPI_DIRECTION_2LINES;
	Spi6Handle.Init.CLKPhase          = SPI_PHASE_2EDGE;
	Spi6Handle.Init.CLKPolarity       = SPI_POLARITY_HIGH;	//MODE 3.
	Spi6Handle.Init.DataSize          = SPI_DATASIZE_8BIT;
	Spi6Handle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
	Spi6Handle.Init.TIMode            = SPI_TIMODE_DISABLE;
	Spi6Handle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
	Spi6Handle.Init.CRCPolynomial     = 7;    
    Spi6Handle.Init.CRCLength         = SPI_CRC_LENGTH_8BIT;
    Spi6Handle.Init.NSS               = SPI_NSS_SOFT;
    Spi6Handle.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;
    Spi6Handle.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;  /* Recommended setting to avoid glitches */
	Spi6Handle.Init.Mode              = SPI_MODE_MASTER;
	
    HAL_SPI_Init(&Spi6Handle);

#if 0
    /* Configure the DMA handler for Transmission process */
    hSpi6DmaTx.Instance                 = DMA2_Stream5;
    hSpi6DmaTx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    hSpi6DmaTx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hSpi6DmaTx.Init.MemBurst            = DMA_MBURST_SINGLE;
    hSpi6DmaTx.Init.PeriphBurst         = DMA_PBURST_SINGLE;
    hSpi6DmaTx.Init.Request             = BDMA_REQUEST_SPI6_TX;
    hSpi6DmaTx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hSpi6DmaTx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hSpi6DmaTx.Init.MemInc              = DMA_MINC_ENABLE;
    hSpi6DmaTx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hSpi6DmaTx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hSpi6DmaTx.Init.Mode                = DMA_NORMAL;
    hSpi6DmaTx.Init.Priority            = DMA_PRIORITY_LOW;

    HAL_DMA_Init(&hSpi6DmaTx);

    /* Associate the initialized DMA handle to the the SPI handle */
    __HAL_LINKDMA(&Spi6Handle, hdmatx, hSpi6DmaTx);
	
    /* Configure the DMA handler for Transmission process */
    hSpi6DmaRx.Instance                 = DMA2_Stream6;
    hSpi6DmaRx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    hSpi6DmaRx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hSpi6DmaRx.Init.MemBurst            = DMA_MBURST_SINGLE;
    hSpi6DmaRx.Init.PeriphBurst         = DMA_PBURST_SINGLE;
    hSpi6DmaRx.Init.Request             = BDMA_REQUEST_SPI6_RX;
    hSpi6DmaRx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hSpi6DmaRx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hSpi6DmaRx.Init.MemInc              = DMA_MINC_ENABLE;
    hSpi6DmaRx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hSpi6DmaRx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hSpi6DmaRx.Init.Mode                = DMA_NORMAL;
    hSpi6DmaRx.Init.Priority            = DMA_PRIORITY_HIGH;

    HAL_DMA_Init(&hSpi6DmaRx);

    /* Associate the initialized DMA handle to the the SPI handle */
    __HAL_LINKDMA(&Spi6Handle, hdmarx, hSpi6DmaRx);
#endif
}
//---------------------------------------------------------

static void fcInitUART(void)
{
    GPIO_InitTypeDef  GPIO_InitStruct;
    RCC_PeriphCLKInitTypeDef RCC_PeriphClkInit;

    __USART1_CLK_ENABLE();
    __USART3_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    // Select SysClk as source of USART1 clocks
    RCC_PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
    RCC_PeriphClkInit.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
    HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphClkInit);
  
    // Select SysClk as source of USART1 clocks
    RCC_PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART3;
    RCC_PeriphClkInit.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
    HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphClkInit);

    // UART1 TX, RX GPIO pin configuration.
    GPIO_InitStruct.Pin       = GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_USART1; //Datasheet 88page, PA.11 (Tim1).
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // UART3 TX, RX GPIO pin configuration.
    GPIO_InitStruct.Pin       = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3; //Datasheet 88page, PA.11 (Tim1).
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    Uart1Handle.Instance        = USART1;
    Uart1Handle.Init.BaudRate   = 38400;
    Uart1Handle.Init.WordLength = UART_WORDLENGTH_8B;
    Uart1Handle.Init.StopBits   = UART_STOPBITS_1;
    Uart1Handle.Init.Parity     = UART_PARITY_NONE;
    Uart1Handle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    Uart1Handle.Init.OverSampling = UART_OVERSAMPLING_16;
    Uart1Handle.Init.Mode       = UART_MODE_TX_RX;
    Uart1Handle.pRxBuffPtr      = gbU1RxFifo;

    if(HAL_UART_Init(&Uart1Handle) != HAL_OK)
    {
        fcErrHandler("HAL_UART1_Init");
    }

    Uart3Handle.Instance        = USART3;
    Uart3Handle.Init.BaudRate   = 38400;//19200;
    Uart3Handle.Init.WordLength = UART_WORDLENGTH_8B;
    Uart3Handle.Init.StopBits   = UART_STOPBITS_1;
    Uart3Handle.Init.Parity     = UART_PARITY_NONE;
    Uart3Handle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    Uart3Handle.Init.OverSampling = UART_OVERSAMPLING_16;
    Uart3Handle.Init.Mode       = UART_MODE_TX_RX;
    Uart3Handle.pRxBuffPtr      = gbU3RxFifo;

    if(HAL_UART_Init(&Uart3Handle) != HAL_OK)
    {
        fcErrHandler("HAL_UART3_Init");
    }

    hUart1DmaTx.Instance                 = DMA2_Stream7;
    ((DMA_Stream_TypeDef *)hUart1DmaTx.Instance)->PAR = (uint32_t)&Uart1Handle.Instance->TDR;
    hUart1DmaTx.Init.Request             = DMA_REQUEST_USART1_TX;
    hUart1DmaTx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hUart1DmaTx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hUart1DmaTx.Init.MemInc              = DMA_MINC_ENABLE;
    hUart1DmaTx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hUart1DmaTx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hUart1DmaTx.Init.Mode                = DMA_NORMAL;
    hUart1DmaTx.Init.Priority            = DMA_PRIORITY_LOW;
    hUart1DmaTx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    hUart1DmaTx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hUart1DmaTx.Init.MemBurst            = DMA_MBURST_INC4;
    hUart1DmaTx.Init.PeriphBurst         = DMA_PBURST_INC4;
    hUart1DmaTx.Init.Request             = DMA_REQUEST_USART1_TX;

    HAL_DMA_Init(&hUart1DmaTx);

    // Associate the initialized DMA handle to the UART handle.
    __HAL_LINKDMA(&Uart1Handle, hdmatx, hUart1DmaTx);
    //Uart1Handle->hdmatx = &hUart1DmaTx;
	
    hUart3DmaTx.Instance                 = DMA1_Stream3;
    ((DMA_Stream_TypeDef *)hUart3DmaTx.Instance)->PAR = (uint32_t)&Uart3Handle.Instance->TDR;
    hUart3DmaTx.Init.Request             = DMA_REQUEST_USART3_TX;
    hUart3DmaTx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hUart3DmaTx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hUart3DmaTx.Init.MemInc              = DMA_MINC_ENABLE;
    hUart3DmaTx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hUart3DmaTx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hUart3DmaTx.Init.Mode                = DMA_NORMAL;
    hUart3DmaTx.Init.Priority            = DMA_PRIORITY_LOW;
    hUart3DmaTx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    hUart3DmaTx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hUart3DmaTx.Init.MemBurst            = DMA_MBURST_INC4;
    hUart3DmaTx.Init.PeriphBurst         = DMA_PBURST_INC4;

    HAL_DMA_Init(&hUart3DmaTx);

    // Associate the initialized DMA handle to the UART handle.
    __HAL_LINKDMA(&Uart3Handle, hdmatx, hUart3DmaTx);
    //Uart1Handle->hdmatx = &hUart1DmaTx;

    hUart1DmaRx.Instance                 = DMA2_Stream5;
    ((DMA_Stream_TypeDef *)hUart1DmaRx.Instance)->PAR = (uint32_t)&Uart1Handle.Instance->RDR;
    hUart1DmaRx.Init.Request             = DMA_REQUEST_USART1_RX;
    hUart1DmaRx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hUart1DmaRx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hUart1DmaRx.Init.MemInc              = DMA_MINC_ENABLE;
    hUart1DmaRx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hUart1DmaRx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hUart1DmaRx.Init.Mode                = DMA_CIRCULAR;
    hUart1DmaRx.Init.Priority            = DMA_PRIORITY_HIGH;
    hUart1DmaRx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    hUart1DmaRx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hUart1DmaRx.Init.MemBurst            = DMA_MBURST_INC4;
    hUart1DmaRx.Init.PeriphBurst         = DMA_PBURST_INC4;
    hUart1DmaRx.Init.Request             = DMA_REQUEST_USART1_RX;

    HAL_DMA_Init(&hUart1DmaRx);

    // Associate the initialized DMA handle to the the UART handle.
    __HAL_LINKDMA(&Uart1Handle, hdmarx, hUart1DmaRx);

    hUart3DmaRx.Instance                 = DMA1_Stream1;
    ((DMA_Stream_TypeDef *)hUart3DmaRx.Instance)->PAR = (uint32_t)&Uart3Handle.Instance->RDR;
    hUart3DmaRx.Init.Request             = DMA_REQUEST_USART3_RX;
    hUart3DmaRx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hUart3DmaRx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hUart3DmaRx.Init.MemInc              = DMA_MINC_ENABLE;
    hUart3DmaRx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hUart3DmaRx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hUart3DmaRx.Init.Mode                = DMA_CIRCULAR;
    hUart3DmaRx.Init.Priority            = DMA_PRIORITY_HIGH;
    hUart3DmaRx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    hUart3DmaRx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hUart3DmaRx.Init.MemBurst            = DMA_MBURST_INC4;
    hUart3DmaRx.Init.PeriphBurst         = DMA_PBURST_INC4;

    HAL_DMA_Init(&hUart3DmaRx);

    // Associate the initialized DMA handle to the the UART handle.
    __HAL_LINKDMA(&Uart3Handle, hdmarx, hUart3DmaRx);

    Uart1Handle.Instance->CR3 |= USART_CR3_DMAR;
    Uart1Handle.Instance->CR3 |= USART_CR3_DMAT;

    Uart3Handle.Instance->CR3 |= USART_CR3_DMAR;
    Uart3Handle.Instance->CR3 |= USART_CR3_DMAT;

    HAL_DMA_Start(&hUart1DmaRx, (uint32_t)(&Uart1Handle.Instance->RDR), (uint32_t)gbU1RxFifo, UART_FIFO_SIZE);
    HAL_DMA_Start(&hUart3DmaRx, (uint32_t)(&Uart3Handle.Instance->RDR), (uint32_t)gbU3RxFifo, UART_FIFO_SIZE);

    gbU1RxNdtrOld = ((DMA_Stream_TypeDef *)hUart1DmaRx.Instance)->NDTR;
    gbU3RxNdtrOld = ((DMA_Stream_TypeDef *)hUart3DmaRx.Instance)->NDTR;
}
//---------------------------------------------------------
#if 0
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
#endif
////---------------------------------------------------------

void fcErrHandler(char *sStr)
{
    NVIC_SystemReset();
  
    while(1)
    {
    }
}
//---------------------------------------------------------------------------
