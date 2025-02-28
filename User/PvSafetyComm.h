#ifndef __PVSFT_COMM_H
#define __PVSFT_COMM_H

#include "PvSafetyPreDef.h"

#define DEVICE_ADDR		0x01
#define MODBUS_MIN_PACKET		8
#define MODBUS_MIN_PACKET_ETH		12

#define READ_COIL_STATUS          1
#define READ_INPUT_STATUS         2
#define READ_HOLDING_REGISTERS    3
#define READ_INPUT_REGISTERS      4
#define FORCE_SINGLE_COIL         5
#define WRITE_SINGLE_REGISTER     6
#define FORCE_MULTIPLE_COILS      15
#define WRITE_MULTIPLE_REGISTERS  16

#define ILLEGAL_FUNCTION      0x01
#define ILLEGAL_DATA_ADDRESS  0x02
#define ILLEGAL_DATA_VALUE    0x03
#define SLAVE_DEVICE_FAILURE  0x04
#define ACKNOWLEDGE           0x05
#define SLAVE_DEVICE_BUSY     0x06
#define NOT_EXIST_EVENT       0x10
#define SBO_TIME_OUT          0x11
#define ILLEAGL_ADU_LENGTH    0x12
#define MAIN_NAK              0x14

#define INPUT_END_ADDR	256

#define FSC_BACKLIGHT_ON    5
#define FSC_CLEAR_ARC_CNT   6

#define FSC_INIT_SYSEV  100
#define FSC_INIT_TRGEV  101
#define FSC_SBO  110

#define RHR_PMU_CFG   0
#define RHR_COM_CFG   1150
#define RHR_LAN_MAC   1200
#define RHR_PHA_FACT  1500
#define RHR_CAL_INFO  2000
#define RHR_RTC_INFO  6000

#define PSR_ADDR_CTRL	123

#define PMR_ADDR_PMU_CFG  RHR_PMU_CFG
#define PMR_ADDR_COM_CFG  RHR_COM_CFG
#define PMR_ADDR_LAC_MAC  RHR_LAN_MAC
#define PMR_ADDR_RTC_SET  RHR_RTC_INFO

#define RHR_PMU_CFG_RCNT   1107
#define RHR_COM_CFG_RCNT   9
#define RHR_LAN_MAC_RCNT   3
#define RHR_PHA_FACT_RCNT  65
#define RHR_CAL_INFO_RCNT  33
#define RHR_RTC_INFO_RCNT  3

#define PMR_REBOOT 100

void fcEtherModbus(uint8_t *RxBuf, uint8_t *TxBuf, uint16_t *TxCnt);
uint8_t fcReadInputReg(uint8_t *RxBuf, uint8_t *TxBuf, uint16_t *bTxCnt);
uint8_t fcReadHoldingReg(uint8_t *RxBuf, uint8_t *TxBuf, uint16_t *bTxCnt);
uint8_t fcWriteMultiReg(uint8_t *RxBuf, uint8_t *TxBuf, uint16_t *bTxCnt);
uint8_t fcWriteSingleReg(uint8_t *RxBuf, uint8_t *TxBuf, uint16_t *bTxCnt);
uint8_t fcSingleCoilReg(uint8_t *RxBuf, uint8_t *TxBuf, uint16_t *bTxCnt);
void fcBroadCast1(uint8_t *RxBuf);
void fcBroadCast3(uint8_t *RxBuf);
uint16_t fcCrc16Mod(uint8_t *data, uint16_t datacnt);

#endif
