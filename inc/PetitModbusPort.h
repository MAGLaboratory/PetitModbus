/******************************************************************************
 * @file PetitModbusPort.h
 * This is a file to support porting PetitModbus.
*******************************************************************************/
#ifndef __PETIT_MODBUS_PORT_H__
#define __PETIT_MODBUS_PORT_H__

/**************************Petit Modbus Contents*******************************/
#define PETIT_CRC_TABULAR   (0x4040)
#define PETIT_CRC_BITWISE   (0x2020)
#define PETIT_CRC_EXTERNAL  (0x9090)

#define PETIT_INTERNAL  (0x1333)
#define PETIT_BOTH      (0x1222)
#define PETIT_EXTERNAL  (0x2222)

#define PETIT_USER_LED_NONE (0x7777)
#define PETIT_USER_LED_FN   (0x5555)
#define PETIT_USER_LED_DEF  (0x3333)

// file with user-defined constants
#include "PetitModbusUserPort.h"

// functions defined by petit modbus
extern void PetitRxBufferReset(void);
extern pb_t PetitRxBufferInsert(pu8_t rcvd);
extern pb_t PetitTxBufferPop(pu8_t* tx);
/******************************User Content************************************/

// data defined for porting
#if defined(PETIT_COIL) && \
	(PETIT_COIL == PETIT_INTERNAL || PETIT_COIL == PETIT_BOTH)
	pu8_t PetitCoils[(NUMBER_OF_PETITCOILS + 7) >> 3];
#endif
#if defined(PETIT_REG) && \
	(PETIT_REG == PETIT_INTERNAL || PETIT_REG == PETIT_BOTH)
extern pu16_t PetitRegisters[NUMBER_OF_PETITREGISTERS];
#endif
extern pu8_t PetitRegChange;

#if defined(PETIT_INPUT_REG) && \
	(PETIT_INPUT_REG == PETIT_INTERNAL ||\
			PETIT_INPUT_REG == PETIT_BOTH)
extern pu16_t PetitInputRegisters[NUMBER_OF_INPUT_PETITREGISTERS];
#endif

// functions to be defined for porting
extern void PetitPortTxBegin(pu8_t tx);
extern void PetitPortTimerStart(void);
extern void PetitPortTimerStop(void);
#if defined(PETIT_CRC) && PETIT_CRC == PETIT_CRC_EXTERNAL
extern void PetitPortCRC16Calc(pu8_t Data, pu16_t* CRC);
#endif

#if defined(PETIT_COIL) && \
	(PETIT_COIL == PETIT_EXTERNAL || PETIT_REG == PETIT_BOTH)
extern pb_t PetitPortCoilRead(pu16_t Addr, pu8_t* Data);
extern pb_t PetitPortCoilWrite(pu16_t Addr, pu16_t Data);
#endif

#if defined(PETIT_REG) && \
	(PETIT_REG == PETIT_EXTERNAL || PETIT_REG == PETIT_BOTH)
extern pb_t PetitPortRegRead(pu8_t Addr, pu16_t* Data);
extern pb_t PetitPortRegWrite(pu8_t Addr, pu16_t Data);
#endif
#if defined(PETIT_INPUT_REG) && \
	(PETIT_INPUT_REG == PETIT_EXTERNAL || \
			PETIT_INPUT_REG == PETIT_BOTH)
extern pb_t PetitPortInputRegRead(pu8_t Addr, pu16_t* Data);
#endif
#if !defined(PETIT_USER_LED) || PETIT_USER_LED == PETIT_USER_LED_NONE
#define PetitLedSuc()
#define PetitLedErrFail()
#define PetitLedCrcFail()
#define PetitLedOff()
#elif defined(PETIT_USER_LED) && PETIT_USER_LED == PETIT_USER_LED_FN
extern void PetitLedSuc(void);
extern void PetitLedErrFail(void);
extern void PetitLedCrcFail(void);
extern void PetitLedOff(void);
#endif
#endif /* __PETIT_MODBUS_PORT_H__ */
