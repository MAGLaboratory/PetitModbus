//==============================================================================
// PetitModbusPort.h
// This is a file to support porting PetitModbus
//==============================================================================
#ifndef __PETIT_MODBUS_PORT_H__
#define __PETIT_MODBUS_PORT_H__

/**************************Petit Modbus Contents*******************************/
#ifndef FALSE
#define FALSE (0)
#endif
#ifndef TRUE
#define TRUE (1)
#endif

#define PETIT_CRC_TABULAR  (0x4040)
#define PETIT_CRC_BITWISE  (0x2020)
#define PETIT_CRC_EXTERNAL (0x9090)

#define PETIT_REG_INTERNAL (0x1333)
#define PETIT_REG_BOTH     (0x1222)
#define PETIT_REG_EXTERNAL (0x2222)

// file with user-defined constants
#include "PetitModbusUserPort.h"

// functions defined by petit modbus
extern void PetitRxBufferReset(void);
extern pu8_t PetitRxBufferInsert(pu8_t rcvd);
extern pu8_t PetitTxBufferPop(pu8_t* tx);
/******************************User Content************************************/

// data defined for porting
#if defined(PETIT_REG) && \
	(PETIT_REG == PETIT_REG_INTERNAL || PETIT_REG == PETIT_REG_BOTH)
extern pu16_t PetitRegisters[NUMBER_OF_PETITREGISTERS];
#endif
extern pu8_t PetitRegChange;

#if defined(PETIT_INPUT_REG) && \
	(PETIT_INPUT_REG == PETIT_REG_INTERNAL ||\
			PETIT_INPUT_REG == PETIT_REG_BOTH)
extern pu16_t PetitInputRegisters[NUMBER_OF_INPUT_PETITREGISTERS];
#endif

// functions to be defined for porting
extern void PetitPortTxBegin(pu8_t tx);
extern void PetitPortTimerStart(void);
extern void PetitPortTimerStop(void);
extern void PetitPortDirTx(void);
extern void PetitPortDirRx(void);
#if defined(PETIT_CRC) && PETIT_CRC == PETIT_CRC_EXTERNAL
extern void PetitPortCRC16Calc(pu8_t Data, pu16_t* CRC);
#endif

#if defined(PETIT_REG) && \
	(PETIT_REG == PETIT_REG_EXTERNAL || PETIT_REG == PETIT_REG_BOTH)
extern pu8_t PetitPortRegRead(pu8_t Addr, pu16_t* Data);
extern pu8_t PetitPortRegWrite(pu8_t Addr, pu16_t Data);
#endif
#if defined(PETIT_INPUT_REG) && \
	(PETIT_INPUT_REG == PETIT_REG_EXTERNAL || \
			PETIT_INPUT_REG == PETIT_REG_BOTH)
extern pu8_t PetitPortInputRegRead(pu8_t Addr, pu16_t* Data);
#endif
#endif /* __PETIT_MODBUS_PORT_H__ */
