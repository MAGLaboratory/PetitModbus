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

#define PETIT_INPUT_REG_INTERNAL (0x2333)
#define PETIT_INPUT_REG_BOTH     (0x2222)
#define PETIT_INPUT_REG_EXTERNAL (0x3222)


// file with user-defined constants
#include "PetitModbusUserPort.h"

// functions defined by petit modbus
extern void PetitRxBufferReset(void);
extern unsigned char PetitRxBufferInsert(unsigned char rcvd);
extern unsigned char PetitTxBufferPop(unsigned char* tx);
/******************************User Content************************************/

// data defined for porting
typedef short PetitRegStructure;
#if defined(PETIT_REG) && \
	(PETIT_REG == PETIT_REG_INTERNAL || PETIT_REG == PETIT_REG_BOTH)
extern PetitRegStructure    PetitRegisters[NUMBER_OF_PETITREGISTERS];
#endif
extern unsigned char PetitRegChange;

// functions to be defined for porting
extern void PetitPortTxBegin(unsigned char tx);
extern void PetitPortTimerStart(void);
extern void PetitPortTimerStop(void);
extern void PetitPortDirTx(void);
extern void PetitPortDirRx(void);
#if defined(PETIT_CRC) && PETIT_CRC == PETIT_CRC_EXTERNAL
extern void PetitPortCRC16Calc(unsigned char Data, unsigned short* CRC);
#endif

#if defined(PETIT_REG) && \
	(PETIT_REG == PETIT_REG_EXTERNAL || PETIT_REG == PETIT_REG_BOTH)
extern unsigned char PetitPortRegRead(unsigned char Addr, unsigned short* Data);
extern unsigned char PetitPortRegWrite(unsigned char Addr, unsigned short Data);
#endif
#endif /* __PETIT_MODBUS_PORT_H__ */
