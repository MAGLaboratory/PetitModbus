//=========================================================
// PetitModbusPort.h
// This is a file to support porting PetitModbus
//=========================================================
#ifndef __PETIT_MODBUS_PORT_H__
#define __PETIT_MODBUS_PORT_H__

#include "PetitModbusUserPort.h"

#ifndef FALSE
#define FALSE (0)
#endif
#ifndef TRUE
#define TRUE (1)
#endif

typedef short PetitRegStructure;
extern PetitRegStructure    PetitRegisters[NUMBER_OF_OUTPUT_PETITREGISTERS];
extern unsigned char PetitRegChange;

// functions to be defined for porting
extern void PetitPortTxBegin(unsigned char tx);
extern void PetitPortTimerStart(void);
extern void PetitPortTimerStop(void);
extern void PetitPortDirTx(void);
extern void PetitPortDirRx(void);

// functions defined by petit modbus
extern void PetitRxBufferReset(void);
extern unsigned char PetitRxBufferInsert(unsigned char rcvd);
extern unsigned char PetitTxBufferPop(unsigned char* tx);

#endif
