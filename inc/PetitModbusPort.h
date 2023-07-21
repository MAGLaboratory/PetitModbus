//=========================================================
// PetitModbusPort.h
// This is a file to support porting PetitModbus
//=========================================================
#ifndef __PETIT_MODBUS_PORT_H__
#define __PETIT_MODBUS_PORT_H__

#include "PetitModbus.h"

#ifndef FALSE
#define FALSE (0)
#endif
#ifndef TRUE
#define TRUE (1)
#endif

// functions to be defined for porting
extern void PetitPortUartBegin(unsigned char tx);
extern void PetitPortTimerStart(void);
extern void PetitPortTimerStop(void);
extern void PetitPortTxPinOn(void);
extern void PetitPortTxPinOff(void);

// functions defined by petit modbus
extern void PetitRxBufferReset(void);
extern unsigned char PetitRxBufferInsert(unsigned char rcvd);
extern unsigned char PetitTxBufferPop(unsigned char* tx);

#endif
