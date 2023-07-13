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

extern unsigned short PetitModbusTimerValue;

extern unsigned int PetitRxCounter;
extern unsigned int PetitRxRemaining;
extern unsigned char* Petit_Rx_Ptr;
extern unsigned char PetitRxTxBuffer[];
extern unsigned char PetitExpectedReceiveCount;

extern unsigned int Petit_Tx_Buf_Size;
extern unsigned char* Petit_Tx_Ptr;

extern PETIT_RXTX_STATE Petit_RxTx_State;

#endif
