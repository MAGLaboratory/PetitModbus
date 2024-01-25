
/******************************************************************************
 * @file PetitModbus.h
 *
 * This header file is for the core of petitmodbus.
 *****************************************************************************/

#ifndef __PETITMODBUS__H
#define __PETITMODBUS__H

// Petit Modbus Port Header
#include "PetitModbusPort.h"

/****************************Don't Touch This**********************************/
// Buffers for Petit Modbus RTU Slave
// sized to hold a write to all registers
// +2 address; +2 number of registers; +1 number of bytes to follow; +2 CRC16
// +1 slave address; +1 function
#define PETITMODBUS_RXTX_BUFFER_SIZE  (2*(NUMBER_OF_REGISTERS_IN_BUFFER) + 9)

#if PETIT_CRC == PETIT_CRC_TABULAR
extern PETIT_CODE const pu16_t PetitCRCtable[] PETIT_FLASH_ATTR;
#endif

typedef enum
{
    PETIT_RXTX_RX = 0,
	PETIT_RXTX_PROCESS,
    PETIT_RXTX_TX_DATABUF,
	PETIT_RXTX_TX_DLY,
    PETIT_RXTX_TX,
    PETIT_RXTX_TIMEOUT
}  PETIT_RXTX_STATE;

extern PETIT_RXTX_STATE Petit_RxTx_State;

// Main Functions
extern void                  ProcessPetitModbus(void);

#endif
