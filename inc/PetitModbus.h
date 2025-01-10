
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
#define C_PETITMODBUS_RXTX_BUFFER_SIZE  (2*(NUMBER_OF_REGISTERS_IN_BUFFER) + 9)

#if C_PETIT_CRC == PETIT_CRC_TABULAR
extern PETIT_CODE const pu16_t PetitCRCtable[] PETIT_FLASH_ATTR;
#endif

typedef enum
{
    E_PETIT_RXTX_RX = 0,
	E_PETIT_RXTX_PROCESS,
    E_PETIT_RXTX_TX_DATABUF,
	E_PETIT_RXTX_TX_DLY,
    E_PETIT_RXTX_TX,
    E_PETIT_RXTX_TIMEOUT
}  PETIT_RXTX_STATE;

extern PETIT_RXTX_STATE Petit_RxTx_State;

// Main Functions
extern void                  PETIT_MODBUS_Process(void);

#endif
