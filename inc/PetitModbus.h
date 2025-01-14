
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
}  T_PETIT_XMIT_STATE;

typedef enum
{
	E_PETIT_FALSE_FUNCTION = 0,
	E_PETIT_FALSE_SLAVE_ADDRESS,
	E_PETIT_DATA_NOT_READY,
	E_PETIT_DATA_READY
} T_PETIT_BUFFER_STATUS;

typedef struct
{
	T_PETIT_XMIT_STATE Xmit_State;
	pu8_t Buffer[C_PETITMODBUS_RXTX_BUFFER_SIZE];
	pu16_t CRC16;
	pu16_t BufI;
	pu16_t BufJ;
	pu8_t *Ptr;
	pu16_t Tx_Ctr;
	pu16_t Expected_RX_Cnt;
	void (*Timer_Start)(void);
	void (*Timer_Stop)(void);
	void (*Tx_Begin)(pu8_t);
} T_PETIT_MODBUS;

// Initialization Function
void PETIT_MODBUS_Init(T_PETIT_MODBUS *Petit);

// Main Functions
void PETIT_MODBUS_Process(T_PETIT_MODBUS *Petit);

// functions defined by petit modbus
void PetitRxBufferReset(T_PETIT_MODBUS *Petit);
pb_t PetitRxBufferInsert(T_PETIT_MODBUS *Petit, pu8_t rcvd);
pb_t PetitTxBufferPop(T_PETIT_MODBUS *Petit, pu8_t* tx);

#endif
