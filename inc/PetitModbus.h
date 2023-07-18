/*  PetitModbus Version 1.0
 *  Author  :   Firat DEVECI
 *  Date    :   27.07.16
 *  
 *  Tips    :   If you want to use RS485 you have to use RX-Pull-Up Resistor!
 */

#ifndef __PETITMODBUS__H
#define __PETITMODBUS__H

#define NUMBER_OF_OUTPUT_PETITREGISTERS           ( 3 )                      // Petit Modbus RTU Slave Output Register Number
                                                                             // Have to put a number of registers here
                                                                             // It has to be bigger than 0 (zero)!!
#define PETITMODBUS_DLY_TOP                       ( 1 )                      // Timeout Constant for Petit Modbus RTU Slave [millisecond]

#define PETITMODBUS_READ_HOLDING_REGISTERS_ENABLED      ( 1 )                   // If you want to use make it 1, or 0
#define PETITMODBUSWRITE_SINGLE_REGISTER_ENABLED        ( 1 )                   // If you want to use make it 1, or 0
#define PETITMODBUS_WRITE_MULTIPLE_REGISTERS_ENABLED    ( 1 )                   // If you want to use make it 1, or 0
#define PETITMODBUS_RX_SPLIT                            ( 1 )
/****************************Don't Touch This**********************************/
// Buffers for Petit Modbus RTU Slave
// sized to hold a write to all registers
// +2 address; +2 number of registers; +1 number of bytes to follow; +2 CRC16
#define PETITMODBUS_RXTX_DATA_SIZE                 (NUMBER_OF_OUTPUT_PETITREGISTERS*2 + 7)
// last offset, +1 address; +1 function
#define PETITMODBUS_RXTX_BUFFER_SIZE               (NUMBER_OF_OUTPUT_PETITREGISTERS*2 + 9)

// Variable for Slave Address
extern unsigned char PETITMODBUS_SLAVE_ADDRESS;                                 // Petit Modbus RTU Slave icin adres numarasi [0 to 255]

typedef short PetitRegStructure;

extern PetitRegStructure    PetitRegisters[NUMBER_OF_OUTPUT_PETITREGISTERS];
extern unsigned char PetitRegChange;

typedef enum
{
    PETIT_RXTX_IDLE = 0,
	PETIT_RXTX_TX_DLY,
    PETIT_RXTX_TX,
    PETIT_RXTX_START,
	PETIT_RXTX_RX_PROCESS,
    PETIT_RXTX_DATABUF,
    PETIT_RXTX_TIMEOUT
}  PETIT_RXTX_STATE;

// Main Functions
extern void             ProcessPetitModbus(void);

// Petit Modbus Port Header
#include "PetitModbusPort.h"

#endif
