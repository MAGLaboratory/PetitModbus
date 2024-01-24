/******************************************************************************
 * @file PetitReg.c
 *
 * This file contains the definitions for register files in Petit Modbus.
 * These constants are controlled by flags which configure the library for
 * internal or external register file use.
 *****************************************************************************/
#include "PetitModbus.h"
#include "PetitModbusPort.h"

/***********************Input/Output Coils and Registers***********************/
#if defined(NUMBER_OF_COILS) && NUMBER_OF_COILS > 0
#if defined(PETIT_COIL) && \
	(PETIT_COIL == PETIT_INTERNAL || PETTI_COIL == PETIT_BOTH)
pu8_t PetitCoils           [NUMBER_OF_PETITCOILS + 7 >> 3]
#endif
#endif

#if defined(NUMBER_OF_PETITREGISTERS) && NUMBER_OF_PETITREGISTERS > 0
#if defined(PETIT_REG) && \
	(PETIT_REG == PETIT_INTERNAL || PETIT_REG == PETIT_BOTH)
pu16_t  PetitRegisters     [NUMBER_OF_PETITREGISTERS];
#endif
#endif

#if defined(NUMBER_OF_INPUT_PETITREGISTERS) && \
	NUMBER_OF_INPUT_PETITREGISTERS > 0
#if defined(PETIT_INPUT_REG) && \
		(PETIT_INPUT_REG == PETIT_INTERNAL ||\
				PETIT_INPUT_REG == PETIT_BOTH)
pu16_t PetitInputRegisters [NUMBER_OF_INPUT_PETITREGISTERS];
#endif
#endif

#if !defined(NUMBER_OF_COILS) || !defined(PETIT_COIL)
#error "Could not determine number of coils."
#endif

#if !defined(NUMBER_OF_PETITREGISTERS) || !defined(PETIT_REG)
#error "Could not determine number of holding registers."
#endif

#if !defined(NUMBER_OF_INPUT_PETITREGISTERS) || !defined(PETIT_INPUT_REG)
#error "Could not determine number of input registers."
#endif

unsigned char PetitRegChange = 0;
