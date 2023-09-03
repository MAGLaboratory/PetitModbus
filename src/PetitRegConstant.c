#include "PetitModbus.h"
#include "PetitModbusPort.h"

/***********************Input/Output Coils and Registers***********************/
#if defined(PETIT_REG) && \
	(PETIT_REG == PETIT_REG_INTERNAL || PETIT_REG == PETIT_REG_BOTH)
pu16_t  PetitRegisters       [NUMBER_OF_PETITREGISTERS];
#endif

#if defined(PETIT_INPUT_REG) && \
		(PETIT_INPUT_REG == PETIT_REG_INTERNAL ||\
				PETIT_INPUT_REG == PETIT_REG_BOTH)
pu16_t PetitInputRegisters [NUMBER_OF_INPUT_PETITREGISTERS];
#endif

unsigned char PetitRegChange = 0;
