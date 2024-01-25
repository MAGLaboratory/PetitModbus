/******************************************************************************
 * @file PetitModbus.c
 *
 * This file contains the core of PetitModbus.
 *****************************************************************************/

#include "PetitModbus.h"

/*******************************ModBus Functions*******************************/
#define PETITMODBUS_READ_COILS                  (1)
#define PETITMODBUS_READ_DISCRETE_INPUTS        (2)
#define PETITMODBUS_READ_HOLDING_REGISTERS      (3)
#define PETITMODBUS_READ_INPUT_REGISTERS        (4)
#define PETITMODBUS_WRITE_SINGLE_COIL           (5)
#define PETITMODBUS_WRITE_SINGLE_REGISTER       (6)
#define PETITMODBUS_WRITE_MULTIPLE_COILS        (15)
#define PETITMODBUS_WRITE_MULTIPLE_REGISTERS    (16)
/****************************End of ModBus Functions***************************/

#define PETIT_FALSE_FUNCTION                    (0)
#define PETIT_FALSE_SLAVE_ADDRESS               (1)
#define PETIT_DATA_NOT_READY                    (2)
#define PETIT_DATA_READY                        (3)

#define PETIT_ERROR_CODE_01                     (0x01)                            // Function code is not supported
#define PETIT_ERROR_CODE_02                     (0x02)                            // Register address is not allowed or write-protected
#define PETIT_ERROR_CODE_03						(0X03)                            // Third field incorrect
#define PETIT_ERROR_CODE_04						(0x04)                            // Fourth or subsequent field incorrect

#define PETIT_BUF_FN_CODE_I 					(1)
#define PETIT_BUF_BYTE_CNT_I                    (6)
/**
 * This macro extracts the contents of the buffer at index as a 16-bit
 * unsigned integer
 */
#define PETIT_BUF_DAT_M(Idx) (((pu16_t)(PetitBuffer[2*(Idx) + 2]) << 8) \
			| (pu16_t) (PetitBuffer[2*(Idx) + 3]))

/**********************Slave Transmit and Receive Variables********************/
PETIT_RXTX_STATE Petit_RxTx_State = PETIT_RXTX_RX;
pu8_t PetitBuffer[PETITMODBUS_RXTX_BUFFER_SIZE];
pu16_t Petit_CRC16 = 0xFFFF;
// of the two, PetitBufI is used more for RX validation and TX
// PetitBufJ is used more for internal processing
pu16_t PetitBufI = 0;
pu16_t PetitBufJ = 0;

pu8_t *Petit_Ptr = &PetitBuffer[0];

pu16_t Petit_Tx_Delay = 0;

pu16_t PetitExpectedReceiveCount = 0;
/*************************************************k****************************/
/**
 * Reset the modbus buffer.
 *
 * This function is called by the interrupt code that handles byte
 * to byte time overrun.
 * It is also called by the validation function to reject data that is not for
 * this device before more system resources are taken.
 */
void PetitRxBufferReset()
{
	PetitBufI = 0;
	Petit_Ptr = &(PetitBuffer[0]);
	PetitExpectedReceiveCount = 0;
	return;
}

/**
 * Inserts bits into the buffer on device receive.
 * @param[in] rcvd the byte to insert into the buffer
 * @return bytes "left" to insert into buffer (1 if byte insertion failed)
 */
pb_t PetitRxBufferInsert(pu8_t rcvd)
{
	if (PetitBufI < PETITMODBUS_RXTX_BUFFER_SIZE
			&& Petit_RxTx_State == PETIT_RXTX_RX)
	{
		*Petit_Ptr++ = rcvd;
		PetitBufI++;
		PetitPortTimerStart();
		return 0;
	}
	return 1;
}

/**
 * This function removes a byte from the buffer and places it on "tx" to be
 * sent over rs485.
 * @param[out] tx
 * @return 1 if there is a byte to be sent, 0 otherwise if the buffer is empty
 */
pb_t PetitTxBufferPop(pu8_t* tx)
{
	if (Petit_RxTx_State == PETIT_RXTX_TX)
	{
		if (PetitBufI != 0)
		{
			*tx = *Petit_Ptr++;
			PetitBufI--;
			return 1;
		}
		else
		{
			// transmission complete.  return to receive mode.
			// the direction pin is handled by the porting code
			Petit_RxTx_State = PETIT_RXTX_RX;
			Petit_Ptr = &(PetitBuffer[0]);
			// PetitBufI is already set at 0 at this point
			PetitLedOff();
			return 0;
		}
	}
	return 0;
}

/**
 * @fn CRC16_Calc
 * This function does the CRC16 calculation for modbus.  Specifically, modbus
 * calls for CRC16 using the CRC-16-IBM polynomial.
 * @param[in] Data
 * @note initialize Petit_CRC16 to 0xFFFF beforehand
 */
#if PETIT_CRC == PETIT_CRC_TABULAR
void Petit_CRC16_Calc(const pu16_t Data)
{
	Petit_CRC16 = (Petit_CRC16 >> 8) ^
			PetitCRCtable[(Petit_CRC16 ^ (Data)) & 0xFF];
	return;
}
#elif PETIT_CRC == PETIT_CRC_BITWISE
void Petit_CRC16_Calc(const pu16_t Data)
{
	pu8_t i;

    Petit_CRC16 = Petit_CRC16 ^(pu16_t) Data;
    for (i = 8; i > 0; i--)
    {
        if (Petit_CRC16 & 0x0001)
            Petit_CRC16 = (Petit_CRC16 >> 1) ^ 0xA001;
        else
            Petit_CRC16 >>= 1;
    }
}
#elif PETIT_CRC == PETIT_CRC_EXTERNAL
#define Petit_CRC16_Calc(Data) PetitPortCRC16Calc((Data), &Petit_CRC16)
#else
#error "No Valid CRC Algorithm!"
#endif

/**
 * @fn PetitSendMessage
 * This function starts to send messages.
 * @return always returns true
 */
pb_t PetitSendMessage(void)
{
	PetitBufI = 0;
	Petit_RxTx_State = PETIT_RXTX_TX_DATABUF;

	return true;
}

/******************************************************************************/

/**
 * @fn HandlePetitModbusError
 * This function transmits generated errors to Modbus Master.
 * @param[in] ErrorCode contains the modbus error code to be sent back.
 */
void HandlePetitModbusError(pu8_t ErrorCode)
{
	// Initialise the output buffer. The first byte in the buffer says how many registers we have read
	PetitBuffer[PETIT_BUF_FN_CODE_I] |= 0x80;
	PetitBuffer[2] = ErrorCode;
	PetitBufJ = 3;
	PetitLedErrFail();
	PetitSendMessage();
}

/******************************************************************************/
/**
 * @fn HandlePetitModbusReadCoils
 * Modbus function 01 - Read Coils
 */
#if PETITMODBUS_READ_COILS_ENABLED != 0
void HandlePetitModbusReadCoils(void)
{
	pu16_t Petit_StartCoil = 0;
	pu16_t Petit_NumberOfCoils = 0;
	pu16_t Petit_i = 0;

	// The message contains the requested start address and number of registers
	Petit_StartCoil = PETIT_BUF_DAT_M(0);
	Petit_NumberOfCoils = PETIT_BUF_DAT_M(1);

	// If it is bigger than RegisterNumber return error to Modbus Master
	// there is an interesting calculation done with the number of coils here
	// since the first coil in a byte starts a new byte, we add seven to the
	// number of coils.
	// the left shift by three is a method of dividing by eight (2^3) without
	// specifically using the divide function because divisions are expensive
	// the number of registers in buffer are multiplied by two since each
	// register in modbus is 16 bits
	if ((Petit_StartCoil + Petit_NumberOfCoils)
			> NUMBER_OF_PETITCOILS ||
			(Petit_NumberOfCoils + 7) >> 3 > NUMBER_OF_REGISTERS_IN_BUFFER * 2 ||
			Petit_NumberOfCoils == 0)
	{
		HandlePetitModbusError(PETIT_ERROR_CODE_02);
	}
	else
	{
		pu8_t Petit_CurrentData = 0;
		// Initialize the output buffer.
		// The first byte in the PDU says how many bytes are in response
		PetitBufJ = 2; // at least three bytes are in response.
					   // set less here to accommodate the following loop
		PetitBuffer[2] = 0;

		for (Petit_i = 0; Petit_i < Petit_NumberOfCoils; Petit_i++)
		{
			pu8_t Petit_CurrentBit;
			if ((Petit_i & 7) == 0)
			{
				PetitBuffer[PetitBufJ++] = Petit_CurrentData;
				Petit_CurrentData = 0;
			}
#if defined(PETIT_COIL) && \
	(PETIT_COIL == PETIT_INTERNAL || PETIT_COIL == PETIT_BOTH)
			// test the current coil bit
			Petit_CurrentBit =
					(PetitCoils[(Petit_StartCoil + Petit_i) >> 3]
					& 1 << ((Petit_StartCoil + Petit_i) & 7)) != 0;
			Petit_CurrentData |= (pu8_t) Petit_CurrentBit << (Petit_i & 7);
#endif
#if defined(PETIT_COIL) && \
	(PETIT_COIL == PETIT_EXTERNAL || PETIT_COIL == PETIT_BOTH)
			if (!PetitPortCoilRead(Petit_StartCoil + Petit_i,
					&Petit_CurrentBit))
			{
				HandlePetitModbusError(PETIT_ERROR_CODE_04);
				return;
			}
			Petit_CurrentData |= (pu8_t) (Petit_CurrentBit != 0) << (Petit_i & 7);
#endif
		}
		PetitBuffer[PetitBufJ++] = Petit_CurrentData;
		PetitBuffer[2] = PetitBufJ - 3;
		PetitLedSuc();
		PetitSendMessage();
	}
}
#endif /* PETITMODBUS_READ_COILS_ENABLED */

/**
 * @fn HandlePetitModbusReadHoldingRegisters
 * Modbus function 03 - Read holding registers
 */
#if PETITMODBUS_READ_HOLDING_REGISTERS_ENABLED != 0
void HandlePetitModbusReadHoldingRegisters(void)
{
	// Holding registers are effectively numerical outputs that can be written to by the host.
	// They can be control registers or analogue outputs.
	// We potentially have one - the pwm output value
	pu16_t Petit_StartAddress = 0;
	pu16_t Petit_NumberOfRegisters = 0;
	pu16_t Petit_i = 0;

	// The message contains the requested start address and number of registers
	Petit_StartAddress = PETIT_BUF_DAT_M(0);
	Petit_NumberOfRegisters = PETIT_BUF_DAT_M(1);

	// If it is bigger than RegisterNumber return error to Modbus Master
	if ((Petit_StartAddress + Petit_NumberOfRegisters)
			> NUMBER_OF_PETITREGISTERS ||
			Petit_NumberOfRegisters > NUMBER_OF_REGISTERS_IN_BUFFER)
		HandlePetitModbusError(PETIT_ERROR_CODE_02);
	else
	{
		// Initialise the output buffer.
		// The first byte in the PDU says how many registers we have read
		PetitBufJ = 3;
		PetitBuffer[2] = 0;

		for (Petit_i = 0; Petit_i < Petit_NumberOfRegisters; Petit_i++)
		{
			pu16_t Petit_CurrentData;
#if defined(PETIT_REG) && \
	(PETIT_REG == PETIT_INTERNAL || PETIT_REG == PETIT_BOTH)
			Petit_CurrentData = PetitRegisters[Petit_StartAddress
					+ Petit_i];
#endif
#if defined(PETIT_REG) && \
	(PETIT_REG == PETIT_EXTERNAL || PETIT_REG == PETIT_BOTH)
			if (!PetitPortRegRead(Petit_StartAddress + Petit_i, &Petit_CurrentData))
			{
				HandlePetitModbusError(PETIT_ERROR_CODE_04);
				return;
			}
#endif
			PetitBuffer[PetitBufJ] =
					(pu8_t) ((Petit_CurrentData & 0xFF00) >> 8);
			PetitBuffer[PetitBufJ + 1] =
					(pu8_t) (Petit_CurrentData & 0xFF);
			PetitBufJ += 2;
		}
		PetitBuffer[2] = PetitBufJ - 3;
		PetitLedSuc();
		PetitSendMessage();
	}
}
#endif /* PETITMODBUS_READ_HOLDING_REGISTERS_ENABLED */

/**
 * @fn HandlePetitModbusReadInputRegisters
 * Modbus function 04 - Read input registers
 *
 * The registers read here are input registers only, so they can not be written
 * to using another function code.
 */
#if PETITMODBUS_READ_INPUT_REGISTERS_ENABLED != 0
void HandlePetitModbusReadInputRegisters(void)
{
	pu16_t Petit_StartAddress = 0;
	pu16_t Petit_NumberOfRegisters = 0;
	pu16_t Petit_i = 0;

	// The message contains the requested start address and number of registers
	Petit_StartAddress = PETIT_BUF_DAT_M(0);
	Petit_NumberOfRegisters = PETIT_BUF_DAT_M(1);

	// If it is bigger than RegisterNumber return error to Modbus Master
	if ((Petit_StartAddress + Petit_NumberOfRegisters)
			> NUMBER_OF_INPUT_PETITREGISTERS ||
			Petit_NumberOfRegisters > NUMBER_OF_REGISTERS_IN_BUFFER)
		HandlePetitModbusError(PETIT_ERROR_CODE_02);
	else
	{
		// Initialise the output buffer.
		// The first byte in the PDU says how many registers we have read
		PetitBufJ = 3;
		PetitBuffer[2] = 0;

		for (Petit_i = 0; Petit_i < Petit_NumberOfRegisters; Petit_i++)
		{
			pu16_t Petit_CurrentData;
#if defined(PETIT_INPUT_REG) && \
	(PETIT_INPUT_REG == PETIT_INTERNAL || PETIT_INPUT_REG == PETIT_BOTH)
			Petit_CurrentData = PetitInputRegisters[Petit_StartAddress
					+ Petit_i];
#endif
#if defined(PETIT_INPUT_REG) && \
	(PETIT_INPUT_REG == PETIT_EXTERNAL || PETIT_INPUT_REG == PETIT_BOTH)
			if (!PetitPortInputRegRead(Petit_StartAddress + Petit_i, &Petit_CurrentData))
			{
				HandlePetitModbusError(PETIT_ERROR_CODE_04);
				return;
			}
#endif
			PetitBuffer[PetitBufJ] =
					(pu8_t) ((Petit_CurrentData & 0xFF00) >> 8);
			PetitBuffer[PetitBufJ + 1] =
					(pu8_t) (Petit_CurrentData & 0xFF);
			PetitBufJ += 2;
		}
		PetitBuffer[2] = PetitBufJ - 3;
		PetitLedSuc();
		PetitSendMessage();
	}
}
#endif /* PETITMODBUS_READ_INPUT_REGISTERS_ENABLED */

/**
 * @fn HandlePetitModbusWriteSingleCoil
 * Modbus function 06 - Write single register
 */
#if PETITMODBUS_WRITE_SINGLE_COIL_ENABLED != 0
void HandlePetitModbusWriteSingleCoil(void)
{
	// Write single numerical output
	pu16_t Petit_Address = 0;
	pu16_t Petit_Value = 0;

	// The message contains the requested start address and number of registers
	Petit_Address = PETIT_BUF_DAT_M(0);
	Petit_Value = PETIT_BUF_DAT_M(1);

	// Initialise the output buffer. The first byte in the buffer says how many registers we have read
	PetitBufJ = 6;

	if (Petit_Address >= NUMBER_OF_PETITCOILS)
		HandlePetitModbusError(PETIT_ERROR_CODE_02);
	else if (Petit_Value != 0x0000 && Petit_Value != 0xFF00)
		HandlePetitModbusError(PETIT_ERROR_CODE_03);
	else
	{
#if defined(PETIT_COIL) && \
		(PETIT_COIL == PETIT_INTERNAL || PETIT_COIL == PETIT_BOTH)
		if (Petit_Value)
			PetitCoils[Petit_Address >> 3] |= 1 << (Petit_Address & 7u);
		else
			PetitCoils[Petit_Address >> 3] &= ~(1 << (Petit_Address & 7u));
#endif
#if defined(PETIT_COIL) && \
	(PETIT_COIL == PETIT_EXTERNAL || PETIT_COIL == PETIT_BOTH)
		if(!PetitPortCoilWrite(Petit_Address, Petit_Value))
		{
			HandlePetitModbusError(PETIT_ERROR_CODE_04);
			return;
		}
#endif
		// Output data buffer is exact copy of input buffer
	}
	PetitLedSuc();
	PetitSendMessage();

}
#endif /* PETITMODBUS_WRITE_SINGLE_COIL_ENABLED */

/**
 * @fn HandlePetitModbusWriteSingleRegister
 * Modbus function 06 - Write single register
 */
#if PETITMODBUS_WRITE_SINGLE_REGISTER_ENABLED != 0
void HandlePetitModbusWriteSingleRegister(void)
{
	// Write single numerical output
	pu16_t Petit_Address = 0;
	pu16_t Petit_Value = 0;

	// The message contains the requested start address and number of registers
	Petit_Address = PETIT_BUF_DAT_M(0);
	Petit_Value = PETIT_BUF_DAT_M(1);

	// Initialise the output buffer. The first byte in the buffer says how many registers we have read
	PetitBufJ = 6;

	if (Petit_Address >= NUMBER_OF_PETITREGISTERS)
		HandlePetitModbusError(PETIT_ERROR_CODE_02);
	else
	{
		PetitRegChange = 1;
#if defined(PETIT_REG) && \
	(PETIT_REG == PETIT_INTERNAL || PETIT_REG == PETIT_BOTH)
		PetitRegisters[Petit_Address] = Petit_Value;
#endif
#if defined(PETIT_REG) && \
	(PETIT_REG == PETIT_EXTERNAL || PETIT_REG == PETIT_BOTH)
		if(!PetitPortRegWrite(Petit_Address, Petit_Value))
		{
			HandlePetitModbusError(PETIT_ERROR_CODE_04);
			return;
		}
#endif
		// Output data buffer is exact copy of input buffer
	}
	PetitLedSuc();
	PetitSendMessage();
}
#endif /* PETITMODBUS_WRITE_SINGLE_REGISTER_ENABLED */

/******************************************************************************/
/**
 * @fn HandlePetitModbusWriteMultipleCoils
 * Modbus function 15 - Write multiple coils
 */
#if PETITMODBUS_WRITE_MULTIPLE_COILS_ENABLED != 0
void HandlePetitModbusWriteMultipleCoils(void){
	// Write single numerical output
	pu16_t Petit_StartCoil = 0;
	pu8_t Petit_ByteCount = 0;
	pu16_t Petit_NumberOfCoils = 0;
	pu16_t Petit_i = 0;
	pu8_t Petit_CurrentBit = 0;

	// The message contains the requested start address and number of registers
	Petit_StartCoil = PETIT_BUF_DAT_M(0);
	Petit_NumberOfCoils = PETIT_BUF_DAT_M(1);
	Petit_ByteCount = PetitBuffer[PETIT_BUF_BYTE_CNT_I];

	// If it is bigger than RegisterNumber return error to Modbus Master
	if ((Petit_StartCoil + Petit_NumberOfCoils)
			> NUMBER_OF_PETITCOILS)
		HandlePetitModbusError(PETIT_ERROR_CODE_02);
	else if (Petit_NumberOfCoils > (255 - 9) * 8 || Petit_NumberOfCoils == 0)
		HandlePetitModbusError(PETIT_ERROR_CODE_03);
	else
	{
		// Initialise the output buffer. The first byte in the buffer says how many outputs we have set
		PetitBufJ = 6;

		// Output data buffer is exact copy of input buffer
		for (Petit_i = 0; Petit_i < Petit_NumberOfCoils; Petit_i++)
		{
			// 7 is the index beyond the header for the function
			Petit_CurrentBit = (PetitBuffer[(Petit_i >> 3) + 7]
					& 1 << (Petit_i & 7))
							!= 0;
#if defined(PETIT_COIL) && \
		(PETIT_COIL == PETIT_INTERNAL || PETIT_COIL == PETIT_BOTH)
			if (Petit_CurrentBit)
				PetitCoils[(Petit_StartCoil + Petit_i) >> 3] |=
						1 << ((Petit_StartCoil + Petit_i) & 7);
			else
				PetitCoils[(Petit_StartCoil + Petit_i) >> 3] &=
						~(1 << ((Petit_StartCoil + Petit_i) & 7));
#endif
#if defined(PETIT_COIL) && \
		( PETIT_COIL == PETIT_EXTERNAL || PETIT_COIL == PETIT_BOTH)
			if (!PetitPortCoilWrite(Petit_StartCoil + Petit_i, Petit_CurrentBit))
			{
				HandlePetitModbusError(PETIT_ERROR_CODE_04);
				return;
			}
#endif
		}
		PetitLedSuc();
		PetitSendMessage();
	}
}
#endif /* PETITMODBUS_WRITE_MULTIPLE_COILS_ENABLED */

/**
 * @fn HandlePetitModbusWriteMultipleRegisters
 * Modbus function 16 - Write multiple registers
 */
#if PETITMODBUS_WRITE_MULTIPLE_REGISTERS_ENABLED != 0
void HandlePetitModbusWriteMultipleRegisters(void)
{
	// Write single numerical output
	pu16_t Petit_StartAddress = 0;
	pu8_t Petit_ByteCount = 0;
	pu16_t Petit_NumberOfRegisters = 0;
	pu8_t Petit_i = 0;
	pu16_t Petit_Value = 0;

	// The message contains the requested start address and number of registers
	Petit_StartAddress = PETIT_BUF_DAT_M(0);
	Petit_NumberOfRegisters = PETIT_BUF_DAT_M(1);
	Petit_ByteCount = PetitBuffer[PETIT_BUF_BYTE_CNT_I];

	// If it is bigger than RegisterNumber return error to Modbus Master
	if ((Petit_StartAddress + Petit_NumberOfRegisters)
			> NUMBER_OF_PETITREGISTERS)
		HandlePetitModbusError(PETIT_ERROR_CODE_02);
	else
	{
		// Initialise the output buffer. The first byte in the buffer says how many outputs we have set
		PetitBufJ = 6;
		PetitRegChange = 1;

		// Output data buffer is exact copy of input buffer
		for (Petit_i = 0; Petit_i < Petit_NumberOfRegisters; Petit_i++)
		{
			// 7 is the index beyond the header for the function
			Petit_Value = (PetitBuffer[2*Petit_i + 7] << 8)
					| (PetitBuffer[2*Petit_i + 8]);
#if defined(PETIT_REG) && \
		(PETIT_REG == PETIT_INTERNAL || PETIT_REG == PETIT_BOTH)
			PetitRegisters[Petit_StartAddress + Petit_i] = Petit_Value;
#endif
#if defined(PETIT_REG) && \
		( PETIT_REG == PETIT_EXTERNAL || PETIT_REG == PETIT_BOTH)
			if (!PetitPortRegWrite(Petit_StartAddress + Petit_i, Petit_Value))
			{
				HandlePetitModbusError(PETIT_ERROR_CODE_04);
				return;
			}
#endif
		}
		PetitLedSuc();
		PetitSendMessage();
	}
}
#endif /* PETITMODBUS_WRITE_MULTIPLE_REGISTERS_ENABLED */

/******************************************************************************/

/**
 * @fn CheckPetitModbusBufferComplete
 * @return 	DATA_READY 			If data is ready
 * 			FALSE_SLAVE_ADDRESS	If slave address is wrong
 *			DATA_NOT_READY		If data is not ready
 *			FALSE_FUNCTION		If functions is wrong
 */
pu8_t CheckPetitModbusBufferComplete(void)
{
	if (PetitBufI > 0 && PetitBuffer[0] != PETITMODBUS_SLAVE_ADDRESS)
	{
		return PETIT_FALSE_SLAVE_ADDRESS;
	}

	if (PetitBufI > 6 && PetitExpectedReceiveCount == 0)
	{
		if (PetitBuffer[PETIT_BUF_FN_CODE_I] >= 0x01
				&& PetitBuffer[PETIT_BUF_FN_CODE_I] <= 0x06)  // RHR
		{
			PetitExpectedReceiveCount = 8;
		}
		else if (PetitBuffer[PETIT_BUF_FN_CODE_I] == 0x0F
				|| PetitBuffer[PETIT_BUF_FN_CODE_I] == 0x10)
		{
			PetitExpectedReceiveCount = PetitBuffer[PETIT_BUF_BYTE_CNT_I] + 9;
			if (PetitExpectedReceiveCount > PETITMODBUS_RXTX_BUFFER_SIZE)
			{
				return PETIT_FALSE_FUNCTION;
			}
		}
		else
		{
			return PETIT_FALSE_FUNCTION;
		}
	}

	if (PetitExpectedReceiveCount
			&& PetitBufI >= PetitExpectedReceiveCount)
	{
		return PETIT_DATA_READY;
	}

	return PETIT_DATA_NOT_READY;
}

/******************************************************************************/

/**
 * @fn Petit_RxRTU
 * Check for data ready, if it is good return answer
 */
void Petit_RxRTU(void)
{
	pu8_t Petit_i;
	pu8_t Petit_ReceiveBufferControl = 0;
	Petit_ReceiveBufferControl = CheckPetitModbusBufferComplete();

	if (Petit_ReceiveBufferControl == PETIT_DATA_READY)
	{
		// disable timeout
		PetitPortTimerStop();

		// CRC calculate
		Petit_CRC16 = 0xFFFF;
		// subtract two to skip the CRC in the ADU
		PetitBufJ = PetitExpectedReceiveCount - 2;
		for (Petit_i = 0; Petit_i < PetitBufJ; ++Petit_i)
		{
			Petit_CRC16_Calc(PetitBuffer[Petit_i]);
		}

		PetitRxBufferReset();
		if (((pu16_t) PetitBuffer[PetitBufJ]
				+ ((pu16_t) PetitBuffer[PetitBufJ
						+ 1] << 8)) == Petit_CRC16)
		{
			// Valid message!
			Petit_RxTx_State = PETIT_RXTX_PROCESS;
		}
		else
		{
			PetitLedCrcFail();
			Petit_RxTx_State = PETIT_RXTX_RX;
		}
	}
}

/******************************************************************************/

/**
 * @fn Petit_TxRTU
 * If data is ready send answers!
 */
void Petit_TxRTU(void)
{
	Petit_CRC16 = 0xFFFF;
	for (PetitBufI = 0; PetitBufI < PetitBufJ;
			PetitBufI++)
	{
		Petit_CRC16_Calc(PetitBuffer[PetitBufI]);
	}

	PetitBuffer[PetitBufI++] = Petit_CRC16;
	PetitBuffer[PetitBufI++] = Petit_CRC16 >> 8;

	Petit_Ptr = &(PetitBuffer[0]);

	Petit_Tx_Delay = 0;
	Petit_RxTx_State = PETIT_RXTX_TX_DLY;
}

/**
 * @fn Petit_ResponseProcess
 * This function processes the modbus response once it has been determined that
 * the message is for this node and the length is correct.
 * @note Only use this function if rx is clear for processing
 */
void Petit_ResponseProcess(void)
{
	// Data is for us but which function?
	switch (PetitBuffer[PETIT_BUF_FN_CODE_I])
	{
#if PETITMODBUS_READ_COILS_ENABLED > 0
	case PETITMODBUS_READ_COILS:
		HandlePetitModbusReadCoils();
		break;
#endif
#if PETITMODBUS_READ_HOLDING_REGISTERS_ENABLED > 0
	case PETITMODBUS_READ_HOLDING_REGISTERS:
		HandlePetitModbusReadHoldingRegisters();
		break;
#endif
#if PETITMODBUS_READ_INPUT_REGISTERS_ENABLED > 0
	case PETITMODBUS_READ_INPUT_REGISTERS:
		HandlePetitModbusReadInputRegisters();
		break;
#endif
#if PETITMODBUS_WRITE_SINGLE_COIL_ENABLED > 0
	case PETITMODBUS_WRITE_SINGLE_COIL:
		HandlePetitModbusWriteSingleCoil();
		break;
#endif
#if PETITMODBUS_WRITE_SINGLE_REGISTER_ENABLED > 0
	case PETITMODBUS_WRITE_SINGLE_REGISTER:
		HandlePetitModbusWriteSingleRegister();
		break;
#endif
#if PETITMODBUS_WRITE_MULTIPLE_COILS_ENABLED > 0
	case PETITMODBUS_WRITE_MULTIPLE_COILS:
		HandlePetitModbusWriteMultipleCoils();
		break;
#endif
#if PETITMODBUS_WRITE_MULTIPLE_REGISTERS_ENABLED > 0
	case PETITMODBUS_WRITE_MULTIPLE_REGISTERS:
		HandlePetitModbusWriteMultipleRegisters();
		break;
#endif
	default:
		HandlePetitModbusError(PETIT_ERROR_CODE_01);
		break;
	}
	return;
}

/******************************************************************************/

/**
 * @fn ProcessPetitModbus
 * ModBus main core! Call this function into main!
 * @mermaid{ProcessPetitModbus}
 */
void ProcessPetitModbus(void)
{
	switch (Petit_RxTx_State)
	{
#if PETITMODBUS_PROCESS_POSITION >= 1
	case PETIT_RXTX_PROCESS:
		Petit_ResponseProcess();
		// no break here.  Position 1 blends processing with TxRTU.
#endif
	case PETIT_RXTX_TX_DATABUF: // if the answer is ready, send it
		Petit_TxRTU();
		// no break here.  TxRTU always exits in a correct state.
	case PETIT_RXTX_TX_DLY:
		// process the TX delay
		if (Petit_Tx_Delay < PETITMODBUS_DLY_TOP)
		{
			Petit_Tx_Delay++;
		}
		else
		{
			// print first character to start UART peripheral
			Petit_RxTx_State = PETIT_RXTX_TX;
			PetitPortTxBegin(*Petit_Ptr++);
			PetitBufI--;
		}
		break;
	case PETIT_RXTX_TX:
		// no work is done here.  wait until transmission completes.
		break;
	// position 0 has the RX process on its own.
#if PETITMODBUS_PROCESS_POSITION <= 0
	case PETIT_RXTX_PROCESS:
		Petit_ResponseProcess();
		break;
#endif
	default:
		Petit_RxRTU();
		break;
	}
}


#if !defined(PETIT_COIL) || (PETIT_COIL != PETIT_INTERNAL && \
		PETIT_COIL != PETIT_BOTH && PETIT_COIL != PETIT_EXTERNAL)
#error "PETIT_COIL not defined or not valid."
#endif

#if !defined(PETIT_REG) || (PETIT_REG != PETIT_INTERNAL && \
		PETIT_REG != PETIT_BOTH && PETIT_REG != PETIT_EXTERNAL)
#error "PETIT_REG not defined or not valid."
#endif

#if !defined(PETIT_INPUT_REG) || (PETIT_INPUT_REG != PETIT_INTERNAL && \
		PETIT_INPUT_REG != PETIT_BOTH && PETIT_INPUT_REG != PETIT_EXTERNAL)
#error "PETTI_INPUT_REG not defined or not valid."
#endif
