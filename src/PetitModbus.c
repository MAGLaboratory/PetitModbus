/******************************************************************************
 * @file PetitModbus.c
 *
 * This file contains the core of PetitModbus.
 *****************************************************************************/

#include "PetitModbus.h"

/*******************************ModBus Functions*******************************/
#define C_FCODE_READ_COILS                  (1)
#define C_FCODE_READ_DISCRETE_INPUTS        (2)
#define C_FCODE_READ_HOLDING_REGISTERS      (3)
#define C_FCODE_READ_INPUT_REGISTERS        (4)
#define C_FCODE_WRITE_SINGLE_COIL           (5)
#define C_FCODE_WRITE_SINGLE_REGISTER       (6)
#define C_FCODE_WRITE_MULTIPLE_COILS        (15)
#define C_FCODE_WRITE_MULTIPLE_REGISTERS    (16)
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
PETIT_RXTX_STATE Petit_RxTx_State = E_PETIT_RXTX_RX;
pu8_t PetitBuffer[C_PETITMODBUS_RXTX_BUFFER_SIZE];
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

/******************************************************************************/

/**
 * @fn CheckPetitModbusBufferComplete
 * @return 	DATA_READY 			If data is ready
 * 			FALSE_SLAVE_ADDRESS	If slave address is wrong
 *			DATA_NOT_READY		If data is not ready
 *			FALSE_FUNCTION		If functions is wrong
 */
static pu8_t check_buffer_complete(void)
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
			if (PetitExpectedReceiveCount > C_PETITMODBUS_RXTX_BUFFER_SIZE)
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

/**
 * Inserts bits into the buffer on device receive.
 * @param[in] rcvd the byte to insert into the buffer
 * @return bytes "left" to insert into buffer (1 if byte insertion failed)
 */
pb_t PetitRxBufferInsert(pu8_t rcvd)
{
	if (PetitBufI < C_PETITMODBUS_RXTX_BUFFER_SIZE
			&& Petit_RxTx_State == E_PETIT_RXTX_RX)
	{
		*Petit_Ptr++ = rcvd;
		PetitBufI++;
		PetitPortTimerStart();
		if (check_buffer_complete() == PETIT_DATA_READY)
		{
			PetitPortTimerStop();
		}
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
	if (Petit_RxTx_State == E_PETIT_RXTX_TX)
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
			Petit_RxTx_State = E_PETIT_RXTX_RX;
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
#if C_PETIT_CRC == PETIT_CRC_TABULAR
static void CRC16_calc(const pu16_t Data)
{
	Petit_CRC16 = (Petit_CRC16 >> 8) ^
			PetitCRCtable[(Petit_CRC16 ^ (Data)) & 0xFF];
	return;
}
#elif C_PETIT_CRC == PETIT_CRC_BITWISE
static void CRC16_calc(const pu16_t Data)
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
#elif C_PETIT_CRC == PETIT_CRC_EXTERNAL
#define Petit_CRC16_Calc(Data) PetitPortCRC16Calc((Data), &Petit_CRC16)
#else
#error "No Valid CRC Algorithm!"
#endif

/**
 * @fn PetitSendMessage
 * This function starts to send messages.
 * @return always returns true
 */
static pb_t prepare_tx(void)
{
	PetitBufI = 0;
	Petit_RxTx_State = E_PETIT_RXTX_TX_DATABUF;

	return true;
}

/******************************************************************************/

/**
 * @fn HandlePetitModbusError
 * This function transmits generated errors to Modbus Master.
 * @param[in] ErrorCode contains the modbus error code to be sent back.
 */
static void handle_error(pu8_t ErrorCode)
{
	// Initialise the output buffer. The first byte in the buffer says how many registers we have read
	PetitBuffer[PETIT_BUF_FN_CODE_I] |= 0x80;
	PetitBuffer[2] = ErrorCode;
	PetitBufJ = 3;
	PetitLedErrFail();
	prepare_tx();
}

/******************************************************************************/
/**
 * @fn HandlePetitModbusReadCoils
 * Modbus function 01 - Read Coils
 */
#if PETITMODBUS_READ_COILS_ENABLED != 0
static void read_coils(void)
{
	pu16_t start_coil = 0;
	pu16_t number_of_coils = 0;
	pu16_t i = 0;

	// The message contains the requested start address and number of registers
	start_coil = PETIT_BUF_DAT_M(0);
	number_of_coils = PETIT_BUF_DAT_M(1);

	// If it is bigger than RegisterNumber return error to Modbus Master
	// there is an interesting calculation done with the number of coils here
	// since the first coil in a byte starts a new byte, we add seven to the
	// number of coils.
	// the left shift by three is a method of dividing by eight (2^3) without
	// specifically using the divide function because divisions are expensive
	// the number of registers in buffer are multiplied by two since each
	// register in modbus is 16 bits
	if ((start_coil + number_of_coils)
			> NUMBER_OF_PETITCOILS ||
			(number_of_coils + 7) >> 3 > NUMBER_OF_REGISTERS_IN_BUFFER * 2 ||
			number_of_coils == 0)
	{
		handle_error(PETIT_ERROR_CODE_02);
	}
	else
	{
		pu8_t data = 0;
		// Initialize the output buffer.
		// The first byte in the PDU says how many bytes are in response
		PetitBufJ = 2; // at least three bytes are in response.
					   // set less here to accommodate the following loop
		PetitBuffer[2] = 0;

		for (i = 0; i < number_of_coils; i++)
		{
			pu8_t bit;
			if ((i & 7) == 0)
			{
				PetitBuffer[PetitBufJ++] = data;
				data = 0;
			}
#if defined(PETIT_COIL) && \
	(PETIT_COIL == PETIT_INTERNAL || PETIT_COIL == PETIT_BOTH)
			// test the current coil bit
			bit =
					(PetitCoils[(start_coil + i) >> 3]
					& 1 << ((start_coil + i) & 7)) != 0;
			data |= (pu8_t) bit << (i & 7);
#endif
#if defined(PETIT_COIL) && \
	(PETIT_COIL == PETIT_EXTERNAL || PETIT_COIL == PETIT_BOTH)
			if (!PetitPortCoilRead(start_coil + Petit_i,
					&bit))
			{
				handle_error(PETIT_ERROR_CODE_04);
				return;
			}
			data |= (pu8_t) (bit != 0) << (i & 7);
#endif
		}
		PetitBuffer[PetitBufJ++] = data;
		PetitBuffer[2] = PetitBufJ - 3;
		PetitLedSuc();
		prepare_tx();
	}
}
#endif /* PETITMODBUS_READ_COILS_ENABLED */

/**
 * @fn HandlePetitModbusReadHoldingRegisters
 * Modbus function 03 - Read holding registers
 */
#if PETITMODBUS_READ_HOLDING_REGISTERS_ENABLED != 0
static void read_holding_registers(void)
{
	// Holding registers are effectively numerical outputs that can be written to by the host.
	// They can be control registers or analogue outputs.
	// We potentially have one - the pwm output value
	pu16_t start_address = 0;
	pu16_t number_of_registers = 0;
	pu16_t i = 0;

	// The message contains the requested start address and number of registers
	start_address = PETIT_BUF_DAT_M(0);
	number_of_registers = PETIT_BUF_DAT_M(1);

	// If it is bigger than RegisterNumber return error to Modbus Master
	if ((start_address + number_of_registers)
			> NUMBER_OF_PETITREGISTERS ||
			number_of_registers > NUMBER_OF_REGISTERS_IN_BUFFER)
		handle_error(PETIT_ERROR_CODE_02);
	else
	{
		// Initialise the output buffer.
		// The first byte in the PDU says how many registers we have read
		PetitBufJ = 3;
		PetitBuffer[2] = 0;

		for (i = 0; i < number_of_registers; i++)
		{
			pu16_t Petit_CurrentData;
#if defined(PETIT_REG) && \
	(PETIT_REG == PETIT_INTERNAL || PETIT_REG == PETIT_BOTH)
			Petit_CurrentData = PetitRegisters[start_address
					+ i];
#endif
#if defined(PETIT_REG) && \
	(PETIT_REG == PETIT_EXTERNAL || PETIT_REG == PETIT_BOTH)
			if (!PetitPortRegRead(start_address + i, &Petit_CurrentData))
			{
				handle_error(PETIT_ERROR_CODE_04);
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
		prepare_tx();
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
static void read_input_registers(void)
{
	pu16_t start_address = 0;
	pu16_t number_of_registers = 0;
	pu16_t i = 0;

	// The message contains the requested start address and number of registers
	start_address = PETIT_BUF_DAT_M(0);
	number_of_registers = PETIT_BUF_DAT_M(1);

	// If it is bigger than RegisterNumber return error to Modbus Master
	if ((start_address + number_of_registers)
			> NUMBER_OF_INPUT_PETITREGISTERS ||
			number_of_registers > NUMBER_OF_REGISTERS_IN_BUFFER)
		handle_error(PETIT_ERROR_CODE_02);
	else
	{
		// Initialise the output buffer.
		// The first byte in the PDU says how many registers we have read
		PetitBufJ = 3;
		PetitBuffer[2] = 0;

		for (i = 0; i < number_of_registers; i++)
		{
			pu16_t data;
#if defined(PETIT_INPUT_REG) && \
	(PETIT_INPUT_REG == PETIT_INTERNAL || PETIT_INPUT_REG == PETIT_BOTH)
			data = PetitInputRegisters[start_address
					+ i];
#endif
#if defined(PETIT_INPUT_REG) && \
	(PETIT_INPUT_REG == PETIT_EXTERNAL || PETIT_INPUT_REG == PETIT_BOTH)
			if (!PetitPortInputRegRead(start_address + i, &data))
			{
				handle_error(PETIT_ERROR_CODE_04);
				return;
			}
#endif
			PetitBuffer[PetitBufJ] =
					(pu8_t) ((data & 0xFF00) >> 8);
			PetitBuffer[PetitBufJ + 1] =
					(pu8_t) (data & 0xFF);
			PetitBufJ += 2;
		}
		PetitBuffer[2] = PetitBufJ - 3;
		PetitLedSuc();
		prepare_tx();
	}
}
#endif /* PETITMODBUS_READ_INPUT_REGISTERS_ENABLED */

/**
 * @fn HandlePetitModbusWriteSingleCoil
 * Modbus function 06 - Write single register
 */
#if PETITMODBUS_WRITE_SINGLE_COIL_ENABLED != 0
static void write_single_coil(void)
{
	// Write single numerical output
	pu16_t address = 0;
	pu16_t value = 0;

	// The message contains the requested start address and number of registers
	address = PETIT_BUF_DAT_M(0);
	value = PETIT_BUF_DAT_M(1);

	// Initialise the output buffer. The first byte in the buffer says how many registers we have read
	PetitBufJ = 6;

	if (address >= NUMBER_OF_PETITCOILS)
		handle_error(PETIT_ERROR_CODE_02);
	else if (value != 0x0000 && value != 0xFF00)
		handle_error(PETIT_ERROR_CODE_03);
	else
	{
#if defined(PETIT_COIL) && \
		(PETIT_COIL == PETIT_INTERNAL || PETIT_COIL == PETIT_BOTH)
		if (value)
			PetitCoils[address >> 3] |= 1 << (address & 7u);
		else
			PetitCoils[address >> 3] &= ~(1 << (address & 7u));
#endif
#if defined(PETIT_COIL) && \
	(PETIT_COIL == PETIT_EXTERNAL || PETIT_COIL == PETIT_BOTH)
		if(!PetitPortCoilWrite(address, value))
		{
			handle_error(PETIT_ERROR_CODE_04);
			return;
		}
#endif
		// Output data buffer is exact copy of input buffer
	}
	PetitLedSuc();
	prepare_tx();

}
#endif /* PETITMODBUS_WRITE_SINGLE_COIL_ENABLED */

/**
 * @fn HandlePetitModbusWriteSingleRegister
 * Modbus function 06 - Write single register
 */
#if PETITMODBUS_WRITE_SINGLE_REGISTER_ENABLED != 0
static void write_single_register(void)
{
	// Write single numerical output
	pu16_t address = 0;
	pu16_t value = 0;

	// The message contains the requested start address and number of registers
	address = PETIT_BUF_DAT_M(0);
	value = PETIT_BUF_DAT_M(1);

	// Initialise the output buffer. The first byte in the buffer says how many registers we have read
	PetitBufJ = 6;

	if (address >= NUMBER_OF_PETITREGISTERS)
		handle_error(PETIT_ERROR_CODE_02);
	else
	{
		PetitRegChange = 1;
#if defined(PETIT_REG) && \
	(PETIT_REG == PETIT_INTERNAL || PETIT_REG == PETIT_BOTH)
		PetitRegisters[address] = value;
#endif
#if defined(PETIT_REG) && \
	(PETIT_REG == PETIT_EXTERNAL || PETIT_REG == PETIT_BOTH)
		if(!PetitPortRegWrite(address, value))
		{
			handle_error(PETIT_ERROR_CODE_04);
			return;
		}
#endif
		// Output data buffer is exact copy of input buffer
	}
	PetitLedSuc();
	prepare_tx();
}
#endif /* PETITMODBUS_WRITE_SINGLE_REGISTER_ENABLED */

/******************************************************************************/
/**
 * @fn HandlePetitModbusWriteMultipleCoils
 * Modbus function 15 - Write multiple coils
 */
#if PETITMODBUS_WRITE_MULTIPLE_COILS_ENABLED != 0
static void write_multiple_coils(void){
	// Write single numerical output
	pu16_t start_coil = 0;
	pu8_t byte_count = 0;
	pu16_t number_of_coils = 0;
	pu16_t i = 0;
	pu8_t current_bit = 0;

	// The message contains the requested start address and number of registers
	start_coil = PETIT_BUF_DAT_M(0);
	number_of_coils = PETIT_BUF_DAT_M(1);
	byte_count = PetitBuffer[PETIT_BUF_BYTE_CNT_I];

	// If it is bigger than RegisterNumber return error to Modbus Master
	if ((start_coil + number_of_coils)
			> NUMBER_OF_PETITCOILS)
		handle_error(PETIT_ERROR_CODE_02);
	else if (number_of_coils > (255 - 9) * 8 || number_of_coils == 0)
		handle_error(PETIT_ERROR_CODE_03);
	else
	{
		// Initialise the output buffer. The first byte in the buffer says how many outputs we have set
		PetitBufJ = 6;

		// Output data buffer is exact copy of input buffer
		for (i = 0; i < number_of_coils; i++)
		{
			// 7 is the index beyond the header for the function
			current_bit = (PetitBuffer[(i >> 3) + 7]
					& 1 << (i & 7))
							!= 0;
#if defined(PETIT_COIL) && \
		(PETIT_COIL == PETIT_INTERNAL || PETIT_COIL == PETIT_BOTH)
			if (current_bit)
				PetitCoils[(start_coil + i) >> 3] |=
						1 << ((start_coil + i) & 7);
			else
				PetitCoils[(start_coil + i) >> 3] &=
						~(1 << ((start_coil + i) & 7));
#endif
#if defined(PETIT_COIL) && \
		( PETIT_COIL == PETIT_EXTERNAL || PETIT_COIL == PETIT_BOTH)
			if (!PetitPortCoilWrite(start_coil + i, current_bit))
			{
				handle_error(PETIT_ERROR_CODE_04);
				return;
			}
#endif
		}
		PetitLedSuc();
		prepare_tx();
	}
}
#endif /* PETITMODBUS_WRITE_MULTIPLE_COILS_ENABLED */

/**
 * @fn HandlePetitModbusWriteMultipleRegisters
 * Modbus function 16 - Write multiple registers
 */
#if PETITMODBUS_WRITE_MULTIPLE_REGISTERS_ENABLED != 0
static void write_multiple_registers(void)
{
	// Write single numerical output
	pu16_t start_address = 0;
	pu8_t byte_count = 0;
	pu16_t num_registers = 0;
	pu8_t i = 0;
	pu16_t value = 0;

	// The message contains the requested start address and number of registers
	start_address = PETIT_BUF_DAT_M(0);
	num_registers = PETIT_BUF_DAT_M(1);
	byte_count = PetitBuffer[PETIT_BUF_BYTE_CNT_I];

	// If it is bigger than RegisterNumber return error to Modbus Master
	if ((start_address + num_registers)
			> NUMBER_OF_PETITREGISTERS)
		handle_error(PETIT_ERROR_CODE_02);
	else
	{
		// Initialise the output buffer. The first byte in the buffer says how many outputs we have set
		PetitBufJ = 6;
		PetitRegChange = 1;

		// Output data buffer is exact copy of input buffer
		for (i = 0; i < num_registers; i++)
		{
			// 7 is the index beyond the header for the function
			value = (PetitBuffer[2*i + 7] << 8)
					| (PetitBuffer[2*i + 8]);
#if defined(PETIT_REG) && \
		(PETIT_REG == PETIT_INTERNAL || PETIT_REG == PETIT_BOTH)
			PetitRegisters[start_address + i] = value;
#endif
#if defined(PETIT_REG) && \
		( PETIT_REG == PETIT_EXTERNAL || PETIT_REG == PETIT_BOTH)
			if (!PetitPortRegWrite(start_address + i, value))
			{
				handle_error(PETIT_ERROR_CODE_04);
				return;
			}
#endif
		}
		PetitLedSuc();
		prepare_tx();
	}
}
#endif /* PETITMODBUS_WRITE_MULTIPLE_REGISTERS_ENABLED */

/******************************************************************************/

/**
 * @fn Petit_RxRTU
 * Check for data ready, if it is good return answer
 */
static void rx_rtu(void)
{
	pu8_t i;
	pu8_t buf_stat = 0;
	buf_stat = check_buffer_complete();

	if (buf_stat == PETIT_DATA_READY)
	{
		// disable timeout
		PetitPortTimerStop();

		// CRC calculate
		Petit_CRC16 = 0xFFFF;
		// subtract two to skip the CRC in the ADU
		PetitBufJ = PetitExpectedReceiveCount - 2;
		for (i = 0; i < PetitBufJ; ++i)
		{
			CRC16_calc(PetitBuffer[i]);
		}

		PetitRxBufferReset();
		if (((pu16_t) PetitBuffer[PetitBufJ]
				+ ((pu16_t) PetitBuffer[PetitBufJ
						+ 1] << 8)) == Petit_CRC16)
		{
			// Valid message!
			Petit_RxTx_State = E_PETIT_RXTX_PROCESS;
		}
		else
		{
			PetitLedCrcFail();
			Petit_RxTx_State = E_PETIT_RXTX_RX;
		}
	}
}

/******************************************************************************/

/**
 * @fn Petit_TxRTU
 * If data is ready send answers!
 */
static void tx_rtu(void)
{
	Petit_CRC16 = 0xFFFF;
	for (PetitBufI = 0; PetitBufI < PetitBufJ;
			PetitBufI++)
	{
		CRC16_calc(PetitBuffer[PetitBufI]);
	}

	PetitBuffer[PetitBufI++] = Petit_CRC16;
	PetitBuffer[PetitBufI++] = Petit_CRC16 >> 8;

	Petit_Ptr = &(PetitBuffer[0]);

	Petit_Tx_Delay = 0;
	Petit_RxTx_State = E_PETIT_RXTX_TX_DLY;
}

/**
 * @fn Petit_ResponseProcess
 * This function processes the modbus response once it has been determined that
 * the message is for this node and the length is correct.
 * @note Only use this function if rx is clear for processing
 */
static void response_process(void)
{
	// Data is for us but which function?
	switch (PetitBuffer[PETIT_BUF_FN_CODE_I])
	{
#if PETITMODBUS_READ_COILS_ENABLED > 0
	case C_FCODE_READ_COILS:
		read_coils();
		break;
#endif
#if PETITMODBUS_READ_HOLDING_REGISTERS_ENABLED > 0
	case C_FCODE_READ_HOLDING_REGISTERS:
		read_holding_registers();
		break;
#endif
#if PETITMODBUS_READ_INPUT_REGISTERS_ENABLED > 0
	case C_FCODE_READ_INPUT_REGISTERS:
		read_input_registers();
		break;
#endif
#if PETITMODBUS_WRITE_SINGLE_COIL_ENABLED > 0
	case C_FCODE_WRITE_SINGLE_COIL:
		write_single_coil();
		break;
#endif
#if PETITMODBUS_WRITE_SINGLE_REGISTER_ENABLED > 0
	case C_FCODE_WRITE_SINGLE_REGISTER:
		write_single_register();
		break;
#endif
#if PETITMODBUS_WRITE_MULTIPLE_COILS_ENABLED > 0
	case C_FCODE_WRITE_MULTIPLE_COILS:
		write_multiple_coils();
		break;
#endif
#if PETITMODBUS_WRITE_MULTIPLE_REGISTERS_ENABLED > 0
	case C_FCODE_WRITE_MULTIPLE_REGISTERS:
		write_multiple_registers();
		break;
#endif
	default:
		handle_error(PETIT_ERROR_CODE_01);
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
void PETIT_MODBUS_Process(void)
{
	switch (Petit_RxTx_State)
	{
#if PETITMODBUS_PROCESS_POSITION >= 1
	case E_PETIT_RXTX_PROCESS:
		response_process();
		// no break here.  Position 1 blends processing with TxRTU.
#endif
	case E_PETIT_RXTX_TX_DATABUF: // if the answer is ready, send it
		tx_rtu();
		// no break here.  TxRTU always exits in a correct state.
	case E_PETIT_RXTX_TX_DLY:
		// process the TX delay
		if (Petit_Tx_Delay < PETITMODBUS_DLY_TOP)
		{
			Petit_Tx_Delay++;
		}
		else
		{
			// print first character to start UART peripheral
			Petit_RxTx_State = E_PETIT_RXTX_TX;
			PetitPortTxBegin(*Petit_Ptr++);
			PetitBufI--;
		}
		break;
	case E_PETIT_RXTX_TX:
		// no work is done here.  wait until transmission completes.
		break;
	// position 0 has the RX process on its own.
#if PETITMODBUS_PROCESS_POSITION <= 0
	case E_PETIT_RXTX_PROCESS:
		response_process();
		break;
#endif
	default:
		rx_rtu();
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
