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
#define PETIT_BUF_DAT_M(Idx) (((pu16_t)(Petit->Buffer[2U*(Idx) + 2U]) << 8U) \
			| (pu16_t) (Petit->Buffer[2U*(Idx) + 3U]))

/*************************************************k****************************/
void PETIT_MODBUS_Init(T_PETIT_MODBUS *Petit)
{
	Petit->Xmit_State = E_PETIT_RXTX_RX;
	Petit->CRC16 = 0xFFFF;
	// of the two, PetitBufI is used more for RX validation and TX
	// PetitBufJ is used more for internal processing
	// so the usage is I then J then I again
	// perhaps I can be called "xmit" and J can be called "process"
	Petit->BufI = 0;
	Petit->BufJ = 0;
	Petit->Ptr = Petit->Buffer;
	Petit->Tx_Ctr = 0;
	Petit->Expected_RX_Cnt = 0;
}

/******************************************************************************/

/**
 * Reset the modbus buffer.
 *
 * This function is called by the interrupt code that handles byte
 * to byte time overrun.
 * It is also called by the validation function to reject data that is not for
 * this device before more system resources are taken.
 */
void PetitRxBufferReset(T_PETIT_MODBUS *Petit)
{
	Petit->BufI = 0;
	Petit->Ptr = Petit->Buffer;
	Petit->Expected_RX_Cnt = 0;
	return;
}

/******************************************************************************/

/**
 * @fn check_buffer_complete
 * @return 	DATA_READY 			If data is ready
 * 			FALSE_SLAVE_ADDRESS	If slave address is wrong
 *			DATA_NOT_READY		If data is not ready
 *			FALSE_FUNCTION		If functions is wrong
 */
static T_PETIT_BUFFER_STATUS check_buffer_complete(T_PETIT_MODBUS *Petit)
{
	if (Petit->BufI > 0 && Petit->Buffer[0] != PETITMODBUS_SLAVE_ADDRESS)
	{
		return E_PETIT_FALSE_SLAVE_ADDRESS;
	}

	if (Petit->BufI > 6 && Petit->Expected_RX_Cnt == 0)
	{
		if (Petit->Buffer[PETIT_BUF_FN_CODE_I] >= 0x01U
				&& Petit->Buffer[PETIT_BUF_FN_CODE_I] <= 0x06U)  // RHR
		{
			Petit->Expected_RX_Cnt = 8U;
		}
		else if (Petit->Buffer[PETIT_BUF_FN_CODE_I] == 0x0FU
				|| Petit->Buffer[PETIT_BUF_FN_CODE_I] == 0x10U)
		{
			Petit->Expected_RX_Cnt = Petit->Buffer[PETIT_BUF_BYTE_CNT_I] + 9U;
			if (Petit->Expected_RX_Cnt > C_PETITMODBUS_RXTX_BUFFER_SIZE)
			{
				return E_PETIT_FALSE_FUNCTION;
			}
		}
		else
		{
			return E_PETIT_FALSE_FUNCTION;
		}
	}

	if (Petit->Expected_RX_Cnt && Petit->BufI >= Petit->Expected_RX_Cnt)
	{
		return E_PETIT_DATA_READY;
	}

	return E_PETIT_DATA_NOT_READY;
}

/**
 * Inserts bits into the buffer on device receive.
 * @param[in] rcvd the byte to insert into the buffer
 * @return bytes "left" to insert into buffer (1 if byte insertion failed)
 */
pb_t PetitRxBufferInsert(T_PETIT_MODBUS *Petit, pu8_t rcvd)
{
	if (Petit->BufI < C_PETITMODBUS_RXTX_BUFFER_SIZE
			&& Petit->Xmit_State == E_PETIT_RXTX_RX)
	{
		*Petit->Ptr++ = rcvd;
		Petit->BufI++;
		Petit->Timer_Start();
		if (check_buffer_complete(&Petit) == E_PETIT_DATA_READY)
		{
			Petit->Timer_Stop();
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
pb_t PetitTxBufferPop(T_PETIT_MODBUS *Petit, pu8_t* tx)
{
	if (Petit->Xmit_State == E_PETIT_RXTX_TX)
	{
		if (Petit->BufI != 0)
		{
			*tx = *Petit->Ptr++;
			Petit->BufI--;
			return 1;
		}
		else
		{
			// transmission complete.  return to receive mode.
			// the direction pin is handled by the porting code
			Petit->Xmit_State = E_PETIT_RXTX_RX;
			Petit->Ptr = Petit->Buffer;
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
static void CRC16_calc(T_PETIT_MODBUS *Petit, const pu16_t Data)
{
	Petit->CRC16 = (Petit->CRC16 >> 8) ^
			PetitCRCtable[(Petit->CRC16 ^ (Data)) & 0xFF];
	return;
}
#elif C_PETIT_CRC == PETIT_CRC_BITWISE
static void CRC16_calc(T_PETIT_MODBUS *Petit, const pu16_t Data)
{
	pu8_t i;

    Petit->CRC16 = Petit->CRC16 ^(pu16_t) Data;
    for (i = 8U; i > 0; i--)
    {
        if (Petit->CRC16 & 0x0001U)
            Petit->CRC16 = (Petit->CRC16 >> 1U) ^ 0xA001U;
        else
            Petit->CRC16 >>= 1U;
    }
}
#elif C_PETIT_CRC == PETIT_CRC_EXTERNAL
#define Petit_CRC16_Calc(Data) PetitPortCRC16Calc((Data), &Petit->CRC16)
#else
#error "No Valid CRC Algorithm!"
#endif

/**
 * @fn PetitSendMessage
 * This function starts to send messages.
 * @return always returns true
 */
static pb_t prepare_tx(T_PETIT_MODBUS *Petit)
{
	Petit->BufI = 0;
	Petit->Xmit_State = E_PETIT_RXTX_TX_DATABUF;

	return true;
}

/******************************************************************************/

/**
 * @fn HandlePetitModbusError
 * This function transmits generated errors to Modbus Master.
 * @param[in] ErrorCode contains the modbus error code to be sent back.
 */
static void handle_error(T_PETIT_MODBUS *Petit, pu8_t ErrorCode)
{
	// Initialise the output buffer. The first byte in the buffer says how many registers we have read
	Petit->Buffer[PETIT_BUF_FN_CODE_I] |= 0x80U;
	Petit->Buffer[2U] = ErrorCode;
	Petit->BufJ = 3U;
	PetitLedErrFail();
	prepare_tx(Petit);
}

/******************************************************************************/
/**
 * @fn HandlePetitModbusReadCoils
 * Modbus function 01 - Read Coils
 */
#if PETITMODBUS_READ_COILS_ENABLED != 0
static void read_coils(T_PETIT_MODBUS *Petit)
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
		handle_error(Petit, PETIT_ERROR_CODE_02);
	}
	else
	{
		pu8_t data = 0;
		// Initialize the output buffer.
		// The first byte in the PDU says how many bytes are in response
		Petit->BufJ = 2U; // at least three bytes are in response.
					   // set less here to accommodate the following loop
		Petit->Buffer[2U] = 0;

		for (i = 0; i < number_of_coils; i++)
		{
			pu8_t bit;
			if ((i & 7U) == 0)
			{
				Petit->Buffer[Petit->BufJ++] = data;
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
			data |= (pu8_t) (bit != 0) << (i & 7U);
#endif
		}
		Petit->Buffer[Petit->BufJ++] = data;
		Petit->Buffer[2U] = Petit->BufJ - 3U;
		PetitLedSuc();
		prepare_tx(Petit);
	}
}
#endif /* PETITMODBUS_READ_COILS_ENABLED */

/**
 * @fn HandlePetitModbusReadHoldingRegisters
 * Modbus function 03 - Read holding registers
 */
#if PETITMODBUS_READ_HOLDING_REGISTERS_ENABLED != 0
static void read_holding_registers(T_PETIT_MODBUS *Petit)
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
		handle_error(Petit, PETIT_ERROR_CODE_02);
	else
	{
		// Initialise the output buffer.
		// The first byte in the PDU says how many registers we have read
		Petit->BufJ = 3U;
		Petit->Buffer[2U] = 0;

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
				handle_error(Petit, PETIT_ERROR_CODE_04);
				return;
			}
#endif
			Petit->Buffer[Petit->BufJ] =
					(pu8_t) ((Petit_CurrentData & 0xFF00U) >> 8U);
			Petit->Buffer[Petit->BufJ + 1U] =
					(pu8_t) (Petit_CurrentData & 0xFFU);
			Petit->BufJ += 2U;
		}
		Petit->Buffer[2U] = Petit->BufJ - 3U;
		PetitLedSuc();
		prepare_tx(Petit);
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
static void read_input_registers(T_PETIT_MODBUS *Petit)
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
		handle_error(Petit, PETIT_ERROR_CODE_02);
	else
	{
		// Initialise the output buffer.
		// The first byte in the PDU says how many registers we have read
		Petit->BufJ = 3U;
		Petit->Buffer[2U] = 0;

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
				handle_error(Petit, PETIT_ERROR_CODE_04);
				return;
			}
#endif
			Petit->Buffer[Petit->BufJ] =
					(pu8_t) ((data & 0xFF00U) >> 8U);
			Petit->Buffer[Petit->BufJ + 1U] =
					(pu8_t) (data & 0xFFU);
			Petit->BufJ += 2U;
		}
		Petit->Buffer[2U] = Petit->BufJ - 3U;
		PetitLedSuc();
		prepare_tx(Petit);
	}
}
#endif /* PETITMODBUS_READ_INPUT_REGISTERS_ENABLED */

/**
 * @fn HandlePetitModbusWriteSingleCoil
 * Modbus function 06 - Write single register
 */
#if PETITMODBUS_WRITE_SINGLE_COIL_ENABLED != 0
static void write_single_coil(T_PETIT_MODBUS *Petit)
{
	// Write single numerical output
	pu16_t address = 0;
	pu16_t value = 0;

	// The message contains the requested start address and number of registers
	address = PETIT_BUF_DAT_M(0);
	value = PETIT_BUF_DAT_M(1);

	// Initialise the output buffer. The first byte in the buffer says how many registers we have read
	Petit->BufJ = 6U;

	if (address >= NUMBER_OF_PETITCOILS)
		handle_error(Petit, PETIT_ERROR_CODE_02);
	else if (value != 0x0000 && value != 0xFF00)
		handle_error(Petit, PETIT_ERROR_CODE_03);
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
	prepare_tx(Petit);

}
#endif /* PETITMODBUS_WRITE_SINGLE_COIL_ENABLED */

/**
 * @fn HandlePetitModbusWriteSingleRegister
 * Modbus function 06 - Write single register
 */
#if PETITMODBUS_WRITE_SINGLE_REGISTER_ENABLED != 0
static void write_single_register(T_PETIT_MODBUS *Petit)
{
	// Write single numerical output
	pu16_t address = 0;
	pu16_t value = 0;

	// The message contains the requested start address and number of registers
	address = PETIT_BUF_DAT_M(0);
	value = PETIT_BUF_DAT_M(1);

	// Initialise the output buffer. The first byte in the buffer says how many registers we have read
	Petit->BufJ = 6U;

	if (address >= NUMBER_OF_PETITREGISTERS)
		handle_error(Petit, PETIT_ERROR_CODE_02);
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
			handle_error(Petit, PETIT_ERROR_CODE_04);
			return;
		}
#endif
		// Output data buffer is exact copy of input buffer
	}
	PetitLedSuc();
	prepare_tx(Petit);
}
#endif /* PETITMODBUS_WRITE_SINGLE_REGISTER_ENABLED */

/******************************************************************************/
/**
 * @fn HandlePetitModbusWriteMultipleCoils
 * Modbus function 15 - Write multiple coils
 */
#if PETITMODBUS_WRITE_MULTIPLE_COILS_ENABLED != 0
static void write_multiple_coils(T_PETIT_MODBUS *Petit)
{
	// Write single numerical output
	pu16_t start_coil = 0;
	pu8_t byte_count = 0;
	pu16_t number_of_coils = 0;
	pu16_t i = 0;
	pu8_t current_bit = 0;

	// The message contains the requested start address and number of registers
	start_coil = PETIT_BUF_DAT_M(0);
	number_of_coils = PETIT_BUF_DAT_M(1);
	byte_count = Petit->Buffer[PETIT_BUF_BYTE_CNT_I];

	// If it is bigger than RegisterNumber return error to Modbus Master
	if ((start_coil + number_of_coils)
			> NUMBER_OF_PETITCOILS)
		handle_error(Petit, PETIT_ERROR_CODE_02);
	else if (number_of_coils > (255U - 9U) * 8U || number_of_coils == 0)
		handle_error(Petit, PETIT_ERROR_CODE_03);
	else
	{
		// Initialise the output buffer. The first byte in the buffer says how many outputs we have set
		Petit->BufJ = 6U;

		// Output data buffer is exact copy of input buffer
		for (i = 0; i < number_of_coils; i++)
		{
			// 7 is the index beyond the header for the function
			current_bit = (Petit->Buffer[(i >> 3U) + 7U]
					& 1U << (i & 7U))
							!= 0;
#if defined(PETIT_COIL) && \
		(PETIT_COIL == PETIT_INTERNAL || PETIT_COIL == PETIT_BOTH)
			if (current_bit)
				PetitCoils[(start_coil + i) >> 3U] |=
						1U << ((start_coil + i) & 7U);
			else
				PetitCoils[(start_coil + i) >> 3U] &=
						~(1U << ((start_coil + i) & 7U));
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
		prepare_tx(Petit);
	}
}
#endif /* PETITMODBUS_WRITE_MULTIPLE_COILS_ENABLED */

/**
 * @fn HandlePetitModbusWriteMultipleRegisters
 * Modbus function 16 - Write multiple registers
 */
#if PETITMODBUS_WRITE_MULTIPLE_REGISTERS_ENABLED != 0
static void write_multiple_registers(T_PETIT_MODBUS *Petit)
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
	byte_count = Petit->Buffer[PETIT_BUF_BYTE_CNT_I];

	// If it is bigger than RegisterNumber return error to Modbus Master
	if ((start_address + num_registers)
			> NUMBER_OF_PETITREGISTERS)
		handle_error(Petit, PETIT_ERROR_CODE_02);
	else
	{
		// Initialise the output buffer. The first byte in the buffer says how many outputs we have set
		Petit->BufJ = 6U;
		PetitRegChange = 1U;

		// Output data buffer is exact copy of input buffer
		for (i = 0; i < num_registers; i++)
		{
			// 7 is the index beyond the header for the function
			value = (Petit->Buffer[2U*i + 7U] << 8U)
					| (Petit->Buffer[2U*i + 8U]);
#if defined(PETIT_REG) && \
		(PETIT_REG == PETIT_INTERNAL || PETIT_REG == PETIT_BOTH)
			PetitRegisters[start_address + i] = value;
#endif
#if defined(PETIT_REG) && \
		( PETIT_REG == PETIT_EXTERNAL || PETIT_REG == PETIT_BOTH)
			if (!PetitPortRegWrite(start_address + i, value))
			{
				handle_error(Petit, PETIT_ERROR_CODE_04);
				return;
			}
#endif
		}
		PetitLedSuc();
		prepare_tx(Petit);
	}
}
#endif /* PETITMODBUS_WRITE_MULTIPLE_REGISTERS_ENABLED */

/******************************************************************************/

/**
 * @fn Petit_RxRTU
 * Check for data ready, if it is good return answer
 */
static void rx_rtu(T_PETIT_MODBUS *Petit)
{
	pu8_t i;
	T_PETIT_BUFFER_STATUS buf_stat = E_PETIT_FALSE_FUNCTION;
	buf_stat = check_buffer_complete(Petit);

	if (buf_stat == E_PETIT_DATA_READY)
	{
		// disable timeout
		PetitPortTimerStop();

		// CRC calculate
		Petit->CRC16 = 0xFFFF;
		// subtract two to skip the CRC in the ADU
		Petit->BufJ = Petit->Expected_RX_Cnt - 2U;
		for (i = 0; i < Petit->BufJ; ++i)
		{
			CRC16_calc(Petit, Petit->Buffer[i]);
		}

		PetitRxBufferReset(Petit);
		if (((pu16_t) Petit->Buffer[Petit->BufJ]
				+ ((pu16_t) Petit->Buffer[Petit->BufJ
						+ 1U] << 8U)) == Petit->CRC16)
		{
			// Valid message!
			Petit->Xmit_State = E_PETIT_RXTX_PROCESS;
		}
		else
		{
			PetitLedCrcFail();
			Petit->Xmit_State = E_PETIT_RXTX_RX;
		}
	}
}

/******************************************************************************/

/**
 * @fn Petit_TxRTU
 * If data is ready send answers!
 */
static void tx_rtu(T_PETIT_MODBUS *Petit)
{
	Petit->CRC16 = 0xFFFF;
	for (Petit->BufI = 0; Petit->BufI < Petit->BufJ;
			Petit->BufI++)
	{
		CRC16_calc(Petit, Petit->Buffer[Petit->BufI]);
	}

	Petit->Buffer[Petit->BufI++] = Petit->CRC16;
	Petit->Buffer[Petit->BufI++] = Petit->CRC16 >> 8U;

	Petit->Ptr = Petit->Buffer;

	Petit->Tx_Ctr = 0;
	Petit->Xmit_State = E_PETIT_RXTX_TX_DLY;
}

/**
 * @fn Petit_ResponseProcess
 * This function processes the modbus response once it has been determined that
 * the message is for this node and the length is correct.
 * @note Only use this function if rx is clear for processing
 */
static void response_process(T_PETIT_MODBUS *Petit)
{
	// Data is for us but which function?
	switch (Petit->Buffer[PETIT_BUF_FN_CODE_I])
	{
#if PETITMODBUS_READ_COILS_ENABLED > 0
	case C_FCODE_READ_COILS:
		read_coils(Petit);
		break;
#endif
#if PETITMODBUS_READ_HOLDING_REGISTERS_ENABLED > 0
	case C_FCODE_READ_HOLDING_REGISTERS:
		read_holding_registers(Petit);
		break;
#endif
#if PETITMODBUS_READ_INPUT_REGISTERS_ENABLED > 0
	case C_FCODE_READ_INPUT_REGISTERS:
		read_input_registers(Petit);
		break;
#endif
#if PETITMODBUS_WRITE_SINGLE_COIL_ENABLED > 0
	case C_FCODE_WRITE_SINGLE_COIL:
		write_single_coil(Petit);
		break;
#endif
#if PETITMODBUS_WRITE_SINGLE_REGISTER_ENABLED > 0
	case C_FCODE_WRITE_SINGLE_REGISTER:
		write_single_register(Petit);
		break;
#endif
#if PETITMODBUS_WRITE_MULTIPLE_COILS_ENABLED > 0
	case C_FCODE_WRITE_MULTIPLE_COILS:
		write_multiple_coils(Petit);
		break;
#endif
#if PETITMODBUS_WRITE_MULTIPLE_REGISTERS_ENABLED > 0
	case C_FCODE_WRITE_MULTIPLE_REGISTERS:
		write_multiple_registers(Petit);
		break;
#endif
	default:
		handle_error(Petit, PETIT_ERROR_CODE_01);
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
void PETIT_MODBUS_Process(T_PETIT_MODBUS *Petit)
{
	switch (Petit->Xmit_State)
	{
#if PETITMODBUS_PROCESS_POSITION >= 1
	case E_PETIT_RXTX_PROCESS:
		response_process(Petit);
		// no break here.  Position 1 blends processing with TxRTU.
#endif
	case E_PETIT_RXTX_TX_DATABUF: // if the answer is ready, send it
		tx_rtu(Petit);
		// no break here.  TxRTU always exits in a correct state.
	case E_PETIT_RXTX_TX_DLY:
		// process the TX delay
		if (Petit->Tx_Ctr < PETITMODBUS_DLY_TOP)
		{
			Petit->Tx_Ctr++;
		}
		else
		{
			// print first character to start UART peripheral
			Petit->Xmit_State = E_PETIT_RXTX_TX;
			Petit->Tx_Begin(*Petit->Ptr++);
			Petit->BufI--;
		}
		break;
	case E_PETIT_RXTX_TX:
		// no work is done here.  wait until transmission completes.
		break;
	// position 0 has the RX process on its own.
#if PETITMODBUS_PROCESS_POSITION <= 0
	case E_PETIT_RXTX_PROCESS:
		response_process(Petit);
		break;
#endif
	default:
		rx_rtu(Petit);
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
