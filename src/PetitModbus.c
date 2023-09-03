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
#define PETIT_ERROR_CODE_04						(0x04)
#define PETIT_BUF_FN_CODE_I 					(1)
#define PETIT_BUF_BYTE_CNT_I                    (6)
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

void PetitRxBufferReset()
{
	PetitBufI = 0;
	Petit_Ptr = &(PetitBuffer[0]);
	PetitExpectedReceiveCount = 0;
	return;
}

pu8_t PetitRxBufferInsert(pu8_t rcvd)
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

pu8_t PetitTxBufferPop(pu8_t* tx)
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
			return 0;
		}
	}
	return 0;
}

/*
 * Function Name        : CRC16_Calc
 * @param[in]           : Data
 * @How to use          : initialize Petit_CRC16 to 0xFFFF beforehand
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
/*
 * Function Name        : SendMessage
 * @param[out]          : TRUE/FALSE
 * @How to use          : This function start to sending messages
 */
pu8_t PetitSendMessage(void)
{
	PetitBufI = 0;
	Petit_RxTx_State = PETIT_RXTX_TX_DATABUF;

	return TRUE;
}

/******************************************************************************/

/*
 * Function Name        : HandleModbusError
 * @How to use          : This function generated errors to Modbus Master
 */
void HandlePetitModbusError(pu8_t ErrorCode)
{
	// Initialise the output buffer. The first byte in the buffer says how many registers we have read
	PetitBuffer[PETIT_BUF_FN_CODE_I] |= 0x80;
	PetitBuffer[2] = ErrorCode;
	PetitBufJ = 3;
	PetitSendMessage();
}

/******************************************************************************/

/*
 * Function Name        : HandleModbusReadHoldingRegisters
 * @How to use          : Modbus function 03 - Read holding registers
 */
#if PETITMODBUS_READ_HOLDING_REGISTERS_ENABLED > 0
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
	(PETIT_REG == PETIT_REG_INTERNAL || PETIT_REG == PETIT_REG_BOTH)
			Petit_CurrentData = PetitRegisters[Petit_StartAddress
					+ Petit_i];
#endif
#if defined(PETIT_REG) && PETIT_REG == PETIT_REG_EXTERNAL
			if (!PetitPortRegRead(Petit_StartAddress + Petit_i, &Petit_CurrentData))
			{
				HandlePetitModbusError(PETIT_ERROR_CODE_04);
				break;
			}
#endif
			PetitBuffer[PetitBufJ] =
					(pu8_t) ((Petit_CurrentData & 0xFF00) >> 8);
			PetitBuffer[PetitBufJ + 1] =
					(pu8_t) (Petit_CurrentData & 0xFF);
			PetitBufJ += 2;
		}
		PetitBuffer[2] = PetitBufJ - 3;

		PetitSendMessage();
	}
}
#endif

/*
 * Function Name        : HandleModbusReadInputRegisters
 * @How to use          : Modbus function 04 - Read holding registers
 */
#if PETITMODBUS_READ_INPUT_REGISTERS_ENABLED > 0
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
	(PETIT_INPUT_REG == PETIT_REG_INTERNAL ||\
			PETIT_INPUT_REG == PETIT_REG_BOTH)
			Petit_CurrentData = PetitInputRegisters[Petit_StartAddress
					+ Petit_i];
#endif
#if defined(PETIT_INPUT_REG) && PETIT_INPUT_REG == PETIT_REG_EXTERNAL
			if (!PetitPortInputRegRead(Petit_StartAddress + Petit_i, &Petit_CurrentData))
			{
				HandlePetitModbusError(PETIT_ERROR_CODE_04);
				break;
			}
#endif
			PetitBuffer[PetitBufJ] =
					(pu8_t) ((Petit_CurrentData & 0xFF00) >> 8);
			PetitBuffer[PetitBufJ + 1] =
					(pu8_t) (Petit_CurrentData & 0xFF);
			PetitBufJ += 2;
		}
		PetitBuffer[2] = PetitBufJ - 3;

		PetitSendMessage();
	}
}
#endif

/******************************************************************************/

/*
 * Function Name        : HandleModbusReadInputRegisters
 * @How to use          : Modbus function 06 - Write single register
 */
#if PETITMODBUSWRITE_SINGLE_REGISTER_ENABLED > 0
void HandlePetitModbusWriteSingleRegister(void)
{
	// Write single numerical output
	pu16_t Petit_Address = 0;
	pu16_t Petit_Value = 0;
	pu8_t Petit_i = 0;

	// The message contains the requested start address and number of registers
	Petit_Address = PETIT_BUF_DAT_M(0);
	Petit_Value = PETIT_BUF_DAT_M(1);

	// Initialise the output buffer. The first byte in the buffer says how many registers we have read
	PetitBufJ = 6;

	if (Petit_Address >= NUMBER_OF_PETITREGISTERS)
		HandlePetitModbusError(PETIT_ERROR_CODE_02);
	else
	{
#if defined(PETIT_REG) && \
		(PETIT_REG == PETIT_REG_INTERNAL || PETIT_REG == PETIT_REG_BOTH)
		PetitRegisters[Petit_Address] = Petit_Value;
#endif
#if defined(PETIT_REG) && PETIT_REG == PETIT_REG_EXTERNAL
		if(!PetitPortRegWrite(Petit_Address, Petit_Value))
		{
			HandlePetitModbusError(PETIT_ERROR_CODE_04);
		}
#endif
		// Output data buffer is exact copy of input buffer
		PetitRegChange = 1;
	}

	PetitSendMessage();
}
#endif

/******************************************************************************/

/*
 * Function Name        : HandleModbusWriteMultipleRegisters
 * @How to use          : Modbus function 16 - Write multiple registers
 */
#if PETITMODBUS_WRITE_MULTIPLE_REGISTERS_ENABLED > 0
void HandleMPetitodbusWriteMultipleRegisters(void)
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

		// Output data buffer is exact copy of input buffer
		for (Petit_i = 0; Petit_i < Petit_NumberOfRegisters; Petit_i++)
		{
			// 7 is the index beyond the header for the function
			Petit_Value = (PetitBuffer[2*Petit_i + 7] << 8)
					| (PetitBuffer[2*Petit_i + 8]);
#if defined(PETIT_REG) && \
		(PETIT_REG == PETIT_REG_INTERNAL || PETIT_REG == PETIT_REG_BOTH)
			PetitRegisters[Petit_StartAddress + Petit_i] = Petit_Value;
#endif
#if defined(PETIT_REG) && PETIT_REG == PETIT_REG_EXTERNAL
			PetitPortRegWrite(Petit_StartAddress + Petit_i, Petit_Value);
#endif
		}

		PetitRegChange = 1;

		PetitSendMessage();
	}
}
#endif

/******************************************************************************/

/*
 * Function Name        : CheckBufferComplete
 * @return              : If data is ready, return              DATA_READY
 *                        If slave address is wrong, return     FALSE_SLAVE_ADDRESS
 *                        If data is not ready, return          DATA_NOT_READY
 *                        If functions is wrong, return         FALSE_FUNCTION
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

/*
 * Function Name        : RxRTU
 * @How to use          : Check for data ready, if it is good return answer
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
			Petit_RxTx_State = PETIT_RXTX_RX;
		}
	}
}

/******************************************************************************/

/*
 * Function Name        : TxRTU
 * @How to use          : If it is ready send answers!
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

/*
 * Function Name        : Petit_RxProcess
 * @How to use          : Only use this function if rx is clear for processing
 */
void Petit_ResponseProcess(void)
{
	// Data is for us but which function?
	switch (PetitBuffer[PETIT_BUF_FN_CODE_I])
	{
#if PETITMODBUS_READ_HOLDING_REGISTERS_ENABLED > 0
	case PETITMODBUS_READ_HOLDING_REGISTERS:
		HandlePetitModbusReadHoldingRegisters();
		break;
#endif
#if PETITMODBUS_READ_INPUT_REGISTERS_ENABLED > 0
	case PETITMODBUS_READ_INPUT_REGISTERS:
		break;
#endif
#if PETITMODBUSWRITE_SINGLE_REGISTER_ENABLED > 0
	case PETITMODBUS_WRITE_SINGLE_REGISTER:
		HandlePetitModbusWriteSingleRegister();
		break;
#endif
#if PETITMODBUS_WRITE_MULTIPLE_REGISTERS_ENABLED > 0
	case PETITMODBUS_WRITE_MULTIPLE_REGISTERS:
		HandleMPetitodbusWriteMultipleRegisters();
		break;
#endif
	default:
		HandlePetitModbusError(PETIT_ERROR_CODE_01);
		break;
	}
	return;
}

/******************************************************************************/

/*
 * Function Name        : ProcessModbus
 * @How to use          : ModBus main core! Call this function into main!
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

