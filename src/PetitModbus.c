#include "PetitModbus.h"
#include <SI_EFM8BB1_Register_Enums.h>

/*******************************ModBus Functions*******************************/
#define PETITMODBUS_READ_COILS                  1
#define PETITMODBUS_READ_DISCRETE_INPUTS        2
#define PETITMODBUS_READ_HOLDING_REGISTERS      3
#define PETITMODBUS_READ_INPUT_REGISTERS        4
#define PETITMODBUS_WRITE_SINGLE_COIL           5
#define PETITMODBUS_WRITE_SINGLE_REGISTER       6
#define PETITMODBUS_WRITE_MULTIPLE_COILS        15
#define PETITMODBUS_WRITE_MULTIPLE_REGISTERS    16
/****************************End of ModBus Functions***************************/

#define PETIT_FALSE_FUNCTION                    0
#define PETIT_FALSE_SLAVE_ADDRESS               1
#define PETIT_DATA_NOT_READY                    2
#define PETIT_DATA_READY                        3

#define PETIT_ERROR_CODE_01                     0x01                            // Function code is not supported
#define PETIT_ERROR_CODE_02                     0x02                            // Register address is not allowed or write-protected

unsigned char PETITMODBUS_SLAVE_ADDRESS = 1;

typedef struct
{
	unsigned char Address;
	unsigned char Function;
	unsigned char DataBuf[PETITMODBUS_RXTX_DATA_SIZE];
	unsigned short DataLen;
} PETIT_RXTX_DATA;

extern code const unsigned short PetitCRCtable[];

/**********************Slave Transmit and Receive Variables********************/
PETIT_RXTX_STATE Petit_RxTx_State = PETIT_RXTX_IDLE;
unsigned char PetitRxTxBuffer[PETITMODBUS_RXTX_BUFFER_SIZE];
unsigned int Petit_CRC16 = 0xFFFF;

PETIT_RXTX_DATA Petit_Tx_Data;
unsigned int Petit_Tx_Delay = 0;
unsigned int Petit_Tx_Current = 0;
unsigned char *Petit_Tx_Ptr = &PetitRxTxBuffer[0];
unsigned int Petit_Tx_Buf_Size = 0;

PETIT_RXTX_DATA Petit_Rx_Data;
unsigned char *Petit_Rx_Ptr = &PetitRxTxBuffer[0];
unsigned int PetitRxRemaining = PETITMODBUS_RXTX_BUFFER_SIZE;
unsigned int PetitRxCounter = 0;

uint8_t PetitExpectedReceiveCount = 0;
/*****************************************************************************/

/*
 * Function Name        : CRC16_Calc
 * @param[in]           : Data
 * @How to use          : initialize Petit_CRC16 to 0xFFFF beforehand
 */
void Petit_CRC16_Calc(unsigned char Data)
{
	Petit_CRC16 = (Petit_CRC16 >> 8) ^
			PetitCRCtable[(Petit_CRC16 ^ (Data)) & 0xFF];
	return;
}

/*
 * Function Name        : SendMessage
 * @param[out]          : TRUE/FALSE
 * @How to use          : This function start to sending messages
 */
unsigned char PetitSendMessage(void)
{
	if (Petit_RxTx_State != PETIT_RXTX_RX_PROCESS)
		return FALSE;

	Petit_Tx_Current = 0;
	Petit_RxTx_State = PETIT_RXTX_START;

	return TRUE;
}

/******************************************************************************/

/*
 * Function Name        : HandleModbusError
 * @How to use          : This function generated errors to Modbus Master
 */
void HandlePetitModbusError(char ErrorCode)
{
	// Initialise the output buffer. The first byte in the buffer says how many registers we have read
	Petit_Tx_Data.Function = ErrorCode | 0x80;
	Petit_Tx_Data.Address = PETITMODBUS_SLAVE_ADDRESS;
	Petit_Tx_Data.DataLen = 1;
	Petit_Tx_Data.DataBuf[0] = ErrorCode;
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
	unsigned int Petit_StartAddress = 0;
	unsigned int Petit_NumberOfRegisters = 0;
	unsigned int Petit_i = 0;

	// The message contains the requested start address and number of registers
	Petit_StartAddress = ((unsigned int) (Petit_Rx_Data.DataBuf[0]) << 8)
			+ (unsigned int) (Petit_Rx_Data.DataBuf[1]);
	Petit_NumberOfRegisters = ((unsigned int) (Petit_Rx_Data.DataBuf[2]) << 8)
			+ (unsigned int) (Petit_Rx_Data.DataBuf[3]);

	// If it is bigger than RegisterNumber return error to Modbus Master
	if ((Petit_StartAddress + Petit_NumberOfRegisters)
			> NUMBER_OF_OUTPUT_PETITREGISTERS)
		HandlePetitModbusError(PETIT_ERROR_CODE_02);
	else
	{
		// Initialise the output buffer. The first byte in the buffer says how many registers we have read
		Petit_Tx_Data.Function = PETITMODBUS_READ_HOLDING_REGISTERS;
		Petit_Tx_Data.Address = PETITMODBUS_SLAVE_ADDRESS;
		Petit_Tx_Data.DataLen = 1;
		Petit_Tx_Data.DataBuf[0] = 0;

		for (Petit_i = 0; Petit_i < Petit_NumberOfRegisters; Petit_i++)
		{
			unsigned short Petit_CurrentData = PetitRegisters[Petit_StartAddress
					+ Petit_i];

			Petit_Tx_Data.DataBuf[Petit_Tx_Data.DataLen] =
					(unsigned char) ((Petit_CurrentData & 0xFF00) >> 8);
			Petit_Tx_Data.DataBuf[Petit_Tx_Data.DataLen + 1] =
					(unsigned char) (Petit_CurrentData & 0xFF);
			Petit_Tx_Data.DataLen += 2;
			Petit_Tx_Data.DataBuf[0] = Petit_Tx_Data.DataLen - 1;
		}

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
	unsigned int Petit_Address = 0;
	unsigned int Petit_Value = 0;
	unsigned char Petit_i = 0;

	// The message contains the requested start address and number of registers
	Petit_Address = ((unsigned int) (Petit_Rx_Data.DataBuf[0]) << 8)
			+ (unsigned int) (Petit_Rx_Data.DataBuf[1]);
	Petit_Value = ((unsigned int) (Petit_Rx_Data.DataBuf[2]) << 8)
			+ (unsigned int) (Petit_Rx_Data.DataBuf[3]);

	// Initialise the output buffer. The first byte in the buffer says how many registers we have read
	Petit_Tx_Data.Function = PETITMODBUS_WRITE_SINGLE_REGISTER;
	Petit_Tx_Data.Address = PETITMODBUS_SLAVE_ADDRESS;
	Petit_Tx_Data.DataLen = 4;

	if (Petit_Address >= NUMBER_OF_OUTPUT_PETITREGISTERS)
		HandlePetitModbusError(PETIT_ERROR_CODE_02);
	else
	{
		PetitRegisters[Petit_Address] = Petit_Value;
		// Output data buffer is exact copy of input buffer
		for (Petit_i = 0; Petit_i < 4; ++Petit_i)
			Petit_Tx_Data.DataBuf[Petit_i] = Petit_Rx_Data.DataBuf[Petit_i];

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
	unsigned int Petit_StartAddress = 0;
	unsigned char Petit_ByteCount = 0;
	unsigned int Petit_NumberOfRegisters = 0;
	unsigned char Petit_i = 0;
	unsigned int Petit_Value = 0;

	// The message contains the requested start address and number of registers
	Petit_StartAddress = ((unsigned int) (Petit_Rx_Data.DataBuf[0]) << 8)
			+ (unsigned int) (Petit_Rx_Data.DataBuf[1]);
	Petit_NumberOfRegisters = ((unsigned int) (Petit_Rx_Data.DataBuf[2]) << 8)
			+ (unsigned int) (Petit_Rx_Data.DataBuf[3]);
	Petit_ByteCount = Petit_Rx_Data.DataBuf[4];

	// If it is bigger than RegisterNumber return error to Modbus Master
	if ((Petit_StartAddress + Petit_NumberOfRegisters)
			> NUMBER_OF_OUTPUT_PETITREGISTERS)
		HandlePetitModbusError(PETIT_ERROR_CODE_02);
	else
	{
		// Initialise the output buffer. The first byte in the buffer says how many outputs we have set
		Petit_Tx_Data.Function = PETITMODBUS_WRITE_MULTIPLE_REGISTERS;
		Petit_Tx_Data.Address = PETITMODBUS_SLAVE_ADDRESS;
		Petit_Tx_Data.DataLen = 4;
		Petit_Tx_Data.DataBuf[0] = Petit_Rx_Data.DataBuf[0];
		Petit_Tx_Data.DataBuf[1] = Petit_Rx_Data.DataBuf[1];
		Petit_Tx_Data.DataBuf[2] = Petit_Rx_Data.DataBuf[2];
		Petit_Tx_Data.DataBuf[3] = Petit_Rx_Data.DataBuf[3];

		// Output data buffer is exact copy of input buffer
		for (Petit_i = 0; Petit_i < Petit_NumberOfRegisters; Petit_i++)
		{
			Petit_Value = (Petit_Rx_Data.DataBuf[5 + 2 * Petit_i] << 8)
					+ (Petit_Rx_Data.DataBuf[6 + 2 * Petit_i]);
			PetitRegisters[Petit_StartAddress + Petit_i] = Petit_Value;
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
unsigned char CheckPetitModbusBufferComplete(void)
{
	if (!PetitExpectedReceiveCount)
	{
		if (PetitRxCounter > 5)
		{
			if (PetitRxTxBuffer[1] >= 0x01 && PetitRxTxBuffer[1] <= 0x06)  // RHR
			{
				PetitExpectedReceiveCount = 8;
			}
			else if (PetitRxTxBuffer[1] == 0x0F || PetitRxTxBuffer[1] == 0x10)
			{
				PetitExpectedReceiveCount = PetitRxTxBuffer[6] + 9;
				if (PetitExpectedReceiveCount > PETITMODBUS_RXTX_BUFFER_SIZE)
				{
					PetitRxCounter = 0;
					PetitRxRemaining = PETITMODBUS_RXTX_BUFFER_SIZE;
					Petit_Rx_Ptr = &(PetitRxTxBuffer[0]);
					PetitExpectedReceiveCount = 0;
					return PETIT_FALSE_FUNCTION;
				}
			}
			else
			{
				PetitRxCounter = 0;
				PetitRxRemaining = PETITMODBUS_RXTX_BUFFER_SIZE;
				Petit_Rx_Ptr = &(PetitRxTxBuffer[0]);
				PetitExpectedReceiveCount = 0;
				return PETIT_FALSE_FUNCTION;
			}
		}

	}

	if (PetitExpectedReceiveCount
			&& PetitRxCounter >= PetitExpectedReceiveCount)
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
	unsigned char Petit_i;
	unsigned char Petit_ReceiveBufferControl = 0;
	Petit_ReceiveBufferControl = CheckPetitModbusBufferComplete();

	if (Petit_ReceiveBufferControl == PETIT_DATA_READY)
	{
		// disable timeout
		TCON_TR0 = false;
		TL0 = (0x20 << TL0_TL0__SHIFT);

		Petit_CRC16 = 0xFFFF;
		// move to internal datastructure
		Petit_Rx_Data.Address = PetitRxTxBuffer[0];
		Petit_CRC16_Calc(Petit_Rx_Data.Address);
		Petit_Rx_Data.Function = PetitRxTxBuffer[1];
		Petit_CRC16_Calc(Petit_Rx_Data.Function);
		Petit_Rx_Data.DataLen = 0;

		for (Petit_i = 2; Petit_i < PetitExpectedReceiveCount; Petit_i++)
			Petit_Rx_Data.DataBuf[Petit_Rx_Data.DataLen++] =
					PetitRxTxBuffer[Petit_i];

		Petit_RxTx_State = PETIT_RXTX_DATABUF;

		PetitRxCounter = 0;
		PetitRxRemaining = PETITMODBUS_RXTX_BUFFER_SIZE;
		Petit_Rx_Ptr = &(PetitRxTxBuffer[0]);
		PetitExpectedReceiveCount = 0;
	}

	if ((Petit_RxTx_State == PETIT_RXTX_DATABUF) && (Petit_Rx_Data.DataLen >= 2))
	{
		// Finish off our CRC check
		Petit_Rx_Data.DataLen -= 2;
		for (Petit_i = 0; Petit_i < Petit_Rx_Data.DataLen; ++Petit_i)
		{
			Petit_CRC16_Calc(Petit_Rx_Data.DataBuf[Petit_i]);
		}

		// allow LSB read

		if (((unsigned int) Petit_Rx_Data.DataBuf[Petit_Rx_Data.DataLen]
				+ ((unsigned int) Petit_Rx_Data.DataBuf[Petit_Rx_Data.DataLen
						+ 1] << 8)) == Petit_CRC16)
		{
			// Valid message!
			Petit_RxTx_State = PETIT_RXTX_RX_PROCESS;
		}
		else
		{
			Petit_RxTx_State = PETIT_RXTX_IDLE;
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
	Petit_Tx_Buf_Size = 0;
	Petit_CRC16 = 0xFFFF;
	PetitRxTxBuffer[Petit_Tx_Buf_Size++] = Petit_Tx_Data.Address;
	Petit_CRC16_Calc(Petit_Tx_Data.Address);
	PetitRxTxBuffer[Petit_Tx_Buf_Size++] = Petit_Tx_Data.Function;
	Petit_CRC16_Calc(Petit_Tx_Data.Function);
	for (Petit_Tx_Current = 0; Petit_Tx_Current < Petit_Tx_Data.DataLen;
			Petit_Tx_Current++)
	{
		PetitRxTxBuffer[Petit_Tx_Buf_Size++] =
				Petit_Tx_Data.DataBuf[Petit_Tx_Current];
		Petit_CRC16_Calc(Petit_Tx_Data.DataBuf[Petit_Tx_Current]);
	}

	PetitRxTxBuffer[Petit_Tx_Buf_Size++] = Petit_CRC16;
	PetitRxTxBuffer[Petit_Tx_Buf_Size++] = Petit_CRC16 >> 8;

	Petit_Tx_Ptr = &(PetitRxTxBuffer[0]);

	// one cycle for RxRTU()
	// one cycle for TxRTU()
	Petit_Tx_Delay = 2;
	Petit_RxTx_State = PETIT_RXTX_TX_DLY;
}

/*
 * Function Name        : Petit_RxProcess
 * @How to use          : Only use this function if rx is clear for processing
 */
void Petit_RxProcess(void)
{
	// Is Data for us?
	if (Petit_Rx_Data.Address == PETITMODBUS_SLAVE_ADDRESS)
	{
		// Data is for us but which function?
		switch (Petit_Rx_Data.Function)
		{
#if PETITMODBUS_READ_HOLDING_REGISTERS_ENABLED > 0
		case PETITMODBUS_READ_HOLDING_REGISTERS:
			HandlePetitModbusReadHoldingRegisters();
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
	case PETIT_RXTX_START: // if the answer is ready, send it
		Petit_TxRTU();
		// no break here to allow zero-delay sending
	case PETIT_RXTX_TX_DLY:
		// process the TX delay
		if (Petit_Tx_Delay < PETITMODBUS_DLY_TOP)
		{
			Petit_Tx_Delay++;
		}
		else
		{
			// print first character to start UART peripheral
			P0_B3 = true;
			SBUF0 = *Petit_Tx_Ptr++;
			Petit_Tx_Buf_Size--;
			Petit_RxTx_State = PETIT_RXTX_TX;
		}
		break;
	case PETIT_RXTX_TX:
		// no work is done here.  wait until transmission completes.
		break;
#if PETITMODBUS_RX_SPLIT > 0
	case PETIT_RXTX_RX_PROCESS:
		Petit_RxProcess();
		break;
#endif
	default:
		Petit_RxRTU();

#if PETITMODBUS_RX_SPLIT <= 0
		if (Petit_RxTx_State == PETIT_RXTX_RX_PROCESS)
		{
			Petit_RxProcess();
		}
#endif
		break;
	}
}

