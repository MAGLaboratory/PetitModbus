  Project Name               :   Petit Modbus RTU \
  Project Version            :   Ver 2.0 \
  First Release Date         :   27.07.2016 \
  Last Compile Date          :   27.09.2023 \
  Original Author            :   FIRAT DEVECI \
  Original Author's Web Site :   www.firatdeveci.com                

## Description
  This is very small, tiny Modbus RTU Slave library for small microcontrollers. It supports Modbus 3-6 and 16 functions!
  Easy to use, easy to port! Examples with PIC and ARM Microcontrollers! 
 
## Using
  You only need two things:
  - Communications Send/Receive Functions
  - Tick Timer Functions

An include file called `PetitModbusUserPort.h` and a source file called
`PettiModbusPort.c` are provided.
 
  This library was tried these MCUs:
  - All of Microchip 8-16-32bit MCUs
  - STM32F0/F1/F3/F4/G3
  - MSP430
  - Atmega Series
  - Nuvoton MCUs
  - Texas DSP

### Implementation Notes
Tick timer functions can be implemented using a single timer at 250 us and
performing the timer action after +1 cycles after the normal timer expiration.
After the maximum inter word transmission period, the buffer should be cleared
if the RS485 line is not idle.

Communications functions include a direction change function if your hardware
does not support a direction pin.
The RX buffer transfer is relatively simple, and the inter word timer was
covered in the previous section.
The TX buffer transfer can be implemented on either interface buffer empty or
interface TX complete with caveats on how direction change is implemented.
If the TX buffer transfer is implemented on TX complete, the direction change
can be performed when `PetitTxBufferPop()` reports an empty TX buffer.
If the TX buffer transfer is implemented on interface buffer empty, the
direction change should first check if the buffer has completed transmission.
This can be accomplished through checking the status of the RXTX state
machine, `.Xmit_State` which is set to `E_PETIT_RXTX_RX` when transmission
completes.

## License
  It's free to use with non-commercial projects.            
 
## Support
  For support please give feedback for it!
  
## Versions
  V1.0 - First Release \
  V1.1 - Error Codes Problem is Solved \
  V2.0 - Reachitecting including CRC table, memory consumption optimization, and increased documentation.

## Contributors
| Name | Github |
| ---- | ------ |
| Firat Deveci | FxDev |
| Muhammed B. Aydemir | phantom1299 |
| İbrahim Tanağardıgil | IbrahimTanagardigil |
| Brandon Kirisaki | blu006 |

## Contact
  The original author also has a big ModBus RTU library, with the
  Master and Slave in one file.
  If you want to use this Modbus library please contact him.
  
  info [@] firatdeveci.com
