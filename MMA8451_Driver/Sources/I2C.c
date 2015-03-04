/*HEADER******************************************************************************************
*
* Comments:
*
*
**END********************************************************************************************/
///////////////////////////////////////////////////////////////////////////////////////////////////
//                                      Includes Section                        
///////////////////////////////////////////////////////////////////////////////////////////////////
#include <derivative.h>
#include "NVIC.h"
#include "I2C.h"
#include "I2C_Config.h"
///////////////////////////////////////////////////////////////////////////////////////////////////
//                                   Defines & Macros Section                   
///////////////////////////////////////////////////////////////////////////////////////////////////

//! Macro used to poll the status register
#define I2C_CHECK_FRAME(X)	(I2CwFrameStatus&(1<<X))
//! Macro used to set specific status
#define I2C_SET_FRAME(X)	(I2CwFrameStatus |= (1<<X))
//! Macro used to set specific status
#define I2C_CLEAR_FRAME(X)	(I2CwFrameStatus &= ~(1<<X))
//! Amount of configuration registers for I2C modules
#define I2C_REGISTERS	(12)
//! I2C sends a start condition
#define I2C_START(reg)	(reg |= I2C_C1_MST_MASK)
//! I2C sends a stop condition
#define I2C_STOP(reg)	(reg &= ~I2C_C1_MST_MASK)
//! I2C transmit
#define I2C_TX(reg)		(reg |= I2C_C1_TX_MASK)
//! I2C receive
#define I2C_RX(reg)		(reg &= ~I2C_C1_TX_MASK)
//! I2C sends a repeated start condition
#define I2C_REPEAT_START(reg)	(reg |= I2C_C1_RSTA_MASK)
//! I2C sends a AK
#define I2C_TX_AK(reg)	(reg &= ~I2C_C1_TXAK_MASK)
//! I2C sends a NAK
#define I2C_TX_NAK(reg)	(reg |= I2C_C1_TXAK_MASK)
//! Slave address write
#define I2C_SLAVE_ADDRESS_WRITE(address)	((address<<1)&0XFE)
//! Slave address read
#define I2C_SLAVE_ADDRESS_READ(address)	((address<<1)|0X01)

enum eI2CFrameStates
{
	I2C_FRAME_WRITE = 0,
	I2C_FRAME_READ,
	I2C_FRAME_WRITE_READ,
	I2C_FRAME_FIRST_READ
};

enum eI2C_Registers
{
	I2C_A1 = 0,
	I2C_F,
	I2C_C1,
	I2C_S,
	I2C_D,
	I2C_C2,
	I2C_FLT,
	I2C_RA,
	I2C_SMB,
	I2C_A2,
	I2C_SLTH,
	I2C_SLTL
};
///////////////////////////////////////////////////////////////////////////////////////////////////
//                                       Typedef Section                        
///////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////
//                                  Function Prototypes Section                 
///////////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////////
//                                   Global Constants Section                   
///////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////
//                                   Static Constants Section                   
///////////////////////////////////////////////////////////////////////////////////////////////////
//! Array with each I2C clock gate masks
static const uint32_t I2C_gadwClockGateMask[MAX_I2C] =
{
		SIM_SCGC4_I2C0_MASK,
		SIM_SCGC4_I2C1_MASK
};


//! Each UART NVIC source
static const uint32_t I2C_gadwNvicSources[MAX_I2C] =
{
		NVIC_I2C0,
		NVIC_I2C1
};
///////////////////////////////////////////////////////////////////////////////////////////////////
//                                   Global Variables Section                   
///////////////////////////////////////////////////////////////////////////////////////////////////

uint32_t I2C_gdwDriverStatus = 0;

///////////////////////////////////////////////////////////////////////////////////////////////////
//                                   Static Variables Section                   
///////////////////////////////////////////////////////////////////////////////////////////////////
//! Pointer to data output
static uint8_t * I2C_pbDataOutput = 0;
//! Slave address to be used
static uint8_t I2C_bSlaveAddress = 0;
//! Amount of bytes to be sent
static uint8_t I2C_bDataOutCounter = 0;
//! Amount of bytes to receive
static uint8_t I2C_bDataInCounter = 0;
//! Pointer to data output
static uint8_t * I2C_pbDataInput = 0;

static volatile uint16_t I2CwFrameStatus = 0;
//! Each I2C module registers
static volatile uint8_t * const I2C_gapbRegisters[MAX_I2C][I2C_REGISTERS] =
{
		{
			&I2C0_A1,
			&I2C0_F,
			&I2C0_C1,
			&I2C0_S,
			&I2C0_D,
			&I2C0_C2,
			&I2C0_FLT,
			&I2C0_RA,
			&I2C0_SMB,
			&I2C0_A2,
			&I2C0_SLTH,
			&I2C0_SLTL
		},
		{
			&I2C1_A1,
			&I2C1_F,
			&I2C1_C1,
			&I2C1_S,
			&I2C1_D,
			&I2C1_C2,
			&I2C1_FLT,
			&I2C1_RA,
			&I2C1_SMB,
			&I2C1_A2,
			&I2C1_SLTH,
			&I2C1_SLTL
		},

};

///////////////////////////////////////////////////////////////////////////////////////////////////
//                                      Functions Section                       
///////////////////////////////////////////////////////////////////////////////////////////////////

void I2C_vfnInitialization(uint8_t bModuleToEnable)
{
	uint8_t * pbControlRegister = 0;
	uint8_t * pbFrequencyRegister = 0;
	
	if(bModuleToEnable < MAX_I2C)
	{
		/* Turn on the clock gate for the proper I2C */	
		SIM_SCGC4 |= I2C_gadwClockGateMask[bModuleToEnable];
		
		pbFrequencyRegister = (uint8_t*)I2C_gapbRegisters[bModuleToEnable][I2C_F];
		
		/* Clear the register*/
		*pbFrequencyRegister = 0;
		/* Write the new I2C speed*/
		*pbFrequencyRegister = I2C_F_MULT(I2C_MULT_FACTOR - 1U) | I2C_F_ICR(I2C_CLOCK_RATE);
		
		/* Once the speed is set, enable the module */
		pbControlRegister = (uint8_t*)I2C_gapbRegisters[bModuleToEnable][I2C_C1];
			
		*pbControlRegister |= I2C_C1_IICEN_MASK | I2C_C1_IICIE_MASK;
		
		/* Enable the interrupt of the proper UART */
		NVIC_vfnEnableIRQ(I2C_gadwNvicSources[bModuleToEnable]);
	}
}

uint8_t I2C_bfnTxBuffer(uint8_t bModuleToUse, uint8_t bSalveAddress, uint8_t * pbDataBuffer, uint8_t bDataSize)
{
	uint8_t * pbControlRegister;
	uint8_t * pbDataRegister;
	uint8_t bFrameStatus = I2C_BUSY;
	
	if(!I2C_CHECK_STATUS(I2C_FRAME_IN_PROGRESS))
	{
		I2C_SET_STATUS(I2C_FRAME_IN_PROGRESS);
		
		bFrameStatus = I2C_OK;
		
		I2C_pbDataOutput = pbDataBuffer;
		
		I2C_bDataOutCounter = bDataSize;
		
		I2C_SET_FRAME(I2C_FRAME_WRITE);
		
		pbControlRegister = (uint8_t*)I2C_gapbRegisters[bModuleToUse][I2C_C1];
		
		pbDataRegister = (uint8_t*)I2C_gapbRegisters[bModuleToUse][I2C_D];
		/* set transmit mode */
		I2C_TX(*pbControlRegister);
		/* Generate a start condition and send the slave address for write */
		I2C_START(*pbControlRegister);
		
		*pbDataRegister = I2C_SLAVE_ADDRESS_WRITE(bSalveAddress);
	}
	
	return(bFrameStatus);
}

uint8_t I2C_bfnRxBuffer(uint8_t bModuleToUse, uint8_t bSalveAddress, uint8_t * pbDataBuffer, uint8_t bDataSize)
{
	uint8_t * pbControlRegister;
	uint8_t * pbDataRegister;
	uint8_t bFrameStatus = I2C_BUSY;
	
	if(!I2C_CHECK_STATUS(I2C_FRAME_IN_PROGRESS))
	{
		I2C_SET_STATUS(I2C_FRAME_IN_PROGRESS);
		
		bFrameStatus = I2C_OK;
		
		I2C_pbDataInput = pbDataBuffer;
		
		I2C_bDataInCounter = bDataSize;
		
		I2C_SET_FRAME(I2C_FRAME_READ);
		
		I2C_CLEAR_FRAME(I2C_FRAME_FIRST_READ);
		
		pbControlRegister = (uint8_t*)I2C_gapbRegisters[bModuleToUse][I2C_C1];
		
		pbDataRegister = (uint8_t*)I2C_gapbRegisters[bModuleToUse][I2C_D];
		
		
		
		if(bDataSize != 1)
		{
			I2C_TX_AK(*pbControlRegister);
		}
		else
		{
			/* set transmit mode */
			I2C_TX_NAK(*pbControlRegister);
		}
		
		/* set transmit mode */
		I2C_TX(*pbControlRegister);
		/* Generate a start condition and send the slave address for write */
		I2C_START(*pbControlRegister);
		
		*pbDataRegister = I2C_SLAVE_ADDRESS_READ(bSalveAddress);
	}
	
	return(bFrameStatus);
}

uint8_t I2C_bfnTxRxBuffer(uint8_t bModuleToUse, uint8_t bSalveAddress, uint8_t * pbDataBufferOut, uint8_t bDataOutSize, uint8_t * pbDataBufferIn, uint8_t bDataInSize)
{
	uint8_t * pbControlRegister;
	uint8_t * pbDataRegister;
	uint8_t bFrameStatus = I2C_BUSY;
	
	if(!I2C_CHECK_STATUS(I2C_FRAME_IN_PROGRESS))
	{
		I2C_SET_STATUS(I2C_FRAME_IN_PROGRESS);
		
		bFrameStatus = I2C_OK;
		
		
		/* backup all pointers and data sizes*/
		I2C_pbDataOutput = pbDataBufferOut;
		
		I2C_bDataOutCounter = bDataOutSize;
		
		I2C_SET_FRAME(I2C_FRAME_WRITE);
		
		pbControlRegister = (uint8_t*)I2C_gapbRegisters[bModuleToUse][I2C_C1];
		
		pbDataRegister = (uint8_t*)I2C_gapbRegisters[bModuleToUse][I2C_D];
		
		I2C_pbDataInput = pbDataBufferIn;
				
		I2C_bDataInCounter = bDataInSize;
		
		I2C_bSlaveAddress = bSalveAddress;
		
		I2C_SET_FRAME(I2C_FRAME_READ);
		
		I2C_CLEAR_FRAME(I2C_FRAME_FIRST_READ);
		
		I2C_SET_FRAME(I2C_FRAME_WRITE_READ);
		/* If the amount of data to be sent is just 1, make sure TXNACK is sent */
		if(bDataInSize != 1)
		{
			I2C_TX_AK(*pbControlRegister);
		}
		else
		{
			/* set transmit mode */
			I2C_TX_NAK(*pbControlRegister);
		}
		/* set transmit mode */
		I2C_TX(*pbControlRegister);
		/* Generate a start condition and send the slave address for write */
		I2C_START(*pbControlRegister);
		
		*pbDataRegister = I2C_SLAVE_ADDRESS_WRITE(bSalveAddress);
	}
	
	return(bFrameStatus);	
}


void I2C0_IRQHandler(void)
{
	I2C0_S |= I2C_S_IICIF_MASK;
	
	if(!(I2C0_S & I2C_S_RXAK_MASK) || (I2C_CHECK_FRAME(I2C_FRAME_READ)))
	{
		if(I2C0_S & I2C_S_TCF_MASK)
		{
			if(I2C_CHECK_FRAME(I2C_FRAME_WRITE))
			{
				/* Send data as long as there's data size */
				if(I2C_bDataOutCounter)
				{
					I2C_bDataOutCounter--;
					I2C0_D = *I2C_pbDataOutput;
					I2C_pbDataOutput++;
				}
				else
				{
					/* In case READ_WRITE is enable, generate a repeated start 	*/
					/* or a stop otherwise										*/
					if(!I2C_CHECK_FRAME(I2C_FRAME_WRITE_READ))
					{
						I2C_STOP(I2C0_C1);
						I2C_CLEAR_STATUS(I2C_FRAME_IN_PROGRESS);
					}
					else
					{
						I2C_CLEAR_FRAME(I2C_FRAME_WRITE);
						I2C_REPEAT_START(I2C0_C1);
						I2C0_D = I2C_SLAVE_ADDRESS_READ(I2C_bSlaveAddress);
					}
					
				}
			}
			else
			{
				if(I2C_CHECK_FRAME(I2C_FRAME_READ))
				{

					if(I2C_CHECK_FRAME(I2C_FRAME_FIRST_READ))
					{
						/* Keep reading data */
						if(I2C_bDataInCounter)
						{
							/* send a TXNAK in case one byte is left to be read */
							if(I2C_bDataInCounter == 1)
							{
								I2C_TX_NAK(I2C0_C1);
							}
							
							I2C_bDataInCounter--;
							*I2C_pbDataInput = I2C0_D; 
							I2C_pbDataInput++;
						}
						else
						{
							/* generate a stop signal and read the last byte */
							I2C_STOP(I2C0_C1);
							I2C_CLEAR_STATUS(I2C_FRAME_IN_PROGRESS);
							*I2C_pbDataInput = I2C0_D; 
						}
					}
					else
					{
						/* Set I2C as RX and perform a dummy read (just if its the first time)*/
						I2C_RX(I2C0_C1);
						I2C_SET_FRAME(I2C_FRAME_FIRST_READ);
						I2C_bDataInCounter--;
						(void)I2C0_D;
					}
				}
			}
		}
	}
	else
	{
		I2C_SET_STATUS(I2C_NAK_RECEIVED);
		I2C_CLEAR_STATUS(I2C_FRAME_IN_PROGRESS);
		I2C_STOP(I2C0_C1);
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////////
// EOF
///////////////////////////////////////////////////////////////////////////////////////////////////
