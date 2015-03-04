/*
 * main implementation: use this 'C' sample to create your own application
 *
 */





#include "derivative.h" /* include peripheral declarations */
#include "BoardConfig.h"
#include "I2C.h"

#define ACCELEROMETER_SLAVE_ADDRESS	(0x1D)

#define TEST	(3)

uint8_t baDataOut[2] =
{
		0x01,
		0x2A
};

uint8_t baEnableAccel[2] =
{
		0x2A,
		0x01
		
};

uint8_t baDataIn[2];

uint8_t baXAxis[6];

int main(void)
{
	uint32_t dwTest = 0xFFFFFFFF;
	
	BoardConfig_vfnInit();
	while(dwTest--)
	{
		
	}
	I2C_vfnInitialization(I2C0);
#if TEST == 0
	I2C_bfnTxBuffer(I2C0,ACCELEROMETER_SLAVE_ADDRESS,&baDataOut[0],sizeof(baDataOut));
#elif TEST == 1
	I2C_bfnRxBuffer(I2C0,ACCELEROMETER_SLAVE_ADDRESS,&baDataIn[0],sizeof(baDataIn));
#elif TEST == 2
	I2C_bfnTxRxBuffer(I2C0, ACCELEROMETER_SLAVE_ADDRESS, &baDataOut[0], 1, &baDataIn[0],1);
#elif TEST == 3
	I2C_bfnTxBuffer(I2C0,ACCELEROMETER_SLAVE_ADDRESS,&baEnableAccel[0],sizeof(baEnableAccel));
#endif
	for(;;) 
	{	 
		
#if TEST == 3		
		if(!I2C_CHECK_STATUS(I2C_FRAME_IN_PROGRESS))
		{
			I2C_bfnTxRxBuffer(I2C0, ACCELEROMETER_SLAVE_ADDRESS, &baDataOut[0], 1, &baXAxis[0],6);
		}
#endif
	}
	
	return 0;
}
