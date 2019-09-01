#include<LPC17xx.h>
#include"I2C.h"

void Config_I2C(unsigned short I2Cx,unsigned short Mode,unsigned short tran_rec)
{
	switch(I2Cx)
	{
		case I2C0:I2C0CONCLR&=0x0000006C;
			        break;
		
		case I2C1:I2C1CONCLR&=0x0000006C;
			        break;
		
		case I2C2:I2C2CONCLR&=0x0000006C;
			        break;
	}
	
}
