#include <MKL25Z4.H>
#include "mma8451.h"
#include "gpio_defs.h"
#include "i2c.h"
#include "delay.h"
#include "LEDs.h"
extern volatile uint8_t ISR_rx_data[6];
extern uint8_t dataready;
extern int c3,c4,c5;
volatile uint8_t isrdata[6];
int16_t acc_X=0, acc_Y=0, acc_Z=0;
int flag = 0;
//initializes mma8451 sensor
//i2c has to already be enabled
int init_mma()
{
	uint8_t data[1];
	
	//set active mode, 14 bit samples, 2g full scale and 800 Hz ODR 
	data[0] = 0x01;
	i2c_write_bytes(MMA_ADDR, REG_CTRL1, data, 1);
	return 1;
}

void read_full_xyz()
{
	
	int i;
	uint8_t data[6];
	int16_t temp[3];
	dataready=0;
if ((!c3)){
i2c_read_bytes_isr(MMA_ADDR, REG_XHI);
}
if((!c4)){

	while (dataready==0){
	i2c_read_bytes_fsm(MMA_ADDR, REG_XHI, data,6);
ShortDelay(8);
	}
}
	
if((!c5)){
		
	i2c_read_bytes(MMA_ADDR,REG_XHI,data,6);
}
	

if((!c3)){
	for ( i=0; i<3; i++ ) {
		temp[i] = (int16_t) ((isrdata[2*i]<<8) | isrdata[2*i+1]);
	}
}
if((!c4)|(!c5)){
for ( i=0; i<3; i++ ) {
		temp[i] = (int16_t) ((data[2*i]<<8) | data[2*i+1]);
	}


}

	// Align for 14 bits
	acc_X = temp[0]/4;
	acc_Y = temp[1]/4;
	acc_Z = temp[2]/4;

}
	//}
