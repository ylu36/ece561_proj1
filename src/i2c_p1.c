#include	 <MKL25Z4.H>
#include	 "i2c.h"
#include 	"gpio_defs.h"
#include "LEDs.h"
#include "delay.h"
#include "mma8451.h"
extern uint8_t dataready;
volatile uint8_t transmissioncomplete=0;
volatile uint8_t receptioncomplete=0;
volatile uint8_t datareadyisr,numberofbyteswritten,numberofbytesread;
extern  volatile uint8_t isrdata[6];
//init i2c0
void i2c_init( void )
{
 //clock i2c peripheral and port E
	SIM->SCGC4		 |= SIM_SCGC4_I2C0_MASK;
	SIM->SCGC5		 |= SIM_SCGC5_PORTE_MASK;

	//set pins to I2C function
	PORTE->PCR[ 24 ] |= PORT_PCR_MUX( 5 );
	PORTE->PCR[ 25 ] |= PORT_PCR_MUX( 5 );

	//set baud rate
	//baud = bus freq/(scl_div+mul)
	I2C0->F				= ( I2C_F_ICR( 0x11 ) | I2C_F_MULT( 0 ) );

	//enable i2c and set to master mode
	I2C0->C1		 |= ( I2C_C1_IICEN_MASK );

	// Select high drive mode
	I2C0->C2		 |= ( I2C_C2_HDRS_MASK );
}

void i2c_wait( void )
{
	//Control_RGB_LEDs(1,0,0);
	//Delay(100);
	SET_BIT(DEBUG3_POS);
	while( ( I2C0->S & I2C_S_IICIF_MASK ) == 0 ) {
		;
	}
	I2C0->S |= I2C_S_IICIF_MASK;
	CLEAR_BIT(DEBUG3_POS);
	//Control_RGB_LEDs(0,0,0);
	//Delay(50);
}

int i2c_read_bytes(uint8_t dev_adx, uint8_t reg_adx, uint8_t * data, uint8_t data_count) {
	uint8_t dummy, num_bytes_read=0, is_last_read=0;
	SET_BIT(DEBUG1_POS);
	I2C_TRAN;		//	set to transmit mode		
		SET_BIT(DEBUG2_POS);
	I2C_M_START;											//	send start										
	I2C0->D = dev_adx;								//	send dev address (write)							
	i2c_wait();												//	wait for completion								
	I2C0->D = reg_adx;								//	send register address								
	i2c_wait();												//	wait for completion								

	I2C_M_RSTART;											//	repeated start									
	I2C0->D = dev_adx | 0x01 ;				//	send dev address (read)							
	i2c_wait();												//	wait for completion								

	I2C_REC;													//	set to receive mode								
	while (num_bytes_read < data_count) {
		is_last_read = (num_bytes_read == data_count-1)? 1: 0;
		if (is_last_read){
			NACK;													// tell HW to send NACK after read							
		} else {
			ACK;													// tell HW to send ACK after read								
		}

		dummy = I2C0->D;								//	dummy read										
		i2c_wait();											//	wait for completion								

		if (is_last_read){
			I2C_M_STOP;				//	send stop		
			CLEAR_BIT(DEBUG2_POS);
		}
		data[num_bytes_read++] = I2C0->D; //	read data					
	}
//	CLEAR_BIT(DEBUG2_POS);
	CLEAR_BIT(DEBUG1_POS);
	return 1;
}

int i2c_write_bytes(uint8_t dev_adx, uint8_t reg_adx, uint8_t * data, uint8_t data_count) {
	uint8_t num_bytes_written=0;
	
	I2C_TRAN;													//	set to transmit mode							
	I2C_M_START;											//	send start										
	I2C0->D = dev_adx;								//	send dev address (write)							
	i2c_wait();												//	wait for completion								

	I2C0->D = reg_adx;								//	send register address								
	i2c_wait();												//	wait for completion								

	while (num_bytes_written < data_count) {
		I2C0->D = data[num_bytes_written++]; //	write data										
		i2c_wait();											//	wait for completion								
	}
	I2C_M_STOP;												//		send stop										
	
	return 1;
}
// write bytes FSM function
int i2c_read_bytes_fsm(uint8_t dev_adx, uint8_t reg_adx, uint8_t * data, uint8_t data_count)
{
		 
			static uint8_t num_bytes_read, is_last_read;
			uint8_t dummy;
			static enum {s1,s2,s3,s4,s5,s6,s7,s8,s9,s10} next_state=s1;
			// have to run this FSM only when we have new data
				CLEAR_BIT(DEBUG3_POS);
				SET_BIT(DEBUG1_POS);
			switch(next_state)
			{
				case(s1):
										
										num_bytes_read=0;
										is_last_read=0;
										
										//dataready=0;
										I2C_TRAN;
										SET_BIT(DEBUG2_POS);
										I2C_M_START;
										I2C0_D=dev_adx;
										
										next_state=s2;
									
				break;
				case(s2):
				
										
										if((I2C0_S & I2C_S_IICIF_MASK)==0)
										{
												next_state=s2;
													
										}
										else
										{
											
												next_state=s3;
											  I2C0->S |= I2C_S_IICIF_MASK;
										}
										
				break;
				case (s3):
										//dataready=0;
										
										I2C0->D=reg_adx;
										next_state=s4;
										
				break;
				case(s4):
										
										
										if((I2C0_S & I2C_S_IICIF_MASK)==0)
										{  //got stuck here
											next_state=s4;
											
										}
										else
										{
											next_state=s5;
											
											I2C0->S |= I2C_S_IICIF_MASK;
										}
								
				break;
				case(s5):
										
					
										//dataready=0;
										
										I2C_M_RSTART;											//	repeated start									
										I2C0->D = dev_adx| 0x01 ;
				
										next_state=s6;
										
				break;
				case(s6):
										//dataready=0;
							
										if((I2C0_S & I2C_S_IICIF_MASK)==0)
										{
												next_state=s6;
												
						
										}
										else
										{
											next_state=s7;
											
											I2C0->S |= I2C_S_IICIF_MASK;
										}
									
				break;				
				case(s7):
										
										//dataready=0;
										I2C_REC;
										next_state=s8;
									
				break;
				case(s8):
										
										is_last_read=(num_bytes_read==data_count-1)?1:0;
										//dataready=0;
										if(is_last_read)
										{
											NACK;
										}
										else
										{
											ACK;
										}// do we need to break the state here ?
										dummy=I2C0->D;
										
										next_state=s9;
				break;
				case(s9):
										//dataready=0;
										
										if((I2C0_S & I2C_S_IICIF_MASK)==0)
										{
												next_state=s9;
										}
										else
										{
											next_state=s10;
											I2C0->S |= I2C_S_IICIF_MASK;
										}
										
					break;
				case(s10):
										// should we declare is last read as static??
									
										if(is_last_read)
										{
											I2C_M_STOP;
											CLEAR_BIT(DEBUG2_POS);
										
										}
									data[num_bytes_read++] = I2C0->D;
										//newdataavailable =1;
										if(num_bytes_read<data_count)
										{//will this give the correct result??
												next_state=s8;
											//dataready=0;
										}
										else 
										{
											/*Control_RGB_LEDs(0,1,0);
										Delay(100);
										Control_RGB_LEDs(0,0,0);
										Delay(100);*/
												dataready=1;
												next_state=s1;// can we do this or we go to state 1 only when there is new data??
										}
									
			break;
			}
			CLEAR_BIT(DEBUG1_POS);
			return 1 ;
	
}



//implementing the flowchart entirely

int i2c_read_bytes_isr(uint8_t dev_adx, uint8_t reg_adx)
{
	//SET_BIT(DEBUG1_POS);
I2C_TRAN;
	SET_BIT(DEBUG2_POS);
I2C_M_START;
	
I2C0->D=dev_adx;
	//datareadyisr=0;
numberofbyteswritten++;
	while(transmissioncomplete==0){;}
		transmissioncomplete=0;
		

while(datareadyisr==0){

	;

}


//datareadyisr=0;
//CLEAR_BIT(DEBUG1_POS);
numberofbytesread=0;
return 1;
}
void I2C0_IRQHandler(void){
	uint8_t dummy;
	volatile uint8_t last_read;
SET_BIT(DEBUG1_POS);
	
		//CLEAR_BIT(DEBUG1_POS);
	
	I2C0->S |= I2C_S_IICIF_MASK;
	//transmissioncomplete=1;

	if(I2C0->C1 & I2C_C1_TX_MASK)
	{
		

		if(numberofbyteswritten==3){
				I2C_REC;
			//ACK;
				dummy=I2C0_D;
				ACK;
			numberofbyteswritten=0;
				//dummyreadcomplete=1;
				transmissioncomplete=1;
		}
		else if(numberofbyteswritten==2){
			I2C_M_RSTART;
		I2C0_D=MMA_ADDR|0x01;
			numberofbyteswritten++;
		transmissioncomplete=1;
		}
		else if(numberofbyteswritten==1){
			numberofbyteswritten++;
		I2C0_D=REG_XHI;
		
		}
	}
	else if(!(I2C0->C1 & I2C_C1_TX_MASK)){

				if(numberofbytesread<6)
				{
				
					last_read=(numberofbytesread==5)?1:0;
					if(numberofbytesread==4){
					NACK;
					
					}
					else{
					
					ACK;
					}
					

			
					if(numberofbytesread==5){

					I2C_M_STOP;
						CLEAR_BIT(DEBUG2_POS);
					//datareadyisr=1;
					}
						isrdata[numberofbytesread]=I2C0->D;
					numberofbytesread++;
				if(last_read){
					numberofbytesread=0;
				datareadyisr=1;
				}
				
				}
	
	
	
	}
CLEAR_BIT(DEBUG1_POS);
	
	}
