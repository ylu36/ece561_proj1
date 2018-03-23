/*----------------------------------------------------------------------------
 *----------------------------------------------------------------------------*/
#include <MKL25Z4.H>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "gpio_defs.h"
#include "LEDs.h"
#include "i2c.h"
#include "mma8451.h"
#include "delay.h"
uint8_t dataready=0;
 int c3,c4,c5;
#define FLASH_DELAY 10
#define ACC_SENSITIVITY 40
void Init_Debug_Signals(void) {
	
		SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;//enable clock to portb
	//making the pins gpio
	PORTB->PCR[DEBUG1_POS] &= ~PORT_PCR_MUX_MASK; 
	PORTB->PCR[DEBUG1_POS] |= PORT_PCR_MUX(1);
	PORTB->PCR[DEBUG2_POS] &= ~PORT_PCR_MUX_MASK; 
	PORTB->PCR[DEBUG2_POS] |= PORT_PCR_MUX(1);
	PORTB->PCR[DEBUG3_POS] &= ~PORT_PCR_MUX_MASK; 
	PORTB->PCR[DEBUG3_POS] |= PORT_PCR_MUX(1);

	//make the pins output
	PTB->PDDR |= MASK(DEBUG1_POS) | MASK(DEBUG2_POS) | MASK(DEBUG3_POS);
	//make the initial values 0
	PTB->PCOR |= MASK(DEBUG1_POS) | MASK(DEBUG2_POS) | MASK(DEBUG3_POS);
}

void Init_Config_Signals(void) {
		SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;// ENABLE CLOCK TO PORT E
		PORTE->PCR[CONFIG1_POS]=PORT_PCR_MUX(1)|PORT_PCR_PS_MASK|PORT_PCR_PE_MASK;
		PORTE->PCR[CONFIG2_POS]=PORT_PCR_MUX(1)|PORT_PCR_PS_MASK|PORT_PCR_PE_MASK;
		PORTE->PCR[CONFIG3_POS]=PORT_PCR_MUX(1)|PORT_PCR_PS_MASK|PORT_PCR_PE_MASK;
		PTE->PDDR &= ~MASK(CONFIG1_POS);
		PTE->PDDR &= ~MASK(CONFIG2_POS);
		PTE->PDDR &= ~MASK(CONFIG3_POS);
	
}
void Initialize_Interrupts(void) {
  	/* Configure NVIC */
		NVIC_SetPriority( I2C0_IRQn, 127);
  	NVIC_ClearPendingIRQ( I2C0_IRQn); 
  	I2C0->C1 |= I2C_C1_IICIE_MASK; /* enable device specific interrupt flag */
  	NVIC_EnableIRQ( I2C0_IRQn);
		__enable_irq();
}
/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/
int main (void) {
	int16_t prev_acc_X, prev_acc_Y, prev_acc_Z;
	int n;
	int dataready=0;
	Init_RGB_LEDs();
	Init_Debug_Signals();
	Init_Config_Signals();
	c3 =(PTE->PDIR & MASK(CONFIG1_POS));
	c4=(PTE->PDIR & MASK(CONFIG2_POS));
	c5=(PTE->PDIR & MASK(CONFIG3_POS));
	
	Control_RGB_LEDs(1, 1, 0);								/* yellow: starting up */
	i2c_init();																/* init i2c	*/
	Delay(200);
	
	if (!init_mma()) {												/* init mma peripheral */
		Control_RGB_LEDs(1, 0, 0);							/* Light red error LED */
		while (1)																/* not able to initialize mma */
			;
	}
	Control_RGB_LEDs(0, 0, 0);
if((!c3)){
	
	
	
Initialize_Interrupts();
}
//							
//#endif
	Delay(50);
	
	while (1) {
		Delay(50);
		//if(dataready==1){
			dataready=0;
		prev_acc_X = acc_X;
		prev_acc_Y = acc_Y;
		prev_acc_Z = acc_Z;
		//}
		
	read_full_xyz();

	if ((abs(prev_acc_X - acc_X) > ACC_SENSITIVITY) || 
			(abs(prev_acc_Y - acc_Y) > ACC_SENSITIVITY) || 
			(abs(prev_acc_Z - acc_Z) > ACC_SENSITIVITY)) {
			// Flash LEDs
				for (n=0; n<2; n++) {
					Control_RGB_LEDs(1, 1, 1);
					Delay(FLASH_DELAY);
					Control_RGB_LEDs(0, 0, 0);							
					Delay(FLASH_DELAY*2);		
			}
		}
		
	}
	
	}
//}
