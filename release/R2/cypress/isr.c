#include <stdio.h>
#include "fx2.h"
#include "fx2regs.h"
#include "fx2sdly.h"
extern WriteByteS0();
extern	transmit();
xdata volatile unsigned char D2ON         _at_ 0x8800;
xdata volatile unsigned char D2OFF        _at_ 0x8000;
unsigned char dut;
static int w;


void ISR_USART0(void) interrupt 4
  { 
	if (RI)
		{  
			if((EP2468STAT & bmEP8EMPTY))   // check if EP8 is empty
	  			{  
			  		 RI=0;
					 EP8FIFOBUF [0] = SBUF0;// copies received data to SBUF0
					 EP8BCH = 0;    
			         SYNCDELAY;   
			         EP8BCL = 1; 
			         SYNCDELAY;  
				     dut=D2ON;
				     w^=1;	
					 if (w)
				       {
				           dut=D2OFF;
						} 
		      	}
	 	}
	   		if (TI)
				{	
					TI=0;
					transmit();
	    	   	}
   }
 
