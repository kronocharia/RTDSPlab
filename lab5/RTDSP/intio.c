/*************************************************************************************
			       DEPARTMENT OF ELECTRICAL AND ELECTRONIC ENGINEERING
					   		     IMPERIAL COLLEGE LONDON 

 				      EE 3.19: Real Time Digital Signal Processing
					       Dr Paul Mitcheson and Daniel Harvey

				        		  LAB 3: Interrupt I/O

 				            ********* I N T I O. C **********

  Demonstrates inputing and outputing data from the DSK's audio port using interrupts. 

 *************************************************************************************
 				Updated for use on 6713 DSK by Danny Harvey: May-Aug 2006
				Updated for CCS V4 Sept 10
 ************************************************************************************/
/*
 *	You should modify the code so that interrupts are used to service the 
 *  audio port.
 */
/**************************** Pre-processor statements ******************************/

#include <stdlib.h>
//  Included so program can make use of DSP/BIOS configuration tool.  
#include <stdio.h>
#include "dsp_bios_cfg.h"

/* The file dsk6713.h must be included in every program that uses the BSL.  This 
   example also includes dsk6713_aic23.h because it uses the 
   AIC23 codec module (audio interface). */
#include "dsk6713.h"
#include "dsk6713_aic23.h"

#include <math.h>

// Some functions to help with writing/reading the audio ports when using interrupts.
#include <helper_functions_ISR.h>

#include "coeff_jerry.txt"	//contains filter coefficients b[]

// PI defined here for use in your code 
#define PI 3.141592653589793


#define FILTER_CONST 16 //2RC/T_s
/******************************* Global declarations ********************************/

/* Audio port configuration settings: these values set registers in the AIC23 audio 
   interface to configure it. See TI doc SLWS106D 3-3 to 3-10 for more info. */
DSK6713_AIC23_Config Config = { \
			 /**********************************************************************/
			 /*   REGISTER	            FUNCTION			      SETTINGS         */ 
			 /**********************************************************************/\
    0x0017,  /* 0 LEFTINVOL  Left line input channel volume  0dB                   */\
    0x0017,  /* 1 RIGHTINVOL Right line input channel volume 0dB                   */\
    0x01f9,  /* 2 LEFTHPVOL  Left channel headphone volume   0dB                   */\
    0x01f9,  /* 3 RIGHTHPVOL Right channel headphone volume  0dB                   */\
    0x0011,  /* 4 ANAPATH    Analog audio path control       DAC on, Mic boost 20dB*/\
    0x0000,  /* 5 DIGPATH    Digital audio path control      All Filters off       */\
    0x0000,  /* 6 DPOWERDOWN Power down control              All Hardware on       */\
    0x0043,  /* 7 DIGIF      Digital audio interface format  16 bit                */\
    0x008d,  /* 8 SAMPLERATE Sample rate control             8 KHZ                 */\
    0x0001   /* 9 DIGACT     Digital interface activation    On                    */\
			 /**********************************************************************/
};


// Codec handle:- a variable used to identify audio interface  
DSK6713_AIC23_CodecHandle H_Codec;

 /******************************* Function prototypes ********************************/
void init_hardware(void);     
void init_HWI(void);      		//interrupt settings
void ISR_AIC(void);        		//interrupt function     

double lowpassRCFilter(double);
double iirBPDirectForm2(double);
double iirBPTransposed(double);
//*************************************Global Vars***********************************/
const int sampling_freq = 8000;
double circBuffer[N] = {0};
//double circBuffer[N] = {0,0,0,0,0};
int writePtr = N - 1;	     //write pointer for circular buffer
double transBuffer[N-1] = {0};	//used in transposed iir filter
//double transBuffer[N-1] = {0,0,0,0};

 
double previousSample = 0;
double previousOutput = 0;		//for single pole filter lab5
const float RC= 0.001;		//constant for single pole filter
//const double filterConstant =2*RC*sampling_freq;
/********************************** Main routine ************************************/
void main(){
	int j;
	for (j=0; j < N-1; j++){
		transBuffer[j] = 0;
		circBuffer[j] = 0;
	}
	circBuffer[j] = 0;
	// initialize board and the audio port
  	init_hardware();

  	/* initialize hardware interrupts */
  	init_HWI();
  	
 	 /* loop indefinitely, waiting for interrupts */  					
  	while(1) {

  	};
  
}
        
/********************************** init_hardware() **********************************/  
void init_hardware()
{
    // Initialize the board support library, must be called first 
    DSK6713_init();
    
    // Start the AIC23 codec using the settings defined above in config 
    H_Codec = DSK6713_AIC23_openCodec(0, &Config);

	/* Function below sets the number of bits in word used by MSBSP (serial port) for 
	receives from AIC23 (audio port). We are using a 32 bit packet containing two 
	16 bit numbers hence 32BIT is set for  receive */
	MCBSP_FSETS(RCR1, RWDLEN1, 32BIT);	

	/* Configures interrupt to activate on each consecutive available 32 bits 
	from Audio port hence an interrupt is generated for each L & R sample pair */	
	MCBSP_FSETS(SPCR1, RINTM, FRM);

	/* These commands do the same thing as above but applied to data transfers to  
	the audio port */
	MCBSP_FSETS(XCR1, XWDLEN1, 32BIT);	
	MCBSP_FSETS(SPCR1, XINTM, FRM);	
	

}

/********************************** init_HWI() **************************************/  
void init_HWI(void)
{
	IRQ_globalDisable();			// Globally disables interrupts before changes are made to interrupt configurations
	IRQ_nmiEnable();				// Enables the Non Maskable Interupts (used by the debugger)
	IRQ_map(IRQ_EVT_XINT1,4);		// Maps an event to a physical interrupt, here maps RINT1 to priority 4
	IRQ_enable(IRQ_EVT_XINT1);		// Enables the event symbol IRQ_EVT_RINT1 representing the event generating the interrupt
	IRQ_globalEnable();				// Globally enables interrupts as we're done config'ing

} 

/******************** WRITE YOUR INTERRUPT SERVICE ROUTINE HERE***********************/  
void ISR_AIC(void){

	double samp;
	double out;
	samp = mono_read_16Bit();			//read sample from codec. reads L & R sample from audio port and creates a mono average. returns 16bit integer
	
//	out = lowpassRCFilter(samp);	
//	out = iirBPDirectForm2(samp);
	out = iirBPTransposed(samp);
	
	mono_write_16Bit((Int16)out);		//write out rectified value. 
}


double lowpassRCFilter(double samp){
	
	double output =0;
	
	//Time domain implementation of filter of first order lowpass RC filter
	output =  samp /17.0 + previousSample/17.0 - ((-15.0/17.0)*previousOutput);
	
	//storing previous values (global vars) to feedback into filter
	previousSample = samp;
	previousOutput = output;
	return output;
}

double iirBPDirectForm2(double samp){

	double sum = 0;
	int i;
/*	int readPtr = (writePtr+1)%N;

	circBuffer[writePtr] = samp;	//

	for (i=1; i<N; i++){
		circBuffer[writePtr] -= a[i]*circBuffer[readPtr];
		if(++readPtr == N)
			readPtr = 0;
	}
	
	if (--writePtr == -1)
		writePtr = N-1;
	
	for (i = 0; i < N; i++){
		//filter maths
        sum +=  b[i]*circBuffer[readPtr];
		if (++readPtr == N)
			readPtr = 0;
	}*/
	
	int readPtr = writePtr;
	circBuffer[writePtr]=0;		//prevents oldest value from being used in loop calculation
	
	for(i=1; i<N; i++){
		if (++readPtr == N)		//increment readPtr and wraps around
			readPtr = 0;
		circBuffer[writePtr] -= a[i]*circBuffer[readPtr];   //a coeff MAC
		sum+= b[i]*circBuffer[readPtr];						//b coeff MAC
	}
	circBuffer[writePtr] += samp;							//add input to sum
	sum+= b[0]*circBuffer[writePtr];						//do b0 multiplication and add to sum
	
	if (--writePtr == -1)	//decrement and wraps writePtr
		writePtr = N-1;
	return sum;
}

double iirBPTransposed(double samp){
	double sum = 0;
	int i;

	sum = samp*b[0]+transBuffer[0];
	
	for(i=1; i<N-1; i++){
		transBuffer[i-1] = transBuffer[i] + samp*b[i]-a[i]*sum;
	}
	transBuffer[N-2] = samp*b[N-1]-a[N-1]*sum;
	
	return sum;
}

