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

// math library (trig functions)
#include <math.h>

// Some functions to help with writing/reading the audio ports when using interrupts.
#include <helper_functions_ISR.h>

//// Some functions to help with configuring hardware
//#include "helper_functions_polling.h"

//#include "fir_coeff.txt"
#include "fir_87.txt" //contains filter coefficients b[]

// PI defined here for use in your code 
#define PI 3.141592653589793

//sine generation look up table size
#define SINE_TABLE_SIZE 256

//number of elements in delay buffer
#define N 87	//number of taps

//#define USECIRCULARBUFFER 
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
float sinegen(void);
void sine_init(void);
double non_cir_FIR(double);
double cir_FIR(double);
//*************************************Global Vars***********************************/
int sampling_freq = 8000;
float sine_freq = 1000.0;         
double table[SINE_TABLE_SIZE];
double index = 0;
//int N = sizeof(b);	//number of taps
double x[N]={0};			 //non circular delay buffer
double cirBuffer[N] = {0};	 //circular buffer
int writePtr = N - 1;	     //write pointer for circular buffer
/********************************** Main routine ************************************/
void main(){
 
	// initialize board and the audio port
  	init_hardware();
	sine_init();
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

	#ifdef USECIRCULARBUFFER
		out = cir_FIR(samp);			//FIR filter function with circular buffer
	#else
		out = non_cir_FIR(samp);		//FIR filter function with non-circular buffer
	#endif

	mono_write_16Bit((Int16)out);		//write out rectified value. 
	
	
	//mono_write_16Bit(samp);				//allpass to analyse board response
	
}



double non_cir_FIR(double samp){	//FIR filter function with non-circular buffer
	double sum = 0;
	int i;
	
	//array shuffling with convolution
	for(i = N-1; i>0; i--){
		x[i] = x[i-1];		//move data along buffer shifting up one index along array
		sum += x[i]*b[i];			//where b is the array of filter coefficients 
	}
	x[0] = samp;			//put new sample into buffer, first array element
	sum += x[0]*b[0];			//where b is the array of filter coefficients 

	/*
	//array shuffling
	for(i = N-1; i>0; i--){
		x[i] = x[i-1];		//move data along buffer shifting up one index along array
	}
	x[0] = samp;			//put new sample into buffer, first array element

	//convolution
	for(i=0; i<N; i++){
		sum += x[i]*b[i];			//where b is the array of filter coefficients 
	}                               //and x is already flipped   
	
	*/
	return sum;
}

double cir_FIR(double samp){	//FIR filter funcion with circular buffer
	
	double sum = 0;
	int i;
	int readPtr = writePtr;       //start reading from where we wrote last
	
	cirBuffer[writePtr] = samp;	  //put new sample into cir buffer
                                  //writePtr initialised to last element in the array  

	if(writePtr == 0)             //wraps the writePtr around the array achieving
		writePtr = N - 1;         //circular nature
	else
		writePtr--;               //otherwise decrements the pointer through the array
	
	//convolution
	for(i=0; i<N; i++){ 
		sum += cirBuffer[readPtr]*b[i];
		if(readPtr < N-1)		//buffer implementation 1
			readPtr++;
		else
			readPtr = 0;
		
		/*readPtr++;				//buffer implementation 2
		readPtr = readPtr%N;*/				
	}
	return sum;
}

float sinegen(void)
{
	// x is global float variable
	float jump;												 //gap to next sample in lookup table

 	jump = (SINE_TABLE_SIZE*sine_freq/sampling_freq); 	
 	index += jump;											//increment x by jump
 	
	while(index>255){										//wrap round lookup table
		index-=SINE_TABLE_SIZE;
	}
    return(table[(int)round(index)]);   
}

void sine_init(void){                                   //populates sine table
	int i;
	for(i=0; i<SINE_TABLE_SIZE; i++){
		table[i]=sin(i*2*PI/SINE_TABLE_SIZE);
	} 
}
