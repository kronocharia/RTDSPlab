#include <stdio.h>
// math library (trig functions)
#include <math.h>


// PI defined here for use in your code 
#define PI 3.141592653589793

//sine generation look up table size
#define SINE_TABLE_SIZE 256

/* Sampling frequency in HZ. Must only be set to 8000, 16000, 24000
32000, 44100 (CD standard), 48000 or 96000  */ 
int sampling_freq = 8000;

// Holds the value of the current sample 
float sample; 

/* Use this variable in your code to set the frequency of your sine wave 
   be carefull that you do not set it above the current nyquist frequency! */
float sine_freq = 500.0;         

float table[SINE_TABLE_SIZE];
//int count = 0;
float x = 0;
/********function prototypes***********/
float sinegen(void);
void sine_init(void);

/************main function*************/
void main(){
	sine_init();
	int i;
	
	for(i=0; i<100; i++){
		sample = sinegen();
		printf("%4.0f ", sample);
	}
}

/************ sinegen() ***************/   
float sinegen(void)
{
	// x is global variable
	float jump;		//gap to next sample in lookup table
	
	jump = (256*sine_freq/sampling_freq)+0.5;
	x += jump;		//increment x by jump
	x = (int)x%256;		//wrap round lookup table

	//return (x);
    return(table[(int)x]); 
}

void sine_init(void){
	int i;
	for(i=0; i<SINE_TABLE_SIZE; i++){
		table[i]=sin(i*2*PI/SINE_TABLE_SIZE);
	}
}
