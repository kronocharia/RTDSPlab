 /*************************************************************************************
                   DEPARTMENT OF ELECTRICAL AND RAINBOW FUNTIME ENGINEERING
                                 IMPERIAL COLLEGE LONDON 

                      EE 3.19: Real Time Digital Signal Processing
                           Dr Paul Mitcheson and Daniel Harvey

                                 PROJECT: Frame Processing

                            ********* ENHANCE. C **********
                             Shell for speech enhancement 

        Demonstrates overlap-add frame processing (interrupt driven) on the DSK. 

 *************************************************************************************
                             By Danny Harvey: 21 July 2006
                             Updated for use on CCS v4 Sept 2010
 ************************************************************************************/

/**************************** Pre-processor statements ******************************/
//  library required when using calloc
#include <stdlib.h>
//  Included so program can make use of DSP/BIOS configuration tool.  
#include "dsp_bios_cfg.h"

/* The file dsk6713.h must be included in every program that uses the BSL.  This 
   example also includes dsk6713_aic23.h because it uses the 
   AIC23 codec module (audio interface). */
#include "dsk6713.h"
#include "dsk6713_aic23.h"

// math library (trig functions)
#include <math.h>

/* Some functions to help with Complex algebra and FFT. */
#include "cmplx.h"      
#include "fft_functions.h"  

// Some functions to help with writing/reading the audio ports when using interrupts.
#include <helper_functions_ISR.h>

#define WINCONST 0.85185            /* 0.46/0.54 for Hamming window */
#define FSAMP 8000.0                /* sample frequency, ensure this matches Config for AIC */
#define FFTLEN 256                  /* fft length = frame length 256/8000 = 32 ms*/
#define NFREQ (1+FFTLEN/2)          /* number of frequency bins from a real FFT */
#define OVERSAMP 4                  /* oversampling ratio (2 or 4) */  
#define FRAMEINC (FFTLEN/OVERSAMP)  /* Frame increment */
#define CIRCBUFFLEN (FFTLEN+FRAMEINC)   /* length of I/O buffers */
#define NBUFFLEN 4                  /* number of noise minimum buffers */
#define FRAMESLIMIT 80  /* used for calculating the index of current noise buffer */

#define OUTGAIN 16000.0             /* Output gain for DAC */
#define INGAIN  (1.0/16000.0)       /* Input gain for ADC  */
// PI defined here for use in your code 
#define PI 3.141592653589793
#define TFRAME (FRAMEINC/FSAMP)       /* time between calculation of each frame */
//#define MIN_NR 0.25          //min noise reduction value (lambda)
//#define OVER_EST_FACTOR 2   //over-estimation factor for alpha --for enhancement 6 
#define ALPHA 4.0          //oversubraction factor for noise
#define TAU 0.05            //time constant for input low pass filter <0ms> -- for enhancement 1
 
/******************************* Global declarations ********************************/

/* Audio port configuration settings: these values set registers in the AIC23 audio 
   interface to configure it. See TI doc SLWS106D 3-3 to 3-10 for more info. */
DSK6713_AIC23_Config Config = { \
             /**********************************************************************/
             /*   REGISTER              FUNCTION                  SETTINGS         */ 
             /**********************************************************************/\
    0x0017,  /* 0 LEFTINVOL  Left line input channel volume  0dB                   */\
    0x0017,  /* 1 RIGHTINVOL Right line input channel volume 0dB                   */\
    0x01f9,  /* 2 LEFTHPVOL  Left channel headphone volume   0dB                   */\
    0x01f9,  /* 3 RIGHTHPVOL Right channel headphone volume  0dB                   */\
    0x0011,  /* 4 ANAPATH    Analog audio path control       DAC on, Mic boost 20dB*/\
    0x0000,  /* 5 DIGPATH    Digital audio path control      All Filters off       */\
    0x0000,  /* 6 DPOWERDOWN Power down control              All Hardware on       */\
    0x0043,  /* 7 DIGIF      Digital audio interface format  16 bit                */\
    0x008d,  /* 8 SAMPLERATE Sample rate control        8 KHZ-ensure matches FSAMP */\
    0x0001   /* 9 DIGACT     Digital interface activation    On                    */\
             /**********************************************************************/
};

// Codec handle:- a variable used to identify audio interface  
DSK6713_AIC23_CodecHandle H_Codec;

float *inBuffer, *outBuffer;        /* Input/output circular buffers */
float *inFrame, *outFrame;          /* Input and output frames */
float *inWin, *outWin;              /* Input and output windows */

complex *fft_input;                 /* FFT of input */
complex *fft_output;                /* FFT of output */
complex **recentThreeFrames;         // Last three frames

//float *fftMag_noise;                /* MAG FFT of Noise */
float *fft_gain;                    /* gain we multiply with the FFT of input */
float inGain, outGain;              /* ADC and DAC gains */ 
float cpuFrac;                      /* Fraction of CPU time used */
float alpha = ALPHA;                /* Oversubtraction factor */
float minNR;               			/* Minimum noise reduction */
volatile int io_ptr=0;              /* Input/ouput pointer for circular buffers */
volatile int frame_ptr=0;           /* Frame pointer */

/*
typedef struct  {
    float* fSpectrum;           //stores frequency spectrum of noise
    float power;            //magnitude of the whole freq spectrum
} noiseBufferStruct;
*/
typedef struct  {
    complex s[3];                      
} Last3Freq;
Last3Freq autoRecent3Frames[FFTLEN];


//noiseBufferStruct* noiseBuffers;  /* ptr to array of noise buffer struct */
int nIndex = 0;                         /*current noise buffer index */
int numFrames = 0;                      /* counter for number of frames passed, used to calculate noise buffer index */
int prevNIndex = 0;  

//*********** intermediate buffer ***************//
float *lowPass;           //low passed filtered inpt estimate for current frame (P_t(w))

//********** Enhancement parameters *************//
float inputLPF_k;   //low pass filter time constant pole in z plane for input frames (k)
float noiseLPF_k;   //low pass filter time constant pole in z plane for noise estimate
float inputLPF4_k;   //alternate for enhancement 4
float noiseLPF4_k;   
float tau4 = 0.2; //tau for enhance 4
float MIN_NR = 0.08; //lambda down here to change in watch
//********* non-circular noise buffers ********//
float* M1;      //current noise buffer
float* M2;
float* M3;
float* M4;
float* minM;    //stores minimum accross all M

//********** Enhancement switches ************/
volatile short enhancement1or2 = 2;     //1->enhancement1, 2->enhancement2, else->default
volatile short enhancement4or5 = 4;     //4->enhancement4, 5->enhancement5, else->default
volatile short enhancement3 = 1;        //1->enhancement3, else->enhancement3 off
volatile short enhancement4 = 2;        //cases 0-4
volatile short enhancement5 = 0;        //cases 0-4
volatile short enhancement6 = 0;
volatile short enhancement8 = 0;
volatile short original = 0;            //overwrites everything, play unprocessed input


 /******************************* Function prototypes *******************************/
void init_hardware(void);           /* Initialize codec */ 
void init_HWI(void);                /* Initialize hardware interrupts */
void ISR_AIC(void);                 /* Interrupt service routine for codec */
void process_frame(void);           /* Frame processing routine */
//int getMinNoiseBufferIndex(noiseBufferStruct*, int);  /*find min noise power buffer index*/
float maxOfFloats(float,float);     
float noiseRatioThresh = 0.5;
float tau = TAU;
float pOverX;
float noiseRatio;
float prevNoiseRatio;
float pureNoiseRatio;
/********************************** Main routine ************************************/
void main()
{

    int k; // used in various for loops
    int ii; //used in 2d array init
    //int tau = TAU;  //time constant for input low pass filter --enhancement 1
    
/*  Initialize and zero fill arrays */  

    inBuffer    = (float *) calloc(CIRCBUFFLEN, sizeof(float)); /* Input array */
    outBuffer   = (float *) calloc(CIRCBUFFLEN, sizeof(float)); /* Output array */
    inFrame     = (float *) calloc(FFTLEN, sizeof(float));  /* Array for processing*/
    outFrame    = (float *) calloc(FFTLEN, sizeof(float));  /* Array for processing*/
    inWin       = (float *) calloc(FFTLEN, sizeof(float));  /* Input window */
    outWin      = (float *) calloc(FFTLEN, sizeof(float));  /* Output window */
    
    fft_input = (complex*)calloc(FFTLEN, sizeof(complex));  //Freq components of input
    lowPass = (float*)calloc(FFTLEN, sizeof(float));      //Low passed of freq components of input
    
  //  fftMag_noise = (float*)calloc(FFTLEN, sizeof(float));   //Mag of Freq components of best noise guess
    fft_gain = (float*)calloc(FFTLEN, sizeof(float));       //What we multiply input with to subtract noise
    
    /* intialise 2D array */
    /*noiseBuffers = (noiseBufferStruct*)calloc(NBUFFLEN, sizeof(noiseBufferStruct*));  //an array of noiseBuffer struct

    for(ii = 0; ii<NBUFFLEN; ii++){
        noiseBuffers[ii].fSpectrum = (float*) calloc(NFREQ, sizeof(float));
        noiseBuffers[ii].power = 0;
    }*/
    recentThreeFrames = (complex**)calloc(FFTLEN, sizeof(complex*));  //an array of compelex struct
    for(ii = 0; ii<3; ii++){
        recentThreeFrames[ii] = (complex*) calloc(FFTLEN, sizeof(complex));
    }

    inputLPF_k = exp(-TFRAME/tau); 
    noiseLPF_k = exp(-TFRAME/tau);
    inputLPF4_k = exp(-TFRAME/tau4); 
    noiseLPF4_k = exp(-TFRAME/tau4);
    M1  = (float *) calloc(FFTLEN, sizeof(float));
    M2  = (float *) calloc(FFTLEN, sizeof(float));
    M3  = (float *) calloc(FFTLEN, sizeof(float));
    M4  = (float *) calloc(FFTLEN, sizeof(float));
    minM= (float *) calloc(FFTLEN, sizeof(float));
    /*  noiseBuffer = calloc(OVERSAMP, sizeof(float *));
    for(ii = 0; ii < NFREQ; ii++) { 
        noiseBuffer[ii] = calloc(NFREQ, sizeof(float));
    }   */   
 
    /* initialize board and the audio port */
    init_hardware();
  
    /* initialize hardware interrupts */
    init_HWI();    
  
/* initialize algorithm constants */  
                       
    for (k=0;k<FFTLEN;k++){                           
        inWin[k] = sqrt((1.0-WINCONST*cos(PI*(2*k+1)/FFTLEN))/OVERSAMP);
        outWin[k] = inWin[k]; 
    } 
    inGain=INGAIN;
    outGain=OUTGAIN;        

                            
    /* main loop, wait for interrupt */  
    while(1)    process_frame();
}
    
/********************************** init_hardware() *********************************/  
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

    /* These commands do the same thing as above but applied to data transfers to the 
    audio port */
    MCBSP_FSETS(XCR1, XWDLEN1, 32BIT);  
    MCBSP_FSETS(SPCR1, XINTM, FRM); 
    

}
/********************************** init_HWI() **************************************/ 
void init_HWI(void)
{
    IRQ_globalDisable();            // Globally disables interrupts
    IRQ_nmiEnable();                // Enables the NMI interrupt (used by the debugger)
    IRQ_map(IRQ_EVT_RINT1,4);       // Maps an event to a physical interrupt
    IRQ_enable(IRQ_EVT_RINT1);      // Enables the event
    IRQ_globalEnable();             // Globally enables interrupts

}
        
/******************************** process_frame() ***********************************/  
void process_frame(void)
{
    int k, m; 
    int io_ptr0;
    float cabsInput;
    float cabs2Input;
    float *tmpM;
    float tmpMinNoise;
    float tmpResult;
    //static float noiseRatio;
    //static float prevNoiseRatio;
    
    complex tmpMin3Frames;
    
    /* work out fraction of available CPU time used by algorithm */    
    cpuFrac = ((float) (io_ptr & (FRAMEINC - 1)))/FRAMEINC;  
        
    /* wait until io_ptr is at the start of the current frame */    
    while((io_ptr/FRAMEINC) != frame_ptr); 
    
    /* then increment the framecount (wrapping if required) */ 
    if (++frame_ptr >= (CIRCBUFFLEN/FRAMEINC)) frame_ptr=0;
    
    /* save a pointer to the position in the I/O buffers (inBuffer/outBuffer) where the 
    data should be read (inBuffer) and saved (outBuffer) for the purpose of processing */
    io_ptr0=frame_ptr * FRAMEINC;
    
    /* copy input data from inBuffer into inFrame (starting from the pointer position) */ 
     
    m=io_ptr0; //start from current io_ptr
    
    for (k=0;k<FFTLEN;k++)
    {                           
        inFrame[k] = inBuffer[m] * inWin[k];  //windows the input samples
        if (++m >= CIRCBUFFLEN) m=0; /* wrap if required */
    } 
    
    /************************* DO PROCESSING OF FRAME  HERE **************************/
    
    
    /* please add your code, at the moment the code simply copies the input to the 
    ouptut with no processing */    
    for (k=0;k<FFTLEN;k++){             //copy inFrame to working set for FFT
        fft_input[k].r = inFrame[k]; 
        fft_input[k].i = 0;
    }
                                      
    fft(FFTLEN,fft_input);
  
    //************** shuffle noise buffers *******************//
    if(++numFrames >= FRAMESLIMIT){
        numFrames = 0;
        tmpM = M4;
        M4 = M3;
        M3 = M2;
        M2 = M1;
        M1 = tmpM;
    }
    
    //*************** processing *****************************//
    for(k=0; k<NFREQ; k++){
        cabsInput = cabs(fft_input[k]); //|X(w)|
        cabs2Input = cabsInput*cabsInput; //|X(w)|**2

    
		//************** ENHANCEMENT 1/2 *****************//
        if (enhancement1or2 == 1)
            lowPass[k] = (1-inputLPF_k)*cabsInput +inputLPF_k*lowPass[k]; //lowpass formula from notes

        else if (enhancement1or2 == 2)
            lowPass[k] = sqrt((1-inputLPF_k)*cabs2Input+ inputLPF_k*lowPass[k]*lowPass[k]); //lowpass on power

        else
            lowPass[k] = cabsInput;  //doesnt LPF at all

        if(numFrames == 0){  		// if first frame after swapping noise buffer pointers
            M1[k] = lowPass[k];     //then youre *hapless laughter* so youre filling the entire M1 with the whole input frame. lowPass[k] is either the whole input frame OR lowpassed version based off the enhancement case
                                	//min noise is just the frame

            tmpMinNoise = M4[k];        //this finds the min noise out of the the buffers M4-M2 + a tiny bit M1 cos its still doing it?!! for the particular frequency k
            if(tmpMinNoise > M3[k])
                tmpMinNoise = M3[k];
            if(tmpMinNoise > M2[k])
                tmpMinNoise = M2[k];
            if(tmpMinNoise > M1[k])
                tmpMinNoise = M1[k];
        }
        else if (lowPass[k] < M1[k]){     //else if not the first frame then and the the current noise is smaller than whats in M1
            M1[k] = lowPass[k];               //replace with the smaller one
            if(minM[k] > M1[k])
                tmpMinNoise = M1[k];    //so if this is smaller than the current minimum, then udpate
            else 
                tmpMinNoise = minM[k];  //tmpMinNoise is for enhancement 3 - single minimum(for this k because of the for loop but is declared as a float)
        }       
        else
            tmpMinNoise = minM[k];
        
       
        //************* ENHANCEMENT 3 *************//
        if(enhancement3 == 1)       //low pass filter noise estimate
            minM[k] = (1- noiseLPF_k)*tmpMinNoise + noiseLPF_k*minM[k]; //lowpass formula from notes
        else
            minM[k] = tmpMinNoise;
        //************* ENHANCEMENT 6 *************// 
        if(enhancement6 == 1) 

            
            if (pureNoiseRatio < 0.1) 
                alpha = 1;
            else if (pureNoiseRatio > 0.562)
                alpha = 5;

            else
                alpha = 8.658*pureNoiseRatio + 0.134;
        

        prevNoiseRatio = noiseRatio;
   		pureNoiseRatio = (minM[k]/cabsInput) ;
        noiseRatio = (alpha*pureNoiseRatio); //|N(w)|/|X(w)|
        //************* ENHANCEMENT 4 *************//
   
        
        if (enhancement4or5 == 4) {
            switch(enhancement4){
                case 1:      
                    fft_gain[k] = 1.0 - noiseRatio;      
                    minNR = MIN_NR*(alpha*(minM[k])/cabsInput);
                    break;
                case 2:
					pOverX = (lowPass[k]/cabsInput);
                    fft_gain[k] = 1.0 - noiseRatio;
                    minNR = MIN_NR*pOverX;
                    break;
                case 3:
                    fft_gain[k] = 1.0 - (alpha*(minM[k])/lowPass[k]); 
                    minNR = MIN_NR*(alpha*(minM[k])/lowPass[k]);
                    break;
                case 4:
                    fft_gain[k] = 1.0 - (alpha*(minM[k])/lowPass[k]); 
                    minNR = MIN_NR;
                    break;
                default:
                    fft_gain[k] = 1.0 - (noiseRatio);    
                    minNR = MIN_NR;
                    break;
            }
        }
        

        else if (enhancement4or5 == 5){
       
             switch(enhancement5){
                case 1:
                  //  tmpResult = ((alpha*alpha*(minM[k])*(minM[k]))/cabs2Input);
                    tmpResult = pureNoiseRatio*pureNoiseRatio;
                    fft_gain[k] = sqrt(1.0 - tmpResult);
                    minNR = MIN_NR*tmpResult;
                    break;
                case 2:
                    //fft_gain[k] = sqrt(1.0 - ((alpha*alpha*(minM[k])*(minM[k]))/cabs2Input));
                    fft_gain[k] = sqrt(1.0 - ((lowPass[k]/cabsInput)*(lowPass[k]/cabsInput)) );
                    minNR = MIN_NR*(lowPass[k]/cabsInput);
                    break;
                case 3:
                    tmpResult = ((minM[k]*minM[k])/(lowPass[k]*lowPass[k]));
                    fft_gain[k] = sqrt(1.0 - tmpResult); 
                    minNR = MIN_NR*tmpResult;
                    break;
                case 4:
                    fft_gain[k] = sqrt(1.0 - ((minM[k]*minM[k])/(lowPass[k]*lowPass[k]))); 
                    minNR = MIN_NR;
                    break;
                default:
                    fft_gain[k] = sqrt(1.0 - (minM[k]*minM[k])/cabs2Input);    
                    minNR = MIN_NR;
                    break;
            }
        }
        
        else {
            fft_gain[k] = 1.0 - noiseRatio;    
            minNR = MIN_NR;
        }
        
        //vanilla g(w) which is the gain for each frequency
        fft_gain[k] = maxOfFloats(minNR,fft_gain[k]);           //where minNR is lambda in note, 
        
        //************* ENHANCEMENT 8 ***************//
        if (enhancement8==1) {
 
            autoRecent3Frames[k].s[2] = autoRecent3Frames[k].s[1];
            autoRecent3Frames[k].s[1] = autoRecent3Frames[k].s[0];
            autoRecent3Frames[k].s[0] = rmul(fft_gain[k], fft_input[k]);     

	        if ( prevNoiseRatio > noiseRatioThresh) {	

                tmpMin3Frames = autoRecent3Frames[k].s[0];
                if (cabs(tmpMin3Frames) > cabs(autoRecent3Frames[k].s[1]))
                    tmpMin3Frames = autoRecent3Frames[k].s[1];
                if (cabs(tmpMin3Frames) > cabs(autoRecent3Frames[k].s[2]))
                    tmpMin3Frames = autoRecent3Frames[k].s[2];

	            fft_input[k] = tmpMin3Frames;
	        }
	        else {

                fft_input[k] = autoRecent3Frames[k].s[1];
	        }
	    }
	    //enhancement 8 off
		else{
	        fft_input[k] = rmul(fft_gain[k], fft_input[k]);
		}
		
		if (k!=0 || k!=NFREQ-1)
			fft_input[FFTLEN-k] = conjg(fft_input[k]);		//symmetric copy
    }//end of giantfor loop iterating over frequencies


    ifft(FFTLEN,fft_input);  //TODO ask about the fft and why its half, and what do we do with the other empty array size.

    if (original == 1) {
        for (k=0;k<FFTLEN;k++) //loop over the current frame                           
            outFrame[k] = inFrame[k];// copy input straight into output 
    }
    else{ 
	    for (k=0;k<FFTLEN;k++){ //loop over the current frame                           
	            outFrame[k] = (fft_input[k].r);// copy input straight into output  
	    } 
    }
    /********************************************************************************/
    
    /* multiply outFrame by output window and overlap-add into output buffer */  
                           
    m=io_ptr0;//start from current io_ptr
    
    for (k=0;k<(FFTLEN-FRAMEINC);k++) 
    {                                           /* this loop adds into outBuffer */                       
        outBuffer[m] = outBuffer[m]+outFrame[k]*outWin[k];   
        if (++m >= CIRCBUFFLEN) m=0; /* wrap if required */
    }         
    for (;k<FFTLEN;k++) 
    {                           
        outBuffer[m] = outFrame[k]*outWin[k];   /* this loop over-writes outBuffer */        
        m++;
    }                                      
}        
/*************************** INTERRUPT SERVICE ROUTINE  *****************************/

// Map this to the appropriate interrupt in the CDB file
   
void ISR_AIC(void)
{       
    short sample;
    /* Read and write the ADC and DAC using inBuffer and outBuffer */
    
    sample = mono_read_16Bit();
    inBuffer[io_ptr] = ((float)sample)*inGain;
        /* write new output data */
    mono_write_16Bit((int)(outBuffer[io_ptr]*outGain)); 
    
    /* update io_ptr and check for buffer wraparound */    
    
    if (++io_ptr >= CIRCBUFFLEN) io_ptr=0;
}

/************************************************************************************/
/*
int getMinNoiseBufferIndex(noiseBufferStruct* noiseBuffers1, int currentIndex){ //gives the mininum mag of the 4 buffers
    int i;
    int minIndex = -1;
    float minPower = -1;
    
    for(i=0; i<NBUFFLEN; i++){
        if(i != currentIndex){
            if(minPower < 0 || noiseBuffers1[i].power < minPower)
                minPower = noiseBuffers1[i].power;
                minIndex = i;
        }
    }
    
    return minIndex;
}
*/
float maxOfFloats(float arg1, float arg2){ //returns arg1 if they are the same
    float result;
    if (arg1 >= arg2)
        result = arg1;
    else
        result = arg2;
    return result;
}
