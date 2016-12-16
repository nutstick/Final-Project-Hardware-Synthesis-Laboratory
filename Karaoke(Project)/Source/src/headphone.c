/**
 * @file         headphone.c
 * @version      1.0
 * @date         2015
 * @author       Christoph Lauer
 * @compiler     armcc
 * @copyright    Christoph Lauer engineering
 */
 
#include <main.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <misc.h>
#include <dsp.h>

static uint8_t volume = 100;
extern volatile uint8_t LED_Toggle;
extern volatile int user_mode;

/*
 * We have three buffers: two output buffers used in a ping-pong arrangement, and an input
 * (microphone) circular buffer. Because the output buffers are written directly to the I2S
 * interface with DMA, they must be stereo. The microphone buffer is mono and its duration
 * is exactly 3 times the length of one of the ping-pong buffers. The idea is that during
 * normal operation, the microphone buffer will vary between about 1/3 full and 2/3 full
 * (with a 1/3 buffer margin on either side).
 */
#define SAMPLE_RATE 16000        // sampling rate
#define OUT_BUFFER_SAMPLES 1024  // number of samples per output ping-pong buffer
#define MIC_BUFFER_SAMPLES (OUT_BUFFER_SAMPLES * 3 / 2)
#define PI 3.14159265f
#define LENGTH 12
#define MUSIC_LENGTH 25
const uint8_t music[] = {	1, 1, 1,3, 3, 3,	5, 5, 5, 0,	5, 5, 5,	3, 3, 3,	1, 1, 1,	8, 8, 8,	8, 8, 8,};

#define MUSIC_LENGTH2 156
const uint8_t music2[] = {
  13,13,13,13,10,10,10,13,15,15,17,17,15,15,17,17,8,8,8,8,8,10,12,12,10,10,10,10,10,10,1,1,3,3,1,5,3,3,5,5,3,3,1,1,8,8,8,8,13,13,15,17,17,20,20,13,13,15,15,17,17,20,20,10,10,10,10,13,13,12,12,12,12,13,15,17,17,20,20,22,22,20,20,17,17,8,8,13,13,13,13,10,10,17,17,15,15,15,15,13,13,15,17,17,20,20,22,22,20,20,17,17,15,15,10,10,10,10,13,13,8,8,8,8,17,17,20,17,17,20,20,22,22,20,20,17,17,15,15,13,13,13,13,10,10,13,13,15,15,15,15,17,15,13,13,13
};

#define MUSIC_LENGTH3 150
const uint8_t music3[] = {2,6,6,2,5,5,4,3,3,3,3,8,10,10,10,10,10,10,10,10,2,6,6,2,3,3,4,5,5,5,5,4,11,11,11,11,2,10,10,6,8,8,8,8,8,8,2,6,6,2,8,8,2,6,6,2,8,8,4,10,10,10,10,2,6,6,2,5,5,4,3,3,3,3,4,8,8,8,8,2,6,6,4,5,5,5,5,1,3,1,6,4,6,6,6,6,4,3,3,3,3,4,10,10,10,10,4,6,6,6,6,4,5,5,5,5,4,15,15,15,15,4,13,13,13,13,2,11,11,1,10,1,8,16,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10};
// These functions will have different instances depending on the global function selected below
static void fill_init (void);
static void fill_buffer (int16_t *buffer, int num_samples);

static int16_t buff0 [OUT_BUFFER_SAMPLES], buff1 [OUT_BUFFER_SAMPLES], micbuff [MIC_BUFFER_SAMPLES];
static volatile uint16_t mic_head, mic_tail;    // head and tail indices to mic buffer
static volatile uint8_t next_buff;              // next output buffer to write

#define NOTE_KEY 26
double freq[] = {
	0, 130.82, 138.59, 146.83, 155.56, 164.81, 174.61, 185, 196, 207.65, 220, 233.08, 246.94,
	261.63, 277.18, 293.66, 311.13, 329.63, 349.23, 369.99, 392, 415.3, 440, 466.16, 493.88,
	523.24
};
double scale = 80.;
uint16_t wav[40][1024];
double sampleSize[40];
uint8_t currentNote, currentKey, waveCount, currentIndex;
int mode = 0, cC = 0;

Accelerometer accelerometer;

void WaveRecorderCallback (int16_t *buffer, int num_samples)
{
    static int clip_timer;
    int clip = 0, i;

    for (i = 0; i < num_samples; ++i) {
        int16_t sample = *buffer++;
        if (sample >= 32700 || sample <= -32700)
            clip = 1;
        micbuff [mic_head + i] = sample;
    }    
    mic_head = (mic_head + num_samples >= MIC_BUFFER_SAMPLES) ? 0 : mic_head + num_samples;
    if (clip_timer) {
        if (!--clip_timer)
            STM_EVAL_LEDOff(LED5);
    }
    else if (clip)
        STM_EVAL_LEDOn(LED5);
    if (clip)
        clip_timer = 50;
}

void WavePlayBack(uint32_t AudioFreq)
{ 
  /* First, we start sampling internal microphone */
  WaveRecorderBeginSampling ();

  /* Initialize wave player (Codec, DMA, I2C) */
  WavePlayerInit(SAMPLE_RATE);
	
  /* Initialize the buffer filling function */
  fill_init ();

  /* Let the microphone data buffer get 2/3 full (which is 2 playback buffers) */
  while (mic_head < MIC_BUFFER_SAMPLES * 2 / 3);

  /* Fill the second playback buffer (the first will just be zeros to start) */
  fill_buffer (buff1, OUT_BUFFER_SAMPLES);
  
  /* Start audio playback on the first buffer (which is all zeros now) */
  Audio_MAL_Play((uint32_t)buff0, OUT_BUFFER_SAMPLES * 2);
  next_buff = 1; 

  /* LED Green Start toggling */
  LED_Toggle = LED_CTRL_GREEN_TOGGLE;
  
  /* This is the main loop of the program. We simply wait for a buffer to be exhausted
   * and then we refill it. The callback (which is triggered by DMA completion) actually
   * handles starting the next buffer playing, so we don't need to be worried about that
   * latency here. The functionality of the fill_buffer() function determines what it is
   * that we are doing (e.g., playing tones, echoing the mic, being a nervous dog, etc.)
   */

  while (1) {
    while (next_buff == 1);
    fill_buffer (buff0, OUT_BUFFER_SAMPLES);
    while (next_buff == 0);
    fill_buffer (buff1, OUT_BUFFER_SAMPLES);
		
		Accelerometer_GetData(&accelerometer);
		
		int x = accelerometer.x;
		if (0x9D <= accelerometer.y && accelerometer.y <= 0xA4) {
			//cC++;
      STM_EVAL_LEDOn(LED6);
			mode = 0;
		} else {
			//cC = 0;
      STM_EVAL_LEDOff(LED6);
			mode = 1;
		}
		/*
		if (cC >= 20) {
			cC = 0;
			mode = !mode;
		}
		*/
  }
}
 
int WavePlayerInit(uint32_t AudioFreq)
{ 
  /* Initialize I2S interface */  
  EVAL_AUDIO_SetAudioInterface(AUDIO_INTERFACE_I2S);
  
  /* Initialize the Audio codec and all related peripherals (I2S, I2C, IOExpander, IOs...) */  
  EVAL_AUDIO_Init(OUTPUT_DEVICE_AUTO, volume, AudioFreq );  
  
  return 0;
}

void EVAL_AUDIO_TransferComplete_CallBack(uint32_t pBuffer, uint32_t Size)
{
  if (next_buff == 0) {
    Audio_MAL_Play((uint32_t)buff0, OUT_BUFFER_SAMPLES * 2);
    next_buff = 1; 
  }
  else {
    Audio_MAL_Play((uint32_t)buff1, OUT_BUFFER_SAMPLES * 2);
    next_buff = 0; 
  }
}
 
void EVAL_AUDIO_HalfTransfer_CallBack(uint32_t pBuffer, uint32_t Size)
{  
}

void EVAL_AUDIO_Error_CallBack(void* pData)
{
  while (1) /* Stop the program with an infinite loop */
  {} /* could also generate a system reset to recover from the error */
}

uint16_t EVAL_AUDIO_GetSampleCallBack(void)
{
  return 0;
}

static void fill_init (void)
{
	int j, i;
	sampleSize[ 0 ] = 10;
	for (i = 1; i < NOTE_KEY; i++) {
		sampleSize[ i ] = round( (double)SAMPLE_RATE / freq[ i ] );
	}
  for (j = 1; j < NOTE_KEY; j++) {
    for (i = 0; i < sampleSize[j]; i++) {
      wav[j][i] = round( sin(2 * PI / sampleSize[j] * i) * 4000 );//32767;
    }
  }
	freq[0] = 1;
	currentNote = 0;
	currentKey = music2[ currentNote ];
	currentIndex = 0;
	waveCount = (int) freq[currentKey] / scale;
}

static void fill_buffer (int16_t *buffer, int num_samples)
{
	int count = num_samples / 2;
	
	dsp(micbuff + mic_tail, count);
	
	// make mono to stereo here
  while (count--) 
	{
		if (mode == 0) {
			*buffer++ = micbuff [mic_tail] + wav[ currentKey ][ currentIndex ];
			*buffer++ = micbuff [mic_tail] + wav[ currentKey ][ currentIndex ];
				
			mic_tail = (mic_tail + 1 >= MIC_BUFFER_SAMPLES) ? 0 : mic_tail + 1;
			currentIndex++;
			if (currentIndex >= (int) sampleSize[ currentKey ]){
				currentIndex = 0;
			}
		} else {
			*buffer++ = micbuff [mic_tail];
			*buffer++ = micbuff [mic_tail];
			mic_tail = (mic_tail + 1 >= MIC_BUFFER_SAMPLES) ? 0 : mic_tail + 1;
		}
    // *buffer++ = micbuff [mic_tail] + wav[ currentKey ][ i ];
    // *buffer++ = micbuff [mic_tail] + wav[ currentKey ][ i ];
    // mic_tail = (mic_tail + 1 >= MIC_BUFFER_SAMPLES) ? 0 : mic_tail + 1;
  }
	if (waveCount > 0)
		waveCount--;
	else {
		// temp = currentNote
		// if (currentIndex == 0) {
			currentNote = ( currentNote + 1 ) % MUSIC_LENGTH2;
			currentKey = music2[ currentNote ];
			waveCount = round( freq[currentKey] / scale );
		// }
	}
}