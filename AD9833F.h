////Init AD9833 
// ad9833_init();

////Set frequency 0 to 10kHz
// ad9833_set_frequency(0,10000);
////Select frequency 0 as output
// ad9833_set_freq_out(0);

////Set output mode to Sine
// ad9833_set_mode(AD_SINE);
#ifndef AD9833_h
#define AD9833_h

#if (ARDUINO >= 100)
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

//name AD9833 waveform output modes
//Parameters of \ref ad9833_set_mode()
#define AD_OFF      0
#define AD_TRIANGLE 1
#define AD_SQUARE   2
#define AD_SINE     3

//Control bits p13
#define AD_B28     13
#define AD_HLB     12
#define AD_FSELECT 11
#define AD_PSELECT 10
#define AD_RESET   8
#define AD_SLEEP1  7
#define AD_SLEEP12 6
#define AD_OPBITEN 5
#define AD_DIV2    3
#define AD_MODE    1

//AD9833 register addresses
#define AD_FREQ0  (1<<14)
#define AD_FREQ1  (1<<15)
#define AD_PHASE0 (3<<14)
#define AD_PHASE1 ((3<<14)|(1<<13))

//name AD9833 calculation macros
#define AD_F_MCLK 25000000 ///<Clock speed of the ad9833 reference clock
#define AD_2POW28 268435456 ///<used in calculating output freq

//Macro that calculates the value for a ad9833 frequency register from a frequency
#define AD_FREQ_CALC(freq) (uint32_t)(((double)AD_2POW28/(double)AD_F_MCLK*freq)*4)

//Macro that calculates value for Timer1 output compare from a frequency
#define AD_MOD_FREQ_CALC(freq) (F_CPU/(64*(uint32_t)freq))

//Macro that calculates the value for a ad9833 phase register from a phase in degrees
#define AD_PHASE_CALC(phase_deg) (uint16_t)((512*phase_deg)/45)

//name Timer1 Macros
// #define TIMER_START() ICCR1B |=   (1<<CS11)|(1<<CS10)  ///< Macro for starting the Timer1
// #define TIMER_STOP()  ICCR1B &= ~((1<<CS11)|(1<<CS10)) ///< Macro for stopping the Timer1

//some functions could be done more efficiently, maybe as inline functions...

class AD9833
{
	public:		
		AD9833(uint8_t csPin);
		void begin();
		void init(void);

		void setMode(uint8_t mode);

		void setFrequency(uint8_t reg, double freq);
		double getFrequency(uint8_t reg);

		void setPhase(uint8_t reg, double phase);
		double getPhase(uint8_t reg);

		void setFreqOut(uint8_t freq_out);
		uint8_t getFreqOut(void);

		void setPhaseOut(uint8_t phase_out);
		uint8_t getPhaseOut(void);

		void setModFreq(uint16_t freq);
		void setModBytes(uint8_t num, uint8_t *bytes, uint8_t repeat);
	private:
		void send(uint16_t packet);
		//void ISR(TIMER1_COMPA_vect);
		uint8_t transfer(uint8_t data);
		uint8_t _cs;
};

//Struct that holds all the configuration it's initialized as a global variable
typedef struct 
{
	float    freq[2]; ///<Holds the frequencies of 
    float    phase[2];
    float    mod_freq;
    uint8_t  freq_out;
    uint8_t  phase_out;
    uint8_t  mode;
    uint16_t command_reg;
} 	ad9833_settings_t;
#endif