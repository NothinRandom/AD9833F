#include "AD9833F.h"

// SPI2X SPR1 SPR0 SCK Freq.  Actual SCK speed @ 16Mhz
// 0     0    0    fosc/4     4Mhz    
// 0     0    1    fosc/16    1Mhz
// 0     1    0    fosc/64    250Khz
// 0     1    1    fosc/128   125Khz
// 1     0    0    fosc/2     8Mhz
// 1     0    1    fosc/8     2Mhz
// 1     1    0    fosc/32    500Khz
// 1     1    1    fosc/64    125Khz

ad9833_settings_t ad_settings; //<This is used to store all settings.

AD9833::AD9833(uint8_t csPin)
{
	_cs = csPin;
	pinMode(_cs, OUTPUT);
	digitalWrite(_cs, HIGH);
	pinMode(SCK, OUTPUT);
	pinMode(MOSI, OUTPUT);	
}

// Initializes the AD9833 and the relevant variables.
// Also initializes the Timer1 peripheral that is used for modulation timing.
void AD9833::init(void)
{
    //init FSYNC pin (aka Chip select)
    // ad_settings.command_reg |= (1<<AD_B28);
    // AD_FSYNC_DDR |= (1<<AD_FSYNC_BIT);
    // AD_FSYNC_HI();

    //init timer modulation
    // TCCR1B |= (1<<WGM12);  //timer in CTC mode
    // TCCR1B |= (1<<CS11)|(1<<CS10);//clockdiv 64
    // TIMSK1 |= (1<<OCIE1A);
    // OCR1A = 0xFFF0;

    //SPCR = (1<<SPE)|(1<<MSTR)|(1<<CPOL); //SPCR = B010111000;
    //SPCR     = (1<<SPE)|(1<<MSTR)|setup;	
	
    //some datasheet-proscribed delay here
    //delayMicroseconds(10);//_delay_us(10);

    //start as half-asleep
	//begin();
    digitalWrite(_cs, LOW);
    delayMicroseconds(5);//_delay_us(5);
    send((1<<AD_SLEEP12)|(1<<AD_RESET));
    ad_settings.command_reg |= (1<<AD_SLEEP12);
    delayMicroseconds(5);//_delay_us(5);
    digitalWrite(_cs, HIGH); 

    //set some nice default values
    setFrequency(0, 0);
    setFrequency(1, 0);
    setPhase(0, 0);
    setPhase(1, 0);
    setFreqOut(0);
    setPhaseOut(0);	
}

void AD9833::begin() //mode2
{
    SPCR = (1<<SPE)|(1<<MSTR)|(1<<CPOL)|(1<<SPR0); //only work with SPI mode2! //SPCR = B010111001;	
}
 
// Sets the ad9833 output waveform to the one given as a parameter.
// param mode possible values:
	// - AD_OFF
	// - AD_TRIANGLE
	// - AD_SQUARE
	// - AD_SINE
void AD9833::setMode(uint8_t mode)
{
    ad_settings.mode = mode;
    switch (mode)
	{
        case AD_OFF:
            ad_settings.command_reg |= (1<<AD_SLEEP12);
            ad_settings.command_reg |= (1<<AD_SLEEP1);
            break;
        case AD_TRIANGLE:
            ad_settings.command_reg &= ~(1<<AD_OPBITEN);
            ad_settings.command_reg |=  (1<<AD_MODE);
            ad_settings.command_reg &= ~(1<<AD_SLEEP12);
            //ad_settings.command_reg &= ~(1<<AD_SLEEP12);
            ad_settings.command_reg &= ~(1<<AD_SLEEP1);
            break;
        case AD_SQUARE:
            ad_settings.command_reg |=  (1<<AD_OPBITEN);
            ad_settings.command_reg &= ~(1<<AD_MODE);
            ad_settings.command_reg |=  (1<<AD_DIV2);
            ad_settings.command_reg &= ~(1<<AD_SLEEP12);
            ad_settings.command_reg &= ~(1<<AD_SLEEP1);
            break;
        case AD_SINE:
            ad_settings.command_reg &= ~(1<<AD_OPBITEN);
            ad_settings.command_reg &= ~(1<<AD_MODE);
            ad_settings.command_reg &= ~(1<<AD_SLEEP12);
            ad_settings.command_reg &= ~(1<<AD_SLEEP1);
            break;
    }

    digitalWrite(_cs, LOW);
    delayMicroseconds(5);//_delay_us(5);
    send(ad_settings.command_reg);
    delayMicroseconds(5);//_delay_us(5);
    digitalWrite(_cs, HIGH); 
}

// sets the desired ad9833 internal phase register to a value that produces the desired phase.
// param reg the desired phase register to be manipulated, either 0 or 1
// param phase the desired phase
void AD9833::setPhase(uint8_t reg, double phase)
{
    uint16_t reg_reg; //probably should be renamed...
    if (reg==1)
        reg_reg = AD_PHASE1;
    else
        reg_reg = AD_PHASE0;

    ad_settings.phase[reg] = phase;

    digitalWrite(_cs, LOW);
    delayMicroseconds(5);//_delay_us(5);
    send(reg_reg | AD_PHASE_CALC(ad_settings.phase[reg]));
    delayMicroseconds(5);//_delay_us(5);
    digitalWrite(_cs, HIGH); 
}

// returns the phase of the selected register
// param reg the register of which value we want to get
// return the phase of the selected register
double AD9833::getPhase(uint8_t reg)
{
    return ad_settings.phase[reg];
}

// Selects which frequency register is used to generate the output.
// Also used to select FSK.
// param phase_out possible values:
	// - 0 = use phase register 0
	// - 1 = use phase register 1
	// - 2 = PSK
void AD9833::setFreqOut(uint8_t freq_out)
{
    ad_settings.freq_out = freq_out;
    switch (freq_out){
        case 0:
            ad_settings.command_reg &= ~(1<<AD_FSELECT);
            break;
        case 1:
            ad_settings.command_reg |= (1<<AD_FSELECT);
            break;
        case 2:
            //TODO
            break;
    }

    digitalWrite(_cs, LOW);
    delayMicroseconds(5);//_delay_us(5);
    send(ad_settings.command_reg);
    delayMicroseconds(5);//_delay_us(5);
    digitalWrite(_cs, HIGH); 
}

//returns the previously set frequency output mode.
uint8_t AD9833::getFreqOut(void)
{
    return ad_settings.freq_out;
}

// Selects which phase register is used to generate the output
// Also used to select PSK
// param phase_out possible values:
	// - 0 = use phase register 0
	// - 1 = use phase register 1
	// - 2 = PSK
void AD9833::setPhaseOut(uint8_t phase_out)
{
    ad_settings.phase_out = phase_out;
    switch (phase_out)
	{
        case 0:
            ad_settings.command_reg &= ~(1<<AD_PSELECT);
            break;
        case 1:
            ad_settings.command_reg |= (1<<AD_PSELECT);
            break;
        case 2:
            //TODO
            break;
    }

    digitalWrite(_cs, LOW);
    delayMicroseconds(5);//_delay_us(5);
    send(ad_settings.command_reg);
    delayMicroseconds(5);//_delay_us(5);
    digitalWrite(_cs, HIGH); 
}

// returns the previously set phase output mode.
// return the previously set phase out mode
uint8_t AD9833::getPhaseOut(void)
{
    return ad_settings.phase_out;
}

// sets the desired ad9833 internal frequency register to a value that produces the desired frequency.
// param reg the desired frequency register to be manipulated, either 0 or 1
// param freq the desired frequency
void AD9833::setFrequency(uint8_t reg, double freq)
{
    uint32_t freq_reg;
    uint16_t reg_reg; //probably should be renamed...
    freq_reg = AD_FREQ_CALC(freq);
    ad_settings.freq[reg] = freq;

    if (reg==1)
        reg_reg = AD_FREQ1;
    else
        reg_reg = AD_FREQ0;

    digitalWrite(_cs, LOW);
    delayMicroseconds(5);//_delay_us(5);
    send((1<<AD_B28) | ad_settings.command_reg);
    send(reg_reg | (0x3FFF&(uint16_t)(freq_reg>>2 )));
    send(reg_reg | (0x3FFF&(uint16_t)(freq_reg>>16)));
    delayMicroseconds(5);//_delay_us(5);
    digitalWrite(_cs, HIGH);
}

// returns the frequency of the selected register
// param reg the register of which value we want to get
// return the frequency of the selected register
double AD9833::getFrequency(uint8_t reg)
{
    return ad_settings.freq[reg];
}

// sets the modulation frequency to the desired value
// param freq the desired modulation frequency
void AD9833::setModFreq(uint16_t freq)
{
    ad_settings.mod_freq = freq;
    OCR1A = AD_MOD_FREQ_CALC(freq);
}


// Sets the bytes to be modulated
// NOT IMPLEMENTED YET
// param num number of bytes to be sent
// param bytes pointer to an array of bytes to be sent
// param repeat should the sending be repeated
void AD9833::setModBytes(uint8_t num, uint8_t *bytes, uint8_t repeat)
{
    //TODO implements this thing
}

// /**
 // * Timer interrupt for handling modulation
 // */
// void AD9833::ISR(TIMER1_COMPA_vect)
// {
    // uint16_t check = ad_settings.command_reg;
    //TODO implement modulation of real signals
    // if (ad_settings.freq_out  == 2)
        // ad_settings.command_reg ^= ((uint16_t)1<<AD_FSELECT);
    // if (ad_settings.phase_out == 2)
        // ad_settings.command_reg ^= ((uint16_t)1<<AD_PSELECT);

    // if (check != ad_settings.command_reg)
	// {
        // digitalWrite(_cs, LOW);
        // _delay_us(5);
        // send(ad_settings.command_reg);
        // _delay_us(5);
        // AD_FSYNC_HI();
    // }
// }

uint8_t AD9833::transfer(uint8_t data)
{
	begin();
    SPDR = data;
    while(!(SPSR & (1<<SPIF)));
	return SPDR;
}


// a wrapper function for sending 16-bit SPI packets.
// param packet 16-bit value to be sent over SPI.
void AD9833::send(uint16_t packet)
{
    transfer((uint8_t)(packet>>8)); //spi_send_byte((uint8_t)(packet>>8));
    transfer((uint8_t)packet); //spi_send_byte((uint8_t)packet);
}