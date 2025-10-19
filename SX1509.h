/******************************************************************************
SparkFunSX1509.h
SparkFun SX1509 I/O Expander Library Header File
Jim Lindblom @ SparkFun Electronics
Original Creation Date: September 21, 2015
https://github.com/sparkfun/SparkFun_SX1509_Arduino_Library

Here you'll find the Arduino code used to interface with the SX1509 I2C
16 I/O expander. There are functions to take advantage of everything the
SX1509 provides - input/output setting, writing pins high/low, reading 
the input value of pins, LED driver utilities (blink, breath, pwm), and
keypad engine utilites.

Development environment specifics:
	IDE: Arduino 1.6.5
	Hardware Platform: Arduino Uno
	SX1509 Breakout Version: v2.0

This code is beerware; if you see me (or any other SparkFun employee) at the
local, and you've found our code helpful, please buy us a round!

Distributed as-is; no warranty is given.
******************************************************************************/


#define 	REG_INPUT_DISABLE_B		0x00	//	RegInputDisableB Input buffer disable register _ I/O[15_8] (Bank B) 0000 0000
#define 	REG_INPUT_DISABLE_A		0x01	//	RegInputDisableA Input buffer disable register _ I/O[7_0] (Bank A) 0000 0000
#define 	REG_LONG_SLEW_B			0x02	//	RegLongSlewB Output buffer long slew register _ I/O[15_8] (Bank B) 0000 0000
#define 	REG_LONG_SLEW_A			0x03	//	RegLongSlewA Output buffer long slew register _ I/O[7_0] (Bank A) 0000 0000
#define 	REG_LOW_DRIVE_B			0x04	//	RegLowDriveB Output buffer low drive register _ I/O[15_8] (Bank B) 0000 0000
#define 	REG_LOW_DRIVE_A			0x05	//	RegLowDriveA Output buffer low drive register _ I/O[7_0] (Bank A) 0000 0000
#define 	REG_PULL_UP_B			0x06	//	RegPullUpB Pull_up register _ I/O[15_8] (Bank B) 0000 0000
#define 	REG_PULL_UP_A			0x07	//	RegPullUpA Pull_up register _ I/O[7_0] (Bank A) 0000 0000
#define 	REG_PULL_DOWN_B			0x08	//	RegPullDownB Pull_down register _ I/O[15_8] (Bank B) 0000 0000
#define 	REG_PULL_DOWN_A			0x09	//	RegPullDownA Pull_down register _ I/O[7_0] (Bank A) 0000 0000
#define 	REG_OPEN_DRAIN_B		0x0A	//	RegOpenDrainB Open drain register _ I/O[15_8] (Bank B) 0000 0000
#define 	REG_OPEN_DRAIN_A		0x0B	//	RegOpenDrainA Open drain register _ I/O[7_0] (Bank A) 0000 0000
#define 	REG_POLARITY_B			0x0C	//	RegPolarityB Polarity register _ I/O[15_8] (Bank B) 0000 0000
#define 	REG_POLARITY_A			0x0D	//	RegPolarityA Polarity register _ I/O[7_0] (Bank A) 0000 0000
#define 	REG_DIR_B				0x0E	//	RegDirB Direction register _ I/O[15_8] (Bank B) 1111 1111
#define 	REG_DIR_A				0x0F	//	RegDirA Direction register _ I/O[7_0] (Bank A) 1111 1111
#define 	REG_DATA_B				0x10	//	RegDataB Data register _ I/O[15_8] (Bank B) 1111 1111*
#define 	REG_DATA_A				0x11	//	RegDataA Data register _ I/O[7_0] (Bank A) 1111 1111*
#define 	REG_INTERRUPT_MASK_B	0x12	//	RegInterruptMaskB Interrupt mask register _ I/O[15_8] (Bank B) 1111 1111
#define 	REG_INTERRUPT_MASK_A	0x13	//	RegInterruptMaskA Interrupt mask register _ I/O[7_0] (Bank A) 1111 1111
#define 	REG_SENSE_HIGH_B		0x14	//	RegSenseHighB Sense register for I/O[15:12] 0000 0000
#define 	REG_SENSE_LOW_B			0x15	//	RegSenseLowB Sense register for I/O[11:8] 0000 0000
#define 	REG_SENSE_HIGH_A		0x16	//	RegSenseHighA Sense register for I/O[7:4] 0000 0000
#define 	REG_SENSE_LOW_A			0x17	//	RegSenseLowA Sense register for I/O[3:0] 0000 0000
#define 	REG_INTERRUPT_SOURCE_B	0x18	//	RegInterruptSourceB Interrupt source register _ I/O[15_8] (Bank B) 0000 0000
#define 	REG_INTERRUPT_SOURCE_A	0x19	//	RegInterruptSourceA Interrupt source register _ I/O[7_0] (Bank A) 0000 0000
#define 	REG_EVENT_STATUS_B		0x1A	//	RegEventStatusB Event status register _ I/O[15_8] (Bank B) 0000 0000
#define 	REG_EVENT_STATUS_A		0x1B	//	RegEventStatusA Event status register _ I/O[7_0] (Bank A) 0000 0000
#define 	REG_LEVEL_SHIFTER_1		0x1C	//	RegLevelShifter1 Level shifter register 0000 0000
#define 	REG_LEVEL_SHIFTER_2		0x1D	//	RegLevelShifter2 Level shifter register 0000 0000
#define 	REG_CLOCK				0x1E	//	RegClock Clock management register 0000 0000
#define 	REG_MISC				0x1F	//	RegMisc Miscellaneous device settings register 0000 0000
#define 	REG_LED_DRIVER_ENABLE_B	0x20	//	RegLEDDriverEnableB LED driver enable register _ I/O[15_8] (Bank B) 0000 0000
#define 	REG_LED_DRIVER_ENABLE_A	0x21	//	RegLEDDriverEnableA LED driver enable register _ I/O[7_0] (Bank A) 0000 0000
// Debounce and Keypad Engine		
#define 	REG_DEBOUNCE_CONFIG		0x22	//	RegDebounceConfig Debounce configuration register 0000 0000
#define 	REG_DEBOUNCE_ENABLE_B	0x23	//	RegDebounceEnableB Debounce enable register _ I/O[15_8] (Bank B) 0000 0000
#define 	REG_DEBOUNCE_ENABLE_A	0x24	//	RegDebounceEnableA Debounce enable register _ I/O[7_0] (Bank A) 0000 0000
#define 	REG_KEY_CONFIG_1		0x25	//	RegKeyConfig1 Key scan configuration register 0000 0000
#define 	REG_KEY_CONFIG_2		0x26	//	RegKeyConfig2 Key scan configuration register 0000 0000
#define 	REG_KEY_DATA_1			0x27	//	RegKeyData1 Key value (column) 1111 1111
#define 	REG_KEY_DATA_2			0x28	//	RegKeyData2 Key value (row) 1111 1111
// LED Driver (PWM, blinking, breathing)		
#define 	REG_T_ON_0				0x29	//	RegTOn0 ON time register for I/O[0] 0000 0000
#define 	REG_I_ON_0				0x2A	//	RegIOn0 ON intensity register for I/O[0] 1111 1111
#define 	REG_OFF_0				0x2B	//	RegOff0 OFF time/intensity register for I/O[0] 0000 0000
#define 	REG_T_ON_1				0x2C	//	RegTOn1 ON time register for I/O[1] 0000 0000
#define 	REG_I_ON_1				0x2D	//	RegIOn1 ON intensity register for I/O[1] 1111 1111
#define 	REG_OFF_1				0x2E	//	RegOff1 OFF time/intensity register for I/O[1] 0000 0000
#define 	REG_T_ON_2				0x2F	//	RegTOn2 ON time register for I/O[2] 0000 0000
#define 	REG_I_ON_2				0x30	//	RegIOn2 ON intensity register for I/O[2] 1111 1111
#define 	REG_OFF_2				0x31	//	RegOff2 OFF time/intensity register for I/O[2] 0000 0000
#define 	REG_T_ON_3				0x32	//	RegTOn3 ON time register for I/O[3] 0000 0000
#define 	REG_I_ON_3				0x33	//	RegIOn3 ON intensity register for I/O[3] 1111 1111
#define 	REG_OFF_3				0x34	//	RegOff3 OFF time/intensity register for I/O[3] 0000 0000
#define 	REG_T_ON_4				0x35	//	RegTOn4 ON time register for I/O[4] 0000 0000
#define 	REG_I_ON_4				0x36	//	RegIOn4 ON intensity register for I/O[4] 1111 1111
#define 	REG_OFF_4				0x37	//	RegOff4 OFF time/intensity register for I/O[4] 0000 0000
#define 	REG_T_RISE_4			0x38	//	RegTRise4 Fade in register for I/O[4] 0000 0000
#define 	REG_T_FALL_4			0x39	//	RegTFall4 Fade out register for I/O[4] 0000 0000
#define 	REG_T_ON_5				0x3A	//	RegTOn5 ON time register for I/O[5] 0000 0000
#define 	REG_I_ON_5				0x3B	//	RegIOn5 ON intensity register for I/O[5] 1111 1111
#define 	REG_OFF_5				0x3C	//	RegOff5 OFF time/intensity register for I/O[5] 0000 0000
#define 	REG_T_RISE_5			0x3D	//	RegTRise5 Fade in register for I/O[5] 0000 0000
#define 	REG_T_FALL_5			0x3E	//	RegTFall5 Fade out register for I/O[5] 0000 0000
#define 	REG_T_ON_6				0x3F	//	RegTOn6 ON time register for I/O[6] 0000 0000
#define 	REG_I_ON_6				0x40	//	RegIOn6 ON intensity register for I/O[6] 1111 1111
#define 	REG_OFF_6				0x41	//	RegOff6 OFF time/intensity register for I/O[6] 0000 0000
#define 	REG_T_RISE_6			0x42	//	RegTRise6 Fade in register for I/O[6] 0000 0000
#define 	REG_T_FALL_6			0x43	//	RegTFall6 Fade out register for I/O[6] 0000 0000
#define 	REG_T_ON_7				0x44	//	RegTOn7 ON time register for I/O[7] 0000 0000
#define 	REG_I_ON_7				0x45	//	RegIOn7 ON intensity register for I/O[7] 1111 1111
#define 	REG_OFF_7				0x46	//	RegOff7 OFF time/intensity register for I/O[7] 0000 0000
#define 	REG_T_RISE_7			0x47	//	RegTRise7 Fade in register for I/O[7] 0000 0000
#define 	REG_T_FALL_7			0x48	//	RegTFall7 Fade out register for I/O[7] 0000 0000
#define 	REG_T_ON_8				0x49	//	RegTOn8 ON time register for I/O[8] 0000 0000
#define 	REG_I_ON_8				0x4A	//	RegIOn8 ON intensity register for I/O[8] 1111 1111
#define 	REG_OFF_8				0x4B	//	RegOff8 OFF time/intensity register for I/O[8] 0000 0000
#define 	REG_T_ON_9				0x4C	//	RegTOn9 ON time register for I/O[9] 0000 0000
#define 	REG_I_ON_9				0x4D	//	RegIOn9 ON intensity register for I/O[9] 1111 1111
#define 	REG_OFF_9				0x4E	//	RegOff9 OFF time/intensity register for I/O[9] 0000 0000
#define 	REG_T_ON_10				0x4F	//	RegTOn10 ON time register for I/O[10] 0000 0000
#define 	REG_I_ON_10				0x50	//	RegIOn10 ON intensity register for I/O[10] 1111 1111
#define 	REG_OFF_10				0x51	//	RegOff10 OFF time/intensity register for I/O[10] 0000 0000
#define 	REG_T_ON_11				0x52	//	RegTOn11 ON time register for I/O[11] 0000 0000
#define 	REG_I_ON_11				0x53	//	RegIOn11 ON intensity register for I/O[11] 1111 1111
#define 	REG_OFF_11				0x54	//	RegOff11 OFF time/intensity register for I/O[11] 0000 0000
#define 	REG_T_ON_12				0x55	//	RegTOn12 ON time register for I/O[12] 0000 0000
#define 	REG_I_ON_12				0x56	//	RegIOn12 ON intensity register for I/O[12] 1111 1111
#define 	REG_OFF_12				0x57	//	RegOff12 OFF time/intensity register for I/O[12] 0000 0000
#define 	REG_T_RISE_12			0x58	//	RegTRise12 Fade in register for I/O[12] 0000 0000
#define 	REG_T_FALL_12			0x59	//	RegTFall12 Fade out register for I/O[12] 0000 0000
#define 	REG_T_ON_13				0x5A	//	RegTOn13 ON time register for I/O[13] 0000 0000
#define 	REG_I_ON_13				0x5B	//	RegIOn13 ON intensity register for I/O[13] 1111 1111
#define 	REG_OFF_13				0x5C	//	RegOff13 OFF time/intensity register for I/O[13] 0000 0000
#define 	REG_T_RISE_13			0x5D	//	RegTRise13 Fade in register for I/O[13] 0000 0000
#define 	REG_T_FALL_13			0x5E	//	RegTFall13 Fade out register for I/O[13] 0000 0000
#define 	REG_T_ON_14				0x5F	//	RegTOn14 ON time register for I/O[14] 0000 0000
#define 	REG_I_ON_14				0x60	//	RegIOn14 ON intensity register for I/O[14] 1111 1111
#define 	REG_OFF_14				0x61	//	RegOff14 OFF time/intensity register for I/O[14] 0000 0000
#define 	REG_T_RISE_14			0x62	//	RegTRise14 Fade in register for I/O[14] 0000 0000
#define 	REG_T_FALL_14			0x63	//	RegTFall14 Fade out register for I/O[14] 0000 0000
#define 	REG_T_ON_15				0x64	//	RegTOn15 ON time register for I/O[15] 0000 0000
#define 	REG_I_ON_15				0x65	//	RegIOn15 ON intensity register for I/O[15] 1111 1111
#define 	REG_OFF_15				0x66	//	RegOff15 OFF time/intensity register for I/O[15] 0000 0000
#define 	REG_T_RISE_15			0x67	//	RegTRise15 Fade in register for I/O[15] 0000 0000
#define 	REG_T_FALL_15			0x68	//	RegTFall15 Fade out register for I/O[15] 0000 0000
// 	Miscellaneous		
#define 	REG_HIGH_INPUT_B		0x69	//	RegHighInputB High input enable register _ I/O[15_8] (Bank B) 0000 0000
#define 	REG_HIGH_INPUT_A		0x6A	//	RegHighInputA High input enable register _ I/O[7_0] (Bank A) 0000 0000
//  Software Reset		
#define 	REG_RESET				0x7D	//	RegReset Software reset register 0000 0000
#define 	REG_TEST_1				0x7E	//	RegTest1 Test register 0000 0000
#define 	REG_TEST_2				0x7F	//	RegTest2 Test register 0000 0000


#ifndef SparkFunSX1509_H
#define SparkFunSX1509_H

#ifndef I2C_ERROR_OK
#define I2C_ERROR_OK 0
#endif

#include "daisy.h"
#include "per/i2c.h"
#include "per/gpio.h"

extern uint8_t REG_I_ON[16];
extern uint8_t REG_T_ON[16];
extern uint8_t REG_OFF[16];
extern uint8_t REG_T_RISE[16];
extern uint8_t REG_T_FALL[16];

// These are used for setting LED driver to linear or log mode:
#define SX_LINEAR 0
#define SX_LOGARITHMIC 1

// These are used for clock config:
#define SX_INTERNAL_CLOCK_2MHZ 2
#define SX_EXTERNAL_CLOCK 1

// These are used for reset:
#define SX_SOFTWARE_RESET 0
#define SX_HARDWARE_RESET 1

// States for Pin
#define SX_HIGH 1
#define SX_LOW 0

// Interrupt mode
#define SX_CHANGE 1
#define SX_FALLING 2
#define SX_RISING 3

// Pin mode
#define SX_PIN_INPUT 0
#define SX_PIN_OUTPUT 1
#define SX_PIN_INPUT_PULLUP 2
#define SX_PIN_ANALOG_OUTPUT 3 // To set a pin mode for PWM output

using namespace daisy;

class SX1509Config
{
public:
	daisy::I2CHandle i2c_;
    uint8_t          i2c_address_;
    uint8_t          timeout{10};

    struct Config
    {
		I2CHandle::Config i2c_config;
        uint8_t           i2c_address;
		unsigned long _clkX;
        void Defaults()
        {
            i2c_config.periph         = I2CHandle::Config::Peripheral::I2C_1;
            i2c_config.speed          = I2CHandle::Config::Speed::I2C_400KHZ;
            i2c_config.mode           = I2CHandle::Config::Mode::I2C_MASTER;
            i2c_config.pin_config.scl = Pin(PORTB, 8);
            i2c_config.pin_config.sda = Pin(PORTB, 9);
			// for new versions of library(v7.2.0 and above). 
			// for earlier versions use i2c_address = (0x3E << 1) instead
			i2c_address = 0x3E;
        }
    };
	
    void Init()
    {
        Config config;
        config.Defaults();
        i2c_.Init(config.i2c_config);
    };

    void Init(const Config& config)
	{
		// for new versions of library(v7.2.0 and above). 
		// for earlier versions use i2c_address = (config.i2c_address << 1) instead
		i2c_address_ = config.i2c_address;
		i2c_.Init(config.i2c_config);
	}
};

class SX1509
{
public:
    SX1509Config transport;
	uint16_t pin_states[16];
	uint16_t current_state;
	uint16_t previous_state;
	SX1509();

	GPIO resetPin;
	GPIO interruptPin;
	GPIO oscillatorPin;
	unsigned long _clkX;
	struct Config
	{
		SX1509Config::Config transport_config;
		Pin resetPin;
		Pin interruptPin;
		Pin oscillatorPin;
		unsigned long _clkX = 0;
	};

	void Init()
    {
		Config config;
		config.transport_config.Defaults();
		Init(config);
    };

	void Init(const Config& config)
    {
		resetPin.Init(config.resetPin, GPIO::Mode::OUTPUT, GPIO::Pull::NOPULL);
		interruptPin.Init(config.interruptPin, GPIO::Mode::INPUT, GPIO::Pull::NOPULL);
		oscillatorPin.Init(config.oscillatorPin, GPIO::Mode::INPUT, GPIO::Pull::NOPULL);
		resetPin.Write(SX_HIGH);
        transport.Init(config.transport_config);	
    }

// Helper functions:
// calculateLEDTRegister - Try to estimate an LED on/off duration register,
// given the number of milliseconds and LED clock frequency.
uint8_t CalculateLEDTRegister(unsigned long ms);

// calculateSlopeRegister - Try to estimate an LED rise/fall duration
// register, given the number of milliseconds and LED clock frequency.
uint8_t CalculateSlopeRegister(unsigned long ms, uint8_t onIntensity, uint8_t offIntensity);

	// -----------------------------------------------------------------------------

	// -----------------------------------------------------------------------------
	// reset(bool hardware): This function resets the SX1509 - either a hardware
	//		reset or software. A hardware reset (hardware parameter = 1) pulls the
	//		reset line low, pausing, then pulling the reset line high. A software
	//		reset writes a 0x12 then 0x34 to the REG_RESET as outlined in the
	//		datasheet.
	//
	//  Input:
	//	 	- hardware: 0 executes a software reset, 1 executes a hardware reset
	// -----------------------------------------------------------------------------
	uint8_t Check(void);
	
	void Reset(bool hardware);

	// -----------------------------------------------------------------------------
	// SetPinMode(uint8_t pin, uint8_t inOut, uint8_t debounce): This function sets
	//		one of the SX1509's 16 outputs to either an INPUT or OUTPUT.
	//
	//	Inputs:
	//		- pin: should be a value between 0 and 15
	//		- inOut: The Arduino INPUT and OUTPUT constants should be used for the
	//		 inOut parameter. They do what they say!
	// -----------------------------------------------------------------------------
	void SetPinMode(uint8_t pin, uint8_t inOut, uint8_t debounce);

	// -----------------------------------------------------------------------------
	// CheckPinsChanged(): This function checks if the HIGH/LOW status of all 16 pins has changed.
	// -----------------------------------------------------------------------------
	bool ReadAllPins();

	bool CurrentPinState(uint8_t pin);
	bool PreviousPinState(uint8_t pin);

	// -----------------------------------------------------------------------------
	// isRisingEdge(uint8_t pin): This function checks if a pin has transitioned from
	//		LOW to HIGH.
	// -----------------------------------------------------------------------------
	bool isRisingEdge(uint8_t pin);

	// -----------------------------------------------------------------------------
		// isFallingEdge(uint8_t pin): This function checks if a pin has transitioned from
	//		HIGH to LOW.
	// -----------------------------------------------------------------------------
	bool isFallingEdge(uint8_t pin);

	// -----------------------------------------------------------------------------
	// IsPressed(uint8_t pin): This function checks if a pin is pressed.
	// -----------------------------------------------------------------------------
	bool IsPressed(uint8_t pin);	

	// -----------------------------------------------------------------------------
	// WritePin(uint8_t pin, uint8_t highLow): This function writes a pin to either high
	//		or low if it's configured as an OUTPUT. If the pin is configured as an
	//		INPUT, this method will activate either the PULL-UP	or PULL-DOWN
	//		resistor (HIGH or LOW respectively).
	//
	//	Inputs:
	//		- pin: The SX1509 pin number. Should be a value between 0 and 15.
	//		- highLow: should be Arduino's defined HIGH or LOW constants.
	// -----------------------------------------------------------------------------

	bool WritePin(uint8_t pin, uint8_t highLow); // Legacy - use digitalWrite

	// -----------------------------------------------------------------------------
	// digitalRead(uint8_t pin): This function reads the HIGH/LOW status of a pin.
	//		The pin should be configured as an INPUT, using the pinDir function.
	//
	//	Inputs:
	//	 	- pin: The SX1509 pin to be read. should be a value between 0 and 15.
	//  Outputs:
	//		This function returns a 1 if HIGH, 0 if LOW
	// -----------------------------------------------------------------------------

	bool ReadPin(const uint8_t pin);

	// -----------------------------------------------------------------------------
	// ledDriverInit(uint8_t pin, uint8_t freq, bool log): This function initializes LED
	//		driving on a pin. It must be called if you want to use the pwm or blink
	//		functions on that pin.
	//
	//	Inputs:
	//		- pin: The SX1509 pin connected to an LED. Should be 0-15.
	//   	- freq: Sets LED clock frequency divider.
	//		- log: selects either linear or logarithmic mode on the LED drivers
	//			- log defaults to 0, linear mode
	//			- currently log sets both bank A and B to the same mode
	//	Note: this function automatically decides to use the internal 2MHz osc.
	// -----------------------------------------------------------------------------
	void LedDriverInit(uint8_t pin, uint8_t freq = 1, bool log = false);

	// -----------------------------------------------------------------------------
	// analogWrite(uint8_t pin, uint8_t iOn):	This function can be used to control the intensity
	//		of an output pin connected to an LED.
	//
	//	Inputs:
	//		- pin: The SX1509 pin connecte to an LED.Should be 0-15.
	//		- iOn: should be a 0-255 value setting the intensity of the LED
	//			- 0 is completely off, 255 is 100% on.
	//
	//	Note: ledDriverInit should be called on the pin before calling this.
	// -----------------------------------------------------------------------------
	void WritePWM(uint8_t pin, uint8_t iOn); // Legacy - use analogWrite

	// -----------------------------------------------------------------------------
	// setupBlink(uint8_t pin, uint8_t tOn, uint8_t tOff, uint8_t offIntensity, uint8_t tRise, uint8_t
	//		tFall):  blink performs both the blink and breath LED driver functions.
	//
	// 	Inputs:
	//  	- pin: the SX1509 pin (0-15) you want to set blinking/breathing.
	//		- tOn: the amount of time the pin is HIGH
	//			- This value should be between 1 and 31. 0 is off.
	//		- tOff: the amount of time the pin is at offIntensity
	//			- This value should be between 1 and 31. 0 is off.
	//		- offIntensity: How dim the LED is during the off period.
	//			- This value should be between 0 and 7. 0 is completely off.
	//		- onIntensity: How bright the LED will be when completely on.
	//			- This value can be between 0 (0%) and 255 (100%).
	//		- tRise: This sets the time the LED takes to fade in.
	//			- This value should be between 1 and 31. 0 is off.
	//			- This value is used with tFall to make the LED breath.
	//		- tFall: This sets the time the LED takes to fade out.
	//			- This value should be between 1 and 31. 0 is off.
	// 	 Notes:
	//		- The breathable pins are 4, 5, 6, 7, 12, 13, 14, 15 only. If tRise and
	//			tFall are set on 0-3 or 8-11 those pins will still only blink.
	// 		- ledDriverInit should be called on the pin to be blinked before this.
	// -----------------------------------------------------------------------------
	void SetupBlink(uint8_t pin, uint8_t tOn, uint8_t toff, uint8_t onIntensity = 255, uint8_t offIntensity = 0, uint8_t tRise = 0, uint8_t tFall = 0, bool log = false);

	// -----------------------------------------------------------------------------
	// blink(uint8_t pin, unsigned long tOn, unsigned long tOff, uint8_t onIntensity, uint8_t offIntensity);
	//  	Set a pin to blink output for estimated on/off millisecond durations.
	//
	// 	Inputs:
	//  	- pin: the SX1509 pin (0-15) you want to set blinking
	//   	- tOn: estimated number of milliseconds the pin is LOW (LED sinking current will be on)
	//   	- tOff: estimated number of milliseconds the pin is HIGH (LED sinking current will be off)
	//   	- onIntensity: 0-255 value determining LED on brightness
	//   	- offIntensity: 0-255 value determining LED off brightness
	// 	 Notes:
	//		- The breathable pins are 4, 5, 6, 7, 12, 13, 14, 15 only. If tRise and
	//			tFall are set on 0-3 or 8-11 those pins will still only blink.
	// 		- ledDriverInit should be called on the pin to be blinked before this.
	// -----------------------------------------------------------------------------
	void Blink(uint8_t pin, unsigned long tOn, unsigned long tOff, uint8_t onIntensity = 255, uint8_t offIntensity = 0);

	// -----------------------------------------------------------------------------
	// breathe(uint8_t pin, unsigned long tOn, unsigned long tOff, unsigned long rise, unsigned long fall, uint8_t onInt, uint8_t offInt, bool log);
	//  	Set a pin to breathe output for estimated on/off millisecond durations, with
	//  	estimated rise and fall durations.
	//
	// 	Inputs:
	//  	- pin: the SX1509 pin (0-15) you want to set blinking
	//   	- tOn: estimated number of milliseconds the pin is LOW (LED sinking current will be on)
	//   	- tOff: estimated number of milliseconds the pin is HIGH (LED sinking current will be off)
	//   	- rise: estimated number of milliseconds the pin rises from LOW to HIGH
	//   	- falll: estimated number of milliseconds the pin falls from HIGH to LOW
	//   	- onIntensity: 0-255 value determining LED on brightness
	//   	- offIntensity: 0-255 value determining LED off brightness
	// 	 Notes:
	//		- The breathable pins are 4, 5, 6, 7, 12, 13, 14, 15 only. If tRise and
	//			tFall are set on 0-3 or 8-11 those pins will still only blink.
	// 		- ledDriverInit should be called on the pin to be blinked before this,
	//  	  Or call pinMode(<pin>, ANALOG_OUTPUT);
	// -----------------------------------------------------------------------------
	void Breath(uint8_t pin, unsigned long tOn, unsigned long tOff, unsigned long rise, unsigned long fall, uint8_t onInt = 255, uint8_t offInt = 0, bool log = SX_LINEAR);

	// -----------------------------------------------------------------------------
	// keypad(uint8_t rows, uint8_t columns, uint8_t sleepTime, uint8_t scanTime, uint8_t debounceTime)
	//		Initializes the keypad function on the SX1509. Millisecond durations for sleep,
	//		scan, and debounce can be set.
	//
	//	Inputs:
	//		- rows: The number of rows in the button matrix.
	//			- This value must be between 1 and 7. 0 will turn it off.
	//			- eg: 1 = 2 rows, 2 = 3 rows, 7 = 8 rows, etc.
	//		- columns: The number of columns in the button matrix
	//			- This value should be between 0 and 7.
	//			- 0 = 1 column, 7 = 8 columns, etc.
	//		- sleepTime: Sets the auto-sleep time of the keypad engine.
	//  	  Should be a millisecond duration between 0 (OFF) and 8000 (8 seconds).
	//   	  Possible values are 0, 128, 256, 512, 1000, 2000, 4000, 8000
	//		- scanTime: Sets the scan time per row. Must be set above debounce.
	//  	  Should be a millisecond duration between 1 and 128.
	//   	  Possible values are 1, 2, 4, 8, 16, 32, 64, 128.
	//		- debounceTime: Sets the debounc time per button. Must be set below scan.
	//  	  Should be a millisecond duration between 0 and 64.
	//   	  Possible values are 0 (0.5), 1, 2, 4, 8, 16, 32, 64.
	// -----------------------------------------------------------------------------
	void Keypad(uint8_t rows, uint8_t columns, uint16_t sleepTime = 0, uint8_t scanTime = 1, uint8_t debounceTime = 0);

	// -----------------------------------------------------------------------------
	// ReadKeypad(): This function returns a 16-bit value containing the status of
	//		keypad engine.
	//
	//	Output:
	//		A 16-bit value is returned. The lower 8 bits represent the up-to 8 rows,
	//		while the MSB represents the up-to 8 columns. Bit-values of 1 indicate a
	//		button in that row or column is being pressed. As such, at least two
	//		bits should be set.
	// -----------------------------------------------------------------------------
	uint16_t ReadKeypad();
	uint16_t ReadKeyData(); // Legacy: use ReadKeypad();

	// -----------------------------------------------------------------------------
	// GetRow(): This function returns the first active row from the return value of
	//  	ReadKeypad().
	//
	//	Input:
	//      - keyData: Should be the uint16_t value returned from ReadKeypad().
	//	Output:
	//		A 16-bit value is returned. The lower 8 bits represent the up-to 8 rows,
	//		while the MSB represents the up-to 8 columns. Bit-values of 1 indicate a
	//		button in that row or column is being pressed. As such, at least two
	//		bits should be set.
	// -----------------------------------------------------------------------------
	uint8_t GetRow(uint16_t keyData);

	// -----------------------------------------------------------------------------
	// GetCol(): This function returns the first active column from the return value of
	//  	ReadKeypad().
	//
	//	Input:
	//      - keyData: Should be the uint16_t value returned from ReadKeypad().
	//	Output:
	//		A 16-bit value is returned. The lower 8 bits represent the up-to 8 rows,
	//		while the MSB represents the up-to 8 columns. Bit-values of 1 indicate a
	//		button in that row or column is being pressed. As such, at least two
	//		bits should be set.
	// -----------------------------------------------------------------------------
	uint8_t GetColumn(uint16_t keyData);

	// -----------------------------------------------------------------------------
	// Sync(void): this function resets the PWM/Blink/Fade counters, syncing any
	//		blinking LEDs. Bit 2 of REG_MISC is set, which alters the functionality
	//		of the nReset pin. The nReset pin is toggled low->high, which should
	//		reset all LED counters. Bit 2 of REG_MISC is again cleared, returning
	//		nReset pin to POR functionality
	// -----------------------------------------------------------------------------
	void Sync(void);

	// -----------------------------------------------------------------------------
	// DebounceConfig(uint8_t configValue): This method configures the debounce time of
	//		every input.
	//
	//	Input:
	//		- configValue: A 3-bit value configuring the debounce time.
	//			000: 0.5ms * 2MHz/fOSC
	//			001: 1ms * 2MHz/fOSC
	//			010: 2ms * 2MHz/fOSC
	//			011: 4ms * 2MHz/fOSC
	//			100: 8ms * 2MHz/fOSC
	//			101: 16ms * 2MHz/fOSC
	//			110: 32ms * 2MHz/fOSC
	//			111: 64ms * 2MHz/fOSC
	//
	//	Note: fOSC is set with the configClock function. It defaults to 2MHz.
	// -----------------------------------------------------------------------------
	void DebounceConfig(uint8_t configValue);

	// -----------------------------------------------------------------------------
	// DebounceTime(uint8_t configValue): This method configures the debounce time of
	//		every input to an estimated millisecond time duration.
	//
	//	Input:
	//		- time: A millisecond duration estimating the debounce time. Actual
	//		  debounce time will depend on fOSC. Assuming it's 2MHz, debounce will
	//		  be set to the 0.5, 1, 2, 4, 8, 16, 32, or 64 ms (whatever's closest)
	//
	//	Note: fOSC is set with the configClock function. It defaults to 2MHz.
	// -----------------------------------------------------------------------------
	void DebounceTime(uint8_t time);

	// -----------------------------------------------------------------------------
	// DebouncePin(uint8_t pin): This method enables debounce on SX1509 input pin.
	//
	//	Input:
	//		- pin: The SX1509 pin to be debounced. Should be between 0 and 15.
	// -----------------------------------------------------------------------------
	void DebouncePin(uint8_t pin);

	// -----------------------------------------------------------------------------
	// DebounceKeypad(uint8_t pin): This method enables debounce on all pins connected
	//  to a row/column keypad matrix.
	//
	//	Input:
	//		- time: Millisecond time estimate for debounce (see debounceTime()).
	//		- numRows: The number of rows in the keypad matrix.
	//		- numCols: The number of columns in the keypad matrix.
	// -----------------------------------------------------------------------------
	void DebounceKeypad(uint8_t time, uint8_t numRows, uint8_t numCols);

	// -----------------------------------------------------------------------------
	// EnableInterrupt(uint8_t pin, uint8_t riseFall): This function sets up an interrupt
	//		on a pin. Interrupts can occur on all SX1509 pins, and can be generated
	//		on rising, falling, or both.
	//
	//	Inputs:
	//		-pin: SX1509 input pin that will generate an input. Should be 0-15.
	//		-riseFall: Configures if you want an interrupt generated on rise fall or
	//			both. For this param, send the pin-change values previously defined
	//			by Arduino:
	//			#define CHANGE 1	<-Both
	//			#define FALLING 2	<- Falling
	//			#define RISING 3	<- Rising
	//
	//	Note: This function does not set up a pin as an input, or configure	its
	//		pull-up/down resistors! Do that before (or after).
	// -----------------------------------------------------------------------------
	void EnableInterrupt(uint8_t pin, uint8_t riseFall);

	// -----------------------------------------------------------------------------
	// InterruptSource(void): Returns an uint16_t representing which pin caused
	//		an interrupt.
	//
	//	Output: 16-bit value, with a single bit set representing the pin(s) that
	//		generated an interrupt. E.g. a return value of	0x0104 would mean pins 8
	//		and 3 (bits 8 and 3) have generated an interrupt.
	//  Input:
	//  	- clear: boolean commanding whether the interrupt should be cleared
	//  	  after reading or not.
	// -----------------------------------------------------------------------------
	uint16_t InterruptSource(bool clear = true);

	// -----------------------------------------------------------------------------
	// CheckInterrupt(void): Checks if a single pin generated an interrupt.
	//
	//	Output: Boolean value. True if the requested pin has triggered an interrupt/
	//  Input:
	//  	- pin: Pin to be checked for generating an input.
	// -----------------------------------------------------------------------------
	bool CheckInterrupt(uint8_t pin);

	// -----------------------------------------------------------------------------
	// configClock(uint8_t oscSource, uint8_t oscPinFunction, uint8_t oscFreqOut, uint8_t oscDivider)
	//		This function configures the oscillator source/speed
	//		and the clock, which is used to drive LEDs and time debounces.
	//
	//	Inputs:
	//	- oscSource: Choose either internal 2MHz oscillator or an external signal
	//		applied to the OSCIO pin.
	//		- INTERNAL_CLOCK and EXTERNAL_CLOCK are defined in the header file.
	//			Use those.
	//		- This value defaults to internal.
	//	- oscDivider: Sets the clock divider in REG_MISC.
	//		- ClkX = fOSC / (2^(RegMisc[6:4] -1))
	//		- This value defaults to 1.
	//	- oscPinFunction: Allows you to set OSCIO as an input or output.
	//		- You can use Arduino's INPUT, OUTPUT defines for this value
	//		- This value defaults to input
	//	- oscFreqOut: If oscio is configured as an output, this will set the output
	//		frequency
	//		- This should be a 4-bit value. 0=0%, 0xF=100%, else
	//			fOSCOut = FOSC / (2^(RegClock[3:0]-1))
	//		- This value defaults to 0.
	// -----------------------------------------------------------------------------
	void ConfigClock(uint8_t oscSource = 2, uint8_t oscPinFunction = 0, uint8_t oscFreqOut = 0, uint8_t oscDivider = 1); // Legacy, use clock();

	// -----------------------------------------------------------------------------
	// clock(uint8_t oscSource, uint8_t oscDivider, uint8_t oscPinFunction, uint8_t oscFreqOut)
	//		This function configures the oscillator source/speed
	//		and the clock, which is used to drive LEDs and time debounces.
	//  	This is just configClock in a bit more sane order.
	//
	// -----------------------------------------------------------------------------
	void Clock(uint8_t oscSource = 2, uint8_t oscDivider = 1, uint8_t oscPinFunction = 0, uint8_t oscFreqOut = 0);


//-----------------------------PRIVATE-------------------------------------//
private:

unsigned long last_update;
unsigned long last_increment_time;

// uint8_t readByte
uint8_t ReadReg(uint8_t registerAddress);	

// uint16_t readWord
uint16_t ReadWord(uint8_t registerAddress);

// bool SX1509::readBytes
bool ReadRegistersOk(uint8_t firstRegisterAddress, uint8_t *destination, uint8_t length);

// uint8_t readByte
bool ReadReg(uint8_t registerAddress, uint8_t *value);

// uint16_t readWord
bool ReadWord(uint8_t registerAddress, uint16_t *value);

// bool writeBytes
bool WriteRegisters(uint8_t firstRegisterAddress, uint8_t *writeArray, uint8_t length);
// bool writeByte
bool WriteReg(uint8_t registerAddress, uint8_t writeValue);

// bool writeWord
bool WriteWord(uint8_t registerAddress, uint16_t writeValue);

// -----------------------------------------------------------------------------
// pinMode(uint8_t pin, uint8_t inOut): This function sets one of the SX1509's 16
//		outputs to either an INPUT or OUTPUT.
//
//	Inputs:
//	 	- pin: should be a value between 0 and 15
//	 	- inOut: The Arduino INPUT and OUTPUT constants should be used for the
//		 inOut parameter. They do what they say!
// -----------------------------------------------------------------------------
void PinMode(uint8_t pin, uint8_t inOut, uint8_t initialLevel = SX_HIGH);

};

template<typename T>
T constrain(T value, T Min, T Max)
{
  return (value < Min)? Min : (value > Max)? Max : value;
}

#endif // SX1509_library_H
