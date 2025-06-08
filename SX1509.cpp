/******************************************************************************
SparkFunSX1509.cpp
SparkFun SX1509 I/O Expander Library Source File
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

#include "SX1509.h"

uint8_t REG_I_ON[16] = {REG_I_ON_0, REG_I_ON_1, REG_I_ON_2, REG_I_ON_3,
	REG_I_ON_4, REG_I_ON_5, REG_I_ON_6, REG_I_ON_7,
	REG_I_ON_8, REG_I_ON_9, REG_I_ON_10, REG_I_ON_11,
	REG_I_ON_12, REG_I_ON_13, REG_I_ON_14, REG_I_ON_15};
	
uint8_t REG_T_ON[16] = {REG_T_ON_0, REG_T_ON_1, REG_T_ON_2, REG_T_ON_3,
	REG_T_ON_4, REG_T_ON_5, REG_T_ON_6, REG_T_ON_7,
	REG_T_ON_8, REG_T_ON_9, REG_T_ON_10, REG_T_ON_11,
	REG_T_ON_12, REG_T_ON_13, REG_T_ON_14, REG_T_ON_15};
	
uint8_t REG_OFF[16] = {REG_OFF_0, REG_OFF_1, REG_OFF_2, REG_OFF_3,
	REG_OFF_4, REG_OFF_5, REG_OFF_6, REG_OFF_7,
	REG_OFF_8, REG_OFF_9, REG_OFF_10, REG_OFF_11,
	REG_OFF_12, REG_OFF_13, REG_OFF_14, REG_OFF_15};

uint8_t REG_T_RISE[16] = {0xFF, 0xFF, 0xFF, 0xFF,
	REG_T_RISE_4, REG_T_RISE_5, REG_T_RISE_6, REG_T_RISE_7,
	0xFF, 0xFF, 0xFF, 0xFF,
	REG_T_RISE_12, REG_T_RISE_13, REG_T_RISE_14, REG_T_RISE_15};
	
uint8_t REG_T_FALL[16] = {0xFF, 0xFF, 0xFF, 0xFF,
	REG_T_FALL_4, REG_T_FALL_5, REG_T_FALL_6, REG_T_FALL_7,
	0xFF, 0xFF, 0xFF, 0xFF,
	REG_T_FALL_12, REG_T_FALL_13, REG_T_FALL_14, REG_T_FALL_15};

SX1509::SX1509() : current_state(0), previous_state(0), last_update(0) {
	memset(pin_states, 0, sizeof(pin_states));
	_clkX = 0;
}

uint8_t SX1509::Check() // Arduino init()
{
	// If the reset pin is connected
	if (!resetPin.Read())
	{
		Reset(true);
	} else
	{
		Reset(false);
	}

	// Communication test. We'll read from two registers with different
	// default values to verify communication.
	uint16_t testRegisters = 0;
	testRegisters = ReadWord(REG_INTERRUPT_MASK_A); // This should return 0xFF00

	// Then read a byte that should be 0x00
	if (testRegisters == 0xFF00)
	{
		// Set the clock to a default of 2MHz using internal
		Clock(INTERNAL_CLOCK_2MHZ);

		return 1;
	}
	return 0;
}

void SX1509::Reset(bool hardware)
{
	// if hardware bool is set
	if (hardware)
	{
		// Check if bit 2 of REG_MISC is set
		// if so nReset will not issue a POR, we'll need to clear that bit first
		uint8_t regMisc = ReadReg(REG_MISC);
		if (regMisc & (1 << 2))
		{
			regMisc &= ~(1 << 2);
			WriteReg(REG_MISC, regMisc);
		}
		// Reset the SX1509, the pin is active low
		resetPin.Write(LOW);	  // pull reset pin low
		System::Delay(1);					  // Wait for the pin to settle
		resetPin.Write(HIGH);	  // pull reset pin back high
	}
	else
	{
		// Software reset command sequence:
		WriteReg(REG_RESET, 0x12);
		WriteReg(REG_RESET, 0x34);
	}
}

void SX1509::PinMode(uint8_t pin, uint8_t inOut, uint8_t initialLevel)
{
	// The SX1509 RegDir registers: REG_DIR_B, REG_DIR_A
	//	0: IO is configured as an output
	//	1: IO is configured as an input
	uint8_t modeBit;
	if ((inOut == PIN_OUTPUT) || (inOut == PIN_ANALOG_OUTPUT))
	{
		uint16_t tempRegData = ReadWord(REG_DATA_B);
		if (initialLevel == LOW)
		{
			tempRegData &= ~(1 << pin);
			WriteWord(REG_DATA_B, tempRegData);
		}
		modeBit = 0;
	}
	else
	{
		modeBit = 1;
	}

	uint16_t tempRegDir = ReadWord(REG_DIR_B);
	if (modeBit)
		tempRegDir |= (1 << pin);
	else
		tempRegDir &= ~(1 << pin);

	WriteWord(REG_DIR_B, tempRegDir);

	// If PIN_INPUT_PULLUP was called, set up the pullup too:
	if (inOut == PIN_INPUT_PULLUP)
		WritePin(pin, HIGH);

	// If PIN_ANALOG_OUTPUT was called, set up the LED driver for it:
	if (inOut == PIN_ANALOG_OUTPUT)
	{
		LedDriverInit(pin);
	}
}

void SX1509::SetPinMode(uint8_t pin, uint8_t inOut, uint8_t debounce) {
	PinMode(pin, inOut, 0);
	if (debounce) {
		DebouncePin(pin);
	}
}

bool SX1509::WritePin(uint8_t pin, uint8_t highLow)
{

	uint16_t tempRegDir = ReadWord(REG_DIR_B);

	if ((0xFFFF ^ tempRegDir) & (1 << pin)) // If the pin is an output, write high/low
	{
		uint16_t tempRegData = ReadWord(REG_DATA_B);
		if (highLow)
			tempRegData |= (1 << pin);
		else
			tempRegData &= ~(1 << pin);
		return WriteWord(REG_DATA_B, tempRegData);
	}
	else // Otherwise the pin is an input, pull-up/down
	{
		uint16_t tempPullUp = ReadWord(REG_PULL_UP_B);
		uint16_t tempPullDown = ReadWord(REG_PULL_DOWN_B);

		if (highLow) // if HIGH, do pull-up, disable pull-down
		{
			tempPullUp |= (1 << pin);
			tempPullDown &= ~(1 << pin);
			return WriteWord(REG_PULL_UP_B, tempPullUp) && WriteWord(REG_PULL_DOWN_B, tempPullDown);
		}
		else // If LOW do pull-down, disable pull-up
		{
			tempPullDown |= (1 << pin);
			tempPullUp &= ~(1 << pin);
			return WriteWord(REG_PULL_UP_B, tempPullUp) && WriteWord(REG_PULL_DOWN_B, tempPullDown);
		}
	}
}

bool SX1509::ReadAllPins() {
	
    previous_state = current_state;
	if (previous_state == current_state) return false;
    current_state = ReadWord(REG_DATA_B);
    last_update = System::GetNow();
        
    for (int i = 0; i < 16; i++) {
        bool bit = (current_state >> i) & 0x01;
        pin_states[i] = (pin_states[i] << 1) | bit;
    }
    return true; // return true if the state has changed
}

bool SX1509::isRisingEdge(uint8_t pin) {
    return (pin_states[pin] & 0x03) == 0x02;
}

bool SX1509::isFallingEdge(uint8_t pin) {
    return (pin_states[pin] & 0x03) == 0x01;
}

bool SX1509::IsPressed(uint8_t pin) {
    return (pin_states[pin] & 0x03) == 0x03;
}

int8_t SX1509::EncoderInc(uint8_t pinA, uint8_t pinB) {
    
    bool currentA = (current_state >> pinA) & 0x01;
        bool previousA = (previous_state >> pinA) & 0x01;
        bool currentB = (current_state >> pinB) & 0x01;
        bool previousB = (previous_state >> pinB) & 0x01;
        
        if (previousA != currentA) {
            return (currentA == currentB) ? -1 : 1;
        }
        
        if (previousB != currentB) {
            return (currentA != currentB) ? -1 : 1;
        }
        
        return 0;
}	

bool SX1509::ReadPin(uint8_t pin)
{
	uint16_t tempRegDir = ReadWord(REG_DIR_B);

	if (tempRegDir & (1 << pin)) // If the pin is an input
	{
		uint16_t tempRegData = ReadWord(REG_DATA_B);
		if (tempRegData & (1 << pin))
			return true;
	}
	else
	{
		// log_d("Pin %d not INPUT, REG_DIR_B: %d", pin, tempRegDir);
	}
	return false;
}

void SX1509::LedDriverInit(uint8_t pin, uint8_t freq /*= 1*/, bool log /*= false*/)
{
	uint16_t tempWord;
	uint8_t tempByte;

	// Disable input buffer
	// Writing a 1 to the pin bit will disable that pins input buffer
	tempWord = ReadWord(REG_INPUT_DISABLE_B);
	tempWord |= (1 << pin);
	WriteWord(REG_INPUT_DISABLE_B, tempWord);

	// Disable pull-up
	// Writing a 0 to the pin bit will disable that pull-up resistor
	tempWord = ReadWord(REG_PULL_UP_B);
	tempWord &= ~(1 << pin);
	WriteWord(REG_PULL_UP_B, tempWord);

	// Set direction to output (REG_DIR_B)
	tempWord = ReadWord(REG_DIR_B);
	tempWord &= ~(1 << pin); // 0=output
	WriteWord(REG_DIR_B, tempWord);

	// Enable oscillator (REG_CLOCK)
	tempByte = ReadReg(REG_CLOCK);
	tempByte |= (1 << 6);  // Internal 2MHz oscillator part 1 (set bit 6)
	tempByte &= ~(1 << 5); // Internal 2MHz oscillator part 2 (clear bit 5)
	WriteReg(REG_CLOCK, tempByte);

	// Configure LED driver clock and mode (REG_MISC)
	tempByte = ReadReg(REG_MISC);
	if (log)
	{
		tempByte |= (1 << 7); // set logarithmic mode bank B
		tempByte |= (1 << 3); // set logarithmic mode bank A
	}
	else
	{
		tempByte &= ~(1 << 7); // set linear mode bank B
		tempByte &= ~(1 << 3); // set linear mode bank A
	}

	// Use configClock to setup the clock divder
	if (_clkX == 0) // Make clckX non-zero
	{
		// _clkX = 2000000.0 / (1 << (1 - 1)); // Update private clock variable
		_clkX = 2000000.0;

		// uint8_t freq = (1 & 0x07) << 4; // freq should only be 3 bits from 6:4
		// tempByte |= freq;
	}

	freq = (freq & 0x7) << 4;	// mask only 3 bits and shift to bit position 6:4 
	tempByte |= freq;

	WriteReg(REG_MISC, tempByte);

	// Enable LED driver operation (REG_LED_DRIVER_ENABLE)
	tempWord = ReadWord(REG_LED_DRIVER_ENABLE_B);
	tempWord |= (1 << pin);
	WriteWord(REG_LED_DRIVER_ENABLE_B, tempWord);

	// Set REG_DATA bit low ~ LED driver started
	tempWord = ReadWord(REG_DATA_B);
	tempWord &= ~(1 << pin);
	WriteWord(REG_DATA_B, tempWord);
}

void SX1509::WritePWM(uint8_t pin, uint8_t iOn)
{
	// Write the on intensity of pin
	// Linear mode: Ion = iOn
	// Log mode: Ion = f(iOn)
	WriteReg(REG_I_ON[pin], iOn);
}

void SX1509::Blink(uint8_t pin, unsigned long tOn, unsigned long tOff, uint8_t onIntensity, uint8_t offIntensity)
{
	uint8_t onReg = CalculateLEDTRegister(tOn);
	uint8_t offReg = CalculateLEDTRegister(tOff);

	SetupBlink(pin, onReg, offReg, onIntensity, offIntensity, 0, 0);
}

void SX1509::Breath(uint8_t pin, unsigned long tOn, unsigned long tOff, unsigned long rise, unsigned long fall, uint8_t onInt, uint8_t offInt, bool log)
{
	offInt = constrain(offInt, uint8_t(0), uint8_t(7));

	uint8_t onReg = CalculateLEDTRegister(tOn);
	uint8_t offReg = CalculateLEDTRegister(tOff);

	uint8_t riseTime = CalculateSlopeRegister(rise, onInt, offInt);
	uint8_t fallTime = CalculateSlopeRegister(fall, onInt, offInt);

	SetupBlink(pin, onReg, offReg, onInt, offInt, riseTime, fallTime, log);
}

void SX1509::SetupBlink(uint8_t pin, uint8_t tOn, uint8_t tOff, uint8_t onIntensity, uint8_t offIntensity, uint8_t tRise, uint8_t tFall, bool log)
{
	LedDriverInit(pin, log);

	// Keep parameters within their limits:
	tOn &= 0x1F;  // tOn should be a 5-bit value
	tOff &= 0x1F; // tOff should be a 5-bit value
	offIntensity &= 0x07;
	// Write the time on
	// 1-15:  TON = 64 * tOn * (255/ClkX)
	// 16-31: TON = 512 * tOn * (255/ClkX)
	WriteReg(REG_T_ON[pin], tOn);

	// Write the time/intensity off register
	// 1-15:  TOFF = 64 * tOff * (255/ClkX)
	// 16-31: TOFF = 512 * tOff * (255/ClkX)
	// linear Mode - IOff = 4 * offIntensity
	// log mode - Ioff = f(4 * offIntensity)
	WriteReg(REG_OFF[pin], (tOff << 3) | offIntensity);

	// Write the on intensity:
	WriteReg(REG_I_ON[pin], onIntensity);

	// Prepare tRise and tFall
	tRise &= 0x1F; // tRise is a 5-bit value
	tFall &= 0x1F; // tFall is a 5-bit value

	// Write regTRise
	// 0: Off
	// 1-15:  TRise =      (regIOn - (4 * offIntensity)) * tRise * (255/ClkX)
	// 16-31: TRise = 16 * (regIOn - (4 * offIntensity)) * tRise * (255/ClkX)
	if (REG_T_RISE[pin] != 0xFF)
		WriteReg(REG_T_RISE[pin], tRise);
	// Write regTFall
	// 0: off
	// 1-15:  TFall =      (regIOn - (4 * offIntensity)) * tFall * (255/ClkX)
	// 16-31: TFall = 16 * (regIOn - (4 * offIntensity)) * tFall * (255/ClkX)
	if (REG_T_FALL[pin] != 0xFF)
		WriteReg(REG_T_FALL[pin], tFall);
}

void SX1509::Keypad(uint8_t rows, uint8_t columns, uint16_t sleepTime, uint8_t scanTime, uint8_t debounceTime)
{
	uint16_t tempWord;
	uint8_t tempByte;

	// If clock hasn't been set up, set it to internal 2MHz
	if (_clkX == 0)
		Clock(INTERNAL_CLOCK_2MHZ);

	// Set regDir 0:7 outputs, 8:15 inputs:
	tempWord = ReadWord(REG_DIR_B);
	for (uint8_t i = 0; i < rows; i++)
		tempWord &= ~(1 << i);
	for (uint8_t i = 8; i < (columns * 2); i++)
		tempWord |= (1 << i);
	WriteWord(REG_DIR_B, tempWord);

	// Set regOpenDrain on 0:7:
	tempByte = ReadReg(REG_OPEN_DRAIN_A);
	for (uint8_t i = 0; i < rows; i++)
		tempByte |= (1 << i);
	WriteReg(REG_OPEN_DRAIN_A, tempByte);

	// Set regPullUp on 8:15:
	tempByte = ReadReg(REG_PULL_UP_B);
	for (uint8_t i = 0; i < columns; i++)
		tempByte |= (1 << i);
	WriteReg(REG_PULL_UP_B, tempByte);

	// Debounce Time must be less than scan time
	debounceTime = constrain(debounceTime, uint8_t(1), uint8_t(64));
	scanTime = constrain(scanTime, uint8_t(1), uint8_t(128));
	if (debounceTime >= scanTime)
	{
		debounceTime = scanTime >> 1; // Force debounceTime to be less than scanTime
	}
	DebounceKeypad(debounceTime, rows, columns);

	// Calculate scanTimeBits, based on scanTime
	uint8_t scanTimeBits = 0;
	for (uint8_t i = 7; i > 0; i--)
	{
		if (scanTime & (1 << i))
		{
			scanTimeBits = i;
			break;
		}
	}

	// Calculate sleepTimeBits, based on sleepTime
	uint8_t sleepTimeBits = 0;
	if (sleepTime != 0)
	{
		for (uint8_t i = 7; i > 0; i--)
		{
			if (sleepTime & ((uint16_t)1 << (i + 6)))
			{
				sleepTimeBits = i;
				break;
			}
		}
		// If sleepTime was non-zero, but less than 128,
		// assume we wanted to turn sleep on, set it to minimum:
		if (sleepTimeBits == 0)
			sleepTimeBits = 1;
	}

	// RegKeyConfig1 sets the auto sleep time and scan time per row
	sleepTimeBits = (sleepTimeBits & 0b111) << 4;
	scanTimeBits &= 0b111; // Scan time is bits 2:0
	tempByte = sleepTime | scanTimeBits;
	WriteReg(REG_KEY_CONFIG_1, tempByte);

	// RegKeyConfig2 tells the SX1509 how many rows and columns we've got going
	rows = (rows - 1) & 0b111;		 // 0 = off, 0b001 = 2 rows, 0b111 = 8 rows, etc.
	columns = (columns - 1) & 0b111; // 0b000 = 1 column, ob111 = 8 columns, etc.
	WriteReg(REG_KEY_CONFIG_2, (rows << 3) | columns);
}

uint16_t SX1509::ReadKeypad()
{
	return ReadKeyData();
}

uint16_t SX1509::ReadKeyData()
{
	return (0xFFFF ^ ReadWord(REG_KEY_DATA_1));
}

uint8_t SX1509::GetRow(uint16_t keyData)
{
	uint8_t rowData = uint8_t(keyData & 0x00FF);

	for (uint8_t i = 0; i < 8; i++)
	{
		if (rowData & (1 << i))
			return i;
	}
	return 0;
}

uint8_t SX1509::GetColumn(uint16_t keyData)
{
	uint8_t colData = uint8_t((keyData & 0xFF00) >> 8);

	for (uint8_t i = 0; i < 8; i++)
	{
		if (colData & (1 << i))
			return i;
	}
	return 0;
}

void SX1509::Sync(void)
{
	// First check if nReset functionality is set
	uint8_t regMisc = ReadReg(REG_MISC);
	if (!(regMisc & 0x04))
	{
		regMisc |= (1 << 2);
		WriteReg(REG_MISC, regMisc);
	}

	// Toggle nReset pin to sync LED timers	  // set reset pin as output
	resetPin.Write(LOW);  // pull reset pin low
	System::Delay(1);					  // Wait for the pin to settle
	resetPin.Write(HIGH); // pull reset pin back high

	// Return nReset to POR functionality
	WriteReg(REG_MISC, (regMisc & ~(1 << 2)));
}

void SX1509::DebounceConfig(uint8_t configValue)
{
	// First make sure clock is configured
	uint8_t tempByte = ReadReg(REG_MISC);
	if ((tempByte & 0x70) == 0)
	{
		tempByte |= (1 << 4); // Just default to no divider if not set
		WriteReg(REG_MISC, tempByte);
	}
	tempByte = ReadReg(REG_CLOCK);
	if ((tempByte & 0x60) == 0)
	{
		tempByte |= (1 << 6); // default to internal osc.
		WriteReg(REG_CLOCK, tempByte);
	}

	configValue &= 0b111; // 3-bit value
	WriteReg(REG_DEBOUNCE_CONFIG, configValue);
}

void SX1509::DebounceTime(uint8_t time)
{
	if (_clkX == 0)					   // If clock hasn't been set up.
		Clock(INTERNAL_CLOCK_2MHZ, 1); // Set clock to 2MHz.

	// Debounce time-to-byte map: (assuming fOsc = 2MHz)
	// 0: 0.5ms		1: 1ms
	// 2: 2ms		3: 4ms
	// 4: 8ms		5: 16ms
	// 6: 32ms		7: 64ms
	// 2^(n-1)
	uint8_t configValue = 0;
	// We'll check for the highest set bit position,
	// and use that for debounceConfig
	for (int8_t i = 7; i >= 0; i--)
	{
		if (time & (1 << i))
		{
			configValue = i + 1;
			break;
		}
	}
	configValue = constrain(configValue, uint8_t(0), uint8_t(7));

	DebounceConfig(configValue);
}

void SX1509::DebouncePin(uint8_t pin)
{
	uint16_t debounceEnable = ReadWord(REG_DEBOUNCE_ENABLE_B);
	debounceEnable |= (1 << pin);
	WriteWord(REG_DEBOUNCE_ENABLE_B, debounceEnable);
}

void SX1509::DebounceKeypad(uint8_t time, uint8_t numRows, uint8_t numCols)
{
	// Set up debounce time:
	DebounceTime(time);

	// Set up debounce pins:
	for (uint8_t i = 0; i < numRows; i++)
		DebouncePin(i);
	for (uint8_t i = 0; i < (8 + numCols); i++)
		DebouncePin(i);
}

void SX1509::EnableInterrupt(uint8_t pin, uint8_t riseFall)
{
	// Set REG_INTERRUPT_MASK
	uint16_t tempWord = ReadWord(REG_INTERRUPT_MASK_B);
	tempWord &= ~(1 << pin); // 0 = event on IO will trigger interrupt
	WriteWord(REG_INTERRUPT_MASK_B, tempWord);

	uint8_t sensitivity = 0;
	switch (riseFall)
	{
	case CHANGE:
		sensitivity = 0b11;
		break;
	case FALLING:
		sensitivity = 0b10;
		break;
	case RISING:
		sensitivity = 0b01;
		break;
	}

	// Set REG_SENSE_XXX
	// Sensitivity is set as follows:
	// 00: None
	// 01: Rising
	// 10: Falling
	// 11: Both
	uint8_t pinMask = (pin & 0x07) * 2;
	uint8_t senseRegister;

	// Need to select between two words. One for bank A, one for B.
	if (pin >= 8)
		senseRegister = REG_SENSE_HIGH_B;
	else
		senseRegister = REG_SENSE_HIGH_A;

	tempWord = ReadWord(senseRegister);
	tempWord &= ~(0b11 << pinMask);		  // Mask out the bits we want to write
	tempWord |= (sensitivity << pinMask); // Add our new bits
	WriteWord(senseRegister, tempWord);
}

uint16_t SX1509::InterruptSource(bool clear /* =true*/)
{
	uint16_t intSource = ReadWord(REG_INTERRUPT_SOURCE_B);
	if (clear)
		WriteWord(REG_INTERRUPT_SOURCE_B, 0xFFFF); // Clear interrupts
	return intSource;
}

bool SX1509::CheckInterrupt(uint8_t pin)
{
	if (InterruptSource(false) & (1 << pin))
		return true;

	return false;
}

void SX1509::Clock(uint8_t oscSource, uint8_t oscDivider, uint8_t oscPinFunction, uint8_t oscFreqOut)
{
	ConfigClock(oscSource, oscPinFunction, oscFreqOut, oscDivider);
}

void SX1509::ConfigClock(uint8_t oscSource /*= 2*/, uint8_t oscPinFunction /*= 0*/, uint8_t oscFreqOut /*= 0*/, uint8_t oscDivider /*= 1*/)
{
	// RegClock constructed as follows:
	//	6:5 - Oscillator frequency souce
	//		00: off, 01: external input, 10: internal 2MHz, 1: reserved
	//	4 - OSCIO pin function
	//		0: input, 1 ouptut
	//	3:0 - Frequency of oscout pin
	//		0: LOW, 0xF: high, else fOSCOUT = FoSC/(2^(RegClock[3:0]-1))
	oscSource = (oscSource & 0b11) << 5;		// 2-bit value, bits 6:5
	oscPinFunction = (oscPinFunction & 1) << 4; // 1-bit value bit 4
	oscFreqOut = (oscFreqOut & 0b1111);			// 4-bit value, bits 3:0
	uint8_t regClock = oscSource | oscPinFunction | oscFreqOut;
	WriteReg(REG_CLOCK, regClock);

	// Config RegMisc[6:4] with oscDivider
	// 0: off, else ClkX = fOSC / (2^(RegMisc[6:4] -1))
	oscDivider = constrain(oscDivider, uint8_t(1), uint8_t(7));
	_clkX = 2000000.0 / (1 << (oscDivider - 1)); // Update private clock variable
	oscDivider = (oscDivider & 0b111) << 4;		 // 3-bit value, bits 6:4

	uint8_t regMisc = ReadReg(REG_MISC);
	regMisc &= ~(0b111 << 4);
	regMisc |= oscDivider;
	WriteReg(REG_MISC, regMisc);
}
// ---------------------------------------------------------------
// ---------------------------------------------------------------

// uint8_t readByte
uint8_t SX1509::ReadReg(uint8_t registerAddress)
    {
        uint8_t data[1];
        transport.i2c_.ReadDataAtAddress(
            transport.i2c_address_, registerAddress, 
            1, data, 1, transport.timeout);
        return data[0];
    }

// uint16_t readWord
uint16_t SX1509::ReadWord(uint8_t registerAddress)
    {
        uint8_t data[2];
		uint16_t word;
        transport.i2c_.ReadDataAtAddress(
            transport.i2c_address_, registerAddress, 
            1, data, 2, transport.timeout);
		word = (data[0] << 8) | data[1];
        return word;
    }

// bool SX1509::readBytes
bool SX1509::ReadRegistersOk(uint8_t firstRegisterAddress, uint8_t *destination, uint8_t length){
	I2CHandle::Result result = transport.i2c_.ReadDataAtAddress(
		transport.i2c_address_, firstRegisterAddress, 
		length, destination, length, transport.timeout);
	return result == I2CHandle::Result::OK;
}

// uint8_t readByte
bool SX1509::ReadReg(uint8_t registerAddress, uint8_t *value)
{
    return ReadRegistersOk(registerAddress, value, 1);
}

// uint16_t readWord
bool SX1509::ReadWord(uint8_t registerAddress, uint16_t *value){
	uint8_t data[2] = {0x00, 0x00};
	if (ReadRegistersOk(registerAddress, data, 2)) 
	{
		*value = (data[0] << 8) | data[1];
		return true;
	}
	return false;
}

// bool writeBytes
bool SX1509::WriteRegisters(uint8_t firstRegisterAddress, uint8_t *writeArray, uint8_t length)
{
	I2CHandle::Result result = transport.i2c_.WriteDataAtAddress(
		transport.i2c_address_, firstRegisterAddress, 
		length, writeArray, length, transport.timeout);
    return result == I2CHandle::Result::OK;
}
// bool writeByte
bool SX1509::WriteReg(uint8_t registerAddress, uint8_t writeValue)
    {
		I2CHandle::Result result = transport.i2c_.WriteDataAtAddress(
			transport.i2c_address_, registerAddress, 
			1, &writeValue, 1, transport.timeout);
		return result == I2CHandle::Result::OK;
    }

// bool writeWord
bool SX1509::WriteWord(uint8_t registerAddress, uint16_t writeValue)
{
	uint8_t data[2] = {0x00, 0x00};
	data[0] = (writeValue & 0xFF00) >> 8;
	data[1] = (writeValue & 0x00FF);

	I2CHandle::Result result = transport.i2c_.WriteDataAtAddress(
		transport.i2c_address_, registerAddress, 
		1, data, 2, transport.timeout);
	return result == I2CHandle::Result::OK;
}

// Helper functions:
// CalculateLEDTRegister - Tries to estimate an LED on/off duration register,
// given the number of milliseconds and LED clock frequency.
uint8_t SX1509::CalculateLEDTRegister(unsigned long ms)
{
	uint8_t regOn1, regOn2;
	float timeOn1, timeOn2;

	if (_clkX == 0)
		return 0;

	regOn1 = (float)(ms / 1000.0) / (64.0 * 255.0 / (float)_clkX);
	regOn2 = regOn1 / 8;
	regOn1 = constrain(regOn1, uint8_t(1), uint8_t(15));
	regOn2 = constrain(regOn2, uint8_t(16), uint8_t(31));

	timeOn1 = 64.0 * regOn1 * 255.0 / _clkX * 1000.0;
	timeOn2 = 512.0 * regOn2 * 255.0 / _clkX * 1000.0;

	if (abs(timeOn1 - ms) < abs(timeOn2 - ms))
		return regOn1;
	else
		return regOn2;
}

// CalculateSlopeRegister - Tries to estimate an LED rise/fall duration
// register, given the number of milliseconds and LED clock frequency.
uint8_t SX1509::CalculateSlopeRegister(unsigned long ms, uint8_t onIntensity, uint8_t offIntensity)
{
	uint16_t regSlope1, regSlope2;
	float regTime1, regTime2;

	if (_clkX == 0)
		return 0;

	float tFactor = ((float)onIntensity - (4.0 * (float)offIntensity)) * 255.0 / (float)_clkX;
	float timeS = float(ms) / 1000.0;

	regSlope1 = timeS / tFactor;
	regSlope2 = regSlope1 / 16;

	regSlope1 = constrain(regSlope1, uint16_t(1), uint16_t(15));
	regSlope2 = constrain(regSlope2, uint16_t(16), uint16_t(31));

	regTime1 = regSlope1 * tFactor * 1000.0;
	regTime2 = 16 * regTime1;

	if (abs(regTime1 - ms) < abs(regTime2 - ms))
		return regSlope1;
	else
		return regSlope2;
}