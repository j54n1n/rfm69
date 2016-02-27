#ifndef SPI_DEVICE_H
#define SPI_DEVICE_H

#include <stdint.h>

#if !defined(ARDUINO)
#error Architecture not supported!
#endif


#ifdef ARDUINO
#include <Arduino.h>
#include <SPI.h>
#endif

enum SpiBitOrder {
#ifdef ARDUINO
	SpiBitOrderLsbFirst = LSBFIRST,
	SpiBitOrderMsbFirst = MSBFIRST
#endif
};

enum SpiMode {
#ifdef ARDUINO
	SpiMode0 = SPI_MODE0,
	SpiMode1 = SPI_MODE1,
	SpiMode2 = SPI_MODE2,
	SpiMode3 = SPI_MODE3
#endif
};

template<
	uint8_t PIN_SS,
	uint32_t F_SCK = 4000000/*Hz*/,
	SpiBitOrder BIT_ORDER = SpiBitOrderMsbFirst,
	SpiMode MODE = SpiMode0>
class SpiDevice {

public:
	static void master(void) {
#ifdef ARDUINO
		digitalWrite(PIN_SS, HIGH);
		pinMode(PIN_SS, OUTPUT);
		SPI.begin();
#endif
	}

	static uint8_t transferRegister(uint8_t command, uint8_t value) {
#ifdef ARDUINO
		SPI.beginTransaction(SPISettings(F_SCK, BIT_ORDER, MODE));
		digitalWrite(PIN_SS, LOW);
		SPI.transfer(command);
		uint8_t in = SPI.transfer(value);
		digitalWrite(PIN_SS, HIGH);
		SPI.endTransaction();
		return in;
#else
		return 0;
#endif
	}
};

#endif // SPI_DEVICE_H
