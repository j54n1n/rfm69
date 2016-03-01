/*******************************************************************************
The MIT License (MIT)

Copyright (c) 2014 Jean-Claude Wippler
Copyright (c) 2014 Felix Rusu
Copyright (c) 2015 André Heßling
Copyright (c) 2016 Julian Sanin

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*******************************************************************************/

#ifndef RFM_69_H
#define RFM_69_H

#include <stdint.h>
#include <stdlib.h>

#ifdef ARDUINO
#include <Arduino.h>
#endif

//! Available RFM69 models.
enum Rfm69Model {
	Rfm69ModelW, //!< Model RFM69W max. transmit power +13dBm.
	Rfm69ModelCW = Rfm69ModelW, //!< Model RFM69CW max. transmit power +13dBm.
	Rfm69ModelHW, //!< Model RFM69HW max. transmit power +20dBm.
	Rfm69ModelHCW = Rfm69ModelHW //!< Model RFM69HCW max. transmit power +20dBm.
};

//! Native mode RF69 driver. See also http://jeelabs.org/book/1522c/ and
//! https://github.com/ahessling/RFM69-STM32.
template<typename SpiDevice, Rfm69Model RFM69_MODEL = Rfm69ModelW>
class Rfm69 {

	enum {
		REG_FIFO = 0x00,
		REG_OPMODE = 0x01,
		REG_FRFMSB = 0x07,
		REG_OSC1 = 0x0A,
		REG_PALEVEL = 0x11,
		REG_OCP = 0x13,
		REG_LNAVALUE = 0x18,
		REG_AFCMSB = 0x1F,
		REG_AFCLSB = 0x20,
		REG_FEIMSB = 0x21,
		REG_FEILSB = 0x22,
		REG_RSSIVALUE = 0x24,
		REG_IRQFLAGS1 = 0x27,
		REG_IRQFLAGS2 = 0x28,
		REG_SYNCVALUE1 = 0x2F,
		REG_SYNCVALUE2 = 0x30,
		REG_NODEADDR = 0x39,
		REG_BCASTADDR = 0x3A,
		REG_FIFOTHRESH = 0x3C,
		REG_PKTCONFIG2 = 0x3D,
		REG_AESKEYMSB = 0x3E,
		REG_TEMP1 = 0x4E,
		REG_TEMP2 = 0x4F,
		REG_TESTPA1 = 0x5A,
		REG_TESTPA2 = 0x5C,

		MODE_SLEEP = 0 << 2,
		MODE_STANDBY = 1 << 2,
		MODE_TRANSMIT = 3 << 2,
		MODE_RECEIVE = 4 << 2,

		START_TX = 0xC2,
		STOP_TX = 0x42,

		IRQ1_MODEREADY = 1 << 7,
		IRQ1_RXREADY = 1 << 6,
		IRQ1_SYNADDRMATCH = 1 << 0,

		IRQ2_FIFONOTEMPTY = 1 << 6,
		IRQ2_PACKETSENT = 1 << 3,
		IRQ2_PAYLOADREADY = 1 << 2,
	};

	void setMode(uint8_t newMode);
	void configure(const uint8_t* p);
	void setFrequency(uint32_t freq);

	uint8_t readReg(uint8_t address) {
		return spi.transferRegister(address, 0);
	}
	void writeReg(uint8_t address, uint8_t value) {
		spi.transferRegister(address | 0x80, value);
	}

	SpiDevice spi;
	volatile uint8_t mode;
	bool paBoost;
	uint8_t rssi;
	uint8_t lna;

public:
	int16_t afc;
	uint8_t myId;
	uint8_t parity;

	void init(uint8_t id, uint8_t group, int freq);
	void encrypt(const char* key);
	//! Set the transmit power in dBm.
	//! For W/CW models valid values are from -18dB (min.) to +13dBm (max.).
	//! For HW/HCW models valid values are from -2dB (min.) to +20dBm (max.).
	//! \returns EXIT_FAILURE for invalid values or otherwise EXIT_SUCCESS.
	int setTransmitPower(int8_t dBm);
	//! Retrive the automatic LNA gain setting from the last received packet.
	//! \returns the gain ranging from +0dB (max. gain) to -48dB (min. gain).
	//! \returns a non-negative number greater than 0 in case an error occurred.
	int8_t getLnaGain(void);
	//! Retrive the strength indicator in dBm of the last received packet.
	//! \returns a value ranging from -115dBm (weak) to +0dBm (strong).
	int8_t getRssiValue(void);
	//! Retrieve the temperature in degrees Celsius (°C) of transceiver chip.
	//! \param offsetCalibration adds a correction offset to the reading in °C.
	//! Recalibration of the internal oscillator is recommended in case of large
	//! temperature changes.
	//! \see calibrateOscillator
	//! \returns the internal chip temperature in °C.
	int8_t getTemperature(int8_t offsetCalibration = 0);
	//! Recalibrates the internal oscillator of the transceiver chip.
	//! Recalibration of the internal oscillator is recommended in case of large
	//! temperature changes.
	//! \see getTemperature
	void calibrateOscillator(void);

	int receive(void* ptr, int length);
	void send(uint8_t header, const void* ptr, int length);
	void sleep();
};

// driver implementation

template<typename SpiDevice, Rfm69Model RFM69_MODEL>
void Rfm69<SpiDevice, RFM69_MODEL>::setMode(uint8_t newMode) {
	const uint8_t REG_OCP_ON = 0x1A;
	const uint8_t REG_OCP_OFF = 0x0F;
	const uint8_t REG_TESTPA1_HIPOWER = 0x5D;
	const uint8_t REG_TESTPA1_LOPOWER = 0x55;
	const uint8_t REG_TESTPA2_HIPOWER = 0x7C;
	const uint8_t REG_TESTPA2_LOPOWER = 0x70;
	mode = newMode;
	if ((RFM69_MODEL == Rfm69ModelHW) && (mode == MODE_TRANSMIT) && paBoost) {
		writeReg(REG_OCP, REG_OCP_OFF);
		writeReg(REG_TESTPA1, REG_TESTPA1_HIPOWER);
		writeReg(REG_TESTPA2, REG_TESTPA2_HIPOWER);
	}
	else {
		writeReg(REG_OCP, REG_OCP_ON);
		writeReg(REG_TESTPA1, REG_TESTPA1_LOPOWER);
		writeReg(REG_TESTPA2, REG_TESTPA2_LOPOWER);
	}
	writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | newMode);
	while ((readReg(REG_IRQFLAGS1) & IRQ1_MODEREADY) == 0)
		;
}

template<typename SpiDevice, Rfm69Model RFM69_MODEL>
void Rfm69<SpiDevice, RFM69_MODEL>::setFrequency(uint32_t hz) {
	// accept any frequency scale as input, including KHz and MHz
	// multiply by 10 until freq >= 100 MHz (don't specify 0 as input!)
	while (hz < 100000000)
		hz *= 10;

	// Frequency steps are in units of (32,000,000 >> 19) = 61.03515625 Hz
	// use multiples of 64 to avoid multi-precision arithmetic, i.e. 3906.25 Hz
	// due to this, the lower 6 bits of the calculated factor will always be 0
	// this is still 4 ppm, i.e. well below the radio's 32 MHz crystal accuracy
	// 868.0 MHz = 0xD90000, 868.3 MHz = 0xD91300, 915.0 MHz = 0xE4C000
	uint32_t frf = (hz << 2) / (32000000L >> 11);
	writeReg(REG_FRFMSB, frf >> 10);
	writeReg(REG_FRFMSB + 1, frf >> 2);
	writeReg(REG_FRFMSB + 2, frf << 6);
}

template<typename SpiDevice, Rfm69Model RFM69_MODEL>
void Rfm69<SpiDevice, RFM69_MODEL>::configure(const uint8_t* p) {
	while (true) {
		uint8_t cmd = p[0];
		if (cmd == 0)
			break;
		writeReg(cmd, p[1]);
		p += 2;
	}
}

template<typename SpiDevice, Rfm69Model RFM69_MODEL>
void Rfm69<SpiDevice, RFM69_MODEL>::init(uint8_t id, uint8_t group, int freq) {
	static const uint8_t configRegs[] = {
		// POR value is better for first rf_sleep  0x01, 0x00, // OpMode = sleep
		0x02, 0x00, // DataModul = packet mode, fsk
		0x03, 0x02, // BitRateMsb, data rate = 49,261 khz
		0x04, 0x8A, // BitRateLsb, divider = 32 MHz / 650
		0x05, 0x02, // FdevMsb = 45 KHz
		0x06, 0xE1, // FdevLsb = 45 KHz
		0x0B, 0x20, // Low M
		0x19, 0x4A, // RxBw 100 KHz
		0x1A, 0x42, // AfcBw 125 KHz
		0x1E, 0x0C, // AfcAutoclearOn, AfcAutoOn
					//0x25, 0x40, //0x80, // DioMapping1 = SyncAddress (Rx)
		0x26, 0x07, // disable clkout
		0x29, 0xA0, // RssiThresh -80 dB
		0x2D, 0x05, // PreambleSize = 5
		0x2E, 0x88, // SyncConfig = sync on, sync size = 2
		0x2F, 0x2D, // SyncValue1 = 0x2D
		0x37, 0xD0, // PacketConfig1 = fixed, white, no filtering
		0x38, 0x42, // PayloadLength = 0, unlimited
		0x3C, 0x8F, // FifoTresh, not empty, level 15
		0x3D, 0x12, // 0x10, // PacketConfig2, interpkt = 1, autorxrestart off
		0x6F, 0x20, // TestDagc ...
		0x71, 0x02, // RegTestAfc
		0
	};

	myId = id;

	// b7 = group b7^b5^b3^b1, b6 = group b6^b4^b2^b0
	parity = group ^ (group << 4);
	parity = (parity ^ (parity << 2)) & 0xC0;

	spi.master();
	do {
		writeReg(REG_SYNCVALUE1, 0xAA);
	} while (readReg(REG_SYNCVALUE1) != 0xAA);
	do {
		writeReg(REG_SYNCVALUE1, 0x55);
	} while (readReg(REG_SYNCVALUE1) != 0x55);

	configure(configRegs);
	configure(configRegs); // TODO why is this needed ???
	setFrequency(freq);

	writeReg(REG_SYNCVALUE2, group);

	// Set best power amplifier value that is common for both W and HW models.
	setTransmitPower(+13/*dBm*/);
}

template<typename SpiDevice, Rfm69Model RFM69_MODEL>
void Rfm69<SpiDevice, RFM69_MODEL>::encrypt(const char* key) {
	uint8_t cfg = readReg(REG_PKTCONFIG2) & ~0x01;
	if (key) {
		for (int i = 0; i < 16; ++i) {
			writeReg(REG_AESKEYMSB + i, *key);
			if (*key != 0)
				++key;
		}
		cfg |= 0x01;
	}
	writeReg(REG_PKTCONFIG2, cfg);
}

template<typename SpiDevice, Rfm69Model RFM69_MODEL>
int Rfm69<SpiDevice, RFM69_MODEL>::setTransmitPower(int8_t dBm) {
	const int8_t LO_POWER_PA0_MIN = -18; // dBm
	const int8_t HI_POWER_PA1_MIN = -2; // dBm
	const int8_t LO_POWER_PA0_MAX = +13; // dBm
	const int8_t HI_POWER_PA1_MAX = LO_POWER_PA0_MAX;
	const int8_t HI_POWER_PA1_PA2_MAX = +17; // dBm
	const int8_t HI_POWER_PABOOST_MAX = +20; // dBm
	const uint8_t LO_POWER_PA0_OFFSET = +18U;
	const uint8_t HI_POWER_PA1_OFFSET = LO_POWER_PA0_OFFSET;
	const uint8_t HI_POWER_PA1_PA2_OFFSET = +14U;
	const uint8_t HI_POWER_PABOOST_OFFSET = +11U;
	const uint8_t REG_PA0_ON = 0x80;
	const uint8_t REG_PA1_ON = 0x40;
	const uint8_t REG_PA1_PA2_ON = 0x60;
	// Allow only valid power ranges.
	if ((dBm < LO_POWER_PA0_MIN) || (dBm > HI_POWER_PABOOST_MAX)) {
		return EXIT_FAILURE;
	}
	// Low power devices support transmit powers from -18dBm to +13dBm.
	if ((RFM69_MODEL == Rfm69ModelW) && (dBm > LO_POWER_PA0_MAX)) {
		return EXIT_FAILURE;
	}
	// High power devices support transmit powers from -2dBm to +20dBm.
	if ((RFM69_MODEL == Rfm69ModelHW) && (dBm < HI_POWER_PA1_MIN)) {
		return EXIT_FAILURE;
	}
	if (RFM69_MODEL == Rfm69ModelW) {
		// Only power amplifier PA0 is allowed.
		uint8_t powerLevel = LO_POWER_PA0_OFFSET + dBm;
		writeReg(REG_PALEVEL, REG_PA0_ON | powerLevel);
	}
	if (RFM69_MODEL == Rfm69ModelHW) {
		if ((dBm >= HI_POWER_PA1_MIN) && (dBm <= HI_POWER_PA1_MAX)) {
			// Use power amplifier PA1 on PABOOST.
			uint8_t powerLevel = HI_POWER_PA1_OFFSET + dBm;
			writeReg(REG_PALEVEL, REG_PA1_ON | powerLevel);
		}
		else if ((dBm > HI_POWER_PA1_MAX) && (dBm <= HI_POWER_PA1_PA2_MAX)) {
			// Use combined power amplifiers PA1 and PA2 on PABOOST.
			uint8_t powerLevel = HI_POWER_PA1_PA2_OFFSET + dBm;
			writeReg(REG_PALEVEL, REG_PA1_PA2_ON | powerLevel);
		}
		else {
			// Use PA1 and PA2 with high power amplifier on PABOOST.
			// Activate the afterburner ^^.
			uint8_t powerLevel = HI_POWER_PABOOST_OFFSET + dBm;
			writeReg(REG_PALEVEL, REG_PA1_PA2_ON | powerLevel);
			paBoost = true;
			return EXIT_SUCCESS;
		}
	}
	paBoost = false;
	return EXIT_SUCCESS;
}

template<typename SpiDevice, Rfm69Model RFM69_MODEL>
void Rfm69<SpiDevice, RFM69_MODEL>::sleep() {
	setMode(MODE_SLEEP);
}

template<typename SpiDevice, Rfm69Model RFM69_MODEL>
int Rfm69<SpiDevice, RFM69_MODEL>::receive(void* ptr, int length) {
	if (mode != MODE_RECEIVE)
		setMode(MODE_RECEIVE);
	else {
		static uint8_t lastFlag;
		if ((readReg(REG_IRQFLAGS1) & IRQ1_RXREADY) != lastFlag) {
			lastFlag ^= IRQ1_RXREADY;
			if (lastFlag) { // flag just went from 0 to 1
				rssi = readReg(REG_RSSIVALUE);
				lna = (readReg(REG_LNAVALUE) >> 3) & 0x7;
				afc = readReg(REG_AFCMSB) << 8;
				afc |= readReg(REG_AFCLSB);
			}
		}

		if (readReg(REG_IRQFLAGS2) & IRQ2_PAYLOADREADY) {

			int count = readReg(REG_FIFO);
			for (int i = 0; i < count; ++i) {
				uint8_t v = readReg(REG_FIFO);
				if (i < length)
					((uint8_t*)ptr)[i] = v;
			}

			// only accept packets intended for us, or broadcasts
			// ... or any packet if we're the special catch-all node
			uint8_t dest = *(uint8_t*)ptr;
			if ((dest & 0xC0) == parity) {
				uint8_t destId = dest & 0x3F;
				if (destId == myId || destId == 0 || myId == 63)
					return count;
			}
		}
	}
	return -1;
}

template<typename SpiDevice, Rfm69Model RFM69_MODEL>
int8_t Rfm69<SpiDevice, RFM69_MODEL>::getLnaGain(void) {
	const int8_t LNA_GAINS[] = { 0x7F/*reserved*/,
		0/*dB*/, -6/*dB*/, -12/*dB*/, -24/*dB*/, -36/*dB*/, -48/*dB*/
	};
	if (lna >= sizeof(LNA_GAINS)) {
		return LNA_GAINS[0];
	}
	return LNA_GAINS[lna];
}

template<typename SpiDevice, Rfm69Model RFM69_MODEL>
int8_t Rfm69<SpiDevice, RFM69_MODEL>::getRssiValue(void) {
	return -rssi / 2;
}

template<typename SpiDevice, Rfm69Model RFM69_MODEL>
void Rfm69<SpiDevice, RFM69_MODEL>::send(
		uint8_t header, const void* ptr, int length) {
	setMode(MODE_SLEEP);

	writeReg(REG_FIFO, length + 2);
	writeReg(REG_FIFO, (header & 0x3F) | parity);
	writeReg(REG_FIFO, (header & 0xC0) | myId);
	for (int i = 0; i < length; ++i)
		writeReg(REG_FIFO, ((const uint8_t*)ptr)[i]);

	setMode(MODE_TRANSMIT);
	while ((readReg(REG_IRQFLAGS2) & IRQ2_PACKETSENT) == 0) {
		// TODO: Do something else than busy wait.
	}
	setMode(MODE_STANDBY);
}

template<typename SpiDevice, Rfm69Model RFM69_MODEL>
int8_t Rfm69<SpiDevice, RFM69_MODEL>::getTemperature(int8_t offsetCalibration) {
	const uint8_t TEMP_MEAS_START = 0x08;
	const uint8_t TEMP_MEAS_RUNNING = 0x04;
	const int8_t TEMP_COEFFICIENT = -90; // °C.
	setMode(MODE_STANDBY);
	writeReg(REG_TEMP1, TEMP_MEAS_START);
	while (readReg(REG_TEMP1) & TEMP_MEAS_RUNNING);
	return (~readReg(REG_TEMP2) + TEMP_COEFFICIENT + offsetCalibration);
}

template<typename SpiDevice, Rfm69Model RFM69_MODEL>
void Rfm69<SpiDevice, RFM69_MODEL>::calibrateOscillator(void) {
	const uint8_t RC_CAL_START = 0x80;
	const uint8_t RC_CAL_DONE = 0x40;
	setMode(MODE_STANDBY);
	writeReg(REG_OSC1, RC_CAL_START);
	while ((readReg(REG_OSC1) & RC_CAL_DONE) == 0); // Wait till calibrated.
}

#endif
