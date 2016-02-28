/*******************************************************************************
The MIT License(MIT)

Copyright(c) 2014 Jean - Claude Wippler
Copyright(c) 2015 André Heßling
Copyright(c) 2016 Julian Sanin

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files(the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and / or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*******************************************************************************/

// Report received data on the serial port.
#include <SPI.h>
#include <SpiDevice.h>
#include <Rfm69.h>

Rfm69<SpiDevice<10> > rfm69;

uint8_t rxBuf[64];
uint8_t txBuf[62];
uint16_t cnt = 0;

void setup() {
	Serial.begin(57600);
	Serial.println("\n[rf69demo]");

	rfm69.init(28, 42, 8686);
	//rfm69.encrypt("mysecret");
	rfm69.setTransmitPower(-2/*dBm*/); // For RFM69W from -18dBm to +13dBm.

	for (int i = 0; i < (int) sizeof txBuf; ++i)
		txBuf[i] = i;
}

void loop() {
	if (++cnt == 0) {
		int txLen = ++txBuf[0] % (sizeof txBuf + 1);
		Serial.print(" > #");
		Serial.print(txBuf[0]);
		Serial.print(", ");
		Serial.print(txLen);
		Serial.println("b");
		rfm69.send(0, txBuf, txLen);
	}

	int len = rfm69.receive(rxBuf, sizeof rxBuf);
	if (len >= 0) {
		Serial.print("OK ");
		for (int i = 0; i < len; ++i) {
			Serial.print(rxBuf[i] >> 4, HEX);
			Serial.print(rxBuf[i] & 0xF, HEX);
		}
		Serial.print(" (");
		Serial.print(rfm69.rssi);
		Serial.print(rfm69.afc < 0 ? "" : "+");
		Serial.print(rfm69.afc);
		Serial.print(":");
		Serial.print(rfm69.lna);
		Serial.println(")");
	}
}