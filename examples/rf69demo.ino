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
	rfm69.txPower(15); // 0 = min .. 31 = max

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