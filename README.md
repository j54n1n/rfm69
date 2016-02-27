# rfm69
RFM69 native packet mode driver

## Instructions (Arduino)
* Include the library by adding following headers to your sketch:
```
#include <SPI.h>
#include <SpiDevice.h>
#include <Rfm69.h>
```
* then declare an instance of the RFM69 radio driver:
```
Rfm69<SpiDevice<10> > rfm69;
```
* Here the `SpiDevice` represents the  SPI device abstraction of the attached
RFM69 radio. In this case for example the radio was attached to the SPI bus with
the Slave Select line connected to Arduino digital pin number 10.
* The initalisation routine takes as arguments the *Node ID*, the *Group ID*,
and the *Transceiver Frequency*. For example following code will setup *Node ID*
= 28, *Group ID* = 42, and *Transceiver Frequency* = 868,6MHz:
```
rfm69.init(28, 42, 8686);
```
* The *Node ID* can range from 1 to 60. *Node ID* = 61 is for send only nodes.
*Node ID* = 62 is reserved, and *Node ID* = 63 is for receive-everything nodes.
* An PSK encryption can be set up by calling `rfm69.encrypt(myPassword)` with a
string *myPassword* of 1 to 16 characters. Calling `rfm69.encrypt(0)` will
disable encryption.
* Transmit power can be reduced by calling `rfm69.txPower(myLevel)` where
*myLevel* is a value ranging from 0 to 31. 31 is the default maximum power
level.
* Sending and receiving methods allow up to 62 bytes of data. The special header
byte is normally set to 0. It encodes in bits from 0 to 5 either the destination
or origin Node ID and in bits from 6 to 7 some status flags.

### Further reading
http://jeelabs.org/book/1513d
http://jeelabs.org/book/1522c
http://jeelabs.org/wp-content/uploads/2015/05/20/rfm69-on-raspberry-pi
http://jeelabs.org/2015/05/27/rfm69-on-atmega/