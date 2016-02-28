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
* Transmit power can be modified by calling `rfm69.setTransmitPower(myPower)`
where *myPower* is a value ranging from -18dBm to +13dBm for RFM69W models and
from -2dBm to +20dBm for RFM69HW models. +13dBm is the default power setting for
both models. The functions returns either `stdlib.h` `EXIT_SUCCESS` or
`EXIT_FAILURE` if the power setting is outside of the limits. To enable the
higher power settings for the HW modules at the declaration must be appended
either `Rfm69ModelHW` or `Rfm69ModelHCW`. For example:
```
Rfm69<SpiDevice<10>, Rfm69ModelHW> rfm69;
rfm69.setTransmitPower(+17/*dBm*/);
```
* Pay attention that the +20dBm power setting may be not legal in your country.
Please check your regulations for telecomunications.
* Sending and receiving methods allow up to 62 bytes of data. The special header
byte is normally set to 0. It encodes in bits from 0 to 5 either the destination
or origin Node ID and in bits from 6 to 7 some status flags.

### Further reading
* http://jeelabs.org/book/1513d
* http://jeelabs.org/book/1522c
* http://jeelabs.org/wp-content/uploads/2015/05/20/rfm69-on-raspberry-pi
* http://jeelabs.org/2015/05/27/rfm69-on-atmega/
* http://blog.andrehessling.de/2015/02/07/figuring-out-the-power-level-settings-of-hoperfs-rfm69-hwhcw-modules/