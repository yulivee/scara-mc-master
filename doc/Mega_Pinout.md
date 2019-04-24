### Mega Pinout

|Pin|Name|Function|
|:-:|:-:|:-:|
|18|TX1||
|19|RX1||
|22||Slave select Pin 1|
|23||Slave select Pin 2|
|24||Slave select Pin 3|
|25||Slave select Pin 4|
|26||Slave select Pin 5|
|27||Slave select Pin 6|
|28||Slave select Pin 7|
|13||Led Onboard|

ss_pin[7] = {22,23,24,25,26,27,28}; //Slave select lines tell slaves when they can use the bus
const int led_pin = 13;
