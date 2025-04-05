## Hardware abstraction layer for the RFM69 radio for PI PICO
Using https://github.com/e-mo/rfm69_rp2040 as hardware driver.  
  
This library intends to make it easy to configure and use the rfm69 radio in interrupt or blocking mode mode with a pi pico 1 or 2.
  
To test this module check out the examples.  
Note that the examples pulls in a couple of submodules using CMAKE FetchContent. Check the the examples/example/CMakeLists.txt, for source repos.  
  
## Connect the RFM69 chip to Pi Pico As follows to run in Non-Blocking mode:
To use the non blocking functions interrupt pins must be connected.  
  
PICO 16 - MISO  
PICO 17 - NSS  
PICO 18 - SCK  
PICO 19 - MOSI  
PICO 20 - RESET  
PICO 21 - DIO0  
PICO 15 - DIO1

For the non blocking code to work the halRadioProcess must be called repeatedly in the system while(1) loop.  
Avoid any lengthy blocking code in main loop for it to work properly.  
  
The blocking functions can be used as long as the all Non-Blocking functions are inactive.

## Connect the RFM69 chip to Pi Pico As follows to run in Blocking mode:
If only the blocking functions are used no interrupt pins are necessary.  

PICO 16 - MISO  
PICO 17 - NSS  
PICO 18 - SCK  
PICO 19 - MOSI  
PICO 20 - RESET  

There is no need to call the halRadioProcess when used in pure blocking mode.  
  
## Known limitations and planned future work
The library seems to be stable and work well. I have done quite extensive testing to verify functionality, but there might still be some quirks.  

An obvious issue with this library is current consumption of the rfm69 radio. It is never put to sleep, only stand by mode. I might look into this in the future.
  
The usage halRadioProcess requires a high polling pace to keep the Non-Blocking functions working. I might look into using a more event based approach in the future to remove the need to continous polling.  
  
The access to the SPI interface is allways blocking forcing the processor to quite lengthy wait idle cases. I might look into using non blocking DMA access to the spi interface.  
  
The set radio mode calls are slow, ~100us making those non blocking to free up the processor for other task could improve the performance of the library.  