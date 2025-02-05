## Hardware abstraction layer for the RFM69 radio for PI PICO
Using https://github.com/e-mo/rfm69_rp2040 as hardware driver.  
  
This library intends to make it easy to configure and use the rfm69 radio in interrupt mode with a pi pico.  
  
To test this module check out the examples.  
Note that the examples pulls in a couple of submodules using CMAKE FetchContent. Check the the examples/example/CMakeLists.txt, for source repos.  
  
## Connect the RFM69 chip to Pi Pico As follows:
  
PICO 16 - MISO  
PICO 17 - NSS  
PICO 18 - SCK  
PICO 19 - MOSI  
PICO 20 - RESET  
PICO 21 - DIO0  
  
It works well but I intend to keep on working on this library to improve performance and add more features.