# Source code for VO2Max project
This directory contains the source code for the VO2Max project.

Source code is written for ESP32 with PlatformIO (Core 6.1.18, Home 3.4.4).

## PlatformIO board settings
The settings are defined in ```platformio.ini``` but the key values are listed here also.

 * Board: Lilygo-t-display
 * Upload Speed: 921600
 * CPU Frequency: 240Mhz (WiFi/BT)
 * Flash Frequency: 80Mhz
 * Flash Mode: QIO
 * Partition Scheme: Default.csv [4MB with spiffs (1.2MB APP/1.5 SPIFFS)]
 * Core Debug Level: None --> For debugging can set also different values, for release select None
 * PSRAM: No

## External library requrirements
 * TFT_eSPI (tested with version 2.5.34)
 * NimBLE_Arduino (tested with version 1.4.1)

## Other stuff
Remember to modify the TFT_eSPI configuration file to select T-Display! File should be located in ```.pio/libdeps/lilygo-t-display/TFT_eSPI/User_Setup_Select.h```. The correct line to uncomment is 
```
#include <User_Setups/Setup25_TTGO_T_Display.h>    // Setup file for ESP32 and TTGO T-Display ST7789V SPI bus TFT
```

Don't forget to comment out the default custom ```User_Setup.h``` configuration file
```
//#include <User_Setup.h>           // Default setup is root library folder
```

With NimBLE-Arduino the code just barely fits into default partition (1.25 MB App, 1.375 MB SPIFFS).

It would be possible to change to huge_app (3 MB app, 0.875 MB SPIFFS) if more application space is needed.


## Library requirements
### External
 * TFT_eSPI @ 2.5.43
 * NimBLE-Arduino @ 1.4.1

### Built into ESP32 platform
 * EEPROM @ 2.0.0
 * ESP32 Async UDP @ 2.0.0
 * WiFi @ 2.0.0
