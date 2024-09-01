# Source code for VO2Max project
This directory contains the source code for the VO2Max project.

Source code is written for ESP32 with Arduino IDE 2.3.2.

## Arduino IDE board settings
 * Board: ESP32 Dev Module
 * Upload Speed: 921600
 * CPU Frequency: 240Mhz (WiFi/BT)
 * Flash Frequency: 80Mhz
 * Flash Mode: QIO
 * Flash Size: 4MB (32Mb)
 * Partition Scheme: Default 4MB with spiffs (1.2MB APP/1.5 SPIFFS)
 * Core Debug Level: None --> For debugging can set also different values, for release select None
 * PSRAM: Disabled

## External library requrirements
 * TFT_eSPI (tested with version 2.5.34)
 * NimBLE_Arduino (tested with version 1.4.1)

## Other stuff
 Remember to modify the TFT_eSPI configuration file to select T-Display!
 
With NimBLE-Arduino the code just barely fits into default partition.
In case of memory problems, one can use

```Huge APP (3 MB No OTA / 1 MB SPIFFS)```

partition scheme to make the program fit.
