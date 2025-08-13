# Vo2max
A portable and affordable DIY [spirometer](https://en.wikipedia.org/wiki/Spirometer) project to measure [VO2 Max](https://en.wikipedia.org/wiki/VO2_max) during an excercise. This is still a prototype and heavily in development.

This project turned out to be a complete rewrite and redesign of [Ivor Hewitt's VO2max](https://github.com/ihewitt/VO2max) spirometer project, which in turn was based on [Rabbitcreek's project](https://www.instructables.com/Accurate-VO2-Max-for-Zwift-and-Strava/).

The main idea was to have a fully 3D printable system, but printing the mask part turned out to be difficult so the current version uses parts from a commercial (but affordable) mask. There are plans to develop a model that can be 3D printed with soft material (TPU/TPE).

Current features:
 - [X] Exhalation volume measurement (with calibration function)
 - [X] Exhalation oxygen level measurement (with calibration function)
 - [X] VE and VO2(max) calculations
 - [X] Ambient pressure and temperature measurement for reducing measurements to STDP 
 - [X] Bluetooth LE heart rate sensor support
 - [X] Streaming data over WiFi to computer
 - [X] PC companion program to monitor and record the WiFi stream
 - [ ] Bluetooth [Golden Cheetah](https://www.goldencheetah.org/) support as VO2 Master (based on Ihewitt's code)
 
Main architectural changes in the code:
 - Restructured to use FreeRTOS tasks
 - Time-accurate sampling of sensors, e.g. air flow is measured every 20 ms
 - Lower priority tasks like bluetooth, wifi and display do not interrupt sensor reading
 - Code is divided to "sub modules" for easier upkeep
 
## Hardware
This is a listing of the hardware that is used and supported by the code and 3D models. Some of the hardware were different from the reference projects for availability reasons in the EU, or due to technical reasons like mounting.

 - A rubber (silicone?) half face dust mask with detachable parts, see links/pictures below
 - [LilyGo T-Display](https://www.lilygo.cc/products/lilygo%C2%AE-ttgo-t-display-1-14-inch-lcd-esp32-control-board) ESP32 development board with 1.14" display
 - [DFRobot SEN0322](https://www.dfrobot.com/product-2052.html) Gravity: Electrochemical Oxygen sensor (0-25% Vol, I2C)
 - [Sensirion SDP801](https://sensirion.com/products/catalog/SDP801-500Pa/) Digital +/-500Pa differential pressure sensor, I2C, manifold connection
 - [Bosch BMP388](https://www.bosch-sensortec.com/products/environmental-sensors/pressure-sensors/bmp388/) ambient pressure sensor (e.g. [DFRobot SEN0251 Gravity: BMP388](https://www.dfrobot.com/product-1792.html))
 - Thread forming 2.5x8mm screws (6 pcs) (e.g. [Bossard 3034065](https://www.bossard.com/eshop/global-en/direct-assembly-screws/direct-assembly-screws-for-plastics/ecosyn-plast/p/82428/?category=01.200.300&index=7)) and 2.5x16mm screws (2 pcs) (e.g. [Bossard 3034077](https://www.bossard.com/eshop/global-en/direct-assembly-screws/direct-assembly-screws-for-plastics/ecosyn-plast/p/82428/?category=01.200.300&index=7))
 - 14500 size Lithium-ion battery and suitable connection parts (spring terminals or similar)
 - 2 pcs of 4.5 x 1.5 mm O-rings
 
## Where to buy
Most of the necessary electronics components were bought from [Mouser](https://eu.mouser.com/).

The display and half mask was bought from [Aliexpress](https://www.aliexpress.com). Many stores had similar masks so a random, somewhat professional looking one was selected and the mask that was received was ok. Prepare for a long delivery time, 6 weeks or so.

Links to the main components that were used are below with their price at the time of building (incl. VAT etc.).

| Component | Source | Price |
| --------- | ------ | ----- |
| DFRobot Sen0322 | [Mouser](https://eu.mouser.com/ProductDetail/DFRobot/SEN0322?qs=sPbYRqrBIVnCRTCgFw%2FJyQ%3D%3D) | 50.13 € |
| Sensirion SDP801 | [Mouser](https://eu.mouser.com/ProductDetail/Sensirion/SDP801-500Pa?qs=PzGy0jfpSMsYh%2Fiz3%252Braew%3D%3D) | 30.50 € |
| DFRobot SEN0251 BMP388 | [Mouser](https://eu.mouser.com/ProductDetail/DFRobot/SEN0251?qs=sGAEpiMZZMsvmS05OVPORkos2oZHucfRYva4AowfgwtRmiD%2F%252BmISIg%3D%3D) | 6.42 € |
| Lilygo T-Display 16MB CH9102F | [Aliexpress lilygo Official store](https://www.aliexpress.com/item/33048962331.html) | 16.18 € |
| Dustproof half-mask | [Aliepress CKE-Labour Pro Store](https://www.aliexpress.com/item/1005005820042955.html) | $ 6.24 |



# Assembly

## Valve assembly
Exhalation and inhalation valves are required. Valve itself consists of two 3D printed parts (body and membrane holder), membrane template and the membrane itself. In addition, inhalation valve needs a counterpart for mounting. Current design uses two different sizes of valves due to how the commercial mask was built.

 1. Print all the valve parts
 2. Using the template, cut the membrane from suitable material. Rubber glove can be used as the membrane. One may experiment with different materials
 3. Insert the membrane into the valve body
 4. Place the membrane holder on the membrane. Carefully push the ends of the holder slightly and rotate the holder into the clips. Take care to ensure that the membrane is still centered and has no wrinkles
 
## I2C distribution board
A simple I2C distribution board is needed to be mounted on top of the Sensirion SDP801 sensor. This also helps in mounting the sensor.

Mill or etch and drill or order the board, and then solder the sensor wires to them. I recommend measuring the wires yourself but the lengths of the wires are roughly:
 - 20 cm for the oxygen sensor
 - 16 cm for the ambient pressure sensor
 - TBD cm for the display/processor
 
I recommend routing one sensor wire from the backside and other wires from the front.

Also solder 1.3 kOhm pull-up resistors between VDD and SDA and SCL.

Finally the Sensirion SDP801 can be soldered to the distribution board.

## Main body assembly
Main body consists of a single 3D-printed main part. Support is needed at the bottom parts and about 6 mm outer brim should be used to ensure that the part stays upright. With a bed-slinger printer, orient the print so that the wide side is parallel to the moving axis.

 1. Print the body and remove all support material
 2. Mount the oxygen sensor on top with 4x 8mm screws
 3. Mount the ambient pressure/temperature sensor on the side with 2x 8mm screws
 4. Place the O-rings on the recesses of the pressure sensor location
 5. Mount the SDP801 pressure sensor with 2x 16mm screws
 6. Wire the battery terminals to the T-display battery connector
 7. Install the battery terminals and connect the connector to T-display
 6. Place the display on the front of the unit
 7. Carefully measure cables, cut and solder them, removing parts from the body if necessary during soldering
 8. Use 1.3 kOhm external pull-up resistors for I2C SDA and SCL (preferably soldered to the I2C distribution board)

Note that the battery terminals nor the power switch are currently included in the model. One should find suitable parts for this and glue them in place.

The battery holder might be a bit tight, so some sanding/cutting of the edges may be necessary.
 
## Flashing the firmware
The firmware was built with PlatformIO v core 6.1.18, home 3.4.4. One needs to install lilygo-t-display board (or it may get installed automatically when building...)

Following external libraries are needed:

 - TFT_eSPI (version 2.5.34)
 - NimBLE-Arduino for Bluetooth LE (version 1.4.1)
 
Other drivers are included in the repository with necessary modifications.

To build the software select following as the board settings:

 * Board: Lilygo-t-display
 * Upload Speed: 921600
 * CPU Frequency: 240Mhz (WiFi/BT)
 * Flash Frequency: 80Mhz
 * Flash Mode: QIO
 * Partition Scheme: Default.csv [4MB with spiffs (1.2MB APP/1.5 SPIFFS)]
 * Core Debug Level: None --> For debugging can set also different values, for release select None
 * PSRAM: No

 
# Disclaimer
This repository contains code, models and similar for educational purposes only and are supposed 
to be used for reference. Everything is provided as is, without any warranty. The author is not 
liable for any damages or other liability arising from use of anything provided here.

# Copyright
Copyright 2024, 2025 Lauri Peltonen

Licensed under GPLv3.

Drivers included in this repository may have their own licenses which are included in the relevant 
source files.
