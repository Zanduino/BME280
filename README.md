# Work-In-Progress
Once the initial version of the library and example programs is completed this text will be removed. At the moment this GIT allows for centralized developmen using several machines

# BME280 library
<img src="https://github.com/SV-Zanshin/BME280/blob/master/Images/sensors_pinout.jpg" width="175" align="right"/> *Arduino* library for using the [Bosch BME280](https://www.bosch-sensortec.com/bst/products/all_products/bme280) sensor which senses temperature, humidity and pressure. Since the package is quite small and difficult to breadboard with, the [Adafruit BME280 Breakout](https://www.adafruit.com/product/2652) board was used for development.

A complete description of sensor can be found in the [BME280 Datasheet](https://ae-bst.resource.bosch.com/media/_tech/media/datasheets/BST-BME280_DS001-11.pdf) while the [Adafruit tutorial](https://learn.adafruit.com/adafruit-bme280-humidity-barometric-pressure-temperature-sensor-breakout) describes additional board information.

## Sensing
The BME280 allows the user to individually adjust the sampling rates for each component sensing (temperature, humidity and pressure) as well as to set the inter-measurement delay time and the [IIR](https://en.wikipedia.org/wiki/Infinite_impulse_response) filter sampling coeffient. Combining these settings allows very fine-grained control of measurement times vs. measurement accuracy.

## Temperature sensing
## Humidity sensing
## Pressure sensing

A detailed library description and further details are available at the [GitHub DSFamily Wiki](https://github.com/SV-Zanshin/BME280/wiki)

![Zanshin Logo](https://www.sv-zanshin.com/r/images/site/gif/zanshinkanjitiny.gif) <img src="https://www.sv-zanshin.com/r/images/site/gif/zanshintext.gif" width="75"/>
