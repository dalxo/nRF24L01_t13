# nRF24L01+ for ATtiny13A ultra low-power wireless sensor
![Cover image](media/cover_image.jfif)

OSI Layer 2 driver for nRF24L01+ on [ATtiny13A](https://www.microchip.com/en-us/product/ATtiny13) (1KiB flash + 64B SRAM) for ultra low-power wireless applications. It was devised to optimize:
- **Pin-count:** It supports three generic pin-optimization HW configurations that require none or a single resistor. The nRF24 module occupies on MCU only 3 pins instead of 6. This leaves theother 2 pins (+1 if RST used as IO) for attaching sensor(s) or a fieldbuss.

- **Power-consumption:** It was dedicatedly made for ultra-low-power applications. The energy-saving is achieved by the optimized library with a very small number of instructions. This reduces overall execution time. The pin-count is minimized in a way that offers better energy management of the nRF24 module than other similar approaches (shared CE/CSN vs. CE permanently active).

- **Footprint:** Very low footprint - demo applications have 378 bytes (transmitter) and 434 bytes (receiver) including application's logic. It can easily fit into ATtiny13A and leave enough space for other applications and attached devices and field busses. It is configurable for either half-duplex or simplex radio communication to further minimize the footprint. Unlike other generic libraries, it can be also deployed on any other MCU from the ATtiny sub-family with the same or a greater memory. SPI interface is implemented as bit-banging, thus there is no dependency on peripherals such as Universal Serial Interface (USI).

[**More detailed describtion of the library on Hackster.io**](https://www.hackster.io/orfanus/nrf24l01-for-ultra-low-power-sensor-with-attiny13a-3-pins-a51b2c)

## HW Configurations

### (1) Shared CE/CSN pins
Both interface signals are connected to a single pin:

![Shared CE and CSN pins](media/Shared_CE_CSN-2.png)

This is useful if the MCU is used for transmissions of data (sensors) to the central hub. Between transmissions the module 
is in the power saving mode (`CE == 0`, i.e. radio is off)  to conserve energy. Applicable for battery powered applications. 
The configuration is enabled with macro:
 
`#define NRF24L01_SHARED_CE_CSN`

If an application is intended to spend more time in the RX mode than TX, it might be more efficient to not merge CE and CSN. Instead, pulling CE to Vcc (high) would allow ATtiny13A to pool for new messages without a need to switch off the radio.


### (2) Bi-directoinal 3-wire SPI
For situations where MCU needs bi-directional access to the nRF radio module, e.g. reading registers or receiving data. Rather than consuming additional pin on MCU (beside MOSI), both MISO and MOSI can be connected to a single MCU pin via a resistor:

![Merged MISO and MOSI](media/Shared_MISO_MOSI.png)

In our case values from 4.7k up to 10k worked well. Merged MISO/MOSI saves one pin which can be used for direct control of CE. If desired, this configuration can be mixed with the first one:

![3-wire SPI with shared CE/CSN](media/MIMO_shared_CE_CSN.png)

The configuration is enabled with macro:

`#define NRF24L01_3WIRE_SPI`

### (3) Uni-directional SPI
There are cases when we do not read anything from the nRF24 module. The device is used only as a data source and for transmission. Thus, we can reduce entire communication on SPI bus only to writting into the module. Signal MISO is not used:

![3-wire SPI with shared CE/CSN](media/Only_MOSI.png)

We can combine this configuration with the first one (shared CE/CSN) to further optimize the required pin-count on MCU as shown in the following figure:

![3-wire SPI with shared CE/CSN](media/Only_MOSI_Shared_CE_CSN.png)


The configuration is enabled with macro: 

`#define NRF24L01_DO_NOT_USE_MISO`


### (4) Full (4-wire) SPI
In this case we have separate pins for MOSI and MISO signals, i.e. standard 4-wire SPI. This does not conserve pins on MCU. Nevertheless, the footprint of this solution is 10 bytes (5 instructions) smaller than shared pins. This is useful in application where the code footprint is more critical than number of used pins.

This is the default configuration - unless it is overridden by macros from configuration (2) or (3). It is possible to combine it with the first configuration (shared CE/CSN).


## SW Configuration
By default, the library works with full 4-wire SPI and independent CE and CSN signals. Configuration of the library to work with one of the above described HW options is done by defining corresponding macros in the [projdefs.h](projdefs.h) file.


The HW configuration (1) is **ORTHOGONAL** to the remaining ones (2) -- (4), i.e. it can be used together with any MISO/MOSI combinations. Configurations (2) -- (4) are **MUTUALY EXCLUSIVE**, i.e. only one of them can be enabled (used) in the compiled code.

List of macros for selecting features:
````C
#define NRF24L01_SHARED_CE_CSN`		// Enable HW config (1), can be concurrently used with features (2), (3), and (4).
#define NRF24L01_DO_NOT_USE_MISO	// Enable HW config (2), mutually exclusive with feature (3) and (4) 
#define NRF24L01_3WIRE_SPI		// Enable HW config (3), mutually exclusive with feature (2) and (4)
````
HW config (4) is enabled be default unless config (2) or (3) is enabled (defined). 


Definition to which port is attached which nRF24 signal is also done via macros in the [projdefs.h](projdefs.h) header. Pin configuration macros are:
````C
// MANDATORY macros for port output, port direction and pin reading:
#define NRF24L01_PORT		PORTB
#define NRF24L01_DDR		DDRB
#define NRF24L01_INPORT		PINB
````

Define the following macros (where applicable - see configurations above) for a particular pin setup:
````C
#define NRF24L01_CE     PB0    // Optional. Do not define if shared CE/SCN is enabled 
#define NRF24L01_CSN    PB2    // Mandatory
#define NRF24L01_SCK	PB1    // Mandatory
#define NRF24L01_MOSI	PB3    // Mandatory
#define NRF24L01_MISO	PB4    // Optional. Define only if 4-wire SPI is used.
````

### API
