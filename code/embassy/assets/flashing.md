## The previous steps to Flash the Zephyr (written in C) Firmware Samples

### Overview
Nordic's nRF52840 (or nRF52833) system-on-a-chip (SoC) sits at the heart of b-parasites. It's an ARM Cortex M4, and it's programmed and debug via the Serial Wire Debug (SWD) interface.

To flash our b-parasite firmware to the SoC, we need a SWD programmer.

### SWD programmers
J-Link debuggers are the most popular choice of SWD programmers. The J-Link EDU and J-Link EDU Mini versions are good choices for hobbyists. You can find the EDU Mini on Adafruit for around $20.

For more in-depth information on how to program generic ARM SoCs with SWD, check out this tutorial by SparkFun.

### Connections
You can choose to program the b-parasite while it's powered by its own CR2032 or by the SWD programmer itself.

#### Using the SWD power supply
MAKE SURE THE BATTERY IS REMOVED BEFORE YOU START! Four wires should be connected between the SWD programmer and b-parasite:

Vcc - 3.3V power supplly
GND - ground connection
SWCLK - data clock
SWIO - data
Note

Some programmers like the J-Link EDU Mini do not power their target. You must provide external power to b-parasite, either via a power supply or with a battery as described below. Additionally, you must connect their VTref pin to Vcc device.

These pads are exposed on the b-parasite board as follows:



#### Using a CR2032 Battery
Make sure the battery is in. Three wires should be connected between the SWD programmer and b-parasite:

GND - ground connection
SWCLK - data clock
SWIO - data
The Vcc pin should not be connected, as the CR2032 cell is already powering the SoC. Connecting the Vcc in addition to the battery will likely damage the b-parasite, the battery or both.

#### Soldering vs. Using Pogo Pins
For programming a few b-parasites, soldering wires to the pads is reasonable. For programming many b-parasites or constantly debugging them, I recommend one of these pogo pin clamps from aliexpress:

pogo-pins.mp4
The pads are spaced by the standard 2.54mm, which is the same spacing as the usual pin headers and most protoboards use.

## Build the Code & Flash It
Follow How to Build the Samples.

### Options for Flashing
#### The Standard Way
Nordic's nRF Connect for VS Code is a all-in-one plugin that lets you build and flash your code from VSCode. All you need is to connect your JLink and hit the "Flash" button. Check out this video from Nordic for a tutorial. image

#### Calling nrfjprog
This is what the nRF Connect for VS Code extension does behind the scenes. For example, once a sample is build, you can flash it from it's directory as follows:

nrfjprog -f nrf52 --program build/zephyr/zephyr.hex --sectorerase
nrfjprog with Docker (Linux only!)
Similarly to the instructions for building with Docker, it is possible to use the nrfconnect-sdk image to flash the board:

docker run --privileged --rm -v ${PWD}:/workdir/project nordicplayground/nrfconnect-sdk:main \
nrfjprog -f nrf52 --program /workdir/project/code/nrf-connect/samples/ble/build/zephyr/zephyr.hex --sectorerase
This method is currently not available on mac OS, as USB passthrough is not supported -- see docker/for-mac/issues/6771.

### Other Methods
There are many other ways to flash the samples to the board. Issue #43 discusses some of them. Notably:

Using a blackmagic probe, blog post
Using a bluepill
The nrfmicro Wiki uses the same E73 module as us, and has a great compilation of methods of flashing the firmware, including using a Raspberry Pi.