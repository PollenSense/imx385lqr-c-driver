# IMX385 Camera Driver

This camera driver is based off of the **IMX290** camera driver

Targets kernel version 5.10.92

## Device Tree Source Files

Device tree overlay based on IMX290 dts files form Raspberry Pi Kernel, which uses 3 files:
 * imx290-overlay.dts
 * imx290_327.dtsi
 * imx290_327-overlay.dtsi


## Building DTBO file

Our current method for building the dtbo is to
1. Have buildroot build the `imx385.dtb` file
2. Copy to a PiOS installation and use `dtc`
3. `dtc -I dtb -O dts -o /output/path/imx385.dts /path/to/imx385.dtb`
4. `dtc -I dts -O dtb -o /output/path/imx385.dtbo /path/to/imx385.dts`
5. Copy to final destination `/boot/overlay/imx385.dtbo`
