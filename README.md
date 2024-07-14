Alternative firmware based on libopencm3 for Makerbase CANable 2.0
USB Adapter ([schematics][schematics], [aliexpress][aliexpress])
based on STM32G4 MCU + TJA105T CANbus transceiver

It uses USB CDC-ACM (taken from [libopencm3-examples][usbacm] and
adapted for using with STM32G4) as a transport for CANbus frames in
the same proprietary format as used by [Gree Data Converter
(ZTS6L)][ebay]

[schematics]: https://raw.githubusercontent.com/makerbase-mks/CANable-MKS/main/Hardware/MKS%20CANable%20V2.0/MKS%20CANable%20V2.0_001%20schematic.pdf
[aliexpress]: https://www.aliexpress.com/item/1005005455241016.html
[ebay]: https://www.ebay.com/itm/203692017680
[usbacm]: https://github.com/libopencm3/libopencm3-examples/blob/master/examples/stm32/f3/stm32f3-discovery/usb_cdcacm/cdcacm.c
