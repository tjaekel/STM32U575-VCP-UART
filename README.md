# STM32U575 VCP UART
 VCP UART for my own STMU575 board

## Project for own STM32U575 board
This is my project for my own PCB with a STM32U575 as LQPF64 package.

It uses USB FS (not HS, not available onn this chip) as VCP UART.

It is based on old (legacy) USB Middleware (the "old" USB device drivers,
not the "new" AZURE ThreadX stuff.

I run this board with MCU VDD 1V8 - and it works.
But it is very sensitive on the VDD 1V8 voltage:
a bit too slow - even MCU runs - but the USB not anymore. You need at least
1.796V on MCU VDD, even USB is powered by 3V3.

## Features
The FW provides a UART command line (shell) on VCP UART.
You can fire commands. It is intended to generate QSPI transactions on an
external QSPI chip (running at 1V8).

This project works fine on my own PCB, with STM32U575. Not tested on a
NUCLEO-U575 board (might work as well, with some GPIO changes).

It is NOT usable on a NUCLEO-U5A5ZJ-Q boar, neither on a U5A5 chip: this one
has an integrated USB HS PHY and needs a completely different setup.

