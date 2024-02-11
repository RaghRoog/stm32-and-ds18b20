# Microprocessors project

## General info
This project implements my communication protocol. Communication between STM32 and PC is ensured by USART with usage of ring buffer.
The ds18b20 thermometer works on a one-wire bus.

## Features
* Entering frames via PuTTY
* Displaying the last measured temperature
* Changing measurement interval
* Searching devices on one wire bus
* Calculating CRC-8 and comparaing it with checksum provided in a frame
* Frame validation
