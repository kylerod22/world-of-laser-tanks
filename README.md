
# World of Laser Tanks

Didn't think "World of Tanks" was immersive enough? What better than seeing the tanks you control with your own eyes?

## Overview
Our group wanted to make our own version of "World of Tanks" where the players actually control real-world miniature tanks! Our implementation is designed for two players, who score points by successfully shooting and disabling the enemy tank via a laser.

## Project Organization
This repo is divided into two sections: "Testing" and "Development." The former contains projects used to test individual components or certain combinations, while the latter contains the projects that we will use in our final uploads.

## Functional Diagram
![image](https://github.com/kylerod22/world-of-laser-tanks/assets/95047264/7dba4668-064a-4153-bb55-096710bce11f)

https://lucid.app/lucidchart/12ba26ac-ad22-4cd6-ad6b-3e57993b56a6/edit?view_items=QffTtPlSCTbG&invitationId=inv_eee534a5-d97f-417b-9019-eea6b3811451

#### Important Notes:
- PS2 Controllers communicate via SPI protocol
    - Useful link: https://store.curiousinventor.com/guides/PS2/
    - Use 3.3v power supply.
    - A lot of websites say SCLK should be at 500 kHz, but I found it to be working properly at frequencies closer to 30 kHz.
    - The DATA (or MISO) pin needs to have a pull-up resistor (1-10 kOhms), as the controller is only able to connect the line to ground.
- XBee Modules communicate wirelessly, and they communicate with the Nucleo boards via UART protocol
    - Use 3.3v power supply.
    - When configuring the XBee's, make sure the channel and PAN ID match. 
    - Also see our group folder for what else needs to be set.

## Pins for Controller Transmitter
- SPI3 Interface with Controller
    - SCLK: PC_10
    - MISO: PC_11
    - MOSI: PC_12
    - CS 0: PC_9
    - CS 1: PC_8
- UART1 Interface with XBee
    - TX -> Xbee Din: PB_6
    - Xbee Dout -> RX: PA_10
- Both controller and XBee need to be connected to 3.3v and ground.

## Pins for Tank Receiver
- UART6 Interface with XBee
    - TX -> Xbee Din: PA_11
    - Xbee Dout -> RX: PA_12
- PWM Generators
    - Right Motor Speed (TIM3_CH2): PC_7
    - Left Motor Speed (TIM4_CH1): PB_6
    - Servo TBD
- GPIO Pins
    - Right Motor Direction: PA_8
        - LOW is forward, HIGH is backward.
    - Left Motor Direction: PA_9
