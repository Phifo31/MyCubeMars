
# MyCubeMars

SW tools and tests for my Cube Mars actuators

## Activation du bus CAN sur Linux

$ ip link set can0 down
$ ip link set can0 type can bitrate 500000
$ ip link set can0 up

> candump can0
> cansend can0 001#1122334455667788


## Cablage Esp32C6 / Mikroe Click CAN FD 4

    GND ==> GND
    5V ==> 5V ( /!\ ne fonctionne pas en 3V3)
    CS ==> GND
    TX ==> Rx de TWAI Esp32 - pin 22 (à travers un pont diviseur 10K / 15K)
    RX ==> Tx de TWAI Esp32 - pin 21


    
