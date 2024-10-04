# ESP IDF driver for BMP180 sensor with altitude estimation

## Overview
Inner structure of the driver is a simple FSM with 4 states: Initiate temperature measurement -(if conversion is ready)-> Read output registers -> Initiate pressure measurement -(if conversion is ready)-> Read output registers

For height estimation simple hydrostatic formula is used: dP = -ρgdZ = -ρGdH, i.e. p - p0 = -ρg(h - h0).

No synchronization mechanisms are used inside so it should be done by the user.

Also driver doesn't call any delays like vTaskDelay() or ets_delay_us() so you may do it yourself as shown in bmp180-altitude.c example.

## Build
Include it in your project as a usual ESP IDF component: clone repo inside project components/ directory, delete example file bmp180-altitude.c.
