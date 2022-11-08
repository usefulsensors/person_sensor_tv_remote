# Person Sensor TV Remote
Arduino project for Circuit Playground Express that pauses and plays your TV based on whether you're there.

## Introduction

The [Person Sensor](https://usfl.ink/ps) from [Useful Sensors](https://usefulsensors.com)
is a small, low-cost hardware module that detects nearby peoples’ faces, and
returns information about how many there are, where they are relative to the
device, and performs facial recognition. It is designed to be used as an input
to a larger system, and this example shows how to use it to control a TV so that
it pauses what you're watching when you leave the viewing area, and resumes when
you return.

## BoM

To build this project you'll need:

 - [Circuit Playground Express](https://www.adafruit.com/product/3333).
 - [Person Sensor from Useful Sensors](https://usfl.ink/ps).
 - [Qwiic connector cable with alligator clips](https://www.adafruit.com/product/4398).
 - Blu Tack or something similar to hold the sensor in place.

No soldering is required. You need to make sure you get the "Express" version of
the Circuit Playground board, because this is the only one with a built-in IR
transmitter and receiver. You can probably adapt this to other boards with
external IR modules, but you'll need to figure out how to interface them with
CircuitPython. At the time of writing, the Circuit Playground Express board was
out of stock at Adafruit, but the slightly more expensive [base kit](https://www.adafruit.com/product/3517)
that includes one was in stock.

## Assembling

Wiring up the sensor requires a cable with a Qwiic/Stemma QT connector on one
end, and four alligator clips on the other. These clips connect VDD, GND, SDA
and SCL from the sensor to the board. First plug in the connector to the sensor.
This should only connect one way round, with the exposed metal facing the board.
The alligator clips then connect to four tabs on the Circuit Playground. If the
side with the buttons and components on is facing you, you start at the 11 
o'clock position with GND (black), the anti clockwise with SDC (yellow), then
SDA (blue), and finally 3.3V (red). Here's a wiring diagram:

![Person Sensor TV Remote Wiring Diagram](pico_person_sensor_tv_remote_bb.png)

Once the Circuit Playground is plugged into USB, you should see the green light
on the person sensor glow when a face is visible.

### Mounting

The Person Sensor needs to be mounted on the back of the board, on the opposite
side to the buttons. This will be looking back at you on the couch, while the IR
transmitter on the other side sends commands to the TV to play or pause
depending on whether you're around. I bought [an enclosure](https://www.adafruit.com/product/3915)
and used Blu Tack to mount the sensor in the right position, but I'm sure those
of you more skilled can come up with a more elegant solution.

![Blu Tack mounting horror](tv_sensor_mounting.jpg)
