# RadioRocket
This repository contains information related to version 1 of my [radio rocket project.](https://n3vem.com/rocket)
This rocket crashed on it's first flight, so I'm now in the process of building version 2 - i.e. this repository is here for historical info - you'll likely want to head over to the [repository for version 2](https://github.com/N3VEM/RadioRocketV2) for the latest and greatest!

## LightAPRS - In the Rocket
The rocket contained a lightAPRS. Light APRS was set up according to the information available on the [lightAPRS github page.](https://github.com/lightaprs/LightAPRS-1.0) The code located here is simply the lightAPRS-hab code available there, but with a few minor tweaks to maximize the number of packets sent, and to disable the path details, so that the packets won't get digipeated and cause a flood of APRS traffic during the rockets flight.

##APRS - In the Ground Station
APRS reception was accomlished with an RTL-SDR dongle and Direwolf, which allowed the ground station to act like a networked TNC so that any connected client (I used my laptop running Pinpoint APRS) could grap the APRS data and display it.

## LoRa - In the Rocket
The rocket also contains an adafruit feather with 70cm LoRa and some other sensors. Code for that is located in the featherLoRaRocket folder.

## LoRa - In the Ground Station
The ground station consisted of a second feather with 70cm LoRa to receive the data sent by the one on the rocket. The code on that one is pretty basic, as most of the 'work' is then done by node-red running on a Libre Computer LePotato single board computer.

## Node-Red - Served by the Ground Station
a node red dashboard running on the computer that the ground station is connected to can display the telemetry data sent by the rocket.  
