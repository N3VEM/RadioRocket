# RadioRocket
This repository will contain information related to my [radio rocket project.](https://n3vem.com/rocket)

## LightAPRS
The rocket currently contains a lightAPRS. Light APRS is set up according to the information available on the [lightAPRS github page.](https://github.com/lightaprs/LightAPRS-1.0) The code located here is simply the lightAPRS-hab code available there, but with a few minor tweaks to maximize the number of packets sent, and to disable the path details, so that the packets won't get digipeated and cause a flood of APRS traffic during the rockets flight.

## In the rocket
The rocket also contains an adafruit feather with 70cm LoRa and some other sensors.  Code for that is located in the featherLoRaRocket folder.
More details to be added sometime, maybe.

## Ground Station
currently the ground station consists of a second feather with 70cm LoRa to receive the data sent by the one on the rocket.  That code is in the featherLoRaGroundStation folder

## Dashboard - node-red
a node red dashboard running on the computer that the ground station is connected to can display the telemetry data sent by the rocket.  The flow for that is located in the node-red folder. In addition to displaying the dashboard, the node-red flow will dump the received data out to a flat file for later analysis.
