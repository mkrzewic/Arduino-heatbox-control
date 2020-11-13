# Safe heatbox thermostat
This code controls the temperature of both the environment of a heat box and the temperature of the heating element to avoid overheating the contents locally.

An additional safety feature is built in to deal with cheap chinese solid state relays - when it finally shorts out and starts glowing, a secondary mechanical relay
disconnects the system.

All that is meant to increase the confidence in leaving the system running unattended.

The code runs on an Arduino pro mini, and should therefore run without problems on any atmega328p based arduino without changes.

In case something does not compile: my forks of the RotaryEncoder and DallasTemperature libraries should be used...
