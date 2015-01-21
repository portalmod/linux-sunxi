#!/usr/bin/env bash

# file: headphones_init.sh
# desc: initialize the headphone gain.

PIN_CLK=14 # PA09 -> CLOCK
PIN_VOLUME=13 # PA07 -> VOLUME UP/DOWN

# We are working with inverted (negative) boolean logic due to
# the transistors between the CPU and the headphone gain-control IC:
HIGH=0
LOW=1

# Export pin to access.
function pin_export() {
    pin=`echo /sys/class/gpio/gpio$1_*`
    test -e "$pin" || echo "$1" > /sys/class/gpio/export
}

# Input or output.
function pin_direction() {
    pin=`echo /sys/class/gpio/gpio$1_*`
    echo "$2" > "$pin/direction"
}

# Set pin value.
function pin_set_value() {
    pin=`echo /sys/class/gpio/gpio$1_*`
    echo "$2" > "$pin/value"
}

# pins setup
pin_export $PIN_CLK
pin_export $PIN_VOLUME
pin_direction $PIN_CLK out
pin_direction $PIN_VOLUME out

UP=$HIGH
DOWN=$LOW

# Set volume to minimum
pin_set_value $PIN_VOLUME $DOWN
for i in `seq 1 50`;
do
  pin_set_value $PIN_CLK $LOW
  pin_set_value $PIN_CLK $HIGH
done

pin_set_value $PIN_VOLUME $UP
for i in `seq 1 $1`;
do
  pin_set_value $PIN_CLK $LOW
  pin_set_value $PIN_CLK $HIGH
done
