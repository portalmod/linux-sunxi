#!/usr/bin/env bash

# file: headphones_init.sh
# desc: initialize the headphone gain.

PIN_CLK=14 # PA09 -> CLOCK
PIN_UP_DOWN=13 # PA07 -> VOLUME UP/DOWN

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
pin_export $PIN_UP_DOWN
pin_direction $PIN_CLK out
pin_direction $PIN_UP_DOWN out

# Set volume to minimum
pin_set_value $PIN_UP_DOWN 0
for i in `seq 1 16`;
do
  pin_set_value $PIN_CLK 1
  pin_set_value $PIN_CLK 0
done

pin_set_value $PIN_UP_DOWN 1
for i in `seq 1 $1`;
do
  pin_set_value $PIN_CLK 1
  pin_set_value $PIN_CLK 0
done
