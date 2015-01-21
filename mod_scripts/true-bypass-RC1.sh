#!/usr/bin/env bash
 
# file: true-bypass.sh
# desc: turn on or off the "true bypass" relays
 
# defines the true bypass pins
TRUE_BYPASS_R=15
TRUE_BYPASS_L=16
 
function pin_export() {
    pin=`echo /sys/class/gpio/gpio$1_*`
    test -e "$pin" || echo "$1" > /sys/class/gpio/export
}
 
function pin_direction() {
    pin=`echo /sys/class/gpio/gpio$1_*`
    echo "$2" > "$pin/direction"
}
 
function pin_set_value() {
    pin=`echo /sys/class/gpio/gpio$1_*`
    echo "$2" > "$pin/value"
}
 
# check command line arguments
if [ "$#" -ne 1 ]; then
    echo -e "Usage: $0 <bypass|process>" >&2
    exit 1
fi
 
# pins setup
pin_export $TRUE_BYPASS_R
pin_export $TRUE_BYPASS_L
pin_direction $TRUE_BYPASS_R out
pin_direction $TRUE_BYPASS_L out
 
# set pins value
OPT="$1"
case "$OPT" in
   "bypass") 
       pin_set_value $TRUE_BYPASS_R 0
       pin_set_value $TRUE_BYPASS_L 0
   ;;
   "process") 
       pin_set_value $TRUE_BYPASS_R 1
       pin_set_value $TRUE_BYPASS_L 1
   ;;
   *)
       echo "Invalid option. Type $0 without parameters to see the help."
esac
