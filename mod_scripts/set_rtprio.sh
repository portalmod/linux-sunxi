#!/bin/bash

if [[ "$#" != "2" ]]; then
    echo "Usage: $0 <irq_pattern> <priority>"
    exit 1
fi


n_pids=`ps -e | grep $1 | awk '{print $1}' | wc -l`

if [[ "$n_pids" > "1" ]]; then
    echo "Found more than one process with this pattern:"
    ps -e | grep $1
    exit 1
fi

if [[ "$n_pids" == "0" ]]; then
    echo "No process found with this pattern"
    exit 1
fi

pid=`ps -e | grep $1 | awk '{print $1}'`

prio=$2
chrt -f -p $prio $pid
