#!/bin/sh
echo "Stopping mularsky service"

sync
sleep 3
killall -INT mularsky
sync
sleep 3
