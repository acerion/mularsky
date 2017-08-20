#!/bin/sh
echo "Stopping mularsky service"

sync
killall -9 cat
sync
sleep 1
/etc/init.d/ntp stop
sleep 1
sync
sleep 3
killall -INT mularsky
sync
sleep 3
