#!/bin/sh
echo "Starting mularsky service"
sleep 10


#echo "Stopping getty"
#sudo systemctl stop serial-getty@ttyAMA0.service
#sleep 1

echo "Creating gps device"
ln -s  /dev/ttyAMA0 /dev/gps1
sleep 1

echo "Stopping other services"
systemctl stop console-getty.service
systemctl disable console-getty.service
systemctl stop serial-getty@ttyAMA0.service
systemctl disable serial-getty@ttyAMA0.service
systemctl stop container-getty@AMA0.service
systemctl disable container-getty@AMA0.service
sleep 1

echo "Running stty"
stty -F /dev/ttyAMA0 raw 9600 cs8 clocal -cstopb
sleep 1

echo "Restarting ntpd"
/etc/init.d/ntp restart
sleep 5





YEAR=`date +"%Y"`


# Wait for ntpd getting date from GPS
#while true; do
#	YEAR=`date +"%Y"`
#	if [ "$YEAR" = "1970" ] 
#	then
#		echo "Starting mularsky service, waiting for update of clock..."
#		sleep 10
#	else
#		break;
#	fi
#done


DIR_NAME=/home/pi/data/`date +"%Y_%m_%d_%H_%M_%S"`

mkdir $DIR_NAME
chmod 0777 $DIR_NAME

stty -F /dev/ttyAMA0 raw 9600 cs8 clocal -cstopb
cat /dev/ttyAMA0 > $DIR_NAME/nmea.txt &
# tail -f /var/log/auth.log &
/home/pi/sw/mularsky/mularsky $DIR_NAME &
