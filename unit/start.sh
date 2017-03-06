#!/bin/sh
echo "Starting mularsky service"


YEAR=`date +"%Y"`


# Wait for ntpd getting date from GPS
while true; do
	YEAR=`date +"%Y"`
	if [ "$YEAR" = "1970" ] 
	then
		echo "Starting mularsky service, waiting for update of clock..."
		sleep 3
	else
		break;
	fi
done


DIR_NAME=/home/pi/data/`date +"%Y_%m_%d_%H_%M_%S"`

mkdir $DIR_NAME
chmod 0777 $DIR_NAME

stty -F /dev/ttyAMA0 raw 9600 cs8 clocal -cstopb
cat /dev/ttyAMA0 > $DIR_NAME/nmea.txt &
# tail -f /var/log/auth.log &
/home/pi/sw/mularsky/mularsky $DIR_NAME &
